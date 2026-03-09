#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_err.h"

#include "wav_utils.h"
#include "hearing_test.h"
#include "audio_processor.h"

static const char *TAG = "main";

#define WAV_INPUT_PATH   "/spiffs/input.wav"
#define WAV_OUTPUT_PATH  "/spiffs/output.wav"
#define PROCESS_CHUNK    1024

static esp_err_t init_spiffs(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS mount failed: %s", esp_err_to_name(ret));
        return ret;
    }

    size_t total = 0, used = 0;
    esp_spiffs_info(NULL, &total, &used);
    ESP_LOGI(TAG, "SPIFFS: %d KB total, %d KB used", (int)(total / 1024), (int)(used / 1024));
    return ESP_OK;
}

/**
 * Wait for the host Python script to send "READY\n".
 */
static void wait_for_host(void)
{
    ESP_LOGI(TAG, "Waiting for host connection (send READY)...");
    char line[32];
    int pos = 0;
    while (1) {
        int c = fgetc(stdin);
        if (c != EOF) {
            if (c == '\n' || c == '\r') {
                line[pos] = '\0';
                if (strncmp(line, "READY", 5) == 0) {
                    ESP_LOGI(TAG, "Host connected");
                    return;
                }
                pos = 0;
            } else if (pos < (int)sizeof(line) - 1) {
                line[pos++] = (char)c;
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

/**
 * Stream processed audio back to the host over serial.
 * Protocol: "AUDIO <sample_rate> <num_samples>\n" followed by raw int16 PCM data.
 */
static void stream_audio_to_host(const float *buf, uint32_t num_samples, uint32_t sample_rate)
{
    printf("AUDIO %lu %lu\n", (unsigned long)sample_rate, (unsigned long)num_samples);
    fflush(stdout);

    // Send as raw int16 PCM
    for (uint32_t i = 0; i < num_samples; i++) {
        float s = buf[i];
        if (s > 1.0f) s = 1.0f;
        if (s < -1.0f) s = -1.0f;
        int16_t sample = (int16_t)(s * 32767.0f);
        // Write raw bytes (little-endian, same as ESP32 native)
        fwrite(&sample, sizeof(int16_t), 1, stdout);
    }
    fflush(stdout);

    printf("DONE\n");
    fflush(stdout);

    ESP_LOGI(TAG, "Streamed %lu samples to host", (unsigned long)num_samples);
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== USFMH - User Calibrated Frequency Mapping Headset ===");

    // Make stdin non-blocking so fgetc returns EOF immediately when no data
    // (prevents watchdog timeout while polling for serial input)
    int flags = fcntl(fileno(stdin), F_GETFL, 0);
    fcntl(fileno(stdin), F_SETFL, flags | O_NONBLOCK);

    // 1. Initialize SPIFFS
    if (init_spiffs() != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS init failed, halting");
        return;
    }

    // 2. Wait for host
    wait_for_host();

    // 3. Run hearing test (binary search over serial)
    ESP_LOGI(TAG, "--- Phase 1: Hearing Test ---");
    hearing_result_t hearing;
    hearing_test_run(&hearing);

    // Print summary
    ESP_LOGI(TAG, "Hearing test complete. Results:");
    for (int i = 0; i < NUM_TEST_FREQS; i++) {
        ESP_LOGI(TAG, "  Band %2d: %5.0f Hz — %s (gain: %.1f dB)",
                 i, test_frequencies[i],
                 hearing.heard[i] ? "HEARD" : "NOT HEARD",
                 hearing.gain_db[i]);
    }

    // 4. Read WAV file from SPIFFS
    ESP_LOGI(TAG, "--- Phase 2: Audio Processing ---");
    float *audio_buf = NULL;
    uint32_t num_samples = 0;
    uint32_t sample_rate = 0;

    esp_err_t err = wav_read_f32(WAV_INPUT_PATH, &audio_buf, &num_samples, &sample_rate);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WAV: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "Make sure %s exists on SPIFFS", WAV_INPUT_PATH);
        return;
    }

    // 5. Initialize audio processor with hearing test gains
    err = audio_processor_init(&hearing, sample_rate);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Audio processor init failed");
        free(audio_buf);
        return;
    }

    // 6. Process audio in chunks
    ESP_LOGI(TAG, "Processing %lu samples...", (unsigned long)num_samples);
    audio_processor_reset();

    uint32_t offset = 0;
    while (offset < num_samples) {
        uint32_t chunk = num_samples - offset;
        if (chunk > PROCESS_CHUNK) chunk = PROCESS_CHUNK;

        err = audio_processor_apply(audio_buf + offset, chunk);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Processing failed at offset %lu", (unsigned long)offset);
            free(audio_buf);
            return;
        }
        offset += chunk;
    }

    ESP_LOGI(TAG, "Audio processing complete");

    // 7. Write processed audio back to SPIFFS
    err = wav_write_f32(WAV_OUTPUT_PATH, audio_buf, num_samples, sample_rate);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to write output WAV (SPIFFS full?)");
    }

    // 8. Stream processed audio to host for playback
    ESP_LOGI(TAG, "--- Phase 3: Streaming to Host ---");
    stream_audio_to_host(audio_buf, num_samples, sample_rate);

    free(audio_buf);
    ESP_LOGI(TAG, "=== All done! ===");
}
