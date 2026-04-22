#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "sd_card.h"
#include "coeff_loader.h"
#include "wav_utils.h"
#include "hearing_test.h"
#include "audio_processor.h"

static const char *TAG = "main";

#define COEFF_PATH       SD_MOUNT_POINT "/filters.bin"
#define WAV_INPUT_PATH   SD_MOUNT_POINT "/input.wav"
#define WAV_OUTPUT_PATH  SD_MOUNT_POINT "/output.wav"
#define PROCESS_CHUNK    1024
#define YIELD_EVERY_SAMPLES 8192

/**
 * Wait for the host Python script to send "READY\n".
 */
static void wait_for_host(int *out_debug_sample_index, bool *out_debug_sample_enabled,
                          bool *out_debug_all_enabled)
{
    *out_debug_sample_index = -1;
    *out_debug_sample_enabled = false;
    *out_debug_all_enabled = false;

    ESP_LOGI(TAG, "Waiting for host connection (send READY)...");
    char line[32];
    int pos = 0;
    while (1) {
        int c = fgetc(stdin);
        if (c != EOF) {
            if (c == '\n' || c == '\r') {
                line[pos] = '\0';
                if (strncmp(line, "READY", 5) == 0) {
                    int sample_index = -1;
                    if (strcmp(line, "READY SAMPLE ALL") == 0) {
                        *out_debug_all_enabled = true;
                        ESP_LOGI(TAG, "Debug all-bands mode requested");
                    } else if (sscanf(line, "READY SAMPLE %d", &sample_index) == 1) {
                        if (sample_index >= 0 && sample_index < NUM_TEST_FREQS) {
                            *out_debug_sample_index = sample_index;
                            *out_debug_sample_enabled = true;
                            ESP_LOGI(TAG, "Debug sample mode requested (sample index %d, %.0f Hz)",
                                     sample_index, test_frequencies[sample_index]);
                        } else {
                            ESP_LOGW(TAG,
                                     "Ignoring invalid debug sample index %d (valid range: 0-%d)",
                                     sample_index, NUM_TEST_FREQS - 1);
                        }
                    }
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

    for (uint32_t i = 0; i < num_samples; i++) {
        float s = buf[i];
        if (s > 1.0f) s = 1.0f;
        if (s < -1.0f) s = -1.0f;
        int16_t sample = (int16_t)(s * 32767.0f);
        fwrite(&sample, sizeof(int16_t), 1, stdout);

        if ((i % YIELD_EVERY_SAMPLES) == 0) {
            vTaskDelay(1);
        }
    }
    fflush(stdout);

    printf("DONE\n");
    fflush(stdout);

    ESP_LOGI(TAG, "Streamed %lu samples to host", (unsigned long)num_samples);
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== USFMH - User Calibrated Frequency Mapping Headset ===");

    // Make stdin non-blocking
    int flags = fcntl(fileno(stdin), F_GETFL, 0);
    fcntl(fileno(stdin), F_SETFL, flags | O_NONBLOCK);

    // 1. Wait for host and optional debug sample request (before SD card,
    //    so the serial link is up even if hardware isn't wired yet)
    int debug_sample_index = -1;
    bool debug_sample_enabled = false;
    bool debug_all_enabled = false;
    wait_for_host(&debug_sample_index, &debug_sample_enabled, &debug_all_enabled);

    // 2. Initialize SD card
    if (sd_card_init() != ESP_OK) {
        ESP_LOGE(TAG, "SD card init failed, halting");
        return;
    }

    // 3. Load filter coefficients from SD card
    filter_bank_t bank;
    esp_err_t err = coeff_loader_load(COEFF_PATH, &bank);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load coefficients from %s", COEFF_PATH);
        sd_card_deinit();
        return;
    }

    // 4. Run hearing test (binary search over serial)
    ESP_LOGI(TAG, "--- Phase 1: Hearing Test ---");
    hearing_result_t hearing;
    if (debug_all_enabled) {
        ESP_LOGI(TAG, "Skipping hearing test (debug all-bands mode)");
        memset(&hearing, 0, sizeof(hearing));
        for (int i = 0; i < NUM_TEST_FREQS; i++) {
            hearing.heard[i] = true;
            hearing.gain_db[i] = 0.0f;
        }
        hearing.num_tested = NUM_TEST_FREQS;
    } else if (debug_sample_enabled) {
        ESP_LOGI(TAG, "Skipping hearing test (debug sample mode)");
        memset(&hearing, 0, sizeof(hearing));
        for (int i = 0; i < NUM_TEST_FREQS; i++) {
            hearing.heard[i] = false;
            hearing.gain_db[i] = HEARING_TEST_BOOST_DB;
        }
        hearing.heard[debug_sample_index] = true;
        hearing.gain_db[debug_sample_index] = 0.0f;
        hearing.num_tested = 1;
    } else {
        hearing_test_run(&hearing);
    }

    // Print summary
    ESP_LOGI(TAG, "Hearing test complete. Results:");
    for (int i = 0; i < NUM_TEST_FREQS; i++) {
        ESP_LOGI(TAG, "  Band %2d: %5.0f Hz — %s (gain: %.1f dB)",
                 i, test_frequencies[i],
                 hearing.heard[i] ? "HEARD" : "NOT HEARD",
                 hearing.gain_db[i]);
    }

    // 5. Read WAV file from SD card
    ESP_LOGI(TAG, "--- Phase 2: Audio Processing ---");
    float *audio_buf = NULL;
    uint32_t num_samples = 0;
    uint32_t sample_rate = 0;

    err = wav_read_f32(WAV_INPUT_PATH, &audio_buf, &num_samples, &sample_rate);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WAV: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "Make sure %s exists on the SD card", WAV_INPUT_PATH);
        coeff_loader_free(&bank);
        sd_card_deinit();
        return;
    }

    // 6. Initialize audio processor with filter bank and hearing test
    err = audio_processor_init(&bank, &hearing, sample_rate);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Audio processor init failed");
        free(audio_buf);
        coeff_loader_free(&bank);
        sd_card_deinit();
        return;
    }

    // 7. Process audio in chunks
    ESP_LOGI(TAG, "Processing %lu samples...", (unsigned long)num_samples);
    audio_processor_reset();

    uint32_t offset = 0;
    uint32_t next_progress_report = 10;
    int64_t process_start_us = esp_timer_get_time();
    while (offset < num_samples) {
        uint32_t chunk = num_samples - offset;
        if (chunk > PROCESS_CHUNK) chunk = PROCESS_CHUNK;

        err = audio_processor_apply(audio_buf + offset, chunk);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Processing failed at offset %lu", (unsigned long)offset);
            free(audio_buf);
            audio_processor_free();
            coeff_loader_free(&bank);
            sd_card_deinit();
            return;
        }
        offset += chunk;

        uint32_t pct = (offset * 100U) / num_samples;
        if (pct >= next_progress_report) {
            int64_t elapsed_ms = (esp_timer_get_time() - process_start_us) / 1000;
            ESP_LOGI(TAG, "Processing progress: %lu%% (%lu/%lu), elapsed %lld ms",
                     (unsigned long)pct,
                     (unsigned long)offset,
                     (unsigned long)num_samples,
                     (long long)elapsed_ms);
            next_progress_report += 10;
        }

        vTaskDelay(1);
    }

    int64_t total_ms = (esp_timer_get_time() - process_start_us) / 1000;
    ESP_LOGI(TAG, "Audio processing complete in %lld ms", (long long)total_ms);

    // 8. Write processed audio to SD card
    err = wav_write_f32(WAV_OUTPUT_PATH, audio_buf, num_samples, sample_rate);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to write output WAV to SD card");
    }

    // 9. Stream processed audio to host for playback
    ESP_LOGI(TAG, "--- Phase 3: Streaming to Host ---");
    stream_audio_to_host(audio_buf, num_samples, sample_rate);

    // Cleanup
    free(audio_buf);
    audio_processor_free();
    coeff_loader_free(&bank);
    sd_card_deinit();

    ESP_LOGI(TAG, "=== All done! ===");
}
