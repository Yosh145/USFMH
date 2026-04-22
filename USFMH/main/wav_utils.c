#include "wav_utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "wav_utils";
#define WAV_IO_YIELD_INTERVAL 8

// Standard WAV file header structures
typedef struct __attribute__((packed)) {
    char     riff_tag[4];    // "RIFF"
    uint32_t file_size;      // file size - 8
    char     wave_tag[4];    // "WAVE"
} riff_header_t;

typedef struct __attribute__((packed)) {
    char     chunk_id[4];
    uint32_t chunk_size;
} chunk_header_t;

typedef struct __attribute__((packed)) {
    uint16_t audio_format;   // 1 = PCM
    uint16_t num_channels;
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t block_align;
    uint16_t bits_per_sample;
} fmt_chunk_t;

esp_err_t wav_parse_header(const char *filepath, wav_header_info_t *info)
{
    FILE *f = fopen(filepath, "rb");
    if (!f) {
        ESP_LOGE(TAG, "Cannot open %s", filepath);
        return ESP_ERR_NOT_FOUND;
    }

    riff_header_t riff;
    if (fread(&riff, sizeof(riff), 1, f) != 1 ||
        memcmp(riff.riff_tag, "RIFF", 4) != 0 ||
        memcmp(riff.wave_tag, "WAVE", 4) != 0) {
        ESP_LOGE(TAG, "Not a valid WAV file");
        fclose(f);
        return ESP_ERR_INVALID_ARG;
    }

    bool found_fmt = false, found_data = false;
    fmt_chunk_t fmt;

    while (!found_data) {
        chunk_header_t chunk;
        if (fread(&chunk, sizeof(chunk), 1, f) != 1) {
            break;
        }

        if (memcmp(chunk.chunk_id, "fmt ", 4) == 0) {
            if (chunk.chunk_size < sizeof(fmt)) {
                ESP_LOGE(TAG, "fmt chunk too small");
                fclose(f);
                return ESP_ERR_INVALID_SIZE;
            }
            if (fread(&fmt, sizeof(fmt), 1, f) != 1) {
                break;
            }
            // Skip extra fmt bytes if any
            if (chunk.chunk_size > sizeof(fmt)) {
                fseek(f, chunk.chunk_size - sizeof(fmt), SEEK_CUR);
            }
            found_fmt = true;
        } else if (memcmp(chunk.chunk_id, "data", 4) == 0) {
            if (!found_fmt) {
                ESP_LOGE(TAG, "data chunk before fmt");
                fclose(f);
                return ESP_ERR_INVALID_STATE;
            }
            info->data_size = chunk.chunk_size;
            info->data_offset = ftell(f);
            found_data = true;
        } else {
            // Skip unknown chunk
            fseek(f, chunk.chunk_size, SEEK_CUR);
        }
    }

    fclose(f);

    if (!found_fmt || !found_data) {
        ESP_LOGE(TAG, "Missing fmt or data chunk");
        return ESP_ERR_INVALID_ARG;
    }

    if (fmt.audio_format != 1) {
        ESP_LOGE(TAG, "Only PCM format supported (got %d)", fmt.audio_format);
        return ESP_ERR_NOT_SUPPORTED;
    }

    info->num_channels = fmt.num_channels;
    info->sample_rate = fmt.sample_rate;
    info->bits_per_sample = fmt.bits_per_sample;
    info->num_samples = info->data_size / (fmt.bits_per_sample / 8);

    ESP_LOGI(TAG, "WAV: %lu Hz, %d-bit, %d ch, %lu samples",
             (unsigned long)info->sample_rate, info->bits_per_sample,
             info->num_channels, (unsigned long)info->num_samples);

    return ESP_OK;
}

esp_err_t wav_read_f32(const char *filepath, float **out_buf, uint32_t *out_num_samples,
                       uint32_t *out_sample_rate)
{
    wav_header_info_t info;
    esp_err_t err = wav_parse_header(filepath, &info);
    if (err != ESP_OK) return err;

    if (info.bits_per_sample != 16 || info.num_channels != 1) {
        ESP_LOGE(TAG, "Only 16-bit mono WAV supported");
        return ESP_ERR_NOT_SUPPORTED;
    }

    float *buf = malloc(info.num_samples * sizeof(float));
    if (!buf) {
        ESP_LOGE(TAG, "Failed to allocate %lu floats", (unsigned long)info.num_samples);
        return ESP_ERR_NO_MEM;
    }

    FILE *f = fopen(filepath, "rb");
    if (!f) {
        free(buf);
        return ESP_ERR_NOT_FOUND;
    }

    fseek(f, info.data_offset, SEEK_SET);

    // Read in chunks to avoid allocating a second large buffer
    const int CHUNK = 1024;
    int16_t tmp[1024];
    uint32_t remaining = info.num_samples;
    uint32_t offset = 0;

    while (remaining > 0) {
        uint32_t to_read = remaining < CHUNK ? remaining : CHUNK;
        size_t read = fread(tmp, sizeof(int16_t), to_read, f);
        if (read == 0) break;
        for (uint32_t i = 0; i < read; i++) {
            buf[offset + i] = (float)tmp[i] / 32768.0f;
        }
        offset += read;
        remaining -= read;

        if (((offset / CHUNK) % WAV_IO_YIELD_INTERVAL) == 0) {
            vTaskDelay(1);
        }
    }

    fclose(f);

    *out_buf = buf;
    *out_num_samples = offset;
    *out_sample_rate = info.sample_rate;

    ESP_LOGI(TAG, "Read %lu samples at %lu Hz", (unsigned long)offset,
             (unsigned long)info.sample_rate);
    return ESP_OK;
}

esp_err_t wav_write_f32(const char *filepath, const float *buf, uint32_t num_samples,
                        uint32_t sample_rate)
{
    FILE *f = fopen(filepath, "wb");
    if (!f) {
        ESP_LOGE(TAG, "Cannot create %s", filepath);
        return ESP_ERR_NOT_FOUND;
    }

    uint32_t data_size = num_samples * sizeof(int16_t);

    // Write RIFF header
    riff_header_t riff = {
        .riff_tag = {'R', 'I', 'F', 'F'},
        .file_size = 36 + data_size,
        .wave_tag = {'W', 'A', 'V', 'E'}
    };
    fwrite(&riff, sizeof(riff), 1, f);

    // Write fmt chunk
    chunk_header_t fmt_hdr = {
        .chunk_id = {'f', 'm', 't', ' '},
        .chunk_size = 16
    };
    fwrite(&fmt_hdr, sizeof(fmt_hdr), 1, f);

    fmt_chunk_t fmt = {
        .audio_format = 1,
        .num_channels = 1,
        .sample_rate = sample_rate,
        .byte_rate = sample_rate * sizeof(int16_t),
        .block_align = sizeof(int16_t),
        .bits_per_sample = 16
    };
    fwrite(&fmt, sizeof(fmt), 1, f);

    // Write data chunk header
    chunk_header_t data_hdr = {
        .chunk_id = {'d', 'a', 't', 'a'},
        .chunk_size = data_size
    };
    fwrite(&data_hdr, sizeof(data_hdr), 1, f);

    // Write samples in chunks
    const int CHUNK = 1024;
    int16_t tmp[1024];
    uint32_t remaining = num_samples;
    uint32_t offset = 0;

    while (remaining > 0) {
        uint32_t to_write = remaining < CHUNK ? remaining : CHUNK;
        for (uint32_t i = 0; i < to_write; i++) {
            float s = buf[offset + i];
            // Clamp to [-1.0, 1.0]
            if (s > 1.0f) s = 1.0f;
            if (s < -1.0f) s = -1.0f;
            tmp[i] = (int16_t)(s * 32767.0f);
        }
        fwrite(tmp, sizeof(int16_t), to_write, f);
        offset += to_write;
        remaining -= to_write;

        if (((offset / CHUNK) % WAV_IO_YIELD_INTERVAL) == 0) {
            vTaskDelay(1);
        }
    }

    fclose(f);
    ESP_LOGI(TAG, "Wrote %lu samples to %s", (unsigned long)num_samples, filepath);
    return ESP_OK;
}
