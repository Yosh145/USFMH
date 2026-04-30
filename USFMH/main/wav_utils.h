#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

typedef struct {
    uint16_t num_channels;
    uint32_t sample_rate;
    uint16_t bits_per_sample;
    uint32_t data_size;       // in bytes
    uint32_t num_samples;     // total samples (data_size / bytes_per_sample)
    uint32_t data_offset;     // byte offset where PCM data starts in the file
} wav_header_info_t;

/**
 * Parse WAV header from a file on the SD card.
 * Returns ESP_OK on success.
 */
esp_err_t wav_parse_header(const char *filepath, wav_header_info_t *info);

/**
 * Read all PCM samples from a 16-bit mono WAV into a float buffer (normalized -1.0 to 1.0).
 * Caller must free the returned buffer with free().
 * Sets *out_num_samples to the number of float samples read.
 */
esp_err_t wav_read_f32(const char *filepath, float **out_buf, uint32_t *out_num_samples,
                       uint32_t *out_sample_rate);

/**
 * Write float samples (normalized -1.0 to 1.0) to a 16-bit mono WAV file on the SD card.
 */
esp_err_t wav_write_f32(const char *filepath, const float *buf, uint32_t num_samples,
                        uint32_t sample_rate);
