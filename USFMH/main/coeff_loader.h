#pragma once

#include <stdint.h>
#include "esp_err.h"

/**
 * Binary coefficient file layout (little-endian, all float32):
 *
 *   Offset  Size              Field
 *   ──────  ────              ─────
 *   0       4 bytes           magic   0x46495242 ("FIRB")
 *   4       4 bytes (uint32)  num_bands
 *   8       4 bytes (uint32)  num_taps
 *   12      num_bands × 4     min_freqs[num_bands]
 *           num_bands × 4     max_freqs[num_bands]
 *           num_bands × 4     center_freqs[num_bands]
 *           num_bands × num_taps × 4  coeffs[num_bands][num_taps]
 */

#define COEFF_FILE_MAGIC 0x46495242  /* "FIRB" */

typedef struct {
    uint32_t num_bands;
    uint32_t num_taps;
    float   *min_freqs;      // [num_bands]  — PSRAM
    float   *max_freqs;      // [num_bands]  — PSRAM
    float   *center_freqs;   // [num_bands]  — PSRAM
    float   *coeffs;         // [num_bands * num_taps]  — PSRAM (row-major)
} filter_bank_t;

/**
 * Load filter bank coefficients from a binary file on the SD card into PSRAM.
 * Caller must eventually call coeff_loader_free() to release memory.
 *
 * @param path  Full VFS path, e.g. "/sdcard/filters.bin"
 * @param bank  Output struct populated on success
 */
esp_err_t coeff_loader_load(const char *path, filter_bank_t *bank);

/**
 * Free all PSRAM buffers allocated by coeff_loader_load().
 */
void coeff_loader_free(filter_bank_t *bank);
