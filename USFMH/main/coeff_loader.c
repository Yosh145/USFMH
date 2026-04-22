#include "coeff_loader.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_heap_caps.h"

static const char *TAG = "coeff_loader";

esp_err_t coeff_loader_load(const char *path, filter_bank_t *bank)
{
    memset(bank, 0, sizeof(*bank));

    FILE *f = fopen(path, "rb");
    if (!f) {
        ESP_LOGE(TAG, "Cannot open %s", path);
        return ESP_ERR_NOT_FOUND;
    }

    // Read and verify header
    uint32_t magic;
    if (fread(&magic, 4, 1, f) != 1 || magic != COEFF_FILE_MAGIC) {
        ESP_LOGE(TAG, "Bad magic (got 0x%08lx, expected 0x%08lx)",
                 (unsigned long)magic, (unsigned long)COEFF_FILE_MAGIC);
        fclose(f);
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t num_bands, num_taps;
    if (fread(&num_bands, 4, 1, f) != 1 || fread(&num_taps, 4, 1, f) != 1) {
        ESP_LOGE(TAG, "Failed to read header");
        fclose(f);
        return ESP_ERR_INVALID_SIZE;
    }

    if (num_bands == 0 || num_bands > 128 || num_taps == 0 || num_taps > 16384) {
        ESP_LOGE(TAG, "Unreasonable dimensions: %lu bands, %lu taps",
                 (unsigned long)num_bands, (unsigned long)num_taps);
        fclose(f);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Loading %lu bands × %lu taps from %s",
             (unsigned long)num_bands, (unsigned long)num_taps, path);

    bank->num_bands = num_bands;
    bank->num_taps = num_taps;

    // Allocate frequency arrays in PSRAM
    size_t freq_bytes = num_bands * sizeof(float);
    bank->min_freqs    = heap_caps_malloc(freq_bytes, MALLOC_CAP_SPIRAM);
    bank->max_freqs    = heap_caps_malloc(freq_bytes, MALLOC_CAP_SPIRAM);
    bank->center_freqs = heap_caps_malloc(freq_bytes, MALLOC_CAP_SPIRAM);

    // Allocate coefficient matrix in PSRAM
    size_t coeff_bytes = (size_t)num_bands * num_taps * sizeof(float);
    bank->coeffs = heap_caps_malloc(coeff_bytes, MALLOC_CAP_SPIRAM);

    if (!bank->min_freqs || !bank->max_freqs || !bank->center_freqs || !bank->coeffs) {
        ESP_LOGE(TAG, "PSRAM allocation failed (need %lu KB for coeffs)",
                 (unsigned long)(coeff_bytes / 1024));
        coeff_loader_free(bank);
        fclose(f);
        return ESP_ERR_NO_MEM;
    }

    // Read frequency arrays
    if (fread(bank->min_freqs, freq_bytes, 1, f) != 1 ||
        fread(bank->max_freqs, freq_bytes, 1, f) != 1 ||
        fread(bank->center_freqs, freq_bytes, 1, f) != 1) {
        ESP_LOGE(TAG, "Failed to read frequency arrays");
        coeff_loader_free(bank);
        fclose(f);
        return ESP_ERR_INVALID_SIZE;
    }

    // Read coefficient matrix
    if (fread(bank->coeffs, coeff_bytes, 1, f) != 1) {
        ESP_LOGE(TAG, "Failed to read coefficient matrix");
        coeff_loader_free(bank);
        fclose(f);
        return ESP_ERR_INVALID_SIZE;
    }

    fclose(f);

    ESP_LOGI(TAG, "Loaded successfully: %.1f KB coefficients in PSRAM",
             (double)coeff_bytes / 1024.0);
    ESP_LOGI(TAG, "Frequency range: %.1f Hz — %.1f Hz",
             bank->min_freqs[0], bank->max_freqs[num_bands - 1]);

    return ESP_OK;
}

void coeff_loader_free(filter_bank_t *bank)
{
    heap_caps_free(bank->min_freqs);
    heap_caps_free(bank->max_freqs);
    heap_caps_free(bank->center_freqs);
    heap_caps_free(bank->coeffs);
    memset(bank, 0, sizeof(*bank));
}
