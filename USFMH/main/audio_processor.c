#include "audio_processor.h"
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

static const char *TAG = "audio_proc";

#define PROCESS_YIELD_INTERVAL  4096

// Per-band filter state (header only — history lives in PSRAM)
typedef struct {
    float  *history;          // PSRAM: [num_taps]
    size_t  history_index;
    float   center_freq_hz;
    bool    active;
} band_filter_t;

static band_filter_t *filters = NULL;      // PSRAM array [num_bands]
static int *active_filter_indices = NULL;   // PSRAM array [num_bands]
static int active_filter_count = 0;

static const filter_bank_t *s_bank = NULL;
static uint32_t s_num_bands = 0;
static uint32_t s_num_taps = 0;

static bool band_is_heard(const hearing_result_t *result, float freq_hz)
{
    int best_index = 0;
    float best_delta = fabsf(freq_hz - test_frequencies[0]);

    for (int i = 1; i < NUM_TEST_FREQS; i++) {
        float delta = fabsf(freq_hz - test_frequencies[i]);
        if (delta < best_delta) {
            best_delta = delta;
            best_index = i;
        }
    }

    return result->heard[best_index];
}

esp_err_t audio_processor_init(const filter_bank_t *bank,
                               const hearing_result_t *result,
                               uint32_t sample_rate)
{
    s_bank = bank;
    s_num_bands = bank->num_bands;
    s_num_taps = bank->num_taps;

    ESP_LOGI(TAG, "Initializing %lu-band FIR filter bank (%lu taps) at %lu Hz",
             (unsigned long)s_num_bands, (unsigned long)s_num_taps,
             (unsigned long)sample_rate);

    float nyquist = (float)sample_rate / 2.0f;

    if (sample_rate != 48000U) {
        ESP_LOGW(TAG, "Filter bank was generated for 48000 Hz input, got %lu Hz",
                 (unsigned long)sample_rate);
    }

    // Allocate filter state array in PSRAM
    filters = heap_caps_calloc(s_num_bands, sizeof(band_filter_t), MALLOC_CAP_SPIRAM);
    active_filter_indices = heap_caps_malloc(s_num_bands * sizeof(int), MALLOC_CAP_SPIRAM);
    if (!filters || !active_filter_indices) {
        ESP_LOGE(TAG, "Failed to allocate filter state arrays");
        return ESP_ERR_NO_MEM;
    }

    int active_count = 0;

    for (uint32_t i = 0; i < s_num_bands; i++) {
        float low_cut = bank->min_freqs[i];
        float high_cut = bank->max_freqs[i];
        float center = bank->center_freqs[i];

        if (low_cut >= high_cut || center >= nyquist) {
            filters[i].active = false;
            continue;
        }

        // Allocate history buffer in PSRAM
        filters[i].history = heap_caps_calloc(s_num_taps, sizeof(float), MALLOC_CAP_SPIRAM);
        if (!filters[i].history) {
            ESP_LOGE(TAG, "Failed to allocate history buffer for band %lu", (unsigned long)i);
            audio_processor_free();
            return ESP_ERR_NO_MEM;
        }

        filters[i].history_index = 0;
        filters[i].center_freq_hz = center;
        filters[i].active = band_is_heard(result, center);

        if (filters[i].active) {
            active_filter_indices[active_count] = (int)i;
            active_count++;
        }

        ESP_LOGI(TAG, "Band %2lu: %.1f-%.1f Hz, center %.0f Hz, %s",
                 (unsigned long)i, low_cut, high_cut, center,
                 filters[i].active ? "ACTIVE" : "off");
    }

    active_filter_count = active_count;

    if (active_count == 0) {
        ESP_LOGW(TAG, "No active bands — output will pass through unchanged");
    }

    size_t hist_total = (size_t)active_count * s_num_taps * sizeof(float);
    ESP_LOGI(TAG, "%d active FIR bands, %lu taps each (%.1f KB history in PSRAM)",
             active_count, (unsigned long)s_num_taps, (double)hist_total / 1024.0);

    return ESP_OK;
}

esp_err_t audio_processor_apply(float *samples, uint32_t num_samples)
{
    if (active_filter_count == 0) {
        return ESP_OK;
    }

    int num_taps = (int)s_num_taps;

    // Accumulation buffer in PSRAM
    float *output = heap_caps_calloc(num_samples, sizeof(float), MALLOC_CAP_SPIRAM);
    if (!output) {
        ESP_LOGE(TAG, "Failed to allocate output buffer (%lu floats)", (unsigned long)num_samples);
        return ESP_ERR_NO_MEM;
    }

    // Band-first loop: keeps one band's coefficients and history hot in cache
    for (int active_idx = 0; active_idx < active_filter_count; active_idx++) {
        int fi = active_filter_indices[active_idx];
        band_filter_t *filt = &filters[fi];
        const float *coeffs = s_bank->coeffs + (size_t)fi * s_num_taps;
        float *hist = filt->history;
        int head = (int)filt->history_index;

        for (uint32_t s = 0; s < num_samples; s++) {
            hist[head] = samples[s];

            // FIR convolution: two-pass over circular buffer (no branch in MAC loop)
            float acc = 0.0f;

            for (int tap = 0; tap <= head; tap++) {
                acc += coeffs[tap] * hist[head - tap];
            }
            for (int tap = head + 1; tap < num_taps; tap++) {
                acc += coeffs[tap] * hist[num_taps - tap + head];
            }

            output[s] += acc;

            if (++head >= num_taps) head = 0;

            if ((s & (PROCESS_YIELD_INTERVAL - 1)) == 0) {
                taskYIELD();
            }
        }

        filt->history_index = (size_t)head;
    }

    memcpy(samples, output, num_samples * sizeof(float));
    heap_caps_free(output);

    return ESP_OK;
}

void audio_processor_reset(void)
{
    if (!filters) return;
    for (uint32_t i = 0; i < s_num_bands; i++) {
        if (filters[i].history) {
            memset(filters[i].history, 0, s_num_taps * sizeof(float));
            filters[i].history_index = 0;
        }
    }
}

void audio_processor_free(void)
{
    if (filters) {
        for (uint32_t i = 0; i < s_num_bands; i++) {
            heap_caps_free(filters[i].history);
        }
        heap_caps_free(filters);
        filters = NULL;
    }
    heap_caps_free(active_filter_indices);
    active_filter_indices = NULL;
    active_filter_count = 0;
    s_bank = NULL;
    s_num_bands = 0;
    s_num_taps = 0;
}
