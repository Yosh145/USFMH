#include "audio_processor.h"
#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "dsps_biquad.h"

static const char *TAG = "audio_proc";

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Per-band filter state
typedef struct {
    float coeffs[5];    // [b0, b1, b2, a1, a2] (normalized, a0=1)
    float delay[2];     // delay line [w0, w1]
    bool  active;       // false if gain_db == 0 (skip this filter)
} band_filter_t;

static band_filter_t filters[NUM_TEST_FREQS];

/**
 * Compute peaking EQ biquad coefficients using the RBJ Audio Cookbook formula.
 *
 * H(s) = (s^2 + s*(A/Q) + 1) / (s^2 + s/(A*Q) + 1)
 *
 * @param coeffs      Output: [b0, b1, b2, a1, a2] (a0 normalized to 1)
 * @param center_freq Center frequency in Hz
 * @param sample_rate Sample rate in Hz
 * @param gain_db     Gain at center frequency in dB (positive = boost)
 * @param Q           Quality factor (bandwidth control)
 */
static void compute_peaking_eq(float *coeffs, float center_freq, float sample_rate,
                                float gain_db, float Q)
{
    float A     = powf(10.0f, gain_db / 40.0f);
    float w0    = 2.0f * (float)M_PI * center_freq / sample_rate;
    float cos_w = cosf(w0);
    float sin_w = sinf(w0);
    float alpha = sin_w / (2.0f * Q);

    float b0 =  1.0f + alpha * A;
    float b1 = -2.0f * cos_w;
    float b2 =  1.0f - alpha * A;
    float a0 =  1.0f + alpha / A;
    float a1 = -2.0f * cos_w;
    float a2 =  1.0f - alpha / A;

    // Normalize by a0
    coeffs[0] = b0 / a0;
    coeffs[1] = b1 / a0;
    coeffs[2] = b2 / a0;
    coeffs[3] = a1 / a0;
    coeffs[4] = a2 / a0;
}

esp_err_t audio_processor_init(const hearing_result_t *result, uint32_t sample_rate)
{
    ESP_LOGI(TAG, "Initializing %d-band EQ at %lu Hz", NUM_TEST_FREQS,
             (unsigned long)sample_rate);

    float nyquist = (float)sample_rate / 2.0f;
    int active_count = 0;

    for (int i = 0; i < NUM_TEST_FREQS; i++) {
        memset(&filters[i], 0, sizeof(band_filter_t));

        float gain = result->gain_db[i];

        // Skip bands with no gain change
        if (fabsf(gain) < 0.1f) {
            filters[i].active = false;
            continue;
        }

        float center = test_frequencies[i];

        // Skip if center frequency is above Nyquist
        if (center >= nyquist * 0.95f) {
            ESP_LOGW(TAG, "Band %d (%.0f Hz) above Nyquist, skipping", i, center);
            filters[i].active = false;
            continue;
        }

        // Compute Q from band edges: Q = f_center / bandwidth
        float f_low  = band_edges[i];
        float f_high = band_edges[i + 1];
        float bandwidth = f_high - f_low;
        float Q = center / bandwidth;

        // Clamp Q to a reasonable range
        if (Q < 0.5f) Q = 0.5f;
        if (Q > 10.0f) Q = 10.0f;

        compute_peaking_eq(filters[i].coeffs, center, (float)sample_rate, gain, Q);
        filters[i].active = true;
        active_count++;

        ESP_LOGI(TAG, "Band %2d: %.0f Hz, Q=%.2f, gain=%.1f dB",
                 i, center, Q, gain);
    }

    ESP_LOGI(TAG, "%d active EQ bands out of %d", active_count, NUM_TEST_FREQS);
    return ESP_OK;
}

esp_err_t audio_processor_apply(float *samples, uint32_t num_samples)
{
    // Apply each active biquad filter in cascade
    for (int i = 0; i < NUM_TEST_FREQS; i++) {
        if (!filters[i].active) continue;

        esp_err_t err = dsps_biquad_f32(samples, samples, (int)num_samples,
                                         filters[i].coeffs, filters[i].delay);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Biquad filter %d failed: %d", i, err);
            return err;
        }
    }

    return ESP_OK;
}

void audio_processor_reset(void)
{
    for (int i = 0; i < NUM_TEST_FREQS; i++) {
        filters[i].delay[0] = 0.0f;
        filters[i].delay[1] = 0.0f;
    }
}
