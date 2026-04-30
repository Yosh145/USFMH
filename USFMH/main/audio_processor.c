/*
 * audio_processor.c — FFT-based filterbank (overlap-save)
 *
 * Applies a 32-band, per-band-gain EQ to an audio buffer by building a single
 * composite frequency-domain mask H[k] from the hearing-test result, then
 * convolving the input with that mask via the standard overlap-save method.
 *
 *   Block size N = FFT_N = 16384        (FFT length)
 *   Filter len  M = FFT_N / 2 + 1 = 8193 taps (linear-phase, delay D = M/2 = 4096 samples)
 *   Hop        L = N - M + 1 = 8192     (new samples consumed per FFT block)
 *   Bin width  fs / N = 48000 / 16384 ≈ 2.93 Hz
 *
 * Mask construction:
 *   For each bin k (freq f_k = k*fs/N):
 *     H_mag[k] = Σ_j gain[j] * w_j(f_k)
 *   where w_j is a raised-cosine window over band j's passband [fmin,fmax]
 *   with a 2-bin crossover at each edge. Adjacent bands' crossovers add to 1.0,
 *   so a flat hearing profile gives perfect reconstruction (output = input).
 *
 *   Linear phase is applied by multiplying H_mag[k] by exp(-j*2*pi*k*D/N),
 *   shifting the zero-phase impulse response into a causal length-M window.
 *   The resulting group delay of D samples is compensated at the end of
 *   audio_processor_apply() by skipping the first D output samples.
 *
 * Per-band gain mapping:
 *   Each of the 32 filter bands is matched to the nearest of the 27 hearing
 *   test frequencies. If that test frequency was heard, the band's linear gain
 *   is 10^(gain_db/20). If not heard, the band is muted (gain = 0). This
 *   preserves the current ESP32 debug-sample behavior (only the targeted band
 *   passes). Future groups can change this mapping here without touching the
 *   FFT machinery.
 *
 * Why N = 16384?
 *   The narrowest band is ~8 Hz wide near 35 Hz. Bin width must be small enough
 *   to resolve this — 2.93 Hz/bin gives ~3 bins inside the band and enough room
 *   for smooth transitions. N = 8192 is too coarse for the low bands; N = 32768
 *   works too but doubles memory and gives diminishing quality returns.
 */

#include "audio_processor.h"
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_dsp.h"

static const char *TAG = "audio_proc";

#define FFT_N            16384
#define FILTER_LEN       (FFT_N / 2 + 1)           // M = 8193 taps
#define BLOCK_HOP        (FFT_N - FILTER_LEN + 1)  // L = 8192 new samples per block
#define GROUP_DELAY      (FILTER_LEN / 2)          // D = 4096 samples (linear-phase delay)
#define XOVER_BINS       2.0f                      // raised-cosine transition width, in bins

static float            *s_fft_data = NULL;    // [FFT_N * 2] interleaved complex workbuffer
static float            *s_mask     = NULL;    // [FFT_N * 2] interleaved complex mask
static bool              s_fft_tables_ready = false;
static const filter_bank_t *s_bank = NULL;
static uint32_t          s_sample_rate = 48000;


/* Find the nearest hearing-test frequency to a band center. Returns the
 * linear gain to apply (0 if the band is muted). */
static float band_linear_gain(const hearing_result_t *result, float band_center_hz)
{
    int best = 0;
    float best_delta = fabsf(band_center_hz - test_frequencies[0]);
    for (int i = 1; i < NUM_TEST_FREQS; i++) {
        float delta = fabsf(band_center_hz - test_frequencies[i]);
        if (delta < best_delta) {
            best_delta = delta;
            best = i;
        }
    }
    if (!result->heard[best]) {
        return 0.0f;  // Not heard → muted
    }
    return powf(10.0f, result->gain_db[best] / 20.0f);
}

/* Raised-cosine band weight. Returns 0 outside [fmin-Δ/2, fmax+Δ/2],
 * 1 inside [fmin+Δ/2, fmax-Δ/2], half-cosine transition at each edge.
 * At a boundary shared with an adjacent band, each band evaluates to 0.5,
 * so the two bands sum to 1.0 → partition of unity. */
static float band_weight(float freq_hz, float fmin, float fmax, float delta_hz)
{
    const float half = delta_hz * 0.5f;
    if (freq_hz <= fmin - half || freq_hz >= fmax + half) {
        return 0.0f;
    }
    if (freq_hz >= fmin + half && freq_hz <= fmax - half) {
        return 1.0f;
    }
    if (freq_hz < fmin + half) {
        // Rising edge centered at fmin
        float t = (freq_hz - (fmin - half)) / delta_hz;   // 0 → 1
        return 0.5f * (1.0f - cosf((float)M_PI * t));
    }
    // Falling edge centered at fmax
    float t = (freq_hz - (fmax - half)) / delta_hz;       // 0 → 1
    return 0.5f * (1.0f + cosf((float)M_PI * t));
}

/* Build the composite frequency-domain mask s_mask[k] (complex, length FFT_N).
 * Steps:
 *   1. Real magnitude response H_mag[k] for k = 0..N/2 from active bands.
 *   2. Mirror to full [0..N-1] for conjugate symmetry (real-input FFT property).
 *   3. Apply linear-phase delay: multiply by exp(-j*2*pi*k*D/N). */
static esp_err_t build_mask(const filter_bank_t *bank,
                            const hearing_result_t *result,
                            uint32_t sample_rate)
{
    const float bin_hz = (float)sample_rate / (float)FFT_N;
    const float delta_hz = XOVER_BINS * bin_hz;

    float *gain = heap_caps_calloc(bank->num_bands, sizeof(float), MALLOC_CAP_SPIRAM);
    float *Hmag = heap_caps_calloc(FFT_N / 2 + 1, sizeof(float), MALLOC_CAP_SPIRAM);
    if (!gain || !Hmag) {
        heap_caps_free(gain);
        heap_caps_free(Hmag);
        return ESP_ERR_NO_MEM;
    }

    int active_count = 0;
    for (uint32_t j = 0; j < bank->num_bands; j++) {
        gain[j] = band_linear_gain(result, bank->center_freqs[j]);
        if (gain[j] > 0.0f) active_count++;
        ESP_LOGI(TAG, "Band %2u: %7.1f-%7.1f Hz (center %7.1f) gain=%.3f %s",
                 (unsigned)j, (double)bank->min_freqs[j],
                 (double)bank->max_freqs[j], (double)bank->center_freqs[j],
                 (double)gain[j], (gain[j] > 0.0f) ? "ACTIVE" : "muted");
    }

    for (int k = 0; k <= FFT_N / 2; k++) {
        float f = (float)k * bin_hz;
        float m = 0.0f;
        for (uint32_t j = 0; j < bank->num_bands; j++) {
            if (gain[j] == 0.0f) continue;
            m += gain[j] * band_weight(f, bank->min_freqs[j], bank->max_freqs[j], delta_hz);
        }
        Hmag[k] = m;
    }

    heap_caps_free(gain);

    // Fill full complex mask with linear-phase delay of D samples.
    for (int k = 0; k < FFT_N; k++) {
        float mag = (k <= FFT_N / 2) ? Hmag[k] : Hmag[FFT_N - k];  // mirror for k > N/2
        float phase = -2.0f * (float)M_PI * (float)k * (float)GROUP_DELAY / (float)FFT_N;
        s_mask[2 * k + 0] = mag * cosf(phase);
        s_mask[2 * k + 1] = mag * sinf(phase);
    }

    heap_caps_free(Hmag);

    ESP_LOGI(TAG, "Composite mask built: %d/%u bands active, bin = %.2f Hz, crossover = %.2f Hz",
             active_count, (unsigned)bank->num_bands, (double)bin_hz, (double)delta_hz);
    return ESP_OK;
}

esp_err_t audio_processor_init(const filter_bank_t *bank,
                               const hearing_result_t *result,
                               uint32_t sample_rate)
{
    s_bank = bank;
    s_sample_rate = sample_rate;

    ESP_LOGI(TAG, "FFT filterbank init: N=%d, M=%d, L=%d, D=%d samples",
             FFT_N, FILTER_LEN, BLOCK_HOP, GROUP_DELAY);

    if (sample_rate != 48000U) {
        ESP_LOGE(TAG, "Filter bank requires 48000 Hz input, got %u Hz",
                 (unsigned)sample_rate);
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_fft_tables_ready) {
        esp_err_t err = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
        if (err != ESP_OK && err != ESP_ERR_DSP_REINITIALIZED) {
            ESP_LOGE(TAG, "dsps_fft2r_init_fc32 failed: %d", err);
            return err;
        }
        s_fft_tables_ready = true;
    }

    // Allocate FFT workbuf + mask in PSRAM (128 KB each). 16-byte aligned for
    // esp-dsp assembly paths.
    s_fft_data = heap_caps_aligned_alloc(16, (size_t)FFT_N * 2 * sizeof(float),
                                         MALLOC_CAP_SPIRAM);
    s_mask     = heap_caps_aligned_alloc(16, (size_t)FFT_N * 2 * sizeof(float),
                                         MALLOC_CAP_SPIRAM);
    if (!s_fft_data || !s_mask) {
        ESP_LOGE(TAG, "FFT buffer allocation failed (need 2 x %u bytes)",
                 (unsigned)(FFT_N * 2 * sizeof(float)));
        audio_processor_free();
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = build_mask(bank, result, sample_rate);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Mask construction failed");
        audio_processor_free();
        return err;
    }

    ESP_LOGI(TAG, "FFT filterbank ready (≈ %.0f KB PSRAM)",
             (double)(2 * FFT_N * 2 * sizeof(float)) / 1024.0);
    return ESP_OK;
}

esp_err_t audio_processor_apply(float *samples, uint32_t num_samples)
{
    if (!s_fft_data || !s_mask) {
        ESP_LOGE(TAG, "audio_processor_apply called before init");
        return ESP_ERR_INVALID_STATE;
    }
    if (num_samples == 0) return ESP_OK;

    // Output buffer holds num_samples + GROUP_DELAY samples: the extra D samples
    // at the tail correspond to the filter "flushing out" after input ends, and
    // are discarded when aligning back to the caller's timeline.
    const uint32_t output_len = num_samples + GROUP_DELAY;
    float *output = heap_caps_calloc(output_len, sizeof(float), MALLOC_CAP_SPIRAM);
    if (!output) {
        ESP_LOGE(TAG, "Failed to allocate output buffer (%u floats)", (unsigned)output_len);
        return ESP_ERR_NO_MEM;
    }

    // Number of FFT blocks needed to cover output_len samples (each block emits L).
    const int num_blocks = (int)((output_len + BLOCK_HOP - 1) / BLOCK_HOP);
    const float inv_N = 1.0f / (float)FFT_N;

    ESP_LOGI(TAG, "Processing %u samples in %d FFT blocks of %d",
             (unsigned)num_samples, num_blocks, FFT_N);

    int next_progress = 10;
    for (int b = 0; b < num_blocks; b++) {
        // --- Load N real samples from the virtually-padded input stream ---
        // Stream x'[p] is: zeros for p < M-1 (pre-pad), samples[p - (M-1)] in range,
        // zeros beyond num_samples (post-pad, flushes filter tail).
        const int base = b * BLOCK_HOP;
        for (int n = 0; n < FFT_N; n++) {
            int p = base + n;
            int idx = p - (FILTER_LEN - 1);
            float x = 0.0f;
            if (idx >= 0 && idx < (int)num_samples) {
                x = samples[idx];
            }
            s_fft_data[2 * n + 0] = x;
            s_fft_data[2 * n + 1] = 0.0f;
        }

        // --- Forward FFT ---
        dsps_fft2r_fc32(s_fft_data, FFT_N);
        dsps_bit_rev_fc32(s_fft_data, FFT_N);

        // --- Pointwise complex multiply by mask ---
        for (int k = 0; k < FFT_N; k++) {
            float xr = s_fft_data[2 * k + 0];
            float xi = s_fft_data[2 * k + 1];
            float hr = s_mask    [2 * k + 0];
            float hi = s_mask    [2 * k + 1];
            s_fft_data[2 * k + 0] = xr * hr - xi * hi;
            s_fft_data[2 * k + 1] = xr * hi + xi * hr;
        }

        // --- Inverse FFT via conjugate trick ---
        //   IFFT(X)[n] = (1/N) * conj( FFT( conj(X) )[n] )
        //   We only need the real part (input was real, so output is real up to
        //   rounding), and conj doesn't change Re, so we just negate imag before
        //   the forward FFT and scale the real part at the end.
        for (int k = 0; k < FFT_N; k++) {
            s_fft_data[2 * k + 1] = -s_fft_data[2 * k + 1];
        }
        dsps_fft2r_fc32(s_fft_data, FFT_N);
        dsps_bit_rev_fc32(s_fft_data, FFT_N);

        // --- Copy valid samples out ---
        // In overlap-save, the first (M-1) samples of each IFFT block are
        // circular-convolution wrap and must be discarded; the last L samples
        // are the correct linear-convolution output for this block.
        for (int n = 0; n < BLOCK_HOP; n++) {
            uint32_t out_idx = (uint32_t)(base + n);
            if (out_idx >= output_len) break;
            output[out_idx] = s_fft_data[2 * (FILTER_LEN - 1 + n) + 0] * inv_N;
        }

        int pct = ((b + 1) * 100) / num_blocks;
        if (pct >= next_progress) {
            ESP_LOGI(TAG, "FFT filterbank: %d%% (%d/%d)", pct, b + 1, num_blocks);
            next_progress += 10;
        }
        vTaskDelay(1);  // yield to other tasks / feed the watchdog
    }

    // Compensate linear-phase delay: output[n] = y'(M-1 + n), and the filter's
    // peak is at D = M/2, so samples[n] == output[n + D] after alignment.
    memcpy(samples, output + GROUP_DELAY, (size_t)num_samples * sizeof(float));
    heap_caps_free(output);

    return ESP_OK;
}

void audio_processor_reset(void)
{
    // Overlap-save carries no state between audio_processor_apply() calls —
    // each call processes its full buffer internally. Nothing to clear.
}

void audio_processor_free(void)
{
    if (s_fft_data) { heap_caps_free(s_fft_data); s_fft_data = NULL; }
    if (s_mask)     { heap_caps_free(s_mask);     s_mask     = NULL; }
    // We deliberately keep the esp-dsp FFT twiddle tables allocated across
    // free/re-init cycles; dsps_fft2r_deinit_fc32() would free them globally.
    s_bank = NULL;
}
