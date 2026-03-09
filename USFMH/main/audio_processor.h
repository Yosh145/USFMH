#pragma once

#include "hearing_test.h"
#include "esp_err.h"

/**
 * Initialize the audio processor with a hearing test result.
 * Computes peaking EQ biquad coefficients for each of the 27 bands.
 *
 * @param result      Hearing test result containing per-band gain in dB
 * @param sample_rate Sample rate of the audio to be processed (Hz)
 */
esp_err_t audio_processor_init(const hearing_result_t *result, uint32_t sample_rate);

/**
 * Process audio samples through the 27-band EQ filter chain.
 * Applies all active biquad filters in cascade (in-place).
 *
 * @param samples     Audio buffer (float, normalized -1.0 to 1.0). Modified in-place.
 * @param num_samples Number of samples in the buffer
 */
esp_err_t audio_processor_apply(float *samples, uint32_t num_samples);

/**
 * Reset filter delay lines (call when starting a new audio file).
 */
void audio_processor_reset(void);
