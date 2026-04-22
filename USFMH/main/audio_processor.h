#pragma once

#include "hearing_test.h"
#include "coeff_loader.h"
#include "esp_err.h"

/**
 * Initialize the audio processor with a loaded filter bank and hearing test
 * result.  History buffers are allocated in PSRAM so the tap count can be
 * arbitrarily large (limited only by available PSRAM).
 *
 * @param bank        Filter bank loaded from SD card via coeff_loader
 * @param result      Hearing test result containing per-band gain in dB
 * @param sample_rate Sample rate of the audio to be processed (Hz)
 */
esp_err_t audio_processor_init(const filter_bank_t *bank,
                               const hearing_result_t *result,
                               uint32_t sample_rate);

/**
 * Process audio samples through the FIR filter bank in-place.
 *
 * @param samples     Audio buffer (float, normalized -1.0 to 1.0). Modified in-place.
 * @param num_samples Number of samples in the buffer
 */
esp_err_t audio_processor_apply(float *samples, uint32_t num_samples);

/**
 * Reset filter delay lines (call when starting a new audio file).
 */
void audio_processor_reset(void);

/**
 * Free all PSRAM buffers allocated by audio_processor_init().
 */
void audio_processor_free(void);
