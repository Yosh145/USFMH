#pragma once

#include <stdint.h>
#include <stdbool.h>

#define NUM_TEST_FREQS   27
#define NUM_BAND_EDGES   28

// Test frequencies (Hz) — logarithmically spaced from 10 Hz to 14263 Hz
extern const float test_frequencies[NUM_TEST_FREQS];

// Band edge frequencies (Hz) — boundaries between adjacent test bands
// Band i spans [band_edges[i], band_edges[i+1])
extern const float band_edges[NUM_BAND_EDGES];

// Result of the hearing test: per-band gain in dB
typedef struct {
    float gain_db[NUM_TEST_FREQS];   // dB boost for each band (0 = heard, >0 = boost)
    bool  heard[NUM_TEST_FREQS];     // whether the user heard each frequency
    int   num_tested;                // how many frequencies were actually tested
} hearing_result_t;

/**
 * Run the hearing test over serial.
 *
 * Protocol: For each test frequency, sends "TONE <freq_hz> <duration_ms>\n"
 * over stdout, then reads a line from stdin expecting "y" or "n".
 *
 * Uses binary search to find the upper and lower hearing limits efficiently,
 * then marks all frequencies within the range as heard and outside as not heard.
 *
 * @param result  Output: gains and heard flags for all 27 bands
 */
void hearing_test_run(hearing_result_t *result);
