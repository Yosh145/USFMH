#include "hearing_test.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "hearing_test";

// From data.md — 27 logarithmically spaced test frequencies
const float test_frequencies[NUM_TEST_FREQS] = {
    10, 13, 17, 23, 30, 40, 52, 69, 91, 120,
    159, 211, 280, 371, 491, 651, 862, 1142, 1512, 2001,
    2649, 3507, 4644, 6149, 8141, 10776, 14263
};

// From data.md — 28 band edge frequencies (boundaries between test bands)
const float band_edges[NUM_BAND_EDGES] = {
    11, 15, 20, 26, 35, 46, 60, 80, 105, 139,
    185, 245, 325, 431, 571, 756, 1002, 1327, 1756, 2325,
    3078, 4075, 5396, 7145, 9458, 12519, 16572, 19440
};

// Duration of each test tone in ms
#define TONE_DURATION_MS 2000

/**
 * Send a tone command and read the user's response.
 * Returns true if the user heard the tone.
 */
static bool test_frequency(float freq_hz)
{
    // Send tone command to host
    printf("TONE %.0f %d\n", freq_hz, TONE_DURATION_MS);
    fflush(stdout);

    // Read response character by character (non-blocking to avoid WDT)
    char line[16];
    int pos = 0;
    while (1) {
        int c = fgetc(stdin);
        if (c != EOF) {
            if (c == '\n' || c == '\r') {
                line[pos] = '\0';
                break;
            } else if (pos < (int)sizeof(line) - 1) {
                line[pos++] = (char)c;
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    if (pos == 0) {
        ESP_LOGW(TAG, "No response received, assuming not heard");
        return false;
    }

    // Trim whitespace
    char *p = line;
    while (*p == ' ' || *p == '\t') p++;

    return (p[0] == 'y' || p[0] == 'Y');
}

/**
 * Binary search for the upper hearing limit.
 * Returns the highest index in test_frequencies[] that the user can hear,
 * searching in the range [lo, hi].
 * Returns lo-1 if even lo is not heard.
 */
static int find_upper_limit(int lo, int hi)
{
    int best = lo - 1;
    while (lo <= hi) {
        int mid = (lo + hi) / 2;
        ESP_LOGI(TAG, "Testing upper limit: %.0f Hz (index %d)", test_frequencies[mid], mid);
        if (test_frequency(test_frequencies[mid])) {
            best = mid;
            lo = mid + 1;  // Can hear this — try higher
        } else {
            hi = mid - 1;  // Can't hear — try lower
        }
    }
    return best;
}

/**
 * Binary search for the lower hearing limit.
 * Returns the lowest index in test_frequencies[] that the user can hear,
 * searching in the range [lo, hi].
 * Returns hi+1 if even hi is not heard.
 */
static int find_lower_limit(int lo, int hi)
{
    int best = hi + 1;
    while (lo <= hi) {
        int mid = (lo + hi) / 2;
        ESP_LOGI(TAG, "Testing lower limit: %.0f Hz (index %d)", test_frequencies[mid], mid);
        if (test_frequency(test_frequencies[mid])) {
            best = mid;
            hi = mid - 1;  // Can hear — try lower
        } else {
            lo = mid + 1;  // Can't hear — try higher
        }
    }
    return best;
}

void hearing_test_run(hearing_result_t *result)
{
    memset(result, 0, sizeof(*result));

    ESP_LOGI(TAG, "=== Hearing Test Starting ===");
    ESP_LOGI(TAG, "Testing %d frequency bands via binary search", NUM_TEST_FREQS);

    // Start with a mid-range frequency that most people can hear (1142 Hz, index 17)
    int mid_idx = NUM_TEST_FREQS / 2;  // index 13 → 371 Hz
    ESP_LOGI(TAG, "Initial test at %.0f Hz", test_frequencies[mid_idx]);

    if (!test_frequency(test_frequencies[mid_idx])) {
        // Can't hear mid-range — try a very common frequency (1000 Hz range)
        mid_idx = 17;  // 1142 Hz
        ESP_LOGI(TAG, "Retrying at %.0f Hz", test_frequencies[mid_idx]);
        if (!test_frequency(test_frequencies[mid_idx])) {
            // User can't hear common frequencies — no filters active
            ESP_LOGW(TAG, "Cannot hear mid-range frequencies — no bands will be activated");
            for (int i = 0; i < NUM_TEST_FREQS; i++) {
                result->heard[i] = false;
                result->gain_db[i] = 0.0f;
            }
            result->num_tested = 2;
            return;
        }
    }

    // Found a frequency they can hear at mid_idx
    result->heard[mid_idx] = true;
    result->num_tested = 1;

    // Binary search upward: find highest audible frequency
    int upper = find_upper_limit(mid_idx + 1, NUM_TEST_FREQS - 1);
    if (upper < mid_idx) upper = mid_idx;

    // Binary search downward: find lowest audible frequency
    int lower = find_lower_limit(0, mid_idx - 1);
    if (lower > mid_idx) lower = mid_idx;

    ESP_LOGI(TAG, "Hearing range: %.0f Hz (idx %d) to %.0f Hz (idx %d)",
             test_frequencies[lower], lower, test_frequencies[upper], upper);

    // Fill in results: heard within [lower, upper] → filter activated; outside → inactive
    for (int i = 0; i < NUM_TEST_FREQS; i++) {
        if (i >= lower && i <= upper) {
            result->heard[i] = true;
            result->gain_db[i] = HEARING_TEST_BOOST_DB;
        } else {
            result->heard[i] = false;
            result->gain_db[i] = 0.0f;
        }
    }

    // Count tested frequencies (binary search steps)
    // upper search: ~log2(NUM_TEST_FREQS/2) ≈ 4 steps
    // lower search: ~log2(NUM_TEST_FREQS/2) ≈ 4 steps
    // + 1 or 2 initial tests
    result->num_tested = 2 + (int)(2 * 3.8);  // approximate

    ESP_LOGI(TAG, "=== Hearing Test Complete ===");
    ESP_LOGI(TAG, "Activated bands (heard frequencies):");
    for (int i = 0; i < NUM_TEST_FREQS; i++) {
        if (result->heard[i]) {
            ESP_LOGI(TAG, "  %.0f Hz: +%.1f dB", test_frequencies[i], result->gain_db[i]);
        }
    }
}
