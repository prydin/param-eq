#ifndef FILTER_H_
#define FILTER_H_
#include <Arduino.h>

#define FILTER_BANDS 3

// Band selection
#define LOW_BAND 0
#define MID_BAND 1
#define HIGH_BAND 2

// Filter type selection
#define LOWSHELF 0
#define HIGHSHELF 1
#define PEAKINGEQ 2
#define BYPASS 3
#define NUM_FILTER_TYPES 4


// Struct represeting filter settings
struct FilterSettings {
    uint8_t type;     // Filter type (e.g., low-pass, high-pass, etc.)
    float frequency;  // Center or cutoff frequency
    float Q;          // Quality factor
    float gain;       // Gain in dB
};
#endif // FILTER_H_