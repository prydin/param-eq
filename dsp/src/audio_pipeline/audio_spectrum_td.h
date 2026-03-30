// Copyright (c) 2026 Pontus Rydin
// SPDX-License-Identifier: MIT
#pragma once

#include <Arduino.h>
#include <stddef.h>

#include "audio_component.h"

class AudioSpectrumTD : public AudioComponent
{
public:
    enum class Spacing : uint8_t
    {
        Linear = 0,
        Logarithmic = 1,
    };

    enum class Detector : uint8_t
    {
        Peak = 0,
        RMS = 1,
    };

    static constexpr size_t MAX_BINS = 64;

    AudioSpectrumTD();

    void process(AudioBuffer *block) override;

    bool configure(size_t bins,
                   double sampleRate,
                   double minFrequency,
                   double maxFrequency,
                   Spacing spacing,
                   Detector detector,
                   double bandwidthOctaves,
                   double gainDb);

    size_t getBinCount() const { return binCount; }
    Detector getDetector() const { return detectorMode; }
    Spacing getSpacing() const { return spacingMode; }
    void setEnabled(bool enabled) { isEnabled = enabled; }
    bool getEnabled() const { return isEnabled; }

    void getCenterFrequencies(float *out, size_t count) const;
    void getNormalizedBins(float *left, float *right, size_t count) const;

private:
    struct Coefficients
    {
        sample_t b0;
        sample_t b1;
        sample_t b2;
        sample_t a1;
        sample_t a2;
    };

    // 4th-order Butterworth crossover: 2 cascaded 2nd-order biquad stages.
    // Any nth-order Butterworth LP/HP pair satisfies |H_LP|^2 + |H_HP|^2 = 1
    // at every frequency, giving a perfect power partition of unity across bins.
    static constexpr int XOVER_STAGES = 2;
    // Q factors for the two sections of a 4th-order Butterworth:
    //   Q0 = 1 / (2*sin(3pi/8)) ~= 0.5412  (low-Q section)
    //   Q1 = 1 / (2*sin(pi/8))  ~= 1.3066  (high-Q section)
    static constexpr double kBWQ0 = 0.5412;
    static constexpr double kBWQ1 = 1.3066;

    // Single 2nd-order Butterworth biquad (LP or HP) via bilinear transform.
    static bool computeBiquadBW(double cutoffHz, double sampleRate, double q,
                                bool isLP, Coefficients &out);

    // 4th-order Butterworth LP/HP: fills out[XOVER_STAGES].
    bool computeXoverLP(double cutoffHz, double sampleRate,
                        Coefficients out[XOVER_STAGES]) const;
    bool computeXoverHP(double cutoffHz, double sampleRate,
                        Coefficients out[XOVER_STAGES]) const;

    static float sanitizeLevel(float value);

    size_t binCount = 0;
    Spacing spacingMode = Spacing::Logarithmic;
    Detector detectorMode = Detector::RMS;
    float gainNormalization = 1.0f;
    float smoothing = 0.65f;
    bool isEnabled = false;

    float centerFrequencies[MAX_BINS] = {};

    // LP (upper edge) and HP (lower edge) coefficients per bin, two stages each.
    Coefficients lpCoeff[MAX_BINS][XOVER_STAGES] = {};
    Coefficients hpCoeff[MAX_BINS][XOVER_STAGES] = {};
    bool hasLP[MAX_BINS] = {};   // all bins except the last
    bool hasHP[MAX_BINS] = {};   // all bins except the first

    // Filter state: [channel][bin][stage][z1/z2]
    sample_t stateLP[AUDIO_CHANNELS][MAX_BINS][XOVER_STAGES][2] = {};
    sample_t stateHP[AUDIO_CHANNELS][MAX_BINS][XOVER_STAGES][2] = {};

    float binsLeft[MAX_BINS] = {};
    float binsRight[MAX_BINS] = {};
};
