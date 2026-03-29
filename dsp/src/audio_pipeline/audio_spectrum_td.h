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
    static constexpr size_t FILTER_STAGES = 3;

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

    bool computeBandpassCoefficients(double frequency,
                                     double sampleRate,
                                     double bandwidthOctaves,
                                     Coefficients &out) const;
    bool computeBandpassCoefficientsQ(double frequency,
                                      double sampleRate,
                                      double q,
                                      Coefficients &out) const;
    double responseMagnitudeAt(const Coefficients &c,
                               double frequency,
                               double sampleRate) const;
    bool buildDecorrelationMatrix(double sampleRate);

    static float sanitizeLevel(float value);

    size_t binCount = 0;
    Spacing spacingMode = Spacing::Logarithmic;
    Detector detectorMode = Detector::RMS;
    double analyzerBandwidthScale = 1.0;
    float gainNormalization = 1.0f;
    float smoothing = 0.65f;
    bool isEnabled = false;

    float centerFrequencies[MAX_BINS] = {};
    Coefficients coeff[MAX_BINS] = {};
    float binGainComp[MAX_BINS] = {};
    float decorrelation[MAX_BINS][MAX_BINS] = {};
    bool decorrelationReady = false;
    sample_t state[AUDIO_CHANNELS][MAX_BINS][FILTER_STAGES][2] = {};
    float binsLeft[MAX_BINS] = {};
    float binsRight[MAX_BINS] = {};
};
