// Copyright (c) 2026 Pontus Rydin
// SPDX-License-Identifier: MIT
#include "audio_spectrum_td.h"

#include <algorithm>
#include <cmath>
#include <string.h>

namespace
{
constexpr double kMinLogFrequency = 1.0;
constexpr double kDefaultMinFrequency = 20.0;
constexpr double kDefaultMaxFrequency = 20000.0;
constexpr double kDefaultQ = 2.0;
constexpr double kDefaultGainDb = 12.0;
constexpr float kEpsilon = 1e-12f;
}

AudioSpectrumTD::AudioSpectrumTD() : AudioComponent()
{
    configure(32,
              48000.0,
              kDefaultMinFrequency,
              kDefaultMaxFrequency,
              Spacing::Logarithmic,
              Detector::RMS,
              kDefaultQ,
              kDefaultGainDb);
}

bool AudioSpectrumTD::configure(size_t bins,
                                double sampleRate,
                                double minFrequency,
                                double maxFrequency,
                                Spacing spacing,
                                Detector detector,
                                double q,
                                double gainDb)
{
    if (bins == 0)
    {
        return false;
    }
    if (bins > MAX_BINS)
    {
        bins = MAX_BINS;
    }
    if (!std::isfinite(sampleRate) || sampleRate <= 1000.0)
    {
        return false;
    }
    if (!std::isfinite(q) || q <= 0.0)
    {
        q = kDefaultQ;
    }
    (void)gainDb;

    const double nyquist = sampleRate * 0.5;
    if (!std::isfinite(minFrequency))
    {
        minFrequency = kDefaultMinFrequency;
    }
    if (!std::isfinite(maxFrequency))
    {
        maxFrequency = kDefaultMaxFrequency;
    }

    minFrequency = std::max(minFrequency, spacing == Spacing::Logarithmic ? kMinLogFrequency : 0.0);
    maxFrequency = std::min(maxFrequency, nyquist * 0.98);
    if (maxFrequency <= minFrequency)
    {
        maxFrequency = std::min(nyquist * 0.98, std::max(minFrequency + 1.0, 1000.0));
    }

    binCount = bins;
    spacingMode = spacing;
    detectorMode = detector;
    gainNormalization = 1.0f;

    memset(state, 0, sizeof(state));

    for (size_t i = 0; i < binCount; ++i)
    {
        double t = (binCount > 1) ? static_cast<double>(i) / static_cast<double>(binCount - 1) : 0.0;
        double center = 0.0;
        if (spacingMode == Spacing::Linear)
        {
            center = minFrequency + (maxFrequency - minFrequency) * t;
        }
        else
        {
            const double logMin = log10(std::max(minFrequency, kMinLogFrequency));
            const double logMax = log10(std::max(maxFrequency, kMinLogFrequency));
            center = pow(10.0, logMin + (logMax - logMin) * t);
        }

        Coefficients c;
        if (!computeBandpassCoefficients(center, sampleRate, q, c))
        {
            return false;
        }

        centerFrequencies[i] = static_cast<float>(center);
        coeff[i] = c;
        binsLeft[i] = 0.0f;
        binsRight[i] = 0.0f;
    }

    return true;
}

bool AudioSpectrumTD::computeBandpassCoefficients(double frequency,
                                                  double sampleRate,
                                                  double q,
                                                  Coefficients &out) const
{
    if (!std::isfinite(frequency) || !std::isfinite(sampleRate) || frequency <= 0.0 || sampleRate <= 0.0)
    {
        return false;
    }

    const double w0 = frequency * (TWO_PI / sampleRate);
    const double sinW0 = sin(w0);
    const double alpha = sinW0 / (q * 2.0);
    const double cosW0 = cos(w0);
    const double scale = 1.0 / (1.0 + alpha);

    out.b0 = static_cast<sample_t>(alpha * scale);
    out.b1 = static_cast<sample_t>(0.0);
    out.b2 = static_cast<sample_t>((-alpha) * scale);
    out.a1 = static_cast<sample_t>((-2.0 * cosW0) * scale);
    out.a2 = static_cast<sample_t>((1.0 - alpha) * scale);
    return true;
}

float AudioSpectrumTD::sanitizeLevel(float value)
{
    if (!std::isfinite(value) || value < 0.0f)
    {
        return 0.0f;
    }
    return value;
}

void AudioSpectrumTD::process(AudioBuffer *block)
{
    if (block == nullptr)
    {
        return;
    }
    if (!isEnabled || binCount == 0)
    {
        transmit(block);
        return;
    }

    for (size_t band = 0; band < binCount; ++band)
    {
        float leftPeak = 0.0f;
        float rightPeak = 0.0f;
        double leftEnergy = 0.0;
        double rightEnergy = 0.0;

        sample_t z1L = state[0][band][0];
        sample_t z2L = state[0][band][1];
        sample_t z1R = state[1][band][0];
        sample_t z2R = state[1][band][1];
        const Coefficients &c = coeff[band];

        for (size_t i = 0; i < AUDIO_BLOCK_SAMPLES; ++i)
        {
            const sample_t xL = block->data[0][i];
            const sample_t yL = c.b0 * xL + z1L;
            z1L = c.b1 * xL - c.a1 * yL + z2L;
            z2L = c.b2 * xL - c.a2 * yL;

            const sample_t xR = block->data[1][i];
            const sample_t yR = c.b0 * xR + z1R;
            z1R = c.b1 * xR - c.a1 * yR + z2R;
            z2R = c.b2 * xR - c.a2 * yR;

            const float absLeft = fabsf(static_cast<float>(yL));
            const float absRight = fabsf(static_cast<float>(yR));
            leftPeak = std::max(leftPeak, absLeft);
            rightPeak = std::max(rightPeak, absRight);
            leftEnergy += static_cast<double>(yL) * static_cast<double>(yL);
            rightEnergy += static_cast<double>(yR) * static_cast<double>(yR);
        }

        state[0][band][0] = z1L;
        state[0][band][1] = z2L;
        state[1][band][0] = z1R;
        state[1][band][1] = z2R;

        float leftValue = 0.0f;
        float rightValue = 0.0f;
        if (detectorMode == Detector::Peak)
        {
            leftValue = leftPeak;
            rightValue = rightPeak;
        }
        else
        {
            leftValue = sqrtf(static_cast<float>(leftEnergy / AUDIO_BLOCK_SAMPLES));
            rightValue = sqrtf(static_cast<float>(rightEnergy / AUDIO_BLOCK_SAMPLES));
        }

        leftValue = sanitizeLevel(leftValue * gainNormalization);
        rightValue = sanitizeLevel(rightValue * gainNormalization);

        binsLeft[band] = binsLeft[band] * smoothing + leftValue * (1.0f - smoothing);
        binsRight[band] = binsRight[band] * smoothing + rightValue * (1.0f - smoothing);
    }

    transmit(block);
}

void AudioSpectrumTD::getCenterFrequencies(float *out, size_t count) const
{
    if (out == nullptr || count == 0)
    {
        return;
    }

    size_t n = std::min(count, binCount);
    noInterrupts();
    memcpy(out, centerFrequencies, n * sizeof(float));
    interrupts();

    for (size_t i = n; i < count; ++i)
    {
        out[i] = 0.0f;
    }
}

void AudioSpectrumTD::getNormalizedBins(float *left, float *right, size_t count) const
{
    if (count == 0)
    {
        return;
    }

    size_t n = std::min(count, binCount);
    noInterrupts();
    if (left != nullptr)
    {
        memcpy(left, binsLeft, n * sizeof(float));
    }
    if (right != nullptr)
    {
        memcpy(right, binsRight, n * sizeof(float));
    }
    interrupts();

    if (left != nullptr)
    {
        for (size_t i = n; i < count; ++i)
        {
            left[i] = 0.0f;
        }
    }
    if (right != nullptr)
    {
        for (size_t i = n; i < count; ++i)
        {
            right[i] = 0.0f;
        }
    }
}
