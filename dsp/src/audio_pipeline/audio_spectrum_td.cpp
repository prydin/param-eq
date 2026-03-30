// Copyright (c) 2026 Pontus Rydin
// SPDX-License-Identifier: MIT
#include "audio_spectrum_td.h"

#include <algorithm>
#include <cmath>
#include <string.h>

namespace
{
constexpr double kMinLogFrequency    = 1.0;
constexpr double kDefaultMinFrequency = 20.0;
constexpr double kDefaultMaxFrequency = 20000.0;
constexpr double kDefaultGainDb      = 12.0;
}

AudioSpectrumTD::AudioSpectrumTD() : AudioComponent()
{
    configure(32,
              48000.0,
              kDefaultMinFrequency,
              kDefaultMaxFrequency,
              Spacing::Logarithmic,
              Detector::RMS,
              1.0,
              kDefaultGainDb);
}

bool AudioSpectrumTD::configure(size_t bins,
                                double sampleRate,
                                double minFrequency,
                                double maxFrequency,
                                Spacing spacing,
                                Detector detector,
                                double /*q*/,
                                double gainDb)
{
    if (bins == 0)
        return false;
    if (bins > MAX_BINS)
        bins = MAX_BINS;
    if (!std::isfinite(sampleRate) || sampleRate <= 1000.0)
        return false;

    gainNormalization = powf(10.0f, (float)gainDb / 20.0f);

    const double nyquist = sampleRate * 0.5;
    if (!std::isfinite(minFrequency))
        minFrequency = kDefaultMinFrequency;
    if (!std::isfinite(maxFrequency))
        maxFrequency = kDefaultMaxFrequency;

    minFrequency = std::max(minFrequency, spacing == Spacing::Logarithmic ? kMinLogFrequency : 0.0);
    maxFrequency = std::min(maxFrequency, nyquist * 0.98);
    if (maxFrequency <= minFrequency)
        maxFrequency = std::min(nyquist * 0.98, std::max(minFrequency + 1.0, 1000.0));

    binCount    = bins;
    spacingMode = spacing;
    detectorMode = detector;

    memset(stateLP, 0, sizeof(stateLP));
    memset(stateHP, 0, sizeof(stateHP));

    const double logMin = log10(std::max(minFrequency, kMinLogFrequency));
    const double logMax = log10(std::max(maxFrequency, kMinLogFrequency));

    // Step 1: compute center frequencies.
    for (size_t i = 0; i < binCount; ++i)
    {
        double t = (binCount > 1) ? static_cast<double>(i) / static_cast<double>(binCount - 1) : 0.0;
        double center = (spacingMode == Spacing::Logarithmic)
            ? pow(10.0, logMin + (logMax - logMin) * t)
            : minFrequency + (maxFrequency - minFrequency) * t;
        centerFrequencies[i] = static_cast<float>(center);
        binsLeft[i] = binsRight[i] = 0.0f;
    }

    // Step 2: compute LP (upper edge) and HP (lower edge) crossover biquads per bin.
    // Each edge uses a 4th-order Butterworth LP/HP pair with the same cutoff, which
    // guarantees |H_LP|^2 + |H_HP|^2 = 1 at every frequency.  The bands therefore
    // telescope to a perfect power partition of unity: sum_k |H_k(f)|^2 = 1.
    for (size_t i = 0; i < binCount; ++i)
    {
        hasLP[i] = (i < binCount - 1);
        hasHP[i] = (i > 0);

        if (hasLP[i])
        {
            double fEdge = (spacingMode == Spacing::Logarithmic)
                ? sqrt((double)centerFrequencies[i] * (double)centerFrequencies[i + 1])
                : 0.5 * ((double)centerFrequencies[i] + (double)centerFrequencies[i + 1]);
            fEdge = std::min(fEdge, nyquist * 0.98);
            fEdge = std::max(fEdge, 1.0);
            if (!computeXoverLP(fEdge, sampleRate, lpCoeff[i]))
                return false;
        }

        if (hasHP[i])
        {
            double fEdge = (spacingMode == Spacing::Logarithmic)
                ? sqrt((double)centerFrequencies[i - 1] * (double)centerFrequencies[i])
                : 0.5 * ((double)centerFrequencies[i - 1] + (double)centerFrequencies[i]);
            fEdge = std::min(fEdge, nyquist * 0.98);
            fEdge = std::max(fEdge, 1.0);
            if (!computeXoverHP(fEdge, sampleRate, hpCoeff[i]))
                return false;
        }
    }

    return true;
}

// Single 2nd-order Butterworth biquad via bilinear transform.
// K = tan(pi*fc/fs) pre-warps the cutoff.  LP and HP share identical a1/a2
// denominator coefficients; only the numerator differs.
bool AudioSpectrumTD::computeBiquadBW(double cutoffHz, double sampleRate,
                                      double q, bool isLP, Coefficients &out)
{
    if (cutoffHz <= 0.0 || cutoffHz >= sampleRate * 0.5 || sampleRate <= 0.0)
        return false;
    const double K    = tan(M_PI * cutoffHz / sampleRate);
    const double K2   = K * K;
    const double D    = K2 + K / q + 1.0;
    const double invD = 1.0 / D;
    out.a1 = static_cast<sample_t>(2.0 * (K2 - 1.0) * invD);
    out.a2 = static_cast<sample_t>((K2 - K / q + 1.0) * invD);
    if (isLP)
    {
        out.b0 = out.b2 = static_cast<sample_t>(K2 * invD);
        out.b1 = static_cast<sample_t>(2.0 * K2 * invD);
    }
    else
    {
        out.b0 = out.b2 = static_cast<sample_t>(invD);
        out.b1 = static_cast<sample_t>(-2.0 * invD);
    }
    return true;
}

bool AudioSpectrumTD::computeXoverLP(double cutoffHz, double sampleRate,
                                     Coefficients out[XOVER_STAGES]) const
{
    return computeBiquadBW(cutoffHz, sampleRate, kBWQ0, true, out[0]) &&
           computeBiquadBW(cutoffHz, sampleRate, kBWQ1, true, out[1]);
}

bool AudioSpectrumTD::computeXoverHP(double cutoffHz, double sampleRate,
                                     Coefficients out[XOVER_STAGES]) const
{
    return computeBiquadBW(cutoffHz, sampleRate, kBWQ0, false, out[0]) &&
           computeBiquadBW(cutoffHz, sampleRate, kBWQ1, false, out[1]);
}

float AudioSpectrumTD::sanitizeLevel(float value)
{
    if (!std::isfinite(value) || value < 0.0f)
        return 0.0f;
    return value;
}

void AudioSpectrumTD::process(AudioBuffer *block)
{
    if (block == nullptr)
        return;
    if (!isEnabled || binCount == 0)
    {
        transmit(block);
        return;
    }

    for (size_t band = 0; band < binCount; ++band)
    {
        float  leftPeak   = 0.0f;
        float  rightPeak  = 0.0f;
        double leftEnergy  = 0.0;
        double rightEnergy = 0.0;

        // Load filter state into locals for register-friendly access.
        sample_t z1hpL[XOVER_STAGES], z2hpL[XOVER_STAGES];
        sample_t z1hpR[XOVER_STAGES], z2hpR[XOVER_STAGES];
        sample_t z1lpL[XOVER_STAGES], z2lpL[XOVER_STAGES];
        sample_t z1lpR[XOVER_STAGES], z2lpR[XOVER_STAGES];
        for (int s = 0; s < XOVER_STAGES; ++s)
        {
            z1hpL[s] = stateHP[0][band][s][0]; z2hpL[s] = stateHP[0][band][s][1];
            z1hpR[s] = stateHP[1][band][s][0]; z2hpR[s] = stateHP[1][band][s][1];
            z1lpL[s] = stateLP[0][band][s][0]; z2lpL[s] = stateLP[0][band][s][1];
            z1lpR[s] = stateLP[1][band][s][0]; z2lpR[s] = stateLP[1][band][s][1];
        }

        const bool applyHP = hasHP[band];
        const bool applyLP = hasLP[band];

        for (size_t i = 0; i < AUDIO_BLOCK_SAMPLES; ++i)
        {
            sample_t xL = block->data[0][i];
            sample_t xR = block->data[1][i];

            // HP chain (lower edge): reject frequencies below the bin boundary.
            if (applyHP)
            {
                for (int s = 0; s < XOVER_STAGES; ++s)
                {
                    const Coefficients &c = hpCoeff[band][s];
                    sample_t yL  = c.b0 * xL + z1hpL[s];
                    z1hpL[s] = c.b1 * xL - c.a1 * yL + z2hpL[s];
                    z2hpL[s] = c.b2 * xL - c.a2 * yL;
                    xL = yL;
                    sample_t yR  = c.b0 * xR + z1hpR[s];
                    z1hpR[s] = c.b1 * xR - c.a1 * yR + z2hpR[s];
                    z2hpR[s] = c.b2 * xR - c.a2 * yR;
                    xR = yR;
                }
            }

            // LP chain (upper edge): reject frequencies above the bin boundary.
            if (applyLP)
            {
                for (int s = 0; s < XOVER_STAGES; ++s)
                {
                    const Coefficients &c = lpCoeff[band][s];
                    sample_t yL  = c.b0 * xL + z1lpL[s];
                    z1lpL[s] = c.b1 * xL - c.a1 * yL + z2lpL[s];
                    z2lpL[s] = c.b2 * xL - c.a2 * yL;
                    xL = yL;
                    sample_t yR  = c.b0 * xR + z1lpR[s];
                    z1lpR[s] = c.b1 * xR - c.a1 * yR + z2lpR[s];
                    z2lpR[s] = c.b2 * xR - c.a2 * yR;
                    xR = yR;
                }
            }

            // xL/xR are the band-filtered samples.
            const float absL = fabsf(static_cast<float>(xL));
            const float absR = fabsf(static_cast<float>(xR));
            leftPeak   = std::max(leftPeak,  absL);
            rightPeak  = std::max(rightPeak, absR);
            leftEnergy  += static_cast<double>(xL) * static_cast<double>(xL);
            rightEnergy += static_cast<double>(xR) * static_cast<double>(xR);
        }

        // Save state back.
        for (int s = 0; s < XOVER_STAGES; ++s)
        {
            stateHP[0][band][s][0] = z1hpL[s]; stateHP[0][band][s][1] = z2hpL[s];
            stateHP[1][band][s][0] = z1hpR[s]; stateHP[1][band][s][1] = z2hpR[s];
            stateLP[0][band][s][0] = z1lpL[s]; stateLP[0][band][s][1] = z2lpL[s];
            stateLP[1][band][s][0] = z1lpR[s]; stateLP[1][band][s][1] = z2lpR[s];
        }

        float leftValue  = 0.0f;
        float rightValue = 0.0f;
        if (detectorMode == Detector::Peak)
        {
            leftValue  = leftPeak;
            rightValue = rightPeak;
        }
        else
        {
            leftValue  = sqrtf(static_cast<float>(leftEnergy  / AUDIO_BLOCK_SAMPLES));
            rightValue = sqrtf(static_cast<float>(rightEnergy / AUDIO_BLOCK_SAMPLES));
        }

        leftValue  = sanitizeLevel(leftValue  * gainNormalization);
        rightValue = sanitizeLevel(rightValue * gainNormalization);

        binsLeft[band]  = binsLeft[band]  * smoothing + leftValue  * (1.0f - smoothing);
        binsRight[band] = binsRight[band] * smoothing + rightValue * (1.0f - smoothing);
    }

    transmit(block);
}

void AudioSpectrumTD::getCenterFrequencies(float *out, size_t count) const
{
    if (out == nullptr || count == 0)
        return;
    size_t n = std::min(count, binCount);
    noInterrupts();
    memcpy(out, centerFrequencies, n * sizeof(float));
    interrupts();
    for (size_t i = n; i < count; ++i)
        out[i] = 0.0f;
}

void AudioSpectrumTD::getNormalizedBins(float *left, float *right, size_t count) const
{
    if (count == 0)
        return;
    size_t n = std::min(count, binCount);
    noInterrupts();
    if (left  != nullptr) memcpy(left,  binsLeft,  n * sizeof(float));
    if (right != nullptr) memcpy(right, binsRight, n * sizeof(float));
    interrupts();
    if (left  != nullptr) for (size_t i = n; i < count; ++i) left[i]  = 0.0f;
    if (right != nullptr) for (size_t i = n; i < count; ++i) right[i] = 0.0f;
}
