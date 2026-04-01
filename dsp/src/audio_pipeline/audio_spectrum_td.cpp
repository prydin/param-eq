// Copyright (c) 2026 Pontus Rydin
// SPDX-License-Identifier: MIT
#include "audio_spectrum_td.h"


#include <algorithm>
#include <cmath>
#include <string.h>
#include "../../common/constants.h"

namespace
{
constexpr double kMinLogFrequency = 1.0;
constexpr double kDefaultMinFrequency = 20.0;
constexpr double kDefaultMaxFrequency = 20000.0;
constexpr double kDefaultGainDb = 12.0;
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

    binCount = bins;
    spacingMode = spacing;
    detectorMode = detector;

    memset(lpState, 0, sizeof(lpState));
    memset(hpState, 0, sizeof(hpState));
    memset(lpCoeffCmsis, 0, sizeof(lpCoeffCmsis));
    memset(hpCoeffCmsis, 0, sizeof(hpCoeffCmsis));

    const double logMin = log10(std::max(minFrequency, kMinLogFrequency));
    const double logMax = log10(std::max(maxFrequency, kMinLogFrequency));

    for (size_t i = 0; i < binCount; ++i)
    {
        double t = (binCount > 1) ? static_cast<double>(i) / static_cast<double>(binCount - 1) : 0.0;
        double center = (spacingMode == Spacing::Logarithmic)
            ? pow(10.0, logMin + (logMax - logMin) * t)
            : minFrequency + (maxFrequency - minFrequency) * t;
        centerFrequencies[i] = static_cast<float>(center);
        binsLeft[i] = binsRight[i] = 0.0f;
    }

    for (size_t i = 0; i < binCount; ++i)
    {
        hasLP[i] = (i < binCount - 1);
        hasHP[i] = (i > 0);

        if (hasLP[i])
        {
            Coefficients lp[XOVER_STAGES];
            double fEdge = (spacingMode == Spacing::Logarithmic)
                ? sqrt((double)centerFrequencies[i] * (double)centerFrequencies[i + 1])
                : 0.5 * ((double)centerFrequencies[i] + (double)centerFrequencies[i + 1]);
            fEdge = std::min(fEdge, nyquist * 0.98);
            fEdge = std::max(fEdge, 1.0);
            if (!computeXoverLP(fEdge, sampleRate, lp))
                return false;

            for (int s = 0; s < XOVER_STAGES; ++s)
            {
                const int base = s * 5;
                lpCoeffCmsis[i][base + 0] = lp[s].b0;
                lpCoeffCmsis[i][base + 1] = lp[s].b1;
                lpCoeffCmsis[i][base + 2] = lp[s].b2;
                lpCoeffCmsis[i][base + 3] = -lp[s].a1;
                lpCoeffCmsis[i][base + 4] = -lp[s].a2;
            }
        }

        if (hasHP[i])
        {
            Coefficients hp[XOVER_STAGES];
            double fEdge = (spacingMode == Spacing::Logarithmic)
                ? sqrt((double)centerFrequencies[i - 1] * (double)centerFrequencies[i])
                : 0.5 * ((double)centerFrequencies[i - 1] + (double)centerFrequencies[i]);
            fEdge = std::min(fEdge, nyquist * 0.98);
            fEdge = std::max(fEdge, 1.0);
            if (!computeXoverHP(fEdge, sampleRate, hp))
                return false;

            for (int s = 0; s < XOVER_STAGES; ++s)
            {
                const int base = s * 5;
                hpCoeffCmsis[i][base + 0] = hp[s].b0;
                hpCoeffCmsis[i][base + 1] = hp[s].b1;
                hpCoeffCmsis[i][base + 2] = hp[s].b2;
                hpCoeffCmsis[i][base + 3] = -hp[s].a1;
                hpCoeffCmsis[i][base + 4] = -hp[s].a2;
            }
        }

        for (int ch = 0; ch < AUDIO_CHANNELS; ++ch)
        {
            if (hasLP[i])
            {
                arm_biquad_cascade_df2T_init_f32(&lpInst[ch][i],
                                                 XOVER_STAGES,
                                                 lpCoeffCmsis[i],
                                                 lpState[ch][i]);
            }
            if (hasHP[i])
            {
                arm_biquad_cascade_df2T_init_f32(&hpInst[ch][i],
                                                 XOVER_STAGES,
                                                 hpCoeffCmsis[i],
                                                 hpState[ch][i]);
            }
        }
    }

    return true;
}

bool AudioSpectrumTD::computeBiquadBW(double cutoffHz, double sampleRate,
                                      double q, bool isLP, Coefficients &out)
{
    if (cutoffHz <= 0.0 || cutoffHz >= sampleRate * 0.5 || sampleRate <= 0.0)
        return false;
    const double K = tan(M_PI * cutoffHz / sampleRate);
    const double K2 = K * K;
    const double D = K2 + K / q + 1.0;
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

    // Build mono analysis input once per block to cut analyzer work roughly in half.
    for (size_t i = 0; i < AUDIO_BLOCK_SAMPLES; ++i)
    {
        workA[1][i] = 0.5f * (block->data[0][i] + block->data[1][i]);
    }

    for (size_t band = 0; band < binCount; ++band)
    {
        float monoPeak = 0.0f;
        double monoEnergy = 0.0;

        const bool applyHP = hasHP[band];
        const bool applyLP = hasLP[band];

        sample_t *bandOut = nullptr;

        memcpy(workA[0], workA[1], AUDIO_BLOCK_SAMPLES * sizeof(sample_t));
        sample_t *cur = workA[0];
        sample_t *nxt = workB[0];
        if (applyHP)
        {
            arm_biquad_cascade_df2T_f32(&hpInst[0][band], cur, nxt, AUDIO_BLOCK_SAMPLES);
            sample_t *tmp = cur;
            cur = nxt;
            nxt = tmp;
        }
        if (applyLP)
        {
            arm_biquad_cascade_df2T_f32(&lpInst[0][band], cur, nxt, AUDIO_BLOCK_SAMPLES);
            sample_t *tmp = cur;
            cur = nxt;
            nxt = tmp;
        }
        bandOut = cur;

        for (size_t i = 0; i < AUDIO_BLOCK_SAMPLES; ++i)
        {
            const float x = bandOut[i];
            const float absX = fabsf(x);
            monoPeak = std::max(monoPeak, absX);
            monoEnergy += static_cast<double>(x) * static_cast<double>(x);
        }

        float monoValue = 0.0f;
        if (detectorMode == Detector::Peak)
        {
            monoValue = monoPeak;
        }
        else
        {
            monoValue = sqrtf(static_cast<float>(monoEnergy / AUDIO_BLOCK_SAMPLES));
        }

        monoValue = sanitizeLevel(monoValue * gainNormalization);
        binsLeft[band] = binsLeft[band] * smoothing + monoValue * (1.0f - smoothing);
        binsRight[band] = binsRight[band] * smoothing + monoValue * (1.0f - smoothing);
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
    if (left != nullptr)
        memcpy(left, binsLeft, n * sizeof(float));
    if (right != nullptr)
        memcpy(right, binsRight, n * sizeof(float));
    interrupts();
    if (left != nullptr)
        for (size_t i = n; i < count; ++i)
            left[i] = 0.0f;
    if (right != nullptr)
        for (size_t i = n; i < count; ++i)
            right[i] = 0.0f;
}