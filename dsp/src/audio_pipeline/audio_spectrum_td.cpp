// Copyright (c) 2026 Pontus Rydin
// SPDX-License-Identifier: MIT
#include "audio_spectrum_td.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <string.h>

namespace
{
constexpr double kMinLogFrequency = 1.0;
constexpr double kDefaultMinFrequency = 20.0;
constexpr double kDefaultMaxFrequency = 20000.0;
constexpr double kDefaultBandwidthScale = 1.0;
constexpr double kDefaultGainDb = 12.0;
constexpr double kLn2 = 0.69314718055994530942;
constexpr float kMaxBinGainComp = 24.0f;
constexpr float kEpsilon = 1e-12f;
constexpr double kDecorrelationLambda = 0.08;

bool invertMatrix(const double in[AudioSpectrumTD::MAX_BINS][AudioSpectrumTD::MAX_BINS],
                  double out[AudioSpectrumTD::MAX_BINS][AudioSpectrumTD::MAX_BINS],
                  size_t n)
{
    double aug[AudioSpectrumTD::MAX_BINS][AudioSpectrumTD::MAX_BINS * 2] = {};

    for (size_t r = 0; r < n; ++r)
    {
        for (size_t c = 0; c < n; ++c)
        {
            aug[r][c] = in[r][c];
            aug[r][n + c] = (r == c) ? 1.0 : 0.0;
        }
    }

    for (size_t i = 0; i < n; ++i)
    {
        size_t pivotRow = i;
        double maxAbs = fabs(aug[i][i]);
        for (size_t r = i + 1; r < n; ++r)
        {
            const double v = fabs(aug[r][i]);
            if (v > maxAbs)
            {
                maxAbs = v;
                pivotRow = r;
            }
        }
        if (maxAbs < 1e-14)
        {
            return false;
        }

        if (pivotRow != i)
        {
            for (size_t c = 0; c < 2 * n; ++c)
            {
                std::swap(aug[i][c], aug[pivotRow][c]);
            }
        }

        const double pivot = aug[i][i];
        for (size_t c = 0; c < 2 * n; ++c)
        {
            aug[i][c] /= pivot;
        }

        for (size_t r = 0; r < n; ++r)
        {
            if (r == i)
            {
                continue;
            }
            const double factor = aug[r][i];
            if (fabs(factor) < 1e-20)
            {
                continue;
            }
            for (size_t c = 0; c < 2 * n; ++c)
            {
                aug[r][c] -= factor * aug[i][c];
            }
        }
    }

    for (size_t r = 0; r < n; ++r)
    {
        for (size_t c = 0; c < n; ++c)
        {
            out[r][c] = aug[r][n + c];
        }
    }

    return true;
}
}

AudioSpectrumTD::AudioSpectrumTD() : AudioComponent()
{
    configure(32,
              48000.0,
              kDefaultMinFrequency,
              kDefaultMaxFrequency,
              Spacing::Logarithmic,
              Detector::RMS,
              kDefaultBandwidthScale,
              kDefaultGainDb);
}

bool AudioSpectrumTD::configure(size_t bins,
                                double sampleRate,
                                double minFrequency,
                                double maxFrequency,
                                Spacing spacing,
                                Detector detector,
                                double bandwidthScale,
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
    if (!std::isfinite(bandwidthScale) || bandwidthScale <= 0.0)
    {
        bandwidthScale = kDefaultBandwidthScale;
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
    analyzerBandwidthScale = bandwidthScale;
    gainNormalization = 1.0f;

    memset(state, 0, sizeof(state));

    double centers[MAX_BINS] = {};

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

        centers[i] = center;
        centerFrequencies[i] = static_cast<float>(center);
        binsLeft[i] = 0.0f;
        binsRight[i] = 0.0f;
    }

    for (size_t i = 0; i < binCount; ++i)
    {
        const double center = centers[i];
        Coefficients c;
        bool ok = false;

        if (spacingMode == Spacing::Logarithmic)
        {
            double lowerEdge = minFrequency;
            double upperEdge = maxFrequency;

            if (binCount > 1)
            {
                const double prevCenter = (i > 0)
                                              ? centers[i - 1]
                                              : (centers[i] * centers[i]) / centers[i + 1];
                const double nextCenter = (i + 1 < binCount)
                                              ? centers[i + 1]
                                              : (centers[i] * centers[i]) / centers[i - 1];
                lowerEdge = sqrt(std::max(kMinLogFrequency, prevCenter * center));
                upperEdge = sqrt(std::max(kMinLogFrequency, center * nextCenter));
            }

            lowerEdge = std::max(lowerEdge, minFrequency);
            upperEdge = std::min(upperEdge, maxFrequency);
            if (upperEdge <= lowerEdge)
            {
                upperEdge = std::min(maxFrequency, lowerEdge * 1.01);
            }

            double bandwidthOctaves = log2(std::max(upperEdge, kMinLogFrequency) / std::max(lowerEdge, kMinLogFrequency));
            bandwidthOctaves /= bandwidthScale;
            ok = computeBandpassCoefficients(center, sampleRate, bandwidthOctaves, c);
        }
        else
        {
            double lowerEdge = minFrequency;
            double upperEdge = maxFrequency;

            if (binCount > 1)
            {
                const double prevCenter = (i > 0) ? centers[i - 1] : (2.0 * centers[i] - centers[i + 1]);
                const double nextCenter = (i + 1 < binCount) ? centers[i + 1] : (2.0 * centers[i] - centers[i - 1]);
                lowerEdge = 0.5 * (prevCenter + center);
                upperEdge = 0.5 * (center + nextCenter);
            }

            lowerEdge = std::max(lowerEdge, 0.0);
            upperEdge = std::min(upperEdge, maxFrequency);
            const double bandwidthHz = std::max(upperEdge - lowerEdge, 1.0);
            const double q = (center / bandwidthHz) * bandwidthScale;
            ok = computeBandpassCoefficientsQ(center, sampleRate, q, c);
        }

        if (!ok)
        {
            return false;
        }

        coeff[i] = c;

        // Compensate for per-band peak gain differences so low-frequency bins
        // don't appear artificially attenuated.
        const double w0 = center * (TWO_PI / sampleRate);
        const std::complex<double> e1 = std::polar(1.0, -w0);
        const std::complex<double> e2 = std::polar(1.0, -2.0 * w0);
        const std::complex<double> num = static_cast<double>(c.b0) +
                                         static_cast<double>(c.b1) * e1 +
                                         static_cast<double>(c.b2) * e2;
        const std::complex<double> den = 1.0 +
                                         static_cast<double>(c.a1) * e1 +
                                         static_cast<double>(c.a2) * e2;
        double stageGain = std::abs(num / den);
        if (!std::isfinite(stageGain) || stageGain < 1e-9)
        {
            stageGain = 1.0;
        }
        const double totalGain = pow(stageGain, static_cast<double>(FILTER_STAGES));
        float comp = static_cast<float>(1.0 / std::max(totalGain, 1e-9));
        if (!std::isfinite(comp))
        {
            comp = 1.0f;
        }
        binGainComp[i] = std::min(comp, kMaxBinGainComp);
    }

    decorrelationReady = buildDecorrelationMatrix(sampleRate);

    return true;
}

bool AudioSpectrumTD::computeBandpassCoefficients(double frequency,
                                                  double sampleRate,
                                                  double bandwidthOctaves,
                                                  Coefficients &out) const
{
    if (!std::isfinite(frequency) || !std::isfinite(sampleRate) || frequency <= 0.0 || sampleRate <= 0.0)
    {
        return false;
    }

    if (!std::isfinite(bandwidthOctaves) || bandwidthOctaves <= 0.0)
    {
        return false;
    }

    const double w0 = frequency * (TWO_PI / sampleRate);
    const double sinW0 = sin(w0);
    if (fabs(sinW0) < 1e-12)
    {
        return false;
    }
    const double alpha = sinW0 * sinh((kLn2 * 0.5) * bandwidthOctaves * (w0 / sinW0));
    const double cosW0 = cos(w0);
    const double scale = 1.0 / (1.0 + alpha);

    out.b0 = static_cast<sample_t>(alpha * scale);
    out.b1 = static_cast<sample_t>(0.0);
    out.b2 = static_cast<sample_t>((-alpha) * scale);
    out.a1 = static_cast<sample_t>((-2.0 * cosW0) * scale);
    out.a2 = static_cast<sample_t>((1.0 - alpha) * scale);
    return true;
}

bool AudioSpectrumTD::computeBandpassCoefficientsQ(double frequency,
                                                   double sampleRate,
                                                   double q,
                                                   Coefficients &out) const
{
    if (!std::isfinite(frequency) || !std::isfinite(sampleRate) || frequency <= 0.0 || sampleRate <= 0.0)
    {
        return false;
    }
    if (!std::isfinite(q) || q <= 0.0)
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

double AudioSpectrumTD::responseMagnitudeAt(const Coefficients &c,
                                           double frequency,
                                           double sampleRate) const
{
    if (!std::isfinite(frequency) || !std::isfinite(sampleRate) || frequency <= 0.0 || sampleRate <= 0.0)
    {
        return 0.0;
    }

    const double w = frequency * (TWO_PI / sampleRate);
    const std::complex<double> e1 = std::polar(1.0, -w);
    const std::complex<double> e2 = std::polar(1.0, -2.0 * w);
    const std::complex<double> num = static_cast<double>(c.b0) +
                                     static_cast<double>(c.b1) * e1 +
                                     static_cast<double>(c.b2) * e2;
    const std::complex<double> den = 1.0 +
                                     static_cast<double>(c.a1) * e1 +
                                     static_cast<double>(c.a2) * e2;
    const double g = std::abs(num / den);
    return std::isfinite(g) ? g : 0.0;
}

bool AudioSpectrumTD::buildDecorrelationMatrix(double sampleRate)
{
    if (binCount == 0)
    {
        return false;
    }

    double A[MAX_BINS][MAX_BINS] = {};
    double AtA[MAX_BINS][MAX_BINS] = {};
    double AtAInv[MAX_BINS][MAX_BINS] = {};
    double At[MAX_BINS][MAX_BINS] = {};

    for (size_t i = 0; i < binCount; ++i)
    {
        for (size_t j = 0; j < binCount; ++j)
        {
            double g = responseMagnitudeAt(coeff[i], centerFrequencies[j], sampleRate);
            g = pow(g, static_cast<double>(FILTER_STAGES));
            g *= (binGainComp[i] > 0.0f ? binGainComp[i] : 1.0f);
            A[i][j] = g;
            At[j][i] = g;
        }
    }

    for (size_t r = 0; r < binCount; ++r)
    {
        for (size_t c = 0; c < binCount; ++c)
        {
            double sum = 0.0;
            for (size_t k = 0; k < binCount; ++k)
            {
                sum += At[r][k] * A[k][c];
            }
            if (r == c)
            {
                sum += kDecorrelationLambda;
            }
            AtA[r][c] = sum;
        }
    }

    if (!invertMatrix(AtA, AtAInv, binCount))
    {
        return false;
    }

    for (size_t r = 0; r < binCount; ++r)
    {
        for (size_t c = 0; c < binCount; ++c)
        {
            double sum = 0.0;
            for (size_t k = 0; k < binCount; ++k)
            {
                sum += AtAInv[r][k] * At[k][c];
            }
            decorrelation[r][c] = static_cast<float>(sum);
        }
    }

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

    float rawLeft[MAX_BINS] = {};
    float rawRight[MAX_BINS] = {};
    float correctedLeft[MAX_BINS] = {};
    float correctedRight[MAX_BINS] = {};

    for (size_t band = 0; band < binCount; ++band)
    {
        float leftPeak = 0.0f;
        float rightPeak = 0.0f;
        double leftEnergy = 0.0;
        double rightEnergy = 0.0;
        const Coefficients &c = coeff[band];

        for (size_t i = 0; i < AUDIO_BLOCK_SAMPLES; ++i)
        {
            sample_t yL = block->data[0][i];
            sample_t yR = block->data[1][i];

            for (size_t stage = 0; stage < FILTER_STAGES; ++stage)
            {
                sample_t z1L = state[0][band][stage][0];
                sample_t z2L = state[0][band][stage][1];
                const sample_t stageOutL = c.b0 * yL + z1L;
                z1L = c.b1 * yL - c.a1 * stageOutL + z2L;
                z2L = c.b2 * yL - c.a2 * stageOutL;
                state[0][band][stage][0] = z1L;
                state[0][band][stage][1] = z2L;
                yL = stageOutL;

                sample_t z1R = state[1][band][stage][0];
                sample_t z2R = state[1][band][stage][1];
                const sample_t stageOutR = c.b0 * yR + z1R;
                z1R = c.b1 * yR - c.a1 * stageOutR + z2R;
                z2R = c.b2 * yR - c.a2 * stageOutR;
                state[1][band][stage][0] = z1R;
                state[1][band][stage][1] = z2R;
                yR = stageOutR;
            }

            const float absLeft = fabsf(static_cast<float>(yL));
            const float absRight = fabsf(static_cast<float>(yR));
            leftPeak = std::max(leftPeak, absLeft);
            rightPeak = std::max(rightPeak, absRight);
            leftEnergy += static_cast<double>(yL) * static_cast<double>(yL);
            rightEnergy += static_cast<double>(yR) * static_cast<double>(yR);
        }

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

        const float binComp = binGainComp[band] > 0.0f ? binGainComp[band] : 1.0f;
        leftValue = sanitizeLevel(leftValue * gainNormalization * binComp);
        rightValue = sanitizeLevel(rightValue * gainNormalization * binComp);

        rawLeft[band] = leftValue;
        rawRight[band] = rightValue;
    }

    if (decorrelationReady)
    {
        for (size_t i = 0; i < binCount; ++i)
        {
            float sumL = 0.0f;
            float sumR = 0.0f;
            for (size_t j = 0; j < binCount; ++j)
            {
                sumL += decorrelation[i][j] * rawLeft[j];
                sumR += decorrelation[i][j] * rawRight[j];
            }
            correctedLeft[i] = sanitizeLevel(sumL);
            correctedRight[i] = sanitizeLevel(sumR);
        }
    }
    else
    {
        for (size_t i = 0; i < binCount; ++i)
        {
            correctedLeft[i] = rawLeft[i];
            correctedRight[i] = rawRight[i];
        }
    }

    for (size_t band = 0; band < binCount; ++band)
    {
        binsLeft[band] = binsLeft[band] * smoothing + correctedLeft[band] * (1.0f - smoothing);
        binsRight[band] = binsRight[band] * smoothing + correctedRight[band] * (1.0f - smoothing);
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
