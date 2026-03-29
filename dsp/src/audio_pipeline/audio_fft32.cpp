#include "audio_fft32.h"
#include <math.h>

AudioFFT32::AudioFFT32() : AudioComponent()
{
    arm_rfft_fast_init_f32(&fftInstance, FFT_SIZE);
    computeLogBinMap();
}

void AudioFFT32::process(AudioBuffer *block)
{
    if (!block)
        return;

    if (!isEnabled)
    {
        transmit(block);
        return;
    }

    int pos = writePos; // ISR is non-reentrant
    memcpy(&accumLeft[pos],  block->data[0], AUDIO_BLOCK_SAMPLES * sizeof(sample_t));
    memcpy(&accumRight[pos], block->data[1], AUDIO_BLOCK_SAMPLES * sizeof(sample_t));
    pos += AUDIO_BLOCK_SAMPLES;
    if (pos >= FFT_SIZE)
    {
        computeFrameBins(accumLeft, accumRight);
        aggregateFrameBins();
        pos = 0;
    }
    writePos = pos;
    transmit(block);
}

bool AudioFFT32::doFFT(sample_t *left, sample_t *right)
{
    if (!left || !right)
    {
        return false;
    }

    // Snapshot and reset aggregate under interrupt disable.
    uint32_t frames = 0;
    noInterrupts();
    frames = aggregatedFrames;
    if (frames == 0)
    {
        interrupts();
        return false;
    }
    for (int d = 0; d < FFT_DISPLAY_BINS; d++)
    {
        left[d] = aggregatedBinsLeft[d];
        right[d] = aggregatedBinsRight[d];
        aggregatedBinsLeft[d] = 0.0f;
        aggregatedBinsRight[d] = 0.0f;
    }
    aggregatedFrames = 0;
    interrupts();

#ifdef FFT_AGGREGATE_AVERAGE
    const float invFrames = 1.0f / static_cast<float>(frames);
    for (int d = 0; d < FFT_DISPLAY_BINS; d++)
    {
        left[d] *= invFrames;
        right[d] *= invFrames;
    }
#endif

    // Clamp to [0, 1] for safety.
    for (int d = 0; d < FFT_DISPLAY_BINS; d++)
    {
        if (left[d] < 0.0f) left[d] = 0.0f;
        if (left[d] > 1.0f) left[d] = 1.0f;
        if (right[d] < 0.0f) right[d] = 0.0f;
        if (right[d] > 1.0f) right[d] = 1.0f;
    }
    return true;
}

void AudioFFT32::computeFrameBins(const sample_t *inputLeft, const sample_t *inputRight)
{
    const AmplitudeScale currentScale = amplitudeScale;

    memcpy(workLeft, inputLeft, FFT_SIZE * sizeof(sample_t));
    memcpy(workRight, inputRight, FFT_SIZE * sizeof(sample_t));

    // Apply Hann window to reduce spectral leakage.
    for (int i = 0; i < FFT_SIZE; i++)
    {
        float w = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (FFT_SIZE - 1)));
        workLeft[i] *= w;
        workRight[i] *= w;
    }

    // Forward real FFT → packed complex output (FFT_SIZE floats each).
    arm_rfft_fast_f32(&fftInstance, workLeft, fftOutLeft, 0);
    arm_rfft_fast_f32(&fftInstance, workRight, fftOutRight, 0);

    // Compute per-bin magnitudes (FFT_BINS = FFT_SIZE/2 bins from FFT_SIZE complex floats).
    arm_cmplx_mag_f32(fftOutLeft,  magLeft,  FFT_BINS);
    arm_cmplx_mag_f32(fftOutRight, magRight, FFT_BINS);

#ifdef FFT_USE_POWER_SPECTRUM
    // Power spectrum path: |X[k]|^2, normalized by FFT_SIZE^2.
    arm_mult_f32(magLeft, magLeft, magLeft, FFT_BINS);
    arm_mult_f32(magRight, magRight, magRight, FFT_BINS);
    const float normScale = 1.0f / (FFT_SIZE * FFT_SIZE);
#else
    // Magnitude spectrum path (default), normalized by FFT_SIZE.
    const float normScale = 1.0f / FFT_SIZE;
#endif
    arm_scale_f32(magLeft, normScale, magLeft, FFT_BINS);
    arm_scale_f32(magRight, normScale, magRight, FFT_BINS);

    // Map linear FFT bins to log-spaced display bins (peak within each band),
    // then scale linearly into [0, 1].
    for (int d = 0; d < FFT_DISPLAY_BINS; d++)
    {
        float maxL = 0.0f, maxR = 0.0f;
        for (int k = binStart[d]; k <= binEnd[d]; k++)
        {
            if (magLeft[k]  > maxL) maxL = magLeft[k];
            if (magRight[k] > maxR) maxR = magRight[k];
        }

        if (currentScale == AmplitudeScale::Decibels)
        {
            const float dbRange = FFT_DB_CEILING - FFT_DB_FLOOR;
            const float dbL = 20.0f * log10f(maxL > 1e-10f ? maxL : 1e-10f);
            const float dbR = 20.0f * log10f(maxR > 1e-10f ? maxR : 1e-10f);
            frameBinsLeft[d] = (dbL - FFT_DB_FLOOR) / dbRange;
            frameBinsRight[d] = (dbR - FFT_DB_FLOOR) / dbRange;
        }
        else
        {
            frameBinsLeft[d] = maxL * FFT_LINEAR_GAIN;
            frameBinsRight[d] = maxR * FFT_LINEAR_GAIN;
        }
    }
}

void AudioFFT32::aggregateFrameBins()
{
    if (aggregatedFrames == 0)
    {
        for (int d = 0; d < FFT_DISPLAY_BINS; d++)
        {
            aggregatedBinsLeft[d] = frameBinsLeft[d];
            aggregatedBinsRight[d] = frameBinsRight[d];
        }
        aggregatedFrames = 1;
        return;
    }

#ifdef FFT_AGGREGATE_AVERAGE
    for (int d = 0; d < FFT_DISPLAY_BINS; d++)
    {
        aggregatedBinsLeft[d] += frameBinsLeft[d];
        aggregatedBinsRight[d] += frameBinsRight[d];
    }
#else
    for (int d = 0; d < FFT_DISPLAY_BINS; d++)
    {
        if (frameBinsLeft[d] > aggregatedBinsLeft[d])
        {
            aggregatedBinsLeft[d] = frameBinsLeft[d];
        }
        if (frameBinsRight[d] > aggregatedBinsRight[d])
        {
            aggregatedBinsRight[d] = frameBinsRight[d];
        }
    }
#endif
    aggregatedFrames++;
}

void AudioFFT32::setSampleRate(uint32_t rate)
{
    noInterrupts();
    sampleRate = rate;
    computeLogBinMap();
    aggregatedFrames = 0;
    for (int d = 0; d < FFT_DISPLAY_BINS; d++)
    {
        aggregatedBinsLeft[d] = 0.0f;
        aggregatedBinsRight[d] = 0.0f;
    }
    interrupts();
}

void AudioFFT32::setAmplitudeScale(AmplitudeScale scale)
{
    noInterrupts();
    amplitudeScale = scale;
    aggregatedFrames = 0;
    for (int d = 0; d < FFT_DISPLAY_BINS; d++)
    {
        aggregatedBinsLeft[d] = 0.0f;
        aggregatedBinsRight[d] = 0.0f;
    }
    interrupts();
}

void AudioFFT32::setEnabled(bool enabled)
{
    noInterrupts();
    isEnabled = enabled;
    writePos = 0;
    aggregatedFrames = 0;
    for (int d = 0; d < FFT_DISPLAY_BINS; d++)
    {
        aggregatedBinsLeft[d] = 0.0f;
        aggregatedBinsRight[d] = 0.0f;
    }
    interrupts();
}

void AudioFFT32::computeLogBinMap()
{
    const float fMax  = sampleRate / 2.0f;
    const float fMin  = (LOG_F_MIN < (fMax * 0.95f)) ? LOG_F_MIN : (fMax * 0.95f);
    const float ratio = (fMin > 0.0f) ? (fMax / fMin) : 1.0f;

    for (int d = 0; d < FFT_DISPLAY_BINS; d++)
    {
        float fLo = fMin * powf(ratio, (float)d       / FFT_DISPLAY_BINS);
        float fHi = fMin * powf(ratio, (float)(d + 1) / FFT_DISPLAY_BINS);

        // Bin k occupies the frequency range [k * Fs/N, (k+1) * Fs/N).
        int kLo = (int)ceilf(fLo * FFT_SIZE / sampleRate);
        int kHi = (int)floorf(fHi * FFT_SIZE / sampleRate);

        // Clamp: skip DC (bin 0), stay below Nyquist bin; ensure at least one bin.
        kLo = max(kLo, 1);
        kHi = min(kHi, FFT_BINS - 1);
        if (kHi < kLo)
        {
            kHi = kLo;
        }
        binStart[d] = kLo;
        binEnd[d] = kHi;
    }
}


