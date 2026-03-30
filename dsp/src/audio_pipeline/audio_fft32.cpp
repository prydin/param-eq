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

    int pos = writePos; // ISR is non-reentrant
    memcpy(&accumLeft[pos],  block->data[0], AUDIO_BLOCK_SAMPLES * sizeof(sample_t));
    memcpy(&accumRight[pos], block->data[1], AUDIO_BLOCK_SAMPLES * sizeof(sample_t));

    // Build decimated stream for low-band FFT using simple boxcar anti-aliasing.
    for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++)
    {
        decimAccLeft += block->data[0][i];
        decimAccRight += block->data[1][i];
        decimPhase++;
        if (decimPhase >= FFT_SPLIT_DECIMATION)
        {
            lowAccumLeft[lowWritePos] = decimAccLeft / FFT_SPLIT_DECIMATION;
            lowAccumRight[lowWritePos] = decimAccRight / FFT_SPLIT_DECIMATION;
            decimAccLeft = 0.0f;
            decimAccRight = 0.0f;
            decimPhase = 0;
            lowWritePos++;
            if (lowWritePos >= FFT_SIZE)
            {
                computeSpectrum(lowAccumLeft, lowAccumRight, lowMagLeft, lowMagRight);
                lowWritePos = 0;
                lowSpectrumReady = true;
            }
        }
    }

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

    computeSpectrum(workLeft, workRight, magLeft, magRight);

    // Map bins using split FFT: decimated spectrum for low frequencies,
    // full-rate spectrum for high frequencies.
    for (int d = 0; d < FFT_DISPLAY_BINS; d++)
    {
        const bool useLow = lowSpectrumReady && (binCenterHz[d] <= FFT_SPLIT_CROSSOVER_HZ);
        const sample_t *srcLeft = useLow ? lowMagLeft : magLeft;
        const sample_t *srcRight = useLow ? lowMagRight : magRight;
        const int kLo = useLow ? binStartLow[d] : binStartHigh[d];
        const int kHi = useLow ? binEndLow[d] : binEndHigh[d];

        float maxL = 0.0f, maxR = 0.0f;
        for (int k = kLo; k <= kHi; k++)
        {
            if (srcLeft[k] > maxL) maxL = srcLeft[k];
            if (srcRight[k] > maxR) maxR = srcRight[k];
        }

        // Suppress baseline noise floor so no-signal bars rest near zero.
        maxL = max(0.0f, maxL - FFT_MAG_NOISE_FLOOR);
        maxR = max(0.0f, maxR - FFT_MAG_NOISE_FLOOR);

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

void AudioFFT32::computeSpectrum(sample_t *leftInput, sample_t *rightInput,
                                 sample_t *leftMagOut, sample_t *rightMagOut)
{
    // Remove frame DC offset to avoid low-bin bias when no signal is present.
    float meanL = 0.0f;
    float meanR = 0.0f;
    for (int i = 0; i < FFT_SIZE; i++)
    {
        meanL += leftInput[i];
        meanR += rightInput[i];
    }
    meanL /= FFT_SIZE;
    meanR /= FFT_SIZE;
    for (int i = 0; i < FFT_SIZE; i++)
    {
        leftInput[i] -= meanL;
        rightInput[i] -= meanR;
    }

    // Apply 4-term Blackman-Harris window to reduce sidelobes.
    constexpr float kA0 = 0.35875f;
    constexpr float kA1 = 0.48829f;
    constexpr float kA2 = 0.14128f;
    constexpr float kA3 = 0.01168f;
    constexpr float kWindowCoherentGain = kA0;
    for (int i = 0; i < FFT_SIZE; i++)
    {
        const float phase = (2.0f * M_PI * i) / (FFT_SIZE - 1);
        float w = kA0 -
                  kA1 * cosf(phase) +
                  kA2 * cosf(2.0f * phase) -
                  kA3 * cosf(3.0f * phase);
        leftInput[i] *= w;
        rightInput[i] *= w;
    }

    // Forward real FFT → packed complex output (FFT_SIZE floats each).
    arm_rfft_fast_f32(&fftInstance, leftInput, fftOutLeft, 0);
    arm_rfft_fast_f32(&fftInstance, rightInput, fftOutRight, 0);

    // Compute per-bin magnitudes (FFT_BINS = FFT_SIZE/2 bins from FFT_SIZE complex floats).
    arm_cmplx_mag_f32(fftOutLeft, leftMagOut, FFT_BINS);
    arm_cmplx_mag_f32(fftOutRight, rightMagOut, FFT_BINS);

#ifdef FFT_USE_POWER_SPECTRUM
    // Power spectrum path: |X[k]|^2, normalized by FFT_SIZE^2.
    arm_mult_f32(leftMagOut, leftMagOut, leftMagOut, FFT_BINS);
    arm_mult_f32(rightMagOut, rightMagOut, rightMagOut, FFT_BINS);
    const float normScale = 1.0f / ((FFT_SIZE * FFT_SIZE) * (kWindowCoherentGain * kWindowCoherentGain));
#else
    // Magnitude spectrum path (default), normalized by FFT_SIZE.
    const float normScale = 1.0f / (FFT_SIZE * kWindowCoherentGain);
#endif
    arm_scale_f32(leftMagOut, normScale, leftMagOut, FFT_BINS);
    arm_scale_f32(rightMagOut, normScale, rightMagOut, FFT_BINS);
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
    writePos = 0;
    lowWritePos = 0;
    decimPhase = 0;
    decimAccLeft = 0.0f;
    decimAccRight = 0.0f;
    lowSpectrumReady = false;
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

void AudioFFT32::computeLogBinMap()
{
    const float fMax  = sampleRate / 2.0f;
    const float fMaxLow = sampleRate / (2.0f * FFT_SPLIT_DECIMATION);
    const float fMin  = (LOG_F_MIN < (fMax * 0.95f)) ? LOG_F_MIN : (fMax * 0.95f);
    const float ratio = (fMin > 0.0f) ? (fMax / fMin) : 1.0f;

    for (int d = 0; d < FFT_DISPLAY_BINS; d++)
    {
        float fLo = fMin * powf(ratio, (float)d       / FFT_DISPLAY_BINS);
        float fHi = fMin * powf(ratio, (float)(d + 1) / FFT_DISPLAY_BINS);
        binCenterHz[d] = sqrtf(fLo * fHi);

        // High FFT map
        int kLo = (int)ceilf(fLo * FFT_SIZE / sampleRate);
        int kHi = (int)floorf(fHi * FFT_SIZE / sampleRate);

        kLo = max(kLo, 1);
        kHi = min(kHi, FFT_BINS - 1);
        if (kHi < kLo)
        {
            kHi = kLo;
        }
        binStartHigh[d] = kLo;
        binEndHigh[d] = kHi;

        // Low (decimated) FFT map. Clamp requested frequencies to low Nyquist.
        float loL = min(fLo, fMaxLow * 0.98f);
        float hiL = min(fHi, fMaxLow * 0.98f);
        int lLo = (int)ceilf(loL * FFT_SIZE / (sampleRate / FFT_SPLIT_DECIMATION));
        int lHi = (int)floorf(hiL * FFT_SIZE / (sampleRate / FFT_SPLIT_DECIMATION));
        lLo = max(lLo, 1);
        lHi = min(lHi, FFT_BINS - 1);
        if (lHi < lLo)
        {
            lHi = lLo;
        }
        binStartLow[d] = lLo;
        binEndLow[d] = lHi;
    }
}


