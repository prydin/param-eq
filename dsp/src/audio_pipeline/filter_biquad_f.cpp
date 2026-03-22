// Copyright (c) 2026 Pontus Rydin
// SPDX-License-Identifier: MIT
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include <Arduino.h>
#include "filter_biquad_f.h"
#include "base.h"
#include "arm_math.h"

void AudioFilterBiquadFloat::process(AudioBuffer *block)
{
    if (!block)
    {
        return;
    }
    if (num_stages == 0)
    {
        // No stages defined, just pass through the input
        transmit(block);
        return;
    }
    AudioBuffer *outputBlock = getBuffer();
    if (outputBlock == nullptr)
    {
        return;
    }
    for (int ch = 0; ch < AUDIO_CHANNELS; ch++)
    {
        processChannel(block->data[ch], outputBlock->data[ch], &iir_state[ch]);
    }
    transmit(outputBlock);
    release(outputBlock);
}

void AudioFilterBiquadFloat::processChannel(sample_t *input, sample_t *output, filter_param_t *iir_inst)
{
// Use ARM CMSIS-DSP biquad filter implementations
#ifdef USE_DOUBLE_SAMPLES
    arm_biquad_cascade_df2T_f64(iir_inst, input, output, AUDIO_BLOCK_SAMPLES);
#else
    arm_biquad_cascade_df2T_f32(iir_inst, input, output, AUDIO_BLOCK_SAMPLES);
#endif
}

void AudioFilterBiquadFloat::setCoefficients(uint32_t stage, const sample_t *c)
{
    if (stage >= MAX_BIQUAD_STAGES)
    {
        return;
    }

    __disable_irq();
    for (int i = 0; i < STAGE_COEFFICIENTS; i++)
    {
        // ARM CMSIS-DSP uses negative feedback denominator coefficients
        coeff[stage * STAGE_COEFFICIENTS + i] = i > 2 ? -c[i] : c[i];
    }
    if (stage + 1 > num_stages)
    {
        num_stages = stage + 1;
        for (int ch = 0; ch < AUDIO_CHANNELS; ch++)
        {
#ifdef USE_DOUBLE_SAMPLES
            arm_biquad_cascade_df2T_init_f64(&iir_state[ch], num_stages, coeff, state[ch]);
#else
            arm_biquad_cascade_df2T_init_f32(&iir_state[ch], num_stages, coeff, state[ch]);
#endif
        }
    }
    // Serial.printf("Set Biquad Stage %d out of %d Coefficients: b0=%f b1=%f b2=%f a1=%f a2=%f\n",
    //               stage, num_stages, c[0], c[1], c[2], c[3], c[4]);
    __enable_irq();
    // Serial.printf("num_stages = %d\n", num_stages);
}

void AudioFilterBiquadFloat::setSosCoefficients(uint32_t stages, const sample_t *sos)
{
    if (stages > MAX_BIQUAD_STAGES)
    {
        stages = MAX_BIQUAD_STAGES;
    }

    __disable_irq();
    for (uint32_t i = 0; i < stages * STAGE_COEFFICIENTS; i++)
    {
        coeff[i] = sos[i];
    }
    num_stages = stages;
    __enable_irq();
}

void AudioFilterBiquadFloat::setLowpass(uint32_t stage, double frequency, double sampleRate, double q)
{
    sample_t c[STAGE_COEFFICIENTS];
    double w0 = frequency * (TWO_PI / sampleRate);
    double sinW0 = sin(w0);
    double alpha = sinW0 / ((double)q * 2.0);
    double cosW0 = cos(w0);
    double scale = 1.0 / (1.0 + alpha); // which is equal to 1.0 / a0
    /* b0 */ c[0] = ((1.0 - cosW0) / 2.0) * scale;
    /* b1 */ c[1] = (1.0 - cosW0) * scale;
    /* b2 */ c[2] = c[0];
    /* a1 */ c[3] = (-2.0 * cosW0) * scale;
    /* a2 */ c[4] = (1.0 - alpha) * scale;
    setCoefficients(stage, c);
}

void AudioFilterBiquadFloat::setHighpass(uint32_t stage, double frequency, double sampleRate, double q)
{
    sample_t c[STAGE_COEFFICIENTS];
    double w0 = frequency * (TWO_PI / sampleRate);
    double sinW0 = sin(w0);
    double alpha = sinW0 / ((double)q * 2.0);
    double cosW0 = cos(w0);
    double scale = 1.0 / (1.0 + alpha);
    /* b0 */ c[0] = ((1.0 + cosW0) / 2.0) * scale;
    /* b1 */ c[1] = -(1.0 + cosW0) * scale;
    /* b2 */ c[2] = c[0];
    /* a1 */ c[3] = (-2.0 * cosW0) * scale;
    /* a2 */ c[4] = (1.0 - alpha) * scale;
    setCoefficients(stage, c);
}

void AudioFilterBiquadFloat::setBandpass(uint32_t stage, double frequency, double sampleRate, double q)
{
    sample_t c[STAGE_COEFFICIENTS];
    double w0 = frequency * (TWO_PI / sampleRate);
    double sinW0 = sin(w0);
    double alpha = sinW0 / ((double)q * 2.0);
    double cosW0 = cos(w0);
    double scale = 1.0 / (1.0 + alpha);
    /* b0 */ c[0] = alpha * scale;
    /* b1 */ c[1] = 0;
    /* b2 */ c[2] = (-alpha) * scale;
    /* a1 */ c[3] = (-2.0 * cosW0) * scale;
    /* a2 */ c[4] = (1.0 - alpha) * scale;
    setCoefficients(stage, c);
}

void AudioFilterBiquadFloat::setNotch(uint32_t stage, double frequency, double sampleRate, double q)
{
    sample_t c[STAGE_COEFFICIENTS];
    double w0 = frequency * (TWO_PI / sampleRate);
    double sinW0 = sin(w0);
    double alpha = sinW0 / ((double)q * 2.0);
    double cosW0 = cos(w0);
    double scale = 1.0 / (1.0 + alpha); // which is equal to 1.0 / a0
    /* b0 */ c[0] = scale;
    /* b1 */ c[1] = (-2.0 * cosW0) * scale;
    /* b2 */ c[2] = c[0];
    /* a1 */ c[3] = (-2.0 * cosW0) * scale;
    /* a2 */ c[4] = (1.0 - alpha) * scale;
    setCoefficients(stage, c);
}

void AudioFilterBiquadFloat::setLowShelf(uint32_t stage, double frequency, double sampleRate, double gain, double slope)
{
    sample_t c[STAGE_COEFFICIENTS];
    double a = pow(10.0, gain / 40.0);
    double w0 = frequency * (TWO_PI / sampleRate);
    double sinW0 = sin(w0);
    double cosW0 = cos(w0);
    double ss = (a * a + 1.0) * (1.0 / slope - 1.0) + 2.0 * a;
    if (ss < 0.0)
    {
        // Avoid taking the square root of a negative number
        return;
    }

    double sinsq = sinW0 * sqrt(ss);
    double aMinus = (a - 1.0) * cosW0;
    double aPlus = (a + 1.0) * cosW0;
    double scale = 1.0 / ((a + 1.0) + aMinus + sinsq);
    /* b0 */ c[0] = a * ((a + 1.0) - aMinus + sinsq) * scale;
    /* b1 */ c[1] = 2.0 * a * ((a - 1.0) - aPlus) * scale;
    /* b2 */ c[2] = a * ((a + 1.0) - aMinus - sinsq) * scale;
    /* a1 */ c[3] = -2.0 * ((a - 1.0) + aPlus) * scale;
    /* a2 */ c[4] = ((a + 1.0) + aMinus - sinsq) * scale;
    setCoefficients(stage, c);
}

void AudioFilterBiquadFloat::setHighShelf(uint32_t stage, double frequency, double sampleRate, double gain, double slope)
{
    sample_t c[STAGE_COEFFICIENTS];
    double a = pow(10.0, gain / 40.0);
    double w0 = frequency * (TWO_PI / sampleRate);
    double sinW0 = sin(w0);
    double cosW0 = cos(w0);
    double ss = (a * a + 1.0) * (1.0 / (double)slope - 1.0) + 2.0 * a;
    if (ss < 0.0)
    {
        // Avoid taking the square root of a negative number
        return;
    }
    double sinsq = sinW0 * sqrt(ss);
    double aMinus = (a - 1.0) * cosW0;
    double aPlus = (a + 1.0) * cosW0;
    double scale = 1.0 / ((a + 1.0) - aMinus + sinsq);
    /* b0 */ c[0] = a * ((a + 1.0) + aMinus + sinsq) * scale;
    /* b1 */ c[1] = -2.0 * a * ((a - 1.0) + aPlus) * scale;
    /* b2 */ c[2] = a * ((a + 1.0) + aMinus - sinsq) * scale;
    /* a1 */ c[3] = 2.0 * ((a - 1.0) - aPlus) * scale;
    /* a2 */ c[4] = ((a + 1.0) - aMinus - sinsq) * scale;
    setCoefficients(stage, c);
}

void AudioFilterBiquadFloat::setPeakingEQ(uint32_t stage, double frequency, double sampleRate, double q, double gain)
{
    sample_t c[STAGE_COEFFICIENTS];
    double a = pow(10.0, gain / 40.0);
    double w0 = frequency * (TWO_PI / sampleRate);
    double sinW0 = sin(w0);
    double alpha = sinW0 / (q * 2.0f);
    double cosW0 = cos(w0);
    sample_t scale = 1.0 / (1.0 + alpha / a);
    /* b0 */ c[0] = (1.0 + alpha * a) * scale;
    /* b1 */ c[1] = (-2.0 * cosW0) * scale;
    /* b2 */ c[2] = (1.0 - alpha * a) * scale;
    /* a1 */ c[3] = (-2.0 * cosW0) * scale;
    /* a2 */ c[4] = (1.0 - alpha / a) * scale;
    setCoefficients(stage, c);
}

void AudioFilterBiquadFloat::bypass(uint32_t stage)
{
    setCoefficients(stage, identityCoefficients);
}