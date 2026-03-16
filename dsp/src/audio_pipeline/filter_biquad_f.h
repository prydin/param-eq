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
#ifndef filter_biquad_f_h_
#define filter_biquad_f_h_

#include <Arduino.h>
#include "base.h"
#include "arm_math.h"
#include "audio_component.h"

#define MAX_BIQUAD_STAGES 4
#define STAGE_COEFFICIENTS 5
#define NUM_STATES 4

// Flat filter with unity gain.
static const sample_t identityCoefficients[STAGE_COEFFICIENTS] = {1.0, 0.0, 0.0, 0.0, 0.0};

#ifdef USE_DOUBLE_SAMPLES
typedef arm_biquad_cascade_df2T_instance_f64 filter_param_t;
#else
typedef arm_biquad_cascade_df2T_instance_f32 filter_param_t;
#endif

class AudioFilterBiquadFloat : public AudioComponent
{
public:
    AudioFilterBiquadFloat(void) : AudioComponent() {}

    virtual void process(AudioBuffer *block) override;

    // Set the biquad coefficients directly
    void setCoefficients(uint32_t stage, const sample_t *c);

    void setSosCoefficients(uint32_t stages, const sample_t *sos);

    const sample_t *getCoefficients(uint32_t stage)
    {
        // Return identity filter if stage isn't active
        return stage < num_stages ? &coeff[stage * STAGE_COEFFICIENTS] : identityCoefficients;
    }

    // Compute common filter functions
    // http://www.musicdsp.org/files/Audio-EQ-Cookbook.txt
    void setLowpass(uint32_t stage, double frequency, double sampleRate, double q = 0.7071f);
    void setHighpass(uint32_t stage, double frequency, double sampleRate, double q = 0.7071);
    void setBandpass(uint32_t stage, double frequency, double sampleRate, double q = 1.0);
    void setNotch(uint32_t stage, double frequency, double sampleRate, double q = 1.0);
    void setLowShelf(uint32_t stage, double frequency, double sampleRate, double gain, double slope = 1.0f);
    void setHighShelf(uint32_t stage, double frequency, double sampleRate, double gain, double slope = 1.0f);
    void setPeakingEQ(uint32_t stage, double frequency, double sampleRate, double q, double gain);
    void bypass(uint32_t stage);    

private:
    sample_t coeff[STAGE_COEFFICIENTS * MAX_BIQUAD_STAGES];
    sample_t state[AUDIO_CHANNELS][NUM_STATES * MAX_BIQUAD_STAGES]; // extra space for safety
    uint32_t num_stages = 0;                                     // number of stages in use
    filter_param_t iir_state[AUDIO_CHANNELS];
    void processChannel(sample_t *input, sample_t *output, filter_param_t *iir_inst);
};

#endif
