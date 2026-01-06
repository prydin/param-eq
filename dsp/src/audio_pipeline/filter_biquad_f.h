#ifndef filter_biquad_f_h_
#define filter_biquad_f_h_

#include <Arduino.h>
#include "base.h"
#include "audio_controller.h"
#include "arm_math.h"

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
    void setLowpass(uint32_t stage, double frequency, double q = 0.7071f);
    void setHighpass(uint32_t stage, double frequency, double q = 0.7071);
    void setBandpass(uint32_t stage, double frequency, double q = 1.0);
    void setNotch(uint32_t stage, double frequency, double q = 1.0);
    void setLowShelf(uint32_t stage, double frequency, double gain, double slope = 1.0f);
    void setHighShelf(uint32_t stage, double frequency, double gain, double slope = 1.0f);
    void setPeakingEQ(uint32_t stage, double frequency, double q, double gain);
    void bypass(uint32_t stage);    

    // private:
    sample_t coeff[STAGE_COEFFICIENTS * MAX_BIQUAD_STAGES];
    sample_t state[AUDIO_CHANNELS][NUM_STATES * MAX_BIQUAD_STAGES]; // extra space for safety
    uint32_t num_stages = 0;                                     // number of stages in use
    filter_param_t iir_state[AUDIO_CHANNELS];
    void processChannel(sample_t *input, sample_t *output, filter_param_t *iir_inst);
};

#endif
