#ifndef filter_biquad_f_h_
#define filter_biquad_f_h_

#include <Arduino.h>
#include "audio_controller.h"
#include "arm_math.h"

#define MAX_BIQUAD_STAGES 4
#define STAGE_COEFFICIENTS 5
#define NUM_STATES 4

#define PI 3.14159265358979f
#define TWO_PI (2.0f * PI)

typedef float sample_t;

// Flat filter with unity gain.
static const sample_t identityCoefficients[STAGE_COEFFICIENTS] = {1.0, 0.0, 0.0, 0.0, 0.0};

class AudioFilterBiquadFloat : public AudioComponent
{
public:
    AudioFilterBiquadFloat(void) : AudioComponent() {}

    virtual void process(AudioBuffer *block) override;

    // Set the biquad coefficients directly
    void setCoefficients(uint32_t stage, const float *c);

    void setSosCoefficients(uint32_t stages, const sample_t *sos);

    const float *getCoefficients(uint32_t stage)
    {
        // Return identity filter if stage isn't active
        return stage < num_stages ? &coeff[stage * STAGE_COEFFICIENTS] : identityCoefficients;
    }

    // Compute common filter functions
    // http://www.musicdsp.org/files/Audio-EQ-Cookbook.txt
    void setLowpass(uint32_t stage, float frequency, float q = 0.7071f);
    void setHighpass(uint32_t stage, float frequency, float q = 0.7071);
    void setBandpass(uint32_t stage, float frequency, float q = 1.0);
    void setNotch(uint32_t stage, float frequency, float q = 1.0);
    void setLowShelf(uint32_t stage, float frequency, float gain, float slope = 1.0f);
    void setHighShelf(uint32_t stage, float frequency, float gain, float slope = 1.0f);
    void setPeakingEQ(uint32_t stage, float frequency, float q, float gain);
    void bypass(uint32_t stage);

    // private:
    float coeff[STAGE_COEFFICIENTS * MAX_BIQUAD_STAGES];
    float state[AUDIO_CHANNELS][NUM_STATES * MAX_BIQUAD_STAGES]; // extra space for safety
    uint32_t num_stages = 0;                                     // number of stages in use
    arm_biquad_casd_df1_inst_f32 iir_state[AUDIO_CHANNELS];
    void processChannel(sample_t *input, sample_t *output, arm_biquad_casd_df1_inst_f32 *iir_inst);
};

#endif
