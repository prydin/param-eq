#ifndef filter_biquad_f_h_
#define filter_biquad_f_h_

#define USE_ARM_DSP

#include <Arduino.h>
#include "audio_controller.h"
#include "arm_math.h"

#define MAX_BIQUAD_STAGES 4
#define STAGE_COEFFICIENTS 5
#ifdef USE_ARM_DSP
#define NUM_STATES 4
#else
#define NUM_STATES 2
#endif

typedef float sample_t;

// Flat filter with unity gain.
static const sample_t identityCoefficients[STAGE_COEFFICIENTS] = {1.0, 0.0, 0.0, 0.0, 0.0};

class AudioFilterBiquadFloat : public AudioComponent
{
public:
    AudioFilterBiquadFloat(void) : AudioComponent() {}

    virtual void process(AudioBuffer *block) override;

    // Set the biquad coefficients directly
    void setCoefficients(uint32_t stage, const float *c)
    {
        __disable_irq();
        for (int i = 0; i < STAGE_COEFFICIENTS; i++)
        {
            // ARM CMSIS-DSP uses negative feedback denominator coefficients
            coeff[stage * STAGE_COEFFICIENTS + i] = i > 2 ? -c[i] : c[i];
        }
        if (stage + 1 > num_stages)
        {
            num_stages = stage + 1;
        }
        Serial.printf("Set Biquad Stage %d out of %d Coefficients: b0=%f b1=%f b2=%f a1=%f a2=%f\n",
                      stage, num_stages, c[0], c[1], c[2], c[3], c[4]);
        for (int ch = 0; ch < AUDIO_CHANNELS; ch++)
        {
            arm_biquad_cascade_df1_init_f32(&iir_state[ch], num_stages, coeff, state[ch]);
        }
        __enable_irq();
        // Serial.printf("num_stages = %d\n", num_stages);
    }

    void
    setSosCoefficients(uint32_t stages, const sample_t *sos)
    {
        __disable_irq() for (uint32_t i = 0; i < stages * STAGE_COEFFICIENTS; i++)
        {
            coeff[i] = sos[i];
        }
        num_stages = stages;
        __enable_irq()
    }

    const float *getCoefficients(uint32_t stage)
    {
        // Return identity filter if stage isn't active
        return stage < num_stages ? &coeff[stage * STAGE_COEFFICIENTS] : identityCoefficients;
    }

    // Compute common filter functions
    // http://www.musicdsp.org/files/Audio-EQ-Cookbook.txt
    void setLowpass(uint32_t stage, float frequency, float q = 0.7071f)
    {
        sample_t c[STAGE_COEFFICIENTS];
        double w0 = frequency * (2 * 3.141592654 / AudioController::getSampleRate());
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
    void setHighpass(uint32_t stage, float frequency, float q = 0.7071)
    {
        sample_t c[STAGE_COEFFICIENTS];
        double w0 = frequency * (2 * 3.141592654 / AudioController::getSampleRate());
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
    void setBandpass(uint32_t stage, float frequency, float q = 1.0)
    {
        sample_t c[STAGE_COEFFICIENTS];
        double w0 = frequency * (2 * 3.141592654 / AudioController::getSampleRate());
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
    void setNotch(uint32_t stage, float frequency, float q = 1.0)
    {
        sample_t c[STAGE_COEFFICIENTS];
        double w0 = frequency * (2 * 3.141592654 / (double)AudioController::getSampleRate());
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
    void setLowShelf(uint32_t stage, float frequency, float gain, float slope = 1.0f)
    {
        float c[STAGE_COEFFICIENTS];
        double a = pow(10.0, gain / 40.0);
        double w0 = frequency * (2 * 3.141592654 / (double)AudioController::getSampleRate());
        double sinW0 = sin(w0);
        // double alpha = (sinW0 * sqrt((a+1/a)*(1/slope-1)+2) ) / 2.0;
        double cosW0 = cos(w0);
        // generate three helper-values (intermediate results):
        double sinsq = sinW0 * sqrt((pow(a, 2.0) + 1.0) * (1.0 / slope - 1.0) + 2.0 * a);
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
    void setHighShelf(uint32_t stage, float frequency, float gain, float slope = 1.0f)
    {
        float c[STAGE_COEFFICIENTS];
        double a = pow(10.0, gain / 40.0f);
        double w0 = frequency * (2.0f * 3.141592654f / (double)AudioController::getSampleRate());
        double sinW0 = sin(w0);
        // sample_t alpha = (sinW0 * sqrt((a+1/a)*(1/slope-1)+2) ) / 2.0;
        double cosW0 = cos(w0);
        // generate three helper-values (intermediate results):
        double ss = (a * a + 1.0) * (1.0 / (double)slope - 1.0) + 2.0 * a;
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
        /* b1 */ c[1] = -2.0 * a * ((a - 1.0) - aPlus) * scale;
        /* b2 */ c[2] = a * ((a + 1.0) - aMinus - sinsq) * scale;
        /* a1 */ c[3] = 2.0 * ((a - 1.0) + aPlus) * scale;
        /* a2 */ c[4] = ((a + 1.0) + aMinus - sinsq) * scale;
        setCoefficients(stage, c);
    }

    void setPeakingEQ(uint32_t stage, float frequency, float q, float gain)
    {
        float c[STAGE_COEFFICIENTS];
        double a = pow(10.0, gain / 40.0f);
        double w0 = frequency * (2.0f * 3.141592654f / (double)AudioController::getSampleRate());
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

    void bypass(uint32_t stage)
    {
        setCoefficients(stage, identityCoefficients);
    }

    // private:
    float coeff[STAGE_COEFFICIENTS * MAX_BIQUAD_STAGES];
    float state[AUDIO_CHANNELS][NUM_STATES * MAX_BIQUAD_STAGES]; // extra space for safety
    uint32_t num_stages = 0;                                     // number of stages in use
    arm_biquad_casd_df1_inst_f32 iir_state[AUDIO_CHANNELS];
    void processChannel(sample_t *input, sample_t *output, arm_biquad_casd_df1_inst_f32 *iir_inst);
};

#endif
