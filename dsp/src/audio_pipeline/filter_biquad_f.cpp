#ifndef FILTER_BIQUAD_F_H
#define FILTER_BIQUAD_F_H

#include <Arduino.h>
#include "filter_biquad_f.h"
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

void AudioFilterBiquadFloat::processChannel(sample_t *input, sample_t *output, arm_biquad_casd_df1_inst_f32 *iir_inst)
{
    // Use ARM CMSIS-DSP biquad filter implementations
    // Print the coefficients for debugging
    #ifdef VERBOSE
    Serial.printf("Biquad processing with %d stages. Coefficients:\n", num_stages);
    for (uint32_t stage = 0; stage < num_stages; stage++)
    {
        Serial.printf(" Stage %d: b0=%f b1=%f b2=%f a1=%f a2=%f\n",
            stage,
            iir_inst.pCoeffs[stage * STAGE_COEFFICIENTS + 0],
            iir_inst.pCoeffs[stage * STAGE_COEFFICIENTS + 1],
            iir_inst.pCoeffs[stage * STAGE_COEFFICIENTS + 2],
            iir_inst.pCoeffs[stage * STAGE_COEFFICIENTS + 3],
            iir_inst.pCoeffs[stage * STAGE_COEFFICIENTS + 4]);
    }
    #endif
    arm_biquad_cascade_df1_f32(iir_inst, input, output, AUDIO_BLOCK_SAMPLES);
    #ifdef VERBOSE
    // Print all samples in output for debugging
    Serial.printf("Biquad output: ");
    for (uint32_t i = 0; i < AUDIO_BLOCK_SAMPLES; i++)
    {
        Serial.printf("%f %f ",input[i], output[i]);
    }
    Serial.println();
    #endif
}

#endif