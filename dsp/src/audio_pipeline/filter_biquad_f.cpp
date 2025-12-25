#ifndef FILTER_BIQUAD_F_H
#define FILTER_BIQUAD_F_H

#include <Arduino.h>
#include "filter_biquad_f.h"

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
    AudioBuffer* outputBlock = getBuffer();
    if (outputBlock == nullptr) {
        return;
    }
    for (int ch = 0; ch < AUDIO_CHANNELS; ch++)
    {
        processChannel(block->data[ch], outputBlock->data[ch]);
    }
    transmit(outputBlock);
    release(outputBlock);
}

void AudioFilterBiquadFloat::processChannel(sample_t *input, sample_t *output)
{
    sample_t *end = input + AUDIO_BLOCK_SAMPLES;
    for (; input < end; input++)
    {
        sample_t x = ((sample_t)*input) / 32768.0f;
        sample_t y = 0.0f;

        // Second direct form
        for (uint32_t i = 0; i < num_stages; i++)
        {
            sample_t *my_coeff = coeff + i * STAGE_COEFFICIENTS;
            sample_t *my_state = state + i * 2;
            sample_t w = x - my_state[0] * my_coeff[3] - my_state[1] * my_coeff[4];
            y = w * my_coeff[0] + my_state[0] * my_coeff[1] + my_state[1] * my_coeff[2];
            state[i * 2 + 1] = state[i * 2];
            state[i * 2] = w;
            x = y;
        }
        digitalWriteFast(LED_BUILTIN, y > 1.0 || y < -1.0 ? HIGH : LOW);
        *output = (int16_t)(max(-1.0f, min(1.0f, min(y, 1.0f))) * 32768.0f);
    }
}

#endif