#include <Teensy4i2s.h>
#include <arm_math.h>
#include "pipeline.h"

#define NUM_BUFFERS 16

struct AudioBuffer
{
    sample_t data[AUDIO_CHANNELS][AUDIO_BLOCK_SAMPLES];
    AudioBuffer* next;
};

int bufferHead = 0; 
sample_t*** buffers[NUM_BUFFERS];

sample_t** getBuffer()
{
    if(bufferHead == NUM_BUFFERS)
    {
        // No available buffer
        return nullptr;
    }
    return buffers[bufferHead++];
}

void genTestSignal(float **outputs)
{
    // Shouuld read from I2S, but just a simple waveform for testing
    for (size_t i = 0; i < AUDIO_BLOCK_SAMPLES; i++)
    {
        // use can use regular sinf() as well, but it's highly recommended
        // to use these optimised arm-specific functions whenever possible
        sample_t sig = arm_sin_f32(acc * 0.01f * 2.0f * M_PI);
        outputs[0][i] = sig;
        outputs[1][i] = sig;
    }
}

int acc = 0;
void processAudio(int32_t **inputs, int32_t **outputs)
{
}
