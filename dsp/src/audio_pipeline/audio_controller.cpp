#include <Arduino.h>
#include "audio_controller.h"

AudioController::AudioController() : AudioComponent() {
    // This is OK since we're a singleton
    i2sAudioCallback = AudioController::processAudio;
}

void AudioController::process(AudioBuffer* block) {
    // This is the final destination - convert samples to int32 for output
    for(int ch = 0; ch < AUDIO_CHANNELS; ch++) {
        for(int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
            outputs[ch][i] = (int32_t)(max(-1.0f, min(1.0f, block->data[ch][i])) * 2147483647.0f);
        }
    }
}
void AudioController::processAudio(int32_t **inputs, int32_t **outputs)
{
    AudioBuffer* buffer = AudioBufferPool::getInstance().getBuffer();
    
    // Convert inputs from int32 to float
    for(int ch = 0; ch < AUDIO_CHANNELS; ch++) {
        for(int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
            buffer->data[ch][i] = ((sample_t)inputs[ch][i]) / 2147483648.0f;
        }
    }
    
    // Process through the pipeline - this modifies buffer and eventually calls back to process()
    getInstance()->transmit(buffer);
    
    // Copy the processed output to the output buffers
    for(int ch = 0; ch < AUDIO_CHANNELS; ch++) {
        for(int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
            outputs[ch][i] = getInstance()->outputs[ch][i];
        }
    }    
    getInstance()->release(buffer);
}
