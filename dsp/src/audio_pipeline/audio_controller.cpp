#include <Arduino.h>
#include "audio_controller.h"

AudioController::AudioController() : AudioComponent() {
    // This is OK since we're a singleton
    i2sAudioCallback = AudioController::processAudio;
}

void AudioController::process(AudioBuffer* block) {
    // This is the final destination - convert samples to int32 for output
    static bool wasClipped = false;
    bool clipped = false;
    for(int ch = 0; ch < AUDIO_CHANNELS; ch++) {
        for(int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
            sample_t sample  = block->data[ch][i];
            clipped |= (sample > 1.0f) || (sample < -1.0f);
            outputs[ch][i] = (int32_t)(max(-1.0f, min(1.0f, sample)) * 2147483647.0f);
        }
    }
    if(clipped && !wasClipped) {
        if(clipDetector) {
            clipDetector(true);
        }
        wasClipped = true;
    } else if(!clipped) {
        if(wasClipped && clipDetector) {
            clipDetector(false);
        }
        wasClipped = false;
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
