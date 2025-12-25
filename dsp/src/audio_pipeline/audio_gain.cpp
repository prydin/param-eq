#include "audio_gain.h"
#include <Teensy4i2s.h>

AudioGain::AudioGain() : gain(1.0f) {
    // Default to unity gain (no change)
}

void AudioGain::setGain(float linearGain) {
    gain = linearGain;
}

void AudioGain::setGainDb(float gainDb) {
    // Convert decibels to linear: gain = 10^(dB/20)
    gain = powf(10.0f, gainDb / 20.0f);
}

void AudioGain::process(AudioBuffer* block) {
    if (block == nullptr) return;
    
    // Apply gain to all channels and samples in-place
    AudioBuffer* outputBlock = getBuffer();
    if (outputBlock == nullptr) {
        return;
    }
    for (int channel = 0; channel < AUDIO_CHANNELS; channel++) {
        for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
            outputBlock->data[channel][i] = block->data[channel][i] * gain;
        }
    }
    
    // Pass the modified block to next component
    transmit(outputBlock);
    release(outputBlock);
}
