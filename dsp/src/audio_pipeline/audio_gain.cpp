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
    // Process all channels and samples
    for (int channel = 0; channel < AUDIO_CHANNELS; channel++) {
        for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
            // Apply gain to each sample
            AudioBuffer* newBlock =  AudioBufferPool::getInstance().getBuffer();
            newBlock->data[channel][i] = (sample_t)(block->data[channel][i] * gain);
            transmit(newBlock);
            newBlock->release();
        }
    }
}
