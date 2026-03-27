// Copyright (c) 2026 Pontus Rydin
// SPDX-License-Identifier: MIT
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
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
            float in = block->data[channel][i];
            float out = in * gain;
            outputBlock->data[channel][i] = out;
        }
    }
    
    // Pass the modified block to next component
    transmit(outputBlock);
    release(outputBlock);
}
