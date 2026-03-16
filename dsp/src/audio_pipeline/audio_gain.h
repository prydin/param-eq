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
#ifndef AUDIO_GAIN_H
#define AUDIO_GAIN_H

#include "audio_component.h"
#include <cmath>

class AudioGain : public AudioComponent {
public:
    AudioGain();
    
    // Set gain as a linear factor (1.0 = unity gain, 0.5 = half volume, 2.0 = double, etc.)
    void setGain(float linearGain);
    
    // Set gain in decibels (0 dB = unity gain, -6 dB = half volume, +6 dB â‰ˆ double, etc.)
    void setGainDb(float gainDb);
    
    // Override the audio processing method
    void process(AudioBuffer* block) override;
    
private:
    float gain;  // Linear gain factor
};

#endif // AUDIO_GAIN_H
