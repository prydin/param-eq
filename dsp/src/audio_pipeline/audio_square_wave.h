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
#ifndef AUDIO_SQUARE_WAVE_H
#define AUDIO_SQUARE_WAVE_H

#include "audio_component.h"
#include <cmath>

class AudioSquareWave : public AudioComponent {
public:
    AudioSquareWave();
    
    // Set the frequency of the square wave in Hz
    void setFrequency(float frequencyHz);
    
    // Set the amplitude (0.0 to 1.0)
    void setAmplitude(float amplitude);
    
    // Generate square wave audio
    void process(AudioBuffer* block) override;
    
private:
    float frequency;      // Frequency in Hz
    float amplitude;      // Amplitude (0.0 to 1.0)
    float phase;          // Current phase (0.0 to 1.0)
    float sampleRate;     // Sample rate from Teensy4i2s
    float phaseIncrement; // Phase increment per sample
};

#endif // AUDIO_SQUARE_WAVE_H
