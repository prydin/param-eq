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
#include "audio_square_wave.h"
#include "audio_controller.h"

AudioSquareWave::AudioSquareWave()
    : frequency(440.0f),                           // Default to A4 (440 Hz)
      amplitude(0.5f),                             // Default to half amplitude
      phase(0.0f),                                 // Start at zero phase
      sampleRate(44100) // Get sample rate from AudioController
{
}
void AudioSquareWave::setFrequency(float frequencyHz)
{
    frequency = frequencyHz;
    phaseIncrement = frequency / sampleRate;
}

void AudioSquareWave::setAmplitude(float amp)
{
    // Clamp amplitude to valid range
    if (amp < 0.0f)
        amp = 0.0f;
    if (amp > 1.0f)
        amp = 1.0f;
    amplitude = amp;
}

void AudioSquareWave::process(AudioBuffer *block)
{
    // Square wave is a generator - ignore input and create new buffer
    AudioBuffer* outputBlock = AudioBufferPool::getInstance().getBuffer();
    if (outputBlock == nullptr) {
        return;
    }

    // Generate square wave for all channels and samples
    for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++)
    {
        // Square wave: high when phase < 0.5, low when phase >= 0.5
        sample_t value = phase < 0.5f ? amplitude : -amplitude;

        // Output to all channels
        for (int channel = 0; channel < AUDIO_CHANNELS; channel++)
        {
            outputBlock->data[channel][i] = value;
        }

        // Advance phase and wrap
        phase += phaseIncrement;
        // Serial.printf("Phase: %f\n", phase);
        if (phase >= 1.0f)
        {
            phase = 0.0f;
        }
    }
 
    // Transmit our generated buffer to connected components
    transmit(outputBlock);
    release(outputBlock);
}