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
#include <Arduino.h>
#include "audio_controller.h"

uint32_t AudioController::sampleRate;

AudioController::AudioController() : AudioComponent()
{
    // This is OK since we're a singleton
    i2sAudioCallback = AudioController::audioCallback;
}

void AudioController::process(AudioBuffer *block)
{
    // Running the audio chain with unstable sample rate can cause weird behavior.
    if (!getInstance()->enabled || !isInstanceSampleRateStable())
    {
        return;
    }
    // This is the final destination - convert samples to int32 for output
    static bool wasClipped = false;
    bool clipped = false;
    for (int ch = 0; ch < AUDIO_CHANNELS; ch++)
    {
        for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++)
        {
            sample_t sample = block->data[ch][i];
            clipped |= (sample > 1.0f) || (sample < -1.0f);
            outputs[ch][i] = (int32_t)(max(-1.0f, min(1.0f, sample)) * 2147483647.0f);
        }
    }
    if (clipped && !wasClipped)
    {
        if (clipDetector)
        {
            clipDetector(true);
        }
        wasClipped = true;
    }
    else if (!clipped)
    {
        if (wasClipped && clipDetector)
        {
            clipDetector(false);
        }
        wasClipped = false;
    }
}

void AudioController::audioCallback(int32_t **inputs, int32_t **outputs)
{
    getInstance()->processAudio(inputs, outputs);
}

void AudioController::syncSinkSampleRate()
{
    if (!enabled || source == nullptr || sink == nullptr || !source->isSampleRateStable())
    {
        return;
    }

    uint32_t targetRate = source->getStandardizedSampleRate();
    if (targetRate == 0 || targetRate == sink->getSampleRate())
    {
        return;
    }

    sink->setSampleRate(targetRate);
    sampleRate = targetRate;
}

void AudioController::processAudio(int32_t **in, int32_t **out)
{
    // Running the audio chain with unstable sample rate can cause weird behavior.
    if (!enabled || source == nullptr || !isInstanceSampleRateStable())
    {
        for (int ch = 0; ch < AUDIO_CHANNELS; ch++)
        {
            for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++)
            {
                out[ch][i] = 0;
            }
        }
        return;
    }

    AudioBuffer *buffer = getBuffer();
    source->process(buffer);

    // Copy the processed output to the output buffers
    for (int ch = 0; ch < AUDIO_CHANNELS; ch++)
    {
        for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++)
        {
            out[ch][i] = this->outputs[ch][i];
        }
    }
    release(buffer);
}
