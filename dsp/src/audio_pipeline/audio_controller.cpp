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
#include <math.h>
#include <stdint.h>
#include "audio_controller.h"
#include "netconv.h"

uint32_t AudioController::sampleRate;

namespace
{
uint8_t g_inputShiftBits = 0;
bool g_inputAlignmentLocked = false;

uint8_t detectInputShiftBits(int32_t **inputs)
{
    if (inputs == nullptr)
    {
        return 0;
    }

    bool low8AlwaysZero = true;
    bool low16AlwaysZero = true;
    bool top8AlwaysSignExtended = true;
    bool top16AlwaysSignExtended = true;
    uint32_t nonZeroSamples = 0;

    for (int ch = 0; ch < AUDIO_CHANNELS; ch++)
    {
        for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++)
        {
            int32_t sample = inputs[ch][i];
            if (sample == 0)
            {
                continue;
            }

            nonZeroSamples++;
            uint32_t u = static_cast<uint32_t>(sample);
            uint8_t sign8 = sample < 0 ? 0xFFu : 0x00u;
            uint16_t sign16 = sample < 0 ? 0xFFFFu : 0x0000u;

            if ((u & 0x000000FFu) != 0)
            {
                low8AlwaysZero = false;
            }
            if ((u & 0x0000FFFFu) != 0)
            {
                low16AlwaysZero = false;
            }
            if (((u >> 24) & 0xFFu) != sign8)
            {
                top8AlwaysSignExtended = false;
            }
            if (((u >> 16) & 0xFFFFu) != sign16)
            {
                top16AlwaysSignExtended = false;
            }
        }
    }

    // Not enough information yet (near-silence), keep current behavior.
    if (nonZeroSamples < 16)
    {
        return 0;
    }

    // Right-aligned 16-bit PCM in 32-bit words.
    if (top16AlwaysSignExtended && !low16AlwaysZero)
    {
        return 16;
    }

    // Right-aligned 24-bit PCM in 32-bit words.
    if (top8AlwaysSignExtended && !low8AlwaysZero)
    {
        return 8;
    }

    // Already full-scale 32-bit or left-aligned 24-bit.
    return 0;
}

inline int32_t applyInputShift(int32_t sample)
{
    if (g_inputShiftBits == 0)
    {
        return sample;
    }
    return static_cast<int32_t>(static_cast<uint32_t>(sample) << g_inputShiftBits);
}
}

AudioController::AudioController() : AudioComponent() {
    // This is OK since we're a singleton
    i2sAudioCallback = AudioController::processAudio;
}

void AudioController::process(AudioBuffer* block) {
    // Running the audio chain with unstable sample rate can cause weird behavior.
    if(!getInstance()->enabled || !isSampleRateStable()) {
        return;
    }
    // This is the final destination - convert samples to int32 for output
    static bool wasClipped = false;
    bool clipped = false;
    processCount++;
    for(int ch = 0; ch < AUDIO_CHANNELS; ch++) {
        for(int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
            sample_t sample  = block->data[ch][i];
            clipped |= (sample > 1.0f) || (sample < -1.0f);
            outputs[ch][i] = (int32_t) (max(-1.0f, min(1.0f, sample)) * 2147483647.0f);
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
    // Running the audio chain with unstable sample rate can cause weird behavior.
    if(!getInstance()->enabled || !isSampleRateStable() || inputs == nullptr) {
        for(int ch = 0; ch < AUDIO_CHANNELS; ch++) {
            for(int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
                outputs[ch][i] = 0;
            }
        }
        Serial.print("0");
        return;
    }

    if (!g_inputAlignmentLocked)
    {
        g_inputShiftBits = detectInputShiftBits(inputs);
        if (g_inputShiftBits != 0)
        {
            g_inputAlignmentLocked = true;
            Serial.printf("Detected right-aligned PCM input, applying %u-bit left shift.\n", g_inputShiftBits);
        }
    }

    AudioBuffer* buffer = AudioBufferPool::getInstance().getBuffer();
    if (buffer == nullptr)
    {
        for(int ch = 0; ch < AUDIO_CHANNELS; ch++) {
            for(int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
                outputs[ch][i] = 0;
            }
        }
        return;
    }
    
    // Convert inputs from int32 to float
    for(int ch = 0; ch < AUDIO_CHANNELS; ch++) {
        for(int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
            int32_t aligned = applyInputShift(inputs[ch][i]);
            sample_t v = ((sample_t)aligned) / 2147483648.0f;
            buffer->data[ch][i] = v;
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
 