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
#include "audio_component.h"

namespace
{
constexpr uint8_t kTimingMaxDepth = 16;

struct TimingFrame
{
    uint32_t childMicros = 0;
};

static TimingFrame s_timingStack[kTimingMaxDepth];
static uint8_t s_timingDepth = 0;
}

std::vector<AudioComponent*>& AudioComponent::registry()
{
    static std::vector<AudioComponent*> components;
    return components;
}

AudioComponent::AudioComponent()
{
    registry().push_back(this);
}

void AudioComponent::resetProcessTiming()
{
    timingAvgMicros = 0.0f;
    timingPeakMicros = 0.0f;
    timingCalls = 0;
}

void AudioComponent::processMeasured(AudioBuffer *block)
{
    const uint32_t startMicros = micros();

    uint8_t myDepth = s_timingDepth;
    bool stackTracked = false;
    if (myDepth < kTimingMaxDepth)
    {
        s_timingStack[myDepth].childMicros = 0;
        s_timingDepth = myDepth + 1;
        stackTracked = true;
    }

    process(block);

    const uint32_t totalMicros = micros() - startMicros;
    uint32_t childMicros = 0;
    if (stackTracked)
    {
        childMicros = s_timingStack[myDepth].childMicros;
        s_timingDepth = myDepth;
    }

    const uint32_t selfMicros = (totalMicros > childMicros) ? (totalMicros - childMicros) : 0;

    timingCalls++;
    if (timingCalls == 1)
    {
        timingAvgMicros = static_cast<float>(selfMicros);
    }
    else
    {
        // EWMA keeps this stable and cheap to compute in ISR context.
        timingAvgMicros = timingAvgMicros * 0.99f + static_cast<float>(selfMicros) * 0.01f;
    }
    if (static_cast<float>(selfMicros) > timingPeakMicros)
    {
        timingPeakMicros = static_cast<float>(selfMicros);
    }

    // Accumulate total child runtime into the parent frame for exclusive attribution.
    if (stackTracked && myDepth > 0)
    {
        s_timingStack[myDepth - 1].childMicros += totalMicros;
    }
}

void AudioComponent::printTimingReport()
{
    Serial.println("----- Component Timing (self us) -----");
    for (AudioComponent *component : registry())
    {
        if (component == nullptr)
        {
            continue;
        }
        Serial.printf("%s @%p calls=%lu avg=%.2f peak=%.2f\n",
                      component->getTimingName(),
                      component,
                      (unsigned long)component->getProcessCallCount(),
                      component->getProcessAverageMicros(),
                      component->getProcessPeakMicros());
    }
}

void AudioComponent::transmit(AudioBuffer *block)
{
    for (auto recipient : outputs)
    {
        block->grab();
        recipient->processMeasured(block);
        release(block);
    }
}