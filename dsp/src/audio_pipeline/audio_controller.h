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
#ifndef AUDIO_CONTROLLER_H
#define AUDIO_CONTROLLER_H
#include "audio_i2s.h"
#include "audio_component.h"
#include "audio_source.h"

class AudioController : public AudioComponent
{
public:
    void process(AudioBuffer *block) override;

    static void audioCallback(int32_t **inputs, int32_t **outputs);

    void processAudio(int32_t **inputs, int32_t **outputs);
    void syncSinkSampleRate();

    static AudioController *getInstance()
    {
        static AudioController instance;
        return &instance;
    }

    void setClipDetector(void (*detector)(bool clipped))
    {
        getInstance()->clipDetector = detector;
    }

    void setSampleRate(uint32_t rate)
    {
        getInstance()->sampleRate = rate;
    }

    uint32_t getInstanceSampleRate()
    {
        return sampleRate ? sampleRate : (source ? source->getSampleRate() : 44100);
    }

    static uint32_t getSampleRate()
    {
        return getInstance()->getInstanceSampleRate();
    }

    uint32_t getInstanceStandardizedSampleRate()
    {
        return source ? source->getStandardizedSampleRate() : 44100;
    }

    static uint32_t getStandardizedSampleRate()
    {
        return getInstance()->getInstanceStandardizedSampleRate();
    }


    bool isInstanceSampleRateStable()
    {
        return source ? source->isSampleRateStable() : false;
    }

    static bool isSampleRateStable()
    {
        return getInstance()->isInstanceSampleRateStable();
    }

    void enable(bool en)
    {
        enabled = en;
    }

    void setSource(AudioSource *src)
    {
        source = src;
    }

    void setSink(AudioOutputI2S *snk)
    {
        sink = snk;
    }

private:
    AudioController();
    int32_t outputs[AUDIO_CHANNELS][AUDIO_BLOCK_SAMPLES];
    void (*clipDetector)(bool clipped) = nullptr;
    bool enabled = true;
    static uint32_t sampleRate;
    AudioSource *source = nullptr;
    AudioOutputI2S *sink = nullptr;
};

#endif // AUDIO_CONTROLLER_H