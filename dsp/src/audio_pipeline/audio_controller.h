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
#include <Teensy4i2s.h>
#include "audio_component.h"

class AudioController : public AudioComponent {
public:
    void process(AudioBuffer* block) override;

    static void processAudio(int32_t **inputs, int32_t **outputs);

    static AudioController* getInstance() {
        static AudioController instance;
        return &instance;
    }   

    void setClipDetector(void (*detector)(bool clipped)) {
        getInstance()->clipDetector = detector;
    }

    void setSampleRate(uint32_t rate) {
        getInstance()->sampleRate = rate;
    }

    static uint32_t getSampleRate() {
        return sampleRate ? sampleRate : (audioInputI2S ? audioInputI2S->getSampleRate() : 44100);
    }

    static uint32_t getStandardizedSampleRate() {
        return audioInputI2S ? audioInputI2S->getStandardizedSampleRate() : 44100;
    }
    static bool isSampleRateStable() {
        return audioInputI2S ? audioInputI2S->isSampleRateStable() : false;
    }

    static uint32_t getNumStableIntervals() {
        return audioInputI2S ? audioInputI2S->getNumStableIntervals() : 0;
    }

    static uint32_t getProcessCount() {
        return getInstance()->processCount;
    }

    void enable(bool en) {
        enabled = en;
    }

private:
    AudioController();
    int32_t outputs[AUDIO_CHANNELS][AUDIO_BLOCK_SAMPLES];
    void (*clipDetector)(bool clipped) = nullptr;
    bool enabled = true;
    uint32_t processCount = 0;
    static uint32_t sampleRate;
};

#endif // AUDIO_CONTROLLER_H 