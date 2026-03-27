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
#ifndef AUDIO_COMPONENT_H
#define AUDIO_COMPONENT_H
#include <stdlib.h>
#include <vector>
#include "audio_buffer.h"

#define AUDIO_CHANNELS 2

class AudioComponent {
public:
    AudioComponent() {};
    ~AudioComponent() {};
    virtual void process(AudioBuffer* block) {};
    void addReceiver(AudioComponent* recipient) {
        outputs.push_back(recipient);
    }
    AudioBuffer* clone(AudioBuffer* source) {
        return AudioBufferPool::getInstance().clone(source);
    }
    AudioBuffer* getBuffer() {
        return AudioBufferPool::getInstance().getBuffer();
    }   
    void release(AudioBuffer* block) {
        block->release();
    }

protected:  
    void transmit(AudioBuffer* block);

private:
    std::vector<AudioComponent*> outputs;
};
#endif // AUDIO_COMPONENT_H