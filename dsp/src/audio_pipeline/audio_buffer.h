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
#ifndef AUDIO_BUFFER_H
#define AUDIO_BUFFER_H

#include <stdlib.h>
#include <stdio.h>
#include "base.h"

class AudioBuffer
{
public:
    sample_t data[AUDIO_CHANNELS][AUDIO_BLOCK_SAMPLES];
    AudioBuffer *next;
    int refCount;

    void grab()
    {
        refCount++;
    }

    void release();
};

class AudioBufferPool
{
public:
    friend class AudioBuffer;
    AudioBufferPool(size_t poolSize);
    ~AudioBufferPool()
    {
        delete[] pool;
    }

    AudioBuffer *getBuffer();
    AudioBuffer *clone(AudioBuffer *source);
    static AudioBufferPool &getInstance()
    {
        static AudioBufferPool instance(BUFFER_POOL_SIZE);
        return instance;
    }

protected:
    void releaseBuffer(AudioBuffer *buffer);

private:
    AudioBuffer *pool;
    AudioBuffer *freeList;
    size_t poolSize;
};

inline void AudioBuffer::release()
{
    if(refCount <= 0)
    {
        printf("PANIC: AudioBuffer: release() called on buffer with refCount <= 0\n");
        return;
    }
    if(--refCount == 0)
    {
        AudioBufferPool::getInstance().releaseBuffer(this);
    }
}

#endif // AUDIO_BUFFER_H