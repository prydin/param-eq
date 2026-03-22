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
#include "audio_buffer.h"
#include <string.h>

AudioBufferPool::AudioBufferPool(size_t poolSize)
{
    this->poolSize = poolSize;
    pool = new AudioBuffer[poolSize];
    for (size_t i = 0; i < poolSize; i++)
    {
        pool[i].refCount = 0;
        pool[i].next = (i < poolSize - 1) ? &pool[i + 1] : nullptr;
    }
    freeList = &pool[0];
}

AudioBuffer *AudioBufferPool::getBuffer()
{
    if (freeList == nullptr)
    {
        printf("PANIC: AudioBufferPool: No free buffers available!\n");
        return nullptr; // No available buffer
    }
    AudioBuffer *buffer = freeList;
    freeList = freeList->next;
    buffer->refCount = 1;
    buffer->next = nullptr;
    return buffer;
}

AudioBuffer *AudioBufferPool::clone(AudioBuffer *source)
{
    AudioBuffer *newBuffer = getBuffer();
    if (newBuffer == nullptr)
    {
        return nullptr; // No available buffer
    }
    memcpy(newBuffer->data, source->data, sizeof(source->data));
    return newBuffer;
}

void AudioBufferPool::releaseBuffer(AudioBuffer *buffer)
{
    buffer->next = freeList;
    freeList = buffer;
}
