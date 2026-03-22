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
#include "audio_buffer_fifo.h"
#include <string.h>

AudioBufferFifo::AudioBufferFifo(size_t capacity)
    : slots(nullptr), capacity(capacity), head(0), tail(0), count(0), highWatermark(0), overruns(0), underruns(0)
{
    if (capacity > 0)
    {
        slots = new AudioBuffer *[capacity];
        memset(slots, 0, capacity * sizeof(AudioBuffer *));
    }
}

AudioBufferFifo::~AudioBufferFifo()
{
    clear();
    delete[] slots;
}

bool AudioBufferFifo::push(AudioBuffer *buffer)
{
    if (buffer == nullptr || capacity == 0 || count == capacity)
    {
        overruns++;
        return false;
    }

    slots[tail] = buffer;
    tail = (tail + 1) % capacity;
    count++;
    if (count > highWatermark)
    {
        highWatermark = count;
    }
    return true;
}

AudioBuffer *AudioBufferFifo::pop()
{
    if (count == 0)
    {
        underruns++;
        return allocateZeroBuffer();
    }

    return tryPop();
}

AudioBuffer *AudioBufferFifo::tryPop()
{
    if (count == 0)
    {
        underruns++;
        return nullptr;
    }

    AudioBuffer *buffer = slots[head];
    slots[head] = nullptr;
    head = (head + 1) % capacity;
    count--;

    if (buffer == nullptr)
    {
        return allocateZeroBuffer();
    }

    return buffer;
}

void AudioBufferFifo::clear()
{
    while (count > 0)
    {
        AudioBuffer *buffer = slots[head];
        if (buffer)
        {
            buffer->release();
        }
        slots[head] = nullptr;
        head = (head + 1) % capacity;
        count--;
    }

    head = 0;
    tail = 0;
}

void AudioBufferFifo::getStats(AudioBufferFifoStats *stats) const
{
    if (stats == nullptr)
    {
        return;
    }

    stats->level = count;
    stats->highWatermark = highWatermark;
    stats->capacity = capacity;
    stats->overruns = overruns;
    stats->underruns = underruns;
}

void AudioBufferFifo::resetStats()
{
    highWatermark = count;
    overruns = 0;
    underruns = 0;
}

AudioBuffer *AudioBufferFifo::allocateZeroBuffer() const
{
    AudioBuffer *buffer = AudioBufferPool::getInstance().getBuffer();
    if (buffer == nullptr)
    {
        return nullptr;
    }

      memset(buffer->data, 0, sizeof(buffer->data));
    return buffer;
}
