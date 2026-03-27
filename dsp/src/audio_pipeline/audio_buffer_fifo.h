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
#ifndef AUDIO_BUFFER_FIFO_H
#define AUDIO_BUFFER_FIFO_H

#include <stddef.h>
#include <stdint.h>
#include "audio_buffer.h"

struct AudioBufferFifoStats
{
    size_t level;
    size_t highWatermark;
    size_t capacity;
    uint32_t overruns;
    uint32_t underruns;
};

class AudioBufferFifo
{
public:
    explicit AudioBufferFifo(size_t capacity);
    ~AudioBufferFifo();

    // Returns false on overrun and leaves the FIFO unchanged.
    bool push(AudioBuffer *buffer);

    // Returns the next queued buffer, or a zero-filled buffer on underrun.
    AudioBuffer *pop();

    // Returns the next queued buffer, or nullptr if FIFO is empty.
    AudioBuffer *tryPop();

    void clear();
    void getStats(AudioBufferFifoStats *stats) const;
    void resetStats();

    size_t size() const { return count; }
    size_t maxSize() const { return capacity; }
    bool empty() const { return count == 0; }
    bool full() const { return count == capacity; }

private:
    AudioBuffer *allocateZeroBuffer() const;

    AudioBuffer **slots;
    size_t capacity;
    size_t head;
    size_t tail;
    size_t count;
    size_t highWatermark;
    uint32_t overruns;
    uint32_t underruns;
};

#endif // AUDIO_BUFFER_FIFO_H
