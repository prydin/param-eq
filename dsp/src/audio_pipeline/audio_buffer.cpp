#include "audio_buffer.h"

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
        Serial.println("PANIC: AudioBufferPool: No free buffers available!");
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
