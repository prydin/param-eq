#ifndef AUDIO_BUFFER_H
#define AUDIO_BUFFER_H

#include <stdlib.h>
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
        Serial.println("PANIC: AudioBuffer: release() called on buffer with refCount <= 0");
        return;
    }
    if(--refCount == 0)
    {
        AudioBufferPool::getInstance().releaseBuffer(this);
    }
}

#endif // AUDIO_BUFFER_H