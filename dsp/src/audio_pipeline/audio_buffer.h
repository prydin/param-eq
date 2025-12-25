#ifndef AUDIO_BUFFER_H
#define AUDIO_BUFFER_H

#include <stdlib.h>
#include "base.h"

typedef float sample_t;

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

    void release()
    {
        if (--refCount == 0)
        {
            // Return to pool or free
        }
    }
};

class AudioBufferPool
{
public:
    AudioBufferPool(size_t poolSize);
    ~AudioBufferPool()
    {
        delete[] pool;
    }

    AudioBuffer *getBuffer();
    void releaseBuffer(AudioBuffer *buffer);
    static AudioBufferPool &getInstance(size_t poolSize = 16)
    {
        static AudioBufferPool instance(poolSize);
        return instance;
    }

private:
    AudioBuffer *pool;
    AudioBuffer *freeList;
    size_t poolSize;
};

#endif // AUDIO_BUFFER_H