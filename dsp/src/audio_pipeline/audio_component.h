#ifndef AUDIO_COMPONENT_H
#define AUDIO_COMPONENT_H
#include <Teensy4i2s.h>
#include <stdlib.h>
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