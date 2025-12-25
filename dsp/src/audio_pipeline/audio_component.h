#ifndef AUDIO_COMPONENT_H
#define AUDIO_COMPONENT_H
#include <Teensy4i2s.h>
#include <stdlib.h>
#include "audio_buffer.h"

#define AUDIO_CHANNELS 2

class AudioComponent {
public:
    AudioComponent();
    virtual void process(AudioBuffer* block);
    void addRecipient(AudioComponent* recipient) {
        outputs.push_back(recipient);
    }

protected:  
    void transmit(AudioBuffer* block);

private:
    std::vector<AudioComponent*> outputs;
};
#endif // AUDIO_COMPONENT_H