#ifndef AUDIO_CONTROLLER_H
#define AUDIO_CONTROLLER_H
#include <Teensy4i2s.h>
#include "audio_component.h"

class AudioController : public AudioComponent {
public:
    AudioController();
    void process(AudioBuffer* block) override;

    static void processAudio(int32_t **inputs, int32_t **outputs);
private:
};

#endif // AUDIO_CONTROLLER_H