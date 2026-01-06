#ifndef AUDIO_CONTROLLER_H
#define AUDIO_CONTROLLER_H
#include <Teensy4i2s.h>
#include "audio_component.h"

class AudioController : public AudioComponent {
public:
    void process(AudioBuffer* block) override;

    static void processAudio(int32_t **inputs, int32_t **outputs);

    static AudioController* getInstance() {
        static AudioController instance;
        return &instance;
    }   

    static uint32_t getSampleRate() {
        Serial.println("AudioController::getSampleRate called");
        return audioInputI2S ? audioInputI2S->getMeasuredSampleRate() : 44100;
    }
private:
    AudioController();
    int32_t outputs[AUDIO_CHANNELS][AUDIO_BLOCK_SAMPLES];
};

#endif // AUDIO_CONTROLLER_H