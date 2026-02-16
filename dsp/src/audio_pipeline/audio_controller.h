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

    void setClipDetector(void (*detector)(bool clipped)) {
        getInstance()->clipDetector = detector;
    }

    void setSampleRate(uint32_t rate) {
        getInstance()->sampleRate = rate;
    }

    static uint32_t getSampleRate() {
        return sampleRate ? sampleRate : (audioInputI2S ? audioInputI2S->getSampleRate() : 44100);
    }

    static uint32_t getStandardizedSampleRate() {
        return audioInputI2S ? audioInputI2S->getStandardizedSampleRate() : 44100;
    }
    static bool isSampleRateStable() {
        return audioInputI2S ? audioInputI2S->isSampleRateStable() : false;
    }

private:
    AudioController();
    int32_t outputs[AUDIO_CHANNELS][AUDIO_BLOCK_SAMPLES];
    void (*clipDetector)(bool clipped) = nullptr;
    static uint32_t sampleRate;
};

#endif // AUDIO_CONTROLLER_H 