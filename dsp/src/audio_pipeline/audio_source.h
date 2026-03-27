#include "audio_component.h"

#pragma once

class AudioSource : public AudioComponent {
public:
    AudioSource() : AudioComponent() {}
    virtual uint32_t getSampleRate() { return 44100; } // TODO: implement this
    virtual uint32_t getStandardizedSampleRate() { return 44100; } // TODO: implement this
    virtual bool isSampleRateStable() { return true; } // TODO: implement this
};
