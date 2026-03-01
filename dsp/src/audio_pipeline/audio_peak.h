#ifndef AUDIO_PEAK_H
#define AUDIO_PEAK_H

#include <cmath>
#include <algorithm>
#include "audio_component.h"

class AudioPeak : public AudioComponent
{
public:
    AudioPeak() : peak(0.0f) {}

    virtual void process(AudioBuffer* block) override;

private:
    float peak;
};

#endif // AUDIO_PEAK_H