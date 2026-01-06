#ifndef AUDIO_GAIN_H
#define AUDIO_GAIN_H

#include "audio_component.h"
#include <cmath>

class AudioGain : public AudioComponent {
public:
    AudioGain();
    
    // Set gain as a linear factor (1.0 = unity gain, 0.5 = half volume, 2.0 = double, etc.)
    void setGain(float linearGain);
    
    // Set gain in decibels (0 dB = unity gain, -6 dB = half volume, +6 dB ≈ double, etc.)
    void setGainDb(float gainDb);
    
    // Override the audio processing method
    void process(AudioBuffer* block) override;
    
private:
    float gain;  // Linear gain factor
};

#endif // AUDIO_GAIN_H
