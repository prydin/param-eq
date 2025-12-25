#ifndef AUDIO_SQUARE_WAVE_H
#define AUDIO_SQUARE_WAVE_H

#include "audio_component.h"
#include <cmath>

class AudioSquareWave : public AudioComponent {
public:
    AudioSquareWave();
    
    // Set the frequency of the square wave in Hz
    void setFrequency(float frequencyHz);
    
    // Set the amplitude (0.0 to 1.0)
    void setAmplitude(float amplitude);
    
    // Generate square wave audio
    void process(AudioBuffer* block) override;
    
private:
    float frequency;      // Frequency in Hz
    float amplitude;      // Amplitude (0.0 to 1.0)
    float phase;          // Current phase (0.0 to 1.0)
    float sampleRate;     // Sample rate from Teensy4i2s
};

#endif // AUDIO_SQUARE_WAVE_H
