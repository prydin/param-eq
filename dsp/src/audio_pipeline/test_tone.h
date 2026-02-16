#ifndef TEST_TONE_H
#define TEST_TONE_H

#include "audio_component.h"
#include <cmath>

class TestTone : public AudioComponent {
public:
    TestTone(double frequency = 1000.0, double sampleRate = 48000.0) : phase(0.0), frequency_(frequency), sampleRate_(sampleRate) {
        phaseIncrement = 2.0 * M_PI * frequency_ / sampleRate_;
    } 
    void process(AudioBuffer* buffer) override;

private:
    double phase;
    double frequency_;
    double sampleRate_;
    double phaseIncrement;
};

#endif // TEST_TONE_H