#include "test_tone.h"
#include "base.h"
#include <cmath>

void TestTone::process(AudioBuffer* buffer) {
    AudioBuffer* outputBlock = AudioBufferPool::getInstance().getBuffer();

    for (size_t i = 0; i < AUDIO_BLOCK_SAMPLES; ++i) {
        for (size_t ch = 0; ch < AUDIO_CHANNELS; ++ch) {
            outputBlock->data[ch][i] = std::sin(phase);
        }
        phase += phaseIncrement;
        if (phase >= 2.0 * M_PI) {
            phase -= 2.0 * M_PI;
        }
    }
    transmit(outputBlock);
    release(outputBlock);
}