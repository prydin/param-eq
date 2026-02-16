#include "audio_rms.h"
#include <cmath>

void AudioRMS::process(AudioBuffer* block) {
    double sumLeft = 0.0;
    double sumRight = 0.0;

    for (size_t i = 0; i < AUDIO_BLOCK_SAMPLES; ++i) {
        sumLeft += block->data[0][i] * block->data[0][i];
        sumRight += block->data[1][i] * block->data[1][i];
    }

    rmsLeft = sqrt(sumLeft / AUDIO_BLOCK_SAMPLES);
    rmsRight = sqrt(sumRight / AUDIO_BLOCK_SAMPLES);

    transmit(block);
}