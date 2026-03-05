#include "audio_peak.h"
 
void AudioPeak::process(AudioBuffer* block) {
        if (!block) {
            return;
        }
        float maxSample = 0.0f;
        for (size_t ch = 0; ch < AUDIO_CHANNELS; ++ch) {
            for (size_t i = 0; i < AUDIO_BLOCK_SAMPLES; ++i) {
                maxSample = std::max(maxSample, std::abs(block->data[ch][i]));
            }
        }
        peak = maxSample;
        transmit(block);
    }