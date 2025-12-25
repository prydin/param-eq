#include "audio_controller.h"

AudioController::AudioController() : AudioComponent() {
}

void AudioController::process(AudioBuffer* block) {
    transmit(block);
}
void AudioController::processAudio(int32_t **inputs, int32_t **outputs)
{
    
}