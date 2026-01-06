#include "audio_component.h"

void AudioComponent::transmit(AudioBuffer *block)
{
    for (auto recipient : outputs)
    {
        block->grab();
        recipient->process(block);
        release(block);
    }
}