#ifndef PIPELINE_H
#define PIPELINE_H

#include <cstdint>
#include <Arduino.h>
#include "audiocomponent.h"

// Processes audio block data.
// inputs: array of input channel buffers
// outputs: array of output channel buffers
void processAudio(int32_t **inputs, int32_t **outputs);

#endif // PIPELINE_H
