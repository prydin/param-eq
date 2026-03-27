#pragma once

#include <stdint.h>

#define SAMPLERATE 44100 // TODO: implement sample rate changes and make this dynamic
#define BIT_DEPTH 24
#define AUDIO_BLOCK_SAMPLES 64

#define SAMPLE_16_MAX INT16_MAX
#define SAMPLE_16_MIN INT16_MIN
#define SAMPLE_24_MAX 8388607 // 24 bit signed max
#define SAMPLE_24_MIN -8388608 // 24 bit signed min
#define SAMPLE_32_MAX INT32_MAX
#define SAMPLE_32_MIN INT32_MIN
