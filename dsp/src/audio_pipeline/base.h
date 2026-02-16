#ifndef BASE_H
#define BASE_H
#include <Teensy4i2s.h>
#define AUDIO_CHANNELS 2
#define BUFFER_POOL_SIZE 64

// #define USE_DOUBLE_SAMPLES

#ifdef USE_DOUBLE_SAMPLES
typedef double sample_t;
#else
typedef float sample_t;
#endif
#endif // BASE_H
 