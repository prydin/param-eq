#include <Arduino.h>

#define AUDIO_BLOCK_SAMPLES  128

typedef struct audio_block_int32_struct {
	uint8_t  ref_count;
	uint8_t  reserved1;
	uint16_t memory_pool_index;
	int32_t  data[AUDIO_BLOCK_SAMPLES];
} audio_block_int32_t;

typedef struct audio_block_float_struct {
	uint8_t  ref_count;
	uint8_t  reserved1;
	uint16_t memory_pool_index;
	float  data[AUDIO_BLOCK_SAMPLES];
} audio_block_float_t;


