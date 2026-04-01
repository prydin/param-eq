/* Audio Library for Teensy 3.X
 * Copyright (c) 2014, Paul Stoffregen, paul@pjrc.com
 *
 * Development of this audio library was funded by PJRC.COM, LLC by sales of
 * Teensy and Audio Adaptor boards.  Please support PJRC's efforts to develop
 * open source software by purchasing Teensy or other PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "AudioConfig.h"
#include "input_i2s.h"
#include "output_i2s.h"

#define REQUIRED_STABLE_INTERVALS 10
#define SAMPLE_RATE_COUNT_WINDOW_US 200000 // reduced from 1s to 200ms per request

// set up two flip-flopped buffers, one is used for queueing up data for processing, the other receives data from I2S codec
static int32_t dataL[AUDIO_BLOCK_SAMPLES*2] = {0};
static int32_t dataR[AUDIO_BLOCK_SAMPLES*2] = {0};
static int32_t* bufferL[2] = { &dataL[0], &dataL[AUDIO_BLOCK_SAMPLES] };
static int32_t* bufferR[2] = { &dataR[0], &dataR[AUDIO_BLOCK_SAMPLES] };
static volatile uint8_t writeBufferIndex = 0;
static volatile uint8_t readyBufferIndex = 0;
volatile uint32_t AudioInputI2S::interruptIntervalMicros = 1; // initialized to 1 to avoid division by zero on first read
volatile uint32_t AudioInputI2S::numStableIntervals = 0;
volatile uint32_t AudioInputI2S::lastTimeStamp = 0;
static volatile uint32_t countWindowStartMicros = 0;
static volatile uint32_t countWindowInterrupts = 0;

// Common sample rates for rounding to the neastest standard rate
uint32_t standardSampleRates[] {
	8000, 11025, 12000, 16000, 22050, 24000,
	32000, 44100, 48000, 88200, 96000, 176400,
	192000, 384000
};

DMAMEM __attribute__((aligned(32))) static uint64_t i2s_rx_buffer[AUDIO_BLOCK_SAMPLES*2];
DMAChannel AudioInputI2S::dma(false);
int32_t* outBuffers[2]; // temporary holder for the values returned by getData

void AudioInputI2S::begin()
{
	dma.begin(true); // Allocate the DMA channel first

	CORE_PIN8_CONFIG  = 3;  //1:RX_DATA0
	IOMUXC_SAI1_RX_DATA0_SELECT_INPUT = 2;

	dma.TCD->SADDR = (void *)((uint32_t)&I2S1_RDR0 + 0); // source address, read from 0 byte offset as we want the full 32 bits
	dma.TCD->SOFF = 0; // how many bytes to jump from current address on the next move. We're always reading the same register so no jump.
	dma.TCD->ATTR = DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_DSIZE(2); // 1=16bits, 2=32 bits. size of source, size of dest
	dma.TCD->NBYTES_MLNO = 4; // number of bytes to move, minor loop.
	dma.TCD->SLAST = 0; // how many bytes to jump when hitting the end of the major loop. In this case, no change to the source address.
	dma.TCD->DADDR = i2s_rx_buffer; // Destination address.
	dma.TCD->DOFF = 4; // how many bytes to move the destination at each minor loop. jump 4 bytes.
	dma.TCD->CITER_ELINKNO = sizeof(i2s_rx_buffer) / 4; // how many iterations are in the major loop
	dma.TCD->DLASTSGA = -sizeof(i2s_rx_buffer); // how many bytes to jump the destination address at the end of the major loop
	dma.TCD->BITER_ELINKNO = sizeof(i2s_rx_buffer) / 4; // beginning iteration count
	dma.TCD->CSR = DMA_TCD_CSR_INTHALF | DMA_TCD_CSR_INTMAJOR; // Tells the DMA mechanism to trigger interrupt at half and full population of the buffer
	dma.triggerAtHardwareEvent(DMAMUX_SOURCE_SAI1_RX); // run DMA at hardware event when new I2S data transmitted.

	// Enabled transmitting and receiving
	I2S1_RCSR = I2S_RCSR_RE | I2S_RCSR_BCE | I2S_RCSR_FRDE | I2S_RCSR_FR;

	dma.enable();
	dma.attachInterrupt(isr);
}

int32_t** AudioInputI2S::getData()
{
	uint8_t index = readyBufferIndex;
	outBuffers[0] = bufferL[index];
	outBuffers[1] = bufferR[index];
	return outBuffers;
}

void AudioInputI2S::isr(void)
{
	uint32_t currentTime = micros();
	uint32_t rawInterval = currentTime - lastTimeStamp;
	lastTimeStamp = currentTime;

	// Test estimator: count ISR events over a 1 second window.
	// Each ISR handles AUDIO_BLOCK_SAMPLES frames, so:
	// sampleRate ~= interruptsPerSecond * AUDIO_BLOCK_SAMPLES
	if (rawInterval > 0 && rawInterval < 100000)
	{
		if (countWindowStartMicros == 0)
		{
			countWindowStartMicros = currentTime;
			countWindowInterrupts = 0;
		}

		countWindowInterrupts++;

		uint32_t elapsed = currentTime - countWindowStartMicros;
		if (elapsed >= SAMPLE_RATE_COUNT_WINDOW_US)
		{
			uint32_t measuredRate = 0;
			if (countWindowInterrupts > 0 && elapsed > 0)
			{
				uint64_t numerator = (uint64_t)countWindowInterrupts * (uint64_t)AUDIO_BLOCK_SAMPLES * 1000000ULL;
				measuredRate = (uint32_t)((numerator + (elapsed / 2U)) / elapsed);
			}

			if (measuredRate > 0)
			{
				numStableIntervals = REQUIRED_STABLE_INTERVALS;
				interruptIntervalMicros = (AUDIO_BLOCK_SAMPLES * 1000000U + (measuredRate / 2U)) / measuredRate;
			}
			else
			{
				numStableIntervals = 0;
			}

			// Start a fresh counting window.
			countWindowStartMicros = currentTime;
			countWindowInterrupts = 0;
		}
	}
	
	uint32_t daddr;
	const int32_t *src;
	int32_t *dest_left, *dest_right;
	daddr = (uint32_t)(dma.TCD->DADDR);
	dma.clearInterrupt();

	if (daddr < (uint32_t)i2s_rx_buffer + sizeof(i2s_rx_buffer) / 2) 
	{
		// DMA is receiving to the first half of the buffer
		// need to remove data from the second half
		src = (int32_t *)&i2s_rx_buffer[AUDIO_BLOCK_SAMPLES];
	} 
	else 
	{
		// DMA is receiving to the second half of the buffer
		// need to remove data from the first half
		src = (int32_t *)&i2s_rx_buffer[0];
	}

	// Invalidate cache for the DMA-written half before reading it.
	arm_dcache_delete((void*)src, sizeof(i2s_rx_buffer) / 2);

	uint8_t index = writeBufferIndex;
	dest_left = bufferL[index];
	dest_right = bufferR[index];
	
	for (size_t i = 0; i < AUDIO_BLOCK_SAMPLES; i++)
	{
		dest_left[i] = src[2*i];
		dest_right[i] = src[2*i+1];
	}

	readyBufferIndex = index;
	writeBufferIndex = index == 0 ? 1 : 0;
}

void AudioInputI2Sslave::begin(void)
{
	dma.begin(true); // Allocate the DMA channel first

	AudioOutputI2Sslave::config_i2s();

	CORE_PIN8_CONFIG  = 3;  //1:RX_DATA0
	IOMUXC_SAI1_RX_DATA0_SELECT_INPUT = 2;

	// 32-bit DMA reads must use aligned RDR0 address.
	dma.TCD->SADDR = (void *)((uint32_t)&I2S1_RDR0 + 0);
	dma.TCD->SOFF = 0;
	dma.TCD->ATTR = DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_DSIZE(2);
	dma.TCD->NBYTES_MLNO = 4;
	dma.TCD->SLAST = 0;
	dma.TCD->DADDR = i2s_rx_buffer;
	dma.TCD->DOFF = 4;
	dma.TCD->CITER_ELINKNO = sizeof(i2s_rx_buffer) / 4;
	dma.TCD->DLASTSGA = -sizeof(i2s_rx_buffer);
	dma.TCD->BITER_ELINKNO = sizeof(i2s_rx_buffer) / 4;
	dma.TCD->CSR = DMA_TCD_CSR_INTHALF | DMA_TCD_CSR_INTMAJOR;
	dma.triggerAtHardwareEvent(DMAMUX_SOURCE_SAI1_RX);   
	dma.attachInterrupt(isr);
  
	I2S1_RCSR = 0;
	I2S1_RCSR = I2S_RCSR_RE | I2S_RCSR_BCE | I2S_RCSR_FRDE | I2S_RCSR_FR;

	dma.enable();
}

uint32_t AudioInputI2Sslave::getSampleRate(void)
{
	// Return the default sample rate if interruptIntervalMicros is zero to avoid division by zero
	return interruptIntervalMicros > 0 ? (AUDIO_BLOCK_SAMPLES * 1000000) / interruptIntervalMicros : SAMPLERATE;
}

bool AudioInputI2Sslave::isSampleRateStable(void) 
{
	// Consider sample rate stable if the interrupt interval has been stable for more than 10 intervals and 
	// the last interval is within 1s of the current time
	u_int32_t currentTime = micros();
	int32_t t = currentTime > lastTimeStamp ? currentTime - lastTimeStamp : 0;
	return numStableIntervals >= REQUIRED_STABLE_INTERVALS && t < 100000;
 }

 uint32_t AudioInputI2Sslave::getStandardizedSampleRate(void)
 {
	uint32_t measuredRate = getSampleRate();
	uint32_t closestRate = -1;
	for (size_t i = 1; i < sizeof(standardSampleRates) / sizeof(standardSampleRates[0]); i++) {
		closestRate = standardSampleRates[i];
		if(standardSampleRates[i] > measuredRate ) {
			return (measuredRate - standardSampleRates[i-1]) < (standardSampleRates[i] - measuredRate) ? standardSampleRates[i-1] : standardSampleRates[i];
		}
	}
	return closestRate;
}