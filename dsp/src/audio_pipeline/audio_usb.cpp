// Copyright (c) 2026 Pontus Rydin
// SPDX-License-Identifier: MIT
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <string.h>
#include <avr/pgmspace.h>
#include "usb_dev.h"
#include "audio_usb.h"
#include "debug/printf.h"
#include <Arduino.h>

#ifdef AUDIO_INTERFACE

AudioBufferFifo * AudioInputUSBEx::rx_fifo;
AudioBuffer * AudioInputUSBEx::incoming_buffer;
uint16_t AudioInputUSBEx::incoming_count;
uint8_t AudioInputUSBEx::receive_flag;
volatile uint32_t AudioInputUSBEx::measured_sample_rate = 44100;
volatile uint32_t AudioInputUSBEx::standardized_sample_rate = 44100;
volatile uint32_t AudioInputUSBEx::last_packet_time_us = 0;
volatile uint32_t AudioInputUSBEx::rate_window_start_us = 0;
volatile uint32_t AudioInputUSBEx::rate_window_samples = 0;
volatile uint32_t AudioInputUSBEx::stable_rate_windows = 0;
volatile uint32_t AudioInputUSBEx::feedback_holdoff_until_us = 0;

struct usb_audio_features_struct AudioInputUSBEx::features = {0,0,FEATURE_MAX_VOLUME/2};

extern volatile uint8_t usb_high_speed;
static void rx_event(transfer_t *t);
static void tx_event(transfer_t *t);

/*static*/ transfer_t rx_transfer __attribute__ ((used, aligned(32)));
/*static*/ transfer_t sync_transfer __attribute__ ((used, aligned(32)));
/*static*/ transfer_t tx_transfer __attribute__ ((used, aligned(32)));
DMAMEM static uint8_t rx_buffer[AUDIO_RX_SIZE] __attribute__ ((aligned(32)));
DMAMEM static uint8_t tx_buffer[AUDIO_RX_SIZE] __attribute__ ((aligned(32)));
DMAMEM uint32_t usb_audio_sync_feedback __attribute__ ((aligned(32)));

uint8_t usb_audio_receive_setting=0;
uint8_t usb_audio_transmit_setting=0;
uint8_t usb_audio_sync_nbytes;
uint8_t usb_audio_sync_rshift;

uint32_t feedback_accumulator;

volatile uint32_t usb_audio_underrun_count;
volatile uint32_t usb_audio_overrun_count;

static const uint32_t usb_standard_sample_rates[] = {
	8000, 11025, 12000, 16000, 22050, 24000,
	32000, 44100, 48000, 88200, 96000, 176400,
	192000, 384000
};

static const uint32_t USB_RATE_WINDOW_US = 50000;
static const uint32_t USB_RATE_TIMEOUT_US = 1000000;
static const uint32_t USB_RATE_STABLE_WINDOWS = 6;
static const uint32_t USB_FEEDBACK_HOLDOFF_US = 1000000;

uint32_t AudioInputUSBEx::standardizeSampleRate(uint32_t measured_rate)
{
	uint32_t closest_rate = usb_standard_sample_rates[0];
	for (size_t i = 1; i < sizeof(usb_standard_sample_rates) / sizeof(usb_standard_sample_rates[0]); i++) {
		closest_rate = usb_standard_sample_rates[i];
		if (usb_standard_sample_rates[i] >= measured_rate) {
			uint32_t lower = usb_standard_sample_rates[i - 1];
			return (measured_rate - lower) < (usb_standard_sample_rates[i] - measured_rate) ? lower : usb_standard_sample_rates[i];
		}
	}
	return closest_rate;
}

void AudioInputUSBEx::resetRateTracking(uint32_t now)
{
	last_packet_time_us = now;
	rate_window_start_us = now;
	rate_window_samples = 0;
	stable_rate_windows = 0;
}

void AudioInputUSBEx::startFeedbackHoldoff(uint32_t now)
{
	feedback_holdoff_until_us = now + USB_FEEDBACK_HOLDOFF_US;
}

bool AudioInputUSBEx::isFeedbackHoldoffActive(uint32_t now)
{
	uint32_t holdoff_until = feedback_holdoff_until_us;
	return holdoff_until != 0 && (int32_t)(holdoff_until - now) > 0;
}

void AudioInputUSBEx::updateRateTracking(uint32_t sample_frames)
{
	uint32_t now = micros();
	uint32_t last = last_packet_time_us;

	if (last == 0 || (now - last) > USB_RATE_TIMEOUT_US) {
		resetRateTracking(now);
		startFeedbackHoldoff(now);
	}
	last_packet_time_us = now;
	if (rate_window_start_us == 0) {
		rate_window_start_us = now;
	}
	rate_window_samples += sample_frames;

	uint32_t elapsed = now - rate_window_start_us;
	if (elapsed < USB_RATE_WINDOW_US) {
		return;
	}

	uint32_t measured_rate = (uint32_t)(((uint64_t)rate_window_samples * 1000000ULL) / elapsed);
	uint32_t standardized_rate = standardizeSampleRate(measured_rate);
	if (standardized_rate == standardized_sample_rate) {
		if (stable_rate_windows < USB_RATE_STABLE_WINDOWS) {
			stable_rate_windows++;
		}
	} else {
		stable_rate_windows = 1;
	}
	measured_sample_rate = measured_rate;
	standardized_sample_rate = standardized_rate;
	rate_window_start_us = now;
	rate_window_samples = 0;
}


static void rx_event(transfer_t *t)
{	
	if (t) {
		int len = AUDIO_RX_SIZE - ((rx_transfer.status >> 16) & 0x7FFF);
		usb_audio_receive_callback(len);
	}
	usb_prepare_transfer(&rx_transfer, rx_buffer, AUDIO_RX_SIZE, 0);
	arm_dcache_delete(&rx_buffer, AUDIO_RX_SIZE);
	usb_receive(AUDIO_RX_ENDPOINT, &rx_transfer);
}

static void sync_event(transfer_t *t)
{
	// USB 2.0 Specification, 5.12.4.2 Feedback, pages 73-75
	//printf("sync %x\n", sync_transfer.status); // too slow, can't print this much
	usb_audio_sync_feedback = feedback_accumulator >> usb_audio_sync_rshift;
	usb_prepare_transfer(&sync_transfer, &usb_audio_sync_feedback, usb_audio_sync_nbytes, 0);
	arm_dcache_flush(&usb_audio_sync_feedback, usb_audio_sync_nbytes);
	usb_transmit(AUDIO_SYNC_ENDPOINT, &sync_transfer);
}

void usb_audio_configure(void)
{
	printf("usb_audio_configure\n");
	usb_audio_underrun_count = 0;
	usb_audio_overrun_count = 0;
	feedback_accumulator = 739875226; // 44.1 * 2^24
	AudioInputUSBEx::feedback_holdoff_until_us = 0;
	if (usb_high_speed) {
		usb_audio_sync_nbytes = 4;
		usb_audio_sync_rshift = 8;
	} else {
		usb_audio_sync_nbytes = 3;
		usb_audio_sync_rshift = 10;
	}
	memset(&rx_transfer, 0, sizeof(rx_transfer));
	usb_config_rx_iso(AUDIO_RX_ENDPOINT, AUDIO_RX_SIZE, 1, rx_event);
	rx_event(NULL);
	memset(&sync_transfer, 0, sizeof(sync_transfer));
	usb_config_tx_iso(AUDIO_SYNC_ENDPOINT, usb_audio_sync_nbytes, 1, sync_event);
	sync_event(NULL);
	memset(&tx_transfer, 0, sizeof(tx_transfer));
	usb_config_tx_iso(AUDIO_TX_ENDPOINT, AUDIO_TX_SIZE, 1, tx_event);
	tx_event(NULL);
}

void AudioInputUSBEx::begin(void)
{
	if (rx_fifo == NULL) {
		rx_fifo = new AudioBufferFifo(AUDIO_USB_RX_FIFO_SIZE);
	}
	if (rx_fifo != NULL) {
		rx_fifo->clear();
	}
	if (incoming_buffer != NULL) {
		incoming_buffer->release();
		incoming_buffer = NULL;
	}

	incoming_count = 0;
	receive_flag = 0;
	measured_sample_rate = 44100;
	standardized_sample_rate = 44100;
	last_packet_time_us = 0;
	rate_window_start_us = 0;
	rate_window_samples = 0;
	stable_rate_windows = 0;
	feedback_holdoff_until_us = 0;
	// update_responsibility = update_setup();
	// TODO: update responsibility is tough, partly because the USB
	// interrupts aren't sychronous to the audio library block size,
	// but also because the PC may stop transmitting data, which
	// means we no longer get receive callbacks from usb.c
	// update_responsibility = false;
}

AudioBuffer *AudioInputUSBEx::getNextBlock(void)
{
	if (rx_fifo == NULL) {
		return NULL;
	}
	return rx_fifo->tryPop();
}

bool AudioInputUSBEx::getRxFifoStats(AudioBufferFifoStats *stats)
{
	if (rx_fifo == NULL || stats == NULL) {
		return false;
	}
	rx_fifo->getStats(stats);
	return true;
}

bool AudioInputUSBEx::isSampleRateStable()
{
	uint32_t last = last_packet_time_us;
	if (last == 0) {
		return false;
	}
	uint32_t age = micros() - last;
	return usb_audio_receive_setting != 0 && age < USB_RATE_TIMEOUT_US && stable_rate_windows >= USB_RATE_STABLE_WINDOWS;
}

uint32_t AudioInputUSBEx::getLastPacketAgeMicros()
{
	uint32_t last = last_packet_time_us;
	if (last == 0) {
		return 0xFFFFFFFFu;
	}
	return micros() - last;
}

static sample_t usb_sample_to_float(int32_t sample)
{
	// Convert 24-bit signed integer to float (-8388608 to 8388607 -> -1.0 to ~1.0)
	return static_cast<sample_t>(sample) / 8388608.0f;
}

static void copy_to_buffer(const uint8_t *src, AudioBuffer *buffer, unsigned int offset, unsigned int len)
{
	sample_t *left = buffer->data[LEFT_CHANNEL] + offset;
	sample_t *right = buffer->data[RIGHT_CHANNEL] + offset;

	while (len > 0) {
		// Unpack 24-bit little-endian samples (3 bytes each)
		int32_t left_sample = (int32_t)(src[0] | (src[1] << 8) | ((int8_t)src[2] << 16));
		int32_t right_sample = (int32_t)(src[3] | (src[4] << 8) | ((int8_t)src[5] << 16));
		*left++ = usb_sample_to_float(left_sample);
		*right++ = usb_sample_to_float(right_sample);
		src += 6;
		len--;
	}
}

// Called from the USB interrupt when an isochronous packet arrives
// we must completely remove it from the receive buffer before returning
//
#if 1
void usb_audio_receive_callback(unsigned int len)
{
	unsigned int count, avail;
	bool queued;
	AudioBuffer *buffer;
	const uint8_t *data;

	AudioInputUSBEx::receive_flag = 1;
	len /= 6; // 1 stereo sample = 6 bytes (2 channels × 3 bytes)
	AudioInputUSBEx::updateRateTracking(len);
	data = (const uint8_t *)rx_buffer;

	count = AudioInputUSBEx::incoming_count;
	if (AudioInputUSBEx::rx_fifo == NULL) {
		return;
	}
	buffer = AudioInputUSBEx::incoming_buffer;
	if (buffer == NULL) {
		buffer = AudioBufferPool::getInstance().getBuffer();
		if (buffer == NULL) return;
		AudioInputUSBEx::incoming_buffer = buffer;
	}
	while (len > 0) {
		avail = AUDIO_BLOCK_SAMPLES - count;
		if (avail == 0) {
			queued = AudioInputUSBEx::rx_fifo->push(buffer);
			if (!queued) {
				usb_audio_overrun_count++;
				printf("!");
				buffer->release();
			}
			buffer = AudioBufferPool::getInstance().getBuffer();
			if (buffer == NULL) {
				AudioInputUSBEx::incoming_buffer = NULL;
				AudioInputUSBEx::incoming_count = 0;
				return;
			}
			AudioInputUSBEx::incoming_buffer = buffer;
			count = 0;
			continue;
		}

		if (len < avail) {
			copy_to_buffer(data, buffer, count, len);
			count += len;
			len = 0;
			break;
		}

		copy_to_buffer(data, buffer, count, avail);
		data += avail * 6;
		len -= avail;
		count += avail;

		queued = AudioInputUSBEx::rx_fifo->push(buffer);
		if (!queued) {
			usb_audio_overrun_count++;
			printf("!");
			buffer->release();
		}

		buffer = AudioBufferPool::getInstance().getBuffer();
		if (buffer == NULL) {
			AudioInputUSBEx::incoming_buffer = NULL;
			AudioInputUSBEx::incoming_count = 0;
			return;
		}
		AudioInputUSBEx::incoming_buffer = buffer;
		count = 0;
	}
	AudioInputUSBEx::incoming_count = count;
}
#endif

void AudioInputUSBEx::process(AudioBuffer *block)
{
	AudioBuffer *buffer;

	__disable_irq();
	buffer = getNextBlock();
	uint16_t c = incoming_count;
	uint8_t f = receive_flag;
	uint32_t now = micros();
	bool feedback_holdoff = isFeedbackHoldoffActive(now);
	receive_flag = 0;
	__enable_irq();
	if (f && !feedback_holdoff) {
		int diff = AUDIO_BLOCK_SAMPLES/2 - (int)c;
		feedback_accumulator += diff * 1;
		//uint32_t feedback = (feedback_accumulator >> 8) + diff * 100;
		//usb_audio_sync_feedback = feedback;

		//printf(diff >= 0 ? "." : "^");
	}
	//serial_phex(c);
	//serial_print(".");  
	if (!buffer) {
		//printf("#"); // buffer underrun - PC sending too slow
		if (f && !feedback_holdoff)
		{
			feedback_accumulator += 3500;
			usb_audio_underrun_count++;
		}
		else if (f)
		{
			usb_audio_underrun_count++;
		}
	}
	if (buffer) {
		transmit(buffer);
		release(buffer);
	}
}

















#if 1
// bool AudioOutputUSBEx::update_responsibility;
AudioBufferFifo * AudioOutputUSBEx::tx_fifo;
AudioBuffer * AudioOutputUSBEx::active_buffer;
uint16_t AudioOutputUSBEx::active_offset;

/*DMAMEM*/ uint16_t usb_audio_transmit_buffer[AUDIO_TX_SIZE/2] __attribute__ ((used, aligned(32)));


static void tx_event(transfer_t *t)
{
	int len = usb_audio_transmit_callback();
	usb_audio_sync_feedback = feedback_accumulator >> usb_audio_sync_rshift;
	usb_prepare_transfer(&tx_transfer, usb_audio_transmit_buffer, len, 0);
	arm_dcache_flush_delete(usb_audio_transmit_buffer, len);
	usb_transmit(AUDIO_TX_ENDPOINT, &tx_transfer);
}


void AudioOutputUSBEx::begin(void)
{
	// update_responsibility = false;
	if (tx_fifo == NULL) {
		tx_fifo = new AudioBufferFifo(AUDIO_USB_TX_FIFO_SIZE);
	}
	if (tx_fifo != NULL) {
		tx_fifo->clear();
	}
	if (active_buffer != NULL) {
		active_buffer->release();
		active_buffer = NULL;
	}
	active_offset = 0;
}

bool AudioOutputUSBEx::getTxFifoStats(AudioBufferFifoStats *stats)
{
	if (tx_fifo == NULL || stats == NULL) {
		return false;
	}
	tx_fifo->getStats(stats);
	return true;
}

static int32_t float_to_usb_sample(sample_t sample)
{
	// Convert float to 24-bit signed integer (-1.0 to 1.0 -> -8388608 to 8388607)
	if (sample >= 1.0f) return 8388607;
	if (sample <= -1.0f) return -8388608;
	return (int32_t)(sample * 8388607.0f);
}

static void copy_from_buffer(uint8_t *dst, const AudioBuffer *buffer, unsigned int offset, unsigned int len)
{
	const sample_t *left = buffer->data[LEFT_CHANNEL] + offset;
	const sample_t *right = buffer->data[RIGHT_CHANNEL] + offset;

	while (len > 0) {
		// Pack 24-bit little-endian samples (3 bytes each)
		int32_t left_sample = float_to_usb_sample(*left++);
		int32_t right_sample = float_to_usb_sample(*right++);
		dst[0] = (uint8_t)(left_sample & 0xFF);
		dst[1] = (uint8_t)((left_sample >> 8) & 0xFF);
		dst[2] = (uint8_t)((left_sample >> 16) & 0xFF);
		dst[3] = (uint8_t)(right_sample & 0xFF);
		dst[4] = (uint8_t)((right_sample >> 8) & 0xFF);
		dst[5] = (uint8_t)((right_sample >> 16) & 0xFF);
		dst += 6;
		len--;
	}
}

void AudioOutputUSBEx::process(AudioBuffer *block)
{
	bool queued = false;

	if (block == NULL) {
		return;
	}
	if (tx_fifo == NULL) {
		return;
	}
	if (usb_audio_transmit_setting == 0) {
		__disable_irq();
		tx_fifo->clear();
		if (active_buffer != NULL) {
			active_buffer->release();
			active_buffer = NULL;
		}
		active_offset = 0;
		__enable_irq();
		return;
	}

	block->grab();
	__disable_irq();
	queued = tx_fifo->push(block);
	__enable_irq();
	if (!queued) {
		// FIFO overrun: ignore this request and keep existing queued buffers.
		block->release();
	}
}


// Called from the USB interrupt when ready to transmit another
// isochronous packet.  If we place data into the transmit buffer,
// the return is the number of bytes.  Otherwise, return 0 means
// no data to transmit
unsigned int usb_audio_transmit_callback(void)
{
	static uint32_t count=5;
	uint32_t avail, num, target, offset, len=0;
	AudioBuffer *buffer = NULL;

	if (++count < 10) {   // TODO: dynamic adjust to match USB rate
		target = 44;
	} else {
		count = 0;
		target = 45;
	}
	while (len < target) {
		num = target - len;
		if (AudioOutputUSBEx::active_buffer == NULL) {
			if (AudioOutputUSBEx::tx_fifo == NULL) {
				memset((uint8_t *)usb_audio_transmit_buffer + (len * 6), 0, num * 6);
				break;
			}
			AudioOutputUSBEx::active_offset = 0;
			AudioOutputUSBEx::active_buffer = AudioOutputUSBEx::tx_fifo->pop();
		}

		buffer = AudioOutputUSBEx::active_buffer;
		if (buffer == NULL) {
			// Out of pool buffers while handling underrun; output silence.
			memset((uint8_t *)usb_audio_transmit_buffer + (len * 6), 0, num * 6);
			break;
		}
		offset = AudioOutputUSBEx::active_offset;

		avail = AUDIO_BLOCK_SAMPLES - offset;
		if (num > avail) num = avail;

		copy_from_buffer((uint8_t *)usb_audio_transmit_buffer + (len * 6), buffer, offset, num);
		len += num;
		offset += num;
		if (offset >= AUDIO_BLOCK_SAMPLES) {
			buffer->release();
			AudioOutputUSBEx::active_buffer = NULL;
			AudioOutputUSBEx::active_offset = 0;
		} else {
			AudioOutputUSBEx::active_offset = offset;
		}
	}
	return target * 6;
}
#endif




struct setup_struct {
  union {
    struct {
	uint8_t bmRequestType;
	uint8_t bRequest;
	union {
		struct {
			uint8_t bChannel;  // 0=main, 1=left, 2=right
			uint8_t bCS;       // Control Selector
		};
		uint16_t wValue;
	};
	union {
		struct {
			uint8_t bIfEp;     // type of entity
			uint8_t bEntityId; // UnitID, TerminalID, etc.
		};
		uint16_t wIndex;
	};
	uint16_t wLength;
    };
  };
};

int usb_audio_get_feature(void *stp, uint8_t *data, uint32_t *datalen)
{
	struct setup_struct setup = *((struct setup_struct *)stp);
	if (setup.bmRequestType==0xA1) { // should check bRequest, bChannel, and UnitID
			if (setup.bCS==0x01) { // mute
				data[0] = AudioInputUSBEx::features.mute;  // 1=mute, 0=unmute
				*datalen = 1;
				return 1;
			}
			else if (setup.bCS==0x02) { // volume
				if (setup.bRequest==0x81) { // GET_CURR
					data[0] = AudioInputUSBEx::features.volume & 0xFF;
					data[1] = (AudioInputUSBEx::features.volume>>8) & 0xFF;
				}
				else if (setup.bRequest==0x82) { // GET_MIN
					//serial_print("vol get_min\n");
					data[0] = 0;     // min level is 0
					data[1] = 0;
				}
				else if (setup.bRequest==0x83) { // GET_MAX
					data[0] = FEATURE_MAX_VOLUME;  // max level, for range of 0 to MAX
					data[1] = 0;
				}
				else if (setup.bRequest==0x84) { // GET_RES
					data[0] = 1; // increment vol by by 1
					data[1] = 0;
				}
				else { // pass over SET_MEM, etc.
					return 0;
				}
				*datalen = 2;
				return 1;
			}
	}
	return 0;
}

int usb_audio_set_feature(void *stp, uint8_t *buf) 
{
	struct setup_struct setup = *((struct setup_struct *)stp);
	printf("set_feature: bmRequestType=%02x bRequest=%02x wValue=%04x wIndex=%04x wLength=%04x\n", setup.bmRequestType, setup.bRequest, setup.wValue, setup.wIndex, setup.wLength);
	if (setup.bmRequestType==0x21) { // should check bRequest, bChannel and UnitID
			if (setup.bCS==0x01) { // mute
				if (setup.bRequest==0x01) { // SET_CUR
					AudioInputUSBEx::features.mute = buf[0]; // 1=mute,0=unmute
					AudioInputUSBEx::features.change = 1;
					return 1;
				}
			}
			else if (setup.bCS==0x02) { // volume
				if (setup.bRequest==0x01) { // SET_CUR
					AudioInputUSBEx::features.volume = buf[0];
					AudioInputUSBEx::features.change = 1;
					return 1;
				}
			}
	}
	return 0;
}


#endif // AUDIO_INTERFACE
