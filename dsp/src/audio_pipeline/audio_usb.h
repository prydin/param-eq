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

#pragma once

#include <Arduino.h>
#include "usb_desc.h"
#include "audio_buffer.h"
#include "audio_buffer_fifo.h"
#include "audio_source.h"
#ifdef AUDIO_INTERFACE

#ifndef AUDIO_USB_TX_FIFO_SIZE
#define AUDIO_USB_TX_FIFO_SIZE 8
#endif

#ifndef AUDIO_USB_RX_FIFO_SIZE
#define AUDIO_USB_RX_FIFO_SIZE 8
#endif

#define FEATURE_MAX_VOLUME 0xFF  // volume accepted from 0 to 0xFF

#ifdef __cplusplus
extern "C" {
#endif
extern void usb_audio_configure();
extern uint16_t usb_audio_receive_buffer[];
extern uint16_t usb_audio_transmit_buffer[];
extern uint32_t usb_audio_sync_feedback;
extern uint8_t usb_audio_receive_setting;
extern uint8_t usb_audio_transmit_setting;
extern volatile uint32_t usb_audio_underrun_count;
extern volatile uint32_t usb_audio_overrun_count;
extern void usb_audio_receive_callback(unsigned int len);
extern unsigned int usb_audio_transmit_callback(void);
extern int usb_audio_set_feature(void *stp, uint8_t *buf);
extern int usb_audio_get_feature(void *stp, uint8_t *data, uint32_t *datalen);
#ifdef __cplusplus
}
#endif


#ifdef __cplusplus
class AudioInputUSBEx : public AudioSource
{
public:
	AudioInputUSBEx(void) { begin(); }
	virtual void process(AudioBuffer *block) override;
	void begin(void);
	AudioBuffer *getNextBlock(void);
	static bool getRxFifoStats(AudioBufferFifoStats *stats);
	uint32_t getSampleRate() { return measured_sample_rate; }
	uint32_t getStandardizedSampleRate() { return standardized_sample_rate; }
	bool isSampleRateStable();
	uint32_t getStableRateWindows() { return stable_rate_windows; }
	uint32_t getLastPacketAgeMicros();
	friend void usb_audio_configure(void);
	friend void usb_audio_receive_callback(unsigned int len);
	friend int usb_audio_set_feature(void *stp, uint8_t *buf);
	friend int usb_audio_get_feature(void *stp, uint8_t *data, uint32_t *datalen);
	static struct usb_audio_features_struct features;
	float volume(void) {
		if (features.mute) return 0.0;
		return (float)(features.volume) * (1.0 / (float)FEATURE_MAX_VOLUME);
	}
private:
	static AudioBufferFifo *rx_fifo;
	static AudioBuffer *incoming_buffer;
	static uint16_t incoming_count;
	static uint8_t receive_flag;
	static volatile uint32_t measured_sample_rate;
	static volatile uint32_t standardized_sample_rate;
	static volatile uint32_t last_packet_time_us;
	static volatile uint32_t rate_window_start_us;
	static volatile uint32_t rate_window_samples;
	static volatile uint32_t stable_rate_windows;
	static volatile uint32_t feedback_holdoff_until_us;
	static uint32_t standardizeSampleRate(uint32_t measured_rate);
	static void resetRateTracking(uint32_t now);
	static void updateRateTracking(uint32_t sample_frames);
	static void startFeedbackHoldoff(uint32_t now);
	static bool isFeedbackHoldoffActive(uint32_t now);
};

class AudioOutputUSBEx : public AudioComponent
{
public:
	AudioOutputUSBEx(void) { begin(); }
	void begin(void);
	void process(AudioBuffer *block) override;
	static bool getTxFifoStats(AudioBufferFifoStats *stats);
	friend unsigned int usb_audio_transmit_callback(void);

private:
	static AudioBufferFifo *tx_fifo;
	static AudioBuffer *active_buffer;
	static uint16_t active_offset;
};
#endif
#endif