#ifndef PACKETS_H_INCLUDED
#define PACKETS_H_INCLUDED

#include <Arduino.h>
#include "filter.h"

#define PACKET_COEFFS 0x01
#define PACKET_PARAMS 0x02
#define PACKET_LEVELS 0x030
#define PACKET_SAMPLE_RATE 0x04
#define PACKET_CLIP_ALERT 0x05

#define DISPLAY_MODE_INDIVIDUAL 0x00
#define DISPLAY_MODE_COMBINED   0x01

typedef struct {
    uint8_t packetType;
    uint8_t selectedFilterBand;
    uint8_t displayMode;
    uint8_t reserved[1]; // Padding for alignment
    union {
        struct {
            struct {
                uint32_t b0; // Encoded as network order float. Use htonf() to convert.
                uint32_t b1;
                uint32_t b2;
                uint32_t a1;
                uint32_t a2;
            } coeffs[FILTER_BANDS];
            uint32_t masterGain;
        } filters;
        uint32_t masterGain;
        struct {
            uint8_t filterType;
            uint8_t reserved[3]; // Padding for alignment
            uint32_t frequency;
            uint32_t Q;
            uint32_t gain;
            } params[FILTER_BANDS];
        struct {
            uint32_t peakLevel;
            uint32_t rmsLevel;
        } status;
        struct {
            uint32_t sampleRate;
            uint8_t stable;
            uint8_t bits;
            uint8_t reserved[2]; // Padding for alignment
        } sampleRateInfo;
        struct {
            uint8_t clipped;
            uint8_t reserved[3]; // Padding for alignment
        } clipAlert;
    } data;
    uint32_t checksum; // Netwrk order checksum of all previous bytes in the packet
} Packet;

#endif 