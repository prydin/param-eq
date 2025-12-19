#ifndef PACKETS_H_INCLUDED
#define PACKETS_H_INCLUDED

#include <Arduino.h>

#define PACKET_COEFFS 0x01 // <filter index><b0><b1><b2><a1><a2>
#define PACKET_PARAMS 0x02 // <filter index><type><freq><Q><gain> 


typedef struct {
    uint8_t packetType;
    uint8_t reserved; // Padding for alignment
    union {
        struct {
            uint8_t filterIndex;  
            uint8_t reserved[3]; // Padding for alignment
            float b0;
            float b1;
            float b2;
            float a1;
            float a2;
        } coeffs;
        struct {
            uint8_t filterIndex;
            uint8_t filterType;
            uint8_t reserved[3]; // Padding for alignment
            float frequency;
            float Q;
            float gain;
        } params;
    } data;
    uint32_t checksum; // Netwrk order checksum of all previous bytes in the packet
} Packet;

#endif