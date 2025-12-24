#ifndef PACKETS_H_INCLUDED
#define PACKETS_H_INCLUDED

#include <Arduino.h>
#include "filter.h"

#define PACKET_COEFFS 0x01
#define PACKET_PARAMS 0x02

#define DISPLAY_MODE_INDIVIDUAL 0x00
#define DISPLAY_MODE_COMBINED   0x01

/**
 * @brief Network packet structure for audio filter configuration.
 * 
 * This structure represents a communication packet used to transmit audio filter
 * settings between devices. The packet supports two types of data: raw filter
 * coefficients or filter parameters.
 * 
 * @note All multi-byte values are stored in network byte order (big-endian).
 * @note The structure is padded for proper memory alignment.
 * 
 * @var Packet::packetType
 *      Identifies the type of packet being transmitted:
 *      - PACKET_COEFFS (0x01): Filter coefficient data
 *      - PACKET_PARAMS (0x02): Filter parameter data
 *      - PACKET_INDICATORS (0x03): Audio level indicators
 *
 * @var Packet::selectedFilterBand
 *      Index of the currently selected filter band for display or editing.
 *
 * @var Packet::displayMode
 *      Display mode selection:
 *      - DISPLAY_MODE_INDIVIDUAL (0x00): Show individual filter band
 *      - DISPLAY_MODE_COMBINED (0x01): Show combined frequency response
 *
 * @var Packet::reserved
 *      Padding bytes for 4-byte alignment.
 * 
 * @var Packet::data
 *      Union containing either coefficient data or parameter data.
 * 
 * @var Packet::data::coeffs
 *      Biquad filter coefficients structure.
 * 
 * @var Packet::data::coeffs::filterIndex
 *      Index of the filter to configure.
 * 
 * @var Packet::data::coeffs::b0
 *      Feed-forward coefficient b0 (encoded as network order float).
 * 
 * @var Packet::data::coeffs::b1
 *      Feed-forward coefficient b1 (encoded as network order float).
 * 
 * @var Packet::data::coeffs::b2
 *      Feed-forward coefficient b2 (encoded as network order float).
 * 
 * @var Packet::data::coeffs::a1
 *      Feed-back coefficient a1 (encoded as network order float).
 * 
 * @var Packet::data::coeffs::a2
 *      Feed-back coefficient a2 (encoded as network order float).
 * 
 * @var Packet::data::params
 *      Filter parameters structure for high-level filter configuration.
 * 
 * @var Packet::data::params::filterIndex
 *      Index of the filter to configure.
 * 
 * @var Packet::data::params::filterType
 *      Type of filter (e.g., low-pass, high-pass, band-pass, etc.).
 * 
 * @var Packet::data::params::frequency
 *      Center or cutoff frequency (network order).
 * 
 * @var Packet::data::params::Q
 *      Quality factor or bandwidth parameter (network order).
 * 
 * @var Packet::data::params::gain
 *      Gain value in dB for peaking/shelving filters (network order).
 * 
 * @var Packet::checksum
 *      Network order checksum calculated over all preceding bytes in the packet
 *      for integrity verification.
 */
typedef struct {
    uint8_t packetType;
    uint8_t selectedFilterBand;
    uint8_t displayMode;
    uint8_t reserved[1]; // Padding for alignment
    union {
        struct {
            uint8_t reserved[3]; // Padding for alignment
            uint32_t b0; // Encoded as network order float. Use htonf() to convert.
            uint32_t b1;
            uint32_t b2;
            uint32_t a1;
            uint32_t a2;
        } coeffs[FILTER_BANDS];
        struct {
            uint8_t filterType;
            uint8_t reserved[3]; // Padding for alignment
            uint32_t frequency;
            uint32_t Q;
            uint32_t gain;
        } params[FILTER_BANDS];
        struct {
            uint8_t clipped;
            uint8_t reserved[3]; // Padding for alignment
            uint32_t peakLevel;
            uint32_t rmsLevel;
        } levels;
    } data;
    uint32_t checksum; // Netwrk order checksum of all previous bytes in the packet
} Packet;

#endif 