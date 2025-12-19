#ifndef COMMS_H_INCLUDED
#define COMMS_H_INCLUDED

#include "../../../teensy/src/packets.h"

Packet *popPacket();
void readPacketFromI2C(int numBytes);

#endif