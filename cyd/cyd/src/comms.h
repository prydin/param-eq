#ifndef COMMS_H_INCLUDED
#define COMMS_H_INCLUDED

#include "../../../teensy/src/packets.h"

Packet *popPacket();
void processIncomingPacket(int numBytes);

#endif