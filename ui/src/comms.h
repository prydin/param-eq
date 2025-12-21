#ifndef COMMS_H_INCLUDED
#define COMMS_H_INCLUDED

#include "../../common/packets.h"

Packet *popPacket();
void processIncomingPacket(int numBytes);

#endif