#include "comms.h"

#include <Wire.h>
#include <Arduino.h>
#include <ErriezCRC32.h>
#include <netconv.h>
#include "../../../teensy/src/packets.h"
#include "netconv.h"

#define PACKET_QUEUE_SIZE 10

Packet packetQueue[PACKET_QUEUE_SIZE];
volatile int queueHead = 0;
volatile int queueTail = 0;


Packet *popPacket()
{
    noInterrupts();
    if (queueHead == queueTail)
    { 
        interrupts();
        return nullptr; // Queue empty
    }
    Packet *packet = &(packetQueue[queueHead]);
    queueTail = (queueTail - 1) % PACKET_QUEUE_SIZE;
    interrupts();
    return packet;
}

void readPacketFromI2C(int numBytes) // Must be called from ISR
{
    Packet *packet = &(packetQueue[queueHead]);
    uint8_t *ptr = (uint8_t *)packet;
    size_t bytesRead = 0;
    while (Wire.available() && bytesRead < numBytes)
    {
        ptr[bytesRead++] = Wire.read();
    }
    if(bytesRead < sizeof(Packet))
    {
        Serial.printf("Incomplete packet received. Wanted %d bytes, got %d bytes\n", sizeof(Packet), bytesRead);
        return;
    } 
    if(bytesRead > sizeof(Packet))
    {
        Serial.printf("Packet too large. Wanted %d bytes, got %d bytes\n", sizeof(Packet), bytesRead);
        Wire.flush(); // Clear remaining bytes
        return;
    }   
    if (crc32Buffer((uint8_t*) &packet->data, sizeof(packet->data) - 4) != ntohl(packet->checksum))
    {
        Serial.println("Invalid packet checksum");
    }

    int nextTail = (queueTail + 1) % PACKET_QUEUE_SIZE;
    if (nextTail == queueHead)
    {
        Serial.println("Packet queue overflow");
        return;
    }
    queueTail = nextTail;
}