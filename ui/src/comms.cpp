#include "comms.h"

#include <Wire.h>
#include <Arduino.h>
#include <ErriezCRC32.h>
#include <netconv.h>
#include "../../common/packets.h"
#include "netconv.h"

#define PACKET_QUEUE_SIZE 10

Packet packetQueue[PACKET_QUEUE_SIZE];
volatile uint32_t queueHead = 0;
volatile uint32_t queueTail = 0;


/**
 * @brief Removes and returns the oldest packet from the packet queue.
 * 
 * This function retrieves the packet at the head of the circular queue in a
 * thread-safe manner by disabling interrupts during the operation. After
 * retrieving the packet, it advances the head pointer and re-enables interrupts.
 * 
 * @return Packet* Pointer to the oldest packet in the queue, or nullptr if the queue is empty.
 * 
 * @note This function disables interrupts temporarily to ensure atomic access to the queue.
 * @note The returned pointer points to a buffer that may be reused when the queue wraps around.
 * @warning The caller should process or copy the packet data before it gets overwritten.
 */
Packet *popPacket()
{
    noInterrupts();
    if (queueHead == queueTail)
    { 
        interrupts();
        return nullptr; // Queue empty
    }
    Packet *packet = &(packetQueue[queueHead]);
    queueHead = (queueHead + 1) % PACKET_QUEUE_SIZE;
    interrupts();
    return packet;
}

/**
 * @brief Processes an incoming I2C packet and adds it to the packet queue.
 * 
 * This function must be called from an ISR context. It reads data from the I2C Wire
 * interface, validates the packet size and checksum, and adds it to a circular queue.
 * 
 * @param numBytes The number of bytes available to read from the I2C buffer
 * 
 * @note This function performs the following validations:
 *       - Checks for queue overflow before adding the packet
 *       - Verifies that the complete packet was received (exact size match)
 *       - Validates the CRC32 checksum of the received packet
 * 
 * @warning Must be called from ISR context only
 * @warning If packet validation fails (incomplete, too large, or invalid checksum),
 *          the packet is discarded and not added to the queue
 * 
 * @see Packet
 * @see PACKET_QUEUE_SIZE
 * @see crc32Buffer
 */
void processIncomingPacket(int numBytes) // Must be called from ISR
{
    Serial.printf("Push Packet: head=%d, tail=%d\n", queueHead, queueTail);
    int nextTail = (queueTail + 1) % PACKET_QUEUE_SIZE;
    if (nextTail == queueHead)
    {
        Serial.println("Packet queue overflow");
        return;
    }

    Packet *packet = &(packetQueue[queueTail]);
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
    uint32_t checksum = crc32Buffer((uint8_t*) packet, sizeof(Packet) - 4);
    if (checksum != ntohl(packet->checksum))
    {
        Serial.printf("Invalid packet checksum. Expected 0x%08X, got 0x%08X, size %d\n", ntohl(packet->checksum), checksum, sizeof(Packet));
    }
    queueTail = nextTail;
}