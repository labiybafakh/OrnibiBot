#ifndef SERIAL_H
#define SERIAL_H

#include <stdlib.h>
#include <stdio.h>

typedef struct{
    uint32_t timestamp;    // 2 bytes
    uint16_t desired_left;
    uint16_t desired_right;
    float actual_left;  // 2 bytes
    float actual_right; // 2 bytes
    int16_t power_left;   // 2 bytes
    int16_t power_right;  // 2 bytes
} PacketSerial;

typedef struct{
    int serial_port;
    __uint8_t* buffer_serial;
} SerialPort;

typedef struct{
    int shared_memory;
} SharedMemory;

void decodePacket(SerialPort *data_in);

#endif