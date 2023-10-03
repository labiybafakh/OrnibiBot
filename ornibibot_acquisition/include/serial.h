#ifndef SERIAL_H
#define SERIAL_H

#include <stdlib.h>
#include <stdio.h>

typedef struct{
    volatile uint32_t timestamp;    // 2 bytes
    volatile float desired_left;
    volatile float desired_right;
    volatile float actual_left;  // 2 bytes
    volatile float actual_right; // 2 bytes
    volatile float power_left;   // 2 bytes
    volatile float power_right;  // 2 bytes
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