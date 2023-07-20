#ifndef SERIAL_H
#define SERIAL_H

#include <stdlib.h>
#include <stdio.h>

typedef struct{
        uint32_t timestamp;    // 4 bytes
        float desiredLeft;   // 2 bytes
        float desiredRight;  // 2 bytes
        float positionLeft;  // 2 bytes
        float positionRight; // 2 bytes
        float powerleft;   // 2 bytes
        float powerright;  // 2 bytes
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