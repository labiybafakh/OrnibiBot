#ifndef SERIAL_H
#define SERIAL_H

#include <stdlib.h>
#include <stdio.h>

typedef struct{
    uint32_t timestamp;    // 2 bytes
    uint16_t desiredLeft;
    uint16_t desiredRight;
    float positionLeft;  // 2 bytes
    float positionRight; // 2 bytes
    float currentLeft;   // 2 bytes
    float currentRight;  // 2 bytes
    float voltageLeft;   // 2 bytes
    float voltageRight;  // 2 bytes
} PacketSerial;

typedef struct{
    int serial_port;
    __uint8_t* buffer_serial;
} SerialPort;

void decodePacket(SerialPort *data_in);

#endif