#ifndef SHARED_MEMORY_H
#define SHARED_MEMORY_H

#include <stdlib.h>


typedef struct{
    uint32_t _time;
    float currentLeft;
    float curretnRight;
    float voltageLeft;
    float voltageRight;
    float positionLeft;
    float positionRight;
} robot_data;

void initSubscriber(char* type);
void initPublisher(char* type);
void freeSharedMemory();

#endif