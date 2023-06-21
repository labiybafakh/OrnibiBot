#include <iostream>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <unistd.h>
#include "data.hpp"
#include <pthread.h>


pthread_t debugThread;
pthread_t memoryThread;

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

PacketSerial* _data;

void* debugThreads(void* arg){
    while(true){
        printf("Time(ms): %u\nDesired Position(signal): [%d, %d]\nPosition(rads): [%.2f, %.2f]\nCurrent(A): [%.2f, %.2f]\nVoltage(V):{%.2f, %.2f}\n",
        _data->timestamp, _data->desiredLeft, _data->desiredRight, _data->positionLeft, _data->currentRight,
        _data->currentLeft, _data->currentRight, _data->voltageLeft, _data->voltageRight);
    
        usleep(5000);
    }
}

int main() {

    _data = malloc(sizeof(PacketSerial));   
    // Create a unique key for the shared memory segment
    key_t key = ftok("mykey", 'R');

    // Get the ID of the shared memory segment
    int shm_id = shmget(key, sizeof(PacketSerial), 0666);

    // Attach to the shared memory segment
    PacketSerial* shared_data = (PacketSerial*) shmat(shm_id, NULL, 0);

    pthread_create(&debugThread, NULL, debugThreads, NULL);

    pthread_join(debugThread, NULL);

    free(_data);
        // Detach from the shared memory segment
    shmdt(shared_data);

    return 0;
}
