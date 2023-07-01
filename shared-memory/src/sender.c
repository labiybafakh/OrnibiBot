#include <sys/ipc.h>
#include <sys/shm.h>
#include <unistd.h>
#include "data.hpp"
#include <stdlib.h>

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


int numbers=0;
int main() {
    // Create a unique key for the shared memory segment
    key_t key = ftok("mykey", 'R');

    // Create the shared memory segment with read and write permissions
    int shm_id = shmget(key, sizeof(PacketSerial), IPC_CREAT | 0666);

    // Attach to the shared memory segment
    PacketSerial* _shared_data = (PacketSerial*) shmat(shm_id, NULL, 0);

    while(1){

    // Write data to the shared memory segment
        _shared_data->timestamp     = numbers+1;
        _shared_data->desiredLeft   = numbers+1;
        _shared_data->desiredRight  = numbers+2;
        _shared_data->positionLeft  = numbers-2;
        _shared_data->positionRight = numbers+3;
        _shared_data->currentLeft   = numbers-3;
        _shared_data->currentRight  = numbers+5;
        _shared_data->voltageLeft   = numbers+6;
        _shared_data->voltageRight  = numbers+7;
        // printf("sending %d", _shared_data->timestamp);
        numbers++;
        usleep(5000);
    }
        // Detach from the shared memory segment
    shmdt(_shared_data);
    return 0;
}
