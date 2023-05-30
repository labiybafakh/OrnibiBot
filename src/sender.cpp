#include <iostream>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <unistd.h>
#include "data.hpp"
#include <chrono>

int numbers=0;
int main() {
    // Create a unique key for the shared memory segment
    key_t key = ftok("mykey", 'R');

    // Create the shared memory segment with read and write permissions
    int shm_id = shmget(key, sizeof(int), IPC_CREAT | 0666);

    // Attach to the shared memory segment
    int* shared_data = (int*) shmat(shm_id, NULL, 0);

    while(1){

    // Write data to the shared memory segment
    *shared_data = numbers++;
    sleep(1);
    }
        // Detach from the shared memory segment
    shmdt(shared_data);
    return 0;
}
