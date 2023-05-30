#include <iostream>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <unistd.h>
#include "data.hpp"

int main() {
    // Create a unique key for the shared memory segment
    key_t key = ftok("mykey", 'R');

    // Get the ID of the shared memory segment
    int shm_id = shmget(key, sizeof(int), 0666);

    // Attach to the shared memory segment
    int* shared_data = (int*) shmat(shm_id, NULL, 0);

    // Read data from the shared memory segment
    int data = *shared_data;
    while(1){
    std::cout << std::chrono::milliseconds()<< "Received data: "  << *shared_data << std::endl;
    sleep(1);

    }
        // Detach from the shared memory segment
    shmdt(shared_data);

    return 0;
}
