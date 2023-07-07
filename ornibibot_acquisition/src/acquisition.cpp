#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include "serial.h"
#include <errno.h>
#include <string.h>
#include <time.h>
#include <stdbool.h>
#include <sys/mman.h>
#include <signal.h>
#include <rclcpp/rclcpp.hpp>


#define buffer_size 20

SerialPort *_comm;
SharedMemory *_shared;
PacketSerial *_data;
PacketSerial *_shared_data;

pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;

struct termios options;
pthread_t _serialThread;
pthread_t _debugThread;
pthread_t _sharedThread;

bool flag_debug=0;

int shared_handle;
bool exit_pressed = false;

struct timespec start, end;
volatile sig_atomic_t stop=false;

void handle_sigint(int sig){
    stop = true;
    // exit(0);
}

int sharedMemoryInit(char* node_name){
    //Initialize the shared memory segment
    _shared->shared_memory = shm_open(node_name, O_CREAT | O_RDWR, 0666);
    if(_shared->shared_memory == -1){
        fprintf(stderr, "Shared memory initialization is error\n");
        exit(EXIT_FAILURE);
    }

    ftruncate(_shared->shared_memory, sizeof(PacketSerial));

    _shared_data = mmap(NULL, sizeof(PacketSerial), PROT_READ | PROT_WRITE, MAP_SHARED, _shared->shared_memory, 0);

    if(_shared_data == MAP_FAILED){
        fprintf(stderr, "Error during mappping virtual memory\n");
    }

    return 0;
}

int sharedMemoryDestroy(char* node_name){
    munmap(_shared_data, sizeof(PacketSerial));
    close(_shared->shared_memory);
    shm_unlink(node_name);
}

void decodePacket(SerialPort *data_in){
    pthread_mutex_lock(&lock);

    _data->timestamp     = data_in->buffer_serial[0] | (data_in->buffer_serial[1] << 8);
    _data->timestamp     = _data->timestamp | (data_in->buffer_serial[2] << 16);
    _data->timestamp     = _data->timestamp | (data_in->buffer_serial[3] << 24);
                            // | (data_in->buffer_serial[2] << 16) | (data_in->buffer_serial[3] << 24) ;
    _data->desiredLeft   = data_in->buffer_serial[4] | (data_in->buffer_serial[5] << 8);
    _data->desiredRight  = data_in->buffer_serial[6] | (data_in->buffer_serial[7] << 8);
    _data->positionLeft  = (data_in->buffer_serial[8] | (data_in->buffer_serial[9] << 8)) * 0.01f;
    _data->positionRight = (data_in->buffer_serial[10] | (data_in->buffer_serial[11] << 8)) * 0.01f;
    _data->currentLeft   = (data_in->buffer_serial[12] | (data_in->buffer_serial[13] << 8)) * 0.01f;
    _data->currentRight  = (data_in->buffer_serial[14] | (data_in->buffer_serial[15] << 8)) * 0.01f;
    _data->voltageLeft   = (data_in->buffer_serial[16] | (data_in->buffer_serial[17] << 8)) * 0.01f;
    _data->voltageRight  = (data_in->buffer_serial[18] | (data_in->buffer_serial[19] << 8)) * 0.01f;

    pthread_mutex_unlock(&lock);

}

void* sharedThread(void *arg){
    uint32_t last_timestamp;
    while (!stop)
    {
        _shared_data->timestamp     = _data->timestamp;
        _shared_data->desiredLeft   = _data->desiredLeft;
        _shared_data->desiredRight  = _data->desiredRight;
        _shared_data->positionLeft  = _data->positionLeft;
        _shared_data->positionRight = _data->positionRight;
        _shared_data->currentLeft   = _data->currentLeft;
        _shared_data->currentRight  = _data->currentRight;
        _shared_data->voltageLeft   = _data->voltageLeft;
        _shared_data->voltageRight  = _data->voltageRight;
        
        last_timestamp = _shared_data->timestamp;   
    }
    
}

void* debugThread(void *arg){
    while(!stop){
        if(flag_debug){
        printf("Time(ms): %u\nDesired Position(signal): [%d, %d]\nPosition(rads): [%.2f, %.2f]\nCurrent(A): [%.2f, %.2f]\nVoltage(V):{%.2f, %.2f}\n",
               _data->timestamp, _data->desiredLeft, _data->desiredRight, _data->positionLeft, _data->currentRight,
               _data->currentLeft, _data->currentRight, _data->voltageLeft, _data->voltageRight);
        }
        usleep(5000);
    }
}

void* serialThread(void *arg){
    while(!stop){
        clock_gettime(CLOCK_MONOTONIC, &start);

        uint8_t available_bytes=0;

        if(ioctl(_comm->serial_port, FIONREAD, &available_bytes) < 0){
            //Error Handling
            fprintf(stderr, "ioctl error: %s\n", strerror(errno));
            continue;
            flag_debug=0;
        }
        else{
            if(available_bytes >= buffer_size){
                size_t num_bytes = read(_comm->serial_port, _comm->buffer_serial, buffer_size);

                if(num_bytes == -1){
                    fprintf(stderr, "Read Failed: %s\n", strerror(errno));
                    continue;
                }
                else{
                    decodePacket(_comm);
                    flag_debug=1;
                }

            }
        }

        clock_gettime(CLOCK_MONOTONIC, &end);
        double elapsed_time = end.tv_sec - start.tv_sec + 
                            (end.tv_nsec - start.tv_nsec) / 1e9;
        // printf("Elapsed time: %.9f seconds\n", elapsed_time);

        usleep(5000);
    }
    return NULL;
}

int main(int argc, char** argv){

    char* _port = "/dev/ttyACM0";
    char* node = argv[2];

    signal(SIGINT, handle_sigint);

    //Memory allocation for serial
    _comm   = (SerialPort *)malloc(sizeof(SerialPort));
    _comm->buffer_serial = (__uint8_t*)malloc(buffer_size * sizeof(__uint8_t));
    _data = (PacketSerial *)malloc(sizeof(PacketSerial));
    // _shared_data = malloc(sizeof(PacketSerial));
    // _shared = malloc(sizeof(int));

    if(_comm->buffer_serial == NULL){
        fprintf(stderr, "Failed to allocate memory for buffer\n");
        return 1;
    }

    //Initialize the serial port
    _comm->serial_port = open(_port, O_RDWR | O_NOCTTY);

    if(_comm->serial_port == -1) {printf("fail\n"); exit(0);}
    else printf("success\n");

    tcgetattr(_comm->serial_port, &options);
    
    cfsetispeed(&options, 460800);
    cfsetospeed(&options, 460800);

    options.c_cflag = (options.c_cflag & ~CSIZE) | CS8;
    options.c_iflag &= ~IGNBRK;
    options.c_lflag = 0;
    options.c_oflag = 0;

    tcsetattr(_comm->serial_port, TCSANOW, &options);

    // sharedMemoryInit(node);
    

    //Initialize the threads
    pthread_create(&_serialThread, NULL, serialThread, NULL);
    pthread_create(&_debugThread, NULL, debugThread, NULL);
    pthread_create(&_sharedThread, NULL, sharedThread, NULL);

    // while(!stop);

    pthread_join(_serialThread, NULL);
    pthread_join(_debugThread, NULL);
    pthread_join(_sharedThread, NULL);
    
    //end the shared memory process
    // sharedMemoryDestroy(node);

    //close the serial port connection
    close(_comm->serial_port);

    //free the allocated memory for pointer
    free(_comm);
    free(_data);
    // free(_shared_data);
    // free(_shared);
    
    return 0;
}