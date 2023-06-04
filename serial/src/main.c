#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <serial.h>
#include <errno.h>
#include <string.h>
#include <time.h>

#define buffer_size 16

SerialPort *_comm;
PacketSerial *_data;
pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;

struct termios options;
pthread_t _serialThread;

struct timespec start, end;

inline void decodePacket(SerialPort *data_in, PacketSerial *data_out ){
    pthread_mutex_lock(&lock);

    data_out->timestamp = data_in->buffer_serial[1] | (data_in->buffer_serial[2] << 8);
    data_out->positionLeft = (data_in->buffer_serial[3] | (data_in->buffer_serial[4] << 8)) * 0.01f;
    data_out->positionRight = (data_in->buffer_serial[5] | (data_in->buffer_serial[6] << 8)) * 0.01f;
    data_out->currentLeft = (data_in->buffer_serial[7] | (data_in->buffer_serial[8] << 8)) * 0.01f;
    data_out->currentRight = (data_in->buffer_serial[9] | (data_in->buffer_serial[10] << 8)) * 0.01f;
    data_out->voltageLeft = (data_in->buffer_serial[11] | (data_in->buffer_serial[12] << 8)) * 0.01f;
    data_out->voltageRight = (data_in->buffer_serial[13] | (data_in->buffer_serial[14] << 8)) * 0.01f;

    printf("Time(ms): %d\nPosition(rads): [%.2f, %.2f]\nCurrent(A): [%.2f, %.2f]\nVoltage(V):{%.2f, %.2f}\n", 
    data_out->timestamp, data_out->positionLeft, data_out->currentRight,
    data_out->currentLeft, data_out->currentRight, data_out->voltageLeft, data_out->voltageRight);

    pthread_mutex_unlock(&lock);

}

void* serialThread(void *arg){
    while(1){
        clock_gettime(CLOCK_MONOTONIC, &start);

        size_t num_bytes = read(_comm->serial_port, _comm->buffer_serial, buffer_size);
        if(num_bytes == -1){
            fprintf(stderr, "Read Failed: %s\n", strerror(errno));
            continue;
        }

        decodePacket(_comm, _data);

        clock_gettime(CLOCK_MONOTONIC, &end);
        double elapsed_time = end.tv_sec - start.tv_sec + 
                            (end.tv_nsec - start.tv_nsec) / 1e9;
        printf("Elapsed time: %.9f seconds\n", elapsed_time);
        usleep(500);

    
    }
    return NULL;
}

int main(int argc, char** argv){
    char* _port = argv[1];

    //Memory allocation for serial
    _comm = malloc(sizeof(SerialPort));
    _comm->buffer_serial = (__uint8_t*)malloc(buffer_size * sizeof(__uint8_t));
    _data = malloc(sizeof(PacketSerial));

    if(_comm->buffer_serial == NULL){
        fprintf(stderr, "Failed to allocate memory for buffer\n");
        return 1;
    }

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

    pthread_create(&_serialThread, NULL, serialThread, NULL);

    pthread_join(_serialThread, NULL);
    
    close(_comm->serial_port);
    free(_comm);
    free(_data);
    // free(_comm->buffer_serial);

    // cfgetispeed(&options, B460800);



    return 0;
}