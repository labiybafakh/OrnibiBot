#include <sys/ioctl.h>
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
#include <stdbool.h>


#define buffer_size 20

SerialPort *_comm;
PacketSerial *_data;
pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;

struct termios options;
pthread_t _serialThread;
pthread_t _debugThread;
bool flag_debug=0;

struct timespec start, end;

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

void* debugThread(void *arg){
    while(1){
        if(flag_debug){
        printf("Time(ms): %u\nDesired Position(signal): [%d, %d]\nPosition(rads): [%.2f, %.2f]\nCurrent(A): [%.2f, %.2f]\nVoltage(V):{%.2f, %.2f}\n",
               _data->timestamp, _data->desiredLeft, _data->desiredRight, _data->positionLeft, _data->currentRight,
               _data->currentLeft, _data->currentRight, _data->voltageLeft, _data->voltageRight);
        }
        usleep(5000);
    }
}

void* serialThread(void *arg){
    while(1){
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
        printf("Elapsed time: %.9f seconds\n", elapsed_time);

        usleep(5000);
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
    pthread_create(&_debugThread, NULL, debugThread, NULL);

    pthread_join(_serialThread, NULL);
    pthread_join(_debugThread, NULL);

    close(_comm->serial_port);
    free(_comm);
    free(_data);
    // free(_comm->buffer_serial);

    // cfgetispeed(&options, B460800);



    return 0;
}