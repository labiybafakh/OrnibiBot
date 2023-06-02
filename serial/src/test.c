#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

#define buffer_size 16

typedef struct{
    int serial_port;
    __uint8_t* buffer_serial;
} SerialPort;

SerialPort *_comm;

struct termios options;
// struct sigaction signals; 

void signalHandler(int signum){

    size_t num_bytes = read(_comm->serial_port, _comm->buffer_serial, buffer_size);

    if(num_bytes == -1) {
        fprintf(stderr, "read failed\n");
    }

    for(size_t i=0; i<buffer_size; i++){
        printf("%d\n", _comm->buffer_serial[i]);
    }
}

int main(int argc, char** argv){
    char* _port = argv[1];

    //Memory allocation for serial
    _comm = malloc(sizeof(SerialPort));
    _comm->buffer_serial = (__uint8_t*)malloc(buffer_size * sizeof(__uint8_t));

    if(_comm->buffer_serial == NULL){
        fprintf(stderr, "Failed to allocate memory for buffer\n");
        return 1;
    }

    _comm->serial_port = open(_port, O_RDWR | O_NOCTTY);

    if(_comm->serial_port == -1) {printf("fail\n"); exit(0);}
    else printf("success\n");

    // signals.sa_handler = signalHandler;
    // sigemptyset(&signals.sa_mask);
    // signals.sa_flags = 0;

    // if(sigaction(SIGIO, &signals, NULL) == -1){
    //     perror("sigaction failed");
    //     return 1;
    // }

    // if(fcntl(_comm->serial_port, __F_SETOWN, getpid()) == -1){
    //     perror("fcntl __F_SETOWN failed");
    //     return 1;
    // }

    // int flags = fcntl(_comm->serial_port, F_GETFL);
    // if(fcntl(_comm->serial_port, F_SETFL, flags | O_ASYNC) == -1){
    //     perror("fcntl F_SETFL failed");
    //     return 1;
    // }

    tcgetattr(_comm->serial_port, &options);
    
    cfsetispeed(&options, B460800);
    cfsetospeed(&options, B460800);

    options.c_cflag = (options.c_cflag & ~CSIZE) | CS8;
    options.c_iflag &= ~IGNBRK;
    options.c_lflag = 0;
    options.c_oflag = 0;

    tcsetattr(_comm->serial_port, TCSANOW, &options);

    // for(int i=0 ; i<3; i++){
        signalHandler(1);
    //     sleep(1);
    // }

    // while(1){
        // signalHandler(1);
        // pause(); //Wait for I/O signal
    // }
    
    close(_comm->serial_port);

    free(_comm->buffer_serial);
    free(_comm);

    // cfgetispeed(&options, B460800);



    return 0;
}