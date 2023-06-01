#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#define SERIAL_PORT "/dev/ttyACM0"  // Replace with your serial port

int main() {
    int serialPort = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serialPort == -1) {
        perror("Error opening serial port");
        return -1;
    }
    
    // Configure serial port settings
    struct termios options;
    tcgetattr(serialPort, &options);
    cfsetispeed(&options, B9600);  // Set baud rate (e.g., 9600)
    options.c_cflag |= (CLOCAL | CREAD);  // Enable receiver and set local mode
    options.c_cflag &= ~PARENB;  // Disable parity
    options.c_cflag &= ~CSTOPB;  // Set one stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;  // Set data bits to 8
    tcsetattr(serialPort, TCSANOW, &options);
    
    // Read serial data
    char buffer[16];
    ssize_t bytesRead = read(serialPort, buffer, sizeof(buffer));
    if (bytesRead == -1) {
        perror("Error reading serial data");
        close(serialPort);
        return -1;
    }
    
    // Print received data
    printf("Received data: ");
    for (int i = 0; i < bytesRead; i++) {
        printf("%02X ", (unsigned char)buffer[i]);
    }
    printf("\n");
    
    close(serialPort);
    return 0;
}