#ifndef COMMUNICATION_HPP
#define COMMUNICATION_HPP

#include <iostream>
#include <vector>
#include "boost/asio.hpp"
#include "boost/asio/io_service.hpp"
#include "boost/asio/serial_port.hpp"


class Communication{
private:
    struct packetData
    {
        uint32_t timestamp;    // 2 bytes
        int16_t positionLeft;  // 2 bytes
        int16_t positionRight; // 2 bytes
        int16_t currentLeft;   // 2 bytes
        int16_t currentRight;  // 2 bytes
        int16_t voltageLeft;   // 2 bytes
        int16_t voltageRight;  // 2 bytes
    };

    packetData packetSerial;
    packetData *_packetSerial = &packetSerial;
    
    int16_t encodeFloatToInt(float value);
    float decodeFloatToInt(int16_t value);


    boost::asio::streambuf serial_buffer;
    boost::asio::io_context io;
    // boost::asio::steady_timer timer;
    boost::asio::serial_port serial_port(io,);
    std::string portInterface = "/dev/ttyACMO";
    std::vector<std::byte> data;
    

public:
    boost::asio::streambuf *_serial_buffer = &serial_buffer;
    std::string *_port_interface = &portInterface;
    Communication();
    ~Communication();
    bool decodePacket();
}

#endif