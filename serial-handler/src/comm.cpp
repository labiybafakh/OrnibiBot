#include <iostream>
#include "boost/asio.hpp"
#include "boost/asio/io_service.hpp"
#include "boost/asio/serial_port.hpp"
#include <string>
#include <chrono>
#include <vector>

std::string _portInterface = "/dev/ttyACMO";

boost::asio::streambuf serial_buffer;
boost::asio::io_context io;
boost::asio::steady_timer timer(io, std::chrono::microseconds(50));
boost::asio::serial_port serial_port(io);

std::vector<,16> data;

int main(int argc, char* argv[]){

    if(argc > 1)    _portInterface = argv[1];
    
    try{
        serial_port.set_option(boost::asio::serial_port::baud_rate(480000));
        serial_port.open(portInterface);
    }
    catch(boost::system::system_error& e){
        std::cout << "Error:" << e.what() << std::endl;
    }


    _serial_port.set_option();

    return 0;   
}