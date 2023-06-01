#include <iostream>
#include <string>
#include <chrono>
#include <vector>
#include "Communication.hpp"

Communication comm;

int main(int argc, char* argv[]){

    if(argc > 1)    portInterface = argv[1];
    
    try{
        serial_port.set_option(boost::asio::serial_port::baud_rate(480000));
        serial_port.open(portInterface);
        serial_port.async_read_some(serial_buffer, 16)
    }
    catch(boost::system::system_error& e){
        std::cout << "Error:" << e.what() << std::endl;
    }

    // serial_port.set_option();

    return 0;   
}