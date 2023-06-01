#include "Communication.hpp"

Communication::Communication(){

}

Communication::~Communication(){

}

std::vector<std::byte> receiveData(){
    std::vector<std::byte> received_data;
    received_data.reserve(16);

    return received_data;
} 

bool Communication::decodePacket(){

    decodedPacket.timestamp = encodedData[1] | (encodedData[2] << 8);
    decodedPacket.positionLeft = encodedData[3] | (encodedData[4] << 8);
    decodedPacket.positionRight = encodedData[5] | (encodedData[6] << 8);
    decodedPacket.currentLeft = encodedData[7] | (encodedData[8] << 8);
    decodedPacket.currentRight = encodedData[9] | (encodedData[10] << 8);
    decodedPacket.voltageLeft = encodedData[12] | (encodedData[13] << 8);
    decodedPacket.voltageRight = encodedData[14] | (encodedData[15] << 8);
  
    return 1;
}


