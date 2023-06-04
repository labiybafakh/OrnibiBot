#include "serial.h"


// inline void decodePacket(SerialPort &data_in, PacketSerial &data_out ){

//     data_out->timestamp = data_in->buffer_serial[1] | (data_in->buffer_serial[2] << 8);
//     data_out->positionLeft = data_in->buffer_serial[3] | (data_in->buffer_serial[4] << 8);
//     data_out->positionRight = data_in->buffer_serial[5] | (data_in->buffer_serial[6] << 8);
//     data_out->currentLeft = data_in->buffer_serial[7] | (data_in->buffer_serial[8] << 8);
//     data_out->currentRight = data_in->buffer_serial[9] | (data_in->buffer_serial[10] << 8);
//     data_out->voltageLeft = data_in->buffer_serial[11] | (data_in->buffer_serial[12] << 8);
//     data_out->voltageRight = data_in->buffer_serial[13] | (data_in->buffer_serial[14] << 8);
  
// }