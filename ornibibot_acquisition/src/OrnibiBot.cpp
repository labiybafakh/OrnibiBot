#include "OrnibiBot.hpp"


OrnibiBot::OrnibiBot() : Node("OrnibiBot"){
    // signal(SIGINT, signal_handler);
    // serial_thread_ = std::thread(std::bind)

    p_com = (SerialPort *)malloc(sizeof(SerialPort));
    p_com->buffer_serial = (__uint8_t*)malloc(buffer_size * sizeof(__uint8_t));
    p_data = (PacketSerial *)malloc(sizeof(PacketSerial));

    p_com->serial_port = open(_port, O_RDWR | O_NOCTTY);

    if(p_com->serial_port == -1) RCLCPP_ERROR(this->get_logger(), "Failed to connnect to the serial port.");
    else{
        RCLCPP_INFO(this->get_logger(), "Device is connected.");

        tcgetattr(p_com->serial_port, &options);
        
        cfsetispeed(&options, 460800);
        cfsetospeed(&options, 460800);

        options.c_cflag = (options.c_cflag & ~CSIZE) | CS8;
        options.c_iflag &= ~IGNBRK;
        options.c_lflag = 0;
        options.c_oflag = 0;

        tcsetattr(p_com->serial_port, TCSANOW, &options);

    }

    timer_serial = this->create_wall_timer(5ms, std::bind(&OrnibiBot::SerialCallback, this));
    // timer_decode = this->create_wall_timer(5ms, std::bind(&OrnibiBot::DecodePacket, this));
    // this->p_timer_force = std::
}

OrnibiBot::~OrnibiBot(){
    close(p_com->serial_port);

    free(p_com->buffer_serial);
    free(p_com);
    free(p_data);

    RCLCPP_WARN(this->get_logger(), "OrnibiBot has been stopped");
}

void OrnibiBot::ForceCallback(){

}

void OrnibiBot::DecodePacket(SerialPort *data_in){

    p_data->timestamp     = data_in->buffer_serial[0] | (data_in->buffer_serial[1] << 8);
    p_data->timestamp     = p_data->timestamp | (data_in->buffer_serial[2] << 16);
    p_data->timestamp     = p_data->timestamp | (data_in->buffer_serial[3] << 24);
                            // | (data_in->buffer_serial[2] << 16) | (data_in->buffer_serial[3] << 24) ;
    p_data->desired_left   = data_in->buffer_serial[4] | (data_in->buffer_serial[5] << 8);
    p_data->desired_right  = data_in->buffer_serial[6] | (data_in->buffer_serial[7] << 8);
    p_data->actual_left  = (data_in->buffer_serial[8] | (data_in->buffer_serial[9] << 8)) * 0.01f;
    p_data->actual_right = (data_in->buffer_serial[10] | (data_in->buffer_serial[11] << 8)) * 0.01f;
    p_data->power_left   = (data_in->buffer_serial[12] | (data_in->buffer_serial[13] << 8)) * 0.01f;
    p_data->power_right  = (data_in->buffer_serial[14] | (data_in->buffer_serial[15] << 8)) * 0.01f;

    RCLCPP_INFO(this->get_logger(), "timestamp: %d", p_data->power_left);

}

void OrnibiBot::SerialCallback(){
    uint8_t available_bytes=0;

    if(ioctl(p_com->serial_port, FIONREAD, &available_bytes) < 0){
                //Error Handling
                RCLCPP_ERROR_ONCE(this->get_logger(), "ioctl error: %s\n", strerror(errno));
            }
    else
    {
        if(available_bytes >= buffer_size){
            ssize_t num_bytes = read(p_com->serial_port, p_com->buffer_serial, buffer_size);
            if(num_bytes == -1) RCLCPP_ERROR_ONCE(this->get_logger(), "Read Failed: %s\n", strerror(errno));
            else{
                OrnibiBot::DecodePacket(p_com);
            }
        }
    }
}

void OrnibiBot::RestreamData(){

}



int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OrnibiBot>());
    rclcpp::shutdown();
    return 0;
}