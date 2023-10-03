#include "OrnibiBot.hpp"


OrnibiBot::OrnibiBot() : Node("OrnibiBot"){
    // serial_thread_ = std::thread(std::bind)

    p_com = (SerialPort *)malloc(sizeof(SerialPort));
    p_com->buffer_serial = (__uint8_t*)malloc(buffer_size * sizeof(__uint8_t));
    p_data = (PacketSerial *)malloc(sizeof(PacketSerial));
    p_force = (data3D *)malloc(sizeof(data3D));
    p_moment = (data3D *)malloc(sizeof(data3D));

    wrench_sub = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "leptrino", 5, std::bind(&OrnibiBot::ForceCallback, this, _1)
    );

    restream_pub = this->create_publisher<ornibibot_msgs::msg::OrnibiBotData>(
        "ornibibot_data", 5);

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

    timer_serial    = this->create_wall_timer(5ms, std::bind(&OrnibiBot::SerialCallback, this));
    timer_restream  = this->create_wall_timer(5ms, std::bind(&OrnibiBot::RestreamData, this));
    //timer_force     = this->create_wall_timer(5ms, std::bind(&OrnibiBot::ForceCallback, this));
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

void OrnibiBot::ForceCallback(const geometry_msgs::msg::WrenchStamped msg) const{

    if(rclcpp::ok()){
        p_force->x = msg.wrench.force.x;
        p_force->y = msg.wrench.force.y;
        p_force->z = msg.wrench.force.z;
        p_moment->x = msg.wrench.torque.x;
        p_moment->y = msg.wrench.torque.y;
        p_moment->z = msg.wrench.torque.z;

        //RCLCPP_INFO(this->get_logger(), "%f", p_force->x);
    }    
    // RCLCPP_INFO(this->get_logger(),  )
}

void OrnibiBot::DecodePacket(SerialPort *data_in){

    if(rclcpp::ok()){
        p_data->timestamp     = data_in->buffer_serial[0] | (data_in->buffer_serial[1] << 8);
        p_data->timestamp     = p_data->timestamp | (data_in->buffer_serial[2] << 16);
        p_data->timestamp     = p_data->timestamp | (data_in->buffer_serial[3] << 24);
                                // | (data_in->buffer_serial[2] << 16) | (data_in->buffer_serial[3] << 24) ;
        p_data->desired_left   = ((int16_t) (data_in->buffer_serial[4] | (data_in->buffer_serial[5] << 8))) * 0.01f;
        p_data->desired_right  = ((int16_t)  (data_in->buffer_serial[6] | (data_in->buffer_serial[7] << 8))) * 0.01f;
        p_data->actual_left  = ((int16_t)  (data_in->buffer_serial[8] | (data_in->buffer_serial[9] << 8))) * 0.01f;
        p_data->actual_right = ((int16_t)  (data_in->buffer_serial[10] | (data_in->buffer_serial[11] << 8))) * 0.01f;
        p_data->power_left   = ((int16_t)  (data_in->buffer_serial[12] | (data_in->buffer_serial[13] << 8))) * 0.01f;
        p_data->power_right  = ((int16_t)  (data_in->buffer_serial[14] | (data_in->buffer_serial[15] << 8))) * 0.01f;
    // RCLCPP_INFO(this->get_logger(), "timestamp: %f", p_data->actual_left);
    }

}

void OrnibiBot::SerialCallback(){
    
    if(rclcpp::ok()){

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
}

void OrnibiBot::RestreamData(){
    if(rclcpp::ok()){
        auto ornibibot_msg = std::make_shared<ornibibot_msgs::msg::OrnibiBotData>(); 

        ornibibot_msg->time = this->now();
        ornibibot_msg->robot_time = p_data->timestamp;
        ornibibot_msg->actual_left = p_data->actual_left;
        ornibibot_msg->actual_right = p_data->actual_right;
        ornibibot_msg->desired_left = p_data->desired_left;
        ornibibot_msg->desired_right= p_data->desired_right;
        ornibibot_msg->velocity_left = (p_data->actual_left - last_actual_left) / 0.005f;
        ornibibot_msg->velocity_right = (p_data->actual_right - last_actual_right) / 0.005f;
        ornibibot_msg->power_left = p_data->power_left;
        ornibibot_msg->power_right = p_data->power_right;
        ornibibot_msg->wrench.force.x = p_force->x;
        ornibibot_msg->wrench.force.y = p_force->y;
        ornibibot_msg->wrench.force.z = p_force->z;
        ornibibot_msg->wrench.torque.x = p_moment->x;
        ornibibot_msg->wrench.torque.y = p_moment->y;
        ornibibot_msg->wrench.torque.z = p_moment->z;

        last_actual_left = p_data->actual_left;
        last_actual_right = p_data->actual_right;

        restream_pub->publish(*ornibibot_msg);
    }
}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OrnibiBot>());
    rclcpp::shutdown();
    return 0;
}