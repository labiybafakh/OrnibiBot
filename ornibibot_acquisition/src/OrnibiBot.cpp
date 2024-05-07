#include "OrnibiBot.hpp"


OrnibiBot::OrnibiBot() : Node("OrnibiBot"){
    // serial_thread_ = std::thread(std::bind)


    com_ = (SerialPort *)malloc(sizeof(SerialPort));
    com_->buffer_serial = (__uint8_t*)malloc(buffer_size * sizeof(__uint8_t));
    data_serial_ = (PacketSerial *)malloc(sizeof(PacketSerial));
    force_ = (data3D *)malloc(sizeof(data3D));
    moment_ = (data3D *)malloc(sizeof(data3D));

    wrench_sub = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "leptrino", 5, std::bind(&OrnibiBot::ForceCallback, this, _1)
    );

    restream_pub = this->create_publisher<ornibibot_msgs::msg::OrnibiBotData>(
        "ornibibot_data", 5);
    
    gui_command_sub = this->create_subscription<ornibibot_msgs::msg::OrnibiBotGUI>(
        "ornibibot_gui", 50, std::bind(&OrnibiBot::GUICallback, this, _1)
    );

    optitrack_sub = this->create_subscription<optitrack_msgs::msg::OptitrackData>(
        "optitrack_data", 5, std::bind(&OrnibiBot::OptitrackCallback, this, _1)
    );

    com_->serial_port = open(_port, O_RDWR | O_NOCTTY);

    RCLCPP_INFO(this->get_logger(), "OrnibiBot Data Acquisition V1.1.");


    if(com_->serial_port == -1) RCLCPP_ERROR(this->get_logger(), "Failed to connnect to the serial port.");
    else{
        RCLCPP_INFO(this->get_logger(), "Device is connected.");

        tcgetattr(com_->serial_port, &options);
        
        cfsetispeed(&options, 460800);
        cfsetospeed(&options, 460800);

        options.c_cflag = (options.c_cflag & ~CSIZE) | CS8;
        options.c_iflag &= ~IGNBRK;
        options.c_lflag = 0;
        options.c_oflag = 0;

        tcsetattr(com_->serial_port, TCSANOW, &options);

    }

    wing_marker_x = {0,0,0,0,0,0,0,0};
    wing_marker_y = {0,0,0,0,0,0,0,0};
    wing_marker_z = {0,0,0,0,0,0,0,0};

    timer_serial    = this->create_wall_timer(5ms, std::bind(&OrnibiBot::SerialCallback, this));
    timer_restream  = this->create_wall_timer(5ms, std::bind(&OrnibiBot::RestreamData, this));

    RCLCPP_INFO(this->get_logger(), "Data Acquisition has been started");

}

OrnibiBot::~OrnibiBot(){
    close(com_->serial_port);

    free(com_->buffer_serial);
    free(com_);
    free(data_serial_);

    RCLCPP_WARN(this->get_logger(), "OrnibiBot has been stopped");
}

std::string OrnibiBot::getTimeStr() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
    return ss.str();

}

void OrnibiBot::GUICallback(const ornibibot_msgs::msg::OrnibiBotGUI &msg){
    if(rclcpp::ok()){
        // RCLCPP_INFO(this->get_logger(), "Flag record: %d", msg.record_data);

        flapping_frequency = msg.flapping_frequency;
        flapping_mode = msg.flapping_mode;
        flapping_offset = msg.flapping_offset;
        flapping_amplitude = msg.flapping_amplitude;
        flapping_downstroke_periode = msg.flapping_down_stroke_percentage;
        flag_record = msg.record_data;
        
        if(flapping_mode == 0) flapping_pattern = "sine";
        else if(flapping_mode == 1) flapping_pattern = "square";
        else if(flapping_mode == 2) flapping_pattern = "triangle";
        else if(flapping_mode == 3) flapping_pattern = "saw";
        else if(flapping_mode == 4) flapping_pattern = "rev_saw";
        else if(flapping_mode == 5) flapping_pattern = "adjusted_sine";
         
        buffer_parameter[0] = (uint8_t)0xFF;
        buffer_parameter[1] = (uint8_t)(flapping_frequency*10);
        buffer_parameter[2] = (uint8_t)flapping_mode;
        buffer_parameter[3] = (uint8_t)(flapping_offset + 100);
        buffer_parameter[4] = (uint8_t)flapping_amplitude;
        buffer_parameter[5] = (uint8_t)flapping_downstroke_periode;
        buffer_parameter[6] = (uint8_t)0xEE;

        // for(int i = 0; i < sizeof(buffer_parameter); i++) std::cout << i << "," << buffer_parameter[i] << std::endl;
        // RCLCPP_INFO(this->get_logger(), "%f %d %d %d %d", flapping_frequency, flapping_mode, flapping_offset, flapping_amplitude, flapping_downstroke_periode);
    }
}

void OrnibiBot::ForceCallback(const geometry_msgs::msg::WrenchStamped &msg) const{

    // if(rclcpp::ok()){
        force_->x = msg.wrench.force.x;
        force_->y = msg.wrench.force.y;
        force_->z = msg.wrench.force.z;
        moment_->x = msg.wrench.torque.x;
        moment_->y = msg.wrench.torque.y;
        moment_->z = msg.wrench.torque.z;

        //RCLCPP_INFO(this->get_logger(), "%f", force_->x);
    // }    
    // RCLCPP_INFO(this->get_logger(),  )
}

void OrnibiBot::DecodePacket(SerialPort *data_in){

    // if(rclcpp::ok()){
        data_serial_->timestamp     = data_in->buffer_serial[0] | (data_in->buffer_serial[1] << 8);
        data_serial_->timestamp     = data_serial_->timestamp | (data_in->buffer_serial[2] << 16);
        data_serial_->timestamp     = data_serial_->timestamp | (data_in->buffer_serial[3] << 24);
                                // | (data_in->buffer_serial[2] << 16) | (data_in->buffer_serial[3] << 24) ;
        data_serial_->desired_left   = ((int16_t) (data_in->buffer_serial[4] | (data_in->buffer_serial[5] << 8))) * 0.01f;
        data_serial_->desired_right  = ((int16_t)  (data_in->buffer_serial[6] | (data_in->buffer_serial[7] << 8))) * 0.01f;
        data_serial_->actual_left  = ((int16_t)  (data_in->buffer_serial[8] | (data_in->buffer_serial[9] << 8))) * 0.01f;
        data_serial_->actual_right = ((int16_t)  (data_in->buffer_serial[10] | (data_in->buffer_serial[11] << 8))) * 0.01f;
        data_serial_->power_left   = ((int16_t)  (data_in->buffer_serial[12] | (data_in->buffer_serial[13] << 8))) * 0.01f;
        data_serial_->power_right  = ((int16_t)  (data_in->buffer_serial[14] | (data_in->buffer_serial[15] << 8))) * 0.01f;
    // RCLCPP_INFO(this->get_logger(), "timestamp: %f", data_->actual_left);
    // }
        // RCLCPP_INFO(this->get_logger(), "Timestamp: %d, actual left: %.2f, desired left: %.2f", data_serial_->timestamp, data_serial_->actual_left, data_serial_->actual_left);

}

void OrnibiBot::SerialCallback(){

    
    if(rclcpp::ok()){
        uint8_t available_bytes=0;

        if(ioctl(com_->serial_port, FIONREAD, &available_bytes) < 0){
                    //Error Handling
                    RCLCPP_ERROR_ONCE(this->get_logger(), "bytes %d, ioctl error: %s\n", available_bytes, strerror(errno));
        }
        else
        {
            if(available_bytes >= buffer_size){
                ssize_t num_bytes = read(com_->serial_port, com_->buffer_serial, buffer_size);
                if(num_bytes == -1) RCLCPP_ERROR_ONCE(this->get_logger(), "Read Failed: %s\n", strerror(errno));
                else{
                    OrnibiBot::DecodePacket(com_);
                    // write(com_->serial_port, &buffer_parameter, sizeof(buffer_parameter));
                    write(com_->serial_port, &buffer_parameter, sizeof(buffer_parameter));

                    memset(buffer_parameter, '\0', sizeof(buffer_parameter));
                    // write(com_->serial_port, &data_sent, 1);
                    // RCLCPP_INFO(this->get_logger(), "Serial OK");
                }
            }
        }
    }
}

void OrnibiBot::RestreamData(){
    if(rclcpp::ok()){
        if(flag_record == 1){
            if(prev_record == 0){
                RCLCPP_INFO(this->get_logger(), "Start record data.");
                std::string file_name = getTimeStr() + ".csv";
                const char* file_name_ = file_name.c_str();
                file.open(file_name_);

                // Headers
                file << "timestamp,frequency,pattern,desired_left,actual_left,desired_right,actual_right,power_left,power_right,force_x,force_z,moment_y,marker1_x,marker1_y,marker1_z,marker2_x,marker2_y,marker2_z,marker3_x,marker3_y,marker3_z,marker4_x,marker4_y,marker4_z,marker5_x,marker5_y,marker5_z\n";
            
                prev_record = 1;
            }
            else{

                file << data_serial_->timestamp << "," << flapping_frequency << "," << flapping_pattern << ",";
                file << data_serial_->desired_left << "," << data_serial_->actual_left << ",";
                file << data_serial_->desired_right << "," << data_serial_->actual_right << ",";
                file << data_serial_->power_left << "," << data_serial_->power_right << ",";
                file << force_->x << "," << force_->z << "," << moment_->y << ",";
                file << wing_marker_x[0] << "," << wing_marker_y[0] << "," <<wing_marker_z[0] << ",";
                file << wing_marker_x[1] << "," << wing_marker_y[1] << "," <<wing_marker_z[1] << ",";
                file << wing_marker_x[2] << "," << wing_marker_y[2] << "," <<wing_marker_z[2] << ",";
                file << wing_marker_x[3] << "," << wing_marker_y[3] << "," <<wing_marker_z[3] << ",";
                file << wing_marker_x[4] << "," << wing_marker_y[4] << "," <<wing_marker_z[4] << "\n";
            }

        }

        else{
            if(prev_record == 1) {
                RCLCPP_INFO(this->get_logger(), "CSV OK");

                file.close();
                prev_record = 0;
            }
        }

    }
}

void OrnibiBot::OptitrackCallback(const optitrack_msgs::msg::OptitrackData &optitrack_data) const{

        wing_marker_x[0] = optitrack_data.marker1.x;
        wing_marker_y[0] = optitrack_data.marker1.y;
        wing_marker_z[0] = optitrack_data.marker1.z;
        
        wing_marker_x[1] = optitrack_data.marker2.x;
        wing_marker_y[1] = optitrack_data.marker2.y;
        wing_marker_z[1] = optitrack_data.marker2.z;
        
        wing_marker_x[2] = optitrack_data.marker3.x;
        wing_marker_y[2] = optitrack_data.marker3.y;
        wing_marker_z[2] = optitrack_data.marker3.z;
        
        wing_marker_x[3] = optitrack_data.marker4.x;
        wing_marker_y[3] = optitrack_data.marker4.y;
        wing_marker_z[3] = optitrack_data.marker4.z;
        
        wing_marker_x[4] = optitrack_data.marker5.x;
        wing_marker_y[4] = optitrack_data.marker5.y;
        wing_marker_z[4] = optitrack_data.marker5.z;
        
        // wing_marker_x[5] = optitrack_data.marker6.x;
        // wing_marker_y[5] = optitrack_data.marker6.y;
        // wing_marker_z[5] = optitrack_data.marker6.z;
        
        // wing_marker_x[6] = optitrack_data.marker7.x;
        // wing_marker_y[6] = optitrack_data.marker7.y;
        // wing_marker_z[6] = optitrack_data.marker7.z;
        
        // wing_marker_x[7] = optitrack_data.marker8.x;
        // wing_marker_y[7] = optitrack_data.marker8.y;
        // wing_marker_z[7] = optitrack_data.marker8.z;

}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OrnibiBot>());
    rclcpp::shutdown();
    return 0;
}