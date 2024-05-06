#ifndef ORNIBIBOT_HPP
#define ORNIBIBOT_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "ornibibot_msgs/msg/ornibi_bot_data.hpp"
#include "ornibibot_msgs/msg/ornibi_bot_gui.hpp"
#include "optitrack_msgs/msg/optitrack_data.hpp"
#include "signal.h"
#include "memory.h"
#include "serial.h"
#include "string.h"
#include "termios.h"
#include "unistd.h"
#include "fcntl.h"
#include "sys/ioctl.h"
#include <memory>
#include <fstream>

using namespace std::chrono_literals;
using std::placeholders::_1;


typedef struct{
    volatile float x;
    volatile float y;
    volatile float z;
} data3D;

class OrnibiBot : public rclcpp::Node{

    private:
        std::mutex mutex;
        struct termios options;
        const uint8_t buffer_size=16;
        const size_t n_marker = 8;

        std::vector<data3D> wing_data;

        uint8_t data_sent;
        float flapping_frequency;
        uint8_t flapping_mode;
        int8_t flapping_offset;
        uint8_t flapping_amplitude;
        uint8_t flapping_downstroke_periode;
        
        bool flag_record = 0;
        uint8_t buffer_parameter[7];
        

        SerialPort *com_;
        PacketSerial *data_serial_;

        data3D *force_;
        data3D *moment_;

        bool prev_record = 0;

        mutable std::array<float, 8> wing_marker_x;
        mutable std::array<float, 8> wing_marker_y;
        mutable std::array<float, 8> wing_marker_z;    

        volatile float last_actual_left, last_actual_right;

        std::ofstream file;


        const char* _port = "/dev/ttyACM1";

        std::thread serial_thread_;        
        
        rclcpp::TimerBase::SharedPtr timer_serial;
        rclcpp::TimerBase::SharedPtr timer_restream;

        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub;
        rclcpp::Publisher<ornibibot_msgs::msg::OrnibiBotData>::SharedPtr restream_pub;
        rclcpp::Subscription<ornibibot_msgs::msg::OrnibiBotGUI>::SharedPtr gui_command_sub;
        rclcpp::Subscription<optitrack_msgs::msg::OptitrackData>::SharedPtr optitrack_sub;

        void ForceCallback(const geometry_msgs::msg::WrenchStamped &msg) const;
        void RestreamData();
        void SerialCallback();
        void GUICallback(const ornibibot_msgs::msg::OrnibiBotGUI &msg);
        void DecodePacket(SerialPort *data_in);
        void OptitrackCallback(const optitrack_msgs::msg::OptitrackData &optitrack_data) const;
        std::string getTimeStr();

    public:
        OrnibiBot();
        ~OrnibiBot();

};

#endif