#ifndef ORNIBIBOT_HPP
#define ORNIBIBOT_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "ornibibot_msgs/msg/ornibi_bot_data.hpp"
#include "ornibibot_msgs/msg/ornibi_bot_gui.hpp"
#include "signal.h"
#include "memory.h"
#include "serial.h"
#include "string.h"
#include "termios.h"
#include "unistd.h"
#include "fcntl.h"
#include "sys/ioctl.h"

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
        float flapping_frequency;
        uint8_t flapping_mode;

        SerialPort *p_com;
        PacketSerial *p_data;

        data3D *p_force;
        data3D *p_moment;

        volatile float last_actual_left, last_actual_right;

        const char* _port = "/dev/ttyACM1";

        std::thread serial_thread_;        
        
        rclcpp::TimerBase::SharedPtr timer_serial;
        rclcpp::TimerBase::SharedPtr timer_force;
        rclcpp::TimerBase::SharedPtr timer_restream;
        rclcpp::TimerBase::SharedPtr timer_decode;
        rclcpp::TimerBase::SharedPtr timer_gui_command;

        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub;
        rclcpp::Publisher<ornibibot_msgs::msg::OrnibiBotData>::SharedPtr restream_pub;
        rclcpp::Subscription<ornibibot_msgs::msg::OrnibiBotGUI>::SharedPtr gui_command_sub;

        void ForceCallback(const geometry_msgs::msg::WrenchStamped &msg) const;
        void RestreamData();
        void SerialCallback();
        void GUICallback(const ornibibot_msgs::msg::OrnibiBotGUI &msg);
        void DecodePacket(SerialPort *data_in);

    public:
        OrnibiBot();
        ~OrnibiBot();

};

#endif