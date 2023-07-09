#ifndef ORNIBIBOT_HPP
#define ORNIBIBOT_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
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

class OrnibiBot : public rclcpp::Node{

    private:
        std::mutex mutex;
        struct termios options;
        const uint8_t buffer_size=16;

        SerialPort *p_com;
        PacketSerial *p_data;

        const char* _port = "/dev/ttyACM0";

        std::thread serial_thread_;        
        rclcpp::TimerBase::SharedPtr timer_serial;
        rclcpp::TimerBase::SharedPtr timer_force;
        rclcpp::TimerBase::SharedPtr timer_restream;
        rclcpp::TimerBase::SharedPtr timer_decode;

        void ForceCallback();
        void RestreamData();
        void SerialCallback();
        void DecodePacket(SerialPort *data_in);

    public:
        OrnibiBot();
        ~OrnibiBot();

};

#endif