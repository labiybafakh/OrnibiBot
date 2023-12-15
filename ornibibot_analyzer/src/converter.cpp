#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "ornibibot_msgs/msg/ornibi_bot_data.hpp"
#include <iostream>
#include <stdio.h>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;

struct axes_data{
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
};

struct data_ornibibot{
    std::vector<uint32_t> robot_time;
    std::vector<double> desired_left;
    std::vector<double> desired_right;
    std::vector<double> actual_left;
    std::vector<double> actual_right;
    std::vector<double> velocity_left;
    std::vector<double> velocity_right;
    std::vector<double> power_left;
    std::vector<double> power_right;
    axes_data force;
    axes_data moment;
};
    
auto ornibibot_data = std::make_shared<data_ornibibot>();
std::size_t previous_size;
bool bag_played = 1;


void callback_robot(const ornibibot_msgs::msg::OrnibiBotData::SharedPtr received){
    ornibibot_data->robot_time.push_back(received->robot_time);
    ornibibot_data->desired_left.push_back(received->desired_left);
    ornibibot_data->desired_right.push_back(received->desired_right);
    ornibibot_data->actual_left.push_back(received->actual_left);
    ornibibot_data->actual_right.push_back(received->actual_right);
    ornibibot_data->power_left.push_back(received->power_left);
    ornibibot_data->power_right.push_back(received->power_right);

    RCLCPP_INFO(rclcpp::get_logger("subs"), "%f", received->actual_left);
}

void callback_sensor(const geometry_msgs::msg::WrenchStamped::SharedPtr sensor){
    
    ornibibot_data->force.x.push_back(sensor->wrench.force.x);
    ornibibot_data->force.y.push_back(sensor->wrench.force.y);
    ornibibot_data->force.z.push_back(sensor->wrench.force.z);

    ornibibot_data->moment.x.push_back(sensor->wrench.torque.x);
    ornibibot_data->moment.y.push_back(sensor->wrench.torque.y);
    ornibibot_data->moment.z.push_back(sensor->wrench.torque.z);

}

void callback_alert(){
    if(ornibibot_data->robot_time.size() == previous_size) bag_played = 0; 
    else {
        bag_played = 1;
        previous_size = ornibibot_data->robot_time.size();
    }
}

int main(int argc, char** argv){
    
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("converter");
    
    auto force_sub = node->create_subscription<geometry_msgs::msg::WrenchStamped>("leptrino", 5, callback_sensor);
    auto robot_sub = node->create_subscription<ornibibot_msgs::msg::OrnibiBotData>("ornibibot_data", 5, callback_robot);
    auto flag_receive_data = node->create_wall_timer(1000ms, std::bind(callback_alert));

    while(rclcpp::ok() && bag_played){
        rclcpp::spin(node);
    }

    std::cout << "terminate";

    rclcpp::shutdown();
    return 0;
}