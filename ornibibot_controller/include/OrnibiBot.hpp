#ifndef  ORNIBIBOT_HPP

#include <iostream>
#include <vector>
#include <rclcpp/rclcpp.hpp>

class OrnibiBot: public rclcpp::Node{
    private:
        rclcpp::TimerBase::SharedPtr timer_;
        struct wingPosition{
            int16_t right;
            int16_t left;
        };

    public:
        OrnibiBot(): Node("OrnibiBot");
        ~OrnibiBot();
        // wing_right_callback(const std_msgs::Int16::ConstPtr& msg);
        // wing_left_callback(const std_msgs::Int16::ConstPtr& msg);
        // // std::vector<int> get_wing_position();
        // wingPosition wing_position;

};

#endif