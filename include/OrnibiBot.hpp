#ifndef  ORNIBIBOT_HPP

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Int16.h>


class OrnibiBot{
    private:
    struct wingPosition{
        int16_t right;
        int16_t left;
    };

    public:
    OrnibiBot();
    ~OrnibiBot();
    // wing_right_callback(const std_msgs::Int16::ConstPtr& msg);
    // wing_left_callback(const std_msgs::Int16::ConstPtr& msg);
    // // std::vector<int> get_wing_position();
    // wingPosition wing_position;

};

#endif