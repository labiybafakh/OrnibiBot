#include <math.h>
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/Int16.h"
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <vectornav/trueBody.h>
#include <vectornav/dThetaVel.h>
#include <ros/console.h>
#include <vector>
#include <iostream>
#include <bits/stdc++.h>


class OrnibiBot{
    private:
    struct wingPosition{
        int16_t left;
        int16_t right;
    };
    struct force{
        float thrust;
        float lateral;
        float lift;
    };
    struct moment{
        float x;
        float y;
        float z;
    };
    ros::NodeHandle nh;
    // ros::Subscriber forces;
    // ros::Subscriber wing_left;
    // ros::Subscriber wing_right;

    public:
    OrnibiBot();
    // ~OrnibiBot();

    void force_callback(const geometry_msgs::WrenchStamped::ConstPtr &wrench);
    void wing_right_callback(const std_msgs::Int16::ConstPtr &msg);
    void wing_left_callback(const std_msgs::Int16::ConstPtr &msg);
    wingPosition wing_position;
    force _force;
    moment _moment;

};

OrnibiBot::OrnibiBot(){
    ros::Subscriber wing_left = nh.subscribe("wing_left", 1000, &OrnibiBot::wing_left_callback, this);
    ros::Subscriber wing_right = nh.subscribe("wing_right", 1000, &OrnibiBot::wing_right_callback, this);
    ros::Subscriber forces = nh.subscribe("leptrino_force_torque/force_torque", 1000, &OrnibiBot::force_callback, this);
    
}

void OrnibiBot::force_callback(const geometry_msgs::WrenchStamped::ConstPtr& wrench){
        _force.thrust = wrench->wrench.force.x;
        _force.lateral = wrench->wrench.force.y;
        _force.lift = wrench->wrench.force.z;
        _moment.x = wrench->wrench.torque.x;
        _moment.y = wrench->wrench.torque.y;
        _moment.z = wrench->wrench.torque.z; 

}

void OrnibiBot::wing_right_callback(const std_msgs::Int16::ConstPtr& msg){
    wing_position.right = msg->data;

    ROS_INFO_STREAM(msg->data);
}

void OrnibiBot::wing_left_callback(const std_msgs::Int16::ConstPtr& msg){
    wing_position.left = msg->data;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "OrnibibotPC");

    // ros::Time time, lastTime;
    ros::NodeHandle nh;

    ros::Rate rate(100);
    
    while(ros::ok){
        
        OrnibiBot robot;
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}