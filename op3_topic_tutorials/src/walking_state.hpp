#ifndef WALKINGSTATE_H
#define WALKINGSTATE_H

#include <cmath>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

struct jointValue {
    double l_hip_pitch, l_hip_roll, r_hip_pitch, r_hip_roll;
        
};

double walk_position(std::string joint_name);
double state_zero(double cur_pos, double cur_vel, std::string joint_name);
double state_one(double cur_pos, double cur_vel, std::string joint_name);
double state_two(double cur_pos, double cur_vel, std::string joint_name);
double state_three(double cur_pos, double cur_vel, std::string joint_name);
double initial_state_zero(double cur_pos, double cur_vel, std::string joint_name);
sensor_msgs::JointState make_msg(int state, const sensor_msgs::JointState::ConstPtr& msg);
double calTorque(double cur_pos, double des_pos, double cur_vel, double des_vel);



#endif