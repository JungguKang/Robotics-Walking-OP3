#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

#include <sensor_msgs/Imu.h>
#include <string>

#include "robotis_controller_msgs/SetModule.h"
#include "robotis_controller_msgs/SyncWriteItem.h"

#define _USE_MATH_DEFINES
#include <cmath>

#include "walking_state.hpp"
#include "balanceFeedback.hpp"

#include "op3_topic_tutorials/PlotData.h"

void setModule(const std::string& module_name);
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
void detectFootContact(Accelometer cur_acc, const sensor_msgs::Imu::ConstPtr& msg);


// Subscriber for get data from imu sensor
ros::Subscriber imu_sub;
// Subscriber for get joint states
ros::Subscriber joint_state_sub;

ros::Publisher write_joint_pub;
ros::Publisher write_joint_pub2;

ros::ServiceClient set_joint_module_client;

ros::Publisher data_plot_pub;

double start_time;
int cur_state = 4;
bool started = false;
double highest_delta;

Accelometer prev_acc;
bool isValid = false;
bool footContacted = false;
int footContact_start = 0;
double time_interval = 0.12;

void jointstatesCallback(const sensor_msgs::JointState::ConstPtr& msg){
    std::string stance_leg;

    if(started == false){
        start_time = ros::Time::now().toSec();
        started = true;
    }
    
    cout << "Current state: " << cur_state << endl;
    // state transition
    if(cur_state == 4 && (ros::Time::now().toSec()-start_time >= 0.11) ){   // Initial state
        cur_state = 1;
        start_time = ros::Time::now().toSec();
        isValid = false;
        time_interval = 0.03;
    }
    else if(cur_state == 0 && (ros::Time::now().toSec()-start_time >= 0.13) ){
        cur_state = 1;
        start_time = ros::Time::now().toSec();
        isValid = false;
        //footContacted = false;
        time_interval = 0.03;
    }
    else if(cur_state == 1 && footContacted){
        cur_state = 2;
        start_time = ros::Time::now().toSec();
        footContacted = false;

        start_time = ros::Time::now().toSec();
        isValid = false;
        //time_interval = 0.09;
        time_interval = 0.09;

    }
    else if(cur_state == 2 && (ros::Time::now().toSec()-start_time >= 0.15) ){
        start_time = ros::Time::now().toSec();
        isValid = false;
        time_interval = 0.04;
        //footContacted = false;
        cur_state = 3;
    }
        
    else if(cur_state == 3 && footContacted){
        cur_state = 0;
        start_time = ros::Time::now().toSec();
        footContacted = false;

        start_time = ros::Time::now().toSec();
        isValid = false;
        //time_interval = 0.13;
        time_interval = 0.1;

    }
    else ;

    sensor_msgs::JointState write_msg = make_msg(cur_state, msg);


    //ROS_INFO("hip pitch = %lf", hipAngle.pitch);
    /*
    ROS_INFO("distance x = %lf", distance[0]);
    ROS_INFO("distance y = %lf", distance[1]);
    ROS_INFO("distance z = %lf", distance[2]);
    */
    //ROS_INFO("count = %d", count);
    //if(count >30){
    //    cur_state = 0;
    //}
    /*
    if(ros::Time::now() - start_time > ros::Duration(1, 0)){
        cur_state = 0;
    }*/
    write_joint_pub.publish(write_msg);

}

double temp_time;
int main(int argc, char **argv){
    ros::init(argc, argv, "joint_subscriber_tutorial");
    ros::NodeHandle nh(ros::this_node::getName());
    ros::Time::init();

    write_joint_pub = nh.advertise<sensor_msgs::JointState>("/robotis/set_joint_states", 0);
    write_joint_pub2 = nh.advertise<sensor_msgs::JointState>("/robotis/direct_control/set_joint_states", 0);

    data_plot_pub = nh.advertise<op3_topic_tutorials::PlotData>("/plot_data", 0);

    imu_sub = nh.subscribe("/robotis_op3/imu", 1, imuCallback);
    joint_state_sub = nh.subscribe("/robotis/present_joint_states", 1, jointstatesCallback);
    set_joint_module_client = nh.serviceClient<robotis_controller_msgs::SetModule>("/robotis/set_present_ctrl_modules");

    setModule("none");
    ROS_INFO("Change module to none");
    start_time = ros::Time::now().toSec();
    
    temp_time = ros::Time::now().toSec();

    ros::spin();

    return 0;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){

    //cout << "imu time:" << ros::Time::now().toSec() - temp_time << endl;
    temp_time = ros::Time::now().toSec();

    Quaternion q;
    q.w = msg->orientation.w;
    q.x = msg->orientation.x;
    q.y = msg->orientation.y;
    q.z = msg->orientation.z;


    ToEulerAngles(q);

    //2020.05.06
    Accelometer acc, imu_acc, calculated_acc;

    imu_acc.x = msg->linear_acceleration.x;
    imu_acc.y = msg->linear_acceleration.y;
    imu_acc.z = msg->linear_acceleration.z;

    calculated_acc = calVelocity(imu_acc);
    //cout <<"check" << calculated_acc.z << endl;
    detectFootContact(calculated_acc, msg);

    if(footContacted){
        //cout << "Foot contacted!!" << endl << endl;
    }
}

void detectFootContact(Accelometer cur_acc, const sensor_msgs::Imu::ConstPtr& msg){
    //isValid = true;

    
    Accelometer delta;
    delta.x = cur_acc.x - prev_acc.x;
    delta.y = cur_acc.y - prev_acc.y;
    delta.z = cur_acc.z - prev_acc.z;

    prev_acc = cur_acc;

    if(isValid == false){
        //cout << "Elasped time :" << ros::Time::now().toSec() - start_time << endl;
        if(ros::Time::now().toSec() - start_time < time_interval || started == false){
            return;
        }
        else
            isValid = true;
        highest_delta = abs(delta.x) + abs(delta.y) + abs (delta.z);
        cout << "highest_delta : " << highest_delta << endl;
    
    }

    op3_topic_tutorials::PlotData plot_msg;

    plot_msg.deltaX = delta.x;
    plot_msg.deltaY = delta.y;
    plot_msg.deltaZ = delta.z;

    plot_msg.contacted = 0;

    //plot_msg.ori_x = msg->orientation.x;
    //plot_msg.ori_y = msg->orientation.y;
    //plot_msg.ori_z = msg->orientation.z;
    //plot_msg.ori_w = msg->orientation.w;

    plot_msg.vel_x = msg->angular_velocity.x;
    plot_msg.vel_y = msg->angular_velocity.y;
    plot_msg.vel_z = msg->angular_velocity.z;

    EulerAngles global_ang = getAngle();
    plot_msg.ori_x = global_ang.yaw;
    plot_msg.ori_y = global_ang.pitch;
    plot_msg.ori_z = global_ang.roll;


    //cout << "###Accelation Delta###" << endl;
    /*
    cout << "Delta x : " << delta.x << endl; 
    cout << "Delta y : " << delta.y << endl;
    cout << "Delta z : " << delta.z << endl;
    cout << "Total   : " << abs(delta.x) + abs(delta.y) + abs (delta.z) << endl << endl;
    */
    double delta_sum = abs(delta.x) + abs(delta.y) + abs (delta.z);
    cout << "delta sum: " << delta_sum << endl;

    if(delta_sum > highest_delta){
        highest_delta = delta_sum;
        footContact_start = 0;
    }
    else if((cur_state == 1 || cur_state == 3) && footContact_start == 0 && 
        delta_sum < (highest_delta)*0.2){
        footContact_start = 1;
        plot_msg.contacted = 50;
    }
    else if(footContact_start == 1 && delta_sum < (highest_delta)*0.2){ // foot contact start = true and delta sum < 10
        footContact_start = 0;
        plot_msg.contacted = 50;
        footContacted = true;
    }
    else
    {
        footContact_start = 0;
    }
    
    /*
    if((cur_state == 1 || cur_state == 3) &&
        footContact_start == 0 && delta_sum >= 6){
        footContact_start = 1;
        plot_msg.contacted = 50;
        //footContacted = true;
    }
    else if(footContact_start == 1 && delta_sum < 6){ // foot contact start = true and delta sum < 10
        footContact_start = 2;
        plot_msg.contacted = 50;
        //footContacted = true;
    }
    else if(footContact_start == 2 && delta_sum < 2){ // foot contact start = true and delta sum < 10
        footContact_start = 0;
        plot_msg.contacted = 0;
        footContacted = true;
    }
    else if(delta_sum >= 10){
        plot_msg.contacted = 50;
    }
    */

    data_plot_pub.publish(plot_msg);
}



void setModule(const std::string& module_name){
  robotis_controller_msgs::SetModule set_module_srv;
  set_module_srv.request.module_name = module_name;

  if (set_joint_module_client.call(set_module_srv) == false)
  {
    ROS_ERROR("Failed to set module");
    return;
  }

  return ;
}
