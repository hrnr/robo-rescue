#ifndef ACCEL_SUBSCRIB
#define ACCEL_SUBSCRIB
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "p3dx_hal_vrep/StampedFloat32Array.h"
#include <string>
#include <vector>
#include <iostream>

// publisher of Accelerometer data
ros::Publisher publisher;

void callback_val(const std_msgs::String::ConstPtr & msg) {
    std::vector<float> axis_data;
    std::string data=msg->data;
    // linear parser of data in format x;x;x were x is a float
    std::string value=""; // partial data (x)
    for(size_t i=0; i<data.length();++i){
        if(data[i] == ';'){
            // in input data is 0,111 change to 0.111
            std::replace(value.begin(),value.end(), ',','.');
            axis_data.push_back(std::stof(value,nullptr));
            value.clear();
            ++i;
        }
            value+=data[i];
    }
    // last element is added
    if(!value.empty()) axis_data.push_back(std::stof(value));
    if (axis_data.size() > 3){
        ROS_ERROR("HAL(VREP): Accelerometer received more than three values(axis) from vrep");
        axis_data.resize(3);
    }
    // creating new message
    p3dx_hal_vrep::StampedFloat32Array msg_out;
    std_msgs::Header header; // creating header
    //  fill msg
    msg_out.header=header;
    msg_out.data=axis_data;
    publisher.publish(msg_out); // publish vector of floats. One float for each axis X,Y,Z
}

int main(int argc, char ** argv) {
    // init node name as accelerometer
    ros::init(argc, argv, "accelerometer");
    // set relative node namespace
    ros::NodeHandle n("~");
    int robot_no = 0; // robot's index in vrep (if multiple instances)
    n.getParam("vrep_index", robot_no);
    // read msg from vrep topic
    ros::Subscriber sub_val = n.subscribe("/vrep/i" + std::to_string(robot_no) +
        "_Pioneer_p3dx_Accelerometer", 1000, callback_val);
    // creates publisher
    publisher = n.advertise<p3dx_hal_vrep::StampedFloat32Array>("sensor" + std::to_string(robot_no)+"/accelerometer", 1000);
    ROS_INFO("HAL(VREP): Accelerometer node initialized");
    
    // run event loop
    ros::spin();
    return 0;
}
#endif