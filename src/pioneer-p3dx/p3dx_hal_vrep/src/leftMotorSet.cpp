#ifndef LEFT_MOTOR_VREP_SUBSCRIBER
#define LEFT_MOTOR_VREP_SUBSCRIBER
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <string>

// publisher of ROS joint velocity data to vrep
ros::Publisher publisher;

void callback_val(const std_msgs::Float64::ConstPtr &msg) {
  // publish float 64 of value velocity of right wheel
  publisher.publish(msg);
}

int main(int argc, char **argv) {
  // init node name as accelerometer
  ros::init(argc, argv, "leftMotorSet");
  // set relative node namespace
  ros::NodeHandle n("~");
  int vrep_no = 0; // robot's index in vrep (if multiple instances)
  n.getParam("vrep_index", vrep_no);
  // read msg from ROS  wheel control topic
  ros::Subscriber sub_val = n.subscribe("setVel", 1000, callback_val);

  publisher = n.advertise<std_msgs::Float64>(
      "/vrep/i" + std::to_string(vrep_no) + "_Pioneer_p3dx_rightMotor/setVel",
      1000);
  ROS_INFO("HAL(VREP): Left motor publisher to vrep: velocity msg initialized");

  // run event loop
  ros::spin();
  return 0;
}
#endif
