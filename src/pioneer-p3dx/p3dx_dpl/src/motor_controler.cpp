#ifndef MOTOR_CONTROLER
#define MOTOR_CONTROLER
#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include <tf/transform_listener.h>
#include <string>
#include <vector>
#include <cmath>

// publisher of right and left motor velocity values
ros::Publisher right_motor_pub;
ros::Publisher left_motor_pub;

// robot's index in ROS ecosystem (if multiple instances)
int robot_id = 0;
double WHEELS_HALF_SPACING = 0;
double WHEEL_RADIUS = 0.1;

void callback_val(const geometry_msgs::Twist::ConstPtr &msg) {

  // creating new message
  std_msgs::Float64 msg_right_;
  std_msgs::Float64 msg_left_;
  double yaw = 0;

  if (msg.get()->angular.z < -0.0001 || msg.get()->angular.z > 0.0001)
    yaw = msg.get()->angular.z;
  else if (msg.get()->linear.y < -0.0001 || msg.get()->linear.y > 0.0001) {
    // yaw is not set-- trying to calculate from linear x and y
    ROS_WARN("DPL: in Twist msg no yaw component");
    yaw = std::atan2(msg.get()->linear.y, msg.get()->linear.x);
  }

  // calculate from linear x and yaw speed for each wheel
  msg_right_.data =
      (msg.get()->linear.x + WHEELS_HALF_SPACING * yaw) / WHEEL_RADIUS;
  msg_left_.data =
      (msg.get()->linear.x - WHEELS_HALF_SPACING * yaw) / WHEEL_RADIUS;
  // publishing wheel velocities
  right_motor_pub.publish(move(msg_right_));
  left_motor_pub.publish(move(msg_left_));
}

int main(int argc, char **argv) {
  std::string pub_right_ = "hal/rightMotor/setVel";
  std::string pub_left_ = "hal/leftMotor/setVel";
  std::string sub_twist = "/cmd_vel";
  ros::init(argc, argv, "motorControl");

  // read  from command line names of topics to publish data
  if (argc >= 3) {
    std::vector<std::string> argv_(argv + 1, argv + argc);
    pub_right_ = argv_[0];
    pub_left_ = argv_[1];
    if(argc==4) sub_twist = argv_[2];
  }
  // set relative node namespace
  ros::NodeHandle n;
  n.getParam("robot_id", robot_id);

  // tf listener for calculation of wheel spacing
  tf::TransformListener listener;
  tf::StampedTransform transform;
  // get length in between wheels
  try {
    listener.waitForTransform(std::to_string(robot_id) + "/base/joint1",
                              std::to_string(robot_id) + "/base/joint0",
                              ros::Time::now(), ros::Duration(3.0));
    listener.lookupTransform(std::to_string(robot_id) + "/base/joint1",
                             std::to_string(robot_id) + "/base/joint0",
                             ros::Time(0), transform);
    WHEELS_HALF_SPACING = transform.getOrigin().absolute().getY() / 2;
    ROS_INFO("DPL: motor_controler WHEELS_HALF_SPACING: %f",
             WHEELS_HALF_SPACING);

  } catch (tf::TransformException ex) {
    ROS_ERROR("Problem with transformation between wheels: %s", ex.what());
  }

  // read msg from standard ROS topic for receiving Twist commands
  ros::Subscriber twist_sub = n.subscribe("/cmd_vel", 1000, callback_val);

  right_motor_pub = n.advertise<std_msgs::Float64>(pub_right_, 1000);
  left_motor_pub = n.advertise<std_msgs::Float64>(pub_left_, 1000);
  ROS_INFO("DPL: motor_controler node initialized");

  // run event loop
  ros::spin();
  return 0;
}
#endif
