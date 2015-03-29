#ifndef LEFT_MOTOR_VREP_PUBLISHER
#define LEFT_MOTOR_VREP_PUBLISHER
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <string>

// publisher of Accelerometer data
ros::Publisher publisher;
// robot's index in ROS ecosystem (if multiple instances)
std::string tf_prefix = "0";

void callback_val(const sensor_msgs::JointState::ConstPtr &msg) {

  sensor_msgs::JointState msg_out;
  std_msgs::Header header;         // creating header
  header.stamp = ros::Time::now(); // current time of data collection
  header.frame_id = tf_prefix + "/base/joint0";
  // filling new output msg with data from vrep
  msg_out.header = move(header);
  msg_out.effort = msg->effort;
  msg_out.position = msg->position;
  msg_out.velocity = msg->velocity;
  // publish float 64 of joint velocity meassured in Vrep
  publisher.publish(move(msg_out));
}

int main(int argc, char **argv) {
  // init node name as accelerometer
  ros::init(argc, argv, "leftMotorGet");
  // set relative node namespace
  ros::NodeHandle n("~");
  int vrep_no = 0; // robot's index in vrep (if multiple instances)
  n.getParam("vrep_index", vrep_no);
  n.getParam("tf_prefix", tf_prefix);
  // read msg from vrep topic
  ros::Subscriber sub_val = n.subscribe("/vrep/i" + std::to_string(vrep_no) +
                                            "_Pioneer_p3dx_leftMotor/getState",
                                        1000, callback_val);
  publisher = n.advertise<sensor_msgs::JointState>("getState", 1000);
  ROS_INFO("HAL(VREP): Left motor state msg publisher node initialized");

  // run event loop
  ros::spin();
  return 0;
}
#endif
