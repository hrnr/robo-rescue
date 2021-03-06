#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <string>

// publisher joint state from vrep to joint state topic in ros
ros::Publisher pub_toRos;
// publisher from ros motor controler to vrep motor controler
ros::Publisher pub_toVrep;
// robot's index in ROS ecosystem (if multiple instances)
std::string tf_prefix;

void stateCallback(const sensor_msgs::JointState::ConstPtr &msg) {

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
  pub_toRos.publish(move(msg_out));
}

void velCallback(const std_msgs::Float64::ConstPtr &msg) {
  // publish float 64 of value velocity of left wheel
  pub_toVrep.publish(msg);
}



int main(int argc, char **argv) {
  // init node name as accelerometer
  ros::init(argc, argv, "leftMotor");
  // set relative node namespace
  ros::NodeHandle n("~");

  // init parameters
  // resolve `tf_prefix` and `vrep_index` in parent namespaces
  int vrep_no = 0; // robot's index in vrep (if multiple instances)

  std::string tf_prefix_path;
  if (n.searchParam("tf_prefix", tf_prefix_path))
  {
    n.getParam(tf_prefix_path, tf_prefix);
  }

  std::string vrep_index_path;
  if (n.searchParam("vrep_index", vrep_index_path))
  {
    n.getParam(vrep_index_path, vrep_no);
  }
  
  // read msg from vrep topic joint state and publishing it to ROS
  ros::Subscriber sub_from_vrep=n.subscribe("/vrep/i" + std::to_string(vrep_no) +
                                            "_Pioneer_p3dx_leftMotor/getState",
                                        1000, stateCallback);
  pub_toRos = n.advertise<sensor_msgs::JointState>("getState", 1000);
  ROS_INFO("HAL(VREP): Left motor state msg publisher node initialized");

  // publishing velocity from ROS to vrep
  pub_toVrep = n.advertise<std_msgs::Float64>(
      "/vrep/i" + std::to_string(vrep_no) + "_Pioneer_p3dx_leftMotor/setVel",
      1000);
  ros::Subscriber sub_from_ros = n.subscribe("setVel", 1000, velCallback);
  ROS_INFO("HAL(VREP): Left motor publisher to vrep: velocity msg initialized");

  // run event loop
  ros::spin();
  return 0;
}
