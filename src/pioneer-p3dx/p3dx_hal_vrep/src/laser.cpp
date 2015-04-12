#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <string>
#include <utility>

// publisher in HAL for laser
ros::Publisher publisher;

// robot's index in ROS ecosystem (if multiple instances)
std::string tf_prefix;

void laserScanData_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  sensor_msgs::LaserScan laser_msg = *msg;

  // include proper ros time and proper tf info
  laser_msg.header.stamp = ros::Time::now(); // current time of data collection
  laser_msg.header.frame_id = tf_prefix + "/laserSensors/sensor0";

  publisher.publish(std::move(laser_msg));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserScan");
  ros::NodeHandle n("~"); // we want relative namespace

  // init parameters
  int vrep_no = 0; // robot's index in vrep (if multiple instances)
  n.getParam("vrep_index", vrep_no);

  std::string tf_prefix_path;
  if (n.searchParam("tf_prefix", tf_prefix_path))
  {
    n.getParam(tf_prefix_path, tf_prefix);
  }

  // subscribe to laser messages from vrep
  ros::Subscriber vrep_subsriber = n.subscribe("/vrep/i" + std::to_string(vrep_no) +
    "_Pioneer_p3dx_laserScanner", 1000, laserScanData_cb);

  // will publish the laser mesages to the rest of the stack
  // only 1 laserScanner present
  publisher = n.advertise<sensor_msgs::LaserScan>("sensor0/LaserScan", 1000);

  ROS_INFO("HAL(VREP): Laser node initialized");

  // runs event loop
  ros::spin();

  return 0;
}
