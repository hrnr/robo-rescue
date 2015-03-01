#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

// publisher in HAL for laser
ros::Publisher publisher;

void laserScanInfo_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO("Received scan");
  publisher.publish(msg);
  ROS_INFO("Publisher");
}

int main(int argc, char **argv)
{
  // @todo work with multiple instances
  ros::init(argc, argv, "p3dx");
  ros::NodeHandle n;

  // subscribe to laser messages from vrep
  // @todo mltiple instances
  ros::Subscriber vrep_subsriber = n.subscribe("/vrep/i0_Pioneer_p3dx_laserScanner", 1000, laserScanInfo_cb);
  // will publish the laser mesages to the rest of the stack
  publisher = n.advertise<sensor_msgs::LaserScan>("/p3dx/laserScan/0/laser", 1000);

  ROS_INFO("HAL: Laser node initialized");

  // runs event loop
  ros::spin();

  return 0;
}