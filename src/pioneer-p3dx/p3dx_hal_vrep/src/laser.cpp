#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <string>

// publisher in HAL for laser
ros::Publisher publisher;

void laserScanData_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  publisher.publish(msg);
}

int main(int argc, char **argv)
{
  // @todo work with multiple instances
  ros::init(argc, argv, "laserScan");
  ros::NodeHandle n("~"); // we want relative namespace

  // init parameters
  int robot_no = 0; // robot's index in vrep (if multiple instances)
  n.getParam("vrep_index", robot_no);

  // subscribe to laser messages from vrep
  // @todo mltiple instances
  ros::Subscriber vrep_subsriber = n.subscribe("/vrep/i" + std::to_string(robot_no) + 
    "_Pioneer_p3dx_laserScanner", 1000, laserScanData_cb);
  // will publish the laser mesages to the rest of the stack
  publisher = n.advertise<sensor_msgs::LaserScan>("sensor" + std::to_string(robot_no) + "/laser", 1000);

  ROS_INFO("HAL(VREP): Laser node initialized");

  // runs event loop
  ros::spin();

  return 0;
}
