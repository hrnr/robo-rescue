#include "ros/ros.h"
#include "vrep_common/ProximitySensorData.h"
#include "sensor_msgs/Range.h"
#include <string>
#include <sstream>

// publisher in HAL for laser
ros::Publisher publisher;

void ultrasonicSensorData_cb(const vrep_common::ProximitySensorData::ConstPtr& msg)
{
  auto detectedPoint = msg->detectedPoint;
  auto normalVector = msg->normalVector;
  // publisher.publish(msg);
  std::stringstream s;
  s << detectedPoint << " aaa " << normalVector;
  ROS_INFO("%s", s.str().c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ultrasonicSensor");
  ros::NodeHandle n("~"); // we want relative namespace

  // init parameters
  int robot_no = 0; // robot's index in vrep (if multiple instances)
  n.getParam("vrep_index", robot_no);

  // subscribe to ultrasonic messages from vrep
  ros::Subscriber vrep_subsriber = n.subscribe("/vrep/i" + std::to_string(robot_no) + 
    "_Pioneer_p3dx_ultrasonicSensor1", 1000, ultrasonicSensorData_cb);

  // will publish the laser mesages to the rest of the stack
  publisher = n.advertise<sensor_msgs::Range>("sensor" + std::to_string(robot_no) + "/Range", 1000);

  ROS_INFO("HAL(VREP): UltrasonicSensor node initialized");

  // runs event loop
  ros::spin();

  return 0;
}
