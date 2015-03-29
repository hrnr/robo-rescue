#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/Float32.h"
#include <string>
#include <sstream>

// vrep bound for ultrasonic sensor indexing
#define VREP_LOWER_BOUND 1
#define VREP_UPPER_BOUND 16
#define VREP_END VREP_UPPER_BOUND - VREP_LOWER_BOUND + 1

/**
 * @brief Callback processing ulrasonic vrep messages
 * @details Transform vrep ultrasonic message to proper ROS range message
 * 
 * @param msg message from vrep
 * @param index [description]
 */
void ultrasonicSensorData_cb(const ros::Publisher& publisher, const std_msgs::Float32::ConstPtr& msg)
{
  ROS_INFO("received");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ultrasonicSensor");
  ros::NodeHandle n("~"); // we want relative namespace

  // init parameters
  int robot_no = 0; // robot's index in vrep (if multiple instances)
  n.getParam("vrep_index", robot_no);

  // will publish the ultrasonic mesages to the rest of the stack
  std::vector<ros::Publisher> hal_publishers;
  hal_publishers.reserve(VREP_END);
  for (int i = 0; i < VREP_END; ++i)
  {
    hal_publishers.push_back(
      n.advertise<sensor_msgs::Range>(
        "sensor" + std::to_string(i) + "/Range", 
        1000));
  }

  // subscribe to to all ultrasonics from vrep (16)
  std::vector<ros::Subscriber> vrep_subscribers;
  vrep_subscribers.reserve(VREP_END);

  auto it = hal_publishers.cbegin();
  std::size_t vrep_i = VREP_LOWER_BOUND;
  for (; it != hal_publishers.cend(); ++vrep_i, ++it)
  {
    // prepare function for this sensor
    boost::function<void(const std_msgs::Float32::ConstPtr&)> callback = 
      std::bind(ultrasonicSensorData_cb, *it, std::placeholders::_1);
    // subscribe each callback
    vrep_subscribers.push_back(
      n.subscribe(
        "/vrep/i" + std::to_string(robot_no) + "_Pioneer_p3dx_ultrasonicSensor" + std::to_string(vrep_i),
        1000,
        callback
      ));
  }

  ROS_INFO("HAL(VREP): UltrasonicSensor node initialized");

  // runs event loop
  ros::spin();

  return 0;
}
