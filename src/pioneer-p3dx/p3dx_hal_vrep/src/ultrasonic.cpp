#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/Float32.h"
#include <string>
#include <sstream>
#include <cmath>
#include <utility>

// vrep bounds for ultrasonic sensor indexing
#define VREP_LOWER_BOUND 1
#define VREP_UPPER_BOUND 16
#define VREP_END VREP_UPPER_BOUND - VREP_LOWER_BOUND + 1
// parameters for sensor defined in vrep
#define VREP_ANGLE_DETECTION 0.5236 // in [rad]
#define VREP_MIN_DIST_DETECTION 0.0500
#define VREP_MAX_DIST_DETECTION 1.0

// robot's index in ROS ecosystem (if multiple instances)
int robot_id = 0;  

/**
 * @brief Callback processing ulrasonic vrep messages
 * @details Transform vrep ultrasonic message to proper ROS range message
 * 
 * @param msg message from vrep
 * @param sensor_n [description]
 * @param publisher relevat publisher for this messages
 */
void ultrasonicSensorData_cb(const ros::Publisher& publisher, std::size_t sensor_n, const std_msgs::Float32::ConstPtr& msg)
{
  if(msg->data <= VREP_MAX_DIST_DETECTION && msg->data >= VREP_MIN_DIST_DETECTION) {
    sensor_msgs::Range range_msg;

    // data
    range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg.field_of_view = VREP_ANGLE_DETECTION;
    range_msg.min_range = VREP_MIN_DIST_DETECTION;
    range_msg.max_range = VREP_MAX_DIST_DETECTION;
    range_msg.range = msg->data;

    // header
    range_msg.header.stamp = ros::Time::now(); // current time of data collection
    range_msg.header.frame_id = std::to_string(robot_id) + "/ultrasonicSensors/sensor" + std::to_string(sensor_n);

    publisher.publish(std::move(range_msg));
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ultrasonicSensor");
  ros::NodeHandle n("~"); // we want relative namespace

  // init parameters
  int vrep_no = 0; // robot's index in vrep (if multiple instances)
  n.getParam("vrep_index", vrep_no);
  n.getParam("robot_id", robot_id);

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
      std::bind(ultrasonicSensorData_cb, *it, std::distance(hal_publishers.cbegin(), it), std::placeholders::_1);
    // subscribe each callback
    vrep_subscribers.push_back(
      n.subscribe(
        "/vrep/i" + std::to_string(vrep_no) + "_Pioneer_p3dx_ultrasonicSensor" + std::to_string(vrep_i),
        1000,
        std::move(callback)
      ));
  }

  ROS_INFO("HAL(VREP): UltrasonicSensor node initialized");

  // runs event loop
  ros::spin();

  return 0;
}
