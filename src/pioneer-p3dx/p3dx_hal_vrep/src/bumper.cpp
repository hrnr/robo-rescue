#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/Int32.h"
#include <string>
#include <utility>
#include <limits>

// vrep bounds for bumper sensor indexing
#define VREP_LOWER_BOUND 1
#define VREP_UPPER_BOUND 10
#define VREP_END VREP_UPPER_BOUND - VREP_LOWER_BOUND + 1
// parameters for sensor defined in vrep
#define VREP_ANGLE_DETECTION 0.5236 // in [rad]
#define VREP_MIN_DIST_DETECTION 0.0
#define VREP_MAX_DIST_DETECTION 0.0

// robot's index in ROS ecosystem (if multiple instances)
std::string tf_prefix;  

/**
 * @brief Callback processing ulrasonic vrep messages
 * @details Transform vrep bumper message to proper ROS range message
 * 
 * @param msg message from vrep
 * @param sensor_n [description]
 * @param publisher relevat publisher for this messages
 */
void bumperSensorData_cb(const ros::Publisher& publisher, std::size_t sensor_n, const std_msgs::Int32::ConstPtr& msg)
{
  sensor_msgs::Range range_msg;

  // data
  range_msg.radiation_type = 3; // this is bumper
  range_msg.field_of_view = VREP_ANGLE_DETECTION;
  range_msg.min_range = VREP_MIN_DIST_DETECTION;
  range_msg.max_range = VREP_MAX_DIST_DETECTION;

  constexpr auto inf = std::numeric_limits<decltype(range_msg.range)>::infinity();
  if(msg->data) // detected collision
    range_msg.range = -inf;
  else 
    range_msg.range = inf;

  // header
  range_msg.header.stamp = ros::Time::now(); // current time of data collection
  range_msg.header.frame_id = tf_prefix + "/bumperSensors/sensor" + std::to_string(sensor_n);

  publisher.publish(std::move(range_msg));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bumperSensor");
  ros::NodeHandle n("~"); // we want relative namespace

  // init parameters
  int vrep_no = 0; // robot's index in vrep (if multiple instances)
  n.getParam("vrep_index", vrep_no);
  n.getParam("tf_prefix", tf_prefix);

  // will publish the bumper mesages to the rest of the stack
  std::vector<ros::Publisher> hal_publishers;
  hal_publishers.reserve(VREP_END);
  for (int i = 0; i < VREP_END; ++i)
  {
    hal_publishers.push_back(
      n.advertise<sensor_msgs::Range>(
        "sensor" + std::to_string(i) + "/Range", 
        1000));
  }

  // subscribe to to all bumpers from vrep (16)
  std::vector<ros::Subscriber> vrep_subscribers;
  vrep_subscribers.reserve(VREP_END);

  auto it = hal_publishers.cbegin();
  std::size_t vrep_i = VREP_LOWER_BOUND;
  for (; it != hal_publishers.cend(); ++vrep_i, ++it)
  {
    // prepare function for this sensor
    boost::function<void(const std_msgs::Int32::ConstPtr&)> callback = 
      std::bind(bumperSensorData_cb, *it, std::distance(hal_publishers.cbegin(), it), std::placeholders::_1);
    // subscribe each callback
    vrep_subscribers.push_back(
      n.subscribe(
        "/vrep/i" + std::to_string(vrep_no) + "_Pioneer_p3dx_bumper" + std::to_string(vrep_i),
        1000,
        std::move(callback)
      ));
  }

  ROS_INFO("HAL(VREP): BumperSensor node initialized");

  // runs event loop
  ros::spin();

  return 0;
}
