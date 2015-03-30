#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <string>
#include <utility>

// robot's index in ROS ecosystem (if multiple instances)
int robot_id = 0;

// publisher in IMU for imu message
ros::Publisher publisher;

/**
 * @brief Callback publishing imu messages
 * @details Fuses accelerometer and gyro messages to imu message
 * 
 */
void imuData_cb(const geometry_msgs::Vector3Stamped::ConstPtr& accel_msg, const geometry_msgs::Vector3Stamped::ConstPtr& gyro_msg)
{
  ROS_INFO("received");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imuProvider");
  ros::NodeHandle n; // we want relative namespace

  // init parameters
  n.getParam("robot_id", robot_id);

  // message filters used for fusing messages
  message_filters::Subscriber<geometry_msgs::Vector3Stamped> accel_sub (n, "hal/accelerometer/sensor0/axis_data", 500);
  message_filters::Subscriber<geometry_msgs::Vector3Stamped> gyro_sub (n, "hal/gyro/sensor0/axis_data", 500);
  message_filters::TimeSynchronizer<geometry_msgs::Vector3Stamped, geometry_msgs::Vector3Stamped> imu_processor (accel_sub, gyro_sub, 1000);
  imu_processor.registerCallback(imuData_cb);

  ROS_INFO("DPL: imuProvider initialized");

  // runs event loop
  ros::spin();

  return 0;
}
