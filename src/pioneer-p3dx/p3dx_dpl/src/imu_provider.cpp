#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
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
  sensor_msgs::Imu imu_msg;
  // curently we have no orientation data (missing compass)
  imu_msg.orientation_covariance[0] = -1;

  /* missing tf transform @todo lukas! */

  // data from accelerometer, uknown covarince matrix ATM
  imu_msg.linear_acceleration.x = accel_msg->vector.x;
  imu_msg.linear_acceleration.y = accel_msg->vector.y;
  imu_msg.linear_acceleration.z = accel_msg->vector.z;

  // data from gyroscope, uknown covarince matrix ATM
  imu_msg.angular_velocity.x = gyro_msg->vector.x;
  imu_msg.angular_velocity.y = gyro_msg->vector.y;
  imu_msg.angular_velocity.z = gyro_msg->vector.z;

  // header
  imu_msg.header.stamp = ros::Time::now(); // current time of data collection
  imu_msg.header.frame_id = std::to_string(robot_id) + "/base_link";

  publisher.publish(std::move(imu_msg));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imuProvider");
  ros::NodeHandle n; // we want relative namespace

  // init parameters
  n.getParam("robot_id", robot_id);

  // subscribers to acceleromer and gyroscope
  message_filters::Subscriber<geometry_msgs::Vector3Stamped> accel_sub (n, "hal/accelerometer/sensor0/axis_data", 1000);
  message_filters::Subscriber<geometry_msgs::Vector3Stamped> gyro_sub (n, "hal/gyro/sensor0/axis_data", 1000);

  // sync messages using approximate alghorithm
  constexpr int allowed_delay = 10;
  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::Vector3Stamped, geometry_msgs::Vector3Stamped> ImuSyncPolicy;
  message_filters::Synchronizer<ImuSyncPolicy> imu_processor (ImuSyncPolicy(allowed_delay), accel_sub, gyro_sub);
  imu_processor.registerCallback(imuData_cb);

  // publish fused message
  publisher = n.advertise<sensor_msgs::Imu>("Imu", 1000);

  ROS_INFO("DPL: imuProvider initialized");

  // runs event loop
  ros::spin();

  return 0;
}
