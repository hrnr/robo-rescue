#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>
#include <string>
#include <utility>

// robot's index in ROS ecosystem (if multiple instances)
std::string tf_prefix;

// publisher in IMU for imu message
ros::Publisher publisher;

// listener for tf transforms from gyroscope and accelerometer
tf::TransformListener *tf_listener;

/**
 * @brief Callback publishing imu messages
 * @details Fuses accelerometer and gyro messages to imu message
 * 
 */
void imuData_cb(const geometry_msgs::Vector3Stamped::ConstPtr& accel_msg, const geometry_msgs::Vector3Stamped::ConstPtr& gyro_msg)
{
  sensor_msgs::Imu imu_msg;
  geometry_msgs::Vector3Stamped transformed_vec;
  // target frame for whole message
  std::string target_frame = tf_prefix + "/base_link";

  // curently we have no orientation data (missing compass)
  imu_msg.orientation_covariance[0] = -1;

  // data from accelerometer, uknown covarince matrix ATM
  try
  {
    // transform accel data
    tf_listener->transformVector(target_frame, *accel_msg, transformed_vec);
    imu_msg.linear_acceleration = transformed_vec.vector;
  } catch (const tf::TransformException& ex) {
    ROS_WARN("can't transform accelerometer message: %s", ex.what());
  }

  // data from gyroscope, uknown covarince matrix ATM
  try
  {
    // transform gyro data
    tf_listener->transformVector(target_frame, *gyro_msg, transformed_vec);
    imu_msg.angular_velocity = transformed_vec.vector;
  } catch (const tf::TransformException& ex) {
    ROS_WARN("can't transform gyroscope message: %s", ex.what());
  }

  // header
  imu_msg.header.stamp = ros::Time::now(); // current time of data collection
  imu_msg.header.frame_id = target_frame;

  publisher.publish(std::move(imu_msg));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imuProvider");
  ros::NodeHandle n; // we want relative namespace

  // init parameters
  n.getParam("tf_prefix", tf_prefix);

  // initialize tf_listener
  tf::TransformListener tf_listener_;
  tf_listener = &tf_listener_;

  // subscribers to acceleromer and gyroscope
  message_filters::Subscriber<geometry_msgs::Vector3Stamped> accel_sub (n, "hal/accelerometer/sensor0/axis_data", 1000);
  message_filters::Subscriber<geometry_msgs::Vector3Stamped> gyro_sub (n, "hal/gyro/sensor0/axis_data", 1000);

  // sync messages using approximate alghorithm
  constexpr int sync_delay = 50;
  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::Vector3Stamped, geometry_msgs::Vector3Stamped> ImuSyncPolicy;
  message_filters::Synchronizer<ImuSyncPolicy> imu_processor (ImuSyncPolicy(sync_delay), accel_sub, gyro_sub);
  imu_processor.registerCallback(imuData_cb);

  // publish fused message
  publisher = n.advertise<sensor_msgs::Imu>("imu_data", 1000);

  ROS_INFO("DPL: imuProvider initialized");

  // runs event loop
  ros::spin();

  return 0;
}
