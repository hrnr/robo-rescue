#include "ros/ros.h"

#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/transform_listener.h>

#include <string>
#include <utility>

// robot's index in ROS ecosystem (if multiple instances)
int robot_id = 0;

// publisher in IMU for imu message
ros::Publisher publisher;

double WHEELS_HALF_SPACING = 0;
double WHEEL_RADIUS = 0.1;

double pos_x = 0;
double pos_y = 0;
double pos_th = 0;
ros::Time last_time;

// calculates inverse kinematic value of robot heading with differencial drive
double getYaw(const double velRight, const double velLeft) {
  return WHEEL_RADIUS * (velRight - velLeft) / (2*WHEELS_HALF_SPACING);
}

void setWheelSpacing() {
  // tf listener for calculation of wheel spacing
  tf::TransformListener listener;
  tf::StampedTransform transform;
  // get length in between wheels
  try {
    listener.waitForTransform(std::to_string(robot_id) + "/base/joint1",
                              std::to_string(robot_id) + "/base/joint0",
                              ros::Time::now(), ros::Duration(3.0));
    listener.lookupTransform(std::to_string(robot_id) + "/base/joint1",
                             std::to_string(robot_id) + "/base/joint0",
                             ros::Time(0), transform);
    WHEELS_HALF_SPACING = transform.getOrigin().absolute().getY() / 2;
    ROS_INFO("DPL: motor_controler WHEELS_HALF_SPACING: %f",
             WHEELS_HALF_SPACING);

  } catch (tf::TransformException ex) {
    ROS_ERROR("Problem with transformation between wheels: %s", ex.what());
  }
}

void odomData_cb(const sensor_msgs::JointState::ConstPtr &left_motor_msg,
                 const sensor_msgs::JointState::ConstPtr &right_motor_msg) {
  nav_msgs::Odometry odom_msg;
  double yaw;
  double x_speed;
  double dt;

  dt = (left_motor_msg->header.stamp - last_time).toSec();
  // y_speed = 0
  x_speed = WHEEL_RADIUS *
            (right_motor_msg->velocity[0] + left_motor_msg->velocity[0]) / 2;
  yaw = getYaw(right_motor_msg->velocity[0], left_motor_msg->velocity[0]);
  // move old point in 2D space to new predicted position and an angle
  pos_x += (x_speed * cos(pos_th)) * dt;
  pos_y += (x_speed * sin(pos_th)) * dt;
  pos_th += yaw * dt;
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pos_th);

  // create odom msg
  odom_msg.header.frame_id = std::to_string(robot_id) + "/odom";
  odom_msg.header.stamp = last_time;
  odom_msg.pose.pose.position.x = pos_x;
  odom_msg.pose.pose.position.y = pos_y;
  odom_msg.pose.pose.orientation = odom_quat;
  odom_msg.child_frame_id = std::to_string(robot_id) + "/base_link";
  odom_msg.twist.twist.linear.x = x_speed;
  odom_msg.twist.twist.angular.z = yaw;
 
  publisher.publish(odom_msg);

  last_time = left_motor_msg->header.stamp;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "odometryProvider");
  ros::NodeHandle n;
  n.getParam("robot_id", robot_id);
  last_time = ros::Time::now();
  setWheelSpacing();

  // subscribers to motors' states and imu
  message_filters::Subscriber<sensor_msgs::JointState> left_motor_sub(
      n, "hal/leftMotor/getState", 1000);
  message_filters::Subscriber<sensor_msgs::JointState> right_motor_sub(
      n, "hal/rightMotor/getState", 1000);

  // sync messages using approximate alghorithm
  constexpr int allowed_delay = 10;
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::JointState, sensor_msgs::JointState> OdomSyncPolicy;
  message_filters::Synchronizer<OdomSyncPolicy> odom_processor(
      OdomSyncPolicy(allowed_delay), left_motor_sub, right_motor_sub);
  odom_processor.registerCallback(odomData_cb);

  // publish fused message
  publisher = n.advertise<nav_msgs::Odometry>("odom", 1000);

  ROS_INFO("DPL: imuProvider initialized");

  // runs event loop
  ros::spin();

  return 0;
}
