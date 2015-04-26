#include "ros/ros.h"

#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/transform_listener.h>

#include <string>
#include <utility>
#include <math.h>

// robot's index in ROS ecosystem (if multiple instances)
std::string tf_prefix;

// publisher in IMU for imu message
ros::Publisher publisher;

double WHEELS_HALF_SPACING = 0;
double WHEEL_RADIUS = 0.098;

double pos_x = 0;
double pos_y = 0;
double pos_th = 0;
double left_steps=-5; // range from -PI to PI
double right_steps=-5;
bool dir_left = true; // direction of rotation of left wheel true== forward
bool dir_right = true; // direction of rotation of right wheel true== forward
ros::Time last_time;

double left_angular = 0;
double right_angular = 0;

std::string odom_frame_id="/odom";
std::string odom_publish_topic="odom";
std::string odom_right_wheel="hal/rightMotor/getState";
std::string odom_left_wheel="hal/leftMotor/getState";

// calculates inverse kinematic value of robot heading with differencial drive
double getYaw(const double velRight, const double velLeft) {
  return (WHEEL_RADIUS * (velRight - velLeft)) / (2*WHEELS_HALF_SPACING);
}

void setWheelSpacing(const std::string & motor_left_frame,const std::string & motor_right_frame) {
  // tf listener for calculation of wheel spacing
  tf::TransformListener listener;
  tf::StampedTransform transform;
  // get length in between wheels
  try {
    listener.waitForTransform(tf_prefix + motor_left_frame,
                              tf_prefix + motor_right_frame,
                              ros::Time::now(), ros::Duration(3.0));
    listener.lookupTransform(tf_prefix + motor_left_frame,
                             tf_prefix + motor_right_frame,
                             ros::Time(0), transform);
    WHEELS_HALF_SPACING = transform.getOrigin().absolute().getY() / 2;
    ROS_INFO("DPL: motor_controler WHEELS_HALF_SPACING: %f",
             WHEELS_HALF_SPACING);

  } catch (tf::TransformException ex) {
    ROS_ERROR("Problem with transformation between wheels: %s", ex.what());
  }
}
//! @brief Calculates angular velocity from old and new angle on joint
//! @param[in] old_steps - angle on joint in previous iteration
//! @param[in] new_steps - current angle on joint
//! @param[in] dir - direction of rotation true=forward, false = backward
//! @param[in] dt - time elapsed between measurement of old_steps and new_steps
//! @return Current angular velocity on the joint
//!
double getAngularVel(const double old_steps, const double new_steps, const bool dir,const double dt){
  if(dir && old_steps > 0 && new_steps < 0){
    // overflow from PI --> -PI
    return (2* M_PI - (old_steps - new_steps)) / dt;
  }else if (!dir && old_steps < 0 && new_steps > 0){
    // overflow in negative direction from -PI --> PI
    return (-2*M_PI + (new_steps - old_steps)) / dt;
  }
  return (new_steps-old_steps) / dt;
}

void odomData_cb(const sensor_msgs::JointState::ConstPtr &left_motor_msg,
                 const sensor_msgs::JointState::ConstPtr &right_motor_msg) {
  nav_msgs::Odometry odom_msg;
  double yaw;
  double x_speed;
  double dt = (left_motor_msg->header.stamp - last_time).toSec();

  // calculate andular velocities from angular diplacement in both joints
  if(left_steps > -4 || right_steps > -4){
    if(dt != 0){
      left_angular = getAngularVel(left_steps, left_motor_msg->position[0],dir_left,dt);
      right_angular = getAngularVel(right_steps, right_motor_msg->position[0],dir_right,dt);
    }
  }
  if(left_angular > 0)
    dir_left = true;
  else
    dir_left = false;
  if (right_angular > 0)
    dir_right = true;
  else
    dir_right = false;
  right_steps = right_motor_msg->position[0];
  left_steps = left_motor_msg->position[0];
  // calculate robot's forward speed and yaw
  x_speed = (WHEEL_RADIUS *
            (left_angular + right_angular)) / 2;
  yaw = getYaw(right_angular, left_angular);

  // move old point in 2D space to new predicted position and an angle
  if(dt != 0){
    pos_x += (x_speed * cos(pos_th)) * dt;
    pos_y += (x_speed * sin(pos_th)) * dt;
    pos_th += yaw * dt;
  }

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pos_th);

  // create odom msg
  odom_msg.header.frame_id = tf_prefix + odom_frame_id;
  odom_msg.header.stamp = last_time;
  odom_msg.pose.pose.position.x = pos_x;
  odom_msg.pose.pose.position.y = pos_y;
  odom_msg.pose.pose.orientation = odom_quat;
  odom_msg.child_frame_id = tf_prefix + "/base_footprint";
  odom_msg.twist.twist.linear.x = x_speed;
  odom_msg.twist.twist.angular.z = yaw;
  // add covariance
  double c =10e-4;
  odom_msg.pose.covariance={c,0,0,0,0,0,
                            0,c,0,0,0,0,
                            0,0,c,0,0,0,
                            0,0,0,c,0,0,
                            0,0,0,0,c,0,
                            0,0,0,0,0,c};
  odom_msg.twist.covariance={c,0,0,0,0,0,
                             0,c,0,0,0,0,
                             0,0,c,0,0,0,
                             0,0,0,c,0,0,
                             0,0,0,0,c,0,
                             0,0,0,0,0,c};
  publisher.publish(odom_msg);

  last_time = left_motor_msg->header.stamp;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "odometryProvider");
  ros::NodeHandle n;
  std::string frame_id_left_wheel="/base/joint1";
  std::string frame_id_right_wheel="/base/joint0";

  // get parameters
  std::string tf_prefix_path;
  if (n.searchParam("tf_prefix", tf_prefix_path))
  {
    n.getParam(tf_prefix_path, tf_prefix);
  }
  ros::param::get("~leftWheelTopic", odom_left_wheel);
  ros::param::get("~rightWheelTopic", odom_right_wheel);
  ros::param::get("~odomPublTopic", odom_publish_topic);
  ros::param::get("~frameIDLeftWheel", frame_id_left_wheel);
  ros::param::get("~frameIDRightWheel", frame_id_right_wheel);
  ros::param::get("~frameIDodom",  odom_frame_id);

  last_time = ros::Time::now();
  setWheelSpacing(frame_id_left_wheel,frame_id_right_wheel);

  // subscribers to motors' states and imu
  message_filters::Subscriber<sensor_msgs::JointState> left_motor_sub(
      n, odom_left_wheel, 1000);
  message_filters::Subscriber<sensor_msgs::JointState> right_motor_sub(
      n, odom_right_wheel, 1000);

  // sync messages using approximate alghorithm
  constexpr int allowed_delay = 10;
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::JointState, sensor_msgs::JointState> OdomSyncPolicy;
  message_filters::Synchronizer<OdomSyncPolicy> odom_processor(
      OdomSyncPolicy(allowed_delay), left_motor_sub, right_motor_sub);
  odom_processor.registerCallback(odomData_cb);

  // publish fused message
  publisher = n.advertise<nav_msgs::Odometry>(odom_publish_topic, 1000);

  ROS_INFO("DPL: OdomProvider initialized");

  // runs event loop
  ros::spin();

  return 0;
}
