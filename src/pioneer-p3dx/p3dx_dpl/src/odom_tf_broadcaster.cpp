#include "ros/ros.h"

#include <tf/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"
#include <string>

#define RATE 30

int robot_id=0;
double pos_x =0;
double pos_y =0;
geometry_msgs::Quaternion odom_quat;
ros::Time last_time;


void odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
  last_time=msg->header.stamp;
  pos_x = msg->pose.pose.position.x;
  pos_y = msg->pose.pose.position.y;
  odom_quat = msg->pose.pose.orientation;
}
void time_cb(tf::TransformBroadcaster & odom_broadcast, const ros::TimerEvent& tm){
   geometry_msgs::TransformStamped odom_trans;
   odom_trans.header.stamp = last_time;
   odom_trans.header.frame_id = std::to_string(robot_id) + "/odom";
   odom_trans.child_frame_id = std::to_string(robot_id) + "/base_link";
   odom_trans.transform.translation.x = pos_x;
   odom_trans.transform.translation.y = pos_y;
   odom_trans.transform.translation.z = 0.0;
   odom_trans.transform.rotation = odom_quat;
   odom_broadcast.sendTransform(std::move(odom_trans));
}

int main(int argc, char ** argv){

  ros::init(argc, argv, "odom_tf_broadcaster");
  ros::NodeHandle n;
  n.getParam("robot_id", robot_id);
   // broadcaster of odom frame_id to /tf
  tf::TransformBroadcaster odom_broadcaster;
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);
  last_time = ros::Time::now();

  
  boost::function<void(const ros::TimerEvent&)> time_callback = 
      std::bind(time_cb, std::ref(odom_broadcaster), std::placeholders::_1);
  
  // 5 Hz  5 msgs per second
  ros::Timer timer = n.createTimer(ros::Duration(1.0/RATE), std::move(time_callback));
  ros::Subscriber subsc = n.subscribe("odom", 1000, odom_cb);
  ros::spin(); 
  
  return 0;
}