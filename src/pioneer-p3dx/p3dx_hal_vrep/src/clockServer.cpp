
#include "ros/ros.h"
#include "vrep_common/VrepInfo.h"
#include "rosgraph_msgs/Clock.h"

#include <string>

const int SIM_STARTED = 1;
ros::Publisher clock_publisher;
ros::Subscriber vrep_clock_subscriber;

float sim_time = -1;

void updateClock(const vrep_common::VrepInfo::ConstPtr & new_time) {
  if (SIM_STARTED == new_time->simulatorState.data) {
    sim_time = new_time->simulationTime.data;
    rosgraph_msgs::Clock c;
    c.clock.sec = static_cast<uint32_t>(sim_time);
    c.clock.nsec = static_cast<uint32_t>((sim_time - c.clock.sec) * 1000000000);
    clock_publisher.publish(c);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "vrepClockServer");
  ros::NodeHandle nh;
  vrep_clock_subscriber = nh.subscribe("/vrep/info", 1, updateClock);
  clock_publisher = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);
  ros::spin();
}
