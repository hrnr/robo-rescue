#include "ros/ros.h"
#include "vrep_common/VrepInfo.h"
#include "rosgraph_msgs/Clock.h"

#include <string>

constexpr unsigned int SIM_STARTED_MASK = 1;
ros::Publisher clock_publisher;
ros::Subscriber vrep_clock_subscriber;

float sim_time = -1;

void updateClock(const vrep_common::VrepInfo::ConstPtr& new_time) {
  /*
    simulatorState.data is bitwise encoded state
    bit0 set: simulation not stopped
    bit1 set: simulation paused
    bit2 set: real-time switch on
    bit3-bit5: the edit mode type (0=no edit mode, 1=triangle, 2=vertex, 3=edge, 4=path, 5=UI)
  */
  if (!(new_time->simulatorState.data & SIM_STARTED_MASK)) {
    return;
  }

  sim_time = new_time->simulationTime.data;
  rosgraph_msgs::Clock c;
  /* convert float time to sec a nano sec */
  c.clock.sec = static_cast<uint32_t>(sim_time);
  c.clock.nsec = static_cast<uint32_t>((sim_time - c.clock.sec) * 1000000000);

  clock_publisher.publish(c);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "vrepClockServer");
  ros::NodeHandle nh;
  vrep_clock_subscriber = nh.subscribe("/vrep/info", 1, updateClock);
  clock_publisher = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);
  ros::spin();
}
