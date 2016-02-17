
// ROS includes
#include "ros/ros.h"
#include "provider_sonar_node.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "micron_driver");

  ros::NodeHandle nh("~");
  provider_sonar::ProviderSonarNode tritech_micron(nh);

  ros::spin();

  return 0;
}
