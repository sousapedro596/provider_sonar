
// ROS includes
#include "provider_sonar/provider_sonar_node.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "provider_sonar_node");
  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));
  provider_sonar::ProviderSonarNode psn(nh);
  psn.Spin();
  return 0;
}
