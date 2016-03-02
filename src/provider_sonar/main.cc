
// ROS includes
#include "ros/ros.h"
#include "provider_sonar/provider_sonar_node.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "provider_sonar_node");
  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));
  provider_sonar::ProviderSonarNode psn(nh);
  psn.Spin();
  return 0;
}
