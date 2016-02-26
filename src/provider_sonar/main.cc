
// ROS includes
#include "ros/ros.h"
#include "provider_sonar/provider_sonar_node.h"
#include "provider_sonar/scanline_converter.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "provider_sonar_node");

  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));
  provider_sonar::ProviderSonarNode providerSonarNode(nh);
  provider_sonar::ScanLineConverter scanline_converter(nh);

  ros::spin();

  return 0;
}
