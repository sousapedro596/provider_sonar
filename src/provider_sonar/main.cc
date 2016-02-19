
// ROS includes
#include "ros/ros.h"
#include "provider_sonar/provider_sonar_node.h"
#include "provider_sonar/scanline_converter.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "micron_driver");

  ros::NodeHandle nh("~");
  provider_sonar::ProviderSonarNode tritech_micron(nh);
  provider_sonar::ScanLineConverter scanline_converter(nh);

  ros::spin();

  return 0;
}
