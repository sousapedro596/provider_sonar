/**
 * \file	sonar_node.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author  Pierluc Bédard <blindmiror@gmail.com>
 * \author  Francis Masse <francis.masse05@gmail.com>
 * \date	11/02/2016
 *
 * \copyright Copyright (c) 2016 Copyright (C) 2011 Randolph Voorhies
 *
 * \section LICENSE http://www.gnu.org/licenses/gpl-3.0.en.html
 *
 * Changes by: S.O.N.I.A.
 * \copyright Copyright (c) 2016 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef PROVIDER_SONAR_SONAR_NODE_H
#define PROVIDER_SONAR_SONAR_NODE_H

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <provider_sonar/ProviderSonarConfiguration.h>
#include <provider_sonar/ScanLine.h>
#include <provider_sonar/SonarReconfiguration.h>
#include <provider_sonar/SimulationReconfiguration.h>
#include <provider_sonar/PointCloudReconfiguration.h>
#include <sstream>
#include "stdint.h"
#include <sensor_msgs/PointCloud2.h>
#include "sonar_configuration.h"
#include <provider_sonar/Serial.h>
#include <provider_sonar/sonar_driver.h>
#include <lib_atlas/maths/numbers.h>

namespace provider_sonar {

class ProviderSonarNode {
 public:
  //============================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<ProviderSonarNode>;
  using ConstPtr = std::shared_ptr<const ProviderSonarNode>;
  using PtrList = std::vector<ProviderSonarNode::Ptr>;
  using ConstPtrList = std::vector<ProviderSonarNode::ConstPtr>;

  typedef ScanLine ScanlineMsgType;
  typedef IntensityBin IntensityBinMsgType;
  typedef float StepType;
  typedef float AngleType;
  typedef std::vector<uint8_t> IntensityBinsRawType;

  //============================================================================
  // P U B L I C   C / D T O R S

  ProviderSonarNode(ros::NodeHandlePtr &nh);

  ~ProviderSonarNode();

  //============================================================================
  // P U B L I C   M E T H O D S

  void Spin();

  bool SonarReconfiguration(
      provider_sonar::SonarReconfiguration::Request &req,
      provider_sonar::SonarReconfiguration::Response &resp);

  bool SimulationReconfiguration(
      provider_sonar::SimulationReconfiguration::Request &req,
      provider_sonar::SimulationReconfiguration::Response &resp);

  bool PointCloudReconfiguration(
      provider_sonar::PointCloudReconfiguration::Request &req,
      provider_sonar::PointCloudReconfiguration::Response &resp);

  void PublishProviderSonarConfiguration(uint8_t n_bin, float range, float vos,
      uint8_t angle_step_size, uint16_t left_limit, uint16_t right_limit,
      uint8_t ad_span, uint8_t ad_low);

  /**
   * This function is a callback associated with the reception of a scanline
   * message from the sonar.
   *
   * It takes the sonar scanline, formats it as a scanline message and publishes
   * it on the corresponding topic.
   *
   * \param scan_angle  Current angle of the sonar head
   * \param bin_distance_step  The distance between bins
   * \param intensity_bins  Vector containing all bins intensity
   */
  void PublishPointCloud2(AngleType scan_angle, StepType bin_distance_step,
                          IntensityBinsRawType intensity_bins);

  void Simulate();

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  ros::NodeHandlePtr nh_;
  ros::Publisher point_cloud2_pub_;
  ros::Publisher sonar_configuration_pub_;
  ros::ServiceServer sonar_reconfig_server_;
  ros::ServiceServer simulation_reconfig_server_;
  ros::ServiceServer point_cloud_reconfig_server_;

  SonarConfiguration config_;
  SonarDriver *driver_;

  float scan_angle_;
};
}  // namespace provider_sonar

#endif  // PROVIDER_SONAR_SONAR_NODE_H