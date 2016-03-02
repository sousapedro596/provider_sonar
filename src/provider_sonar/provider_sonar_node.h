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
#include <provider_sonar/ScanLine.h>
#include <provider_sonar/sonar_reconfiguration.h>
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
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<ProviderSonarNode>;
  using ConstPtr = std::shared_ptr<const ProviderSonarNode>;
  using PtrList = std::vector<ProviderSonarNode::Ptr>;
  using ConstPtrList = std::vector<ProviderSonarNode::ConstPtr>;

  typedef ScanLine ScanLineMsgType;
  typedef IntensityBin IntensityBinMsgType;
  typedef float StepType;
  typedef float AngleType;
  typedef std::vector<uint8_t> IntensityBinsRawType;

  //==========================================================================
  // P U B L I C   C / D T O R S

  ProviderSonarNode(ros::NodeHandlePtr &nh);

  ~ProviderSonarNode();

  //==========================================================================
  // P U B L I C   M E T H O D S

  void Spin();

  void ScanLineCB(const ScanLineMsgType::ConstPtr &scan_line_msg);

  bool Reconfig(provider_sonar::sonar_reconfiguration::Request &req,
                provider_sonar::sonar_reconfiguration::Response &resp);

  /* This function is a callback associated with the reception of a Scanline
     Message from the Sonar.
     It takes the Sonar scanline, formats it as a Scanline message and publishes
     it on the corresponding topic. */
  void Publish(AngleType scan_angle, StepType bin_distance_step,
               IntensityBinsRawType intensity_bins);

  void PublishPointCloud2(const ScanLineMsgType::ConstPtr &scan_line_msg);

  void Simulate();

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  ros::NodeHandlePtr nh_;
  ros::Publisher scan_line_pub_;  // Create publisher for the sonar scanlines
  // Create a service server to allow dynamic reconfiguration of the sonar
  ros::Subscriber scan_line_sub_;      // Subscriber
  ros::Publisher point_cloud2_pub_;    // Publishers
  ros::ServiceServer reconfigserver_;  // Services
  ros::ServiceServer reconfig_server_;

  SonarConfiguration config_;
  SonarDriver *driver_;

  float scan_angle_;
};

}  // namespace provider_sonar

#endif  // PROVIDER_SONAR_SONAR_NODE_H