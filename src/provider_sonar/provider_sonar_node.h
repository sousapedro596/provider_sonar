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
// Messages includes
#include "std_msgs/String.h"
#include <provider_sonar/ScanLine.h>

// Service includes
#include <provider_sonar/sonar_reconfig.h>

// C++ includes
#include <sstream>
#include "stdint.h"

// Code includes
#include <provider_sonar/Serial.h>
#include <provider_sonar/sonar_driver.h>
#include <provider_sonar/math.h>

namespace provider_sonar {

class ProviderSonarNode {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<ProviderSonarNode>;
  using ConstPtr = std::shared_ptr<const ProviderSonarNode>;
  using PtrList = std::vector<ProviderSonarNode::Ptr>;
  using ConstPtrList = std::vector<ProviderSonarNode::ConstPtr>;

  typedef provider_sonar::ScanLine _ScanLineMsgType;
  typedef provider_sonar::IntensityBin _IntensityBinMsgType;
  typedef float _StepType;
  typedef float _AngleType;
  typedef std::vector<uint8_t> _IntensityBinsRawType;

  //==========================================================================
  // P U B L I C   C / D T O R S

  ProviderSonarNode(ros::NodeHandle &nh);

  ~ProviderSonarNode();

  //==========================================================================
  // P U B L I C   M E T H O D S

  bool reconfig(provider_sonar::sonar_reconfig::Request &req,
                provider_sonar::sonar_reconfig::Response &resp);

  /* This function is a callback associated with the reception of a Scanline
     Message from the Sonar.
     It takes the Sonar scanline, formats it as a Scanline message and publishes
     it on the corresponding
     topic. */
  void publish(_AngleType scan_angle, _StepType bin_distance_step,
               _IntensityBinsRawType intensity_bins);

  void simulate();

  void spin();

  bool getparams(ros::NodeHandle &nh);

  // Create publisher for the sonar scanlines
  ros::Publisher scan_line_pub_;

  // Create a service server to allow dynamic reconfiguration of the sonar
  ros::ServiceServer reconfigserver;

  // Sonar parameters: Will be read from parameter server.
  std::string frame_id_;
  std::string port_;
  int num_bins_;
  double range_;
  double velocity_of_sound_;
  int angle_step_size_;
  int leftLimit_;
  int rightLimit_;
  bool use_debug_mode;
  bool simulate_;
  SonarDriver *driver_;

  // Sonar simulation parameters
  int simulate_num_bins_;
  double simulate_bin_distance_step_;
  double simulate_distance;
  int simulate_intensity;
  double simulate_intensity_variance;
  bool simulate_use_manual_angle;
  double simulate_manual_angle;
  double simulate_scan_angle_velocity;
  float scan_angle;

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  ros::NodeHandlePtr nh_;

};

}  // namespace provider_sonar

#endif  // PROVIDER_SONAR_SONAR_NODE_H