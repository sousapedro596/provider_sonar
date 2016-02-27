/**
 * \file	sonar_node.cc
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

#include "provider_sonar/provider_sonar_node.h"

namespace provider_sonar {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
ProviderSonarNode::ProviderSonarNode(ros::NodeHandlePtr &nh)
    : nh_(nh), config_(nh_) {
  if (!config_.simulate) {
    scan_line_pub_ = nh->advertise<ScanLineMsgType>("scan_line", 100);

    driver_ = new SonarDriver(
        static_cast<uint16_t>(config_.n_bins), config_.range, config_.vos,
        static_cast<uint8_t>(config_.angle_step_size),
        static_cast<uint16_t>(config_.left_limit),
        static_cast<uint16_t>(config_.right_limit), config_.use_debug_mode);

    reconfig_server_ = nh->advertiseService("Sonar_Reconfiguration",
                                            &ProviderSonarNode::Reconfig, this);

    driver_->RegisterScanLineCallback(
        std::bind(&ProviderSonarNode::Publish, this, std::placeholders::_1,
                  std::placeholders::_2, std::placeholders::_3));

    uint8_t angle_step_size_byte = static_cast<uint8_t>(
        std::max(1, std::min(255, config_.angle_step_size)));

    if (!driver_->Connect(config_.port.c_str())) {
      ROS_ERROR("Could not connect to device; simulating instead.");
      config_.simulate = true;
    }
  }
}

//------------------------------------------------------------------------------
//
ProviderSonarNode::~ProviderSonarNode() {
  if (driver_) {
    ROS_INFO("Disconnecting Sonar!");
    driver_->Disconnect();
    delete driver_;
  }
}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
bool ProviderSonarNode::Reconfig(
    provider_sonar::sonar_reconfiguration::Request &req,
    provider_sonar::sonar_reconfiguration::Response &resp) {
  driver_->Reconfigure(req.n_bins, static_cast<float>(req.range),
                       static_cast<float>(req.vos), req.step_angle_size,
                       req.left_limit, req.right_limit, req.debug_mode);
  return true;
}

//------------------------------------------------------------------------------
//
void ProviderSonarNode::Publish(AngleType scan_angle,
                                StepType bin_distance_step,
                                IntensityBinsRawType intensity_bins) {
  ScanLineMsgType::Ptr scan_line_msg(new ScanLineMsgType);
  scan_line_msg->header.stamp = ros::Time::now();
  scan_line_msg->header.frame_id = config_.frame_id;
  scan_line_msg->angle = scan_angle;
  scan_line_msg->bin_distance_step = bin_distance_step;

  scan_line_msg->bins.reserve(intensity_bins.size());

  for (int i = 0; i < intensity_bins.size(); ++i) {
    IntensityBinMsgType bin;
    bin.distance = bin_distance_step * (i + 1);
    bin.intensity = intensity_bins[i];
    scan_line_msg->bins.push_back(bin);
  }

  scan_line_pub_.publish(scan_line_msg);
}

//------------------------------------------------------------------------------
//
void ProviderSonarNode::Simulate() {
  if (!config_.simulate) return;

  // This code simulates the sonar.
  static ros::Time last_time_;
  ros::Time now = ros::Time::now();

  IntensityBinsRawType intensity_bins(config_.simulate_n_bins);
  for (int i = 0; i < config_.simulate_n_bins; ++i) {
    intensity_bins[i] =
        config_.simulate_intensity *
        atlas::NormalizedGaussian(config_.simulate_bin_distance_step * (i + 1) -
                                      config_.simulate_distance,
                                  config_.simulate_intensity_variance);
  }

  Publish(scan_angle_, config_.simulate_bin_distance_step, intensity_bins);

  if (config_.simulate_use_manual_angle)
    scan_angle_ = config_.simulate_manual_angle;
  else
    scan_angle_ +=
        config_.simulate_scan_angle_velocity * (now - last_time_).toSec();

  scan_angle_ = scan_angle_ > 180.0
                    ? scan_angle_ - 360.0
                    : scan_angle_ < -180 ? scan_angle_ + 360 : scan_angle_;

  last_time_ = now;
}
}