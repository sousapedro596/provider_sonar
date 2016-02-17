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

#include "provider_sonar_node.h"

namespace provider_sonar {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
// Constructor
ProviderSonarNode::ProviderSonarNode(ros::NodeHandle &nh){
  getparams(nh);

  if (!simulate_) {
    scan_line_pub_ = nh.advertise<_ScanLineMsgType>("scan_line", 100);

    driver_ = new TritechMicronDriver(num_bins_, range_, velocity_of_sound_,
                                      angle_step_size_, leftLimit_,
                                      rightLimit_, use_debug_mode);
    reconfigserver = nh.advertiseService("Sonar_Reconfiguration",
                                         &ProviderSonarNode::reconfig, this);

    driver_->registerScanLineCallback(
        std::bind(&ProviderSonarNode::publish, this, std::placeholders::_1,
                  std::placeholders::_2, std::placeholders::_3));

    uint8_t angle_step_size_byte =
        std::max(1, std::min(255, angle_step_size_));

    if (!driver_->connect(port_.c_str())) {
      ROS_ERROR("Could not connect to device; simulating instead.");
      simulate_ = true;
    }
  }
}

// Destructor
ProviderSonarNode::~ProviderSonarNode() {
  if (driver_) {
    ROS_INFO("Disconnecting Sonar!");
    driver_->disconnect();
    delete driver_;
  }
}

bool ProviderSonarNode::reconfig(provider_sonar::sonar_reconfig::Request &req,
                                 provider_sonar::sonar_reconfig::Response &resp) {
  driver_->reconfigure(req.nbins, req.range, req.vos, req.angle_step_size,
                       req.leftLimit, req.rightLimit);

  return true;
}

/* This function is a callback associated with the reception of a Scanline
   Message from the Sonar.
   It takes the Sonar scanline, formats it as a Scanline message and publishes
   it on the corresponding
   topic. */
void ProviderSonarNode::publish(_AngleType scan_angle, _StepType bin_distance_step,
                                _IntensityBinsRawType intensity_bins) {
  _ScanLineMsgType::Ptr scan_line_msg(new _ScanLineMsgType);
  scan_line_msg->header.stamp = ros::Time::now();
  scan_line_msg->header.frame_id = frame_id_;
  scan_line_msg->angle = scan_angle;
  scan_line_msg->bin_distance_step = bin_distance_step;

  scan_line_msg->bins.reserve(intensity_bins.size());

  for (int i = 0; i < intensity_bins.size(); ++i) {
    _IntensityBinMsgType bin;
    bin.distance = bin_distance_step * (i + 1);
    bin.intensity = intensity_bins[i];
    scan_line_msg->bins.push_back(bin);
  }

  scan_line_pub_.publish(scan_line_msg);
}

// This function queries the parameter server for the needed parameters. If
// not found, it uses the default values:
bool ProviderSonarNode::getparams(ros::NodeHandle &nh) {
  if (nh.hasParam("/micron_driver/frame_id_"))
    nh.getParam("/micron_driver/frame_id_", frame_id_);
  else {
    frame_id_ = "Micron";
    ROS_WARN(
        "Did not find frame_id on the parameter Server, using default value "
            "instead");
  }

  if (nh.hasParam("/micron_driver/port_"))
    nh.getParam("/micron_driver/port_", port_);
  else {
    port_ = "/dev/ttyS3";
    //port_ = "/dev/ttyUSB0";
    ROS_WARN(
        "Did not find port_ on the parameter Server, using default value "
            "instead");
  }

  if (nh.hasParam("/micron_driver/num_bins_"))
    nh.getParam("/micron_driver/num_bins_", num_bins_);
  else {
    num_bins_ = 200;
    ROS_WARN(
        "Did not find num_bins_ on the parameter Server, using default value "
            "instead");
  }

  if (nh.hasParam("/micron_driver/range_"))
    nh.getParam("/micron_driver/range_", range_);
  else {
    range_ = 9;
    ROS_WARN(
        "Did not find range_ on the parameter Server, using default value "
            "instead");
  }

  if (nh.hasParam("/micron_driver/velocity_of_sound_"))
    nh.getParam("/micron_driver/velocity_of_sound_", velocity_of_sound_);
  else {
    velocity_of_sound_ = 1500;
    ROS_WARN(
        "Did not find velocity_of_sound_ on the parameter Server, using "
            "default value instead");
  }

  if (nh.hasParam("/micron_driver/angle_step_size_"))
    nh.getParam("/micron_driver/angle_step_size_", angle_step_size_);
  else {
    angle_step_size_ = 16;
    ROS_WARN(
        "Did not find angle_step_size_ on the parameter Server, using "
            "default value instead");
  }

  if (nh.hasParam("/micron_driver/leftLimit_"))
    nh.getParam("/micron_driver/leftLimit_", leftLimit_);
  else {
    leftLimit_ = 2400;
    ROS_WARN(
        "Did not find leftLimit_ on the parameter Server, using default "
            "value instead");
  }

  if (nh.hasParam("/micron_driver/rightLimit_"))
    nh.getParam("/micron_driver/rightLimit_", rightLimit_);
  else {
    rightLimit_ = 4000;
    ROS_WARN(
        "Did not find rightLimit_ on the parameter Server, using default "
            "value instead");
  }

  if (nh.hasParam("/micron_driver/use_debug_mode"))
    nh.getParam("/micron_driver/use_debug_mode", use_debug_mode);
  else {
    use_debug_mode = true;
    ROS_WARN(
        "Did not find use_debug_mode on the parameter Server, using default "
            "value instead");
  }

  if (nh.hasParam("/micron_driver/simulate_"))
    nh.getParam("/micron_driver/simulate_", simulate_);
  else {
    simulate_ = false;
    ROS_WARN(
        "Did not find simulate_ on the parameter Server, using default value "
            "instead");
  }

  // Sonar simulation parameters
  if (nh.hasParam("/micron_driver/simulate_num_bins_"))
    nh.getParam("/micron_driver/simulate_num_bins_", simulate_num_bins_);
  else {
    simulate_num_bins_ = false;
    ROS_WARN(
        "Did not find simulate_num_bins_ on the parameter Server, using "
            "default value instead");
  }

  if (nh.hasParam("/micron_driver/simulate_bin_distance_step_"))
    nh.getParam("/micron_driver/simulate_bin_distance_step_",
                simulate_bin_distance_step_);
  else {
    simulate_bin_distance_step_ = false;
    ROS_WARN(
        "Did not find simulate_bin_distance_step_ on the parameter Server, "
            "using default value instead");
  }

  if (nh.hasParam("/micron_driver/simulate_distance"))
    nh.getParam("/micron_driver/simulate_distance", simulate_distance);
  else {
    simulate_distance = false;
    ROS_WARN(
        "Did not find simulate_distance on the parameter Server, using "
            "default value instead");
  }

  if (nh.hasParam("/micron_driver/simulate_intensity"))
    nh.getParam("/micron_driver/simulate_intensity", simulate_intensity);
  else {
    simulate_intensity = false;
    ROS_WARN(
        "Did not find simulate_intensity on the parameter Server, using "
            "default value instead");
  }

  if (nh.hasParam("/micron_driver/simulate_intensity_variance"))
    nh.getParam("/micron_driver/simulate_intensity_variance",
                simulate_intensity_variance);
  else {
    simulate_intensity_variance = false;
    ROS_WARN(
        "Did not find simulate_intensity_variance on the parameter Server, "
            "using default value instead");
  }

  if (nh.hasParam("/micron_driver/simulate_use_manual_angle"))
    nh.getParam("/micron_driver/simulate_use_manual_angle",
                simulate_use_manual_angle);
  else {
    simulate_use_manual_angle = false;
    ROS_WARN(
        "Did not find simulate_use_manual_angle on the parameter Server, "
            "using default value instead");
  }

  if (nh.hasParam("/micron_driver/simulate_manual_angle"))
    nh.getParam("/micron_driver/simulate_manual_angle",
                simulate_manual_angle);
  else {
    simulate_manual_angle = false;
    ROS_WARN(
        "Did not find simulate_manual_angle on the parameter Server, using "
            "default value instead");
  }

  if (nh.hasParam("/micron_driver/simulate_scan_angle_velocity"))
    nh.getParam("/micron_driver/simulate_scan_angle_velocity",
                simulate_scan_angle_velocity);
  else {
    simulate_scan_angle_velocity = false;
    ROS_WARN(
        "Did not find simulate_scan_angle_velocity on the parameter Server, "
            "using default value instead");
  }
}

void ProviderSonarNode::simulate() {
  if (!simulate_) return;

  // This code simulates the sonar.
  static ros::Time last_time_;
  ros::Time now = ros::Time::now();

  _IntensityBinsRawType intensity_bins(simulate_num_bins_);
  for (int i = 0; i < simulate_num_bins_; ++i) {
    intensity_bins[i] =
        simulate_intensity *
            math_utils::normalizedGaussian(
                simulate_bin_distance_step_ * (i + 1) - simulate_distance,
                simulate_intensity_variance);
  }

  publish(scan_angle, simulate_bin_distance_step_, intensity_bins);

  if (simulate_use_manual_angle)
    scan_angle = simulate_manual_angle;
  else
    scan_angle += simulate_scan_angle_velocity * (now - last_time_).toSec();

  scan_angle = scan_angle > 180.0
               ? scan_angle - 360.0
               : scan_angle < -180 ? scan_angle + 360 : scan_angle;

  last_time_ = now;
}
}