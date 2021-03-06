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
    point_cloud2_pub_ =
        nh_->advertise<sensor_msgs::PointCloud2>("point_cloud2", 100);

    sonar_configuration_pub_ =
        nh_->advertise<sonia_msgs::ProviderSonarConfiguration>(
            "provider_sonar_configuration", 100);

    driver_ = new SonarDriver(static_cast<uint16_t>(config_.n_bins),
                              config_.range, config_.vos,
                              static_cast<uint8_t>(config_.angle_step_size),
                              static_cast<uint16_t>(config_.left_limit),
                              static_cast<uint16_t>(config_.right_limit),
                              config_.gain, config_.use_debug_mode);

    sonar_reconfig_server_ =
        nh->advertiseService("sonar_reconfiguration",
                             &ProviderSonarNode::SonarReconfiguration, this);

    driver_->ScanLineCallback(std::bind(
        &ProviderSonarNode::PublishPointCloud2, this, std::placeholders::_1,
        std::placeholders::_2, std::placeholders::_3));

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
void ProviderSonarNode::Spin() {
  while (!ros::isShuttingDown()) {
    ros::Time previous = ros::Time::now();
    while (nh_->ok()) {
      ros::spinOnce();
      ros::Time now = ros::Time::now();
      ros::Duration desired(1, 0);

      if ((now - previous) > desired) {
        PublishProviderSonarConfiguration(
            static_cast<uint16_t>(config_.n_bins), config_.range, config_.vos,
            static_cast<uint8_t>(config_.angle_step_size),
            static_cast<uint16_t>(config_.left_limit),
            static_cast<uint16_t>(config_.right_limit), config_.gain);
        previous = ros::Time::now();
      }
    }
  }
}

//------------------------------------------------------------------------------
//
bool ProviderSonarNode::SonarReconfiguration(
    sonia_msgs::SonarReconfiguration::Request &req,
    sonia_msgs::SonarReconfiguration::Response &resp) {
  config_.n_bins = req.n_bins;
  config_.range = static_cast<float>(req.range);
  config_.vos = static_cast<float>(req.vos);
  config_.angle_step_size = req.step_angle_size;
  config_.left_limit = req.left_limit;
  config_.right_limit = req.right_limit;
  config_.gain = static_cast<float>(req.gain);

  driver_->Reconfigure(
      static_cast<uint16_t>(config_.n_bins), config_.range, config_.vos,
      static_cast<uint8_t>(config_.angle_step_size),
      static_cast<uint16_t>(config_.left_limit),
      static_cast<uint16_t>(config_.right_limit), config_.gain, req.debug_mode);
  return true;
}

//------------------------------------------------------------------------------
//
void ProviderSonarNode::PublishProviderSonarConfiguration(
    uint16_t n_bin, float range, float vos, uint8_t angle_step_size,
    uint16_t left_limit, uint16_t right_limit, float gain) {
  sonia_msgs::ProviderSonarConfiguration provider_sonar_config_msg;

  provider_sonar_config_msg.n_bin = n_bin;
  provider_sonar_config_msg.range = range;
  provider_sonar_config_msg.vos = vos;
  provider_sonar_config_msg.angle_step_size = angle_step_size;
  provider_sonar_config_msg.left_limit = left_limit;
  provider_sonar_config_msg.right_limit = right_limit;
  provider_sonar_config_msg.gain = gain;

  sonar_configuration_pub_.publish(provider_sonar_config_msg);
}

//------------------------------------------------------------------------------
//
void ProviderSonarNode::PublishPointCloud2(
    AngleType scan_angle, StepType bin_distance_step,
    IntensityBinsRawType intensity_bins) {
  std::vector<IntensityBinMsgType> bins;
  bins.reserve(intensity_bins.size());

  for (int i = 0; i < static_cast<int>(intensity_bins.size()); ++i) {
    IntensityBinMsgType bin;
    bin.distance = bin_distance_step * (i + 1);
    bin.intensity = intensity_bins[i];
    bins.push_back(bin);
  }

  sensor_msgs::PointCloud2 point_cloud_msg_;
  // Fill up point cloud header
  point_cloud_msg_.header.stamp = ros::Time::now();
  point_cloud_msg_.header.frame_id = config_.frame_id;
  // Unorganized point cloud
  point_cloud_msg_.width = static_cast<uint32_t>(bins.size());
  point_cloud_msg_.height = 1;
  // - Fields: x, y, z, intensity
  // - Fields describe the binary blob in data
  point_cloud_msg_.fields.resize(4);
  point_cloud_msg_.fields[0].name = "x";
  point_cloud_msg_.fields[1].name = "y";
  point_cloud_msg_.fields[2].name = "z";
  point_cloud_msg_.fields[3].name = "intensity";
  // - Offset from the beginning of the point struct in bytes
  uint32_t offset = 0;
  for (size_t i = 0; i < point_cloud_msg_.fields.size(); ++i, offset += 4) {
    point_cloud_msg_.fields[i].offset = offset;
    point_cloud_msg_.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    point_cloud_msg_.fields[i].count = 1;
  }
  // - Offset per point of data (x, y, z, intensity)
  point_cloud_msg_.point_step = offset;
  point_cloud_msg_.row_step = point_cloud_msg_.width;
  point_cloud_msg_.data.resize(point_cloud_msg_.point_step *
                               point_cloud_msg_.row_step);
  point_cloud_msg_.is_bigendian = 0;
  point_cloud_msg_.is_dense = 0;

  // - Centered at 0 degree. 180 degree is the middle of the sonar scanline
  float delta_x = static_cast<float>(bin_distance_step *
                                     cos(atlas::DegToRad(scan_angle - 180.0)));

  float delta_y = static_cast<float>(bin_distance_step *
                                     sin(atlas::DegToRad(scan_angle - 180.0)));

  // - try with distance * cos(theta)
  float coordinate_x = 0;
  float coordinate_y = 0;
  float coordinate_z = 0;
  for (size_t i = 0; i < bins.size();
       ++i, coordinate_x += delta_x, coordinate_y += delta_y) {
    float bin_intensity = static_cast<float>((bins[i].intensity) / 255.0);
    memcpy(&point_cloud_msg_.data[i * point_cloud_msg_.point_step +
                                  point_cloud_msg_.fields[0].offset],
           &coordinate_x, sizeof(float));
    memcpy(&point_cloud_msg_.data[i * point_cloud_msg_.point_step +
                                  point_cloud_msg_.fields[1].offset],
           &coordinate_y, sizeof(float));
    memcpy(&point_cloud_msg_.data[i * point_cloud_msg_.point_step +
                                  point_cloud_msg_.fields[2].offset],
           &coordinate_z, sizeof(float));
    memcpy(&point_cloud_msg_.data[i * point_cloud_msg_.point_step +
                                  point_cloud_msg_.fields[3].offset],
           &bin_intensity, sizeof(float));
  }
  point_cloud_msg_.header.frame_id = "BODY";
  point_cloud2_pub_.publish(point_cloud_msg_);
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

  PublishPointCloud2(scan_angle_, config_.simulate_bin_distance_step,
                     intensity_bins);

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
