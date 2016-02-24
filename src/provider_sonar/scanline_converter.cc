/**
 * \file	scanline_converter.cc
 * \author  Francis Masse <francis.masse05@gmail.com>
 * \date	19/02/2016
 *
 * \copyright Copyright (c) 2016 Copyright
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

#include "provider_sonar/scanline_converter.h"

namespace provider_sonar {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
ScanLineConverter::ScanLineConverter(ros::NodeHandle &nh) {
  scan_line_sub_ = nh.subscribe("/micron_driver/scan_line", 1,
                                &ScanLineConverter::ScanLineCB, this);
  point_cloud2_pub_ =
      nh.advertise<sensor_msgs::PointCloud2>("point_cloud2", 100);

  reconfigserver = nh.advertiseService("Scanline_Reconfiguration",
                                       &ScanLineConverter::Reconfig, this);

  Getparams(nh);
  ClearLaserStats();
}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
bool ScanLineConverter::Reconfig(provider_sonar::scanline_parser_reconfig::Request &req,
                                 provider_sonar::scanline_parser_reconfig::Response &resp) {
  min_laser_intensity_threshold = req.min_laser_intensity_threshold;
  min_distance_threshold = req.min_distance_threshold;
  min_point_cloud_intensity_threshold =
      req.min_point_cloud_intensity_threshold;
  use_point_cloud_threshold = req.use_point_cloud_threshold;
  only_first_point = req.only_first_point;

  resp.result = true;

  return true;
}

//------------------------------------------------------------------------------
//
bool ScanLineConverter::Getparams(ros::NodeHandle &nh) {
  if (nh.hasParam("/scan_line_parser/use_point_cloud_threshold"))
    nh.getParam("/scan_line_parser/use_point_cloud_threshold",
                use_point_cloud_threshold);
  else {
    use_point_cloud_threshold = true;
    ROS_WARN(
        "Did not find 'use_point_cloud_threshold' on the parameter Server, "
            "using default value instead");
  }
  if (nh.hasParam("/scan_line_parser/min_distance_threshold"))
    nh.getParam("/scan_line_parser/min_distance_threshold",
                min_distance_threshold);
  else {
    min_distance_threshold = 0.0;
    ROS_WARN(
        "Did not find 'min_distance_threshold' on the parameter Server, "
            "using default value instead");
  }
  if (nh.hasParam("/scan_line_parser/min_point_cloud_intensity_threshold"))
    nh.getParam("/scan_line_parser/min_point_cloud_intensity_threshold",
                min_point_cloud_intensity_threshold);
  else {
    min_point_cloud_intensity_threshold = 50;
    ROS_WARN(
        "Did not find 'min_point_cloud_intensity_threshold' on the parameter "
            "Server, using default value instead");
  }
  if (nh.hasParam("/scan_line_parser/min_laser_intensity_threshold"))
    nh.getParam("/scan_line_parser/min_laser_intensity_threshold",
                min_laser_intensity_threshold);
  else {
    min_laser_intensity_threshold = 50;
    ROS_WARN(
        "Did not find 'min_laser_intensity_threshold' on the parameter "
            "Server, using default value instead");
  }
  if (nh.hasParam("/scan_line_parser/only_first_point"))
    nh.getParam("/scan_line_parser/only_first_point", only_first_point);
  else {
    only_first_point = true;
    ROS_WARN(
        "Did not find 'only_first_point' on the parameter Server, using "
            "default value instead");
  }
}

//------------------------------------------------------------------------------
//
void ScanLineConverter::ClearLaserStats() {
  min_laser_distance_ = std::numeric_limits<float>::max();
  max_laser_distance_ = std::numeric_limits<float>::min();
  angular_distance_ = 0;
  last_scan_angle_ = 0;
  last_angular_direction_ = 0;
}

//------------------------------------------------------------------------------
//
// Callback when a scanline is received
void ScanLineConverter::ScanLineCB(const ScanLineMsgType::ConstPtr &scan_line_msg) {
  ROS_INFO("Publishing");
  PublishPointCloud2(scan_line_msg);
}

//------------------------------------------------------------------------------
//
void ScanLineConverter::PublishPointCloud2(const ScanLineMsgType::ConstPtr &scan_line_msg) {
  sensor_msgs::PointCloud2 point_cloud_msg_;
  // - Copy ROS header
  point_cloud_msg_.header = scan_line_msg->header;
  // - TODO: Wtf is height and width
  point_cloud_msg_.width = scan_line_msg->bins.size();
  point_cloud_msg_.height = 1;
  // - Fields: x, y, z, intensity
  // - Fields describe the binary blob in data
  point_cloud_msg_.fields.resize(4);
  point_cloud_msg_.fields[0].name = "x";
  point_cloud_msg_.fields[1].name = "y";
  point_cloud_msg_.fields[2].name = "z";
  point_cloud_msg_.fields[3].name = "intensity";
  // - Offset from the beginning of the point struct in bytes
  int offset = 0;
  for (size_t i = 0; i < point_cloud_msg_.fields.size(); ++i, offset += 4) {
    point_cloud_msg_.fields[i].offset = offset;
    point_cloud_msg_.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    point_cloud_msg_.fields[i].count = 1;
  }
  // - Offset per point of data (x, y, z, intensity)
  point_cloud_msg_.point_step = offset;
  // - length of the row TODO: is it ok?
  point_cloud_msg_.row_step = point_cloud_msg_.width;
  point_cloud_msg_.data.resize(point_cloud_msg_.point_step *
      point_cloud_msg_.row_step);
  point_cloud_msg_.is_bigendian = false;
  point_cloud_msg_.is_dense = false;

  // - Centered at 0 degree. 180 degree is the middle of the sonar scanline
  float delta_x = scan_line_msg->bin_distance_step *
      cos(atlas::DegToRad(scan_line_msg->angle - 180.0));
  float delta_y = scan_line_msg->bin_distance_step *
      sin(atlas::DegToRad(scan_line_msg->angle - 180.0));

  // - try with distance * cos (theta)
  float coordinate_x = 0;
  float coordinate_y = 0;
  float coordinate_z = 0;
  for (size_t i = 0; i < scan_line_msg->bins.size();
       ++i, coordinate_x += delta_x, coordinate_y += delta_y) {
    float bin_intensity = (float) (scan_line_msg->bins[i].intensity) / 255.0;
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
    // point_cloud_msg_.data[i * point_cloud_msg_.point_step +
    // point_cloud_msg_.fields[0].offset] = coordinate_x;
    // point_cloud_msg_.data[i * point_cloud_msg_.point_step +
    // point_cloud_msg_.fields[1].offset] = coordinate_y;
    // point_cloud_msg_.data[i * point_cloud_msg_.point_step +
    // point_cloud_msg_.fields[2].offset] = 0;
    // point_cloud_msg_.data[i * point_cloud_msg_.point_step +
    // point_cloud_msg_.fields[3].offset] =
  }
  ROS_INFO("Publishing PointCloud2");
  point_cloud2_pub_.publish(point_cloud_msg_);
}

//------------------------------------------------------------------------------
//
// This function loops through the received scanline until it finds a
// intensity greater then the defined threshold.
// It then returns the intensity and the corresponding distance
ScanLineConverter::IntensityBinMsgType ScanLineConverter::getThresholdedScanLine(
    const ScanLineMsgType::ConstPtr &scan_line_msg) {
  for (int i = 0; i < scan_line_msg->bins.size(); ++i) {
    if (scan_line_msg->bins[i].distance < min_distance_threshold) continue;

    /*if ( !reconfigure_params_.use_laser_threshold ||
    scan_line_msg->bins[i].intensity >=
    reconfigure_params_.min_laser_intensity_threshold )
    {
            laser_scan_msg->intensities.push_back(
    scan_line_msg->bins[i].intensity / 255.0 );
            laser_scan_msg->ranges.push_back( scan_line_msg->bins[i].distance
    );
    }*/

    if (scan_line_msg->bins[i].intensity >= min_laser_intensity_threshold)
      return scan_line_msg->bins[i];
  }

  return IntensityBinMsgType();
}
}