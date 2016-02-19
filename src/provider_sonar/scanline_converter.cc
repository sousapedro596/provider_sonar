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

#include "scanline_converter.h"

namespace provider_sonar {

ScanLineConverter::ScanLineConverter(ros::NodeHandle &nh) {
  // Subscribers
  scan_line_sub_ = nh.subscribe("/micron_driver/scan_line", 1,
                                &ScanLineConverter::scanLineCB, this);
  // Publishers
  laser_scan_pub_ = nh.advertise<LaserScanMsgType>("laser_scan", 100);
  point_cloud_pub_ = nh.advertise<PointCloudMsgType>("point_cloud", 100);
  point_cloud2_pub_ =
      nh.advertise<sensor_msgs::PointCloud2>("point_cloud2", 100);

  // Service
  reconfigserver = nh.advertiseService("Scanline_Reconfiguration",
                                       &ScanLineConverter::reconfig, this);

  getparams(nh);
  clearLaserStats();
}

bool ScanLineConverter::reconfig(provider_sonar::scanline_parser_reconfig::Request &req,
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

bool ScanLineConverter::getparams(ros::NodeHandle &nh) {
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

void ScanLineConverter::clearLaserStats() {
  // laser_scan_msg_.ranges.clear();
  // laser_scan_msg_.intensities.clear();
  min_laser_distance_ = std::numeric_limits<float>::max();
  max_laser_distance_ = std::numeric_limits<float>::min();
  angular_distance_ = 0;
  last_scan_angle_ = 0;
  last_angular_direction_ = 0;
}

// Callback when a scanline is received
void ScanLineConverter::scanLineCB(const ScanLineMsgType::ConstPtr &scan_line_msg) {
  // publishLaserScan(scan_line_msg);
  // ROS_INFO("Publishing");
  // publishLaserScanTest(scan_line_msg);
  // publishPointCloud(scan_line_msg);
  publishPointCloud2(scan_line_msg);
}
void ScanLineConverter::publishPointCloud2(const ScanLineMsgType::ConstPtr &scan_line_msg) {
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
void ScanLineConverter::publishLaserScanTest(const ScanLineMsgType::ConstPtr &scan_line_msg) {
  // - TODO: Change msg format. Hack.
  // static float angle_min = math_utils::degToRad(135);
  // static float angle_max = math_utils::degToRad(225);
  // static float angle_increment = math_utils::degToRad(0.9);
  LaserScanMsgType laser_scan_msg_;
  // ROS_INFO("Publishing laserscan");

  laser_scan_msg_.range_min = 0;
  laser_scan_msg_.range_max = 9;
  laser_scan_msg_.angle_min =
      (135 / 360 * 2 * 3.1416);  // 2.35619;//angle_min;
  laser_scan_msg_.angle_max =
      (135 / 360 * 2 * 3.1416);  //(225/360*2*3.1416);//3
  // .92699;//angle_max;
  laser_scan_msg_.angle_increment = 0;  //(0.9/360*2*3.1416);//0.015708;
  // angle_increment;
  // Range * 2 / Sound_velocity_water
  laser_scan_msg_.scan_time = 9.0 * 2.0 / 1500.0;
  // -
  laser_scan_msg_.header = scan_line_msg->header;
  laser_scan_msg_.intensities.resize(scan_line_msg->bins.size());
  laser_scan_msg_.ranges.resize(1);  //(scan_line_msg->bins.size());
  laser_scan_msg_.ranges[9];

  for (int i = 0; i < scan_line_msg->bins.size(); i++) {
    laser_scan_msg_.intensities[i] = scan_line_msg->bins[i].intensity;
    // laser_scan_msg_.ranges[i] = (float(i)/9);//scan_line_msg->bins[i]
    // .distance;
  }
  laser_scan_pub_.publish(
      laser_scan_msg_);  // _LaserScanMsgType::Ptr(new
  // _LaserScanMsgType(laser_scan_msg_))
  clearLaserStats();
}
void ScanLineConverter::publishLaserScan(const ScanLineMsgType::ConstPtr &scan_line_msg) {
  IntensityBinMsgType bin = getThresholdedScanLine(scan_line_msg);
  LaserScanMsgType laser_scan_msg_;
  if (laser_scan_msg_.ranges.size() == 0) {
    laser_scan_msg_.angle_min = atlas::DegToRad(scan_line_msg->angle);
    laser_scan_msg_.header = scan_line_msg->header;
    last_scan_angle_ = scan_line_msg->angle;
    last_angular_direction_ = 0;
  }

  // 170 -> -170 = 20
  // -170 -> 170 = -20

  float angular_distance_inc;

  if (last_scan_angle_ > 90 && scan_line_msg->angle < -90)
    angular_distance_inc =
        (180 - last_scan_angle_) + (scan_line_msg->angle + 180);
  else if (last_scan_angle_ < -90 && scan_line_msg->angle > 90)
    angular_distance_inc =
        (-180 - last_scan_angle_) + (scan_line_msg->angle - 180);
  else
    angular_distance_inc = scan_line_msg->angle - last_scan_angle_;

  int angular_direction =
      angular_distance_inc > 0 ? 1 : angular_distance_inc < 0 ? -1 : 0;

  angular_distance_ += angular_distance_inc;

  ROS_INFO("%f %f %f %f %d\n", laser_scan_msg_.angle_min,
           scan_line_msg->angle, last_scan_angle_, angular_distance_,
           angular_direction);

  last_scan_angle_ = scan_line_msg->angle;

  laser_scan_msg_.intensities.push_back(bin.intensity / 255.0);
  laser_scan_msg_.ranges.push_back(bin.distance);

  if (scan_line_msg->bins.front().distance < min_laser_distance_)
    min_laser_distance_ = scan_line_msg->bins.front().distance;
  if (scan_line_msg->bins.back().distance > max_laser_distance_)
    max_laser_distance_ = scan_line_msg->bins.back().distance;

  if (fabs(angular_distance_) >= 300 ||
      (angular_direction != last_angular_direction_ &&
          angular_direction != 0 && last_angular_direction_ != 0)) {
    // gather statistics
    laser_scan_msg_.angle_max = atlas::DegToRad(scan_line_msg->angle);
    laser_scan_msg_.angle_increment =
        laser_scan_msg_.ranges.size() > 1
        ? atlas::DegToRad(angular_distance_ /
            (laser_scan_msg_.ranges.size() - 1))
        : 0;
    laser_scan_msg_.range_min = min_laser_distance_;
    laser_scan_msg_.range_max = max_laser_distance_;

    laser_scan_msg_.scan_time =
        (scan_line_msg->header.stamp - laser_scan_msg_.header.stamp).toSec();
    // laser_scan_msg_.header.stamp = ros::Time::now() - (
    // scan_line_msg->header.stamp - laser_scan_msg_.header.stamp );
    laser_scan_msg_.time_increment =
        laser_scan_msg_.ranges.size() > 1
        ? laser_scan_msg_.scan_time / (laser_scan_msg_.ranges.size() - 1)
        : 0;

    // publish
    if (laser_scan_msg_.ranges.size() > 0)
      laser_scan_pub_.publish(
          LaserScanMsgType::Ptr(new LaserScanMsgType(laser_scan_msg_)));
    clearLaserStats();
  }

  if (angular_direction != 0) last_angular_direction_ = angular_direction;
}

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

void ScanLineConverter::publishPointCloud(const ScanLineMsgType::ConstPtr &scan_line_msg) {
  PointCloudMsgType::Ptr point_cloud_msg(new PointCloudMsgType);
  point_cloud_msg->header = scan_line_msg->header;

  point_cloud_msg->points.reserve(scan_line_msg->bins.size());
  // point_cloud_msg->channels.size();
  sensor_msgs::ChannelFloat32 channel;
  channel.name = "intensity";
  channel.values.reserve(scan_line_msg->bins.size());

  for (int i = 0; i < scan_line_msg->bins.size(); ++i) {
    if (scan_line_msg->bins[i].distance < min_distance_threshold) continue;

    if (!use_point_cloud_threshold ||
        scan_line_msg->bins[i].intensity >=
            min_point_cloud_intensity_threshold) {
      geometry_msgs::Point32 point;
      point.x = scan_line_msg->bins[i].distance *
          cos(atlas::DegToRad(scan_line_msg->angle));
      point.y = scan_line_msg->bins[i].distance *
          sin(atlas::DegToRad(scan_line_msg->angle));
      point.z = 0;
      point_cloud_msg->points.push_back(point);

      channel.values.push_back(scan_line_msg->bins[i].intensity / 255.0);
      if (only_first_point) break;
    }
  }

  point_cloud_msg->channels.push_back(channel);
  ROS_WARN("PUBLISHING TO POiNT CLOuD");
  point_cloud_pub_.publish(point_cloud_msg);
}

}