/**
 * \file	sonar_configuration.cc
 * \author  Francis Masse <francis.masse05@gmail.com>
 * \date	26/02/2016
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

#include "provider_sonar/sonar_configuration.h"

namespace provider_sonar {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
SonarConfiguration::SonarConfiguration(const ros::NodeHandlePtr &nh)
    : frame_id("Micron"),
      port("dev/ttys3"),
      n_bins(400),
      range(8),
      vos(1500),
      angle_step_size(16),
      left_limit(2400),
      right_limit(4000),
      use_debug_mode(false),
      simulate(false),
      simulate_n_bins(400),
      simulate_bin_distance_step(1.0),
      simulate_distance(8.0),
      simulate_intensity(255),
      simulate_intensity_variance(10.0),
      simulate_use_manual_angle(false),
      simulate_manual_angle(180.0),
      simulate_scan_angle_velocity(180.0),
      use_point_cloud_threshold(true),
      min_distance_threshold(0),
      min_point_cloud_intensity_threshold(0),
      only_first_point(false),
      nh_(nh) {}

//------------------------------------------------------------------------------
//
SonarConfiguration::SonarConfiguration(const SonarConfiguration &rhs) {
  frame_id = rhs.frame_id;
  port = rhs.port;
  n_bins = rhs.n_bins;
  range = rhs.range;
  vos = rhs.vos;
  angle_step_size = rhs.angle_step_size;
  left_limit = rhs.left_limit;
  right_limit = rhs.right_limit;
  use_debug_mode = rhs.use_debug_mode;
  simulate = rhs.simulate;
  simulate_n_bins = rhs.simulate_n_bins;
  simulate_bin_distance_step = rhs.simulate_bin_distance_step;
  simulate_distance = rhs.simulate_distance;
  simulate_intensity = rhs.simulate_intensity;
  simulate_intensity_variance = rhs.simulate_intensity_variance;
  simulate_use_manual_angle = rhs.simulate_use_manual_angle;
  simulate_manual_angle = rhs.simulate_manual_angle;
  simulate_scan_angle_velocity = rhs.simulate_scan_angle_velocity;
  use_point_cloud_threshold = rhs.use_point_cloud_threshold;
  min_distance_threshold = rhs.min_distance_threshold;
  min_point_cloud_intensity_threshold = rhs.min_point_cloud_intensity_threshold;
  only_first_point = rhs.only_first_point;
  nh_ = rhs.nh_;
}

//------------------------------------------------------------------------------
//
SonarConfiguration::~SonarConfiguration() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void SonarConfiguration::DeserializeConfiguration() {
  FindParameter("/sonar_driver/frame_id_", frame_id);
  FindParameter("/sonar_driver/port_", port);
  FindParameter("/sonar_driver/n_bins_", n_bins);
  FindParameter("/sonar_driver/range_", range);
  FindParameter("/sonar_driver/vos_", vos);
  FindParameter("/sonar_driver/angle_step_size_", angle_step_size);
  FindParameter("/sonar_driver/left_limit_", left_limit);
  FindParameter("/sonar_driver/right_limit_", right_limit);
  FindParameter("/sonar_driver/use_debug_mode_", use_debug_mode);
  FindParameter("/sonar_driver/simulate_", simulate);
  FindParameter("/sonar_driver/simulate_n_bins_", simulate_n_bins);
  FindParameter("/sonar_driver/simulate_bin_distance_step_",
                simulate_bin_distance_step);
  FindParameter("/sonar_driver/simulate_distance_", simulate_distance);
  FindParameter("/sonar_driver/simulate_intensity_variance_",
                simulate_intensity_variance);
  FindParameter("/sonar_driver/simulate_use_manual_angle_",
                simulate_use_manual_angle);
  FindParameter("/sonar_driver/simulate_manual_angle_", simulate_manual_angle);
  FindParameter("/sonar_driver/simulate_scan_angle_velocity_",
                simulate_scan_angle_velocity);
  FindParameter("/scanline_parser/use_point_cloud_threshold_",
                use_point_cloud_threshold);
  FindParameter("/scanline_parser/min_distance_threshold_",
                min_distance_threshold);
  FindParameter("/scanline_parser/min_point_cloud_intensity_threshold_",
                min_point_cloud_intensity_threshold);
  FindParameter("/scanline_parser/only_first_point_", only_first_point);
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
void SonarConfiguration::FindParameter(const std::string &str, Tp_ &p) {
  if (nh_->hasParam("/provider_sonar" + str)) {
    nh_->getParam("/provider_sonar" + str, p);
  } else {
    ROS_WARN_STREAM("Did not find /provider_sonar" + str
                    << ". Using default value instead.");
  }
}
}
