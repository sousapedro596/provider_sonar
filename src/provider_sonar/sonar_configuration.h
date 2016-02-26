/**
 * \file	sonar_configuration.h
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

#ifndef PROVIDER_SONAR_SONAR_CONFIGURATION_H
#define PROVIDER_SONAR_SONAR_CONFIGURATION_H

#include <ros/ros.h>

namespace provider_sonar {
class SonarConfiguration {
 public:
  //============================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<SonarConfiguration>;
  using ConstPtr = std::shared_ptr<const SonarConfiguration>;
  using PtrList = std::vector<SonarConfiguration::Ptr>;
  using ConstPtrList = std::vector<SonarConfiguration::ConstPtr>;

  //============================================================================
  // P U B L I C   C / D T O R S

  SonarConfiguration(const ros::NodeHandlePtr &nh);
  SonarConfiguration(const SonarConfiguration &rhs);

  ~SonarConfiguration();

  //============================================================================
  // P U B L I C   M E M B E R S

  // Sonar parameters
  std::string frame_id;
  std::string port;
  int n_bins;
  float range;
  float vos;
  int angle_step_size;
  int left_limit;
  int right_limit;
  bool use_debug_mode;
  bool simulate;

  // Sonar simulation parameters
  int simulate_n_bins;
  double simulate_bin_distance_step;
  double simulate_distance;
  int simulate_intensity;
  double simulate_intensity_variance;
  bool simulate_use_manual_angle;
  double simulate_manual_angle;
  double simulate_scan_angle_velocity;

  // Scanline parser parameters
  double min_distance_threshold;
  double min_point_cloud_intensity_threshold;
  bool use_point_cloud_threshold;
  bool only_first_point;

 private:
  //============================================================================
  // P R I V A T E   M E T H O D S

  void DeserializeConfiguration();

  template <typename Tp_>
  void FindParameter(const std::string &str, Tp_ &p);

  //============================================================================
  // P R I V A T E   M E M B E R S

  ros::NodeHandlePtr nh_;
};

}  // namespace provider_sonar

#endif  // PROVIDER_SONAR_SONAR_CONFIGURATION_H
