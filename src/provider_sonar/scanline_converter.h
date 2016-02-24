/*******************************************************************************
 *
 *      scan_line_converter
 *
 *      Copyright (c) 2011, Edward T. Kaszubski ( ekaszubski@gmail.com )
 *      All rights reserved.
 *
 *      Redistribution and use in source and binary forms, with or without
 *      modification, are permitted provided that the following conditions are
 *      met:
 *
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following disclaimer
 *        in the documentation and/or other materials provided with the
 *        distribution.
 *      * Neither the name of "tritech_micron-RelWithDebInfo@tritech_micron" nor
 *the names of its
 *        contributors may be used to endorse or promote products derived from
 *        this software without specific prior written permission.
 *
 *      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *      A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

#ifndef SCAN_LINE_CONVERTER_H_
#define SCAN_LINE_CONVERTER_H_

#include <lib_atlas/maths/trigo.h>
//#include <provider_sonar/math.h>
#include <provider_sonar/ScanLine.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "ros/ros.h"

// Service includes
#include <provider_sonar/scanline_parser_reconfig.h>

namespace provider_sonar {

class ScanLineConverter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M
  typedef ScanLine ScanLineMsgType;
  typedef IntensityBin IntensityBinMsgType;

  typedef sensor_msgs::PointCloud PointCloudMsgType;

  typedef float StepType;
  typedef float AngleType;
  typedef std::vector<unsigned char> IntensityBinsRawType;

  //==========================================================================
  // P U B L I C   C / D T O R S
  ScanLineConverter(ros::NodeHandle &nh);


  //==========================================================================
  // P U B L I C   M E T H O D S
  bool Reconfig(provider_sonar::scanline_parser_reconfig::Request &req,
                provider_sonar::scanline_parser_reconfig::Response &resp);

  bool Getparams(ros::NodeHandle &nh);

  void ClearLaserStats();

  void ScanLineCB(const ScanLineMsgType::ConstPtr &scan_line_msg);

  void PublishPointCloud2(const ScanLineMsgType::ConstPtr &scan_line_msg);

  IntensityBinMsgType getThresholdedScanLine(
      const ScanLineMsgType::ConstPtr &scan_line_msg);

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S
  ros::Subscriber scan_line_sub_;  // Subscriber
  ros::Publisher point_cloud2_pub_;  // Publishers
  ros::ServiceServer reconfigserver;  // Services

  double min_laser_intensity_threshold;
  double min_distance_threshold;
  double min_point_cloud_intensity_threshold;
  bool use_point_cloud_threshold;
  bool only_first_point;
  float min_laser_distance_;
  float max_laser_distance_;
  float angular_distance_;
  float last_scan_angle_;
  int last_angular_direction_;
};
}  // namespace provider_sonar

#endif /* SCAN_LINE_CONVERTER_H_ */
