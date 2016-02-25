// ######################################################################
//
//      TritechMicron - A protocol parser for Tritech Micron sonars.
//      Copyright (C) 2011  Randolph Voorhies
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// ######################################################################

#ifndef TRITECHMICRON_MESSAGETYPES_H
#define TRITECHMICRON_MESSAGETYPES_H

#include <provider_sonar/sonar_driver.h>
#include "messages_id.h"
#include <iostream>
#include "ros/ros.h"
#include <vector>
#include <bitset>
#include <cmath>

namespace provider_sonar {

struct SonarMessage {
  uint8_t header_;
  uint32_t hex_length_;
  uint16_t binary_length_;
  uint8_t tx_node_;
  uint8_t rx_node_;
  uint8_t n_byte_;
  provider_sonar::MessageID id_;
  uint8_t message_sequence_;
  uint8_t node_;
  std::vector<uint8_t> data_;
  uint8_t line_feed_;

  SonarMessage() {
    header_ = 0;
    hex_length_ = 0;
    binary_length_ = 0;
    tx_node_ = 0;
    rx_node_ = 0;
    n_byte_ = 0;
    id_ = mtNull;
    message_sequence_ = 0;
    node_ = 0;
    data_.clear();
    line_feed_ = 0;
  }

  bool MessageLenghtCheck(uint8_t length, MessageID id) const {
    if (data_.size() != length) {
      std::cerr << "Invalid message length (" << data_.size()
                << " != " << length << ")" << std::endl;
      return false;
    }
    if (id_ != id) {
      std::cerr << "Invalid message type (" << id_ << " != " << id << ")"
                << std::endl;
      return false;
    }
    return true;
  }

  bool IsByteEqual(uint8_t byte, uint8_t at) const {
    if (data_.size() < at) {
      std::cerr << "Message too short" << std::endl;
      return false;
    }
    if (data_[at] != byte) {
      std::cerr << "Expected " << std::hex << (int)byte << std::dec << " @"
                << at << " but got " << std::hex << (int)data_[at] << std::endl;
      return false;
    }
    return true;
  }
};

//------------------------------------------------------------------------------
//
static std::vector<uint8_t> mtSendBBUserMsg = {
    0x40,  // header
    0x30,  // hex_length
    0x30,  // hex_length
    0x30,  // hex_length
    0x38,  // hex_length
    0x08,  // binairy_length
    0x00,  // binairy_length
    0xFF,  // tx_node
    0x02,  // rx_node
    0x03,  // n_byte
    0x17,  // mtSendVersion
    0x80,  // message_sequence
    0x02,  // node
    0x0A   // line_feed
};

//------------------------------------------------------------------------------
//
static std::vector<uint8_t> mtSendVersionMsg = {
    0x40,  // header
    0x30,  // hex_length
    0x30,  // hex_length
    0x30,  // hex_length
    0x38,  // hex_length
    0x08,  // binairy_length
    0x00,  // binairy_length
    0xFF,  // tx_node
    0x02,  // rx_node
    0x03,  // n_byte
    0x17,  // mtSendVersion
    0x80,  // message_sequence
    0x02,  // node
    0x0A   // line_feed
};

//------------------------------------------------------------------------------
//
struct mtVersionDataMsg {
  const MessageID id_ = mtVersionData;
  uint8_t tx_node_;
  uint8_t software_version_;
  uint8_t info_bits_;
  uint16_t uid_;
  uint32_t program_length_;
  uint16_t checksum_;

  mtVersionDataMsg(SonarMessage const &msg) {
    msg.MessageLenghtCheck(26, mtVersionData);

    msg.IsByteEqual('@', 1);
    msg.IsByteEqual(mtVersionData, 11);

    tx_node_ = msg.data_[13];

    software_version_ = msg.data_[14];

    info_bits_ = msg.data_[15];

    uid_ = uint16_t(msg.data_[16] << 0);
    uid_ |= uint16_t(msg.data_[17] << 8);

    program_length_ = uint32_t(msg.data_[18] << 0);
    program_length_ |= uint32_t(msg.data_[19] << 8);
    program_length_ |= uint32_t(msg.data_[20] << 16);
    program_length_ |= uint32_t(msg.data_[21] << 24);

    checksum_ = uint32_t(msg.data_[22] << 0);
    checksum_ += uint32_t(msg.data_[23] << 8);

    msg.IsByteEqual(0x0A, 25);
  }

  void print() {
    printf(
        "mtVersionDataMsg: tx_node_ = %#x software_version_ = %#x info_bits_ = "
        "%#x uid_ = %#x "
        "program_length_ = %d checksum_ = %d\n",
        tx_node_, software_version_, info_bits_, uid_, program_length_,
        checksum_);
  }
};

// ######################################################################
struct mtAliveMsg {
  const MessageID id = mtAlive;
  uint8_t tx_node_;
  uint32_t head_time_msec_;
  int motor_pos_;

  // Head Info BitSet
  bool in_centre_;
  bool centered_;
  bool motoring_;
  bool motor_on_;
  bool dir_;
  bool in_scan_;
  bool no_params_;
  bool sent_cfg_;

  mtAliveMsg(SonarMessage const &msg) {
    msg.IsByteEqual('@', 1);
    msg.IsByteEqual(mtAlive, 11);
    msg.IsByteEqual(0x80, 12);

    tx_node_ = msg.data_[13];

    head_time_msec_ = uint32_t(msg.data_[15]) << 0;
    head_time_msec_ |= uint32_t(msg.data_[16]) << 8;
    head_time_msec_ |= uint32_t(msg.data_[17]) << 16;
    head_time_msec_ |= uint32_t(msg.data_[18]) << 24;

    motor_pos_ = msg.data_[19] << 0;
    motor_pos_ |= msg.data_[20] << 8;

    std::bitset<8> head_info_ = msg.data_[21];
    in_centre_ = head_info_[0];
    centered_ = head_info_[1];
    motoring_ = head_info_[2];
    motor_on_ = head_info_[3];
    dir_ = head_info_[4];
    in_scan_ = head_info_[5];
    no_params_ = head_info_[6];
    sent_cfg_ = head_info_[7];

    msg.IsByteEqual(0x0A, 22);
  };

  void print() {
    // TODO: ROSINFO or cout?
    printf(
        "mtAliveMsg: tx_node_ = %#x head_time_msec_ = %d motor_pos_ = %d "
        "in_centre_ = %d "
        "centered_ = %d motoring_ = %d motor_on_"
        "= %d dir_ = %d in_scan_ = %d no_params_ = %d sent_cfg_ = %d\n",
        tx_node_, head_time_msec_, motor_pos_, in_centre_, centered_, motoring_,
        motor_on_, dir_, in_scan_, no_params_, sent_cfg_);
  }
};

// ######################################################################
static std::vector<uint8_t> mtRebootMsg = {
    0x40,  // header
    0x30,  // hex_length
    0x30,  // hex_length
    0x30,  // hex_length
    0x38,  // hex_length
    0x08,  // binairy_length
    0x00,  // binairy_length
    0xFF,  // tx_node
    0x02,  // rx_node
    0x03,  // n_byte
    0x10,  // mtReBoot
    0x80,  // message_sequence
    0x02,  // node
    0x0A   // line_feed
};

// ######################################################################
struct mtHeadCommandMsg {
  mtHeadCommandMsg(uint16_t n_bins = 200, float range = 10, float vos = 1500,
                   uint8_t angle_step_size = 32, uint16_t left_limit = 1,
                   uint16_t right_limit = 6399)
      : n_bins_(n_bins),
        range_(range),
        vos_(vos),
        angle_step_size_(angle_step_size),
        left_limit_(left_limit),
        right_limit_(right_limit) {}

  uint16_t n_bins_;  // The desired number of bins per scanline
  float range_;      // The desired range in meters
  float vos_;        // The velocity of sound in meters per second
  uint16_t left_limit_;
  uint16_t right_limit_;
  // The size of each step of the sonar head
  /* CrazyLow: 7.2°   = 128
     VeryLow:  3.6°   = 64
     Low:      1.8°   = 32
     Medium:   0.9°   = 16
     High:     0.45°  = 8
     Ultimate: 0.225° = 4 */
  uint8_t angle_step_size_;

  // Construct the message vector from the given parameters
  std::vector<uint8_t> construct() {
    // This is the skeleton message, we overwrite some of those bytes
    // with our parameters. We start this vector with 0x00 to match the
    // datasheet bytes numerotation.
    std::vector<uint8_t> msg = {
        0x00, 0x40, 0x30, 0x30, 0x34, 0x43, 0x4C, 0x00, 0xFF, 0x02, 0x47, 0x13,
        0x80, 0x02, 0x1D, 0x01, 0x23, 0x02, 0x99, 0x99, 0x99, 0x02, 0x66, 0x66,
        0x66, 0x05, 0xA3, 0x70, 0x3D, 0x06, 0x70, 0x3D, 0x0A, 0x09, 0x28, 0x00,
        0x3C, 0x00, 0x01, 0x00, 0xFF, 0x18, 0x51, 0x08, 0x54, 0x54, 0x5A, 0x00,
        0x7D, 0x00, 0x19, 0x10, 0x8D, 0x00, 0x5A, 0x00, 0xE8, 0x03, 0x97, 0x03,
        0x40, 0x06, 0x01, 0x00, 0x00, 0x00, 0x50, 0x51, 0x09, 0x08, 0x54, 0x54,
        0x00, 0x00, 0x5A, 0x00, 0x7D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A};

    uint16_t range_scale = uint16_t(floor(range_ * 10 + 0.5));
    msg[36] = uint8_t(range_scale & 0x00FF);
    msg[37] = uint8_t(range_scale >> 8);

    msg[51] = angle_step_size_;

    uint16_t left_limit_grads = left_limit_;

    msg[38] = uint8_t(left_limit_ & 0x00FF);
    msg[39] = uint8_t(left_limit_ >> 8);

    uint16_t rightLimGrads = right_limit_;

    msg[40] = uint8_t(right_limit_ & 0x00FF);
    msg[41] = uint8_t(right_limit_ >> 8);

    msg[54] = uint8_t(n_bins_ & 0x00FF);
    msg[55] = uint8_t(n_bins_ >> 8);

    // time travel = (2 * range / vos) in milliseconds
    double time_travel = 1000.0 * (2.0 * range_ / vos_);
    // sample time = time travel (ms) / number of bins in microseconds
    double sample_time = 1000.0 * (time_travel / double(n_bins_));
    // ADInterval = sample time (us) / 640 (us)
    // Here we round up the value
    uint16_t ad_interval = uint16_t(std::ceil(sample_time / 640.0 * 1000.0));

    msg[52] = uint8_t(ad_interval & 0x00FF);
    msg[53] = uint8_t(ad_interval >> 8);

    msg.erase(msg.begin());
    return msg;
  }
};

// ######################################################################
static std::vector<uint8_t> mtSendDataMsg = {0x40,   // header
                                             0x30,   // hex_length
                                             0x30,   // hex_length
                                             0x30,   // hex_length
                                             0x43,   // hex_length
                                             0x0C,   // binairy_length
                                             0x00,   // binairy_length
                                             0xFF,   // tx_node
                                             0x02,   // rx_node
                                             0x07,   // n_byte
                                             0x19,   // mtSendData
                                             0x80,   // message_sequence
                                             0x02,   // node
                                             0x00,   // current_time
                                             0x00,   // current_time
                                             0x00,   // current_time
                                             0x00,   // current_time
                                             0x0A};  // line_feed

// ######################################################################
struct mtHeadDataMsg {
  uint8_t packet_sequence_;   // If this is part of a multi-packet sequence
                              // which packet is this?
  bool is_last_in_sequence_;  // Is this the last packet in the sequence?
  uint8_t tx_node_;

  //! Head Status Data
  bool head_power_loss_;  // Head is in reset condition
  bool motor_error_;      // Motor has lost sync, re-send parameters.
  bool data_range_;  // When in 8-bit adc datamode, data is 0..255 = 0..80db
  bool message_appended_;  // Message appended after last packet data reply

  enum sweepCode_t {
    Scanning_Normal,
    Scan_AtLeftLimit,
    Scan_AtRightLimit,
    Scan_AtCentre
  };
  sweepCode_t sweep_code_;

  // Head Control
  struct headControl_t {
    bool adc8on_;        // Default = 0 = 4-bit packed data.
    bool cont_;          // Default = 0 = Sector Scanning.
    bool scan_right_;    // Default = 0 = ScanLeft.
    bool invert_;        // Default = 0 = Sonar mounted upright, transducer boot
                         // pointing up.
    bool motor_off_;     // Default = 0 = motor is enabled.
    bool tx_off_;        // Default = 0 = sonar transmitter is enabled.
    bool spare_;         // Default = 0 = Always.
    bool chan2_;         // Default = 0 = LF channel. Always = 0 for
                         // SeaPrince/MiniKing Sonars.
    bool raw_;           // Default = 1 = Always.
    bool has_motor_;     // Default for Sonar = 1 = sonar has motor.
    bool apply_offset_;  // Default = 0 = Do not apply Heading Offsets.
    bool ping_pong_;     // Default for scanning sonar = 0, Default for
                         // Sidescan sonar = 1.
    bool stare_left_limit_;  // Default = 0 = Don’t "Stare" in fixed direction.
    bool reply_asl_;         // Default = 1 = Always for Sonar.
    bool reply_thr_;         // Default = 0 = Always.
    bool ignore_sensor_;     // Default = 0 = Always, 1 in emergencies.
  };
  headControl_t head_control_;

  float ranges_scale_;
  enum rangeUnits_t { meters = 0, feet = 1, fathoms = 2, yards = 3 };
  rangeUnits_t range_units_;

  float step_size_degrees_;
  float bearing_degrees_;  //!< The current bearing of the sonar head

  std::vector<uint8_t> scanLine;

  void print() {
    std::cout << "mtHeadDataMsg: " << std::endl;
    std::cout << "   packetInSequence: " << int(packet_sequence_) << std::endl;
    std::cout << "   isLast?: " << is_last_in_sequence_ << std::endl;
    std::cout << "   Error?: " << motor_error_ << std::endl;
    std::cout << "   Sweep Code: " << sweep_code_ << std::endl;
    std::cout << "   Adc8on?: " << head_control_.adc8on_ << std::endl;
    std::cout << "   Continuos Scan?: " << head_control_.cont_ << std::endl;
    std::cout << "   scanright?: " << head_control_.scan_right_ << std::endl;
    std::cout << "   invert?: " << head_control_.invert_ << std::endl;
    std::cout << "   motoff?: " << head_control_.motor_off_ << std::endl;
    std::cout << "   txoff?: " << head_control_.tx_off_ << std::endl;
    std::cout << "   spare?: " << head_control_.spare_ << std::endl;
    std::cout << "   Range Scale: " << ranges_scale_ << std::endl;
    std::cout << "   Range Scale Units: " << range_units_ << std::endl;
    std::cout << "   Step Size (degrees): " << step_size_degrees_ << std::endl;
    std::cout << "   Current Bearing (degrees): " << bearing_degrees_
              << std::endl;

    std::cout << "   Scanline size: " << scanLine.size() << std::endl;

    // std::cout << "   Scanline [ ";
    // for(uint8_t c : scanLine)
    //	std::cout << int(c) << " ";
    // std::cout << "]" << std::endl;
  }

  mtHeadDataMsg(SonarMessage const &msg) {
    msg.IsByteEqual('@', 1);
    msg.IsByteEqual(mtHeadData, 11);

    uint8_t msg_sequence = msg.data_[12];
    packet_sequence_ = msg_sequence & uint8_t(0xEF);
    is_last_in_sequence_ = msg_sequence & uint8_t(0x80);

    std::bitset<8> head_status = msg.data_[17];
    head_power_loss_ = head_status[0];
    motor_error_ = head_status[1];
    data_range_ = head_status[4];
    message_appended_ = head_status[7];

    switch (msg.data_[18]) {
      case 0:
        sweep_code_ = Scanning_Normal;
        break;
      case 1:
        sweep_code_ = Scan_AtLeftLimit;
        break;
      case 2:
        sweep_code_ = Scan_AtRightLimit;
        break;
      case 5:
        sweep_code_ = Scan_AtCentre;
        break;
      default:
        std::cerr << "Unknown Sweep Code (" << msg.data_[18] << ")"
                  << std::endl;
        break;
    }

    std::bitset<16> head_control =
        msg.data_[19] | (uint16_t(msg.data_[20]) << 8);
    head_control_.adc8on_ = head_control[0];
    head_control_.cont_ = head_control[1];
    head_control_.scan_right_ = head_control[2];
    head_control_.invert_ = head_control[3];
    head_control_.motor_off_ = head_control[4];
    head_control_.tx_off_ = head_control[5];
    head_control_.spare_ = head_control[6];
    head_control_.chan2_ = head_control[7];
    head_control_.raw_ = head_control[8];
    head_control_.has_motor_ = head_control[9];
    head_control_.apply_offset_ = head_control[10];
    head_control_.ping_pong_ = head_control[11];
    head_control_.stare_left_limit_ = head_control[12];
    head_control_.reply_asl_ = head_control[13];
    head_control_.reply_thr_ = head_control[14];
    head_control_.ignore_sensor_ = head_control[15];

    ranges_scale_ =
        ((uint16_t(msg.data_[21]) | (uint16_t(msg.data_[22]) << 8)) & 0xC0FF) /
        10;
    uint8_t range_unit = uint8_t(msg.data_[22]) >> 6;
    switch (range_unit) {
      case 0:
        range_units_ = meters;
        break;
      case 1:
        range_units_ = feet;
        break;
      case 2:
        range_units_ = fathoms;
        break;
      case 3:
        range_units_ = yards;
        break;
      default:
        std::cerr << "Unknown range units Code (" << (int)range_unit << ")"
                  << std::endl;
        break;
    }

    step_size_degrees_ = msg.data_[40] / 16.0 * 180.0 / 200.0;
    bearing_degrees_ =
        (uint16_t(msg.data_[41]) | (uint16_t(msg.data_[42]) << 8)) / 16.0 *
        180.0 / 200.0;

    uint16_t dbytes =
        (uint16_t(msg.data_[43]) | (uint16_t(msg.data_[44]) << 8));
    if (head_control_.adc8on_) {
      scanLine.resize(dbytes);
      if (scanLine.size() != msg.data_.size() - 46) {
        std::cerr << __FUNCTION__ << ":" << __LINE__
                  << " - scanLine appears to be mis-sized.  Size is "
                  << scanLine.size() << ", but should be "
                  << msg.data_.size() - 46 << std::endl;
        return;
      }

      // TODO: std::copy here
      for (size_t i = 0; i < scanLine.size(); ++i)
        scanLine[i] = msg.data_.at(i + 45);
    } else {
      scanLine.resize(dbytes * 2);
      if (scanLine.size() != msg.data_.size() / 2 - 46) {
        std::cerr << __FUNCTION__ << ":" << __LINE__
                  << " - scanLine appears to be mis-sized.  Size is "
                  << scanLine.size() << ", but should be "
                  << msg.data_.size() - 46 << std::endl;
        return;
      }
      std::cerr << __FUNCTION__ << ":" << __LINE__
                << " - Your device is transmitting in packed 4-bit mode. This "
                   "is not yet supported." << std::endl;
      return;
    }
  }
};
}

#endif  // TRITECHMICRON_MESSAGETYPES_H
