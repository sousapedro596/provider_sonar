/**
 * \file	sonar_messages.h
 * \author  Francis Masse <francis.masse05@gmail.com>
 * \date	26/02/2016
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
 *
 * \section contribution
 *
 * A special thanks to Randolph Voorhies to strongly inspire us in the
 * developement of this file.
 *
 * Copyright (C) 2011  Randolph Voorhies
 */

#ifndef PROVIDER_SONAR_SONAR_MESSAGES_H
#define PROVIDER_SONAR_SONAR_MESSAGES_H

#include <provider_sonar/sonar_driver.h>
#include "messages_id.h"
#include <iostream>
#include "ros/ros.h"
#include <vector>
#include <bitset>
#include <cmath>

namespace provider_sonar {

struct SonarMessage {
  //============================================================================
  // P U B L I C   M E M B E R S

  uint8_t header;
  uint32_t hex_length;
  uint16_t binary_length;
  uint8_t tx_node;
  uint8_t rx_node;
  uint8_t n_byte;
  provider_sonar::MessageID id;
  uint8_t message_sequence;
  uint8_t node;
  std::vector<uint8_t> data;
  uint8_t line_feed;

  //============================================================================
  // C / D T O R S   S E C T I O N

  //----------------------------------------------------------------------------
  //
  SonarMessage() {
    header = 0;
    hex_length = 0;
    binary_length = 0;
    tx_node = 0;
    rx_node = 0;
    n_byte = 0;
    id = mtNull;
    message_sequence = 0;
    node = 0;
    data.clear();
    line_feed = 0;
  }

  //============================================================================
  // P U B L I C   M E T H O D S

  //----------------------------------------------------------------------------
  //
  bool LenghtCheck(uint8_t length) const {
    if (data.size() != length) {
      ROS_ERROR("Invalid message length (%lu != %u)", data.size(), length);
      return false;
    }
    return true;
  }

  //----------------------------------------------------------------------------
  //
  bool IdCheck(MessageID id) const {
    if (id != this->id) {
      ROS_ERROR("Invalid message type (%d != %d)", static_cast<int>(id),
                static_cast<int>(this->id));
      return false;
    }
    return true;
  }

  //----------------------------------------------------------------------------
  //
  bool IsByteEqual(uint8_t byte, uint8_t at) const {
    if (data.size() < at) {
      ROS_ERROR("Message too short");
      return false;
    }
    if (data[at] != byte) {
      ROS_ERROR("Expected %X @ %u but got %X", byte, at, data[at]);
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
  //============================================================================
  // P U B L I C   M E M B E R S

  const MessageID id = mtVersionData;
  uint8_t tx_node;
  uint8_t software_version;
  uint8_t info_bits;
  uint16_t uid;
  uint32_t program_length;
  uint16_t checksum;

  //============================================================================
  // P U B L I C   M E T H O D S

  //----------------------------------------------------------------------------
  //
  mtVersionDataMsg(SonarMessage const &msg) {
    msg.LenghtCheck(25);
    msg.IdCheck(mtVersionData);
    msg.IsByteEqual('@', 0);
    msg.IsByteEqual(mtVersionData, 10);

    tx_node = msg.data[12];

    software_version = msg.data[13];

    info_bits = msg.data[14];

    uid = uint16_t(msg.data[15] << 0);
    uid |= uint16_t(msg.data[16] << 8);

    program_length = msg.data[17] << 0;
    program_length |= msg.data[18] << 8;
    program_length |= msg.data[19] << 16;
    program_length |= msg.data[20] << 24;

    checksum = msg.data[21] << 0;
    checksum += msg.data[22] << 8;

    msg.IsByteEqual(0x0A, 24);
  }

  //----------------------------------------------------------------------------
  //
  void Print() {
    ROS_INFO("mtVersionDataMsg");
    ROS_INFO("tx_node = %#x", tx_node);
    ROS_INFO("software_version = %#x", software_version);
    ROS_INFO("info_bits = %#x", info_bits);
    ROS_INFO("uid = %#x", uid);
    ROS_INFO("program_length = %d", program_length);
    ROS_INFO("checksum = %d", checksum);
  }
};

//------------------------------------------------------------------------------
//
struct mtAliveMsg {
  //============================================================================
  // P U B L I C   M E M B E R S

  const MessageID id = mtAlive;
  uint8_t tx_node;
  uint32_t head_time_msec;
  int motor_pos;

  // Head Info BitSet
  bool in_centre;
  bool centered;
  bool motoring;
  bool motor_on;
  bool dir;
  bool in_scan;
  bool no_params;
  bool sent_cfg;

  //============================================================================
  // P U B L I C   M E T H O D S

  //----------------------------------------------------------------------------
  //
  mtAliveMsg(SonarMessage const &msg) {
    msg.IsByteEqual('@', 0);
    msg.IsByteEqual(mtAlive, 10);
    msg.IsByteEqual(0x80, 11);

    tx_node = msg.data[12];

    head_time_msec = msg.data[14] << 0;
    head_time_msec |= msg.data[15] << 8;
    head_time_msec |= msg.data[16] << 16;
    head_time_msec |= msg.data[17] << 24;

    motor_pos = msg.data[18] << 0;
    motor_pos |= msg.data[19] << 8;

    std::bitset<8> head_info_ = msg.data[20];
    in_centre = head_info_[0];
    centered = head_info_[1];
    motoring = head_info_[2];
    motor_on = head_info_[3];
    dir = head_info_[4];
    in_scan = head_info_[5];
    no_params = head_info_[6];
    sent_cfg = head_info_[7];

    msg.IsByteEqual(0x0A, 21);
  };

  //----------------------------------------------------------------------------
  //
  void Print() {
    ROS_INFO("mtAliveMsg");
    ROS_INFO("tx_node = %#X", tx_node);
    ROS_INFO("head_time_msec = %d", head_time_msec);
    ROS_INFO("motor_pos = %d", motor_pos);
    ROS_INFO("in_centre = %d ", in_centre);
    ROS_INFO("centered = %d ", centered);
    ROS_INFO("motoring = %d ", motoring);
    ROS_INFO("motor_on = %d", motor_on);
    ROS_INFO("dir = %d", in_scan);
    ROS_INFO("in_scan = %d", dir);
    ROS_INFO("no_params = %d", no_params);
    ROS_INFO("sent_cfg = %d", sent_cfg);
  }
};

//------------------------------------------------------------------------------
//
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

//------------------------------------------------------------------------------
//
struct mtHeadCommandMsg {
  //============================================================================
  // C / D T O R S   S E C T I O N

  //----------------------------------------------------------------------------
  //
  mtHeadCommandMsg(uint16_t n_bins = 400, float range = 8.0f,
                   float vos = 1500.0f, uint16_t left_limit = 2400,
                   uint16_t right_limit = 4000, uint8_t step_angle_size = 16,
                   uint8_t ad_span = 28, uint8_t ad_low = 20)
      : n_bins(n_bins),
        range(range),
        vos(vos),
        left_limit(left_limit),
        right_limit(right_limit),
        step_angle_size(step_angle_size),
        ad_span(ad_span),
        ad_low(ad_low) {}

  //============================================================================
  // P U B L I C   M E M B E R S

  uint16_t n_bins;  // The desired number of bins per scanline
  float range;      // The desired range in meters
  float vos;        // The velocity of sound in meters per second
  uint16_t left_limit;
  uint16_t right_limit;
  // The size of each step of the sonar head
  /* CrazyLow: 7.2°   = 128
     VeryLow:  3.6°   = 64
     Low:      1.8°   = 32
     Medium:   0.9°   = 16
     High:     0.45°  = 8
     Ultimate: 0.225° = 4 */
  uint8_t step_angle_size;

  uint8_t ad_span;  // Set the width of the sampling window in dB
  uint8_t ad_low;   // Set the low bound of the sampling window in dB

  //============================================================================
  // P U B L I C   M E T H O D S

  // Construct the message vector from the given parameters
  std::vector<uint8_t> Construct() {
    // This is the skeleton message, we overwrite some of those bytes
    // with our parameters. We start this vector with 0x00 to match the
    // datasheet bytes numerotation.
    std::vector<uint8_t> msg = {
        0x40, 0x30, 0x30, 0x34, 0x43, 0x4C, 0x00, 0xFF, 0x02, 0x47, 0x13, 0x80,
        0x02, 0x1D, 0x05, 0x23, 0x11, 0x99, 0x99, 0x99, 0x05, 0xE1, 0x7A, 0x14,
        0x00, 0x99, 0x99, 0x99, 0x05, 0xEB, 0x51, 0xB8, 0x03, 0x32, 0x00, 0x64,
        0x00, 0x60, 0x09, 0xA0, 0x0F, 0x59, 0x3F, 0x69, 0x64, 0x64, 0x00, 0x00,
        0x00, 0x19, 0x10, 0x1A, 0x00, 0x2F, 0x03, 0xE8, 0x03, 0xF4, 0x01, 0x40,
        0x06, 0x01, 0x00, 0x00, 0x00, 0x59, 0x32, 0x3F, 0x00, 0x69, 0x64, 0x00,
        0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A};


    uint16_t range_scale = uint16_t(floor(range * 10 + 0.5));
    msg[35] = static_cast<uint8_t>(range_scale & 0x00FF);
    msg[36] = static_cast<uint8_t>(range_scale >> 8);

    msg[50] = step_angle_size;

    msg[37] = static_cast<uint8_t>(left_limit & 0x00FF);
    msg[38] = static_cast<uint8_t>(left_limit >> 8);

    msg[39] = static_cast<uint8_t>(right_limit & 0x00FF);
    msg[40] = static_cast<uint8_t>(right_limit >> 8);

    msg[53] = static_cast<uint8_t>(n_bins & 0x00FF);
    msg[54] = static_cast<uint8_t>(n_bins >> 8);

    // time travel = (2 * range / vos) in milliseconds
    double time_travel = 1000.0 * (2.0 * range / vos);
    // sample time = time travel (ms) / number of bins in microseconds
    double sample_time = 1000.0 * (time_travel / double(n_bins));
    // ADInterval = sample time (us) / 640 (us)
    // Here we round up the value
    uint16_t ad_interval =
        static_cast<uint16_t>(std::ceil(sample_time / 640.0 * 1000.0));

    // AD Span is sent back to user in dB
    msg[41] = static_cast<uint8_t>(255 * ad_span / 80);

    // AD Low is sent back to user in dB
    msg[42] = static_cast<uint8_t>(255 * ad_low / 80);

    msg[51] = static_cast<uint8_t>(ad_interval & 0x00FF);
    msg[52] = static_cast<uint8_t>(ad_interval >> 8);

    return msg;
  }
};

//------------------------------------------------------------------------------
//
static std::vector<uint8_t> mtSendDataMsg = {
    0x40,  // header
    0x30,  // hex_length
    0x30,  // hex_length
    0x30,  // hex_length
    0x43,  // hex_length
    0x0C,  // binairy_length
    0x00,  // binairy_length
    0xFF,  // tx_node
    0x02,  // rx_node
    0x07,  // n_byte
    0x19,  // mtSendData
    0x80,  // message_sequence
    0x02,  // node
    0x00,  // current_time
    0x00,  // current_time
    0x00,  // current_time
    0x00,  // current_time
    0x0A   // line_feed
};

//------------------------------------------------------------------------------
//
struct mtHeadDataMsg {
  //============================================================================
  // T Y P E D E F   A N D   E N U M
  enum SweepCode {
    Scanning_Normal,
    Scan_AtLeftLimit,
    Scan_AtRightLimit,
    Scan_AtCentre
  };

  struct HeadStatus {
    bool head_power_loss;  // Head is in reset condition
    bool motor_error;      // Motor has lost sync, re-send parameters.
    bool data_range;  // When in 8-bit adc datamode, data is 0..255 = 0..80db
    bool message_appended;  // Message appended after last packet data reply
  };

  struct HeadControl {
    bool adc8on;        // Default = 0 = 4-bit packed data.
    bool cont;          // Default = 0 = Sector Scanning.
    bool scan_right;    // Default = 0 = ScanLeft.
    bool invert;        // Default = 0 = Sonar mounted upright, transducer boot
                        // pointing up.
    bool motor_off;     // Default = 0 = motor is enabled.
    bool tx_off;        // Default = 0 = sonar transmitter is enabled.
    bool spare;         // Default = 0 = Always.
    bool chan2;         // Default = 0 = LF channel. Always = 0 for
                        // SeaPrince/MiniKing Sonars.
    bool raw;           // Default = 1 = Always.
    bool has_motor;     // Default for Sonar = 1 = sonar has motor.
    bool apply_offset;  // Default = 0 = Do not apply Heading Offsets.
    bool ping_pong;     // Default for scanning sonar = 0, Default for
                        // Sidescan sonar = 1.
    bool stare_left_limit;  // Default = 0 = Don’t "Stare" in fixed direction.
    bool reply_asl;         // Default = 1 = Always for Sonar.
    bool reply_thr;         // Default = 0 = Always.
    bool ignore_sensor;     // Default = 0 = Always, 1 in emergencies.
  };

  enum RangeUnits { meters = 0, feet = 1, fathoms = 2, yards = 3 };

  //============================================================================
  // P U B L I C   M E M B E R S
  uint8_t packet_sequence;   // If this is part of a multi-packet sequence
                             // which packet is this?
  bool is_last_in_sequence;  // Is this the last packet in the sequence?
  uint8_t tx_node;           // Tx Node number (copy of Byte 8)
  HeadStatus head_status;
  SweepCode sweep_code;
  HeadControl head_control;
  float range_scale;
  RangeUnits range_units;
  float step_angle_size;
  float transducer_bearing;  // The current bearing of the sonar head
  uint8_t ad_span;
  uint8_t ad_low;
  std::vector<uint8_t> scanline;

  //============================================================================
  // P U B L I C   M E T H O D S
  void Print() {
    ROS_INFO("mtHeadDataMsg: ");
    ROS_INFO("  packet_sequence: %d", static_cast<int>(packet_sequence));
    ROS_INFO("  is_last_in_sequence : %s",
             is_last_in_sequence ? "true" : "false");
    ROS_INFO("  head_status/motor_erro : %s",
             head_status.motor_error ? "true" : "false");
    ROS_INFO("  sweep_code : %d", sweep_code);
    ROS_INFO("  head_control/adc8on : %s",
             head_control.adc8on ? "true" : "false");
    ROS_INFO("  head_control/cont : %s", head_control.cont ? "true" : "false");
    ROS_INFO("  head_control/scan_right : %s",
             head_control.scan_right ? "true" : "false");
    ROS_INFO("  head_control/invert : %s",
             head_control.invert ? "true" : "false");
    ROS_INFO("  head_control/motor_off : %s",
             head_control.motor_off ? "true" : "false");
    ROS_INFO("  head_control/tx_off : %s",
             head_control.tx_off ? "true" : "false");
    ROS_INFO("  range_scale : %.2f %s", range_scale,
             range_units ? "feet" : "meter");
    ROS_INFO("  step_angle_size (Degree) : %.2f", step_angle_size);
    ROS_INFO("  transducer_bearing (Degree) : %.2f", transducer_bearing);
    ROS_INFO("  ad_span : %d", static_cast<int>(ad_span));
    ROS_INFO("  ad_low : %d", static_cast<int>(ad_low));
    ROS_INFO("  scanline.size : %lu", scanline.size());
  }

  mtHeadDataMsg(SonarMessage const &msg) {
    msg.IsByteEqual('@', 0);
    msg.IsByteEqual(mtHeadData, 10);

    uint8_t msg_sequence = msg.data[11];
    packet_sequence = msg_sequence & static_cast<uint8_t>(0xEF);
    is_last_in_sequence = msg_sequence & static_cast<uint8_t>(0x80);

    std::bitset<8> head_status_bitset = msg.data[16];
    head_status.head_power_loss = head_status_bitset[0];
    head_status.motor_error = head_status_bitset[1];
    head_status.data_range = head_status_bitset[4];
    head_status.message_appended = head_status_bitset[7];

    switch (msg.data[17]) {
      case 0:
        sweep_code = Scanning_Normal;
        break;
      case 1:
        sweep_code = Scan_AtLeftLimit;
        break;
      case 2:
        sweep_code = Scan_AtRightLimit;
        break;
      case 5:
        sweep_code = Scan_AtCentre;
        break;
      default:
        ROS_ERROR("Unknown Sweep Code (%u)", msg.data[17]);
        break;
    }

    std::bitset<16> head_control_bitset =
        msg.data[18] | static_cast<uint16_t>(msg.data[19]) << 8;
    head_control.adc8on = head_control_bitset[0];
    head_control.cont = head_control_bitset[1];
    head_control.scan_right = head_control_bitset[2];
    head_control.invert = head_control_bitset[3];
    head_control.motor_off = head_control_bitset[4];
    head_control.tx_off = head_control_bitset[5];
    head_control.spare = head_control_bitset[6];
    head_control.chan2 = head_control_bitset[7];
    head_control.raw = head_control_bitset[8];
    head_control.has_motor = head_control_bitset[9];
    head_control.apply_offset = head_control_bitset[10];
    head_control.ping_pong = head_control_bitset[11];
    head_control.stare_left_limit = head_control_bitset[12];
    head_control.reply_asl = head_control_bitset[13];
    head_control.reply_thr = head_control_bitset[14];
    head_control.ignore_sensor = head_control_bitset[15];

    range_scale = ((static_cast<uint16_t>(msg.data[20]) |
                    (static_cast<uint16_t>(msg.data[21]) << 8)) &
                   0xC0FF) /
                  10;

    switch (msg.data[21] >> 6) {
      case 0:
        range_units = meters;
        break;
      case 1:
        range_units = feet;
        break;
      case 2:
        range_units = fathoms;
        break;
      case 3:
        range_units = yards;
        break;
      default:
        ROS_ERROR("Unknown range units Code (%u)", (msg.data[21] >> 6));
        break;
    }

    // Step angle size is giving in degree (1 Gradian = 0.9°)
    step_angle_size = msg.data[39] / 16.0f * 180.0f / 200.0f;
    // Transducer bearing is giving in degree
    transducer_bearing =
        (msg.data[41] | (msg.data[41]) << 8) / 16.0f * 180.0f / 200.0f;

    // AD Span is sent back to user in dB
    ad_span = static_cast<uint8_t>(msg.data[29] / 255.0f * 80.0f);

    // AD Low is sent back to user in dB
    ad_low = static_cast<uint8_t>(msg.data[30] / 255.0f * 80.0f);

    uint16_t data_bytes =
        static_cast<uint16_t>((msg.data[42]) | (uint16_t(msg.data[43]) << 8));

    // If adc8on = 1, data_bytes = n_bin
    // Else, data_bytes = n_bin / 2
    if (head_control.adc8on) {
      scanline.resize(static_cast<size_t>(data_bytes));
      // Check if the scanline take the right size of msg.data
      if (scanline.size() != msg.data.size() - 46) {
        ROS_ERROR_STREAM("Scanline appears to be mis-sized.  Size is "
                         << scanline.size() << ", but should be "
                         << msg.data.size() - 46);
        return;
      }

      // Copy msg.data to scanline vector
      for (size_t i = 0; i < scanline.size(); ++i)
        scanline[i] = msg.data.at(i + 45);

    } else {
      scanline.resize(static_cast<size_t>(data_bytes * 2));
      if (scanline.size() != msg.data.size() / 2 - 46) {
        ROS_ERROR_STREAM("ScanLine appears to be mis-sized.  Size is "
                         << scanline.size() << ", but should be "
                         << msg.data.size() - 46);
        return;
      }
      ROS_ERROR_STREAM(
          "Your device is transmitting in packed 4-bit mode. This "
          "is not yet supported.");
      return;
    }
  }
};

//------------------------------------------------------------------------------
//
struct mtHeadCommandShortMsg {
  //============================================================================
  // C / D T O R S   S E C T I O N

  //----------------------------------------------------------------------------
  //
  mtHeadCommandShortMsg(uint8_t ad_span = 28, uint8_t ad_low = 20)
      : ad_span(ad_span),
        ad_low(ad_low) {}

  //============================================================================
  // P U B L I C   M E M B E R S

  uint8_t ad_span;  // Set the width of the sampling window in dB
  uint8_t ad_low;   // Set the low bound of the sampling window in dB

  //============================================================================
  // P U B L I C   M E T H O D S

  // Construct the message vector from the given parameters
  std::vector<uint8_t> Construct() {
    // This is the skeleton message, we overwrite some of those bytes
    // with our parameters. We start this vector with 0x00 to match the
    // datasheet bytes numerotation.
    std::vector<uint8_t> msg = {
        0x40, 0x30, 0x30, 0x31, 0x39, 0x19, 0x00, 0xFF, 0x02, 0x14, 0x13, 0x80,
        0x02, 0x1E, 0x5C, 0x32, 0x40, 0x00, 0x69, 0x64, 0x00, 0x00, 0x64, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A};

    // AD Span is sent back to user in dB
    msg[14] = static_cast<uint8_t>(255 * ad_span / 80);

    // AD Low is sent back to user in dB
    msg[16] = static_cast<uint8_t>(255 * ad_low / 80);

    return msg;
  }
};

}  // namespace provider_sonar

#endif  // PROVIDER_SONAR_SONAR_MESSAGES_H
