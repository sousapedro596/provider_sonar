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

  bool LenghtCheck(uint8_t length) const {
    if (data.size() != length) {
      std::cerr << "Invalid message length (" << data.size() << " != " << length
                << ")" << std::endl;
      return false;
    }
    return true;
  }

  bool IdCheck(MessageID id) const {
    if (id != this->id) {
      std::cerr << "Invalid message type (" << id << " != " << id << ")"
                << std::endl;
      return false;
    }
    return true;
  }

  bool IsByteEqual(uint8_t byte, uint8_t at) const {
    if (data.size() < at) {
      std::cerr << "Message too short" << std::endl;
      return false;
    }
    if (data[at] != byte) {
      std::cerr << "Expected " << std::hex << byte << std::dec << " @" << at
                << " but got " << std::hex << data[at] << std::endl;
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
  const MessageID id = mtVersionData;
  uint8_t tx_node;
  uint8_t software_version;
  uint8_t info_bits;
  uint16_t uid;
  uint32_t program_length;
  uint16_t checksum;

  mtVersionDataMsg(SonarMessage const &msg) {
    msg.LenghtCheck(26);
    msg.IdCheck(mtVersionData);
    msg.IsByteEqual('@', 1);
    msg.IsByteEqual(mtVersionData, 11);

    tx_node = msg.data[13];

    software_version = msg.data[14];

    info_bits = msg.data[15];

    uid = uint16_t(msg.data[16] << 0);
    uid |= uint16_t(msg.data[17] << 8);

    program_length = msg.data[18] << 0;
    program_length |= msg.data[19] << 8;
    program_length |= msg.data[20] << 16;
    program_length |= msg.data[21] << 24;

    checksum = msg.data[22] << 0;
    checksum += msg.data[23] << 8;

    msg.IsByteEqual(0x0A, 25);
  }

  void print() {
    printf(
        "mtVersionDataMsg: tx_node_ = %#x software_version_ = %#x info_bits_ = "
        "%#x uid_ = %#x "
        "program_length_ = %d checksum_ = %d\n",
        tx_node, software_version, info_bits, uid, program_length, checksum);
  }
};

//------------------------------------------------------------------------------
//
struct mtAliveMsg {
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

  mtAliveMsg(SonarMessage const &msg) {
    msg.IsByteEqual('@', 1);
    msg.IsByteEqual(mtAlive, 11);
    msg.IsByteEqual(0x80, 12);

    tx_node = msg.data[13];

    head_time_msec = msg.data[15] << 0;
    head_time_msec |= msg.data[16] << 8;
    head_time_msec |= msg.data[17] << 16;
    head_time_msec |= msg.data[18] << 24;

    motor_pos = msg.data[19] << 0;
    motor_pos |= msg.data[20] << 8;

    std::bitset<8> head_info_ = msg.data[21];
    in_centre = head_info_[0];
    centered = head_info_[1];
    motoring = head_info_[2];
    motor_on = head_info_[3];
    dir = head_info_[4];
    in_scan = head_info_[5];
    no_params = head_info_[6];
    sent_cfg = head_info_[7];

    msg.IsByteEqual(0x0A, 22);
  };

  void print() {
    // TODO: ROSINFO or cout?
    printf(
        "mtAliveMsg: tx_node_ = %#x head_time_msec_ = %d motor_pos_ = %d "
        "in_centre_ = %d "
        "centered_ = %d motoring_ = %d motor_on_"
        "= %d dir_ = %d in_scan_ = %d no_params_ = %d sent_cfg_ = %d\n",
        tx_node, head_time_msec, motor_pos, in_centre, centered, motoring,
        motor_on, dir, in_scan, no_params, sent_cfg);
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
  mtHeadCommandMsg(uint16_t n_bins = 200, float range = 10, float vos = 1500,
                   uint8_t angle_step_size = 32, uint16_t left_limit = 1,
                   uint16_t right_limit = 6399)
      : n_bins(n_bins),
        range(range),
        vos(vos),
        step_angle_size(angle_step_size),
        left_limit(left_limit),
        right_limit(right_limit) {}

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

    uint16_t range_scale = uint16_t(floor(range * 10 + 0.5));
    msg[36] = static_cast<uint8_t>(range_scale & 0x00FF);
    msg[37] = static_cast<uint8_t>(range_scale >> 8);

    msg[51] = step_angle_size;

    uint16_t left_limit_grads = left_limit;

    msg[38] = static_cast<uint8_t>(left_limit & 0x00FF);
    msg[39] = static_cast<uint8_t>(left_limit >> 8);

    uint16_t rightLimGrads = right_limit;

    msg[40] = static_cast<uint8_t>(right_limit & 0x00FF);
    msg[41] = static_cast<uint8_t>(right_limit >> 8);

    msg[54] = static_cast<uint8_t>(n_bins & 0x00FF);
    msg[55] = static_cast<uint8_t>(n_bins >> 8);

    // time travel = (2 * range / vos) in milliseconds
    double time_travel = 1000.0 * (2.0 * range / vos);
    // sample time = time travel (ms) / number of bins in microseconds
    double sample_time = 1000.0 * (time_travel / double(n_bins));
    // ADInterval = sample time (us) / 640 (us)
    // Here we round up the value
    uint16_t ad_interval =
        static_cast<uint16_t>(std::ceil(sample_time / 640.0 * 1000.0));

    msg[52] = static_cast<uint8_t>(ad_interval & 0x00FF);
    msg[53] = static_cast<uint8_t>(ad_interval >> 8);

    msg.erase(msg.begin());
    return msg;
  }
};

//------------------------------------------------------------------------------
//
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
  float ranges_scale;
  RangeUnits range_units;
  float step_angle_size;
  float transducer_bearing;  // The current bearing of the sonar head
  std::vector<uint8_t> scanline;

  //============================================================================
  // P U B L I C   M E T H O D S
  void print() {
    std::cout << "mtHeadDataMsg: " << std::endl;
    std::cout << "   packetInSequence: " << int(packet_sequence) << std::endl;
    std::cout << "   isLast?: " << is_last_in_sequence << std::endl;
    std::cout << "   Error?: " << head_status.motor_error << std::endl;
    std::cout << "   Sweep Code: " << sweep_code << std::endl;
    std::cout << "   Adc8on?: " << head_control.adc8on << std::endl;
    std::cout << "   Continuos Scan?: " << head_control.cont << std::endl;
    std::cout << "   scanright?: " << head_control.scan_right << std::endl;
    std::cout << "   invert?: " << head_control.invert << std::endl;
    std::cout << "   motoff?: " << head_control.motor_off << std::endl;
    std::cout << "   txoff?: " << head_control.tx_off << std::endl;
    std::cout << "   spare?: " << head_control.spare << std::endl;
    std::cout << "   Range Scale: " << ranges_scale << std::endl;
    std::cout << "   Range Scale Units: " << range_units << std::endl;
    std::cout << "   Step Size (degrees): " << step_angle_size << std::endl;
    std::cout << "   Current Bearing (degrees): " << transducer_bearing
              << std::endl;

    std::cout << "   Scanline size: " << scanline.size() << std::endl;

    // std::cout << "   Scanline [ ";
    // for(uint8_t c : scanLine)
    //	std::cout << int(c) << " ";
    // std::cout << "]" << std::endl;
  }

  mtHeadDataMsg(SonarMessage const &msg) {
    msg.IsByteEqual('@', 1);
    msg.IsByteEqual(mtHeadData, 11);

    uint8_t msg_sequence = msg.data[12];
    packet_sequence = msg_sequence & static_cast<uint8_t>(0xEF);
    is_last_in_sequence = msg_sequence & static_cast<uint8_t>(0x80);

    std::bitset<8> head_status_bitset = msg.data[17];
    head_status.head_power_loss = head_status_bitset[0];
    head_status.motor_error = head_status_bitset[1];
    head_status.data_range = head_status_bitset[4];
    head_status.message_appended = head_status_bitset[7];

    switch (msg.data[18]) {
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
        std::cerr << "Unknown Sweep Code (" << msg.data[18] << ")" << std::endl;
        break;
    }

    std::bitset<16> head_control_bitset =
        msg.data[19] | static_cast<uint16_t>(msg.data[20]) << 8;
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

    ranges_scale = ((static_cast<uint16_t>(msg.data[21]) |
                     (static_cast<uint16_t>(msg.data[22]) << 8)) &
                    0xC0FF) /
                   10;

    switch (msg.data[22] >> 6) {
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
        std::cerr << "Unknown range units Code (" << (msg.data[22] >> 6) << ")"
                  << std::endl;
        break;
    }

    // Step angle size is giving in degree (1 Gradian = 0.9°)
    step_angle_size = msg.data[40] / 16.0f * 180.0f / 200.0f;
    // Transducer bearing is giving in degree
    transducer_bearing =
        (msg.data[41] | (msg.data[42]) << 8) / 16.0f * 180.0f / 200.0f;

    uint16_t data_bytes =
        (uint16_t(msg.data[43]) | (uint16_t(msg.data[44]) << 8));

    // If adc8on = 1, data_bytes = n_bin
    // Else, data_bytes = n_bin / 2
    if (head_control.adc8on) {
      scanline.resize(static_cast<size_t>(data_bytes));
      // Check if the scanline take the right size of msg.data
      if (scanline.size() != msg.data.size() - 46) {
        std::cerr << "Scanline appears to be mis-sized.  Size is "
                  << scanline.size() << ", but should be "
                  << msg.data.size() - 46 << std::endl;
        return;
      }

      scanline = msg.data;
      for (size_t i = 0; i < scanline.size(); ++i)
        std::cout << "vector = : " << scanline[i];

      //      std::copy(msg.data.begin(), msg.data.end(), scanline);
      //      for (size_t i = 0; i < scanline.size(); ++i)
      //        std::cout << "std:copy : " << scanline[i];

      for (size_t i = 0; i < scanline.size(); ++i)
        scanline[i] = msg.data.at(i + 45);
      for (size_t i = 0; i < scanline.size(); ++i)
        std::cout << "for loop : " << scanline[i];

    } else {
      scanline.resize(static_cast<size_t>(data_bytes * 2));
      if (scanline.size() != msg.data.size() / 2 - 46) {
        std::cerr << __FUNCTION__ << ":" << __LINE__
                  << " - scanLine appears to be mis-sized.  Size is "
                  << scanline.size() << ", but should be "
                  << msg.data.size() - 46 << std::endl;
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
