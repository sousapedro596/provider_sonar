// ######################################################################
//
//      SonarDriver - A protocol parser for Tritech Micron sonars.
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

#include "provider_sonar/sonar_driver.h"
#include "sonar_messages/sonar_messages.h"
#include <iostream>
#include <algorithm>
#include <chrono>
#include "stdint.h"

namespace provider_sonar {
//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
SonarDriver::SonarDriver(uint16_t n_bins, float range, float vos,
                         uint8_t angle_step_size, uint16_t left_limit,
                         uint16_t right_limit, uint8_t ad_span, uint8_t ad_low,
                         bool debug_mode)
    : its_serial_thread_(),
      its_processing_thread_(),
      its_state_(WaitingForAt),
      state_machine_semaphore_(green),
      scanning_callback_semaphore_(green),
      state_(waitingforMtAlive_1),
      its_raw_msg_(),
      its_msg_(),
      has_heard_mtAlive_(false),
      has_heard_mtVersionData_(false),
      has_heard_mtHeadData_(false),
      has_params_(false),
      serial_(),
      its_running_(false),
      its_debug_mode_(debug_mode) {
  ResetMessage();
  SetParameters(n_bins, range, vos, angle_step_size, left_limit, right_limit,
                ad_span, ad_low, debug_mode);
}

//------------------------------------------------------------------------------
//
SonarDriver::~SonarDriver() { Disconnect(); }

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void SonarDriver::Disconnect() {
  ROS_INFO("Disconnecting");
  its_running_ = false;
  if (its_serial_thread_.joinable()) {
    ROS_INFO("Joining...");
    try {
      its_serial_thread_.detach();
    } catch (...) {
      ROS_ERROR("Caught Exception");
    }
    ROS_INFO("Joined");
  }
  ROS_INFO("Disconnected");
}

//------------------------------------------------------------------------------
//
void SonarDriver::SetParameters(uint16_t n_bins, float range, float vos,
                                uint8_t angle_step_size, uint16_t left_limit,
                                uint16_t right_limit, uint8_t ad_span,
                                uint8_t ad_low, bool debug_mode) {
  n_bins_ = n_bins;
  range_ = range;
  vos_ = vos;
  angle_step_size_ = angle_step_size;
  left_limit_ = left_limit;
  right_limit_ = right_limit;
  ad_span_ = ad_span;
  ad_low_ = ad_low;
  its_debug_mode_ = debug_mode;
}

//------------------------------------------------------------------------------
//
bool SonarDriver::Connect(std::string const &devName) {
  ROS_INFO("Connecting...");
  serial_.flush();

  bool connection_success = serial_.connect(devName, 115200);
  if (!connection_success) {
    ROS_ERROR("Could not connect to serial port!");
    return false;
  }
  ROS_INFO("Connected");

  sleep(1);
  serial_.writeVector(mtRebootMsg);

  its_running_ = true;
  its_serial_thread_ =
      boost::thread(std::bind(&SonarDriver::SerialThreadMethod, this));

  its_processing_thread_ =
      boost::thread(std::bind(&SonarDriver::ProcessingThreadMethod, this));

  return true;
}

//------------------------------------------------------------------------------
//
void SonarDriver::Configure() {
  if (has_params_) {
    mtHeadCommandShortMsg headCommandShortMsg(ad_span_, ad_low_);
    serial_.writeVector(headCommandShortMsg.Construct());
  } else {
    mtHeadCommandMsg headCommandMsg(n_bins_, range_, vos_, left_limit_,
                                    right_limit_, angle_step_size_, ad_span_,
                                    ad_low_);
    serial_.writeVector(headCommandMsg.Construct());
  }
}

//------------------------------------------------------------------------------
//
void SonarDriver::Reconfigure(uint16_t n_bins, float range, float vos,
                              uint8_t step_angle_size, uint16_t left_limit,
                              uint16_t right_limit, uint8_t ad_span,
                              uint8_t ad_low, bool debug_mode) {
  // Load the new values
  SetParameters(n_bins, range, vos, step_angle_size, left_limit, right_limit,
                ad_span, ad_low, debug_mode);
  // Set the semaphore to red in order not to access the state variable at the
  // same time
  state_machine_semaphore_ = red;
  // Set the state variable to configure
  state_ = configuring;

  // Set the semaphore to green to allow the driver to continue
  state_machine_semaphore_ = green;
}

//------------------------------------------------------------------------------
//
void SonarDriver::ScanLineCallback(
    std::function<void(float, float, std::vector<uint8_t>)> callback) {
  its_scanline_callback = callback;
}

//------------------------------------------------------------------------------
//
void SonarDriver::SerialThreadMethod() {
  while (its_running_) {
    std::vector<uint8_t> bytes = serial_.read(2048);
    if (bytes.size() > 0) {
      for (size_t i = 0; i < bytes.size(); ++i) ProcessByte(bytes[i]);
      // std::cout << " Finished processing read Vector";
    } else {
      usleep(100000);
      // std::cout << std::endl << "No Serial input" << std::endl;
    }
  }
  ROS_INFO("serialThreadMethod Finished");
}

//------------------------------------------------------------------------------
//
void SonarDriver::ProcessingThreadMethod() {
  int waitedPeriods = 0;
  Semaphore firstSemaphore = green;

  while (its_running_) {
    if (state_machine_semaphore_ == green) {
      switch (state_) {
        case waitingforMtAlive_1:  // Waiting for MtAlive
          while (!has_heard_mtAlive_) {
            sleep(1);
            ROS_INFO("Waiting");
          }
          if (its_debug_mode_)
            ROS_INFO("----------Received mtAlive----------Case 1------");
          state_ = versionData;
          break;
        case versionData:  // Waiting for MtVersion Data
          while (!has_heard_mtVersionData_) {
            serial_.writeVector(mtSendVersionMsg);
            sleep(1);
          }
          if (its_debug_mode_)
            ROS_INFO("----------Received mtVersionData----Case 2------");
          state_ = waitingforMtAlive_2;
          break;
        case waitingforMtAlive_2:  // Waiting for MtAlive
          has_heard_mtAlive_ = false;
          while (!has_heard_mtAlive_) sleep(1);
          if (its_debug_mode_)
            ROS_INFO("----------Received mtAlive----------Case 3------");
          if (has_params_) {
            state_ = scanning;
          } else {
            state_ = configuring;
          }
          break;
        case configuring:  // Configure the Sonar
          if (its_debug_mode_) {
            ROS_INFO("----------Configuring Sonar---------Case 4------");
          }
          Configure();
          if (its_debug_mode_) {
            ROS_INFO("Changing to State 3");
          }
          sleep(5);
          state_ = waitingforMtAlive_2;
          break;
        case scanning:  // Send mtSend Data

          if (firstSemaphore == green) {
            firstSemaphore = red;
            if (its_debug_mode_) {
              ROS_INFO("----------Scanning----------------Case 5------");
            }
            sleep(1);
            serial_.writeVector(mtSendDataMsg);
          }

          if (waitedPeriods > 15) {
            if (its_debug_mode_) ROS_INFO("Scanning Resending request");
            serial_.writeVector(mtSendDataMsg);
            // usleep(100000);
            sleep(1);
          } else {
            waitedPeriods++;
            usleep(100000);
          }

          if (has_heard_mtHeadData_) {
            waitedPeriods = 0;
            has_heard_mtHeadData_ = false;
          }
          break;
      }
    } else {
      usleep(10);
    }
  }

  ROS_INFO("processingThreadMethod Finished");
}

//------------------------------------------------------------------------------
//
void SonarDriver::ResetMessage() {
  its_msg_ = SonarMessage();
  its_state_ = WaitingForAt;
  its_raw_msg_.clear();
}

//------------------------------------------------------------------------------
//
void SonarDriver::ProcessByte(uint8_t byte) {
  its_raw_msg_.push_back(byte);

  if (its_state_ == WaitingForAt) {
    if (byte == '@') {
      its_raw_msg_.clear();
      its_raw_msg_.push_back(byte);
      its_state_ = ReadingHeader;
      its_msg_ = SonarMessage();
      return;
    } else if (its_debug_mode_)
      ROS_INFO_STREAM(" bogus byte: " << std::hex << static_cast<int>(byte)
                                      << std::dec);
  }

  if (its_state_ == ReadingHeader) {
    // Ignore the 'Hex Length' section
    if (its_raw_msg_.size() < 6) return;

    if (its_raw_msg_.size() == 6) {
      its_msg_.binary_length = static_cast<uint16_t>(byte);
      return;
    }
    if (its_raw_msg_.size() == 7) {
      its_msg_.binary_length |= static_cast<uint16_t>(byte) << 8;
      if (its_msg_.binary_length > 1000) {
        if (its_debug_mode_) ROS_ERROR(" Message length too big!");
        ResetMessage();
      }
      return;
    }
    if (its_raw_msg_.size() == 8) {
      its_msg_.tx_node = byte;
      return;
    }
    if (its_raw_msg_.size() == 9) {
      its_msg_.rx_node = byte;
      return;
    }
    if (its_raw_msg_.size() == 10) {
      its_msg_.n_byte = byte;
      return;
    }
    if (its_raw_msg_.size() == 11) {
      its_msg_.id = MessageID(byte);
      its_state_ = ReadingData;
      return;
    }

    ROS_ERROR("Parsing error!");
    ResetMessage();
    return;
  }

  if (its_state_ == ReadingData) {
    // if(itsDebugMode) std::cout <<"  Remaining bytes: "<< std::dec  <<
    // uint16_t(itsMsg.binLength - (itsRawMsg.size() - 7)) << std::dec;
    if (uint16_t(its_msg_.binary_length - (its_raw_msg_.size() - 6)) == 0) {
      if (byte == 0x0A) {
        its_msg_.data = its_raw_msg_;
        ProcessMessage(its_msg_);
        ResetMessage();
      } else {
        if (its_debug_mode_) ROS_ERROR("Message finished, but no LF detected!");
        ResetMessage();
        return;
      }
    }
  }
}

//------------------------------------------------------------------------------
//
void SonarDriver::ProcessMessage(SonarMessage msg) {
  if (msg.id == mtVersionData) {
    mtVersionDataMsg parsedMsg(msg);
    has_heard_mtVersionData_ = true;

    if (its_debug_mode_) ROS_INFO("Received mtVersionData Message");
    if (its_debug_mode_) parsedMsg.Print();
  } else if (msg.id == mtAlive) {
    mtAliveMsg parsedMsg(msg);
    has_heard_mtAlive_ = true;
    has_params_ = !parsedMsg.no_params;

    if (its_debug_mode_) ROS_INFO("Received mtAlive Message");
    if (its_debug_mode_) parsedMsg.Print();
  } else if (msg.id == mtHeadData) {
    mtHeadDataMsg parsedMsg(msg);
    has_heard_mtHeadData_ = true;

    if (state_ == scanning) {
      serial_.writeVector(mtSendDataMsg);
    }

    float range_meters;
    switch (parsedMsg.range_units) {
      case mtHeadDataMsg::meters:
        range_meters = parsedMsg.range_scale;
        break;
      case mtHeadDataMsg::feet:
        range_meters = parsedMsg.range_scale * 0.3048;
        break;
      case mtHeadDataMsg::fathoms:
        range_meters = parsedMsg.range_scale * 1.8288;
        break;
      case mtHeadDataMsg::yards:
        range_meters = parsedMsg.range_scale * 0.9144;
        break;
    }

    float metersPerBin = range_meters / parsedMsg.scanline.size();
    if (its_scanline_callback) {
      its_scanline_callback(parsedMsg.transducer_bearing, metersPerBin,
                            parsedMsg.scanline);
    }

    if (its_debug_mode_) ROS_INFO("Received mtHeadData Message");
    if (its_debug_mode_) parsedMsg.Print();
  } else if (msg.id == mtBBUserData) {
    if (its_debug_mode_) ROS_INFO("Received mtBBUserData Message");
  } else {
    ROS_ERROR_STREAM("Unhandled Message Type: " << msg.id);
  }
}
}
