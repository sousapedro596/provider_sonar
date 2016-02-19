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
#include "provider_sonar/message_types.h"
#include <iostream>
#include <algorithm>
#include <chrono>
#include "stdint.h"

using namespace tritech;

namespace provider_sonar {
//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
SonarDriver::SonarDriver(uint16_t _nBins, float _range, float _VOS, uint8_t _angleStepSize,
                         int _leftLimit, int _rightLimit, bool debugMode)
    : hasHeardMtAlive(false),
      hasHeardMtVersionData(false),
      hasHeardMtHeadData(false),
      its_debug_mode_(debugMode),
      state_machine_semaphore_(green),
      state_(waitingforMtAlive_1) {
  ResetMessage();
  SetParameters(_nBins, _range, _VOS, _angleStepSize, _leftLimit, _rightLimit);
}

//------------------------------------------------------------------------------
//
SonarDriver::~SonarDriver() { Disconnect(); }

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void SonarDriver::Disconnect() {
  std::cout << "Disconnecting" << std::endl;
  its_running_ = false;
  if (its_serial_thread_.joinable()) {
    std::cout << "Joining...";
    try {
      its_serial_thread_.detach();
    } catch (...) { /*screw you!*/
    }
    std::cout << "Joined";
  }

  std::cout << "Disconnected" << std::endl;
}

//------------------------------------------------------------------------------
//
void SonarDriver::SetParameters(uint16_t _nBins, float _range, float _VOS,
                                uint8_t _angleStepSize, int _leftLimit,
                                int _rightLimit) {
  n_bins_ = _nBins;
  range_ = _range;
  vos_ = _VOS;
  angle_step_size_ = _angleStepSize;
  left_limit_ = _leftLimit;
  right_limit_ = _rightLimit;
}

//------------------------------------------------------------------------------
//
bool SonarDriver::Connect(std::string const &devName) {
  if (its_debug_mode_) std::cout << "Connecting...";

  serial_.flush();

  bool connectionSuccess = serial_.connect(devName, 115200);
  if (!connectionSuccess) {
    std::cerr << "Could not connect to serial port!" << std::endl;
    return false;
  }

  if (its_debug_mode_) std::cout << "Connected" << std::endl;

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
  mtHeadCommandMsg headCommandMsg(n_bins_, range_, vos_, angle_step_size_, left_limit_,
                                  right_limit_);
  serial_.writeVector(headCommandMsg.construct());
}

//------------------------------------------------------------------------------
//
void SonarDriver::Reconfigure(uint16_t _nBins, float _range, float _VOS,
                              uint8_t _angleStepSize, int _leftLimit,
                              int _rightLimit) {
  // Load the new values
  SetParameters(_nBins, _range, _VOS, _angleStepSize, _leftLimit, _rightLimit);
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
void SonarDriver::RegisterScanLineCallback(
    std::function<void(float, float, std::vector<uint8_t>)> callback) {
  itsScanLineCallback = callback;
}

//------------------------------------------------------------------------------
//
void SonarDriver::SerialThreadMethod() {
  while (its_running_ == true) {
    std::vector<uint8_t> bytes = serial_.read(2048);
    if (bytes.size() > 0) {
      for (size_t i = 0; i < bytes.size(); ++i) ProcessByte(bytes[i]);
      // std::cout << " Finished processing read Vector";
    } else {
      usleep(100000);
      // std::cout << std::endl << "No Serial input" << std::endl;
    }
  }
  std::cout << "serialThreadMethod Finished" << std::endl;
}

//------------------------------------------------------------------------------
//
void SonarDriver::ProcessingThreadMethod() {
  int waitedPeriods = 0;
  Semaphore firstSemaphore = green;

  while (its_running_) {
    if (state_machine_semaphore_ == green) {
      // std::cout << "In loop: " <<std::endl;
      switch (state_) {
        case waitingforMtAlive_1:  // Waiting for MtAlive
          while (!hasHeardMtAlive) {
            sleep(1);
            std::cout << "Waiting: " << std::endl;
          }
          if (its_debug_mode_)
            std::cout << "----------Received mtAlive----Case 1------"
                << std::endl;
          state_ = versionData;
          break;
        case versionData:  // Waiting for MtVersion Data
          while (!hasHeardMtVersionData) {
            serial_.writeVector(mtSendVersionMsg);
            sleep(1);
          }
          if (its_debug_mode_)
            std::cout << "----------Received mtVersionData----Case 2------"
                << std::endl;
          state_ = waitingforMtAlive_2;
          break;
        case waitingforMtAlive_2:  // Waiting for MtAlive
          hasHeardMtAlive = false;
          while (!hasHeardMtAlive) sleep(1);
          if (its_debug_mode_)
            std::cout << "----------Received mtAlive----Case 3------"
                << std::endl;
          if (hasParams) {
            state_ = scanning;
          } else {
            state_ = configuring;
          }
          break;
        case configuring:  // Configure the Sonar
          if (its_debug_mode_)
            std::cout << "----------Configuring Sonar----Case 4------"
                << std::endl;
          Configure();
          if (its_debug_mode_) std::cout << "Changing to State 3" << std::endl;
          sleep(5);
          state_ = waitingforMtAlive_2;
          break;
        case scanning:  // Send mtSend Data

          if (firstSemaphore == green) {
            firstSemaphore = red;
            if (its_debug_mode_)
              std::cout << "----------Scanning: ...   ----Case 5------"
                  << std::endl;
            sleep(1);
            serial_.writeVector(mtSendDataMsg);
          }

          if (waitedPeriods > 15) {
            if (its_debug_mode_)
              std::cout << "----------Scanning: Resending request" << std::endl;
            serial_.writeVector(mtSendDataMsg);
            // usleep(100000);
            sleep(1);
          } else {
            waitedPeriods++;
            usleep(100000);
          }

          if (hasHeardMtHeadData) {
            waitedPeriods = 0;
            hasHeardMtHeadData = false;
          }
          break;

          // Transition to configuring in case the sonar reports lacking
          // parameters
          if (!hasParams) {
            state_ = configuring;
          }
      }
    } else {
      usleep(10);
    }
  }

  std::cout << "processingThreadMethod Finished" << std::endl;
}

//------------------------------------------------------------------------------
//
void SonarDriver::ResetMessage() {
  its_msg_ = Message();
  its_state_ = WaitingForAt;
  its_raw_msg_.clear();
}

//------------------------------------------------------------------------------
//
void SonarDriver::ProcessByte(uint8_t byte) {
  // if(itsDebugMode) std::cout << std::endl << "Received Byte: " << std::hex <<
  // int(byte) << std::dec ;
  its_raw_msg_.push_back(byte);

  if (its_state_ == WaitingForAt) {
    if (byte == '@') {
      its_raw_msg_.clear();
      // Tritech's datasheet refers to the first byte as '1' rather than '0', so
      // let's push back a bogus byte here just
      // to make reading the datasheet easier.
      its_raw_msg_.push_back(0);
      its_raw_msg_.push_back(byte);
      its_state_ = ReadingHeader;
      its_msg_ = Message();
      return;
    } else if (its_debug_mode_)
      std::cout << " bogus byte: " << std::hex << int(byte)
          << std::dec;  //<< std::endl;
  }
  // std::cout << std::endl;

  if (its_state_ == ReadingHeader) {
    // Ignore the 'Hex Length' section
    if (its_raw_msg_.size() < 7) return;

    if (its_raw_msg_.size() == 7) {
      its_msg_.binLength = uint16_t(byte);
      return;
    }
    if (its_raw_msg_.size() == 8) {
      its_msg_.binLength |= uint16_t(byte) << 8;
      if (its_msg_.binLength > 1000) {
        if (its_debug_mode_) std::cout << " Message length too big!" << std::endl;
        ResetMessage();
      }
      return;
    }
    if (its_raw_msg_.size() == 9) {
      its_msg_.txNode = byte;
      return;
    }
    if (its_raw_msg_.size() == 10) {
      its_msg_.rxNode = byte;
      return;
    }
    if (its_raw_msg_.size() == 11) {
      its_msg_.count = byte;
      return;
    }
    if (its_raw_msg_.size() == 12) {
      its_msg_.type = MessageType(byte);
      its_state_ = ReadingData;
      return;
    }

    std::cerr << "Parsing error! " << __FILE__ << ":" << __LINE__ << std::endl;
    ResetMessage();
    return;
  }

  if (its_state_ == ReadingData) {
    // if(itsDebugMode) std::cout <<"  Remaining bytes: "<< std::dec  <<
    // uint16_t(itsMsg.binLength - (itsRawMsg.size() - 7)) << std::dec;
    if (uint16_t(its_msg_.binLength - (its_raw_msg_.size() - 7)) == 0) {
      if (byte == 0x0A) {
        its_msg_.data = its_raw_msg_;
        ProcessMessage(its_msg_);
        ResetMessage();
      } else {
        if (its_debug_mode_) std::cout << " Message finished, but no LF detected!";
        ResetMessage();
        return;
      }
    }
  }
}

//------------------------------------------------------------------------------
//
void SonarDriver::ProcessMessage(tritech::Message msg) {
  if (msg.type == mtVersionData) {
    mtVersionDataMsg parsedMsg(msg);
    hasHeardMtVersionData = true;

    if (its_debug_mode_)
      std::cout << std::endl
          << "Received mtVersionData Message" << std::endl;
    if (its_debug_mode_) parsedMsg.print();
  } else if (msg.type == mtAlive) {
    mtAliveMsg parsedMsg(msg);
    hasHeardMtAlive = true;
    hasParams = !parsedMsg.noParams;

    if (its_debug_mode_)
      std::cout << std::endl
          << "Received mtAlive Message" << std::endl;
    if (its_debug_mode_) parsedMsg.print();
  } else if (msg.type == mtHeadData) {
    mtHeadDataMsg parsedMsg(msg);
    hasHeardMtHeadData = true;

    if (state_ == scanning) {
      serial_.writeVector(mtSendDataMsg);
    }

    float range_meters;
    switch (parsedMsg.rangeUnits) {
      case mtHeadDataMsg::meters:
        range_meters = parsedMsg.rangeScale;
        break;
      case mtHeadDataMsg::feet:
        range_meters = parsedMsg.rangeScale * 0.3048;
        break;
      case mtHeadDataMsg::fathoms:
        range_meters = parsedMsg.rangeScale * 1.8288;
        break;
      case mtHeadDataMsg::yards:
        range_meters = parsedMsg.rangeScale * 0.9144;
        break;
    }

    float metersPerBin = range_meters / parsedMsg.scanLine.size();
    if (itsScanLineCallback)
      itsScanLineCallback(parsedMsg.bearing_degrees, metersPerBin,
                          parsedMsg.scanLine);

    if (its_debug_mode_)
      std::cout << std::endl
          << "Received mtHeadData Message" << std::endl;
    if (its_debug_mode_) parsedMsg.print();
  } else if (msg.type == mtBBUserData) {
    if (its_debug_mode_)
      std::cout << std::endl
          << "Received mtBBUserData Message" << std::endl;
  } else {
    std::cerr << "Unhandled Message Type: " << msg.type << std::endl;
  }
}
}
