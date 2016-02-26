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

#ifndef PROVIDER_SONAR_SONAR_DRIVER_H
#define PROVIDER_SONAR_SONAR_DRIVER_H

#include "sonar_messages/messages_id.h"
#include "sonar_messages/sonar_messages.h"
#include <vector>
#include "Serial.h"
#include <boost/thread.hpp>
#include <bits/shared_ptr.h>
#include "stdint.h"

namespace provider_sonar {

class SonarDriver {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<SonarDriver>;
  using ConstPtr = std::shared_ptr<const SonarDriver>;
  using PtrList = std::vector<SonarDriver::Ptr>;
  using ConstPtrList = std::vector<SonarDriver::ConstPtr>;

  // Current state of the incoming protocol FSM
  enum StateType {
    WaitingForAt = 0,  // Waiting for an @ to appear
    ReadingHeader =
        1,  // The @ sign has been found, now we're reading the header data
    ReadingData =
        2  // The header has been read, now we're just reading the data
  };

  // Semaphore type
  enum Semaphore { red, green };

  // States for the protocol state machine
  enum StateMachineStates {
    waitingforMtAlive_1,
    waitingforMtAlive_2,
    versionData,
    configuring,
    scanning,
    reset
  };

  //==========================================================================
  // P U B L I C   C / D T O R S

  SonarDriver(uint16_t n_bins, float range, float vos, uint8_t angle_step_size,
              uint16_t left_limit, uint16_t rigth_limit,
              bool debug_mode = true);

  ~SonarDriver();

  //! Connect to a Sonar device, and configure it
  /*! @param nBins The desired number of bins in each scanline
      @param range The desired range (in meters)
      @param VOS The velocity of sound in the current medium
      @param stepAngleSize The angular resolution of each scan */
  bool Connect(std::string const &devName);

  //! Disconnect from the Sonar device and kill all of our associated threads
  void Disconnect();

  //! Configure the sonar
  void Configure();

  void ResetMessage();

  // This function allows the user to reconfigure the sonar once it is running
  // without restarting the driver
  void Reconfigure(uint16_t n_bins, float range, float vos,
                   uint8_t step_angle_size, uint16_t left_limit,
                   uint16_t right_limit, bool debug_mode);

  //! Get access to the scan lines that have been read by the sonar.
  /*! Note that this method is thread safe and will lock the internal scan
      line data structure, therefore one should call this method as
      infrequently as possible */
  std::map<float, std::vector<uint8_t>> scanLines();

  void RegisterScanLineCallback(
      std::function<void(float /*angle*/, float /*meters per bin*/,
                         std::vector<uint8_t> /*scanline*/)>);

 private:
  // This function sets the necessary parameters for the Sonar operation.
  void SetParameters(uint16_t n_bins, float range, float vos,
                     uint8_t angle_step_size, uint16_t left_limit,
                     uint16_t right_limit, bool debug_mode);

  // Process a single incoming byte (add it onto itsRawMsg, etc.)
  void ProcessByte(uint8_t byte);

  // Process an incoming message
  void ProcessMessage(SonarMessage msg);

  // The method being run by its_serial_thread_
  void SerialThreadMethod();

  // The method being run by its_processing_thread
  void ProcessingThreadMethod();

  //============================================================================
  // P R I V A T E   M E M B E R S

  // A thread that just spins and reads data from the serial port
  boost::thread its_serial_thread_;
  // A thread that just spins and interacts with the Sonar
  boost::thread its_processing_thread_;

  // The current state of the FSM controlling the parsing of the incoming
  // protocol
  StateType its_state_;
  Semaphore state_machine_semaphore_;  // State Machine Semaphore
  Semaphore
      scanning_callback_semaphore_;  // Scanning callback routine semaphore
  StateMachineStates state_;  // The variable controlling the state machine
  // The current message buffer begin read in
  // the current incoming message begin constructed from itsRawMsg
  std::vector<uint8_t> its_raw_msg_;
  SonarMessage its_msg_;
  bool has_heard_mtAlive_;  // Have we ever heard an mtAlive message from the
                            // sonar?
  bool has_heard_mtVersionData_;  // Have we ever heard an mtVersionData from
                                  // the sonar?
  bool has_heard_mtHeadData_;  // Have we received a mtHeadData from the sonar?
  bool has_params_;            // Has the Sonar received it's parameters?
  SerialPort serial_;          // The actual serial port
  bool its_running_;           // Are we currently running?
  // Should we be printing debugging information to the console? Warning, this
  // is very noisy and slow.
  bool its_debug_mode_;
  uint16_t n_bins_;
  float range_;
  float vos_;
  uint8_t angle_step_size_;
  uint16_t left_limit_;
  uint16_t right_limit_;

  std::function<void(float /*angle*/, float /*meters per bin*/,
                     std::vector<uint8_t> /*scanline*/)> itsScanLineCallback;
};
}

#endif  // PROVIDER_SONAR_SONAR_DRIVER_H
