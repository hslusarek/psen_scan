// Copyright (c) 2020 Pilz GmbH & Co. KG
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef PSEN_SCAN_CONTROLLER_STATE_MACHINE_H
#define PSEN_SCAN_CONTROLLER_STATE_MACHINE_H

#include <mutex>
#include <psen_scan/controller_msm_state_machine.h>

namespace psen_scan
{

class ControllerStateMachine
{
public:
  explicit ControllerStateMachine();
  virtual ~ControllerStateMachine();


  inline void registerSendStartRequestCallback(SendStartRequestCallback sr) { sm_.InitState_.send_start_request_callback_= sr; };

  bool processStartRequestEvent();
  bool processStartReplyReceivedEvent();
  bool processMonitoringFrameReceivedEvent();
  bool processStopRequestEvent();
  bool processStopReplyReceivedEvent();


private:
  controller_msm_state_machine sm_;
  std::mutex sm_access_mutex_;

//   TODO: Do we need this here?
//
//   /* UDP Stuff*/
//   boost::asio::io_service io_service_;
//   udp::endpoint udp_read_endpoint_{udp::v4(), 5004};  /**< Endpoint for reading from UDP. */
//   udp::socket socket_read_{io_service_, udp_read_endpoint_};               /**< Socket used for reading from UDP. */

//   std::unique_ptr<std::thread> reading_thread_;
//   std::atomic_bool reading_thread_abort_flag_{false};

//   bool readUdp();
};

}  // namespace psen_scan


#endif  // PSEN_SCAN_CONTROLLER_STATE_MACHINE_H
