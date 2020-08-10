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

#include <psen_scan/controller_state_machine.h>

namespace psen_scan
{
ControllerStateMachine::ControllerStateMachine()
{
  std::unique_lock<std::mutex>(sm_access_mutex_);
  sm_.start();
}

ControllerStateMachine::~ControllerStateMachine()
{
  std::unique_lock<std::mutex>(sm_access_mutex_);
  sm_.stop();
}

bool ControllerStateMachine::processStartRequestEvent()
{
  std::unique_lock<std::mutex>(sm_access_mutex_);
  sm_.process_event(start_request_event());

  return true;
}

bool ControllerStateMachine::processStartReplyReceivedEvent()
{
  std::unique_lock<std::mutex>(sm_access_mutex_);
  sm_.process_event(start_reply_received_event());

  return true;
}

bool ControllerStateMachine::processMonitoringFrameReceivedEvent()
{
  std::unique_lock<std::mutex>(sm_access_mutex_);
  sm_.process_event(monitoring_frame_received_event());

  return true;
}

bool ControllerStateMachine::processStopRequestEvent()
{
  std::unique_lock<std::mutex>(sm_access_mutex_);
  sm_.process_event(stop_request_event());

  return true;
}

bool ControllerStateMachine::processStopReplyReceivedEvent()
{
  std::unique_lock<std::mutex>(sm_access_mutex_);
  sm_.process_event(stop_reply_received_event());

  return true;
}

}  // namespace psen_scan
