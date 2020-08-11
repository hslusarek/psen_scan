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
ControllerStateMachine::ControllerStateMachine(const SendStartRequestCallback& sr):
  sm_(sr)
{
  const std::lock_guard<std::mutex> lock (sm_access_mutex_);
  sm_.start();
}

ControllerStateMachine::~ControllerStateMachine()
{
  const std::lock_guard<std::mutex> lock (sm_access_mutex_);
  sm_.send_start_request_callback_ = nullptr;
  sm_.stop();
}

bool ControllerStateMachine::processStartRequestEvent()
{
  const std::lock_guard<std::mutex> lock (sm_access_mutex_);
  sm_.process_event(start_request_event());

  return true;
}

bool ControllerStateMachine::processStartReplyReceivedEvent()
{
  const std::lock_guard<std::mutex> lock (sm_access_mutex_);
  sm_.process_event(start_reply_received_event());

  return true;
}

bool ControllerStateMachine::processMonitoringFrameReceivedEvent()
{
  const std::lock_guard<std::mutex> lock (sm_access_mutex_);
  sm_.process_event(monitoring_frame_received_event());

  return true;
}

bool ControllerStateMachine::processStopRequestEvent()
{
  const std::lock_guard<std::mutex> lock (sm_access_mutex_);
  sm_.process_event(stop_request_event());

  return true;
}

bool ControllerStateMachine::processStopReplyReceivedEvent()
{
  const std::lock_guard<std::mutex> lock (sm_access_mutex_);
  sm_.process_event(stop_reply_received_event());

  return true;
}

}  // namespace psen_scan
