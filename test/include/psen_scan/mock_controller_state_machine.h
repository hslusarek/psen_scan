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

#include <gmock/gmock.h>

#include <psen_scan/controller_state_machine.h>
#include <psen_scan/function_pointers.h>

namespace psen_scan_test
{
class MockControllerStateMachine : public psen_scan::ControllerStateMachine
{
public:
  explicit MockControllerStateMachine(const psen_scan::SendStartRequestCallback& sr){};

public:
  MOCK_METHOD0(processStartRequestEvent, void());
  MOCK_METHOD0(processStartReplyReceivedEvent, void());
  MOCK_METHOD0(processMonitoringFrameReceivedEvent, void());
  MOCK_METHOD0(processStopRequestEvent, void());
  MOCK_METHOD0(processStopReplyReceivedEvent, void());
};

}  // namespace psen_scan_test
