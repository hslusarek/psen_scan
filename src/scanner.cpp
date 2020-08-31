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

#include "psen_scan/scanner.h"

#include "psen_scan/scanner_controller.h"

namespace psen_scan
{
Scanner::Scanner(const ScannerConfiguration& scanner_configuration)
  : state_machine_(std::make_shared<ControllerStateMachineImpl>(
        std::bind(&ScannerController::sendStartRequest, &scanner_controller_)))
  , control_msg_decoder_(std::bind(&ControllerStateMachine::processStartReplyReceivedEvent, state_machine_),
                         std::bind(&ScannerController::handleError, &scanner_controller_, std::placeholders::_1))
  , control_udp_client_(std::make_shared<UdpClientImpl>(
        std::bind(&MsgDecoder::decodeAndDispatch, &control_msg_decoder_, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ScannerController::handleError, &scanner_controller_, std::placeholders::_1),
        scanner_configuration.hostUDPPortControl(),
        scanner_configuration.clientIp(),
        CONTROL_PORT_OF_SCANNER_DEVICE))
  , data_msg_decoder_(std::bind(&ControllerStateMachine::processStartReplyReceivedEvent, state_machine_),
                      std::bind(&ScannerController::handleError, &scanner_controller_, std::placeholders::_1))
  , data_udp_client_(std::make_shared<UdpClientImpl>(
        std::bind(&MsgDecoder::decodeAndDispatch, &data_msg_decoder_, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ScannerController::handleError, &scanner_controller_, std::placeholders::_1),
        scanner_configuration.hostUDPPortData(),
        scanner_configuration.clientIp(),
        DATA_PORT_OF_SCANNER_DEVICE))
  , scanner_controller_(scanner_configuration, state_machine_, control_udp_client_, data_udp_client_)
{
}

void Scanner::start()
{
  scanner_controller_.start();
}

void Scanner::stop()
{
  scanner_controller_.stop();
}

LaserScan Scanner::getCompleteScan()
{
  // TODO: Add implementation in following stories
  throw LaserScanBuildFailure();
}

}  // namespace psen_scan
