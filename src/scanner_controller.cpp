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

#include <psen_scan/scanner_controller.h>

using namespace psen_scan;

ScannerControllerImpl::ScannerControllerImpl(const ScannerConfiguration& scanner_config,
                                             std::shared_ptr<ControllerStateMachine> state_machine,
                                             std::shared_ptr<UdpClient> control_udp_client,
                                             std::shared_ptr<UdpClient> data_udp_client)
  : scanner_config_(scanner_config)
  , state_machine_(std::move(state_machine))
  , control_udp_client_(std::move(control_udp_client))
  , data_udp_client_(std::move(data_udp_client))
{
}

void ScannerControllerImpl::handleError(const std::string& error_msg)
{
  std::cerr << error_msg << std::endl;
  // TODO: Add implementation -> Tell state machine about error
}

void ScannerControllerImpl::start()
{
  state_machine_->processStartRequestEvent();
}

void ScannerControllerImpl::stop()
{
  // TODO: Impl. sending of StopRequest
  state_machine_->processStopRequestEvent();
}

void ScannerControllerImpl::sendStartRequest()
{
  control_udp_client_->startReceiving(RECEIVE_TIMEOUT);
  data_udp_client_->startReceiving(RECEIVE_TIMEOUT);
  StartRequest start_request(scanner_config_, DEFAULT_SEQ_NUMBER);
  const auto start_request_as_byte_stream{ start_request.toCharArray() };
  std::shared_ptr<char> byte_stream_ptr{ std::make_shared<char>(start_request_as_byte_stream.at(0)) };
  control_udp_client_->write(byte_stream_ptr, start_request_as_byte_stream.size());
}
