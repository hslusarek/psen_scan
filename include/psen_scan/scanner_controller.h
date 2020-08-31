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
#ifndef PSEN_SCAN_SCANNER_CONTROLLER_H
#define PSEN_SCAN_SCANNER_CONTROLLER_H

#include <array>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include <psen_scan/msg_decoder.h>
#include <psen_scan/controller_state_machine.h>
#include <psen_scan/udp_client.h>
#include <psen_scan/controller_state_machine.h>
#include <psen_scan/start_request.h>
#include <psen_scan/scanner_configuration.h>

namespace psen_scan
{
// TODO: Move to ScannerController class and read from ScannerConfiguration
static constexpr unsigned short DATA_PORT_OF_SCANNER_DEVICE{ 2000 };
static constexpr unsigned short CONTROL_PORT_OF_SCANNER_DEVICE{ 3000 };

static constexpr std::chrono::milliseconds RECEIVE_TIMEOUT{ 1000 };

static constexpr uint32_t DEFAULT_SEQ_NUMBER{ 0 };

class ScannerController
{
public:
  ScannerController(const ScannerConfiguration& scanner_config,
                    std::shared_ptr<ControllerStateMachine> state_machine,
                    std::shared_ptr<UdpClient> control_udp_client,
                    std::shared_ptr<UdpClient> data_udp_client);

  void start();
  void stop();

  void handleError(const std::string& error_msg);
  void sendStartRequest();

private:
  ScannerConfiguration scanner_config_;

  std::shared_ptr<ControllerStateMachine> state_machine_;

  std::shared_ptr<UdpClient> control_udp_client_;
  std::shared_ptr<UdpClient> data_udp_client_;
};

inline ScannerController::ScannerController(const ScannerConfiguration& scanner_config,
                                            std::shared_ptr<ControllerStateMachine> state_machine,
                                            std::shared_ptr<UdpClient> control_udp_client,
                                            std::shared_ptr<UdpClient> data_udp_client)
  : scanner_config_(scanner_config)
  , state_machine_(state_machine)
  , control_udp_client_(control_udp_client)
  , data_udp_client_(data_udp_client)
{
}

inline void ScannerController::handleError(const std::string& error_msg)
{
  std::cerr << error_msg << std::endl;
  // TODO: Add implementation -> Tell state machine about error
}

inline void ScannerController::start()
{
  state_machine_->processStartRequestEvent();
}

inline void ScannerController::stop()
{
  // TODO: Impl. sending of StopRequest
  state_machine_->processStopRequestEvent();
}

inline void ScannerController::sendStartRequest()
{
  control_udp_client_->startReceiving(RECEIVE_TIMEOUT);
  data_udp_client_->startReceiving(RECEIVE_TIMEOUT);
  StartRequest start_request(scanner_config_, DEFAULT_SEQ_NUMBER);
  const auto start_request_as_byte_stream{ start_request.toCharArray() };
  std::shared_ptr<char> byte_stream_ptr{ std::make_shared<char>(start_request_as_byte_stream.at(0)) };
  control_udp_client_->write(byte_stream_ptr, start_request_as_byte_stream.size());
}

}  // namespace psen_scan

#endif  // PSEN_SCAN_SCANNER_CONTROLLER_H
