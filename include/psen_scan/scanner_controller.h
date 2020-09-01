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

// LCOV_EXCL_START
class ScannerController
{
public:
  virtual ~ScannerController() = default;
  virtual void start() = 0;
  virtual void stop() = 0;
  virtual void handleError(const std::string& error_msg) = 0;
  virtual void sendStartRequest() = 0;
};
// LCOV_EXCL_STOP

class ScannerControllerImpl : public ScannerController
{
public:
  ScannerControllerImpl(const ScannerConfiguration& scanner_config,
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

}  // namespace psen_scan

#endif  // PSEN_SCAN_SCANNER_CONTROLLER_H
