// Copyright (c) 2019-2020 Pilz GmbH & Co. KG
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

#include <functional>
#include <chrono>
#include <array>
#include <string>
#include <iostream>

#include <psen_scan/msg_decoder.h>
#include <psen_scan/controller_state_machine.h>
#include <psen_scan/async_udp_reader.h>
#include <psen_scan/sync_udp_writer.h>
#include <psen_scan/controller_state_machine.h>
#include <psen_scan/start_request.h>

namespace psen_scan
{
// TODO: Move to ScannerController class and read from ScannerConfiguration
static constexpr std::size_t DATA_SIZE_BYTES{ 65507 };

// TODO: Move to ScannerController class and read from ScannerConfiguration
static constexpr unsigned short READ_PORT_OF_SCANNER_CONTROLLER{ 45001 };
static constexpr unsigned short WRITE_PORT_OF_SCANNER_CONTROLLER{ READ_PORT_OF_SCANNER_CONTROLLER + 1 };

// TODO: Move to ScannerController class and read from ScannerConfiguration
static const std::string SCANNER_IP_ADDRESS{ "127.0.0.1" };

// TODO: Move to ScannerController class and read from ScannerConfiguration
static constexpr unsigned short SEND_PORT_OF_SCANNER_DEVICE{ 2000 };
static constexpr unsigned short RECEIVE_PORT_OF_SCANNER_DEVICE{ SEND_PORT_OF_SCANNER_DEVICE + 1 };

static const boost::posix_time::millisec RECEIVE_TIMEOUT{ 1000 };

class ScannerController
{
public:
  void start();
  void stop();

private:
  void handleError(const std::string& error_msg);
  void sendStartRequest();

private:
  ControllerStateMachine state_machine_{ std::bind(&ScannerController::sendStartRequest, this) };

  MsgDecoder msg_decoder_{ std::bind(&ControllerStateMachine::processStartReplyReceivedEvent, &state_machine_) };

  psen_scan::AsyncUdpReader<DATA_SIZE_BYTES> async_udp_reader_{
    std::bind(&MsgDecoder::decodeAndDispatch<DATA_SIZE_BYTES>,
              &msg_decoder_,
              std::placeholders::_1,
              std::placeholders::_2),
    std::bind(&ScannerController::handleError, this, std::placeholders::_1),
    READ_PORT_OF_SCANNER_CONTROLLER,
    SCANNER_IP_ADDRESS,
    SEND_PORT_OF_SCANNER_DEVICE
  };

  psen_scan::SyncUdpWriter sync_udp_writer_{ WRITE_PORT_OF_SCANNER_CONTROLLER,
                                             SCANNER_IP_ADDRESS,
                                             RECEIVE_PORT_OF_SCANNER_DEVICE };
};

inline void ScannerController::handleError(const std::string& error_msg)
{
  std::cerr << error_msg << std::endl;
  // TODO: Add implementation -> Tell state machine about error
}

inline void ScannerController::start()
{
  async_udp_reader_.startReceiving(RECEIVE_TIMEOUT);
  state_machine_.processStartRequestEvent();
}

inline void ScannerController::stop()
{
  // TODO: Impl. sending of StopRequest
  state_machine_.processStopRequestEvent();
}

inline void ScannerController::sendStartRequest()
{
  constexpr auto number_of_bytes_to_send{ sizeof(StartRequest) };
  // TODO: std::array<char, number_of_bytes_to_send> start_request_as_byte_stream{ start_request.toArray() };
  std::array<char, number_of_bytes_to_send> start_request_as_byte_stream;
  sync_udp_writer_.write<number_of_bytes_to_send>(start_request_as_byte_stream);
}

}  // namespace psen_scan

#endif  // PSEN_SCAN_SCANNER_CONTROLLER_H
