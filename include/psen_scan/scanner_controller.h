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
#include <thread>
#include <atomic>
#include <array>
#include <string>
#include <iostream>

#include <boost/asio.hpp>

#include <psen_scan/msg_decoder.h>
#include <psen_scan/start_reply_msg.h>
#include <psen_scan/controller_state_machine.h>
#include <psen_scan/async_udp_reader.h>
#include <psen_scan/sync_udp_writer.h>

namespace psen_scan
{
// TODO: Move to ScannerController class and read from ScannerConfiguration
static constexpr std::size_t DATA_SIZE_BYTES{ 65507 };

// TODO: Move to ScannerController class and read from ScannerConfiguration
static constexpr unsigned short READ_PORT_OF_SCANNER_CONTROLLER{ 45001 };

// TODO: Move to ScannerController class and read from ScannerConfiguration
static const std::string SCANNER_IP_ADDRESS{ "127.0.0.1" };

// TODO: Move to ScannerController class and read from ScannerConfiguration
static constexpr unsigned short SEND_PORT_OF_SCANNER_DEVICE{ 2000 };

static const boost::posix_time::millisec RECEIVE_TIMEOUT{ 1000 };

class ScannerController
{
public:
  ScannerController();

  void start();
  void stop();

private:
  void reactToStartReply(const StartReplyMsg& start_reply);
  void handleError(const std::string& error_msg);

private:
  MsgDecoder msg_decoder_{ std::bind(&ScannerController::reactToStartReply, this, std::placeholders::_1) };

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
};

inline ScannerController::ScannerController()
{
}

inline void ScannerController::handleError(const std::string& error_msg)
{
  // TODO: Add implementation -> Tell state machine about error
}

inline void ScannerController::start()
{
  async_udp_reader_.startReceiving(RECEIVE_TIMEOUT);

  // TODO: Send Start Request
  // TODO: Switch to state: Init
}

inline void ScannerController::stop()
{
  // TODO: Send Stop Request

  try
  {
    async_udp_reader_.close();
  }
  catch (const boost::system::system_error& ex)
  {
    // TODO: What do we want to do with these kind of exceptions?
    std::cerr << "ERROR: " << ex.what() << std::endl;
  }
}

inline void ScannerController::reactToStartReply(const StartReplyMsg& start_reply)
{
  // switch state to "Wait for monitoring frame"
}

}  // namespace psen_scan

#endif  // PSEN_SCAN_SCANNER_CONTROLLER_H
