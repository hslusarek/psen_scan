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
#ifndef SCANNER_CONTROLLER_H
#define SCANNER_CONTROLLER_H

#include <functional>
#include <chrono>
#include <thread>
#include <atomic>
#include <array>

#include <boost/asio.hpp>

#include <psen_scan/scanner_communication_interface.h>
#include <psen_scan/msg_decoder.h>
#include <psen_scan/start_reply_msg.h>
#include <psen_scan/psen_scan_udp_interface.h>

namespace psen_scan
{

class ScannerController
{
public:
  ScannerController();

  void start();
  void stop();

private:
  //! @brief Starts a thread which continously reads data from the communication interface / scanner device.
  void listenForNewMessages();
  void reactToStartReply(const StartReplyMsg& start_reply);

private:
  std::unique_ptr<ScannerCommunicationInterface> communication_interface_;
  MsgDecoder msg_decoder_;
};

using std::placeholders::_1;

inline ScannerController::ScannerController()
  // Inform ScannerController when StartReply is received from ScannerDevice.
  : msg_decoder_( std::bind(&ScannerController::reactToStartReply, this, _1) )

{
  listenForNewMessages();
}

inline void ScannerController::start()
{
  // TODO: Send Start Request
  // TODO: Switch to state: Init
}

inline void ScannerController::stop()
{
  // TODO: Send Stop Request
}

inline void ScannerController::reactToStartReply(const StartReplyMsg& start_reply)
{
  // switch state to "Wait for monitoring frame"
}

inline void ScannerController::listenForNewMessages()
{
  constexpr std::chrono::milliseconds SLEEP_TIME{ 1000 };
  while(true)
  {
    // TODO: Read from communication interface

    // msg_decoder_.decodeAndDispatch(buffer);

    std::this_thread::sleep_for(SLEEP_TIME);
  }
}

}

#endif // SCANNER_CONTROLLER_H
