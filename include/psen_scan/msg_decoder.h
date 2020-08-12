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
#ifndef PSEN_SCAN_MSG_DECODER_H
#define PSEN_SCAN_MSG_DECODER_H

#include <functional>
#include <array>
#include <sstream>
#include <string>

#include "psen_scan/scanner_frames.h"
#include "psen_scan/scanner_frames_conversions.h"
#include "psen_scan/decode_exception.h"

namespace psen_scan
{
using StartReplyCallback = std::function<void()>;
using ErrorCallback = std::function<void(const std::string&)>;

class MsgDecoder
{
public:
  MsgDecoder(const StartReplyCallback& start_reply_callback, const ErrorCallback& error_callback);

  template <std::size_t NumberOfBytes>
  void decodeAndDispatch(const std::array<char, NumberOfBytes>& data, const std::size_t& bytes_received);

private:
  StartReplyCallback start_reply_callback_;
  ErrorCallback error_callback_;
};

inline MsgDecoder::MsgDecoder(const StartReplyCallback& start_reply_callback, const ErrorCallback& error_callback)
  : start_reply_callback_(start_reply_callback), error_callback_(error_callback)
{
}

template <std::size_t NumberOfBytes>
void MsgDecoder::decodeAndDispatch(const std::array<char, NumberOfBytes>& data, const std::size_t& bytes_received)
{
  if (bytes_received == sizeof(DataReply::MemoryFormat))  // Check if this could be a reply
  {
    DataReply::MemoryFormat frame{ decode(data) };  // TODO how to handle throw?

    if (frame.type() == DataReply::Type::Start)
    {
      start_reply_callback_();
    }
    else
    {
      // TODO: Replace with stop reply callback in future.
      error_callback_("Unknown message type (Size " + std::to_string(bytes_received) + ")");
    }
  }
  else
  {
    // TODO: Replace with monitoring frame callback in future.
    error_callback_("Unknown message type (Size " + std::to_string(bytes_received) + ")");
  }
}

}  // namespace psen_scan

#endif  // PSEN_SCAN_MSG_DECODER_H
