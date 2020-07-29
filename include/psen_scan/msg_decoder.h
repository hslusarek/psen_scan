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

#include <boost/asio.hpp>

#include <psen_scan/start_reply_msg.h>

namespace psen_scan
{

using StartReplyCallback = std::function<void(const StartReplyMsg&)>;

class MsgDecoder
{
public:
  MsgDecoder(StartReplyCallback start_reply_callback);
  void decodeAndDispatch(boost::asio::mutable_buffers_1& buffer);

private:
  StartReplyCallback start_reply_callback_;
};

inline MsgDecoder::MsgDecoder(StartReplyCallback start_reply_callback)
  : start_reply_callback_(start_reply_callback)
{

}

inline void MsgDecoder::decodeAndDispatch(boost::asio::mutable_buffers_1& buffer)
{
  // Decode and call callbacks

  // if StartReply message then:
  StartReplyMsg start_reply_msg;
  start_reply_callback_(start_reply_msg);
}

}  // namespace psen_scan

#endif // PSEN_SCAN_MSG_DECODER_H
