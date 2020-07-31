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
#ifndef PSEN_SCAN_MSG_ENCODER_H
#define PSEN_SCAN_MSG_ENCODER_H

#include <boost/asio.hpp>

#include <psen_scan/start_request.h>

namespace psen_scan
{

namespace msg_encoder
{

  inline boost::asio::mutable_buffers_1 toBuffer(StartRequest& start_request)
  {
    boost::asio::mutable_buffers_1 buffer(&start_request, sizeof(start_request));
    return buffer;
  }

}  // namespace msg_encoder

}  // namespace psen_scan

#endif // PSEN_SCAN_MSG_ENCODER_H
