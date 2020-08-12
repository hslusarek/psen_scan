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
#ifndef PSEN_SCAN_SYNC_UDP_WRITER_H
#define PSEN_SCAN_SYNC_UDP_WRITER_H

#include <stdexcept>
#include <array>
#include <string>

#include <arpa/inet.h>

#include <boost/asio.hpp>

#include "psen_scan/open_connection_failure.h"
#include "psen_scan/close_connection_failure.h"

namespace psen_scan
{
class WriteFailure : public std::runtime_error
{
public:
  WriteFailure(const std::string& msg = "Error while writing to socket");
};

class SyncUdpWriter
{
public:
  SyncUdpWriter(const unsigned short& host_port, const in_addr_t& endpoint_ip, const unsigned short& endpoint_port);
  ~SyncUdpWriter();

public:
  //! @brief Sends the specified data via UDP to the client.
  //! @param NumberOfBytes Bytes send to the client.
  template <std::size_t NumberOfBytes>
  void write(const std::array<char, NumberOfBytes>& data);

  void close();

private:
  boost::asio::io_service io_service_;
  boost::asio::ip::udp::endpoint endpoint_;
  boost::asio::ip::udp::socket socket_;
};

inline WriteFailure::WriteFailure(const std::string& msg) : std::runtime_error(msg)
{
}

inline SyncUdpWriter::SyncUdpWriter(const unsigned short& host_port,
                                    const in_addr_t& endpoint_ip,
                                    const unsigned short& endpoint_port)
  : endpoint_(boost::asio::ip::address_v4(endpoint_ip), endpoint_port)
  , socket_(io_service_, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), host_port))
{
  try
  {
    socket_.connect(endpoint_);
  }
  // LCOV_EXCL_START
  catch (const boost::system::system_error& ex)
  {
    throw OpenConnectionFailure(ex.what());
  }
  // LCOV_EXCL_STOP
}

inline void SyncUdpWriter::close()
{
  try
  {
    socket_.close();
  }
  // LCOV_EXCL_START
  catch (const boost::system::system_error& ex)
  {
    throw CloseConnectionFailure(ex.what());
  }
  // LCOV_EXCL_STOP
}

inline SyncUdpWriter::~SyncUdpWriter()
{
  try
  {
    close();
  }
  // LCOV_EXCL_START
  catch (const CloseConnectionFailure& ex)
  {
    std::cout << "ERROR: " << ex.what() << std::endl;
  }
  // LCOV_EXCL_STOP
}

template <std::size_t NumberOfBytes>
inline void SyncUdpWriter::write(const std::array<char, NumberOfBytes>& data)
{
  try
  {
    socket_.send(boost::asio::buffer(data));
  }
  // LCOV_EXCL_START
  catch (const boost::system::system_error& ex)
  {
    throw WriteFailure(ex.what());
  }
  // LCOV_EXCL_STOP
}

}  // namespace psen_scan

#endif  // PSEN_SCAN_SYNC_UDP_WRITER_H
