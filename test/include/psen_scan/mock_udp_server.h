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

#ifndef PSEN_SCAN_TEST_MOCK_UDP_SERVER_H
#define PSEN_SCAN_TEST_MOCK_UDP_SERVER_H

#include <memory>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/array.hpp>
#include <boost/thread.hpp>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

using boost::asio::ip::udp;

namespace psen_scan_test
{
static constexpr unsigned short const SCAN_PORT_WRITE{ 3000 };
static constexpr unsigned short const SCAN_PORT_READ{ 2000 };

static const std::string MOCK_IP_ADDRESS{ "127.0.0.1" };

/**
 * @brief Class for the UDP communication with the scanner.
 *
 */
class MockUDPServer
{
public:
  ~MockUDPServer();

public:
  /**
   * @param mock_port_send Port over which the mock sends data to the client.
   * @param mock_port_receive Port over which the mock receives data from the client.
   */
  MockUDPServer(const unsigned short mock_port_send = SCAN_PORT_WRITE,
                const unsigned short mock_port_receive = SCAN_PORT_READ);

  MOCK_CONST_METHOD0(receivedUdpMsg, void());

public:
  void startIOService();

  void asyncReceive();

  template <unsigned int N>
  void asyncSend(const udp::endpoint& receiver_of_data, const std::array<char, N>& data);

private:
  void handleReceive(const boost::system::error_code& error, std::size_t /*bytes_transferred*/);
  void handleSend(const boost::system::error_code& /*error*/, std::size_t /*bytes_transferred*/);

private:
  udp::endpoint remote_endpoint_;

  boost::array<char, 100> recv_buffer_;

  boost::asio::io_service io_service_;
  // Prevent the run() method of the io_service from returning when there is no more work.
  std::unique_ptr<boost::asio::io_service::work> work_{ new boost::asio::io_service::work(io_service_) };
  boost::thread service_thread_;

  //! Socket over which the mock receives data from the client.
  udp::socket socket_receive_;
  //! Socket over which the mock sends data to the client.
  udp::socket socket_send_;
};

inline void MockUDPServer::startIOService()
{
  service_thread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_));
}

inline MockUDPServer::MockUDPServer(const unsigned short mock_port_send, const unsigned short mock_port_receive)
  : socket_receive_(io_service_, udp::endpoint(udp::v4(), mock_port_receive))
  , socket_send_(io_service_, udp::endpoint(boost::asio::ip::address_v4::from_string(MOCK_IP_ADDRESS), mock_port_send))
{
}

inline MockUDPServer::~MockUDPServer()
{
  io_service_.stop();
  if (service_thread_.joinable())
  {
    service_thread_.join();
  }
  socket_receive_.close();
  socket_send_.close();
}

inline void MockUDPServer::handleSend(const boost::system::error_code& /*error*/, std::size_t /*bytes_transferred*/)
{
}

template <unsigned int N>
inline void MockUDPServer::asyncSend(const udp::endpoint& receiver_of_data, const std::array<char, N>& data)
{
  socket_send_.async_send_to(boost::asio::buffer(data),
                             receiver_of_data,
                             boost::bind(&MockUDPServer::handleSend,
                                         this,
                                         boost::asio::placeholders::error,
                                         boost::asio::placeholders::bytes_transferred));
}

inline void MockUDPServer::handleReceive(const boost::system::error_code& error, std::size_t /*bytes_transferred*/)
{
  receivedUdpMsg();
}

inline void MockUDPServer::asyncReceive()
{
  socket_receive_.async_receive_from(boost::asio::buffer(recv_buffer_),
                                     remote_endpoint_,
                                     boost::bind(&MockUDPServer::handleReceive,
                                                 this,
                                                 boost::asio::placeholders::error,
                                                 boost::asio::placeholders::bytes_transferred));
}

}  // namespace psen_scan_test

#endif  // PSEN_SCAN_TEST_MOCK_UDP_SERVER_H
