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
#ifndef PSEN_SCAN_ASYNC_UDP_READER_H
#define PSEN_SCAN_ASYNC_UDP_READER_H

#include <stdexcept>
#include <string>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <iostream>

#include <arpa/inet.h>

#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include "psen_scan/open_connection_failure.h"
#include "psen_scan/close_connection_failure.h"

namespace psen_scan
{
template <std::size_t NumberOfBytes>
using NewDataHandler = std::function<void(const std::array<char, NumberOfBytes>&, const std::size_t&)>;

using ErrorHandler = std::function<void(const std::string&)>;

template <std::size_t NumberOfBytes>
class AsyncUdpReader
{
public:
  AsyncUdpReader(const NewDataHandler<NumberOfBytes>& data_handler,
                 const ErrorHandler& error_handler,
                 const unsigned short& host_port,
                 const in_addr_t& endpoint_ip,
                 const unsigned short& endpoint_port);
  ~AsyncUdpReader();

public:
  void startReceiving(const boost::posix_time::time_duration timeout);
  void close();

private:
  void asyncReceive(const boost::posix_time::time_duration timeout);
  void handleReceive(const boost::system::error_code& error_code,
                     const std::size_t& bytes_received,
                     const boost::posix_time::time_duration timeout);

private:
  boost::asio::io_service io_service_;
  // Prevent the run() method of the io_service from returning when there is no more work.
  boost::asio::io_service::work work_{ io_service_ };
  boost::asio::deadline_timer timeout_timer_{ io_service_ };
  std::thread io_service_thread_;

  std::array<char, NumberOfBytes> received_data_;

  std::atomic_bool receive_called_{ false };
  std::condition_variable receive_cv_;
  std::mutex receive_mutex_;

  NewDataHandler<NumberOfBytes> data_handler_;
  ErrorHandler error_handler_;

  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint endpoint_;
};

template <std::size_t NumberOfBytes>
inline AsyncUdpReader<NumberOfBytes>::AsyncUdpReader(const NewDataHandler<NumberOfBytes>& data_handler,
                                                     const ErrorHandler& error_handler,
                                                     const unsigned short& host_port,
                                                     const in_addr_t& endpoint_ip,
                                                     const unsigned short& endpoint_port)
  : data_handler_(data_handler)
  , error_handler_(error_handler)
  , socket_(io_service_, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), host_port))
  , endpoint_(boost::asio::ip::address_v4(endpoint_ip), endpoint_port)
{
  if (!data_handler)
  {
    throw std::invalid_argument("New data handler is invalid");
  }

  if (!error_handler)
  {
    throw std::invalid_argument("Error handler is invalid");
  }

  try
  {
    socket_.connect(endpoint_);
  }
  // LCOV_EXCL_START
  // No coverage check because testing the socket is not the objective here.
  catch (const boost::system::system_error& ex)
  {
    throw OpenConnectionFailure(ex.what());
  }
  // LCOV_EXCL_STOP

  assert(!io_service_thread_.joinable());
  io_service_thread_ = std::thread([this]() { io_service_.run(); });
}

template <std::size_t NumberOfBytes>
inline void AsyncUdpReader<NumberOfBytes>::close()
{
  io_service_.stop();
  if (io_service_thread_.joinable())
  {
    io_service_thread_.join();
  }

  try
  {
    // Function is intended to be called from the main thread,
    // therefore, the following socket operation happens on the main thread.
    // To avoid concurrency issues, it has to be ensured that the io_service thread
    // is finished before the socket close operation is performed.
    socket_.close();
  }
  // LCOV_EXCL_START
  // No coverage check because testing the socket is not the objective here.
  catch (const boost::system::system_error& ex)
  {
    throw CloseConnectionFailure(ex.what());
  }
  // LCOV_EXCL_STOP
}

template <std::size_t NumberOfBytes>
inline AsyncUdpReader<NumberOfBytes>::~AsyncUdpReader()
{
  try
  {
    close();
  }
  // LCOV_EXCL_START
  // No coverage check because testing the socket is not the objective here.
  catch (const CloseConnectionFailure& ex)
  {
    std::cerr << "ERROR: " << ex.what() << std::endl;
  }
  // LCOV_EXCL_STOP
}

template <std::size_t NumberOfBytes>
inline void AsyncUdpReader<NumberOfBytes>::handleReceive(const boost::system::error_code& error_code,
                                                         const std::size_t& bytes_received,
                                                         const boost::posix_time::time_duration timeout)
{
  if (error_code || bytes_received == 0)
  {
    error_handler_(error_code.message());
    return;
  }

  data_handler_(received_data_, bytes_received);
  asyncReceive(timeout);
}

template <std::size_t NumberOfBytes>
inline void AsyncUdpReader<NumberOfBytes>::startReceiving(const boost::posix_time::time_duration timeout)
{
  // Function is intended to be called from main thread.
  // To ensure that socket operations only happen on one strand (in this case an implicit one),
  // the asyncReceive() operation is scheduled as task to the io_service thread.
  io_service_.post([this, &timeout]() {
    asyncReceive(timeout);
    receive_called_ = true;
    receive_cv_.notify_all();
  });
  std::unique_lock<std::mutex> lock(receive_mutex_);
  receive_cv_.wait(lock, [this]() { return receive_called_.load(); });
}

template <std::size_t NumberOfBytes>
inline void AsyncUdpReader<NumberOfBytes>::asyncReceive(const boost::posix_time::time_duration timeout)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  timeout_timer_.expires_from_now(timeout);
  timeout_timer_.async_wait([this](const boost::system::error_code& error_code) {
    // LCOV_EXCL_START
    // No coverage check. Testing the if-loop is extremly difficult because of timing issues.
    if (error_code)
    {
      return;
    }
    // LCOV_EXCL_STOP
    socket_.cancel();
  });

  socket_.async_receive(boost::asio::buffer(received_data_, NumberOfBytes),
                        std::bind(&AsyncUdpReader<NumberOfBytes>::handleReceive, this, _1, _2, timeout));
}

}  // namespace psen_scan

#endif  // PSEN_SCAN_ASYNC_UDP_READER_H
