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
#include <functional>
#include <iostream>

#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include "psen_scan/open_connection_failure.h"
#include "psen_scan/close_connection_failure.h"

namespace psen_scan
{
class InvalidDataHandler : public std::invalid_argument
{
public:
  InvalidDataHandler(const std::string& msg = "New data handler is invalid");
};

class InvalidErrorHandler : public std::invalid_argument
{
public:
  InvalidErrorHandler(const std::string& msg = "Error handler is invalid");
};

template <std::size_t NumberOfBytes>
using NewDataHandler = std::function<void(const std::array<char, NumberOfBytes>&, const std::size_t&)>;

using ErrorHandler = std::function<void(const std::string&)>;

static const boost::posix_time::seconds DEFAULT_TIMEOUT{ 1 };

template <std::size_t NumberOfBytes>
class AsyncUdpReader
{
public:
  AsyncUdpReader(const NewDataHandler<NumberOfBytes>& data_handler,
                const ErrorHandler& error_handler,
                const unsigned short& host_port,
                const std::string& endpoint_ip,
                const unsigned short& endpoint_port);
  ~AsyncUdpReader();

public:
  void startReceiving(const boost::posix_time::time_duration timeout = DEFAULT_TIMEOUT);
  void stopReceiving();

  void close();

private:
  void asyncReceive(const boost::posix_time::time_duration timeout);
  void handleTimeout(const boost::system::error_code& error_code);
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
  NewDataHandler<NumberOfBytes> data_handler_;
  ErrorHandler error_handler_;

  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint endpoint_;

  enum class Receiving
  {
    off,
    on,
    shutdown
  };

  std::atomic<Receiving> receive_status_{ Receiving::off };
};

inline InvalidDataHandler::InvalidDataHandler(const std::string& msg) : std::invalid_argument(msg)
{
}

inline InvalidErrorHandler::InvalidErrorHandler(const std::string& msg) : std::invalid_argument(msg)
{
}

template <std::size_t NumberOfBytes>
inline AsyncUdpReader<NumberOfBytes>::AsyncUdpReader(const NewDataHandler<NumberOfBytes>& data_handler,
                                                   const ErrorHandler& error_handler,
                                                   const unsigned short& host_port,
                                                   const std::string& endpoint_ip,
                                                   const unsigned short& endpoint_port)
  : data_handler_(data_handler)
  , error_handler_(error_handler)
  , socket_(io_service_, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), host_port))
  , endpoint_(boost::asio::ip::address_v4::from_string(endpoint_ip), endpoint_port)
{
  if (!data_handler)
  {
    throw InvalidDataHandler();
  }

  if (!error_handler)
  {
    throw InvalidErrorHandler();
  }

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

  assert(!io_service_thread_.joinable());
  io_service_thread_ = std::thread([this]() { io_service_.run(); });
}

template <std::size_t NumberOfBytes>
inline void AsyncUdpReader<NumberOfBytes>::close()
{
  const Receiving old_status{ receive_status_.exchange(Receiving::shutdown) };
  if (old_status == Receiving::shutdown)
  {
    return;
  }

  timeout_timer_.cancel();
  io_service_.stop();
  socket_.cancel();

  if (io_service_thread_.joinable())
  {
    io_service_thread_.join();
  }

  try
  {
    // Do not close the socket before the io_service thread is finished,
    // otherwise a sporadic segmentation fault appears.
    socket_.close();
  }
  // LCOV_EXCL_START
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
  catch (const CloseConnectionFailure& ex)
  {
    std::cout << "ERROR: " << ex.what() << std::endl;
  }
  // LCOV_EXCL_STOP
}

template <std::size_t NumberOfBytes>
void AsyncUdpReader<NumberOfBytes>::handleTimeout(const boost::system::error_code& error_code)
{
  if ((receive_status_ != Receiving::on) || error_code)
  {
    return;
  }
  stopReceiving();
}

template <std::size_t NumberOfBytes>
inline void AsyncUdpReader<NumberOfBytes>::handleReceive(const boost::system::error_code& error_code,
                                                        const std::size_t& bytes_received,
                                                        const boost::posix_time::time_duration timeout)
{
  if (error_code || bytes_received == 0)
  {
    static auto expected_status{ Receiving::on };
    receive_status_.compare_exchange_strong(expected_status, Receiving::off);
    error_handler_(error_code.message());
    return;
  }

  data_handler_(received_data_, bytes_received);
  asyncReceive(timeout);
}

template <std::size_t NumberOfBytes>
inline void AsyncUdpReader<NumberOfBytes>::startReceiving(const boost::posix_time::time_duration timeout)
{
  static auto expected_status{ Receiving::off };
  if (!receive_status_.compare_exchange_strong(expected_status, Receiving::on))
  {
    return;
  }
  asyncReceive(timeout);
}

template <std::size_t NumberOfBytes>
void AsyncUdpReader<NumberOfBytes>::stopReceiving()
{
  // Cancel all outstanding asynchronous receive operations
  socket_.cancel();
}

template <std::size_t NumberOfBytes>
inline void AsyncUdpReader<NumberOfBytes>::asyncReceive(const boost::posix_time::time_duration timeout)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  if (receive_status_.load() != Receiving::on)
  {
    return;
  }

  timeout_timer_.cancel();  // Cancel previous timeout's
  timeout_timer_.expires_from_now(timeout);
  timeout_timer_.async_wait(std::bind(&AsyncUdpReader<NumberOfBytes>::handleTimeout, this, _1));

  socket_.async_receive(boost::asio::buffer(received_data_, NumberOfBytes),
                        std::bind(&AsyncUdpReader<NumberOfBytes>::handleReceive, this, _1, _2, timeout));
}

}  // namespace psen_scan

#endif  // PSEN_SCAN_ASYNC_UDP_READER_H
