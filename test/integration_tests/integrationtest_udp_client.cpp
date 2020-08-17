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

#include <string>
#include <functional>
#include <array>

#include <boost/asio.hpp>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <pilz_testutils/async_test.h>

#include "psen_scan/mock_udp_server.h"
#include "psen_scan/udp_client.h"

using namespace psen_scan;
using namespace ::testing;
using boost::asio::ip::udp;

namespace psen_scan_test
{
static const std::string CLIENT_RECEIVED_DATA{ "CLIENT_RECEIVED_DATA" };
static const std::string MOCK_RECEIVED_DATA{ "MOCK_RECEIVED_DATA" };
static const std::string TIMEOUT_BARRIER_1{ "TIMEOUT_BARRIER_1" };
static const std::string TIMEOUT_BARRIER_2{ "TIMEOUT_BARRIER_2" };

static const std::string HOST_IP_ADDRESS{ "127.0.0.1" };
static constexpr unsigned short HOST_UDP_PORT{ 45001 };

static const std::string UDP_MOCK_IP_ADDRESS{ "127.0.0.1" };
static constexpr unsigned short UDP_MOCK_PORT{ HOST_UDP_PORT + 1 };

static constexpr std::size_t DATA_SIZE_BYTES{ 100 };
static const boost::posix_time::millisec RECEIVE_TIMEOUT{ 50 };

class UdpClientTests : public testing::Test, public testing::AsyncTest
{
public:
  UdpClientTests();
  MOCK_METHOD2(handleNewData, void(const std::array<char, DATA_SIZE_BYTES>&, const std::size_t&));
  MOCK_METHOD1(handleError, void(const std::string&));

public:
  void sendTestDataToClient();

protected:
  MockUDPServer<DATA_SIZE_BYTES> mock_udp_server_{ UDP_MOCK_PORT };

  psen_scan::UdpClient<DATA_SIZE_BYTES> udp_client_{
    std::bind(&UdpClientTests::handleNewData, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&UdpClientTests::handleError, this, std::placeholders::_1),
    HOST_UDP_PORT,
    inet_network(UDP_MOCK_IP_ADDRESS.c_str()),
    UDP_MOCK_PORT
  };

  std::array<char, DATA_SIZE_BYTES> send_array = { "Hello" };
  const udp::endpoint host_endpoint;
};

UdpClientTests::UdpClientTests()
  : host_endpoint(udp::endpoint(boost::asio::ip::address_v4::from_string(HOST_IP_ADDRESS), HOST_UDP_PORT))
{
}

void UdpClientTests::sendTestDataToClient()
{
  mock_udp_server_.asyncSend<DATA_SIZE_BYTES>(host_endpoint, send_array);
}

TEST_F(UdpClientTests, testAsyncReadOperation)
{
  EXPECT_CALL(*this, handleNewData(_, DATA_SIZE_BYTES)).WillOnce(ACTION_OPEN_BARRIER_VOID(CLIENT_RECEIVED_DATA));

  udp_client_.startReceiving(RECEIVE_TIMEOUT);
  sendTestDataToClient();
  BARRIER(CLIENT_RECEIVED_DATA);
}

TEST_F(UdpClientTests, testTwoConsecutiveTimeouts)
{
  EXPECT_CALL(*this, handleError(_))
      .WillOnce(ACTION_OPEN_BARRIER_VOID(TIMEOUT_BARRIER_1))
      .WillOnce(ACTION_OPEN_BARRIER_VOID(TIMEOUT_BARRIER_2));

  udp_client_.startReceiving(RECEIVE_TIMEOUT);
  BARRIER(TIMEOUT_BARRIER_1);

  udp_client_.startReceiving(RECEIVE_TIMEOUT);
  BARRIER(TIMEOUT_BARRIER_2);
}

TEST_F(UdpClientTests, testRestartAfterTimeout)
{
  {
    ::testing::InSequence seq;
    EXPECT_CALL(*this, handleError(_)).WillOnce(ACTION_OPEN_BARRIER_VOID(TIMEOUT_BARRIER_1));
    EXPECT_CALL(*this, handleNewData(_, _)).WillOnce(ACTION_OPEN_BARRIER_VOID(CLIENT_RECEIVED_DATA));
  }

  udp_client_.startReceiving(RECEIVE_TIMEOUT);
  BARRIER(TIMEOUT_BARRIER_1);

  udp_client_.startReceiving(RECEIVE_TIMEOUT);
  sendTestDataToClient();
  BARRIER(CLIENT_RECEIVED_DATA);
}

TEST_F(UdpClientTests, testWriteOperation)
{
  EXPECT_CALL(mock_udp_server_, receivedUdpMsg(_)).WillOnce(ACTION_OPEN_BARRIER_VOID(CLIENT_RECEIVED_DATA));

  mock_udp_server_.asyncReceive();
  std::array<char, DATA_SIZE_BYTES> write_buf = { "Hello!" };
  udp_client_.write(write_buf);

  BARRIER(CLIENT_RECEIVED_DATA);
}

TEST_F(UdpClientTests, testWritingWhileReceiving)
{
  EXPECT_CALL(*this, handleNewData(_, DATA_SIZE_BYTES)).WillOnce(ACTION_OPEN_BARRIER_VOID(CLIENT_RECEIVED_DATA));
  EXPECT_CALL(mock_udp_server_, receivedUdpMsg(_))
      .WillOnce(DoAll(InvokeWithoutArgs(this, &UdpClientTests::sendTestDataToClient),
                      ACTION_OPEN_BARRIER_VOID(MOCK_RECEIVED_DATA)));

  mock_udp_server_.asyncReceive();

  udp_client_.startReceiving(RECEIVE_TIMEOUT);

  std::array<char, DATA_SIZE_BYTES> write_buf = { "Hello!" };
  udp_client_.write(write_buf);

  BARRIER(MOCK_RECEIVED_DATA);
  BARRIER(CLIENT_RECEIVED_DATA);
}

}  // namespace psen_scan_test

int main(int argc, char* argv[])
{
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}