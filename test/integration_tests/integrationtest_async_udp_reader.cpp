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
#include <memory>

#include <boost/asio.hpp>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <pilz_testutils/async_test.h>

#include "psen_scan/mock_udp_server.h"
#include "psen_scan/async_udp_reader.h"

using namespace psen_scan;
using namespace ::testing;
using boost::asio::ip::udp;

namespace psen_scan_test
{
static const std::string MSG_RECEIVED{ "MSG_RECEIVED" };
static const std::string TIMEOUT_BARRIER_1{ "TIMEOUT_BARRIER_1" };
static const std::string TIMEOUT_BARRIER_2{ "TIMEOUT_BARRIER_2" };
static const std::string STOP_PROCESSED{ "STOP_PROCESSED" };

static const std::string HOST_IP_ADDRESS{ "127.0.0.1" };
static constexpr unsigned short HOST_UDP_READ_PORT{ 45001 };

static const std::string UDP_MOCK_IP_ADDRESS{ "127.0.0.1" };
static constexpr unsigned short UDP_MOCK_SEND_PORT{ HOST_UDP_READ_PORT + 1 };
static constexpr unsigned short UDP_MOCK_READ_PORT{ HOST_UDP_READ_PORT + 2 };

static constexpr std::size_t DATA_SIZE_BYTES{ 100 };
static const boost::posix_time::millisec RECEIVE_TIMEOUT{ 50 };

class AsynUdpReadTests : public testing::Test, public testing::AsyncTest
{
public:
  MOCK_METHOD2(handleNewData, void(const std::array<char, DATA_SIZE_BYTES>&, const std::size_t&));
  MOCK_METHOD1(handleError, void(const std::string&));

protected:
  void sendTestDataToClient();

protected:
  MockUDPServer mock_udp_server_{ UDP_MOCK_SEND_PORT, UDP_MOCK_READ_PORT };

  psen_scan::AsyncUdpReader<DATA_SIZE_BYTES> async_udp_reader_{
    std::bind(&AsynUdpReadTests::handleNewData, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&AsynUdpReadTests::handleError, this, std::placeholders::_1),
    HOST_UDP_READ_PORT,
    inet_network(UDP_MOCK_IP_ADDRESS.c_str()),
    UDP_MOCK_SEND_PORT
  };
};

void AsynUdpReadTests::sendTestDataToClient()
{
  const udp::endpoint host_endpoint(boost::asio::ip::address_v4::from_string(HOST_IP_ADDRESS), HOST_UDP_READ_PORT);
  std::array<char, DATA_SIZE_BYTES> send_array = { "Hello" };
  mock_udp_server_.asyncSend<DATA_SIZE_BYTES>(host_endpoint, send_array);
}

TEST_F(AsynUdpReadTests, testAsyncReadOperation)
{
  EXPECT_CALL(*this, handleNewData(::testing::_, DATA_SIZE_BYTES)).WillOnce(ACTION_OPEN_BARRIER_VOID(MSG_RECEIVED));

  async_udp_reader_.startReceiving(RECEIVE_TIMEOUT);
  sendTestDataToClient();
  BARRIER(MSG_RECEIVED);
}

TEST_F(AsynUdpReadTests, testTwoConsecutiveTimeouts)
{
  EXPECT_CALL(*this, handleError(::testing::_))
      .WillOnce(ACTION_OPEN_BARRIER_VOID(TIMEOUT_BARRIER_1))
      .WillOnce(ACTION_OPEN_BARRIER_VOID(TIMEOUT_BARRIER_2));

  async_udp_reader_.startReceiving(RECEIVE_TIMEOUT);
  BARRIER(TIMEOUT_BARRIER_1);

  async_udp_reader_.startReceiving(RECEIVE_TIMEOUT);
  BARRIER(TIMEOUT_BARRIER_2);
}

TEST_F(AsynUdpReadTests, testRestartAfterTimeout)
{
  {
    ::testing::InSequence seq;
    EXPECT_CALL(*this, handleError(::testing::_)).WillOnce(ACTION_OPEN_BARRIER_VOID(TIMEOUT_BARRIER_1));
    EXPECT_CALL(*this, handleNewData(::testing::_, ::testing::_)).WillOnce(ACTION_OPEN_BARRIER_VOID(MSG_RECEIVED));
  }

  async_udp_reader_.startReceiving(RECEIVE_TIMEOUT);
  BARRIER(TIMEOUT_BARRIER_1);

  async_udp_reader_.startReceiving(RECEIVE_TIMEOUT);
  sendTestDataToClient();
  BARRIER(MSG_RECEIVED);
}

}  // namespace psen_scan_test

int main(int argc, char* argv[])
{
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
