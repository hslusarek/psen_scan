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
#include <array>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <pilz_testutils/async_test.h>

#include "psen_scan/mock_udp_server.h"
#include "psen_scan/sync_udp_writer.h"

using namespace psen_scan;
using namespace ::testing;

namespace psen_scan_test
{
static const std::string MSG_RECEIVED{ "MSG_RECEIVED" };

static const std::string HOST_IP_ADDRESS{ "127.0.0.1" };
static constexpr unsigned short HOST_UDP_WRITE_PORT{ 45001 };

static const std::string UDP_MOCK_IP_ADDRESS{ "127.0.0.1" };
static constexpr unsigned short UDP_MOCK_SEND_PORT{ HOST_UDP_WRITE_PORT + 1 };
static constexpr unsigned short UDP_MOCK_READ_PORT{ HOST_UDP_WRITE_PORT + 2 };

static constexpr std::size_t DATA_SIZE_BYTES{ 100 };

class SyncUdpWriterTests : public testing::Test, public testing::AsyncTest
{
protected:
  MockUDPServer mock_udp_server_{ UDP_MOCK_SEND_PORT, UDP_MOCK_READ_PORT };
  psen_scan::SyncUdpWriter sync_udp_writer_{ HOST_UDP_WRITE_PORT, UDP_MOCK_IP_ADDRESS, UDP_MOCK_READ_PORT };
};

TEST_F(SyncUdpWriterTests, testWriteOperation)
{
  EXPECT_CALL(mock_udp_server_, receivedUdpMsg()).WillOnce(ACTION_OPEN_BARRIER_VOID(MSG_RECEIVED));
  mock_udp_server_.startIOService();

  mock_udp_server_.asyncReceive();
  std::array<char, DATA_SIZE_BYTES> write_buf = { "Hello!" };
  sync_udp_writer_.write(write_buf);

  BARRIER(MSG_RECEIVED);
}

}  // namespace psen_scan_test

int main(int argc, char* argv[])
{
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
