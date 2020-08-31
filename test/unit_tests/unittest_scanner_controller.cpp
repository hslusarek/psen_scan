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

#include <math.h>
#include <memory>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <psen_scan/mock_controller_state_machine.h>
#include <psen_scan/mock_udp_client.h>
#include <psen_scan/scanner_configuration.h>
#include <psen_scan/scanner_controller.h>

using namespace psen_scan;

namespace psen_scan_test
{
static const std::string HOST_IP{ "127.0.0.1" };
static constexpr int HOST_UDP_PORT_DATA{ 50505 };
static constexpr int HOST_UDP_PORT_CONTROL{ 55055 };
static const std::string DEVICE_IP{ "127.0.0.100" };
static constexpr double START_ANGLE{ 0. };
static constexpr double END_ANGLE{ 275. * 2 * M_PI / 360. };

class ScannerControllerTest : public ::testing::Test
{
protected:
  ScannerControllerTest()
    : scanner_config_(HOST_IP, HOST_UDP_PORT_DATA, HOST_UDP_PORT_CONTROL, DEVICE_IP, START_ANGLE, END_ANGLE)
    , mock_csm_(std::make_shared<MockControllerStateMachine>())
    , mock_control_udp_client_(std::make_shared<MockUdpClient>())
    , mock_data_udp_client_(std::make_shared<MockUdpClient>())
    , scanner_controller_(scanner_config_, mock_csm_, mock_control_udp_client_, mock_data_udp_client_)
  {
  }

protected:
  ScannerConfiguration scanner_config_;
  std::shared_ptr<MockControllerStateMachine> mock_csm_;
  std::shared_ptr<MockUdpClient> mock_control_udp_client_;
  std::shared_ptr<MockUdpClient> mock_data_udp_client_;
  ScannerController scanner_controller_;
};

TEST_F(ScannerControllerTest, test_start_method_calls_correct_state_machine_event)
{
  EXPECT_CALL(*mock_csm_, processStartRequestEvent()).Times(1);

  scanner_controller_.start();
}

TEST_F(ScannerControllerTest, test_stop_method_calls_correct_state_machine_event)
{
  EXPECT_CALL(*mock_csm_, processStopRequestEvent()).Times(1);

  scanner_controller_.stop();
}

TEST_F(ScannerControllerTest, test_udp_clients_listen_before_sending_start_request)
{
  using ::testing::_;
  using ::testing::Expectation;

  Expectation control_udp_client_start_receiving = EXPECT_CALL(*mock_control_udp_client_, startReceiving(_));
  Expectation data_udp_client_start_receiving = EXPECT_CALL(*mock_data_udp_client_, startReceiving(_));
  EXPECT_CALL(*mock_control_udp_client_, write(_, _))
      .After(control_udp_client_start_receiving, data_udp_client_start_receiving);

  scanner_controller_.sendStartRequest();
}

TEST_F(ScannerControllerTest, test_handle_error_no_throw)
{
  ASSERT_NO_THROW(scanner_controller_.handleError("Error Message."));
}

}  // namespace psen_scan_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
