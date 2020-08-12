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

#include <stdexcept>
#include <string>
#include <limits>

#include <arpa/inet.h>
#include <gtest/gtest.h>

#include <psen_scan/scanner_configuration.h>

using namespace psen_scan;

namespace psen_scan_test
{
static constexpr int MAXIMAL_PORT_NUMBER{ std::numeric_limits<uint16_t>::max() };
static const std::string VALID_IP{ "127.0.0.1" };
static const std::string INVALID_IP{ "invalid_ip" };

TEST(ScannerConfigurationTest, testConstructorSuccess)
{
  const std::string target_ip = VALID_IP;
  const int target_udp_port = MAXIMAL_PORT_NUMBER;

  EXPECT_NO_THROW(ScannerConfiguration sc(target_ip, target_udp_port));
}

TEST(ScannerConfigurationTest, testConstructorInvalidIp)
{
  const std::string target_ip = INVALID_IP;
  const int target_udp_port = MAXIMAL_PORT_NUMBER;

  EXPECT_THROW(ScannerConfiguration sc(target_ip, target_udp_port), std::invalid_argument);
}

TEST(ScannerConfigurationTest, testConstructorInvalidPort)
{
  const std::string target_ip = VALID_IP;
  int target_udp_port{ std::numeric_limits<uint16_t>::min() - 1 };

  EXPECT_THROW(ScannerConfiguration sc(target_ip, target_udp_port), std::invalid_argument);

  target_udp_port = MAXIMAL_PORT_NUMBER + 1;

  EXPECT_THROW(ScannerConfiguration sc(target_ip, target_udp_port), std::invalid_argument);
}

TEST(ScannerConfigurationTest, testTargetIp)
{
  const std::string expected_target_ip = VALID_IP;
  const int target_udp_port = MAXIMAL_PORT_NUMBER;

  ScannerConfiguration sc(expected_target_ip, target_udp_port);

  const auto target_ip = sc.targetIp();
  EXPECT_EQ(4U, sizeof(target_ip));

  // convert target_ip back to string representation
  const auto network_number = inet_makeaddr(target_ip, 0);
  const auto network_number_ascii = inet_ntoa(network_number);
  const std::string target_ip_string(network_number_ascii);

  EXPECT_EQ(expected_target_ip, target_ip_string);
}

TEST(ScannerConfigurationTest, testTargetUDPPort)
{
  const std::string target_ip = VALID_IP;
  const int expected_target_udp_port = MAXIMAL_PORT_NUMBER;

  ScannerConfiguration sc(target_ip, expected_target_udp_port);

  const auto target_udp_port = sc.targetUDPPort();
  EXPECT_EQ(2U, sizeof(target_udp_port));
  EXPECT_EQ(expected_target_udp_port, static_cast<int>(target_udp_port));
}

}  // namespace psen_scan_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
