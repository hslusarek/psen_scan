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
static constexpr float MINIMAL_SCAN_ANGLE{ 0.0 };
static constexpr float MAXIMAL_SCAN_ANGLE{ 275.0 };
static const std::string VALID_IP{ "127.0.0.1" };
static const std::string INVALID_IP{ "invalid_ip" };

class ScannerConfigurationTest : public testing::Test
{
protected:
  std::string host_ip_{ VALID_IP };
  std::string client_ip_{ VALID_IP };
  int host_udp_port_data_{ MAXIMAL_PORT_NUMBER - 1 };
  int host_udp_port_control_{ MAXIMAL_PORT_NUMBER };
  float start_angle_{ MINIMAL_SCAN_ANGLE };
  float end_angle_{ MAXIMAL_SCAN_ANGLE };
};

TEST_F(ScannerConfigurationTest, testConstructorSuccess)
{
  EXPECT_NO_THROW(ScannerConfiguration sc(
      host_ip_, host_udp_port_data_, host_udp_port_control_, client_ip_, start_angle_, end_angle_));
}

TEST_F(ScannerConfigurationTest, testConstructorInvalidHostIp)
{
  host_ip_ = INVALID_IP;

  EXPECT_THROW(ScannerConfiguration sc(
                   host_ip_, host_udp_port_data_, host_udp_port_control_, client_ip_, start_angle_, end_angle_),
               std::invalid_argument);
}

TEST_F(ScannerConfigurationTest, testConstructorInvalidClientIp)
{
  client_ip_ = INVALID_IP;

  EXPECT_THROW(ScannerConfiguration sc(
                   host_ip_, host_udp_port_data_, host_udp_port_control_, client_ip_, start_angle_, end_angle_),
               std::invalid_argument);
}

TEST_F(ScannerConfigurationTest, testConstructorInvalidDataPort)
{
  host_udp_port_data_ = std::numeric_limits<uint16_t>::min() - 1;

  EXPECT_THROW(ScannerConfiguration sc(
                   host_ip_, host_udp_port_data_, host_udp_port_control_, client_ip_, start_angle_, end_angle_),
               std::invalid_argument);

  host_udp_port_data_ = MAXIMAL_PORT_NUMBER + 1;

  EXPECT_THROW(ScannerConfiguration sc(
                   host_ip_, host_udp_port_data_, host_udp_port_control_, client_ip_, start_angle_, end_angle_),
               std::invalid_argument);
}

TEST_F(ScannerConfigurationTest, testConstructorInvalidControlPort)
{
  host_udp_port_control_ = std::numeric_limits<uint16_t>::min() - 1;

  EXPECT_THROW(ScannerConfiguration sc(
                   host_ip_, host_udp_port_data_, host_udp_port_control_, client_ip_, start_angle_, end_angle_),
               std::invalid_argument);

  host_udp_port_control_ = MAXIMAL_PORT_NUMBER + 1;

  EXPECT_THROW(ScannerConfiguration sc(
                   host_ip_, host_udp_port_data_, host_udp_port_control_, client_ip_, start_angle_, end_angle_),
               std::invalid_argument);
}

TEST_F(ScannerConfigurationTest, testConstructorInvalidAngles)
{
  start_angle_ = MINIMAL_SCAN_ANGLE - 0.1;

  EXPECT_THROW(ScannerConfiguration sc(
                   host_ip_, host_udp_port_data_, host_udp_port_control_, client_ip_, start_angle_, end_angle_),
               std::invalid_argument);

  start_angle_ = MAXIMAL_SCAN_ANGLE;
  end_angle_ = MAXIMAL_SCAN_ANGLE - 0.1;

  EXPECT_THROW(ScannerConfiguration sc(
                   host_ip_, host_udp_port_data_, host_udp_port_control_, client_ip_, start_angle_, end_angle_),
               std::invalid_argument);

  end_angle_ = MAXIMAL_SCAN_ANGLE + 0.1;

  EXPECT_THROW(ScannerConfiguration sc(
                   host_ip_, host_udp_port_data_, host_udp_port_control_, client_ip_, start_angle_, end_angle_),
               std::invalid_argument);
}

TEST_F(ScannerConfigurationTest, testTargetIp)
{
  ScannerConfiguration sc(host_ip_, host_udp_port_data_, host_udp_port_control_, client_ip_, start_angle_, end_angle_);

  const auto host_ip = sc.hostIp();
  EXPECT_EQ(4U, sizeof(host_ip));

  // convert host_ip back to string representation
  const auto network_number = inet_makeaddr(host_ip, 0);
  const auto network_number_ascii = inet_ntoa(network_number);
  const std::string host_ip_string(network_number_ascii);

  EXPECT_EQ(host_ip_, host_ip_string);
}

TEST_F(ScannerConfigurationTest, testUDPPorts)
{
  ScannerConfiguration sc(host_ip_, host_udp_port_data_, host_udp_port_control_, client_ip_, start_angle_, end_angle_);

  const auto host_udp_port_data = sc.hostUDPPortData();
  EXPECT_EQ(2U, sizeof(host_udp_port_data));
  EXPECT_EQ(host_udp_port_data_, static_cast<int>(host_udp_port_data));

  const auto host_udp_port_control = sc.hostUDPPortControl();
  EXPECT_EQ(2U, sizeof(host_udp_port_control));
  EXPECT_EQ(host_udp_port_control_, static_cast<int>(host_udp_port_control));
}

TEST_F(ScannerConfigurationTest, testStartAngle)
{
  ScannerConfiguration sc(host_ip_, host_udp_port_data_, host_udp_port_control_, client_ip_, start_angle_, end_angle_);

  const auto start_angle = sc.startAngle();
  EXPECT_EQ(2U, sizeof(start_angle));
  EXPECT_FLOAT_EQ(start_angle_, start_angle * 0.1F);
}

TEST_F(ScannerConfigurationTest, testEndAngle)
{
  ScannerConfiguration sc(host_ip_, host_udp_port_data_, host_udp_port_control_, client_ip_, start_angle_, end_angle_);

  const auto end_angle = sc.endAngle();
  EXPECT_EQ(2U, sizeof(end_angle));
  EXPECT_FLOAT_EQ(end_angle_, end_angle * 0.1F);
}

}  // namespace psen_scan_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
