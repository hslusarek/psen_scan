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

#include <arpa/inet.h>
#include <boost/endian/conversion.hpp>

#include <gtest/gtest.h>
#include <psen_scan/scanner_configuration.h>
#include <psen_scan/start_request.h>
#include <psen_scan/tenth_degree_conversion.h>

using namespace psen_scan;

enum class Endian
{
  LITTLE,
  BIG
};

template <typename T>
::testing::AssertionResult
DecodingEquals(StartRequest::RawType const& data, std::size_t offset, T expected, const Endian& endian = Endian::LITTLE)
{
  T actual_val;
  memcpy(&actual_val, (char*)&data + offset, sizeof(T));

  if (endian == Endian::BIG)
  {
    boost::endian::native_to_big_inplace(actual_val);
  }

  if (actual_val == expected)
  {
    return ::testing::AssertionSuccess();
  }
  else
  {
    return ::testing::AssertionFailure() << actual_val << " not equal to " << expected;
  }
}

namespace psen_scan_test
{
class StartRequestTest : public ::testing::Test
{
};

TEST_F(StartRequestTest, constructorTest)
{
  const std::string& host_ip = "192.168.0.1";
  const uint16_t& host_udp_port_data = 65535;

  const double start_angle{ 0.0 };
  const double end_angle{ 4.71 };

  ScannerConfiguration sc(host_ip, host_udp_port_data, 0 /* irrelevant */, "192.168.0.50", start_angle, end_angle);

  uint32_t sequence_number{ 123 };
  StartRequest sr(sc, sequence_number);

  auto data = sr.toCharArray();

  EXPECT_TRUE(DecodingEquals(data, 0x00, (uint32_t)sr.getCRC()));  // CRC

  EXPECT_TRUE(DecodingEquals(data, 0x04, (uint32_t)sequence_number));  // SequenceNumber

  EXPECT_TRUE(DecodingEquals(data, 0x08, (uint64_t)0));  // Reserved

  EXPECT_TRUE(DecodingEquals(data, 0x10, (uint32_t)0x35));  // OP code

  EXPECT_TRUE(DecodingEquals(data, 0x14, inet_network(host_ip.c_str()), Endian::BIG));  // IP

  EXPECT_TRUE(DecodingEquals(data, 0x18, host_udp_port_data));  // UDP port

  EXPECT_TRUE(DecodingEquals(data, 0x1A, (uint8_t)0b00001000));  // Device enabled

  const uint8_t DEFAULT_VALUE_U8{ 0 };
  EXPECT_TRUE(DecodingEquals(data, 0x1B, DEFAULT_VALUE_U8));  // Intensities enabled
  EXPECT_TRUE(DecodingEquals(data, 0x1C, DEFAULT_VALUE_U8));  // Point in safety enabled
  EXPECT_TRUE(DecodingEquals(data, 0x1D, DEFAULT_VALUE_U8));  // Active zone set enabled
  EXPECT_TRUE(DecodingEquals(data, 0x1E, DEFAULT_VALUE_U8));  // IO Pin enabled
  EXPECT_TRUE(DecodingEquals(data, 0x1F, DEFAULT_VALUE_U8));  // Scan counter enabled
  EXPECT_TRUE(DecodingEquals(data, 0x20, DEFAULT_VALUE_U8));  // Speed encoder enabled
  EXPECT_TRUE(DecodingEquals(data, 0x21, DEFAULT_VALUE_U8));  // Diagnostics enabled

  EXPECT_TRUE(DecodingEquals(data, 0x22, radToTenthDegree(start_angle)));  // Master Start Angle
  EXPECT_TRUE(DecodingEquals(data, 0x24, radToTenthDegree(end_angle)));    // Master End Angle
  EXPECT_TRUE(DecodingEquals(data, 0x26, radToTenthDegree(0.0174533)));    // Master Angle Resolution

  const uint16_t DEFAULT_VALUE_U16{ 0 };
  EXPECT_TRUE(DecodingEquals(data, 0x28, DEFAULT_VALUE_U16));  // Slave 1 Start Angle
  EXPECT_TRUE(DecodingEquals(data, 0x2A, DEFAULT_VALUE_U16));  // Slave 1 End Angle
  EXPECT_TRUE(DecodingEquals(data, 0x2C, DEFAULT_VALUE_U16));  // Slave 1 Angle Resolution

  EXPECT_TRUE(DecodingEquals(data, 0x2E, DEFAULT_VALUE_U16));  // Slave 2 Start Angle
  EXPECT_TRUE(DecodingEquals(data, 0x30, DEFAULT_VALUE_U16));  // Slave 2 End Angle
  EXPECT_TRUE(DecodingEquals(data, 0x32, DEFAULT_VALUE_U16));  // Slave 2 Angle Resolution

  EXPECT_TRUE(DecodingEquals(data, 0x34, DEFAULT_VALUE_U16));  // Slave 3 Start Angle
  EXPECT_TRUE(DecodingEquals(data, 0x36, DEFAULT_VALUE_U16));  // Slave 3 End Angle
  EXPECT_TRUE(DecodingEquals(data, 0x38, DEFAULT_VALUE_U16));  // Slave 3 Angle Resolution
}

}  // namespace psen_scan_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
