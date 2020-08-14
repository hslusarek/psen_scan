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

#include <cmath>
#include <cassert>
#include <stdexcept>
#include <limits>

#include <arpa/inet.h>

#include <psen_scan/scanner_configuration.h>

namespace psen_scan
{
static constexpr float MIN_SCAN_ANGLE{ 0.0 };
static constexpr float MAX_SCAN_ANGLE{ 275.0 };

ScannerConfiguration::ScannerConfiguration(const std::string& host_ip,
                                           const int& host_udp_port,
                                           const std::string& device_ip,
                                           const float& start_angle,
                                           const float& end_angle)
{
  const auto host_ip_number = inet_network(host_ip.c_str());
  if (static_cast<in_addr_t>(-1) == host_ip_number)
  {
    throw std::invalid_argument("Host IP invalid");
  }
  assert(sizeof(host_ip_number) == 4);
  host_ip_ = static_cast<uint32_t>(host_ip_number);

  if (host_udp_port < std::numeric_limits<uint16_t>::min() || host_udp_port > std::numeric_limits<uint16_t>::max())
  {
    throw std::invalid_argument("Host UDP port out of range");
  }
  host_udp_port_ = htole16(static_cast<uint16_t>(host_udp_port));

  const auto device_ip_number = inet_network(device_ip.c_str());
  if (static_cast<in_addr_t>(-1) == device_ip_number)
  {
    throw std::invalid_argument("Device IP invalid");
  }
  assert(sizeof(device_ip_number) == 4);
  device_ip_ = static_cast<uint32_t>(device_ip_number);

  if (start_angle < MIN_SCAN_ANGLE || start_angle > MAX_SCAN_ANGLE)
  {
    throw std::invalid_argument("Start angle out of supported range");
  }
  if (end_angle < start_angle || end_angle > MAX_SCAN_ANGLE)
  {
    throw std::invalid_argument("End angle out of supported range");
  }

  start_angle_ = static_cast<uint16_t>(round(start_angle*10.0));
  end_angle_ = static_cast<uint16_t>(round(end_angle*10.0));
}

uint32_t ScannerConfiguration::hostIp() const
{
  return host_ip_;
}

uint16_t ScannerConfiguration::hostUDPPortRead() const
{
  return host_udp_port_;
}

uint16_t ScannerConfiguration::hostUDPPortWrite() const
{
  return host_udp_port_ + 1;
}

uint32_t ScannerConfiguration::deviceIp() const
{
  return device_ip_;
}

uint16_t ScannerConfiguration::startAngle() const
{
  return start_angle_;
}

uint16_t ScannerConfiguration::endAngle() const
{
  return end_angle_;
}

}  // namespace psen_scan
