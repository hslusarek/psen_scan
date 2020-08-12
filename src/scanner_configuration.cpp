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
#include <limits>

#include <arpa/inet.h>

#include <psen_scan/scanner_configuration.h>

namespace psen_scan
{
ScannerConfiguration::ScannerConfiguration(const std::string& host_ip, const int& host_udp_port)
{
  host_ip_ = inet_network(host_ip.c_str());
  if (static_cast<in_addr_t>(-1) == host_ip_)
  {
    throw std::invalid_argument("Host IP invalid");
  }

  if (host_udp_port < std::numeric_limits<uint16_t>::min() || host_udp_port > std::numeric_limits<uint16_t>::max())
  {
    throw std::invalid_argument("Host UDP port out of range");
  }
  host_udp_port_ = htole16(static_cast<uint16_t>(host_udp_port));
}

uint32_t ScannerConfiguration::hostIp() const
{
  return host_ip_;
}

uint16_t ScannerConfiguration::hostUDPPort() const
{
  return host_udp_port_;
}

}  // namespace psen_scan
