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

#include <psen_scan/scanner_configuration.h>

namespace psen_scan
{
ScannerConfiguration::ScannerConfiguration(const std::string& target_ip, const uint16_t& target_udp_port)
  : target_ip_(target_ip), target_udp_port_(target_udp_port)
{
}

std::string ScannerConfiguration::targetIp() const
{
  return target_ip_;
}

uint16_t ScannerConfiguration::targetUDPPort() const
{
  return target_udp_port_;
}

}  // namespace psen_scan
