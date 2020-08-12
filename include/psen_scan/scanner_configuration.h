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
#ifndef PSEN_SCAN_SCANNER_CONFIGURATION_H
#define PSEN_SCAN_SCANNER_CONFIGURATION_H

#include <string>

#include <arpa/inet.h>

namespace psen_scan
{
class ScannerConfiguration
{
public:
  ScannerConfiguration(const std::string& target_ip, const int& target_udp_port);

public:
  in_addr_t targetIp() const;

  uint16_t targetUDPPort() const;

private:
  in_addr_t target_ip_;
  uint16_t target_udp_port_;
};

}  // namespace psen_scan

#endif  // PSEN_SCAN_SCANNER_CONFIGURATION_H
