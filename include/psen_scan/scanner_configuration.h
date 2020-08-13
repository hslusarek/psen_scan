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
  ScannerConfiguration(const std::string& host_ip,
                       const int& host_udp_port,
                       const std::string& device_ip);

public:
  uint32_t hostIp() const;

  uint16_t hostUDPPortRead() const;
  uint16_t hostUDPPortWrite() const;

  uint32_t deviceIp() const;

private:
  uint32_t host_ip_;
  uint16_t host_udp_port_;

  uint32_t device_ip_;
};

}  // namespace psen_scan

#endif  // PSEN_SCAN_SCANNER_CONFIGURATION_H
