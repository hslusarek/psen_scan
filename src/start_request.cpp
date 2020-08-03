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
#include <endian.h>

#include <psen_scan/crc.h>
#include <psen_scan/start_request.h>

namespace psen_scan
{

StartRequest::StartRequest(const ScannerConfiguration& scanner_configuration)
: target_udp_port_(htobe16(scanner_configuration.targetUDPPort()))
{
  // TODO: What to do with seq_number???
  setTargetIP(scanner_configuration.targetIp());
  setEnableFields(scanner_configuration);
  auto slave_ranges = scanner_configuration.getSlaveRanges();
  std::array<Range, ScannerConfiguration::NUMBER_OF_SLAVES + 1> ranges{scanner_configuration.getMasterRange(), slave_ranges[0], slave_ranges[1], slave_ranges[2]};
  setDeviceFields(ranges);
  setCRC();
}

void StartRequest::setTargetIP(const std::string& target_ip)
{
  in_addr_t ip_addr = inet_network(target_ip.c_str());
  if (static_cast<in_addr_t>(-1) == ip_addr)
  {
    throw; //TODO: What to throw
  }
  target_ip_ = htobe32(ip_addr);
}

void StartRequest::setEnableFields(const ScannerConfiguration& scanner_configuration)
{
  device_enabled_ = 0b00001000
                  + scanner_configuration.getSlaveEnabled(0) * 0b00000100
                  + scanner_configuration.getSlaveEnabled(1) * 0b00000010
                  + scanner_configuration.getSlaveEnabled(2) * 0b00000001;

  intensity_enabled_ = scanner_configuration.intensityEnabled() * device_enabled_;
  point_in_safety_enabled_ = scanner_configuration.pointInSafetyEnabled() * device_enabled_;
  active_zone_set_enabled_ = scanner_configuration.activeZoneSetEnabled() * device_enabled_;
  io_pin_enabled_ = scanner_configuration.ioPinEnabled() * device_enabled_;
  scan_counter_enabled_ = scanner_configuration.scanCounterEnabled() * device_enabled_;
  diagnostics_enabled_ = scanner_configuration.diagnosticsEnabled() * device_enabled_;

  speed_encoder_enabled_ = scanner_configuration.speedEncoderEnabled() * 0b00001111;
}

void StartRequest::setDeviceFields(const std::array<Range, ScannerConfiguration::NUMBER_OF_SLAVES + 1>& ranges)
{
  PSENscanInternalAngle start_angle(0), end_angle(0), resolution(0);
  for(unsigned int i = 0; i < ranges.size(); i++)
  {
    std::tie(start_angle, end_angle, resolution) = ranges[i];
    devices_[i].start_angle_ = htole16(static_cast<uint16_t>(static_cast<int>(start_angle)));
    devices_[i].end_angle_ = htole16(static_cast<uint16_t>(static_cast<int>(end_angle)));
    devices_[i].resolution_ = htole16(static_cast<uint16_t>(static_cast<int>(resolution)));
  }
}

void StartRequest::setCRC()
{
  crc_ = htole32(crc::calcCRC32(&seq_number_, sizeof(StartRequest) - sizeof(crc_)));
}

}  // namespace psen_scan