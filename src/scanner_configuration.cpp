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
: target_ip_(target_ip)
, target_udp_port_(target_udp_port)
, intensity_enabled_(false)
, point_in_safety_enabled_(false)
, active_zone_set_enabled_(false)
, io_pin_enabled_(false)
, scan_counter_enabled_(false)
, speed_encoder_enabled_(false)
, diagnostics_enabled_(false)
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

Range ScannerConfiguration::getMasterRange() const
{
  return master_.getRange();
}

std::array<Range, ScannerConfiguration::NUMBER_OF_SLAVES> ScannerConfiguration::getSlaveRanges() const
{
  std::array<Range, NUMBER_OF_SLAVES> ranges{Range(0,0,0), Range(0,0,0), Range(0,0,0)};
  for(unsigned int i = 0; i < NUMBER_OF_SLAVES; i++)
  {
    ranges[i] = slaves_[i].getRange();
  }
  return ranges;
}

bool ScannerConfiguration::getSlaveEnabled(const unsigned int& index) const
{
  return slaves_[index].enabled();
}

bool ScannerConfiguration::intensityEnabled() const
{
  return intensity_enabled_;
}

bool ScannerConfiguration::pointInSafetyEnabled() const
{
  return point_in_safety_enabled_;
}

bool ScannerConfiguration::activeZoneSetEnabled() const
{
  return active_zone_set_enabled_;
}

bool ScannerConfiguration::ioPinEnabled() const
{
  return io_pin_enabled_;
}

bool ScannerConfiguration::scanCounterEnabled() const
{
  return scan_counter_enabled_;
}

bool ScannerConfiguration::speedEncoderEnabled() const
{
  return speed_encoder_enabled_;
}

bool ScannerConfiguration::diagnosticsEnabled() const
{
  return diagnostics_enabled_;
}

}  // namespace psen_scan
