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

#include <array>

namespace psen_scan
{

typedef struct MasterDeviceConfiguration
{
  /**< The following 'angle' and 'resolution' fields are all in tenth of degrees */
  uint16_t start_angle_;
  uint16_t end_angle_;
  uint16_t resolution_;

  MasterDeviceConfiguration()
  : start_angle_(0)
  , end_angle_(2750)
  , resolution_(1)
  {
  }

protected:
  MasterDeviceConfiguration(const uint16_t& resolution_)
  : start_angle_(0)
  , end_angle_(2750)
  , resolution_(resolution_)
  {
  }
} MasterDeviceConfiguration;

typedef struct SlaveDeviceConfiguration : public MasterDeviceConfiguration
{
  bool enabled_;
  SlaveDeviceConfiguration()
  : MasterDeviceConfiguration(10)
  , enabled_(false)
  {
  }
} SlaveDeviceConfiguration;

typedef struct ScannerConfiguration
{
  int unsigned constexpr static NUMBER_OF_SLAVES {3};

  std::string target_ip_;
  uint16_t target_udp_port_;

  MasterDeviceConfiguration master_;
  std::array<SlaveDeviceConfiguration, NUMBER_OF_SLAVES> slaves_;

  bool intensity_enabled_;
  bool point_in_safety_enabled_;
  bool active_zone_set_enabled_;
  bool io_pin_enabled_;
  bool scan_counter_enabled_;
  bool speed_encoder_enabled_;
  bool diagnostics_enabled_;

  ScannerConfiguration(const std::string& target_ip, const uint16_t& target_udp_port)
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
} ScannerConfiguration;

}

#endif // PSEN_SCAN_SCANNER_CONFIGURATION_H
