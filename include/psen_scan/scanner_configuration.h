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

namespace psen_scan
{

typedef struct ScannerConfiguration
{
  std::string target_ip_;
  uint16_t target_udp_port_;

  bool master_enabled_;
  bool slave1_enabled_;
  bool slave2_enabled_;
  bool slave3_enabled_;

  bool intensity_enabled_;
  bool point_in_safety_enabled_;
  bool active_zone_set_enabled_;
  bool io_pin_enabled_;
  bool scan_counter_enabled_;
  bool speed_encoder_enabled_;
  bool diagnostics_enabled_;

  /**< The following 'angle' and 'resolution' fields are all in tenth of degrees */
  uint16_t master_start_angle_;
  uint16_t master_end_angle_;
  uint16_t master_resolution_;
  uint16_t slave1_start_angle_;
  uint16_t slave1_end_angle_;
  uint16_t slave1_resolution_;
  uint16_t slave2_start_angle_;
  uint16_t slave2_end_angle_;
  uint16_t slave2_resolution_;
  uint16_t slave3_start_angle_;
  uint16_t slave3_end_angle_;
  uint16_t slave3_resolution_;

  ScannerConfiguration(const std::string& target_ip, const uint16_t& target_udp_port)
  : target_ip_(target_ip)
  , target_udp_port_(target_udp_port)
  , master_enabled_(true)
  , slave1_enabled_(false)
  , slave2_enabled_(false)
  , slave3_enabled_(false)
  , intensity_enabled_(false)
  , point_in_safety_enabled_(false)
  , active_zone_set_enabled_(false)
  , io_pin_enabled_(false)
  , scan_counter_enabled_(false)
  , speed_encoder_enabled_(false)
  , diagnostics_enabled_(false)
  , master_start_angle_(0)
  , master_end_angle_(2750)
  , master_resolution_(1)
  , slave1_start_angle_(0)
  , slave1_end_angle_(2750)
  , slave1_resolution_(10)
  , slave2_start_angle_(0)
  , slave2_end_angle_(2750)
  , slave2_resolution_(10)
  , slave3_start_angle_(0)
  , slave3_end_angle_(2750)
  , slave3_resolution_(10)
  {
  }
} ScannerConfiguration;

}

#endif // PSEN_SCAN_SCANNER_CONFIGURATION_H
