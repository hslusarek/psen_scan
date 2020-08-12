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
#ifndef PSEN_SCAN_START_REQUEST_H
#define PSEN_SCAN_START_REQUEST_H

#include <array>
#include <cstdint>
#include <string>

#include <psen_scan/scanner_configuration.h>

#pragma pack(push, 1)

namespace psen_scan
{
/**
 * @brief Frame containing all necessary fields for a Start Request.
 *
 * Unless otherwise indicated the byte order is little endian.
 *
 */
class StartRequest
{
public:
  StartRequest(const ScannerConfiguration& scanner_configuration, const uint32_t& seq_number);

  uint32_t getCRC() const;

private:
  void setHostIP(const uint32_t& host_ip);

private:
  uint32_t crc_; /**< A CRC32 of all the following fields. */
  uint32_t seq_number_;
  uint64_t const RESERVED_{ 0 };           /**< Use all zeros */
  uint32_t const OPCODE_{ htole32(0x35) }; /**< Constant 0x35. */
  uint32_t host_ip_;                     /**< Byte order: big endian */
  uint16_t host_udp_port_;               /**< Byte order: big endian */

  /**< The following 'enable' fields are a 1-byte mask each.
   * Only the last 4 bits (little endian) are used, each of which represents a device.
   * For example, (1000) only enables the Master device, while (1010) enables both the Master
   * and the second Slave device.
   */
  uint8_t device_enabled_{ 0b00001000 };
  uint8_t intensity_enabled_{ 0 };
  uint8_t point_in_safety_enabled_{ 0 };
  uint8_t active_zone_set_enabled_{ 0 };
  uint8_t io_pin_enabled_{ 0 };
  uint8_t scan_counter_enabled_{ 0 };
  uint8_t speed_encoder_enabled_{ 0 }; /**< 0000000bin disabled, 00001111bin enabled.*/
  uint8_t diagnostics_enabled_{ 0 };

  class DeviceField
  {
  public:
    DeviceField() : start_angle_(0), end_angle_(0), resolution_(0)
    {
    }
    DeviceField(const uint16_t& start_angle, const uint16_t& end_angle, const uint16_t& resolution)
      : start_angle_(start_angle), end_angle_(end_angle), resolution_(resolution)
    {
    }

    /**< The following 'angle' and 'resolution' fields are all in tenth of degrees */
    uint16_t start_angle_{ 0 };
    uint16_t end_angle_{ 0 };
    uint16_t resolution_{ 0 };
  };

  DeviceField master_{ 0, 2700, 1 };
  std::array<DeviceField, 3> slaves_;

public:
  using RawType = std::array<char, 58>;
  RawType toCharArray();
};

}  // namespace psen_scan

#pragma pack(pop)

#endif  // PSEN_SCAN_START_REQUEST_H
