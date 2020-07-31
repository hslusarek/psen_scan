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

#include <arpa/inet.h>
#include <cstdint>
#include <endian.h>
#include <string>
#include <vector>

#include <psen_scan/scanner_configuration.h>
#include <psen_scan/crc.h>

#pragma pack(push, 1)

namespace psen_scan
{

/**
 * @brief Frame containing all necessary fields for a Start Request.
 *
 * Unless otherwise indicated the byte order is little endian.
 *
 */
typedef struct StartRequest
{
public:
  uint32_t crc_;                  /**< A CRC32 of all the following fields. */
  uint32_t seq_number_;
  uint64_t const RESERVED_ { 0 };       /**< Use all zeros */
  uint32_t const OPCODE_ { htole32(0x35) };         /**< Constant 0x35. */
  uint32_t target_ip_;            /**< Byte order: big endian */
  uint16_t target_udp_port_;  /**< Byte order: big endian */

  /**< The following 'enable' fields are a 1-byte mask each.
   * Only the last 4 bits (little endian) are used, each of which represents a device.
   * For example, (1000) only enables the Master device, while (1010) enables both the Master
   * and the second Slave device.
   */
  uint8_t device_enabled_;
  uint8_t intensity_enabled_;
  uint8_t point_in_safety_enabled_;
  uint8_t active_zone_set_enabled_;
  uint8_t io_pin_enabled_;
  uint8_t scan_counter_enabled_;
  uint8_t speed_encoder_enabled_;    /**< 0000000bin disabled, 00001111bin enabled.*/
  uint8_t diagnostics_enabled_;

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

public:
  StartRequest(const ScannerConfiguration& scanner_configuration)
  : target_udp_port_(htobe16(scanner_configuration.target_udp_port_))
  , master_start_angle_(htole16(scanner_configuration.master_.start_angle_))
  , master_end_angle_(htole16(scanner_configuration.master_.end_angle_))
  , master_resolution_(htole16(scanner_configuration.master_.resolution_))
  , slave1_start_angle_(htole16(scanner_configuration.slaves_[0].start_angle_))
  , slave1_end_angle_(htole16(scanner_configuration.slaves_[0].end_angle_))
  , slave1_resolution_(htole16(scanner_configuration.slaves_[0].resolution_))
  , slave2_start_angle_(htole16(scanner_configuration.slaves_[1].start_angle_))
  , slave2_end_angle_(htole16(scanner_configuration.slaves_[1].end_angle_))
  , slave2_resolution_(htole16(scanner_configuration.slaves_[1].resolution_))
  , slave3_start_angle_(htole16(scanner_configuration.slaves_[2].start_angle_))
  , slave3_end_angle_(htole16(scanner_configuration.slaves_[2].end_angle_))
  , slave3_resolution_(htole16(scanner_configuration.slaves_[2].resolution_))
  {
    // TODO: What to do with seq_number???
    setTargetIP(scanner_configuration.target_ip_);
    setEnableFields(scanner_configuration);
    setCRC();
  }

private:
  void setTargetIP(const std::string& target_ip)
  {
    in_addr_t ip_addr = inet_network(target_ip.c_str());
    if (static_cast<in_addr_t>(-1) == ip_addr)
    {
      throw; //TODO: What to throw
    }
    target_ip_ = htobe32(ip_addr);
  }

  void setEnableFields(const ScannerConfiguration& scanner_configuration)
  {
    device_enabled_ = 0b00001000
                    + scanner_configuration.slaves_[0].enabled_ * 0b00000100
                    + scanner_configuration.slaves_[1].enabled_ * 0b00000010
                    + scanner_configuration.slaves_[2].enabled_ * 0b00000001;

    intensity_enabled_ = scanner_configuration.intensity_enabled_ * device_enabled_;
    point_in_safety_enabled_ = scanner_configuration.point_in_safety_enabled_ * device_enabled_;
    active_zone_set_enabled_ = scanner_configuration.active_zone_set_enabled_ * device_enabled_;
    io_pin_enabled_ = scanner_configuration.io_pin_enabled_ * device_enabled_;
    scan_counter_enabled_ = scanner_configuration.scan_counter_enabled_ * device_enabled_;
    diagnostics_enabled_ = scanner_configuration.diagnostics_enabled_ * device_enabled_;

    speed_encoder_enabled_ = scanner_configuration.speed_encoder_enabled_ * 0b00001111;
  }

  void setCRC()
  {
    crc_ = htole32(crc::calcCRC32(&seq_number_, sizeof(StartRequest) - sizeof(crc_)));
  }

} StartRequest;

}  // namespace psen_scan

#pragma pack(pop)

#endif // PSEN_SCAN_START_REQUEST_H
