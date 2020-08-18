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
#ifndef PSEN_SCAN_STOP_REQUEST_H
#define PSEN_SCAN_STOP_REQUEST_H

#include <array>
#include <cstdint>
#include <string>

#include <psen_scan/scanner_configuration.h>

namespace psen_scan
{
/**
 * @brief Frame containing all necessary fields for a Stop Request.
 *
 * Unless otherwise indicated the byte order is little endian.
 *
 */
class StopRequest
{
public:
  uint32_t getCRC() const;

private:
  uint32_t crc_;                           /**< A CRC32 of all the following fields. */
  const uint8_t RESERVED_[12] = { 0 };     /**< Use all zeros */
  uint32_t const OPCODE_{ htole32(0x36) }; /**< Constant 0x36. */

public:
  using RawType = std::array<char, 20>;
  RawType toCharArray();
};

}  // namespace psen_scan

#endif  // PSEN_SCAN_STOP_REQUEST_H
