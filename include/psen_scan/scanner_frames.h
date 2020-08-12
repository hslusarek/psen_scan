// Copyright (c) 2019 Pilz GmbH & Co. KG
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

#ifndef PSEN_SCAN_SCANNER_FRAMES_H
#define PSEN_SCAN_SCANNER_FRAMES_H

#include <cstdint>
#include <array>

#include <psen_scan/psen_scan_internal_angle.h>

namespace psen_scan
{
#pragma pack(push, 1)  // Don't allow padding

/**
 * @brief DataReply as coming from Laserscanner. Beeing either StartReply or StopReply
 *        Comments according to
 *        Reference Guide Rev. A – November 2019 Page 6
 */
namespace DataReply
{
static constexpr uint32_t OPCODE_START{ 0x35 };
enum class Type
{
  Start,
  Unknown
};

typedef struct MemoryFormat
{
  Type type() const
  {
    if (opcode_ == OPCODE_START)
    {
      return Type::Start;
    }
    else
    {
      return Type::Unknown;
    }
  }

  uint32_t crc_ = 0;      /**< A CRC32 of all the following fields. */
  uint32_t RESERVED_ = 0; /**< - */
  uint32_t opcode_ = 0;   /**< Operation Code (START 0x35, STOP 0x36). */
  uint32_t res_code_ = 0; /**< Operation result. If the message is accepted,
                               the returned value is 0x00. If the message is
                               refused, the returned value is 0xEB. If the CRC
                               is not correct, the device will not send any
                               message.*/
} MemoryFormat;
}  // namespace DataReply

#pragma pack(pop)
}  // namespace psen_scan

#endif  // PSEN_SCAN_SCANNER_FRAMES_H
