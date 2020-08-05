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
#ifndef PSEN_SCAN_CRC_H
#define PSEN_SCAN_CRC_H

#include <boost/crc.hpp>

#include <psen_scan/scanner_frames.h>

#include <iostream>

namespace psen_scan
{
namespace crc
{
inline uint32_t calcCRC32(const DataReply::MemoryFormat& frame)
{
  boost::crc_32_type result;
  // reading all data except the field of the sent crc at the beginning according to Reference Guide Rev. A – November
  // 2019 Page 14
  result.process_bytes(&(frame.RESERVED_), sizeof(DataReply::MemoryFormat::RESERVED_));
  result.process_bytes(&(frame.opcode_), sizeof(DataReply::MemoryFormat::opcode_));
  result.process_bytes(&(frame.res_code_), sizeof(DataReply::MemoryFormat::res_code_));
  return result.checksum();
}

inline bool checkCRC(const DataReply::MemoryFormat& frame)
{
  return calcCRC32(frame) == frame.crc_;
}

}  // namespace crc

}  // namespace psen_scan

#endif  // PSEN_SCAN_CRC_H
