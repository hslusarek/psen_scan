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

#ifndef PSEN_SCAN_SCANNER_FRAMES_CONVERSIONS_H
#define PSEN_SCAN_SCANNER_FRAMES_CONVERSIONS_H

#include <array>
#include <psen_scan/scanner_frames.h>
#include <psen_scan/crc.h>
#include <psen_scan/decode_exception.h>

namespace psen_scan
{
template <std::size_t NumberOfBytes>
DataReply::MemoryFormat decode(const std::array<char, NumberOfBytes>& data)
{
  DataReply::MemoryFormat frame;
  // Alternativ:
  // typedef boost::iostreams::basic_array_source<char> Device;
  // boost::iostreams::stream<Device> stream((char*)&data, sizeof(DataReply::MemoryFormat));

  std::istringstream stream(std::string((char*)&data, sizeof(DataReply::MemoryFormat)));

  stream.read((char*)&frame.crc_, sizeof(frame.crc_));
  stream.read((char*)&frame.RESERVED_, sizeof(frame.RESERVED_));
  stream.read((char*)&frame.opcode_, sizeof(frame.opcode_));
  stream.read((char*)&frame.res_code_, sizeof(frame.res_code_));

  if (!crc::checkCRC(frame))
  {
    throw DecodeCRCMismatchException();
  }

  return frame;
}
}  // namespace psen_scan

#endif  // PSEN_SCAN_SCANNER_FRAMES_CONVERSIONS_H