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

#include <boost/crc.hpp>

#include <algorithm>
#include <sstream>
#include <string>

#include <psen_scan/stop_request.h>

namespace psen_scan
{
uint32_t StopRequest::getCRC() const
{
  boost::crc_32_type result;

  result.process_bytes((char*)&RESERVED_, sizeof(uint8_t) * 12);
  result.process_bytes((char*)&OPCODE_, sizeof(uint32_t));

  return result.checksum();
}

StopRequest::RawType StopRequest::toCharArray()
{
  std::ostringstream os;

  uint32_t crc{ getCRC() };
  os.write((char*)&crc, sizeof(uint32_t));
  os.write((char*)&RESERVED_, sizeof(uint8_t) * 12);
  os.write((char*)&OPCODE_, sizeof(uint32_t));

  StopRequest::RawType ret_val{};

  // TODO check limits
  std::string data_str(os.str());
  // TODO check if lengths match
  std::copy(data_str.begin(), data_str.end(), ret_val.begin());

  return ret_val;
}

}  // namespace psen_scan
