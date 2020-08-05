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
#ifndef PSEN_SCAN_DECODE_EXCEPTION_H
#define PSEN_SCAN_DECODE_EXCEPTION_H

#include <stdexcept>

namespace psen_scan
{
class DecodeException : public std::exception
{
public:
  virtual char const* what() const noexcept
  {
    return "Decoding failed.";
  }
};

class DecodeCRCMismatchException : DecodeException
{
public:
  virtual char const* what() const noexcept
  {
    return "Decoding failed! CRC did not match!";
  }
};

}  // namespace psen_scan
#endif  // PSEN_SCAN_NOT_IMPLEMENTED_EXCEPTION_H