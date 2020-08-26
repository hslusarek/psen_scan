// Copyright (c) 2019-2020 Pilz GmbH & Co. KG
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

#ifndef PSEN_SCAN_REPLY_MSG_H
#define PSEN_SCAN_REPLY_MSG_H

#include <cstdint>
#include <array>
#include <sstream>

#include <boost/crc.hpp>

#include <psen_scan/raw_scanner_data.h>
#include <psen_scan/decode_exception.h>

namespace psen_scan
{
enum class ReplyMsgType
{
  Start,
  Unknown
};

static constexpr std::size_t REPLY_MSG_SIZE = 16;

/**
 * @brief Represents the reply messages send from the scanner device.
 */
class ReplyMsg
{
public:
  static ReplyMsg fromRawData(const RawScannerData& data);

public:
  ReplyMsg(const uint32_t op_code, const uint32_t res_code);

public:
  ReplyMsgType type() const;

public:
  static uint32_t getStartOpCode();
  static uint32_t calcCRC(const ReplyMsg& msg);

  using RawType = std::array<char, REPLY_MSG_SIZE>;
  RawType toCharArray();

private:
  ReplyMsg() = delete;

private:
  //! A CRC32 of all the following fields.
  uint32_t crc_{ 0 };
  uint32_t reserved_{ 0 };
  //! Operation Code (START 0x35, STOP 0x36).
  uint32_t opcode_{ 0 };
  //! Operation result.
  //! If the message is accepted, the returned value is 0x00.
  //! If the message is refused, the returned value is 0xEB.
  //! If the CRC is not correct, the device will not send any message.
  uint32_t res_code_{ 0 };

private:
  static constexpr uint32_t OPCODE_START{ 0x35 };
};

inline uint32_t ReplyMsg::calcCRC(const ReplyMsg& msg)
{
  boost::crc_32_type result;
  // Read all data except the field of the sent crc at the beginning according to:
  // Reference Guide Rev. A – November 2019 Page 14
  result.process_bytes(&(msg.reserved_), sizeof(ReplyMsg::reserved_));
  result.process_bytes(&(msg.opcode_), sizeof(ReplyMsg::opcode_));
  result.process_bytes(&(msg.res_code_), sizeof(ReplyMsg::res_code_));
  return result.checksum();
}

inline uint32_t ReplyMsg::getStartOpCode()
{
  return OPCODE_START;
}

inline ReplyMsg::ReplyMsg(const uint32_t op_code, const uint32_t res_code) : opcode_(op_code), res_code_(res_code)
{
  crc_ = calcCRC(*this);
}

inline ReplyMsg ReplyMsg::fromRawData(const RawScannerData& data)
{
  ReplyMsg msg{ 0, 0 };

  // Alternatives for transformation:
  // typedef boost::iostreams::basic_array_source<char> Device;
  // boost::iostreams::stream<Device> stream((char*)&data, sizeof(DataReply::MemoryFormat));

  std::istringstream stream(std::string((char*)&data, REPLY_MSG_SIZE));

  stream.read((char*)&msg.crc_, sizeof(ReplyMsg::crc_));
  stream.read((char*)&msg.reserved_, sizeof(ReplyMsg::reserved_));
  stream.read((char*)&msg.opcode_, sizeof(ReplyMsg::opcode_));
  stream.read((char*)&msg.res_code_, sizeof(ReplyMsg::res_code_));

  if (msg.crc_ != calcCRC(msg))
  {
    throw DecodeCRCMismatchException();
  }

  return msg;
}

inline ReplyMsgType ReplyMsg::type() const
{
  if (opcode_ == OPCODE_START)
  {
    return ReplyMsgType::Start;
  }
  return ReplyMsgType::Unknown;
}

inline ReplyMsg::RawType ReplyMsg::toCharArray()
{
  std::ostringstream os;

  uint32_t crc{ calcCRC(*this) };
  os.write((char*)&crc, sizeof(uint32_t));
  os.write((char*)&reserved_, sizeof(uint32_t));
  os.write((char*)&opcode_, sizeof(uint32_t));
  os.write((char*)&res_code_, sizeof(uint32_t));

  ReplyMsg::RawType ret_val{};

  // TODO check limits
  std::string data_str(os.str());
  // TODO check if lengths match
  std::copy(data_str.begin(), data_str.end(), ret_val.begin());

  return ret_val;
}

}  // namespace psen_scan

#endif  // PSEN_SCAN_REPLY_MSG_H
