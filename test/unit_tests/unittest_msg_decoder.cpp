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

#include <cstring>
#include <string>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <psen_scan/msg_decoder.h>

using namespace psen_scan;

class MockCallbackHolder
{
public:
  MOCK_METHOD0(start_reply_callback, void());
  MOCK_METHOD1(errorCallback, void(const std::string&));
};

/**
 * Testing if a StartReply message can be identified correctly with the correct crc value.
 * This should call the start_reply_callback method.
 */
TEST(MsgDecoderTest, decodeStartReply)
{
  MockCallbackHolder mock;
  MsgDecoder decoder(std::bind(&MockCallbackHolder::start_reply_callback, &mock),
                     std::bind(&MockCallbackHolder::errorCallback, &mock, std::placeholders::_1));

  DataReply::MemoryFormat reply;
  reply.opcode_ = DataReply::OPCODE_START;
  reply.crc_ = crc::calcCRC32(reply);

  std::array<char, 60000> data{};
  std::memcpy(&data, &reply, sizeof(DataReply::MemoryFormat));

  EXPECT_CALL(mock, start_reply_callback()).Times(1);

  decoder.decodeAndDispatch<60000>(data, sizeof(DataReply::MemoryFormat));  // TODO get correct size
}

/**
 * Testing if a StartReply message can be identified correctly with an *incorrect* crc value.
 * This should *not* call the start_reply_callback method and raise a ParseMonitoringFrameException.
 */
TEST(MsgDecoderTest, decodeStartReplyCrcFail)
{
  MockCallbackHolder mock;
  MsgDecoder decoder(std::bind(&MockCallbackHolder::start_reply_callback, &mock),
                     std::bind(&MockCallbackHolder::errorCallback, &mock, std::placeholders::_1));

  DataReply::MemoryFormat reply;
  reply.opcode_ = DataReply::OPCODE_START;
  reply.crc_ = crc::calcCRC32(reply) + 1;  // expected crc + 1

  std::array<char, 60000> data{};
  std::memcpy(&data, &reply, sizeof(DataReply::MemoryFormat));

  EXPECT_CALL(mock, start_reply_callback()).Times(0);

  EXPECT_THROW(decoder.decodeAndDispatch<60000>(data, sizeof(DataReply::MemoryFormat)),
               DecodeCRCMismatchException);  // TODO get correct size
}

/**
 * Testing if a StartReply message can not be identified correctly with incorrect size.
 * This should *not* call the start_reply_callback method and raise a NotImplementedException.
 */
TEST(MsgDecoderTest, decodeStartReplyWrongSizeNotImplemented)
{
  MockCallbackHolder mock;
  MsgDecoder decoder(std::bind(&MockCallbackHolder::start_reply_callback, &mock),
                     std::bind(&MockCallbackHolder::errorCallback, &mock, std::placeholders::_1));

  DataReply::MemoryFormat reply;
  reply.opcode_ = DataReply::OPCODE_START;

  std::array<char, 60000> data{};
  std::memcpy(&data, &reply, sizeof(DataReply::MemoryFormat));

  EXPECT_CALL(mock, start_reply_callback()).Times(0);
  EXPECT_CALL(mock, errorCallback(::testing::_)).Times(1);

  decoder.decodeAndDispatch<60000>(data, sizeof(DataReply::MemoryFormat) + 1);  // TODO get correct size
}

/**
 * Testing if a StartReply message can not be identified correctly with incorrect opcode.
 * This should *not* call the start_reply_callback method and raise a NotImplementedException.
 */
TEST(MsgDecoderTest, decodeWrongOpCodeNotImplemented)
{
  MockCallbackHolder mock;  // Needed
  MsgDecoder decoder(std::bind(&MockCallbackHolder::start_reply_callback, &mock),
                     std::bind(&MockCallbackHolder::errorCallback, &mock, std::placeholders::_1));

  DataReply::MemoryFormat reply;
  reply.opcode_ = DataReply::OPCODE_START + 1;
  reply.crc_ = crc::calcCRC32(reply);

  std::array<char, 60000> data{};
  std::memcpy(&data, &reply, sizeof(DataReply::MemoryFormat));

  EXPECT_CALL(mock, start_reply_callback()).Times(0);
  EXPECT_CALL(mock, errorCallback(::testing::_)).Times(1);

  decoder.decodeAndDispatch<60000>(data, sizeof(DataReply::MemoryFormat));  // TODO get correct size
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
