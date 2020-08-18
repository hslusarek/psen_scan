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
#include <memory>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <psen_scan/reply_msg.h>
#include <psen_scan/msg_decoder.h>

using namespace psen_scan;

static constexpr uint32_t DEFAULT_RESULT_CODE{ 0 };

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

  ReplyMsg reply{ ReplyMsg::getStartOpCode(), DEFAULT_RESULT_CODE };

  RawScannerData data{};
  std::memcpy(&data, &reply, REPLY_MSG_SIZE);

  EXPECT_CALL(mock, start_reply_callback()).Times(1);

  decoder.decodeAndDispatch(data, REPLY_MSG_SIZE);  // TODO get correct size
}

TEST(MsgDecoderTest, decodeStartReplyCrcFail)
{
  MockCallbackHolder mock;
  MsgDecoder decoder(std::bind(&MockCallbackHolder::start_reply_callback, &mock),
                     std::bind(&MockCallbackHolder::errorCallback, &mock, std::placeholders::_1));

  ReplyMsg reply{ ReplyMsg::getStartOpCode(), DEFAULT_RESULT_CODE };

  RawScannerData data{};
  std::memcpy(&data, &reply, REPLY_MSG_SIZE);
  data[0] = 'a';

  EXPECT_CALL(mock, start_reply_callback()).Times(0);

  EXPECT_THROW(decoder.decodeAndDispatch(data, REPLY_MSG_SIZE), DecodeCRCMismatchException);  // TODO get correct size
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

  ReplyMsg reply{ ReplyMsg::getStartOpCode(), DEFAULT_RESULT_CODE };

  RawScannerData data{};
  std::memcpy(&data, &reply, REPLY_MSG_SIZE);

  EXPECT_CALL(mock, start_reply_callback()).Times(0);
  EXPECT_CALL(mock, errorCallback(::testing::_)).Times(1);

  decoder.decodeAndDispatch(data, REPLY_MSG_SIZE + 1);  // TODO get correct size
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

  ReplyMsg reply{ ReplyMsg::getStartOpCode() + 1, DEFAULT_RESULT_CODE };

  RawScannerData data{};
  std::memcpy(&data, &reply, REPLY_MSG_SIZE);

  EXPECT_CALL(mock, start_reply_callback()).Times(0);
  EXPECT_CALL(mock, errorCallback(::testing::_)).Times(1);

  decoder.decodeAndDispatch(data, REPLY_MSG_SIZE);  // TODO get correct size
}

TEST(MsgDecoderTest, testDecodeExceptionForCompleteCoverage)
{
  std::unique_ptr<DecodeException> ex{ new DecodeException() };
}

TEST(MsgDecoderTest, testDecodeCRCMismatchExceptionForCompleteCoverage)
{
  std::unique_ptr<DecodeCRCMismatchException> ex{ new DecodeCRCMismatchException() };
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
