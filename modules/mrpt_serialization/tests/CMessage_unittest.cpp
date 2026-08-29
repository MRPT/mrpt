/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <gtest/gtest.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CMessage.h>
#include <mrpt/serialization/archiveFrom_std_vector.h>

#include <cstdint>
#include <string>
#include <vector>

#include "serialization_test_types.h"

using mrpt::serialization::archiveFrom;
using mrpt::serialization::CMessage;

namespace
{
/** Round-trips a message through the binary framing of sendMessage(). */
void checkFrameRoundTrip(uint32_t type, const std::vector<uint8_t>& content)
{
  CMessage out;
  out.type = type;
  out.content = content;

  std::vector<uint8_t> v;
  auto a = archiveFrom(v);
  a.sendMessage(out);

  CMessage in;
  ASSERT_TRUE(a.receiveMessage(in));
  EXPECT_EQ(in.type, type);
  EXPECT_EQ(in.content, content);
}
}  // namespace

TEST(CMessage, tinyFrameRoundTrip) { checkFrameRoundTrip(7, {1, 2, 3, 4, 5}); }

TEST(CMessage, emptyFrameRoundTrip) { checkFrameRoundTrip(0, {}); }

TEST(CMessage, largestTinyFrameRoundTrip)
{
  // 255 bytes is the largest payload the 1-byte-length ("tiny") format holds:
  checkFrameRoundTrip(3, std::vector<uint8_t>(255, 0xAB));
}

TEST(CMessage, largeFrameRoundTrip)
{
  // 256 bytes and above switch to the 2-byte-length frame format:
  checkFrameRoundTrip(4, std::vector<uint8_t>(256, 0xCD));

  std::vector<uint8_t> big(5000);
  for (size_t i = 0; i < big.size(); i++)
  {
    big[i] = static_cast<uint8_t>(i & 0xff);
  }
  checkFrameRoundTrip(9, big);
}

TEST(CMessage, largestRepresentableFrameRoundTrip)
{
  // 65535 bytes is the largest payload the 16-bit length field can describe:
  checkFrameRoundTrip(1, std::vector<uint8_t>(0xffff, 0xEF));
}

TEST(CMessage, oversizedPayloadIsRejected)
{
  CMessage msg;
  msg.type = 1;
  msg.content.assign(0x10000, 0x01);  // one byte too long

  std::vector<uint8_t> v;
  auto a = archiveFrom(v);
  EXPECT_ANY_THROW(a.sendMessage(msg));
  EXPECT_TRUE(v.empty());
}

TEST(CMessage, receiveMessageFromEmptyStreamFails)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  CMessage msg;
  EXPECT_FALSE(a.receiveMessage(msg));
}

TEST(CMessage, receiveMessageRejectsBadStartFlag)
{
  // Three consecutive bytes that are neither 0x69 nor 0x79 exhaust the
  // re-synchronization attempts:
  std::vector<uint8_t> v = {0x00, 0x01, 0x02, 0x03};
  auto a = archiveFrom(v);

  CMessage msg;
  EXPECT_FALSE(a.receiveMessage(msg));
}

TEST(CMessage, receiveMessageRejectsBadEndFlag)
{
  std::vector<uint8_t> v;
  {
    CMessage out;
    out.type = 1;
    out.content = {1, 2, 3};
    auto a = archiveFrom(v);
    a.sendMessage(out);
  }
  ASSERT_FALSE(v.empty());
  v.back() = 0x00;  // corrupt the end flag

  auto a = archiveFrom(v);
  CMessage msg;
  EXPECT_FALSE(a.receiveMessage(msg));
}

TEST(CMessage, receiveMessageOnTruncatedFrameFails)
{
  std::vector<uint8_t> v;
  {
    CMessage out;
    out.type = 1;
    out.content = std::vector<uint8_t>(300, 0x11);
    auto a = archiveFrom(v);
    a.sendMessage(out);
  }
  v.resize(v.size() / 2);

  auto a = archiveFrom(v);
  CMessage msg;
  EXPECT_FALSE(a.receiveMessage(msg));
}

TEST(CMessage, contentAsString)
{
  CMessage msg;
  msg.setContentFromString("hello world");
  EXPECT_EQ(msg.content.size(), 11U);

  std::string s;
  msg.getContentAsString(s);
  EXPECT_EQ(s, "hello world");
}

TEST(CMessage, emptyContentAsString)
{
  CMessage msg;
  msg.setContentFromString("");
  EXPECT_TRUE(msg.content.empty());

  std::string s = "junk";
  msg.getContentAsString(s);
  EXPECT_TRUE(s.empty());
}

TEST(CMessage, serializeObjectIntoNewObject)
{
  SerTestNS::registerTestClasses();

  SerTestNS::Foo src;
  src.m_value = 4242;

  CMessage msg;
  msg.type = 1;
  msg.serializeObject(&src);
  EXPECT_FALSE(msg.content.empty());

  mrpt::serialization::CSerializable::Ptr obj;
  msg.deserializeIntoNewObject(obj);
  ASSERT_TRUE(obj);
  EXPECT_EQ(std::dynamic_pointer_cast<SerTestNS::Foo>(obj)->m_value, 4242U);
}

TEST(CMessage, deserializeIntoNewObjectWithEmptyContent)
{
  CMessage msg;
  auto obj = mrpt::serialization::CSerializable::Ptr(SerTestNS::Foo::Create());
  msg.deserializeIntoNewObject(obj);
  EXPECT_FALSE(obj);
}

TEST(CMessage, serializeObjectIntoExistingObject)
{
  SerTestNS::registerTestClasses();

  SerTestNS::Foo src;
  src.m_value = 17;

  CMessage msg;
  msg.serializeObject(&src);

  SerTestNS::Foo dst;
  msg.deserializeIntoExistingObject(&dst);
  EXPECT_EQ(dst.m_value, 17U);
}

TEST(CMessage, deserializeIntoExistingObjectOfWrongClassThrows)
{
  SerTestNS::registerTestClasses();

  SerTestNS::Foo src;
  CMessage msg;
  msg.serializeObject(&src);

  SerTestNS::Bar dst;
  EXPECT_ANY_THROW(msg.deserializeIntoExistingObject(&dst));
}
