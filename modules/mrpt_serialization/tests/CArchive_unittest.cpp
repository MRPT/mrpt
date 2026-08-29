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
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/serialization/aligned_serialization.h>
#include <mrpt/serialization/archiveFrom_std_streams.h>
#include <mrpt/serialization/archiveFrom_std_vector.h>

#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

#include "serialization_test_types.h"

using mrpt::serialization::archiveFrom;
using mrpt::serialization::CArchive;
using mrpt::serialization::CExceptionEOF;
using mrpt::serialization::CSerializable;

namespace
{
/** Appends the MRPT object-header bytes for the given class name (new format,
 * i.e. with the 0x80 marker in the length byte). */
void appendNewFormatHeader(std::vector<uint8_t>& v, const std::string& className)
{
  v.push_back(static_cast<uint8_t>(className.size() | 0x80));
  v.insert(v.end(), className.begin(), className.end());
}
}  // namespace

TEST(CArchive, podRoundTrip)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  const bool bo = true;
  const int8_t i8 = -8;
  const uint8_t u8 = 8;
  const int16_t i16 = -1600;
  const uint16_t u16 = 1600;
  const int32_t i32 = -320000;
  const uint32_t u32 = 320000;
  const int64_t i64 = -640000000000LL;
  const uint64_t u64 = 640000000000ULL;
  const float f = 1.5f;
  const double d = -2.25;
  const long double ld = 3.125L;

  a << bo << i8 << u8 << i16 << u16 << i32 << u32 << i64 << u64 << f << d << ld;

  bool r_bo = false;
  int8_t r_i8 = 0;
  uint8_t r_u8 = 0;
  int16_t r_i16 = 0;
  uint16_t r_u16 = 0;
  int32_t r_i32 = 0;
  uint32_t r_u32 = 0;
  int64_t r_i64 = 0;
  uint64_t r_u64 = 0;
  float r_f = 0;
  double r_d = 0;
  long double r_ld = 0;

  a >> r_bo >> r_i8 >> r_u8 >> r_i16 >> r_u16 >> r_i32 >> r_u32 >> r_i64 >> r_u64 >> r_f >> r_d >>
      r_ld;

  EXPECT_EQ(r_bo, bo);
  EXPECT_EQ(r_i8, i8);
  EXPECT_EQ(r_u8, u8);
  EXPECT_EQ(r_i16, i16);
  EXPECT_EQ(r_u16, u16);
  EXPECT_EQ(r_i32, i32);
  EXPECT_EQ(r_u32, u32);
  EXPECT_EQ(r_i64, i64);
  EXPECT_EQ(r_u64, u64);
  EXPECT_EQ(r_f, f);
  EXPECT_EQ(r_d, d);
  EXPECT_EQ(r_ld, ld);
}

TEST(CArchive, stringRoundTrip)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  a << std::string("hello") << std::string();

  std::string s1;
  std::string s2 = "not empty";
  a >> s1 >> s2;

  EXPECT_EQ(s1, "hello");
  EXPECT_TRUE(s2.empty());
}

TEST(CArchive, vectorBoolRoundTrip)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  const std::vector<bool> in = {true, false, true, true, false};
  const std::vector<bool> empty;
  a << in << empty;

  std::vector<bool> out;
  std::vector<bool> outEmpty = {true};
  a >> out >> outEmpty;

  EXPECT_EQ(out, in);
  EXPECT_TRUE(outEmpty.empty());
}

template <typename T>
void testNumericVectorRoundTrip(const std::vector<T>& in)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  const std::vector<T> empty;
  a << in << empty;

  std::vector<T> out;
  std::vector<T> outEmpty = in;
  a >> out >> outEmpty;

  EXPECT_EQ(out, in);
  EXPECT_TRUE(outEmpty.empty());
}

TEST(CArchive, numericVectorsRoundTrip)
{
  testNumericVectorRoundTrip<float>({1.0f, -2.5f, 3.75f});
  testNumericVectorRoundTrip<double>({1.0, -2.5, 3.75});
  testNumericVectorRoundTrip<int8_t>({1, -2, 3});
  testNumericVectorRoundTrip<uint8_t>({1, 2, 3});
  testNumericVectorRoundTrip<int16_t>({1, -2, 3});
  testNumericVectorRoundTrip<uint16_t>({1, 2, 3});
  testNumericVectorRoundTrip<int32_t>({1, -2, 3});
  testNumericVectorRoundTrip<uint32_t>({1, 2, 3});
  testNumericVectorRoundTrip<int64_t>({1, -2, 3});
  testNumericVectorRoundTrip<size_t>({1, 2, 3});
}

TEST(CArchive, alignedVectorRoundTrip)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  mrpt::aligned_std_vector<float> in = {1.0f, -2.5f, 3.75f};
  a << in;

  mrpt::aligned_std_vector<float> out;
  a >> out;

  ASSERT_EQ(out.size(), in.size());
  for (size_t i = 0; i < in.size(); i++)
  {
    EXPECT_EQ(out[i], in[i]);
  }
}

TEST(CArchive, vectorOfStringsRoundTrip)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  const std::vector<std::string> in = {"one", "", "three"};
  a << in;

  std::vector<std::string> out;
  a >> out;

  EXPECT_EQ(out, in);
}

TEST(CArchive, clockTimePointRoundTrip)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  const auto in = mrpt::Clock::now();
  a << in;

  mrpt::Clock::time_point out;
  a >> out;

  EXPECT_EQ(out, in);
}

TEST(CArchive, readAsAndWriteAs)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  a.WriteAs<uint16_t>(1234);
  a.WriteAs<int8_t>(-5);
  a << static_cast<uint32_t>(77);

  EXPECT_EQ(a.ReadAs<uint16_t>(), 1234);

  int32_t widened = 0;
  a.ReadAsAndCastTo<int8_t, int32_t>(widened);
  EXPECT_EQ(widened, -5);

  EXPECT_EQ(a.ReadPOD<uint32_t>(), 77U);
}

TEST(CArchive, macroReadPOD)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  a << static_cast<double>(3.5);

  double d = 0;
  MRPT_READ_POD(a, d);
  EXPECT_EQ(d, 3.5);
}

TEST(CArchive, zeroLengthBufferOperations)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  // A zero-length request must be a no-op on an otherwise empty stream:
  int dummy = 0;
  EXPECT_EQ(a.ReadBuffer(&dummy, 0), 0U);
  EXPECT_NO_THROW(a.WriteBuffer(&dummy, 0));
  EXPECT_TRUE(v.empty());
}

TEST(CArchive, nullBufferThrows)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  EXPECT_ANY_THROW(a.ReadBuffer(nullptr, 1));
  EXPECT_ANY_THROW(a.WriteBuffer(nullptr, 1));
}

TEST(CArchive, objectRoundTrip)
{
  SerTestNS::registerTestClasses();

  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  auto obj = SerTestNS::Foo::Create();
  obj->m_value = 0xDEADBEEF;
  a << *obj;

  auto read = a.ReadObject<SerTestNS::Foo>();
  ASSERT_TRUE(read);
  EXPECT_EQ(read->m_value, 0xDEADBEEFU);
}

TEST(CArchive, objectPtrRoundTrip)
{
  SerTestNS::registerTestClasses();

  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  CSerializable::Ptr obj = SerTestNS::Foo::Create();
  std::dynamic_pointer_cast<SerTestNS::Foo>(obj)->m_value = 7;
  a << obj;

  CSerializable::Ptr read;
  a >> read;
  ASSERT_TRUE(read);
  EXPECT_EQ(std::dynamic_pointer_cast<SerTestNS::Foo>(read)->m_value, 7U);
}

TEST(CArchive, nullObjectRoundTrip)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  const CSerializable::Ptr nullObj;
  a << nullObj;

  CSerializable::Ptr read = SerTestNS::Foo::Create();
  a >> read;
  EXPECT_FALSE(read);
}

TEST(CArchive, readObjectIntoExistingObject)
{
  SerTestNS::registerTestClasses();

  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  SerTestNS::Foo src;
  src.m_value = 99;
  a << src;

  SerTestNS::Foo dst;
  a >> dst;
  EXPECT_EQ(dst.m_value, 99U);
}

TEST(CArchive, readObjectIntoExistingObjectOfWrongClassThrows)
{
  SerTestNS::registerTestClasses();

  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  SerTestNS::Foo src;
  a << src;

  SerTestNS::Bar dst;
  EXPECT_ANY_THROW(a >> dst);
}

TEST(CArchive, readObjectIntoExistingObjectOfUnregisteredClassThrows)
{
  std::vector<uint8_t> v;
  appendNewFormatHeader(v, "SerTestNS::NotRegistered");
  v.push_back(0);  // version

  auto a = archiveFrom(v);

  SerTestNS::Foo dst;
  EXPECT_ANY_THROW(a >> dst);
}

TEST(CArchive, readObjectOfUnregisteredClassThrows)
{
  std::vector<uint8_t> v;
  appendNewFormatHeader(v, "SerTestNS::NotRegistered");
  v.push_back(0);  // version

  auto a = archiveFrom(v);
  EXPECT_ANY_THROW(a.ReadObject());
}

TEST(CArchive, readObjectWithCorruptedEndFlagThrows)
{
  SerTestNS::registerTestClasses();

  std::vector<uint8_t> v;
  {
    auto a = archiveFrom(v);
    SerTestNS::Foo src;
    a << src;
  }
  ASSERT_FALSE(v.empty());
  v.back() = 0x00;  // the end flag is the very last byte

  auto a = archiveFrom(v);
  EXPECT_ANY_THROW(a.ReadObject());
}

TEST(CArchive, readObjectWithTooLongClassNameThrows)
{
  std::vector<uint8_t> v;
  // 127 is the largest length representable, and above the 120-char sanity
  // limit that guards against corrupted streams:
  v.push_back(static_cast<uint8_t>(127 | 0x80));
  v.resize(v.size() + 127, 'x');

  auto a = archiveFrom(v);
  EXPECT_ANY_THROW(a.ReadObject());
}

TEST(CArchive, readObjectFromEmptyStreamThrowsEOF)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  EXPECT_THROW(a.ReadObject(), CExceptionEOF);
}

TEST(CArchive, readObjectTruncatedMidObjectThrowsPlainException)
{
  SerTestNS::registerTestClasses();

  std::vector<uint8_t> v;
  {
    auto a = archiveFrom(v);
    SerTestNS::Foo src;
    a << src;
  }
  // Cut the stream in the middle of the payload: an EOF at a *wrong* place is
  // reported as a plain exception, not as CExceptionEOF:
  v.resize(v.size() - 3);

  auto a = archiveFrom(v);
  EXPECT_ANY_THROW(a.ReadObject());
}

TEST(CArchive, readObjectInPreMRPT055Format)
{
  SerTestNS::registerTestClasses();

  // Streams older than MRPT 0.5.5 have no 0x80 marker in the class name length
  // byte, are followed by three 0x00 padding bytes, store the version as an
  // int32 and carry no end flag. The class name has no namespace either, which
  // is why findRegisteredClass() falls back to a namespace-less lookup.
  std::vector<uint8_t> v;
  v.push_back(3);  // strlen("Foo"), no 0x80 marker
  v.insert(v.end(), {0x00, 0x00, 0x00});
  v.insert(v.end(), {'F', 'o', 'o'});
  v.insert(v.end(), {0x00, 0x00, 0x00, 0x00});  // int32 version = 0
  v.insert(v.end(), {0x2A, 0x00, 0x00, 0x00});  // uint32 payload = 42

  auto a = archiveFrom(v);
  auto obj = a.ReadObject<SerTestNS::Foo>();
  ASSERT_TRUE(obj);
  EXPECT_EQ(obj->m_value, 42U);
}

TEST(CArchive, readObjectInPreMRPT055FormatWithBadPaddingThrows)
{
  std::vector<uint8_t> v;
  v.push_back(3);
  v.insert(v.end(), {0x00, 0x01, 0x00});  // non-zero padding: not a valid header
  v.insert(v.end(), {'F', 'o', 'o'});

  auto a = archiveFrom(v);
  EXPECT_ANY_THROW(a.ReadObject());
}

TEST(CArchive, variantRoundTrip)
{
  SerTestNS::registerTestClasses();

  using variant_t = std::variant<std::monostate, SerTestNS::Foo::Ptr>;

  {
    std::vector<uint8_t> v;
    auto a = archiveFrom(v);

    auto foo = SerTestNS::Foo::Create();
    foo->m_value = 11;
    const variant_t in = foo;
    a.WriteVariant(in);

    const auto out = a.ReadVariant<std::monostate, SerTestNS::Foo::Ptr>();
    ASSERT_TRUE(std::holds_alternative<SerTestNS::Foo::Ptr>(out));
    EXPECT_EQ(std::get<SerTestNS::Foo::Ptr>(out)->m_value, 11U);
  }
  {
    // A variant holding no value serializes as the "std::monostate" class:
    std::vector<uint8_t> v;
    auto a = archiveFrom(v);

    const variant_t in;
    a.WriteVariant(in);

    const auto out = a.ReadVariant<std::monostate, SerTestNS::Foo::Ptr>();
    EXPECT_TRUE(std::holds_alternative<std::monostate>(out));
  }
}

TEST(CArchive, variantOfUnregisteredClassThrows)
{
  std::vector<uint8_t> v;
  appendNewFormatHeader(v, "SerTestNS::NotRegistered");
  v.push_back(0);  // version

  auto a = archiveFrom(v);
  EXPECT_ANY_THROW((a.ReadVariant<std::monostate, SerTestNS::Foo::Ptr>()));
}

TEST(CArchive, objectToAndFromOctetVector)
{
  SerTestNS::registerTestClasses();

  SerTestNS::Foo src;
  src.m_value = 55;

  std::vector<uint8_t> data;
  mrpt::serialization::ObjectToOctetVector(&src, data);
  EXPECT_FALSE(data.empty());

  CSerializable::Ptr obj;
  mrpt::serialization::OctetVectorToObject(data, obj);
  ASSERT_TRUE(obj);
  EXPECT_EQ(std::dynamic_pointer_cast<SerTestNS::Foo>(obj)->m_value, 55U);
}

TEST(CArchive, octetVectorToObjectWithEmptyInput)
{
  CSerializable::Ptr obj = SerTestNS::Foo::Create();
  mrpt::serialization::OctetVectorToObject({}, obj);
  EXPECT_FALSE(obj);
}

TEST(CArchive, readOnlyVectorArchiveRejectsWrites)
{
  const std::vector<uint8_t> v = {1, 2, 3};
  auto a = archiveFrom(v);

  uint8_t b = 0;
  EXPECT_ANY_THROW(a.WriteBuffer(&b, 1));

  // ...but reading works, until it runs out of data:
  EXPECT_EQ(a.ReadBuffer(&b, 1), 1U);
  EXPECT_EQ(b, 1);
  uint8_t big[4] = {};
  EXPECT_ANY_THROW(a.ReadBuffer(big, 4));
}

TEST(CArchive, vectorArchiveReadPastEndThrows)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  a << static_cast<uint8_t>(1);

  uint8_t b = 0;
  EXPECT_EQ(a.ReadBuffer(&b, 1), 1U);
  EXPECT_ANY_THROW(a.ReadBuffer(&b, 1));
}

TEST(CArchive, stdIOStreamRoundTrip)
{
  SerTestNS::registerTestClasses();

  std::stringstream ss;
  auto a = mrpt::serialization::archiveFrom<std::iostream>(ss);

  SerTestNS::Foo src;
  src.m_value = 123;
  a << src;

  auto obj = a.ReadObject<SerTestNS::Foo>();
  ASSERT_TRUE(obj);
  EXPECT_EQ(obj->m_value, 123U);
}

TEST(CArchive, stdIStreamIsReadOnly)
{
  std::istringstream in("abcd");
  auto a = mrpt::serialization::archiveFrom<std::istream>(in);

  uint8_t b = 0;
  EXPECT_EQ(a.ReadBuffer(&b, 1), 1U);
  EXPECT_EQ(b, 'a');
  EXPECT_ANY_THROW(a.WriteBuffer(&b, 1));

  // Reading beyond the end reports zero bytes read, i.e. an EOF:
  uint8_t big[8] = {};
  EXPECT_ANY_THROW(a.ReadBuffer(big, 8));
}

TEST(CArchive, stdOStreamIsWriteOnly)
{
  std::ostringstream out;
  auto a = mrpt::serialization::archiveFrom<std::ostream>(out);

  const uint8_t b = 'z';
  EXPECT_NO_THROW(a.WriteBuffer(&b, 1));
  EXPECT_EQ(out.str(), "z");

  uint8_t r = 0;
  EXPECT_ANY_THROW(a.ReadBuffer(&r, 1));
}

TEST(CArchive, archivePtrAndUniquePtrFrom)
{
  std::stringstream ss;
  {
    auto a = mrpt::serialization::archivePtrFrom<std::iostream>(ss);
    ASSERT_TRUE(a);
    (*a) << static_cast<uint32_t>(5);
  }
  {
    auto a = mrpt::serialization::archiveUniquePtrFrom<std::iostream>(ss);
    ASSERT_TRUE(a);
    EXPECT_EQ(a->ReadAs<uint32_t>(), 5U);
  }
}

TEST(CArchive, defaultArchiveDescription)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);
  EXPECT_EQ(a.getArchiveDescription(), "generic CArchive");
}

TEST(CArchive, enumRoundTrip)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  a << SerTestNS::TestEnum::Second;

  auto out = SerTestNS::TestEnum::First;
  a >> out;
  EXPECT_EQ(out, SerTestNS::TestEnum::Second);
}

TEST(CArchive, sharedPtrToNonSerializableRoundTrip)
{
  {
    std::vector<uint8_t> v;
    auto a = archiveFrom(v);

    const auto in = std::make_shared<double>(2.5);
    a << in;

    std::shared_ptr<double> out;
    a >> out;
    ASSERT_TRUE(out);
    EXPECT_EQ(*out, 2.5);
  }
  {
    // A null pointer round-trips as such:
    std::vector<uint8_t> v;
    auto a = archiveFrom(v);

    const std::shared_ptr<double> in;
    a << in;

    auto out = std::make_shared<double>(1.0);
    a >> out;
    EXPECT_FALSE(out);
  }
  {
    // ...and a type mismatch is caught:
    std::vector<uint8_t> v;
    auto a = archiveFrom(v);

    a << std::make_shared<double>(2.5);

    std::shared_ptr<float> out;
    EXPECT_ANY_THROW(a >> out);
  }
}
