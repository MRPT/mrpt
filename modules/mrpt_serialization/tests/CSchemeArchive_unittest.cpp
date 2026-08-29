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
#include <mrpt/serialization/CSchemeArchive.h>
#include <mrpt/serialization/config.h>

#include <sstream>
#include <string>

#include "serialization_test_types.h"

#if MRPT_HAS_JSONCPP

TEST(CSchemeArchive, scalarAssignmentAndConversion)
{
  auto arch = mrpt::serialization::archiveJSON();

  arch["i32"] = static_cast<int32_t>(-5);
  arch["u32"] = static_cast<uint32_t>(5);
  arch["i64"] = static_cast<int64_t>(-6000000000LL);
  arch["u64"] = static_cast<uint64_t>(6000000000ULL);
  arch["f"] = 1.5f;
  arch["d"] = 2.5;
  arch["s"] = std::string("text");
  arch["b"] = true;

  EXPECT_EQ(static_cast<int32_t>(arch["i32"]), -5);
  EXPECT_EQ(static_cast<uint32_t>(arch["u32"]), 5U);
  EXPECT_EQ(static_cast<int64_t>(arch["i64"]), -6000000000LL);
  EXPECT_EQ(static_cast<uint64_t>(arch["u64"]), 6000000000ULL);
  EXPECT_EQ(static_cast<float>(arch["f"]), 1.5f);
  EXPECT_EQ(static_cast<double>(arch["d"]), 2.5);
  EXPECT_EQ(static_cast<std::string>(arch["s"]), "text");
  EXPECT_TRUE(static_cast<bool>(arch["b"]));
}

TEST(CSchemeArchive, listAccessor)
{
  auto arch = mrpt::serialization::archiveJSON();

  arch["list"][static_cast<size_t>(0)] = static_cast<int32_t>(10);
  arch["list"][static_cast<size_t>(1)] = static_cast<int32_t>(20);

  EXPECT_EQ(static_cast<int32_t>(arch["list"][static_cast<size_t>(0)]), 10);
  EXPECT_EQ(static_cast<int32_t>(arch["list"][static_cast<size_t>(1)]), 20);
}

TEST(CSchemeArchive, objectRoundTrip)
{
  SerTestNS::registerTestClasses();

  SerTestNS::WithSchema src;
  src.m_int = 33;
  src.m_str = "hello";

  auto arch = mrpt::serialization::archiveJSON();
  arch = src;

  SerTestNS::WithSchema dst;
  arch.readTo(dst);

  EXPECT_EQ(dst.m_int, 33);
  EXPECT_EQ(dst.m_str, "hello");
}

TEST(CSchemeArchive, streamRoundTrip)
{
  SerTestNS::registerTestClasses();

  SerTestNS::WithSchema src;
  src.m_int = -7;
  src.m_str = "streamed";

  std::stringstream ss;
  {
    auto arch = mrpt::serialization::archiveJSON();
    arch = src;
    ss << arch;
  }
  EXPECT_FALSE(ss.str().empty());

  SerTestNS::WithSchema dst;
  {
    auto arch = mrpt::serialization::archiveJSON();
    ss >> arch;
    arch.readTo(dst);
  }
  EXPECT_EQ(dst.m_int, -7);
  EXPECT_EQ(dst.m_str, "streamed");
}

TEST(CSchemeArchive, readingIntoAnotherClassThrows)
{
  SerTestNS::registerTestClasses();

  SerTestNS::WithSchema src;
  auto arch = mrpt::serialization::archiveJSON();
  arch = src;

  // The "datatype" field pins the class the document was written from:
  SerTestNS::Foo dst;
  EXPECT_ANY_THROW(arch.readTo(dst));
}

#endif  // MRPT_HAS_JSONCPP

TEST(CSchemeArchive, classesWithoutSchemaSupportThrow)
{
  SerTestNS::registerTestClasses();

  // The default implementations in CSerializable reject schema-based archives:
  SerTestNS::Foo obj;
  auto& asSerializable = static_cast<mrpt::serialization::CSerializable&>(obj);

#if MRPT_HAS_JSONCPP
  auto arch = mrpt::serialization::archiveJSON();
  EXPECT_ANY_THROW(arch = asSerializable);
  EXPECT_ANY_THROW(arch.readTo(asSerializable));
#else
  (void)asSerializable;
  EXPECT_ANY_THROW(mrpt::serialization::archiveJSON());
#endif
}
