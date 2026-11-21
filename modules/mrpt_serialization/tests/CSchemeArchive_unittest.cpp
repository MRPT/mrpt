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
#include <mrpt/containers/config.h>
#include <mrpt/serialization/CSchemeArchive.h>

#include <cmath>
#include <limits>
#include <sstream>
#include <string>

#include "serialization_test_types.h"

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

#if MRPT_HAS_FYAML  // parsing text needs libfyaml
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

TEST(CSchemeArchive, streamRoundTripKeepsTypes)
{
  auto arch = mrpt::serialization::archiveJSON();
  arch["numeric_str"] = std::string("123");
  arch["bool_str"] = std::string("yes");
  arch["big"] = static_cast<int64_t>(134397633498449645LL);
  arch["third"] = 1.0 / 3.0;
  arch["nan"] = std::numeric_limits<double>::quiet_NaN();
  arch["inf"] = std::numeric_limits<double>::infinity();

  std::stringstream ss;
  ss << arch;
  // Standard JSON layout:
  EXPECT_NE(ss.str().find("\"numeric_str\": \"123\""), std::string::npos) << ss.str();

  auto arch2 = mrpt::serialization::archiveJSON();
  ss >> arch2;
  EXPECT_EQ(static_cast<std::string>(arch2["numeric_str"]), "123");
  EXPECT_EQ(static_cast<std::string>(arch2["bool_str"]), "yes");
  EXPECT_EQ(static_cast<int64_t>(arch2["big"]), 134397633498449645LL);
  EXPECT_EQ(static_cast<double>(arch2["third"]), 1.0 / 3.0);
  EXPECT_TRUE(std::isnan(static_cast<double>(arch2["nan"])));
  EXPECT_EQ(static_cast<double>(arch2["inf"]), std::numeric_limits<double>::infinity());
}

TEST(CSchemeArchive, streamOfChildArchive)
{
  auto arch = mrpt::serialization::archiveJSON();
  arch["a"]["x"] = static_cast<int32_t>(4);
  arch["b"] = static_cast<int32_t>(5);

  std::stringstream ss;
  ss << arch["a"];
  auto arch2 = mrpt::serialization::archiveJSON();
  ss >> arch2;
  EXPECT_EQ(static_cast<int32_t>(arch2["x"]), 4);
  EXPECT_EQ(static_cast<int32_t>(arch2["b"]), 0);
}
#endif

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

TEST(CSchemeArchive, missingEntriesReadAsDefaults)
{
  auto arch = mrpt::serialization::archiveJSON();
  arch["present"] = static_cast<int32_t>(1);

  EXPECT_EQ(static_cast<int32_t>(arch["missing"]), 0);
  EXPECT_EQ(static_cast<double>(arch["missing"]["deeper"]), 0.0);
  EXPECT_FALSE(static_cast<bool>(arch["list"][static_cast<size_t>(3)]));
  EXPECT_EQ(static_cast<std::string>(arch["missing"]), "");
}

TEST(CSchemeArchive, childStaysValidWhenSiblingsAreAdded)
{
  auto arch = mrpt::serialization::archiveJSON();
  auto first = arch["first"];
  first["v"] = static_cast<int32_t>(1);
  // Many siblings, to force reallocations of the parent containers:
  for (int32_t i = 0; i < 100; i++)
  {
    arch["k" + std::to_string(i)] = i;
    arch["seq"][static_cast<size_t>(i)] = i;
  }
  first["w"] = static_cast<int32_t>(2);

  EXPECT_EQ(static_cast<int32_t>(arch["first"]["v"]), 1);
  EXPECT_EQ(static_cast<int32_t>(arch["first"]["w"]), 2);
  EXPECT_EQ(static_cast<int32_t>(arch["k99"]), 99);
  EXPECT_EQ(static_cast<int32_t>(arch["seq"][static_cast<size_t>(42)]), 42);
}

TEST(CSchemeArchive, movedArchiveStillWritesObjects)
{
  SerTestNS::registerTestClasses();

  SerTestNS::WithSchema src;
  src.m_int = 12;
  src.m_str = "moved";

  auto arch = mrpt::serialization::archiveJSON();
  mrpt::serialization::CSchemeArchiveBase moved(std::move(arch));
  moved = src;

  SerTestNS::WithSchema dst;
  moved.readTo(dst);
  EXPECT_EQ(dst.m_int, 12);
  EXPECT_EQ(dst.m_str, "moved");
}

TEST(CSchemeArchive, classesWithoutSchemaSupportThrow)
{
  SerTestNS::registerTestClasses();

  // The default implementations in CSerializable reject schema-based archives:
  SerTestNS::Foo obj;
  auto& asSerializable = static_cast<mrpt::serialization::CSerializable&>(obj);

  auto arch = mrpt::serialization::archiveJSON();
  EXPECT_ANY_THROW(arch = asSerializable);
  EXPECT_ANY_THROW(arch.readTo(asSerializable));
}
