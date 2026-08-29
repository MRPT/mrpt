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
#include <mrpt/serialization/archiveFrom_std_vector.h>
#include <mrpt/serialization/optional_serialization.h>
#include <mrpt/serialization/stl_serialization.h>

#include <array>
#include <cstdint>
#include <deque>
#include <list>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

using mrpt::serialization::archiveFrom;

TEST(stl_serialization, dequeRoundTrip)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  const std::deque<int32_t> in = {1, -2, 3};
  a << in;

  std::deque<int32_t> out = {99};
  a >> out;
  EXPECT_EQ(out, in);
}

TEST(stl_serialization, listRoundTrip)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  const std::list<double> in = {1.5, -2.5};
  const std::list<double> empty;
  a << in << empty;

  std::list<double> out;
  std::list<double> outEmpty = {1.0};
  a >> out >> outEmpty;
  EXPECT_EQ(out, in);
  EXPECT_TRUE(outEmpty.empty());
}

TEST(stl_serialization, setRoundTrip)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  const std::set<int32_t> in = {3, 1, 2};
  a << in;

  std::set<int32_t> out = {99};
  a >> out;
  EXPECT_EQ(out, in);
}

TEST(stl_serialization, multisetRoundTrip)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  const std::multiset<int32_t> in = {1, 1, 2};
  a << in;

  std::multiset<int32_t> out;
  a >> out;
  EXPECT_EQ(out, in);
}

TEST(stl_serialization, mapRoundTrip)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  const std::map<int32_t, double> in = {
      {1,  1.5},
      {2, -2.5}
  };
  a << in;

  std::map<int32_t, double> out = {
      {99, 0.0}
  };
  a >> out;
  EXPECT_EQ(out, in);
}

TEST(stl_serialization, multimapRoundTrip)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  const std::multimap<int32_t, double> in = {
      {1,  1.5},
      {1,  2.5},
      {2, -2.5}
  };
  a << in;

  std::multimap<int32_t, double> out;
  a >> out;
  EXPECT_EQ(out, in);
}

TEST(stl_serialization, arrayRoundTrip)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  const std::array<double, 3> in = {1.0, 2.0, 3.0};
  a << in;

  std::array<double, 3> out = {};
  a >> out;
  EXPECT_EQ(out, in);
}

TEST(stl_serialization, pairRoundTrip)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  const std::pair<int32_t, std::string> in{7, "seven"};
  a << in;

  std::pair<int32_t, std::string> out;
  a >> out;
  EXPECT_EQ(out, in);
}

TEST(stl_serialization, optionalRoundTrip)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  const std::optional<double> withValue = 3.5;
  const std::optional<double> without;
  a << withValue << without;

  std::optional<double> out1;
  std::optional<double> out2 = 1.0;
  a >> out1 >> out2;

  ASSERT_TRUE(out1.has_value());
  EXPECT_EQ(*out1, 3.5);
  EXPECT_FALSE(out2.has_value());
}

TEST(stl_serialization, sequenceContainerWrongPreambleThrows)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  const std::list<double> in = {1.0};
  a << in;

  // Same element type, different container: the preamble no longer matches.
  std::deque<double> out;
  EXPECT_ANY_THROW(a >> out);
}

TEST(stl_serialization, sequenceContainerWrongElementTypeThrows)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  const std::deque<double> in = {1.0};
  a << in;

  std::deque<int32_t> out;
  EXPECT_ANY_THROW(a >> out);
}

TEST(stl_serialization, simpleAssocContainerWrongPreambleThrows)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  const std::multiset<int32_t> in = {1};
  a << in;

  std::set<int32_t> out;
  EXPECT_ANY_THROW(a >> out);
}

TEST(stl_serialization, simpleAssocContainerWrongKeyTypeThrows)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  const std::set<int32_t> in = {1};
  a << in;

  std::set<double> out;
  EXPECT_ANY_THROW(a >> out);
}

TEST(stl_serialization, mapWrongPreambleThrows)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  const std::multimap<int32_t, double> in = {
      {1, 1.0}
  };
  a << in;

  std::map<int32_t, double> out;
  EXPECT_ANY_THROW(a >> out);
}

TEST(stl_serialization, mapWrongKeyTypeThrows)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  const std::map<int32_t, double> in = {
      {1, 1.0}
  };
  a << in;

  std::map<int64_t, double> out;
  EXPECT_ANY_THROW(a >> out);
}

TEST(stl_serialization, mapWrongValueTypeThrows)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  const std::map<int32_t, double> in = {
      {1, 1.0}
  };
  a << in;

  std::map<int32_t, float> out;
  EXPECT_ANY_THROW(a >> out);
}

TEST(stl_serialization, arrayWrongLengthThrows)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  const std::array<double, 3> in = {1.0, 2.0, 3.0};
  a << in;

  std::array<double, 2> out = {};
  EXPECT_ANY_THROW(a >> out);
}

TEST(stl_serialization, arrayWrongElementTypeThrows)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  const std::array<double, 2> in = {1.0, 2.0};
  a << in;

  std::array<float, 2> out = {};
  EXPECT_ANY_THROW(a >> out);
}

TEST(stl_serialization, pairWrongPreambleThrows)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  const std::set<int32_t> in = {1};
  a << in;

  std::pair<int32_t, int32_t> out;
  EXPECT_ANY_THROW(a >> out);
}

TEST(stl_serialization, pairWrongFirstTypeThrows)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  const std::pair<int32_t, double> in{1, 2.0};
  a << in;

  std::pair<int64_t, double> out;
  EXPECT_ANY_THROW(a >> out);
}

TEST(stl_serialization, pairWrongSecondTypeThrows)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  const std::pair<int32_t, double> in{1, 2.0};
  a << in;

  std::pair<int32_t, float> out;
  EXPECT_ANY_THROW(a >> out);
}

TEST(stl_serialization, optionalWrongPreambleThrows)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  const std::set<int32_t> in = {1};
  a << in;

  std::optional<int32_t> out;
  EXPECT_ANY_THROW(a >> out);
}

TEST(stl_serialization, optionalWrongValueTypeThrows)
{
  std::vector<uint8_t> v;
  auto a = archiveFrom(v);

  const std::optional<double> in = 1.0;
  a << in;

  std::optional<float> out;
  EXPECT_ANY_THROW(a >> out);
}
