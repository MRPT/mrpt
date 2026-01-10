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
#include <mrpt/containers/find_closest.h>
#include <mrpt/core/common.h>

#include <map>

TEST(find_closest, testStdMap)
{
  {
    const std::map<float, char> emptyData = {};

    auto ret = mrpt::containers::find_closest(emptyData, 1.0);
    EXPECT_FALSE(ret.has_value());
  }

  const std::map<double, char> data = {
      {  1.0, 'A'},
      {  2.0, 'B'},
      {  3.0, 'C'},
      {100.0, 'Z'}
  };

  {
    auto ret = mrpt::containers::find_closest(data, 1.0);
    EXPECT_TRUE(ret.has_value());
    EXPECT_EQ(ret.value().second, 'A');
  }
  {
    auto ret = mrpt::containers::find_closest(data, 1.4f);
    EXPECT_TRUE(ret.has_value());
    EXPECT_EQ(ret.value().second, 'A');
  }
  EXPECT_EQ(mrpt::containers::find_closest(data, -100.0).value().second, 'A');
  EXPECT_EQ(mrpt::containers::find_closest(data, 2.1).value().second, 'B');
  EXPECT_EQ(mrpt::containers::find_closest(data, 2.45).value().second, 'B');
  EXPECT_EQ(mrpt::containers::find_closest(data, 2.55).value().second, 'C');
  EXPECT_EQ(mrpt::containers::find_closest(data, 90).value().second, 'Z');
  EXPECT_EQ(mrpt::containers::find_closest(data, 1190).value().second, 'Z');
}

TEST(find_closest, testStdMultiMap)
{
  const std::multimap<double, char> data = {
      {  1.0, 'A'},
      {  2.0, 'B'},
      {  3.0, 'C'},
      {100.0, 'Z'}
  };

  {
    auto ret = mrpt::containers::find_closest(data, 1.0);
    EXPECT_TRUE(ret.has_value());
    EXPECT_EQ(ret.value().second, 'A');
  }
  {
    auto ret = mrpt::containers::find_closest(data, 1.4);
    EXPECT_TRUE(ret.has_value());
    EXPECT_EQ(ret.value().second, 'A');
  }
  EXPECT_EQ(mrpt::containers::find_closest(data, -100.0).value().second, 'A');
  EXPECT_EQ(mrpt::containers::find_closest(data, 2.1).value().second, 'B');
  EXPECT_EQ(mrpt::containers::find_closest(data, 2.45).value().second, 'B');
  EXPECT_EQ(mrpt::containers::find_closest(data, 2.55).value().second, 'C');
  EXPECT_EQ(mrpt::containers::find_closest(data, 90).value().second, 'Z');
  EXPECT_EQ(mrpt::containers::find_closest(data, 1190).value().second, 'Z');
}

TEST(find_closest_with_tolerance, testStdMap)
{
  {
    const std::map<double, char> emptyData = {};

    auto ret = mrpt::containers::find_closest_with_tolerance(emptyData, 1.0, 0.1);
    EXPECT_FALSE(ret.has_value());
  }

  const std::map<double, char> data = {
      {  1.0, 'A'},
      {  2.0, 'B'},
      {  3.0, 'C'},
      {100.0, 'Z'}
  };

  {
    auto ret = mrpt::containers::find_closest_with_tolerance(data, 1.0, 0.1);
    EXPECT_TRUE(ret.has_value());
    EXPECT_EQ(ret.value().second, 'A');
  }

  EXPECT_FALSE(mrpt::containers::find_closest_with_tolerance(data, 0.9, 0.09).has_value());
  EXPECT_EQ(mrpt::containers::find_closest_with_tolerance(data, 0.9, 0.11).value().second, 'A');

  EXPECT_EQ(mrpt::containers::find_closest_with_tolerance(data, 1.0, 0.0).value().second, 'A');

  EXPECT_EQ(mrpt::containers::find_closest_with_tolerance(data, 2.4, 0.5).value().second, 'B');
  EXPECT_EQ(mrpt::containers::find_closest_with_tolerance(data, 2.6, 0.5).value().second, 'C');
  EXPECT_FALSE(mrpt::containers::find_closest_with_tolerance(data, 2.6, 0.3).has_value());

  EXPECT_FALSE(mrpt::containers::find_closest_with_tolerance(data, 200, 10).has_value());
  EXPECT_EQ(mrpt::containers::find_closest_with_tolerance(data, 200, 110).value().second, 'Z');
}
