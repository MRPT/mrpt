/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <gtest/gtest.h>
#include <mrpt/containers/ts_hash_map.h>

template <typename T>
void simple_test_hash_string()
{
  T h1, h2;
  mrpt::containers::reduced_hash("prueba1", h1);
  mrpt::containers::reduced_hash("prueba2", h2);
  EXPECT_NE(h1, h2);
}

TEST(ts_hash_map, string_hash_u8) { simple_test_hash_string<uint8_t>(); }
TEST(ts_hash_map, string_hash_u16) { simple_test_hash_string<uint16_t>(); }
TEST(ts_hash_map, string_hash_u32) { simple_test_hash_string<uint32_t>(); }
TEST(ts_hash_map, string_hash_u64) { simple_test_hash_string<uint64_t>(); }
TEST(ts_hash_map, stdstring_key)
{
  mrpt::containers::ts_hash_map<std::string, double> m;

  EXPECT_TRUE(m.empty());

  m["numero"] = 2.3;
  EXPECT_FALSE(m.empty());
  m.clear();
  EXPECT_TRUE(m.empty());

  m["uno"] = 1.0;
  m["dos"] = 2.0;
  m["tres"] = 3.0;

  EXPECT_EQ(1.0, m["uno"]);
  EXPECT_EQ(2.0, m["dos"]);
  EXPECT_EQ(3.0, m["tres"]);

  m["tres"]++;
  EXPECT_EQ(4.0, m["tres"]) << "Fail after ++ operator applied to reference [].";

  double num = .0;
  for (const auto& e : m)
  {
    num += e.second;
  }
  EXPECT_NEAR(num, 7.0, 1e-10) << "Fail after visiting and summing all entries";

  {
    const auto& it = m.find("pepe");
    EXPECT_TRUE(it == m.end());
  }

  {
    const auto& it = m.find("uno");
    EXPECT_TRUE(it->second == 1.0);
  }
}
