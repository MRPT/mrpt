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

TEST(ts_hash_map, visitAllEntries)
{
  mrpt::containers::ts_hash_map<std::string, double> m;
  m["uno"] = 1.0;
  m["dos"] = 2.0;
  m["tres"] = 3.0;

  // const visitor: sum values and count used entries.
  double sum = .0;
  size_t count = 0;
  const auto& cm = m;
  cm.visitAllEntries(
      [&](const auto& e)
      {
        sum += e.second;
        count++;
      });
  EXPECT_EQ(count, 3U);
  EXPECT_NEAR(sum, 6.0, 1e-12);

  // non-const visitor: mutate in place.
  m.visitAllEntries([](auto& e) { e.second *= 10.0; });
  EXPECT_EQ(10.0, m["uno"]);
  EXPECT_EQ(20.0, m["dos"]);
  EXPECT_EQ(30.0, m["tres"]);
}

TEST(ts_hash_map, find_or_alloc_string_view)
{
  mrpt::containers::ts_hash_map<std::string, double> m;
  m["uno"] = 1.0;

  // Look up an existing key via std::string_view: must resolve to the SAME
  // slot inserted through operator[] (i.e. reduced_hash()/keys_equal() must
  // agree between std::string and std::string_view for identical content).
  const std::string_view sv = "uno";
  double* p = m.find_or_alloc(sv);
  ASSERT_NE(p, nullptr);
  EXPECT_EQ(*p, 1.0);
  *p = 5.0;
  EXPECT_EQ(m["uno"], 5.0) << "string_view lookup did not alias the string key";

  // A new key via string_view must allocate a new entry:
  double* q = m.find_or_alloc(std::string_view("dos"));
  ASSERT_NE(q, nullptr);
  *q = 2.0;
  EXPECT_EQ(m["dos"], 2.0);
}

TEST(ts_hash_map, selfAssignment)
{
  // With a non-recursive mutex, an unguarded self-assignment would deadlock.
  mrpt::containers::ts_hash_map<std::string, double> m;
  m["uno"] = 1.0;
  auto& ref = m;
  m = ref;             // copy self-assignment
  m = std::move(ref);  // move self-assignment
  EXPECT_EQ(m["uno"], 1.0);
}

TEST(ts_hash_map, tooManyCollisionsThrows)
{
  // These 6 keys were found to all reduce to the same uint8_t hash bucket
  // under the current reduced_hash() (dbj2-based) implementation; verify
  // that assumption still holds before relying on it, so a future change to
  // reduced_hash() fails this assertion instead of silently not exercising
  // the "too many collisions" path:
  const char* collidingKeys[] = {"k0", "k197", "k607", "k848", "k925", "k1245"};
  uint8_t firstHash = 0;
  mrpt::containers::reduced_hash(collidingKeys[0], firstHash);
  for (const char* key : collidingKeys)
  {
    uint8_t hash = 0;
    mrpt::containers::reduced_hash(key, hash);
    ASSERT_EQ(hash, firstHash);
  }

  // This exceeds the default of 5 allowed collisions per hash bucket:
  mrpt::containers::ts_hash_map<std::string, double> m;
  for (int i = 0; i < 5; i++)
  {
    m[collidingKeys[i]] = static_cast<double>(i);
  }
  EXPECT_THROW(m[collidingKeys[5]] = 0.0, std::runtime_error);
}
