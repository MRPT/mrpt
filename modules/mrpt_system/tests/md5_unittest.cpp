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

/** MD5 is specified with published test vectors (RFC 1321, appendix A.5), so
 *  the digests below are checked against those rather than against whatever
 *  this implementation happens to produce.
 */

#include <gtest/gtest.h>
#include <mrpt/system/md5.h>

#include <string>
#include <vector>

using mrpt::system::md5;

TEST(md5, rfc1321_test_suite)
{
  EXPECT_EQ(md5(std::string("")), "d41d8cd98f00b204e9800998ecf8427e");
  EXPECT_EQ(md5(std::string("a")), "0cc175b9c0f1b6a831c399e269772661");
  EXPECT_EQ(md5(std::string("abc")), "900150983cd24fb0d6963f7d28e17f72");
  EXPECT_EQ(md5(std::string("message digest")), "f96b697d7cb7938d525a2f31aaf161d0");
  EXPECT_EQ(md5(std::string("abcdefghijklmnopqrstuvwxyz")), "c3fcd3d76192e4007dfb496cca67e13b");
  EXPECT_EQ(
      md5(std::string("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789")),
      "d174ab98d277d9f5a5611c2c9f419d9f");
  EXPECT_EQ(
      md5(std::string("12345678901234567890123456789012345678901234567890"
                      "123456789012345678901234567890")),
      "57edf4a22be3c955ac49da2e2107b67a");
}

TEST(md5, digest_is_32_lowercase_hex_chars)
{
  const std::string d = md5(std::string("anything"));
  ASSERT_EQ(d.size(), 32U);
  for (char c : d)
  {
    EXPECT_TRUE((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f')) << "bad char: " << c;
  }
}

TEST(md5, the_three_overloads_agree)
{
  const std::string s = "The quick brown fox jumps over the lazy dog";
  const std::vector<uint8_t> v(s.begin(), s.end());

  const std::string expected = "9e107d9d372bb6826bd81d3542a419d6";
  EXPECT_EQ(md5(s), expected);
  EXPECT_EQ(md5(v), expected);
  EXPECT_EQ(md5(reinterpret_cast<const unsigned char*>(s.data()), s.size()), expected);
}

TEST(md5, empty_inputs_of_every_overload)
{
  const std::string expected = "d41d8cd98f00b204e9800998ecf8427e";
  EXPECT_EQ(md5(std::string("")), expected);
  EXPECT_EQ(md5(std::vector<uint8_t>()), expected);
  // The raw-buffer overload documents a non-null precondition instead:
  EXPECT_ANY_THROW(md5(nullptr, 0));
}

TEST(md5, embedded_nulls_are_hashed_not_truncated)
{
  std::string a("ab", 2);
  std::string b("a\0b", 3);
  ASSERT_EQ(b.size(), 3U);
  EXPECT_NE(md5(a), md5(b));

  // ... and the byte-buffer overload agrees with the std::string one:
  EXPECT_EQ(md5(reinterpret_cast<const unsigned char*>(b.data()), b.size()), md5(b));
}

TEST(md5, block_boundary_lengths)
{
  // MD5 pads in 64-byte blocks with an 8-byte length trailer, so 55/56/64 are
  // the interesting lengths.
  for (size_t n : {54U, 55U, 56U, 57U, 63U, 64U, 65U, 119U, 120U, 128U})
  {
    const std::string s(n, 'x');
    const std::string d = md5(s);
    EXPECT_EQ(d.size(), 32U) << "n=" << n;
  }

  // Known digests for two of those boundaries:
  EXPECT_EQ(md5(std::string(55, 'a')), "ef1772b6dff9a122358552954ad0df65");
  EXPECT_EQ(md5(std::string(56, 'a')), "3b0c8ac703f828b04c6c197006d17218");
  EXPECT_EQ(md5(std::string(64, 'a')), "014842d480b571495a4a0363793f7367");
}

TEST(md5, binary_data_of_every_byte_value)
{
  std::vector<uint8_t> all(256);
  for (size_t i = 0; i < all.size(); i++)
  {
    all[i] = static_cast<uint8_t>(i);
  }
  const std::string d = md5(all);
  EXPECT_EQ(d.size(), 32U);
  // Same bytes via the pointer overload:
  EXPECT_EQ(md5(all.data(), all.size()), d);
}

TEST(md5, differs_for_a_single_flipped_bit)
{
  const std::string a(100, 'a');
  std::string b = a;
  b[50] = 'b';
  EXPECT_NE(md5(a), md5(b));
}

TEST(md5, is_deterministic)
{
  const std::string s = "repeat me";
  EXPECT_EQ(md5(s), md5(s));
}
