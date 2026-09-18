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
#include <mrpt/img/TColor.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt::img;

TEST(TColor, ConstructAndConvert)
{
  const TColor c(10, 20, 30, 40);
  EXPECT_EQ(c.R, 10);
  EXPECT_EQ(c.G, 20);
  EXPECT_EQ(c.B, 30);
  EXPECT_EQ(c.A, 40);

  const unsigned int packed = static_cast<unsigned int>(c);
  EXPECT_EQ(packed, (10u << 16) | (20u << 8) | 30u);

  const TColor fromPacked(0x0A141E);
  EXPECT_EQ(fromPacked.R, 10);
  EXPECT_EQ(fromPacked.G, 20);
  EXPECT_EQ(fromPacked.B, 30);
  EXPECT_EQ(fromPacked.A, 255);

  const TColor fromPackedAlpha(0x0A141E, 7);
  EXPECT_EQ(fromPackedAlpha.A, 7);
}

TEST(TColor, PredefinedColors)
{
  EXPECT_EQ(TColor::red(), TColor(255, 0, 0));
  EXPECT_EQ(TColor::green(), TColor(0, 255, 0));
  EXPECT_EQ(TColor::blue(), TColor(0, 0, 255));
  EXPECT_EQ(TColor::black(), TColor(0, 0, 0));
  EXPECT_EQ(TColor::white(), TColor(255, 255, 255));
  EXPECT_EQ(TColor::gray(), TColor(127, 127, 127));
}

TEST(TColor, ArithmeticOperators)
{
  const TColor a(10, 20, 30, 40);
  const TColor b(1, 2, 3, 4);

  const TColor sum = a + b;
  EXPECT_EQ(sum, TColor(11, 22, 33, 44));

  const TColor diff = a - b;
  EXPECT_EQ(diff, TColor(9, 18, 27, 36));

  TColor c = a;
  c += b;
  EXPECT_EQ(c, sum);

  c = a;
  c -= b;
  EXPECT_EQ(c, diff);
}

TEST(TColor, EqualityOperators)
{
  const TColor a(1, 2, 3, 4);
  const TColor b(1, 2, 3, 4);
  const TColor c(1, 2, 3, 5);

  EXPECT_TRUE(a == b);
  EXPECT_FALSE(a == c);
  EXPECT_TRUE(a != c);
  EXPECT_FALSE(a != b);
}

TEST(TColor, TextStreaming)
{
  const TColor c(1, 2, 3, 4);
  std::ostringstream ss;
  ss << c;
  EXPECT_EQ(ss.str(), "RGBA=[1,2,3,4]");
}

TEST(TColor, BinaryStreamingRoundTrip)
{
  const TColor c(11, 22, 33, 44);

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << c;

  buf.Seek(0);
  TColor c2;
  arch >> c2;

  EXPECT_EQ(c, c2);
}

TEST(TColorf, ConstructAndConvert)
{
  const TColorf f(0.1f, 0.2f, 0.3f, 0.4f);
  EXPECT_FLOAT_EQ(f.R, 0.1f);
  EXPECT_FLOAT_EQ(f.G, 0.2f);
  EXPECT_FLOAT_EQ(f.B, 0.3f);
  EXPECT_FLOAT_EQ(f.A, 0.4f);

  const TColor c(25, 51, 76, 102);
  const TColorf fromColor(c);
  EXPECT_NEAR(fromColor.R, 25.0f / 255.0f, 1e-4f);
  EXPECT_NEAR(fromColor.G, 51.0f / 255.0f, 1e-4f);
  EXPECT_NEAR(fromColor.B, 76.0f / 255.0f, 1e-4f);
  EXPECT_NEAR(fromColor.A, 102.0f / 255.0f, 1e-4f);

  const TColor back = fromColor.asTColor();
  EXPECT_EQ(back.R, 25);
  EXPECT_EQ(back.G, 51);
  EXPECT_EQ(back.B, 76);
  EXPECT_EQ(back.A, 102);
}

TEST(TColorf, EqualityOperators)
{
  const TColorf a(0.1f, 0.2f, 0.3f, 0.4f);
  const TColorf b(0.1f, 0.2f, 0.3f, 0.4f);
  const TColorf c(0.1f, 0.2f, 0.3f, 0.5f);

  EXPECT_TRUE(a == b);
  EXPECT_FALSE(a == c);
  EXPECT_TRUE(a != c);
  EXPECT_FALSE(a != b);
}

TEST(TColorf, TextStreaming)
{
  const TColorf f(0.5, 0.25, 0.125, 1.0);
  std::ostringstream ss;
  ss << f;
  EXPECT_EQ(ss.str(), "RGBAf=[0.500000,0.250000,0.125000,1.000000]");
}

TEST(TColorf, BinaryStreamingRoundTrip)
{
  const TColorf f(0.1f, 0.2f, 0.3f, 0.4f);

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << f;

  buf.Seek(0);
  TColorf f2;
  arch >> f2;

  EXPECT_EQ(f, f2);
}
