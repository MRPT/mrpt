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
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/math/CVectorFixed.h>
#include <mrpt/math/ops_vectors.h>
#include <mrpt/serialization/CArchive.h>

#include <sstream>
#include <vector>

using namespace mrpt::math;

TEST(ops_vectors, elementWiseOperators)
{
  std::vector<double> a{1.0, 2.0, 3.0};
  const std::vector<double> b{10.0, 20.0, 30.0};

  EXPECT_EQ(a * b, (std::vector<double>{10.0, 40.0, 90.0}));
  EXPECT_EQ(a + b, (std::vector<double>{11.0, 22.0, 33.0}));
  EXPECT_EQ(b - a, (std::vector<double>{9.0, 18.0, 27.0}));

  a *= b;
  EXPECT_EQ(a, (std::vector<double>{10.0, 40.0, 90.0}));
  a += b;
  EXPECT_EQ(a, (std::vector<double>{20.0, 60.0, 120.0}));
  a *= 0.5;
  EXPECT_EQ(a, (std::vector<double>{10.0, 30.0, 60.0}));
  a += 1.0;
  EXPECT_EQ(a, (std::vector<double>{11.0, 31.0, 61.0}));

  // Mismatched sizes are rejected:
  const std::vector<double> shorter{1.0};
  EXPECT_THROW(a *= shorter, std::exception);
  EXPECT_THROW(a += shorter, std::exception);
  EXPECT_THROW((void)(a * shorter), std::exception);
  EXPECT_THROW((void)(a + shorter), std::exception);
  EXPECT_THROW((void)(a - shorter), std::exception);
}

TEST(ops_vectors, streamPrinting)
{
  std::vector<double> v{1.0, 2.5};

  std::stringstream ss;
  ss << v;
  EXPECT_EQ(ss.str(), "[1.0000 2.5000 ]");
  // The stream flags/precision must be restored after printing:
  EXPECT_EQ(ss.precision(), std::stringstream().precision());

  std::stringstream ss2;
  ss2 << &v;
  EXPECT_EQ(ss2.str(), "[1.0000 2.5000 ]");
}

TEST(ops_vectors, cvectorFixedSerialization)
{
  CVectorFixedDouble<3> v;
  v[0] = 1.0;
  v[1] = 2.0;
  v[2] = 3.0;

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << v;

  buf.Seek(0);
  CVectorFixedDouble<3> v2;
  arch >> v2;
  EXPECT_NEAR(v2[0], 1.0, 1e-12);
  EXPECT_NEAR(v2[2], 3.0, 1e-12);

  // A type-name mismatch must be detected:
  buf.Seek(0);
  CVectorFixedFloat<3> vf;
  EXPECT_THROW(arch >> vf, std::exception);
}
