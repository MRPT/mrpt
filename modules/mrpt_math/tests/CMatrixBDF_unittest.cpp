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
#include <mrpt/math/CMatrixB.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixF.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/config.h>

using namespace mrpt::math;

namespace
{
template <typename T>
T roundTripSerialize(const T& obj)
{
  mrpt::io::CMemoryStream membuf;
  auto arch = mrpt::serialization::archiveFrom(membuf);
  arch << obj;
  membuf.Seek(0);
  T obj2;
  arch >> obj2;
  return obj2;
}
}  // namespace

TEST(CMatrixB, SerializationRoundTrip)
{
  CMatrixB m(2, 3);
  m(0, 0) = true;
  m(0, 1) = false;
  m(0, 2) = true;
  m(1, 0) = false;
  m(1, 1) = true;
  m(1, 2) = false;

  const CMatrixB m2 = roundTripSerialize(m);
  ASSERT_EQ(m2.rows(), 2);
  ASSERT_EQ(m2.cols(), 3);
  for (int r = 0; r < 2; r++)
  {
    for (int c = 0; c < 3; c++)
    {
      EXPECT_EQ(m(r, c), m2(r, c));
    }
  }
}

TEST(CMatrixB, SerializationEmptyMatrix)
{
  CMatrixB m(0, 0);
  const CMatrixB m2 = roundTripSerialize(m);
  EXPECT_EQ(m2.rows(), 0);
  EXPECT_EQ(m2.cols(), 0);
}

TEST(CMatrixD, SerializationRoundTrip)
{
  CMatrixD m(2, 2);
  m(0, 0) = 1.0;
  m(0, 1) = 2.0;
  m(1, 0) = 3.0;
  m(1, 1) = 4.0;

  const CMatrixD m2 = roundTripSerialize(m);
  ASSERT_EQ(m2.rows(), 2);
  ASSERT_EQ(m2.cols(), 2);
  EXPECT_NEAR(m(0, 0), m2(0, 0), 1e-12);
  EXPECT_NEAR(m(1, 1), m2(1, 1), 1e-12);
}

TEST(CMatrixD, CopyFromFloatMatrix)
{
  CMatrixFloat mf(2, 2);
  mf(0, 0) = 1.0f;
  mf(0, 1) = 2.0f;
  mf(1, 0) = 3.0f;
  mf(1, 1) = 4.0f;

  CMatrixD md(mf);
  EXPECT_NEAR(md(0, 0), 1.0, 1e-6);
  EXPECT_NEAR(md(1, 1), 4.0, 1e-6);
}

#if MRPT_HAS_JSONCPP
#include <mrpt/serialization/CSchemeArchive.h>
#include <mrpt/serialization/CSchemeArchiveBase.h>

TEST(CMatrixD, SchemaSerializationRoundTrip)
{
  CMatrixD m(2, 2);
  m(0, 0) = 1.0;
  m(0, 1) = 2.0;
  m(1, 0) = 3.0;
  m(1, 1) = 4.0;

  auto arch = mrpt::serialization::archiveJSON();
  arch = m;
  std::stringstream ss;
  ss << arch;

  ss.seekg(0);
  auto arch2 = mrpt::serialization::archiveJSON();
  ss >> arch2;
  CMatrixD m2;
  arch2.readTo(m2);

  ASSERT_EQ(m2.rows(), 2);
  ASSERT_EQ(m2.cols(), 2);
  EXPECT_NEAR(m(0, 0), m2(0, 0), 1e-9);
  EXPECT_NEAR(m(1, 1), m2(1, 1), 1e-9);
}

TEST(CMatrixF, SchemaSerializationRoundTrip)
{
  CMatrixF m(2, 2);
  m(0, 0) = 1.0f;
  m(0, 1) = 2.0f;
  m(1, 0) = 3.0f;
  m(1, 1) = 4.0f;

  auto arch = mrpt::serialization::archiveJSON();
  arch = m;
  std::stringstream ss;
  ss << arch;

  ss.seekg(0);
  auto arch2 = mrpt::serialization::archiveJSON();
  ss >> arch2;
  CMatrixF m2;
  arch2.readTo(m2);

  ASSERT_EQ(m2.rows(), 2);
  ASSERT_EQ(m2.cols(), 2);
  EXPECT_NEAR(m(0, 0), m2(0, 0), 1e-5);
  EXPECT_NEAR(m(1, 1), m2(1, 1), 1e-5);
}
#endif

TEST(CMatrixF, SerializationRoundTrip)
{
  CMatrixF m(2, 2);
  m(0, 0) = 1.0f;
  m(0, 1) = 2.0f;
  m(1, 0) = 3.0f;
  m(1, 1) = 4.0f;

  const CMatrixF m2 = roundTripSerialize(m);
  ASSERT_EQ(m2.rows(), 2);
  ASSERT_EQ(m2.cols(), 2);
  EXPECT_NEAR(m(0, 0), m2(0, 0), 1e-6);
  EXPECT_NEAR(m(1, 1), m2(1, 1), 1e-6);
}

TEST(CMatrixF, CopyFromDoubleMatrix)
{
  CMatrixDynamic<double> md(2, 2);
  md(0, 0) = 1.0;
  md(0, 1) = 2.0;
  md(1, 0) = 3.0;
  md(1, 1) = 4.0;

  CMatrixF mf(md);
  EXPECT_NEAR(mf(0, 0), 1.0f, 1e-6);
  EXPECT_NEAR(mf(1, 1), 4.0f, 1e-6);
}

TEST(CMatrixDynamic, CastFloatDouble)
{
  CMatrixDynamic<double> md(2, 2);
  md(0, 0) = 1.5;
  md(0, 1) = -2.5;
  md(1, 0) = 3.5;
  md(1, 1) = 4.5;

  const CMatrixDynamic<float> mf = md.cast_float();
  ASSERT_EQ(mf.rows(), 2);
  ASSERT_EQ(mf.cols(), 2);
  EXPECT_NEAR(mf(0, 0), 1.5f, 1e-5);
  EXPECT_NEAR(mf(1, 1), 4.5f, 1e-5);

  const CMatrixDynamic<double> md2 = mf.cast_double();
  EXPECT_NEAR(md2(0, 0), 1.5, 1e-5);
  EXPECT_NEAR(md2(1, 1), 4.5, 1e-5);
}

TEST(CMatrixDynamic, LltSolve)
{
  // Symmetric positive-definite 2x2 system.
  CMatrixDynamic<double> A(2, 2);
  A(0, 0) = 4.0;
  A(0, 1) = 1.0;
  A(1, 0) = 1.0;
  A(1, 1) = 3.0;

  CVectorDynamic<double> b(2);
  b[0] = 1.0;
  b[1] = 2.0;

  const CVectorDynamic<double> x = A.llt_solve(b);
  ASSERT_EQ(x.size(), 2);

  // Verify A*x == b
  EXPECT_NEAR(A(0, 0) * x[0] + A(0, 1) * x[1], b[0], 1e-6);
  EXPECT_NEAR(A(1, 0) * x[0] + A(1, 1) * x[1], b[1], 1e-6);
}

TEST(CMatrixDynamic, LuSolve)
{
  CMatrixDynamic<double> A(2, 2);
  A(0, 0) = 2.0;
  A(0, 1) = 1.0;
  A(1, 0) = 1.0;
  A(1, 1) = -1.0;

  CVectorDynamic<double> b(2);
  b[0] = 3.0;
  b[1] = 0.0;

  const CVectorDynamic<double> x = A.lu_solve(b);
  ASSERT_EQ(x.size(), 2);

  EXPECT_NEAR(A(0, 0) * x[0] + A(0, 1) * x[1], b[0], 1e-6);
  EXPECT_NEAR(A(1, 0) * x[0] + A(1, 1) * x[1], b[1], 1e-6);
}
