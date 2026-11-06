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
#include <mrpt/math/CMatrixD.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixF.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/serialization/CSchemeArchiveBase.h>
#include <mrpt/serialization/config.h>
#include <mrpt/system/filesystem.h>

#include <Eigen/Dense>
#include <cmath>
#include <cstdint>
#include <sstream>

#if MRPT_HAS_JSONCPP
#include <mrpt/serialization/CSchemeArchive.h>
#endif

using mrpt::math::CMatrixDouble;
using mrpt::math::CMatrixDouble33;

namespace
{
CMatrixDouble awkwardValues()
{
  CMatrixDouble m(2, 3);
  m(0, 0) = 1.0 / 3.0;
  m(0, 1) = -123456.78901234567;
  m(0, 2) = 6.02214076e23;
  m(1, 0) = 1e-300;
  m(1, 1) = -0.0;
  m(1, 2) = std::nextafter(1.0, 2.0);
  return m;
}
}  // namespace

TEST(MatrixTextIO, matlabFormatRoundTripIsExactWith17Digits)
{
  const CMatrixDouble m = awkwardValues();
  const std::string s = m.inMatlabFormat(17);
  CMatrixDouble m2;
  ASSERT_TRUE(m2.fromMatlabStringFormat(s));
  ASSERT_EQ(m2.rows(), m.rows());
  ASSERT_EQ(m2.cols(), m.cols());
  for (int r = 0; r < m.rows(); r++)
  {
    for (int c = 0; c < m.cols(); c++)
    {
      EXPECT_EQ(m2(r, c), m(r, c)) << s;
    }
  }
}

TEST(MatrixTextIO, matlabFormatFixedSizeChecks)
{
  CMatrixDouble33 m;
  EXPECT_TRUE(m.fromMatlabStringFormat("[1 2 3; 4 5 6; 7 8 9]"));
  EXPECT_EQ(m(2, 1), 8.0);
  EXPECT_FALSE(m.fromMatlabStringFormat("[1 2 3; 4 5 6]"));
  EXPECT_FALSE(m.fromMatlabStringFormat("[1 2; 4 5; 7 8]"));
  EXPECT_FALSE(m.fromMatlabStringFormat("[1 2 3; 4 5 6; 7 8 9; 1 1 1]"));
  CMatrixDouble d;
  EXPECT_FALSE(d.fromMatlabStringFormat("[1 2 3; 4 5]"));
  EXPECT_FALSE(d.fromMatlabStringFormat("1 2 3"));
}

// Commas are column separators, as in MATLAB
TEST(MatrixTextIO, matlabFormatAcceptsCommas)
{
  CMatrixDouble m;
  ASSERT_TRUE(m.fromMatlabStringFormat("[1, 2, 3]"));
  ASSERT_EQ(m.rows(), 1);
  ASSERT_EQ(m.cols(), 3);
  EXPECT_EQ(m(0, 2), 3.0);

  ASSERT_TRUE(m.fromMatlabStringFormat("[1,2,3;4, 5 ,6]"));
  ASSERT_EQ(m.rows(), 2);
  ASSERT_EQ(m.cols(), 3);
  EXPECT_EQ(m(1, 1), 5.0);

  // Trailing commas, also before ';'
  ASSERT_TRUE(m.fromMatlabStringFormat("[1, -2, ; 3, 4e1, ]"));
  ASSERT_EQ(m.rows(), 2);
  ASSERT_EQ(m.cols(), 2);
  EXPECT_EQ(m(0, 1), -2.0);
  EXPECT_EQ(m(1, 1), 40.0);

  mrpt::math::CMatrixDouble33 f;
  EXPECT_TRUE(f.fromMatlabStringFormat("[1, 0, 0; 0, 1, 0; 0, 0, 1]"));
  EXPECT_EQ(f(2, 2), 1.0);
}

// Tokens that are not numbers make the string malformed
TEST(MatrixTextIO, matlabFormatRejectsGarbageTokens)
{
  CMatrixDouble m;
  EXPECT_FALSE(m.fromMatlabStringFormat("[1 2 abc 3]"));
  EXPECT_FALSE(m.fromMatlabStringFormat("[1 2; 3 4x]"));
  EXPECT_FALSE(m.fromMatlabStringFormat("[1; 2 / 3]"));
  mrpt::math::CMatrixDynamic<int16_t> mi;
  EXPECT_FALSE(mi.fromMatlabStringFormat("[1 2.5]"));
  EXPECT_TRUE(mi.fromMatlabStringFormat("[1 -2]"));
}

TEST(MatrixTextIO, textFileRoundTripIsExact)
{
  const CMatrixDouble m = awkwardValues();
  const std::string fil = mrpt::system::getTempFileName();
  m.saveToTextFile(fil, mrpt::math::MATRIX_FORMAT_ENG, true, "% user header");
  CMatrixDouble m2;
  m2.loadFromTextFile(fil);
  ASSERT_EQ(m2.rows(), m.rows());
  ASSERT_EQ(m2.cols(), m.cols());
  for (int r = 0; r < m.rows(); r++)
  {
    for (int c = 0; c < m.cols(); c++)
    {
      EXPECT_EQ(m2(r, c), m(r, c));
    }
  }
  mrpt::system::deleteFile(fil);
}

// The file determines the size, regardless of the previous one
TEST(MatrixTextIO, loadFromTextFileIntoPreSizedMatrix)
{
  CMatrixDouble m(10, 10);
  std::stringstream ss("1 2 3\n4 5 6\n");
  ASSERT_NO_THROW(m.loadFromTextFile(ss));
  EXPECT_EQ(m.rows(), 2);
  EXPECT_EQ(m.cols(), 3);
  if (m.rows() == 2 && m.cols() == 3)
  {
    EXPECT_EQ(m(1, 2), 6.0);
  }

  CMatrixDouble m1(10, 10);
  std::stringstream ss1("1 2 3\n");
  m1.loadFromTextFile(ss1);
  EXPECT_EQ(m1.rows(), 1);
  EXPECT_EQ(m1.cols(), 3);
}

// Fixed-size matrices must be completely filled from the file
TEST(MatrixTextIO, loadFromTextFileFixedSizeTooFewRowsThrows)
{
  CMatrixDouble33 m;
  std::stringstream ss("1 2 3\n4 5 6\n");
  EXPECT_ANY_THROW(m.loadFromTextFile(ss));
}

TEST(MatrixTextIO, loadFromTextFileFixedSizeTooManyRowsThrows)
{
  CMatrixDouble33 m;
  std::stringstream ss("1 2 3\n4 5 6\n7 8 9\n1 1 1\n");
  EXPECT_ANY_THROW(m.loadFromTextFile(ss));
}

#if MRPT_HAS_JSONCPP
// Schema (JSON) serialization must keep full precision
TEST(MatrixTextIO, CMatrixDSchemaRoundTripKeepsDoublePrecision)
{
  mrpt::math::CMatrixD m(1, 2);
  m(0, 0) = 1.0 / 3.0;
  m(0, 1) = 123456.789012345;

  auto arch = mrpt::serialization::archiveJSON();
  arch = m;
  std::stringstream ss;
  ss << arch;
  ss.seekg(0);
  auto arch2 = mrpt::serialization::archiveJSON();
  ss >> arch2;
  mrpt::math::CMatrixD m2;
  arch2.readTo(m2);
  ASSERT_EQ(m2.rows(), 1);
  ASSERT_EQ(m2.cols(), 2);
  EXPECT_EQ(m2(0, 0), m(0, 0));
  EXPECT_EQ(m2(0, 1), m(0, 1));

  mrpt::math::CMatrixF mf(1, 2);
  mf(0, 0) = 1.0f / 3.0f;
  mf(0, 1) = 123456.789f;
  auto archF = mrpt::serialization::archiveJSON();
  archF = mf;
  std::stringstream ssF;
  ssF << archF;
  ssF.seekg(0);
  auto archF2 = mrpt::serialization::archiveJSON();
  ssF >> archF2;
  mrpt::math::CMatrixF mf2;
  archF2.readTo(mf2);
  ASSERT_EQ(mf2.rows(), 1);
  ASSERT_EQ(mf2.cols(), 2);
  EXPECT_EQ(mf2(0, 0), mf(0, 0));
  EXPECT_EQ(mf2(0, 1), mf(0, 1));
}
#endif
