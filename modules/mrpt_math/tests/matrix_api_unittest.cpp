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
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/CVectorFixed.h>
#include <mrpt/math/ops_matrices.h>

#include <Eigen/Dense>
#include <sstream>

using namespace mrpt::math;

TEST(CMatrixDynamic, cropConstructorAndSwap)
{
  CMatrixDouble m(3, 3);
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++) m(r, c) = 10 * r + c;

  // Crop to the top-left 2x2 block:
  const CMatrixDouble cropped(m, 2, 2);
  EXPECT_EQ(cropped.rows(), 2U);
  EXPECT_EQ(cropped.cols(), 2U);
  EXPECT_NEAR(cropped(1, 1), 11.0, 1e-12);

  // Cropping to a larger size is rejected:
  EXPECT_THROW(CMatrixDouble(m, 4, 2), std::exception);
  EXPECT_THROW(CMatrixDouble(m, 2, 4), std::exception);

  CMatrixDouble a(1, 1), b(2, 2);
  a(0, 0) = 7.0;
  a.swap(b);
  EXPECT_EQ(a.rows(), 2U);
  EXPECT_EQ(b.rows(), 1U);
  EXPECT_NEAR(b(0, 0), 7.0, 1e-12);
}

TEST(CMatrixDynamic, moveSemanticsAndSizes)
{
  CMatrixDouble m(2, 3);
  m.fill(4.0);

  const CMatrixDouble moved(std::move(m));
  EXPECT_EQ(moved.rows(), 2U);
  EXPECT_EQ(moved.cols(), 3U);
  EXPECT_NEAR(moved(1, 2), 4.0, 1e-12);

  CMatrixDouble m2(1, 1);
  m2 = CMatrixDouble(4, 5);
  EXPECT_EQ(m2.rows(), 4U);

  const auto dims = moved.size();
  EXPECT_EQ(dims[0], 2U);
  EXPECT_EQ(dims[1], 3U);
}

TEST(CMatrixDynamic, resizeOverloads)
{
  CMatrixDouble m;

  // Nx1 vector-like resize:
  m.resize(5U);
  EXPECT_EQ(m.rows(), 5U);
  EXPECT_EQ(m.cols(), 1U);

  // Shrinking the rows while growing the columns, asking for the new cells to
  // be zeroed: the "zero the new columns" pass must stay within the *new*
  // buffer (2x4 = 8 cells), not iterate over the 5 old rows.
  matrix_size_t siz;
  siz[0] = 2;
  siz[1] = 4;
  m.resize(siz, true);
  EXPECT_EQ(m.rows(), 2U);
  EXPECT_EQ(m.cols(), 4U);
  // Zeroed new elements:
  EXPECT_NEAR(m(1, 3), 0.0, 1e-12);

  // Growing with zeroNewElements keeps old content and zeroes the rest:
  CMatrixDouble g(1, 1);
  g(0, 0) = 9.0;
  g.setSize(2, 2, true);
  EXPECT_NEAR(g(0, 0), 9.0, 1e-12);
  EXPECT_NEAR(g(0, 1), 0.0, 1e-12);
  EXPECT_NEAR(g(1, 1), 0.0, 1e-12);

  // Shrinking keeps the top-left block:
  g.setSize(1, 1);
  EXPECT_EQ(g.rows(), 1U);
  EXPECT_NEAR(g(0, 0), 9.0, 1e-12);

  // Setting the same size is a no-op:
  g.setSize(1, 1);
  EXPECT_EQ(g.rows(), 1U);
}

TEST(CMatrixDynamic, arrayConstructorSizeMismatch)
{
  double values[6] = {1, 2, 3, 4, 5, 6};
  const CMatrixDouble ok(2, 3, values);
  EXPECT_NEAR(ok(1, 2), 6.0, 1e-12);

  EXPECT_THROW(CMatrixDouble(2, 2, values), std::exception);
}

TEST(CMatrixDynamic, constIterationAndIndexing)
{
  CMatrixDouble m(1, 3);
  m(0, 0) = 1.0;
  m(0, 1) = 2.0;
  m(0, 2) = 3.0;

  const CMatrixDouble& cm = m;
  EXPECT_NEAR(cm[2], 3.0, 1e-12);
  EXPECT_EQ(std::distance(cm.cbegin(), cm.cend()), 3);
  EXPECT_EQ(std::distance(cm.begin(), cm.end()), 3);
}

TEST(CMatrixFixed, assignmentsFromOtherTypes)
{
  CMatrixDouble dyn(2, 2);
  dyn(0, 0) = 1.0;
  dyn(1, 1) = 2.0;

  CMatrixDouble22 fixed;
  fixed = dyn;
  EXPECT_NEAR(fixed(1, 1), 2.0, 1e-12);

  // From a plain Eigen expression:
  Eigen::MatrixXd e(2, 2);
  e.setZero();
  e(1, 1) = 4.0;
  fixed = e;
  EXPECT_NEAR(fixed(1, 1), 4.0, 1e-12);

  // The (rows, cols) constructor only accepts the compile-time size:
  EXPECT_NO_THROW(CMatrixDouble22(2, 2));
  EXPECT_THROW(CMatrixDouble22(3, 2), std::exception);
  EXPECT_THROW(CMatrixDouble22(2, 3), std::exception);
}

TEST(CMatrixFixed, resizeChecksAndSize)
{
  CMatrixDouble22 m;
  EXPECT_NO_THROW(m.resize(2, 2));
  EXPECT_THROW(m.resize(3, 2), std::exception);

  matrix_size_t siz;
  siz[0] = 2;
  siz[1] = 2;
  EXPECT_NO_THROW(m.resize(siz));

  // The 1-argument resize() is only meaningful for row/column matrices:
  EXPECT_THROW(m.resize(size_t(2)), std::exception);

  CVectorFixedDouble<3> v;
  EXPECT_NO_THROW(v.resize(size_t(3)));
  EXPECT_THROW(v.resize(size_t(4)), std::exception);

  const auto dims = m.size();
  EXPECT_EQ(dims[0], 2U);
  EXPECT_EQ(dims[1], 2U);
}

TEST(CMatrixFixed, linearIndexingAndSwap)
{
  CMatrixDouble22 m;
  m(0, 0) = 1.0;
  m(0, 1) = 2.0;
  m(1, 0) = 3.0;
  m(1, 1) = 4.0;

  // Row-major linear access:
  EXPECT_NEAR(m(2), 3.0, 1e-12);
  const CMatrixDouble22& cm = m;
  EXPECT_NEAR(cm(3), 4.0, 1e-12);
  EXPECT_EQ(std::distance(cm.begin(), cm.end()), 4);

  CMatrixDouble22 other;
  other.fill(9.0);
  m.swap(other);
  EXPECT_NEAR(m(0, 0), 9.0, 1e-12);
  EXPECT_NEAR(other(0, 0), 1.0, 1e-12);
}

TEST(CMatrixFixed, sum_At)
{
  CMatrixDouble22 m;
  m.fill(0);
  CMatrixDouble22 a;
  a(0, 0) = 1.0;
  a(0, 1) = 2.0;
  a(1, 0) = 3.0;
  a(1, 1) = 4.0;

  m.sum_At(a);
  EXPECT_NEAR(m(0, 1), 3.0, 1e-12);
  EXPECT_NEAR(m(1, 0), 2.0, 1e-12);

  // Non-square matrices have no in-place transpose sum:
  CMatrixFixed<double, 2, 3> nonSquare;
  nonSquare.fill(0);
  EXPECT_THROW(nonSquare.sum_At(nonSquare), std::runtime_error);
}

TEST(MatrixVectorBase, minMaxCoeffWithIndices)
{
  CVectorDouble v(4);
  v[0] = 3.0;
  v[1] = -1.0;
  v[2] = 7.0;
  v[3] = 0.0;

  matrix_index_t idx = 0;
  EXPECT_NEAR(v.minCoeff(idx), -1.0, 1e-12);
  EXPECT_EQ(idx, 1U);
  EXPECT_NEAR(v.maxCoeff(idx), 7.0, 1e-12);
  EXPECT_EQ(idx, 2U);

  CMatrixDouble m(2, 2);
  m(0, 0) = 5.0;
  m(0, 1) = -2.0;
  m(1, 0) = 1.0;
  m(1, 1) = 3.0;

  matrix_index_t r = 0, c = 0;
  EXPECT_NEAR(m.minCoeff(r, c), -2.0, 1e-12);
  EXPECT_EQ(r, 0U);
  EXPECT_EQ(c, 1U);
  EXPECT_NEAR(m.maxCoeff(r, c), 5.0, 1e-12);
  EXPECT_EQ(r, 0U);
  EXPECT_EQ(c, 0U);

  // The 1-index signatures are only valid for column vectors:
  EXPECT_THROW(m.minCoeff(idx), std::runtime_error);
  EXPECT_THROW(m.maxCoeff(idx), std::runtime_error);
}

TEST(MatrixVectorBase, scalarInPlaceAddAndDot)
{
  CVectorDouble v(3);
  v.fill(1.0);
  v += 2.0;
  EXPECT_NEAR(v[0], 3.0, 1e-12);
  EXPECT_NEAR(v[2], 3.0, 1e-12);

  CVectorDouble w(3);
  w.fill(2.0);
  EXPECT_NEAR(v.dot(w), 18.0, 1e-12);

  // dot() is only defined for column vectors:
  CMatrixDouble m(2, 2);
  m.fill(1.0);
  CMatrixDouble m2(2, 2);
  m2.fill(1.0);
  EXPECT_THROW(m.dot(m2), std::exception);
}

TEST(MatrixVectorBase, fromMatlabStringFormatErrors)
{
  CMatrixDouble m;

  EXPECT_TRUE(m.fromMatlabStringFormat("[1 2 3; 4 5 6]"));
  EXPECT_EQ(m.rows(), 2U);
  EXPECT_EQ(m.cols(), 3U);

  // Rows of different length:
  std::stringstream errs;
  EXPECT_FALSE(m.fromMatlabStringFormat("[1 2 3; 4 5]", errs));
  EXPECT_FALSE(errs.str().empty());

  // Not a matrix at all:
  EXPECT_FALSE(m.fromMatlabStringFormat("hello"));

  // A fixed-size matrix rejects a mismatched number of rows/columns:
  CMatrixDouble22 f;
  std::stringstream errs2;
  EXPECT_FALSE(f.fromMatlabStringFormat("[1 2 3; 4 5 6; 7 8 9]", errs2));
  EXPECT_FALSE(errs2.str().empty());

  std::stringstream errs3;
  EXPECT_FALSE(f.fromMatlabStringFormat("[1 2]", errs3));
  EXPECT_FALSE(errs3.str().empty());
}
