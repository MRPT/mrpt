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

// Corner cases of the public container API (matrices, vectors and the TPoint
// tuple-like accessors) that the algorithm-oriented tests never reach: generic
// iterators, linear indexing, in-place resizing that preserves contents, and
// the text-file I/O error paths.

#include <gtest/gtest.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/CVectorFixed.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/ops_matrices.h>
#include <mrpt/system/filesystem.h>

#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

using namespace mrpt::math;

// ---------------------------------------------------------------------------
//  TPoint2D / TPoint3D
// ---------------------------------------------------------------------------

TEST(TPoint3D, MatrixLikeShapeAccessors)
{
  TPoint3D p(1.0, 2.0, 3.0);
  EXPECT_EQ(p.rows(), 3U);
  EXPECT_EQ(p.cols(), 1U);
  EXPECT_EQ(p.size(), 3U);

  // data() gives contiguous access to (x,y,z):
  double* d = p.data();
  EXPECT_EQ(d[0], 1.0);
  EXPECT_EQ(d[2], 3.0);
  d[1] = 20.0;
  EXPECT_EQ(p.y, 20.0);

  const TPoint3D& cp = p;
  EXPECT_EQ(cp.data()[1], 20.0);
}

TEST(TPoint3D, OperatorBracketOutOfRangeThrows)
{
  TPoint3D p(1.0, 2.0, 3.0);
  const TPoint3D& cp = p;

  EXPECT_EQ(p[2], 3.0);
  EXPECT_EQ(cp[0], 1.0);
  EXPECT_THROW((void)p[3], std::out_of_range);
  EXPECT_THROW((void)cp[3], std::out_of_range);
}

TEST(TPoint3D, DotCrossAndScalarDivision)
{
  const TPoint3D a(1.0, 2.0, 3.0);
  const TPoint3D b(4.0, -5.0, 6.0);

  EXPECT_DOUBLE_EQ(a.dot(b), (1 * 4) + (2 * -5) + (3 * 6));

  const TPoint3D c = a.cross(b);
  EXPECT_DOUBLE_EQ(c.x, (2 * 6) - (3 * -5));
  EXPECT_DOUBLE_EQ(c.y, (3 * 4) - (1 * 6));
  EXPECT_DOUBLE_EQ(c.z, (1 * -5) - (2 * 4));
  // The cross product is orthogonal to both operands:
  EXPECT_NEAR(c.dot(a), 0.0, 1e-12);
  EXPECT_NEAR(c.dot(b), 0.0, 1e-12);

  const TPoint3D d = a / 2.0;
  EXPECT_DOUBLE_EQ(d.x, 0.5);
  EXPECT_DOUBLE_EQ(d.z, 1.5);
}

TEST(TPoint3D, InPlaceTranslationAndDifference)
{
  TPoint3D p(1.0, 2.0, 3.0);
  p += TPoint3D(1.0, 1.0, 1.0);
  EXPECT_DOUBLE_EQ(p.x, 2.0);
  p -= TPoint3D(0.5, 0.5, 0.5);
  EXPECT_DOUBLE_EQ(p.y, 2.5);
}

TEST(TPoint3D, FromString)
{
  const auto p = TPoint3D::FromString("[1.0 2.0 3.0]");
  EXPECT_DOUBLE_EQ(p.x, 1.0);
  EXPECT_DOUBLE_EQ(p.z, 3.0);
}

TEST(TPoint2D, MatrixLikeShapeAccessorsAndArithmetic)
{
  TPoint2D p(3.0, 4.0);
  EXPECT_EQ(p.rows(), 2U);
  EXPECT_EQ(p.cols(), 1U);
  EXPECT_EQ(p.size(), 2U);

  const TPoint2D& cp = p;
  EXPECT_EQ(cp.data()[0], 3.0);

  p *= 2.0;
  EXPECT_DOUBLE_EQ(p.x, 6.0);
  p /= 4.0;
  EXPECT_DOUBLE_EQ(p.x, 1.5);
  EXPECT_DOUBLE_EQ(p.y, 2.0);

  const TPoint2D q = p / 2.0;
  EXPECT_DOUBLE_EQ(q.x, 0.75);

  p += TPoint2D(1.0, 1.0);
  EXPECT_DOUBLE_EQ(p.x, 2.5);
  p -= TPoint2D(0.5, 0.5);
  EXPECT_DOUBLE_EQ(p.y, 2.5);
}

// ---------------------------------------------------------------------------
//  CMatrixFixed
// ---------------------------------------------------------------------------

TEST(CMatrixFixed, GenericContainerApi)
{
  CMatrixDouble33 m;
  m.setIdentity();

  EXPECT_EQ(m.size()[0], 3U);
  EXPECT_EQ(m.size()[1], 3U);

  // Const iterators over the flat storage:
  double sum = 0;
  for (auto it = m.cbegin(); it != m.cend(); ++it)
  {
    sum += *it;
  }
  EXPECT_DOUBLE_EQ(sum, 3.0);

  // derived() is the CRTP hook used by the MatrixVectorBase mixins:
  EXPECT_EQ(&m.derived(), &m);
  const CMatrixDouble33& cm = m;
  EXPECT_EQ(&cm.derived(), &cm);

  // conservativeResize() on a fixed-size matrix is a no-op sanity check:
  m.conservativeResize(3, 3);
  EXPECT_EQ(m.rows(), 3);

  CMatrixDouble33 other;
  other.setZero();
  m.swap(other);
  EXPECT_DOUBLE_EQ(m(0, 0), 0.0);
  EXPECT_DOUBLE_EQ(other(0, 0), 1.0);
}

TEST(CMatrixFixed, LinearIndexing)
{
  CMatrixDouble33 m;
  for (int i = 0; i < 9; i++)
  {
    m(i) = i * 1.0;
  }
  const CMatrixDouble33& cm = m;
  for (int i = 0; i < 9; i++)
  {
    EXPECT_DOUBLE_EQ(cm(i), i * 1.0);
  }
  // Row-major storage:
  EXPECT_DOUBLE_EQ(m(1, 2), 5.0);
}

// ---------------------------------------------------------------------------
//  CMatrixDynamic
// ---------------------------------------------------------------------------

TEST(CMatrixDynamic, SetSizePreservesContents)
{
  CMatrixDouble m(2, 2);
  m(0, 0) = 1;
  m(0, 1) = 2;
  m(1, 0) = 3;
  m(1, 1) = 4;

  // Grow in both directions: the old block is kept, new cells are zeroed.
  m.setSize(3, 4, true /* zeroNewElements */);
  ASSERT_EQ(m.rows(), 3);
  ASSERT_EQ(m.cols(), 4);
  EXPECT_DOUBLE_EQ(m(0, 0), 1);
  EXPECT_DOUBLE_EQ(m(1, 1), 4);
  EXPECT_DOUBLE_EQ(m(2, 0), 0);  // new row
  EXPECT_DOUBLE_EQ(m(0, 3), 0);  // new column

  // Shrink: the surviving top-left block keeps its values.
  m.setSize(1, 2, true);
  ASSERT_EQ(m.rows(), 1);
  ASSERT_EQ(m.cols(), 2);
  EXPECT_DOUBLE_EQ(m(0, 1), 2);
}

TEST(CMatrixDynamic, GenericContainerApi)
{
  CMatrixDouble m(2, 3);
  m.fill(7.0);

  EXPECT_EQ(m.size()[0], 2U);
  EXPECT_EQ(m.size()[1], 3U);
  EXPECT_EQ(&m.derived(), &m);
  const CMatrixDouble& cm = m;
  EXPECT_EQ(&cm.derived(), &cm);

  m.conservativeResize(2, 2);
  EXPECT_EQ(m.cols(), 2);

  // Flat operator[] on the const overload:
  const CMatrixDouble& cm2 = m;
  EXPECT_DOUBLE_EQ(cm2[3], 7.0);
}

// ---------------------------------------------------------------------------
//  CVectorDynamic
// ---------------------------------------------------------------------------

TEST(CVectorDynamic, GenericContainerApi)
{
  CVectorDouble v(4);
  v.fill(2.0);

  EXPECT_EQ(v.cols(), 1);
  EXPECT_EQ(v.rows(), 4);

  double sum = 0;
  for (auto it = v.cbegin(); it != v.cend(); ++it)
  {
    sum += *it;
  }
  EXPECT_DOUBLE_EQ(sum, 8.0);
}

TEST(CVectorDynamic, OutOfRangeAccessThrows)
{
  CVectorDouble v(3);
  v.fill(0);
  EXPECT_THROW((void)v(5, 0), std::exception);
  EXPECT_THROW((void)v(0, 1), std::exception);
  const CVectorDouble& cv = v;
  EXPECT_THROW((void)cv(5, 0), std::exception);
}

// ---------------------------------------------------------------------------
//  MatrixVectorBase: vector-only setConstant()/setZero() overloads
// ---------------------------------------------------------------------------

TEST(MatrixVectorBase, VectorOnlySetConstantAndSetZero)
{
  CVectorDouble v;
  v.setConstant(4, 3.5);
  ASSERT_EQ(v.size(), 4);
  EXPECT_DOUBLE_EQ(v[0], 3.5);
  EXPECT_DOUBLE_EQ(v[3], 3.5);

  v.setZero(2);
  ASSERT_EQ(v.size(), 2);
  EXPECT_DOUBLE_EQ(v[1], 0.0);
}

TEST(MatrixVectorBase, ArrayAccessorOnConstMatrix)
{
  CMatrixDouble33 m;
  m.setConstant(2.0);
  const CMatrixDouble33& cm = m;
  EXPECT_DOUBLE_EQ(cm.array().sum(), 18.0);
}

TEST(MatrixVectorBase, DotProductRequiresColumnVectors)
{
  CVectorDouble a(3);
  CVectorDouble b(3);
  a.fill(1.0);
  b.fill(2.0);
  EXPECT_DOUBLE_EQ(a.dot(b), 6.0);
}

// ---------------------------------------------------------------------------
//  Text-file I/O error paths
// ---------------------------------------------------------------------------

TEST(MatrixTextIO, SaveToTextFileFormatsAndErrors)
{
  CMatrixDouble m(2, 2);
  m(0, 0) = 1.5;
  m(0, 1) = -2.5;
  m(1, 0) = 3.0;
  m(1, 1) = 4.0;

  const std::string f = mrpt::system::getTempFileName();

  m.saveToTextFile(f, MATRIX_FORMAT_INT);
  {
    CMatrixDouble r;
    r.loadFromTextFile(f);
    EXPECT_EQ(r.rows(), 2);
    EXPECT_DOUBLE_EQ(r(1, 0), 3.0);
  }

  // A user header without a trailing newline must still end up on its own
  // line, or it would be glued to the first data row (which loadFromTextFile
  // would then skip as a comment, silently losing that row).
  m.saveToTextFile(f, MATRIX_FORMAT_FIXED, true, "% a header");
  {
    CMatrixDouble r;
    r.loadFromTextFile(f);
    ASSERT_EQ(r.rows(), 2);
    EXPECT_NEAR(r(0, 0), 1.5, 1e-9);
    EXPECT_NEAR(r(0, 1), -2.5, 1e-9);
  }

  // A header that already ends in a newline is written unchanged:
  m.saveToTextFile(f, MATRIX_FORMAT_ENG, false, "% a header\n");
  {
    CMatrixDouble r;
    r.loadFromTextFile(f);
    ASSERT_EQ(r.rows(), 2);
    EXPECT_NEAR(r(1, 1), 4.0, 1e-9);
  }

  EXPECT_THROW(m.saveToTextFile(f, static_cast<TMatrixTextFileFormat>(99)), std::exception);

  mrpt::system::deleteFile(f);
}

TEST(MatrixTextIO, SaveToTextFileUnwritablePathThrows)
{
  CMatrixDouble m(1, 1);
  m(0, 0) = 0;
  EXPECT_THROW(
      m.saveToTextFile("/this/path/does/not/exist/matrix.txt", MATRIX_FORMAT_FIXED),
      std::runtime_error);
}

TEST(MatrixTextIO, LoadFromTextFileErrors)
{
  CMatrixDouble m;
  EXPECT_THROW(m.loadFromTextFile("/no/such/file/at/all.txt"), std::runtime_error);

  const std::string f = mrpt::system::getTempFileName();
  {
    std::ofstream o(f);
    o << "1 2 3\n4 5\n";  // ragged: not a matrix
  }
  EXPECT_THROW(m.loadFromTextFile(f), std::runtime_error);
  mrpt::system::deleteFile(f);
}

TEST(MatrixTextIO, LoadFromTextFileGrowsBeyondInitialGuess)
{
  // The reader starts with a modest row capacity and grows it as needed; feed
  // it enough rows to force at least one reallocation.
  const std::string f = mrpt::system::getTempFileName();
  {
    std::ofstream o(f);
    for (int i = 0; i < 200; i++)
    {
      o << i << " " << (2 * i) << "\n";
    }
  }
  CMatrixDouble m;
  m.loadFromTextFile(f);
  ASSERT_EQ(m.rows(), 200);
  ASSERT_EQ(m.cols(), 2);
  EXPECT_DOUBLE_EQ(m(199, 1), 398.0);
  mrpt::system::deleteFile(f);
}

TEST(MatrixTextIO, FromMatlabStringFormatTooManyRowsForFixedSize)
{
  CMatrixDouble33 m;
  std::stringstream errors;
  // 4 rows do not fit in a 3x3 fixed-size matrix:
  EXPECT_FALSE(m.fromMatlabStringFormat("[1 2 3; 4 5 6; 7 8 9; 10 11 12]", std::ref(errors)));
  EXPECT_FALSE(errors.str().empty());
}

// ---------------------------------------------------------------------------
//  ops_matrices error paths
// ---------------------------------------------------------------------------

TEST(OpsMatrices, LaplacianRequiresSquareMatrix)
{
  CMatrixDouble g(2, 3);
  g.fill(1.0);
  CMatrixDouble L;
  EXPECT_THROW(laplacian(g, L), std::runtime_error);
}

TEST(OpsMatrices, ExtractSubmatrixSymmetricalErrors)
{
  CMatrixDouble m(3, 4);  // not square
  m.fill(1.0);
  CMatrixDouble out;
  const std::vector<size_t> idxs = {0, 1};

  EXPECT_THROW(extractSubmatrixSymmetrical(m, idxs, out), std::runtime_error);
  EXPECT_THROW((extractSubmatrixSymmetricalBlocks<2>(m, idxs, out)), std::runtime_error);
  EXPECT_THROW(extractSubmatrixSymmetricalBlocksDyn(m, 2, idxs, out), std::runtime_error);

  CMatrixDouble sq(4, 4);
  sq.fill(1.0);
  EXPECT_THROW(extractSubmatrixSymmetricalBlocksDyn(sq, 0, idxs, out), std::runtime_error);

  // ... and the working case, for reference:
  extractSubmatrixSymmetricalBlocksDyn(sq, 2, std::vector<size_t>{0, 1}, out);
  EXPECT_EQ(out.rows(), 4);
}

// ---------------------------------------------------------------------------
//  MatrixBase row/col views and diagonal helpers
// ---------------------------------------------------------------------------

TEST(MatrixBase, RowAndColumnViews)
{
  CMatrixDouble m(2, 3);
  m(0, 0) = 1;
  m(0, 1) = 2;
  m(0, 2) = 3;
  m(1, 0) = 4;
  m(1, 1) = 5;
  m(1, 2) = 6;

  EXPECT_DOUBLE_EQ(m.row(1).sum(), 15.0);
  EXPECT_DOUBLE_EQ(m.col(2).sum(), 9.0);

  const CMatrixDouble& cm = m;
  EXPECT_DOUBLE_EQ(cm.row(0).sum(), 6.0);
  EXPECT_DOUBLE_EQ(cm.col(0).sum(), 5.0);
}

TEST(MatrixBase, MinimumDiagonal)
{
  CMatrixDouble33 m;
  m.setIdentity();
  m(1, 1) = -2.0;
  EXPECT_DOUBLE_EQ(m.minimumDiagonal(), -2.0);
}

TEST(MatrixBase, EigenvectorsSymmetricReturnsVectorsAndValues)
{
  CMatrixDouble33 m;
  m.setZero();
  m(0, 0) = 3.0;
  m(1, 1) = 1.0;
  m(2, 2) = 2.0;

  CMatrixDouble33 eVecs;
  std::vector<double> eVals;
  ASSERT_TRUE(m.eig(eVecs, eVals));
  ASSERT_EQ(eVals.size(), 3U);
  // Non-symmetric-solver path returns them unsorted; check the set instead:
  double sum = 0;
  for (double v : eVals)
  {
    sum += v;
  }
  EXPECT_NEAR(sum, 6.0, 1e-9);
}

// setSize() has separate copy/zero paths for trivially-copyable elements
// (memcpy/memset) and for anything else; only the former is reached by the
// numeric instantiations used everywhere else.
TEST(CMatrixDynamic, SetSizeWithNonTrivialElementType)
{
  CMatrixDynamic<std::string> m(2, 2);
  m(0, 0) = "a";
  m(0, 1) = "b";
  m(1, 0) = "c";
  m(1, 1) = "d";

  m.setSize(3, 3, true /* zeroNewElements */);
  ASSERT_EQ(m.rows(), 3);
  ASSERT_EQ(m.cols(), 3);
  EXPECT_EQ(m(0, 0), "a");
  EXPECT_EQ(m(1, 1), "d");
  EXPECT_TRUE(m(2, 0).empty());  // new row
  EXPECT_TRUE(m(0, 2).empty());  // new column

  m.setSize(1, 1, true);
  ASSERT_EQ(m.rows(), 1);
  EXPECT_EQ(m(0, 0), "a");
}
