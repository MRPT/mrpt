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
#include <mrpt/math/CSparseMatrixTemplate.h>

#include <vector>

using namespace mrpt::math;

TEST(CSparseMatrixTemplate, DefaultCtorAndSize)
{
  CSparseMatrixTemplate<double> m;
  EXPECT_EQ(m.rows(), 0u);
  EXPECT_EQ(m.cols(), 0u);
  EXPECT_TRUE(m.empty());
}

TEST(CSparseMatrixTemplate, SizedCtorAndAccess)
{
  CSparseMatrixTemplate<double> m(3, 4);
  EXPECT_EQ(m.rows(), 3u);
  EXPECT_EQ(m.cols(), 4u);

  // Reading via the const accessor returns the default value, without
  // creating the entry:
  const auto& cm = m;
  EXPECT_EQ(cm(1, 1), 0.0);
  EXPECT_FALSE(m.exists(1, 1));

  // The non-const operator() follows std::map semantics: it creates the
  // entry even when only reading through it.
  m(1, 1) = 5.0;
  EXPECT_TRUE(m.exists(1, 1));
  EXPECT_EQ(m(1, 1), 5.0);
  EXPECT_EQ(m.getNonNullElements(), 1u);
  EXPECT_EQ(m.getNullElements(), 3u * 4u - 1u);
}

TEST(CSparseMatrixTemplate, InsertAndIsNull)
{
  CSparseMatrixTemplate<double> m(2, 2);
  m.insert(0, 1, 7.0);
  EXPECT_TRUE(m.isNotNull(0, 1));
  EXPECT_FALSE(m.isNull(0, 1));
  EXPECT_TRUE(m.isNull(0, 0));
  EXPECT_FALSE(m.isNotNull(0, 0));
}

TEST(CSparseMatrixTemplate, InsertMatrix)
{
  CSparseMatrixTemplate<double> m(4, 4);
  CMatrixDouble block(2, 2);
  block(0, 0) = 1;
  block(0, 1) = 2;
  block(1, 0) = 3;
  block(1, 1) = 4;

  m.insertMatrix(1, 1, block);
  EXPECT_EQ(m(1, 1), 1.0);
  EXPECT_EQ(m(1, 2), 2.0);
  EXPECT_EQ(m(2, 1), 3.0);
  EXPECT_EQ(m(2, 2), 4.0);
  EXPECT_EQ(m.getNonNullElements(), 4u);
}

TEST(CSparseMatrixTemplate, GetSetRow)
{
  CSparseMatrixTemplate<double> m(2, 3);
  std::vector<double> row{1.0, 0.0, 3.0};
  m.setRow(0, row);

  std::vector<double> out;
  m.getRow(0, out);
  ASSERT_EQ(out.size(), 3u);
  EXPECT_EQ(out[0], 1.0);
  EXPECT_EQ(out[1], 0.0);
  EXPECT_EQ(out[2], 3.0);

  // A row that was never set returns all default (null) values:
  std::vector<double> emptyRow;
  m.getRow(1, emptyRow);
  ASSERT_EQ(emptyRow.size(), 3u);
  for (double v : emptyRow)
  {
    EXPECT_EQ(v, 0.0);
  }
}

TEST(CSparseMatrixTemplate, GetSetColumn)
{
  CSparseMatrixTemplate<double> m(3, 2);
  std::vector<double> col{1.0, 0.0, 3.0};
  m.setColumn(1, col);

  std::vector<double> out;
  m.getColumn(1, out);
  ASSERT_EQ(out.size(), 3u);
  EXPECT_EQ(out[0], 1.0);
  EXPECT_EQ(out[2], 3.0);

  // Setting to the nullObject value erases the cell:
  m.setColumn(1, std::vector<double>{0.0, 0.0, 0.0});
  EXPECT_FALSE(m.exists(0, 1));
}

TEST(CSparseMatrixTemplate, SetRowWrongSizeThrows)
{
  CSparseMatrixTemplate<double> m(2, 3);
  EXPECT_THROW(m.setRow(0, std::vector<double>{1.0, 2.0}), std::exception);
}

TEST(CSparseMatrixTemplate, SetColumnWrongSizeThrows)
{
  CSparseMatrixTemplate<double> m(3, 2);
  EXPECT_THROW(m.setColumn(0, std::vector<double>{1.0, 2.0}), std::exception);
}

TEST(CSparseMatrixTemplate, Resize)
{
  CSparseMatrixTemplate<double> m(3, 3);
  m(2, 2) = 9.0;
  m(0, 0) = 1.0;

  // Shrinking must drop out-of-range elements:
  m.resize(2, 2);
  EXPECT_EQ(m.rows(), 2u);
  EXPECT_EQ(m.cols(), 2u);
  EXPECT_FALSE(m.exists(2, 2));
  EXPECT_TRUE(m.exists(0, 0));

  // Resizing to the same size is a no-op:
  m.resize(2, 2);
  EXPECT_TRUE(m.exists(0, 0));
}

TEST(CSparseMatrixTemplate, SubmatrixExtraction)
{
  CSparseMatrixTemplate<double> m(4, 4);
  m(1, 1) = 11.0;
  m(1, 2) = 12.0;
  m(2, 1) = 21.0;
  m(2, 2) = 22.0;
  m(3, 3) = 33.0;  // outside the submatrix range

  const auto sub = m(1, 2, 1, 2);
  EXPECT_EQ(sub.rows(), 2u);
  EXPECT_EQ(sub.cols(), 2u);
  EXPECT_EQ(sub(0, 0), 11.0);
  EXPECT_EQ(sub(0, 1), 12.0);
  EXPECT_EQ(sub(1, 0), 21.0);
  EXPECT_EQ(sub(1, 1), 22.0);
}

TEST(CSparseMatrixTemplate, AsVector)
{
  CSparseMatrixTemplate<double> m(2, 2);
  m(0, 0) = 1.0;
  m(1, 1) = 2.0;

  std::vector<double> v;
  m.asVector(v);
  ASSERT_EQ(v.size(), 2u);
}

TEST(CSparseMatrixTemplate, IteratorsBeginEnd)
{
  CSparseMatrixTemplate<double> m(2, 2);
  m(0, 0) = 1.0;
  m(1, 1) = 2.0;

  double sum = 0;
  for (auto it = m.begin(); it != m.end(); ++it)
  {
    sum += it->second;
  }
  EXPECT_EQ(sum, 3.0);

  double revSum = 0;
  for (auto it = m.rbegin(); it != m.rend(); ++it)
  {
    revSum += it->second;
  }
  EXPECT_EQ(revSum, 3.0);
}

TEST(CSparseMatrixTemplate, ClearAndPurge)
{
  CSparseMatrixTemplate<double> m(2, 2);
  m(0, 0) = 1.0;
  m(0, 1) = 0.0;  // explicitly stored zero
  EXPECT_EQ(m.getNonNullElements(), 2u);

  m.purge(0.0);
  EXPECT_EQ(m.getNonNullElements(), 1u);
  EXPECT_TRUE(m.exists(0, 0));
  EXPECT_FALSE(m.exists(0, 1));

  m.clear();
  EXPECT_TRUE(m.empty());
}

TEST(CSparseSymmetricalMatrix, SymmetricAccess)
{
  CSparseSymmetricalMatrix<double> m;
  m.resize(3);
  m(0, 2) = 5.0;

  // Both orderings must read the same stored value:
  EXPECT_EQ(m(0, 2), 5.0);
  EXPECT_EQ(m(2, 0), 5.0);
}

TEST(CSparseSymmetricalMatrix, CopyCtors)
{
  CSparseSymmetricalMatrix<double> m;
  m.resize(2);
  m(0, 1) = 3.0;

  CSparseSymmetricalMatrix<double> m2(m);
  EXPECT_EQ(m2(1, 0), 3.0);

  CSparseMatrixTemplate<double> base(2, 2);
  base(0, 1) = 4.0;
  CSparseSymmetricalMatrix<double> m3(base);
  EXPECT_EQ(m3(1, 0), 4.0);
}

TEST(CSparseSymmetricalMatrix, OutOfRangeThrows)
{
  CSparseSymmetricalMatrix<double> m;
  m.resize(2);
  EXPECT_THROW(m(0, 5) = 1.0, std::exception);
}
