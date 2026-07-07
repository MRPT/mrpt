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
#include <mrpt/graphs/CGraphPartitioner.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/math/CMatrixF.h>

using namespace mrpt::graphs;
using namespace mrpt::math;

namespace
{
/** Builds a 4-node adjacency matrix with two clearly-separated clusters:
 * {0,1} and {2,3}, with a weak link between them. */
CMatrixDouble buildTwoClusterMatrix()
{
  CMatrixDouble A;
  A.setSize(4, 4);
  A.setZero();
  A(0, 1) = A(1, 0) = 1.0;
  A(2, 3) = A(3, 2) = 1.0;
  A(1, 2) = A(2, 1) = 0.001;
  for (int i = 0; i < 4; i++) A(i, i) = 1.0;
  return A;
}
}  // namespace

TEST(CGraphPartitioner, NCutOfClearClusters)
{
  const CMatrixDouble A = buildTwoClusterMatrix();
  std::vector<uint32_t> p1{0, 1};
  std::vector<uint32_t> p2{2, 3};

  const double ncut = CGraphPartitioner<CMatrixDouble>::nCut(A, p1, p2);
  EXPECT_GT(ncut, 0.0);
  EXPECT_LT(ncut, 0.2);
}

TEST(CGraphPartitioner, NCutZeroWhenNoCrossEdges)
{
  CMatrixDouble A;
  A.setSize(4, 4);
  A.setZero();
  A(0, 1) = A(1, 0) = 1.0;
  A(2, 3) = A(3, 2) = 1.0;

  std::vector<uint32_t> p1{0, 1};
  std::vector<uint32_t> p2{2, 3};
  const double ncut = CGraphPartitioner<CMatrixDouble>::nCut(A, p1, p2);
  EXPECT_DOUBLE_EQ(ncut, 0.0);
}

TEST(CGraphPartitioner, ExactBisectionFindsClusters)
{
  CMatrixDouble A = buildTwoClusterMatrix();
  std::vector<uint32_t> part1, part2;
  double cutValue;

  CGraphPartitioner<CMatrixDouble>::exactBisection(
      A, part1, part2, cutValue, /*forceSimetry=*/true);

  ASSERT_EQ(part1.size() + part2.size(), 4U);
  EXPECT_LT(cutValue, 0.2);

  // {0,1} must end up together, and so must {2,3}:
  auto sameGroup = [&](uint32_t a, uint32_t b)
  {
    auto& g = (std::find(part1.begin(), part1.end(), a) != part1.end()) ? part1 : part2;
    return std::find(g.begin(), g.end(), b) != g.end();
  };
  EXPECT_TRUE(sameGroup(0, 1));
  EXPECT_TRUE(sameGroup(2, 3));
}

TEST(CGraphPartitioner, SpectralBisectionFindsClusters)
{
  CMatrixDouble A = buildTwoClusterMatrix();
  std::vector<uint32_t> part1, part2;
  double cutValue;

  CGraphPartitioner<CMatrixDouble>::SpectralBisection(A, part1, part2, cutValue);

  EXPECT_EQ(part1.size() + part2.size(), 4U);
  EXPECT_GE(cutValue, 0.0);
}

TEST(CGraphPartitioner, SpectralBisectionFloatMatrix)
{
  CMatrixFloat A;
  A.setSize(4, 4);
  A.setZero();
  A(0, 1) = A(1, 0) = 1.0f;
  A(2, 3) = A(3, 2) = 1.0f;
  A(1, 2) = A(2, 1) = 0.001f;
  for (int i = 0; i < 4; i++) A(i, i) = 1.0f;

  std::vector<uint32_t> part1, part2;
  float cutValue;
  CGraphPartitioner<CMatrixFloat, float>::SpectralBisection(A, part1, part2, cutValue);
  EXPECT_EQ(part1.size() + part2.size(), 4U);
}

TEST(CGraphPartitioner, RecursiveSpectralPartitionSingleNode)
{
  CMatrixDouble A;
  A.setSize(1, 1);
  A(0, 0) = 1.0;

  std::vector<std::vector<uint32_t>> parts;
  CGraphPartitioner<CMatrixDouble>::RecursiveSpectralPartition(A, parts);

  ASSERT_EQ(parts.size(), 1U);
  ASSERT_EQ(parts[0].size(), 1U);
  EXPECT_EQ(parts[0][0], 0U);
}

TEST(CGraphPartitioner, RecursiveSpectralPartitionForcedBisectionOnly)
{
  CMatrixDouble A = buildTwoClusterMatrix();
  std::vector<std::vector<uint32_t>> parts;

  CGraphPartitioner<CMatrixDouble>::RecursiveSpectralPartition(
      A, parts, /*threshold_Ncut=*/1.0, /*forceSimetry=*/true, /*useSpectralBisection=*/true,
      /*recursive=*/false);

  ASSERT_EQ(parts.size(), 2U);
  size_t total = 0;
  for (const auto& p : parts) total += p.size();
  EXPECT_EQ(total, 4U);
}

TEST(CGraphPartitioner, RecursiveSpectralPartitionRecursiveExactBisection)
{
  CMatrixDouble A = buildTwoClusterMatrix();
  std::vector<std::vector<uint32_t>> parts;

  CGraphPartitioner<CMatrixDouble>::RecursiveSpectralPartition(
      A, parts, /*threshold_Ncut=*/1.0, /*forceSimetry=*/true, /*useSpectralBisection=*/false,
      /*recursive=*/true, /*minSizeClusters=*/1, /*verbose=*/true);

  size_t total = 0;
  for (const auto& p : parts) total += p.size();
  EXPECT_EQ(total, 4U);
}

TEST(CGraphPartitioner, RecursiveSpectralPartitionRejectsWithHighMinSize)
{
  // With a minimum cluster size larger than any achievable split, the
  // partitioner must give up and return everything as a single group:
  CMatrixDouble A = buildTwoClusterMatrix();
  std::vector<std::vector<uint32_t>> parts;

  CGraphPartitioner<CMatrixDouble>::RecursiveSpectralPartition(
      A, parts, /*threshold_Ncut=*/1.0, /*forceSimetry=*/true, /*useSpectralBisection=*/true,
      /*recursive=*/true, /*minSizeClusters=*/10);

  ASSERT_EQ(parts.size(), 1U);
  EXPECT_EQ(parts[0].size(), 4U);
}

TEST(CGraphPartitioner, NonSquareMatrixThrows)
{
  CMatrixDouble A;
  A.setSize(2, 3);

  std::vector<uint32_t> p1, p2;
  double cutValue;
  EXPECT_THROW(
      CGraphPartitioner<CMatrixDouble>::SpectralBisection(A, p1, p2, cutValue), std::exception);
  EXPECT_THROW(
      CGraphPartitioner<CMatrixDouble>::exactBisection(A, p1, p2, cutValue), std::exception);

  std::vector<std::vector<uint32_t>> parts;
  EXPECT_THROW(
      CGraphPartitioner<CMatrixDouble>::RecursiveSpectralPartition(A, parts), std::exception);
}

TEST(CGraphPartitioner, CMatrixDLegacyTypedefWorks)
{
  CMatrixD A;
  A.setSize(2, 2);
  A(0, 0) = 1.0;
  A(0, 1) = 0.0;
  A(1, 0) = 0.0;
  A(1, 1) = 1.0;

  std::vector<uint32_t> p1, p2;
  double cutValue;
  CGraphPartitioner<CMatrixD>::exactBisection(A, p1, p2, cutValue);
  EXPECT_EQ(p1.size() + p2.size(), 2U);
}
