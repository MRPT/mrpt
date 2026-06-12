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
#include <mrpt/math/ransac.h>
#include <mrpt/random/RandomGenerators.h>

using namespace mrpt::math;

namespace
{
// Model: a 2D line y = a*x + b, represented as a 1x2 matrix [a b].
// Fit a line through the two selected points.
void fitLine(
    const CMatrixDouble& allData, const std::vector<size_t>& useIndices,
    std::vector<CMatrixDouble>& fitModels)
{
  ASSERT_EQUAL_(useIndices.size(), 2U);

  const double x1 = allData(0, static_cast<int>(useIndices[0]));
  const double y1 = allData(1, static_cast<int>(useIndices[0]));
  const double x2 = allData(0, static_cast<int>(useIndices[1]));
  const double y2 = allData(1, static_cast<int>(useIndices[1]));

  if (std::abs(x2 - x1) < 1e-12)
  {
    // Vertical line: cannot be represented by this model, reject.
    fitModels.clear();
    return;
  }

  const double a = (y2 - y1) / (x2 - x1);
  const double b = y1 - a * x1;

  fitModels.resize(1);
  fitModels[0].setSize(1, 2);
  fitModels[0](0, 0) = a;
  fitModels[0](0, 1) = b;
}

void lineDistance(
    const CMatrixDouble& allData, const std::vector<CMatrixDouble>& testModels,
    const double distanceThreshold, unsigned int& out_bestModelIndex,
    std::vector<size_t>& out_inlierIndices)
{
  out_inlierIndices.clear();
  out_bestModelIndex = 0;
  if (testModels.empty())
  {
    return;
  }

  const double a = testModels[0](0, 0);
  const double b = testModels[0](0, 1);
  const double norm = std::sqrt(a * a + 1);

  const auto N = static_cast<size_t>(allData.cols());
  for (size_t i = 0; i < N; i++)
  {
    const double x = allData(0, static_cast<int>(i));
    const double y = allData(1, static_cast<int>(i));
    const double d = std::abs(a * x - y + b) / norm;
    if (d < distanceThreshold)
    {
      out_inlierIndices.push_back(i);
    }
  }
}

bool lineDegenerate(
    [[maybe_unused]] const CMatrixDouble& allData,
    [[maybe_unused]] const std::vector<size_t>& useIndices)
{
  return false;
}
}  // namespace

TEST(RANSAC, FitLineWithOutliers)
{
  mrpt::random::getRandomGenerator().randomize(1234);

  // Ground-truth line: y = 2*x + 1
  const double TRUE_A = 2.0;
  const double TRUE_B = 1.0;

  constexpr size_t N_INLIERS = 50;
  constexpr size_t N_OUTLIERS = 20;

  CMatrixDouble data(2, N_INLIERS + N_OUTLIERS);
  for (size_t i = 0; i < N_INLIERS; i++)
  {
    const double x = mrpt::random::getRandomGenerator().drawUniform(-10.0, 10.0);
    const double y = TRUE_A * x + TRUE_B + mrpt::random::getRandomGenerator().drawGaussian1D(0, 0.01);
    data(0, static_cast<int>(i)) = x;
    data(1, static_cast<int>(i)) = y;
  }
  for (size_t i = 0; i < N_OUTLIERS; i++)
  {
    data(0, static_cast<int>(N_INLIERS + i)) = mrpt::random::getRandomGenerator().drawUniform(-10.0, 10.0);
    data(1, static_cast<int>(N_INLIERS + i)) = mrpt::random::getRandomGenerator().drawUniform(-50.0, 50.0);
  }

  RANSAC ransac;
  std::vector<size_t> inliers;
  CMatrixDouble bestModel;

  const bool found = ransac.execute(
      data, &fitLine, &lineDistance, &lineDegenerate,
      /*distanceThreshold=*/0.5,
      /*minimumSizeSamplesToFit=*/2, inliers, bestModel,
      /*p=*/0.999,
      /*maxIter=*/200);

  ASSERT_TRUE(found);
  EXPECT_GE(inliers.size(), N_INLIERS - 5);

  ASSERT_EQ(bestModel.rows(), 1);
  ASSERT_EQ(bestModel.cols(), 2);
  EXPECT_NEAR(bestModel(0, 0), TRUE_A, 0.2);
  EXPECT_NEAR(bestModel(0, 1), TRUE_B, 0.5);
}

TEST(RANSAC, NoModelFoundWithAllDegenerateData)
{
  // A single repeated point cannot define a line: fitLine() rejects all
  // (vertical/degenerate) samples since x1 == x2 always.
  CMatrixDouble data(2, 5);
  for (int i = 0; i < 5; i++)
  {
    data(0, i) = 1.0;  // same x for all points -> vertical line, rejected by fitLine
    data(1, i) = static_cast<double>(i);
  }

  RANSAC ransac;
  std::vector<size_t> inliers;
  CMatrixDouble bestModel;

  const bool found = ransac.execute(
      data, &fitLine, &lineDistance, &lineDegenerate,
      /*distanceThreshold=*/0.1,
      /*minimumSizeSamplesToFit=*/2, inliers, bestModel,
      /*p=*/0.999,
      /*maxIter=*/50);

  EXPECT_FALSE(found);
  EXPECT_TRUE(inliers.empty());
}
