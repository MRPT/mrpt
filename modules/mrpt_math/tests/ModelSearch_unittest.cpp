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
#include <mrpt/math/model_search.h>

using namespace mrpt::math;

namespace
{
// Fits a 2D line y = a*x + b through 2 points; TModelFit interface for
// mrpt::math::ModelSearch.
struct LineModelFit
{
  using Real = double;
  struct Model
  {
    double a{0};
    double b{0};
  };

  std::vector<double> xs;
  std::vector<double> ys;

  [[nodiscard]] size_t getSampleCount() const { return xs.size(); }

  bool fitModel(const std::vector<size_t>& useIndices, Model& model) const
  {
    ASSERT_EQUAL_(useIndices.size(), 2U);
    const double x1 = xs[useIndices[0]];
    const double y1 = ys[useIndices[0]];
    const double x2 = xs[useIndices[1]];
    const double y2 = ys[useIndices[1]];
    if (std::abs(x2 - x1) < 1e-9)
    {
      return false;
    }
    model.a = (y2 - y1) / (x2 - x1);
    model.b = y1 - model.a * x1;
    return true;
  }

  [[nodiscard]] double testSample(size_t index, const Model& model) const
  {
    const double d = model.a * xs[index] - ys[index] + model.b;
    return d * d;
  }
};

LineModelFit MakeNoisyLineDataset(size_t n, double a, double b)
{
  LineModelFit fit;
  fit.xs.resize(n);
  fit.ys.resize(n);
  for (size_t i = 0; i < n; i++)
  {
    const double x = static_cast<double>(i) * 0.1;
    fit.xs[i] = x;
    fit.ys[i] = a * x + b;
  }
  return fit;
}
}  // namespace

TEST(ModelSearch, RansacSingleModel)
{
  auto fit = MakeNoisyLineDataset(40, 2.0, 1.0);

  ModelSearch ms;
  LineModelFit::Model bestModel;
  std::vector<size_t> inliers;

  const bool found = ms.ransacSingleModel(fit, 2, 1e-6, bestModel, inliers);

  ASSERT_TRUE(found);
  EXPECT_GT(inliers.size(), 0U);
  EXPECT_NEAR(bestModel.a, 2.0, 1e-3);
  EXPECT_NEAR(bestModel.b, 1.0, 1e-3);
}

TEST(ModelSearch, GeneticSingleModelWithMating)
{
  // A dataset where every pair of points defines a valid (non-degenerate)
  // model, so the population stays fully alive across generations and the
  // "mating" code path (which merges sample sets via the std::set overload
  // of pickRandomIndex) gets exercised.
  auto fit = MakeNoisyLineDataset(30, -1.0, 3.0);

  ModelSearch ms;
  LineModelFit::Model bestModel;
  std::vector<size_t> inliers;

  const bool found = ms.geneticSingleModel(
      fit, /*p_kernelSize=*/2, /*p_fitnessThreshold=*/1e-6,
      /*p_populationSize=*/12, /*p_maxIteration=*/3, bestModel, inliers);

  ASSERT_TRUE(found);
  EXPECT_GT(inliers.size(), 0U);
  EXPECT_NEAR(bestModel.a, -1.0, 1e-2);
  EXPECT_NEAR(bestModel.b, 3.0, 1e-2);
}
