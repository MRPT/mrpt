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
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/data_utils.h>
#include <mrpt/math/ops_containers.h>
#include <mrpt/math/ops_matrices.h>
#include <mrpt/math/utils.h>

#include <Eigen/Dense>
#include <cmath>
#include <cstdint>
#include <vector>

using namespace mrpt;
using namespace mrpt::math;
using namespace std;

TEST(CVectorDouble, resize)
{
  {
    mrpt::math::CVectorDouble v;
    EXPECT_TRUE(v.size() == 0);
  }

  for (int i = 0; i < 10; i++)
  {
    mrpt::math::CVectorDouble v(i);
    EXPECT_TRUE(v.size() == i);
  }

  for (int i = 0; i < 10; i++)
  {
    mrpt::math::CVectorDouble v;
    v.resize(i);
    EXPECT_TRUE(v.size() == i);
  }

  for (int i = 10; i >= 0; i--)
  {
    mrpt::math::CVectorDouble v;
    v.resize(i);
    EXPECT_TRUE(v.size() == i);
  }

  {
    mrpt::math::CVectorDouble v;
    for (int i = 0; i < 10; i++)
    {
      v.push_back(double(i));
      EXPECT_TRUE(v.size() == (i + 1));
    }
    for (int i = 0; i < 10; i++) EXPECT_NEAR(v[i], i, 1e-10);
  }
}

TEST(ops_containers, ncc_vector)
{
  {
    // Error on invalid lengths:
    const std::vector<double> s1 = {1.0, 2.0};
    const std::vector<double> s2 = {1.0, 2.0, 3.0};
    EXPECT_ANY_THROW(mrpt::math::ncc_vector(s1, s2));
  }

  {
    // Error on null mean:
    const std::vector<double> s1 = {0.0, 0.0};
    const std::vector<double> s2 = {0.0, 0.0};
    EXPECT_ANY_THROW(mrpt::math::ncc_vector(s1, s2));
  }

  // Known output test case:
  const std::vector<double> s1 = {
      1.000000, 0.840000, 0.705600, 0.592704, 0.497871, 0.418212, 0.351298, 0.295090,
      0.247876, 0.208216, 0.174901, 0.146917, 0.123410, 0.103665, 0.087078, 0.073146,
  };
  const std::vector<double> s2 = {
      1.000000, 0.920000, 0.846400, 0.778688, 0.716393, 0.659082, 0.606355, 0.557847,
      0.513219, 0.472161, 0.434388, 0.399637, 0.367666, 0.338253, 0.311193, 0.286297,
  };
  const double ncc = mrpt::math::ncc_vector(s1, s2);
  const double ncc_gt = 0.9858405444;
  EXPECT_NEAR(ncc, ncc_gt, 1e-4);
}

TEST(ops_containers, xcorr)
{
  // Known output test case:
  const std::vector<double> a = {
      1.0000000000, 0.8400000000, 0.7056000000, 0.5927040000, 0.4978713600, 0.4182119424,
      0.3512980316, 0.2950903466, 0.2478758911, 0.2082157485, 0.1749012288, 0.1469170322,
      0.1234103070, 0.1036646579, 0.0870783126, 0.0731457826,
  };
  const std::vector<double> b = {
      1.0000000000, 0.9200000000, 0.8464000000, 0.7786880000, 0.7163929600, 0.6590815232,
      0.6063550013, 0.5578466012, 0.5132188731, 0.4721613633, 0.4343884542, 0.3996373779,
      0.3676663877, 0.3382530766, 0.3111928305, 0.2862974041,
  };

  const size_t maxLag = 5;
  const std::vector<double> r = mrpt::math::xcorr(a, b, maxLag);

  // Verify that the peak is at the correct spot:
  const size_t maxCoeffIdx_gt = 5;

  ASSERT_EQUAL_(r.size(), maxLag * 2 + 1);
  for (size_t i = 0; i < r.size(); i++)
  {
    if (i == maxCoeffIdx_gt) continue;
    EXPECT_GT(r[maxCoeffIdx_gt], r[i]);
  }
}

namespace
{
// A fixed set of 3D samples (rows) used for the estimator tests.
std::vector<std::vector<double>> sampleSet()
{
  return {
      { 1.0,  2.0, -1.0},
      { 0.5,  3.0,  0.0},
      { 2.0,  1.5, -0.5},
      {-1.0,  2.5,  1.0},
      { 0.0,  0.0,  0.0},
      { 1.5, -1.0,  2.0},
      { 3.0,  2.0,  1.0},
      {-0.5,  1.0, -2.0}
  };
}

// Independent weighted mean and covariance (weights summing to 1).
void refMeanCov(
    const std::vector<std::vector<double>>& s,
    const std::vector<double>& w,
    Eigen::VectorXd& mean,
    Eigen::MatrixXd& cov)
{
  const auto n = s.size();
  const auto d = static_cast<Eigen::Index>(s[0].size());
  mean.setZero(d);
  cov.setZero(d, d);
  for (size_t k = 0; k < n; k++)
  {
    for (Eigen::Index i = 0; i < d; i++)
    {
      mean[i] += w[k] * s[k][static_cast<size_t>(i)];
    }
  }
  for (size_t k = 0; k < n; k++)
  {
    Eigen::VectorXd e(d);
    for (Eigen::Index i = 0; i < d; i++)
    {
      e[i] = s[k][static_cast<size_t>(i)] - mean[i];
    }
    cov += w[k] * e * e.transpose();
  }
}
}  // namespace

TEST(ops_containers, covariancesAndMean_IsBiasedMLEstimator)
{
  const auto s = sampleSet();
  const std::vector<double> w(s.size(), 1.0 / static_cast<double>(s.size()));
  Eigen::VectorXd refMean;
  Eigen::MatrixXd refCov;
  refMeanCov(s, w, refMean, refCov);

  CMatrixDouble C;
  CVectorDouble m;
  mrpt::math::covariancesAndMean(s, C, m);
  ASSERT_EQ(m.size(), 3);
  for (int i = 0; i < 3; i++)
  {
    EXPECT_NEAR(m[i], refMean[i], 1e-12);
    for (int j = 0; j < 3; j++)
    {
      EXPECT_NEAR(C(i, j), refCov(i, j), 1e-12);
    }
  }

  // meanAndCovVec(), meanAndCovMat() and cov() use the same normalization:
  std::vector<double> m1;
  CMatrixDouble C1;
  mrpt::math::meanAndCovVec(s, m1, C1);
  CMatrixDouble M(static_cast<int>(s.size()), 3);
  for (size_t k = 0; k < s.size(); k++)
  {
    for (int i = 0; i < 3; i++)
    {
      M(static_cast<int>(k), i) = s[k][static_cast<size_t>(i)];
    }
  }
  CVectorDouble m2;
  CMatrixDouble C2;
  mrpt::math::meanAndCovMat(M, m2, C2);
  const CMatrixDouble C3 = mrpt::math::cov(M);
  for (int i = 0; i < 3; i++)
  {
    EXPECT_NEAR(m1[static_cast<size_t>(i)], refMean[i], 1e-12);
    EXPECT_NEAR(m2[i], refMean[i], 1e-12);
    for (int j = 0; j < 3; j++)
    {
      EXPECT_NEAR(C1(i, j), refCov(i, j), 1e-12);
      EXPECT_NEAR(C2(i, j), refCov(i, j), 1e-12);
      EXPECT_NEAR(C3(i, j), refCov(i, j), 1e-12);
    }
  }
}

TEST(ops_containers, covariancesAndMeanWeighted_NormalizedWeights)
{
  const auto s = sampleSet();
  std::vector<double> w = {0.05, 0.2, 0.1, 0.15, 0.05, 0.25, 0.1, 0.1};
  Eigen::VectorXd refMean;
  Eigen::MatrixXd refCov;
  refMeanCov(s, w, refMean, refCov);

  CMatrixDouble C;
  CVectorDouble m;
  mrpt::math::covariancesAndMeanWeighted(s, C, m, &w, &w);
  for (int i = 0; i < 3; i++)
  {
    EXPECT_NEAR(m[i], refMean[i], 1e-12);
    for (int j = 0; j < 3; j++)
    {
      EXPECT_NEAR(C(i, j), refCov(i, j), 1e-12);
    }
  }
}

TEST(ops_containers, covariancesAndMean_WrapOnlyAngularDimension)
{
  const double phis[] = {3.0, -3.0, 3.0, -3.0};
  const double xs[] = {-10.0, 10.0, -10.0, 10.0};
  // Expected values from the wrapped angular deviations +/-(pi-3):
  const double dphi = M_PI - 3.0;
  const double varPhi = dphi * dphi;
  const double covXPhi = 10.0 * dphi;
  const double varX = 100.0;

  for (int order = 0; order < 2; order++)
  {
    const int iPhi = order == 0 ? 0 : 1;
    const int iX = 1 - iPhi;
    std::vector<CVectorDouble> el;
    for (int i = 0; i < 4; i++)
    {
      CVectorDouble e(2);
      e[iPhi] = phis[i];
      e[iX] = xs[i];
      el.push_back(e);
    }
    bool wrap[2] = {false, false};
    wrap[iPhi] = true;
    CMatrixDouble C;
    CVectorDouble m;
    mrpt::math::covariancesAndMean(el, C, m, wrap);
    EXPECT_NEAR(std::abs(m[iPhi]), M_PI, 1e-9);
    EXPECT_NEAR(m[iX], 0.0, 1e-12);
    EXPECT_NEAR(C(iPhi, iPhi), varPhi, 1e-9);
    EXPECT_NEAR(C(iX, iX), varX, 1e-9);
    EXPECT_NEAR(C(0, 1), covXPhi, 1e-9);
    EXPECT_NEAR(C(1, 0), covXPhi, 1e-9);
  }
}

TEST(ops_containers, covariancesAndMean_WrappedMeanShortArc)
{
  // Two angles 1.98 rad apart across +-pi: mean on the short arc
  std::vector<CVectorDouble> el(2, CVectorDouble(1));
  el[0][0] = -1.4;
  el[1][0] = 2.9;
  const bool wrap[1] = {true};
  CMatrixDouble C;
  CVectorDouble m;
  mrpt::math::covariancesAndMean(el, C, m, wrap);
  const double halfArc = (2 * M_PI - 4.3) / 2;
  EXPECT_NEAR(m[0], -1.4 - halfArc, 1e-12);
  EXPECT_NEAR(C(0, 0), halfArc * halfArc, 1e-12);
}

TEST(ops_containers, meanAndStd_BiasedAndUnbiased)
{
  const std::vector<double> v = {2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0};
  double m = 0;
  double s = 0;
  mrpt::math::meanAndStd(v, m, s, false);
  EXPECT_NEAR(m, 5.0, 1e-12);
  EXPECT_NEAR(s, 2.0, 1e-12);
  mrpt::math::meanAndStd(v, m, s, true);
  EXPECT_NEAR(s, std::sqrt(32.0 / 7.0), 1e-12);
  EXPECT_NEAR(mrpt::math::stddev(v, true), std::sqrt(32.0 / 7.0), 1e-12);

  const std::vector<double> one = {3.5};
  mrpt::math::meanAndStd(one, m, s);
  EXPECT_DOUBLE_EQ(m, 3.5);
  EXPECT_DOUBLE_EQ(s, 0.0);
  const std::vector<double> none;
  mrpt::math::meanAndStd(none, m, s);
  EXPECT_DOUBLE_EQ(m, 0.0);
  EXPECT_DOUBLE_EQ(s, 0.0);
  EXPECT_DOUBLE_EQ(mrpt::math::mean(none), 0.0);
}

TEST(ops_containers, cumsumNormalizeMinMax)
{
  const std::vector<double> v = {1.0, -2.0, 3.5, 0.5};
  const auto c = mrpt::math::cumsum(v);
  ASSERT_EQ(c.size(), 4U);
  EXPECT_DOUBLE_EQ(c[0], 1.0);
  EXPECT_DOUBLE_EQ(c[1], -1.0);
  EXPECT_DOUBLE_EQ(c[2], 2.5);
  EXPECT_DOUBLE_EQ(c[3], 3.0);

  EXPECT_DOUBLE_EQ(mrpt::math::maximum(v), 3.5);
  EXPECT_DOUBLE_EQ(mrpt::math::minimum(v), -2.0);
  double mn = 0;
  double mx = 0;
  mrpt::math::minimum_maximum(v, mn, mx);
  EXPECT_DOUBLE_EQ(mn, -2.0);
  EXPECT_DOUBLE_EQ(mx, 3.5);

  Eigen::VectorXd e(3);
  e << 3.0, -1.0, 2.0;
  mrpt::math::minimum_maximum(e, mn, mx);
  EXPECT_DOUBLE_EQ(mn, -1.0);
  EXPECT_DOUBLE_EQ(mx, 3.0);

  CVectorDouble cv(4);
  for (int i = 0; i < 4; i++)
  {
    cv[i] = v[static_cast<size_t>(i)];
  }
  const auto cc = mrpt::math::cumsum(cv);
  ASSERT_EQ(cc.size(), 4);
  EXPECT_DOUBLE_EQ(cc[3], 3.0);

  mrpt::math::normalize(cv, 10.0, 20.0);
  EXPECT_NEAR(cv[0], 10.0 + 10.0 * 3.0 / 5.5, 1e-12);
  EXPECT_NEAR(cv[1], 10.0, 1e-12);
  EXPECT_NEAR(cv[2], 20.0, 1e-12);

  const std::vector<int> a = {1, 2, 3, 4};
  const std::vector<int> b = {3, 4, 5};
  EXPECT_EQ(mrpt::math::countCommonElements(a, b), 2U);
}

TEST(ops_containers, adjustRange)
{
  CVectorDouble v(3);
  v[0] = 0.0;
  v[1] = 1.0;
  v[2] = 2.0;
  mrpt::math::adjustRange(v, 10.0, 20.0);
  EXPECT_DOUBLE_EQ(v[0], 10.0);
  EXPECT_DOUBLE_EQ(v[1], 15.0);
  EXPECT_DOUBLE_EQ(v[2], 20.0);

  CVectorDouble k(2);
  k.fill(4.0);
  mrpt::math::adjustRange(k, -1.0, 1.0);
  EXPECT_DOUBLE_EQ(k[0], -1.0);
  EXPECT_DOUBLE_EQ(k[1], -1.0);
}

TEST(ops_containers, dotProductAndSumRetType)
{
  CVectorDouble a(3);
  CVectorDouble b(3);
  a.fill(1.0);
  b.fill(2.0);
  EXPECT_DOUBLE_EQ(mrpt::math::dotProduct(a, b), 6.0);

  mrpt::math::CVectorDynamic<uint8_t> u(3);
  u.fill(200);
  EXPECT_EQ((mrpt::math::sumRetType<mrpt::math::CVectorDynamic<uint8_t>, int>(u)), 600);
}

TEST(ops_containers, weightedHistogram)
{
  const std::vector<double> values = {0.0, 0.1, 1.0, 2.0};
  const std::vector<double> weights = {1.0, 1.0, 2.0, 4.0};
  std::vector<double> centers;
  std::vector<double> hits;
  mrpt::math::weightedHistogram(values, weights, 1.0f, centers, hits);
  ASSERT_EQ(hits.size(), 3U);
  ASSERT_EQ(centers.size(), 3U);
  EXPECT_DOUBLE_EQ(centers[0], 0.0);
  EXPECT_DOUBLE_EQ(centers[2], 2.0);
  EXPECT_NEAR(hits[0], 0.25, 1e-12);
  EXPECT_NEAR(hits[1], 0.25, 1e-12);
  EXPECT_NEAR(hits[2], 0.5, 1e-12);

  const std::vector<double> logw = {0.0, 0.0, std::log(2.0), std::log(4.0)};
  mrpt::math::weightedHistogramLog(values, logw, 1.0f, centers, hits);
  ASSERT_EQ(hits.size(), 3U);
  EXPECT_NEAR(hits[0], 0.25, 1e-12);
  EXPECT_NEAR(hits[2], 0.5, 1e-12);

  // MRPT vectors, and all values equal (a single bin):
  CVectorDouble cvals(3);
  CVectorDouble cw(3);
  cvals.fill(5.0);
  cw.fill(1.0);
  CVectorDouble cc;
  CVectorDouble ch;
  mrpt::math::weightedHistogram(cvals, cw, 0.5f, cc, ch);
  ASSERT_EQ(ch.size(), 1);
  EXPECT_DOUBLE_EQ(cc[0], 5.0);
  EXPECT_DOUBLE_EQ(ch[0], 1.0);
}

// The end point must be exact, without accumulated round-off
TEST(ContainerOps, linspaceEndpointsAreExact)
{
  std::vector<double> v;
  mrpt::math::linspace(0.0, 1.0, 11, v);
  ASSERT_EQ(v.size(), 11U);
  EXPECT_EQ(v.front(), 0.0);
  EXPECT_EQ(v.back(), 1.0);
  for (size_t i = 0; i < v.size(); i++)
  {
    EXPECT_NEAR(v[i], 0.1 * static_cast<double>(i), 1e-15);
  }
}
