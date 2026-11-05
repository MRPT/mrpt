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
#include <mrpt/math/distributions.h>
#include <mrpt/slam/data_association.h>

#include <Eigen/Dense>
#include <cmath>
#include <map>
#include <string>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::math;
using namespace std;

TEST(DataAssociation, TestNoICs)
{
  // Try to do DA when no individual compatible pairings exist.
  // Based on test proposed by Mauricio Soto Alvarez:
  // See:
  // https://sourceforge.net/tracker/?func=detail&aid=3562885&group_id=205280&atid=993006

  CMatrixDouble y, y_cov, z;
  y.setSize(1, 1);
  y_cov.setSize(1, 1);
  z.setSize(1, 1);

  y(0, 0) = 0.0;
  y_cov(0, 0) = 1.0;
  z(0, 0) = 10.0;

  const TDataAssociationMethod dams[2] = {assocNN, assocJCBB};
  const TDataAssociationMetric damets[2] = {metricMaha, metricML};

  for (unsigned int da_metric = 0; da_metric < sizeof(damets) / sizeof(damets[0]); ++da_metric)
  {
    const TDataAssociationMetric damet = damets[da_metric];

    for (unsigned int da_method = 0; da_method < sizeof(dams) / sizeof(dams[0]); ++da_method)
    {
      const TDataAssociationMethod dam = dams[da_method];

      TDataAssociationResults DAresults;
      data_association_independent_predictions(
          z, y, y_cov, DAresults, dam, damet, 0.99, true, std::vector<prediction_index_t>(),
          metricMaha, 0.0);

      EXPECT_EQ(0u, DAresults.associations.size())
          << "For da_method=" << da_method << " and da_metric=" << da_metric << "\n";
    }
  }
}

namespace
{
// Three predicted landmarks and three observations that match them almost
// exactly, so every association method must find all three pairings.
void buildEasyProblem(CMatrixDouble& z, CMatrixDouble& y, CMatrixDouble& y_cov)
{
  const size_t nPred = 3, lenO = 2;

  y.setSize(nPred, lenO);
  y(0, 0) = 0.0;
  y(0, 1) = 0.0;
  y(1, 0) = 10.0;
  y(1, 1) = 0.0;
  y(2, 0) = 0.0;
  y(2, 1) = 10.0;

  // Block-diagonal covariance: one 2x2 block per prediction
  y_cov.setSize(nPred * lenO, nPred * lenO);
  y_cov.setZero();
  for (size_t i = 0; i < nPred * lenO; i++) y_cov(i, i) = 0.25;

  z.setSize(nPred, lenO);
  z(0, 0) = 0.05;
  z(0, 1) = 0.0;
  z(1, 0) = 10.05;
  z(1, 1) = 0.0;
  z(2, 0) = 0.0;
  z(2, 1) = 9.95;
}
}  // namespace

TEST(DataAssociation, FullCovarianceAllMethodsAndMetrics)
{
  CMatrixDouble z, y, y_cov;
  buildEasyProblem(z, y, y_cov);

  for (const auto method : {assocNN, assocJCBB})
  {
    for (const auto metric : {metricMaha, metricML})
    {
      for (const bool useKdTree : {true, false})
      {
        TDataAssociationResults r;
        data_association_full_covariance(
            z, y, y_cov, r, method, metric, 0.99, useKdTree, std::vector<prediction_index_t>(),
            metricMaha, 0.0);

        const std::string sMsg = "method=" + std::to_string(static_cast<int>(method)) +
                                 " metric=" + std::to_string(static_cast<int>(metric)) +
                                 " kdtree=" + std::to_string(useKdTree);

        ASSERT_EQ(r.associations.size(), 3U) << sMsg;
        // Each observation matches the prediction it was built from, so the
        // pairing is the identity; a mere size check would also accept any
        // permutation of it:
        for (size_t i = 0; i < 3; i++)
        {
          const auto it = r.associations.find(i);
          ASSERT_NE(it, r.associations.end()) << sMsg << " obs #" << i;
          EXPECT_EQ(it->second, i) << sMsg << " obs #" << i;
        }
        EXPECT_EQ(r.indiv_distances.rows(), 3);
        EXPECT_EQ(r.indiv_distances.cols(), 3);
      }
    }
  }
}

TEST(DataAssociation, FullCovarianceWithMLCompatibilityTest)
{
  CMatrixDouble z, y, y_cov;
  buildEasyProblem(z, y, y_cov);

  // Individual compatibility judged by the matching likelihood instead of the
  // chi2 test on the Mahalanobis distance:
  TDataAssociationResults r;
  data_association_full_covariance(
      z, y, y_cov, r, assocJCBB, metricML, 0.99, false, std::vector<prediction_index_t>(), metricML,
      -20.0);

  EXPECT_EQ(r.associations.size(), 3U);
}

TEST(DataAssociation, FullCovarianceRemapsPredictionIDs)
{
  CMatrixDouble z, y, y_cov;
  buildEasyProblem(z, y, y_cov);

  const std::vector<prediction_index_t> ids{100, 200, 300};

  TDataAssociationResults r;
  data_association_full_covariance(
      z, y, y_cov, r, assocNN, metricMaha, 0.99, false, ids, metricMaha, 0.0);

  ASSERT_EQ(r.associations.size(), 3U);
  // The identity pairing, with the prediction indices replaced by their IDs:
  const std::map<observation_index_t, prediction_index_t> expected{
      {0, 100},
      {1, 200},
      {2, 300}
  };
  for (const auto& [obsIdx, expectedId] : expected)
  {
    const auto it = r.associations.find(obsIdx);
    ASSERT_NE(it, r.associations.end()) << "obs #" << obsIdx;
    EXPECT_EQ(it->second, expectedId) << "obs #" << obsIdx;
  }

  // A mismatched ID vector is rejected:
  TDataAssociationResults r2;
  const std::vector<prediction_index_t> badIds{1, 2};
  EXPECT_THROW(
      data_association_full_covariance(
          z, y, y_cov, r2, assocNN, metricMaha, 0.99, false, badIds, metricMaha, 0.0),
      std::exception);
}

TEST(DataAssociation, FullCovarianceArgumentChecks)
{
  CMatrixDouble z, y, y_cov;
  buildEasyProblem(z, y, y_cov);

  TDataAssociationResults r;

  // Out-of-range chi2 quantile:
  EXPECT_THROW(
      data_association_full_covariance(z, y, y_cov, r, assocNN, metricMaha, 0.0), std::exception);
  EXPECT_THROW(
      data_association_full_covariance(z, y, y_cov, r, assocNN, metricMaha, 1.0), std::exception);

  // No predictions / no observations:
  CMatrixDouble empty(0, 2);
  EXPECT_THROW(
      data_association_full_covariance(z, empty, y_cov, r, assocNN, metricMaha), std::exception);
  EXPECT_THROW(
      data_association_full_covariance(empty, y, y_cov, r, assocNN, metricMaha), std::exception);
}

// Two predictions with strongly correlated errors (as caused by the vehicle
// pose uncertainty) and observations displaced in opposite directions: each
// pairing is individually compatible, but both together are not.
TEST(DataAssociation, JCBBRejectsJointlyIncompatiblePairings)
{
  CMatrixDouble y(2, 2);
  CMatrixDouble z(2, 2);
  CMatrixDouble S(4, 4);
  y.setZero();
  y(1, 0) = 10;
  z.setZero();
  z(0, 0) = 2;
  z(1, 0) = 8;
  S.setZero();
  for (int i = 0; i < 4; i++)
  {
    S(i, i) = 1.0;
  }
  S(0, 2) = S(2, 0) = 0.99;
  S(1, 3) = S(3, 1) = 0.99;

  // Independent check of the premises: individual d2=4, joint d2=800
  ASSERT_LT(4.0, mrpt::math::chi2inv(0.99, 2));
  Eigen::Vector4d v(2.0, 0.0, -2.0, 0.0);
  const double joint_d2 = v.dot(S.asEigen().inverse() * v);
  ASSERT_GT(joint_d2, mrpt::math::chi2inv(0.99, 4));

  for (const bool useKdTree : {true, false})
  {
    TDataAssociationResults r;
    data_association_full_covariance(z, y, S, r, assocJCBB, metricMaha, 0.99, useKdTree);
    EXPECT_EQ(r.associations.size(), 1U) << "kdtree=" << useKdTree;
    EXPECT_NEAR(r.distance, 4.0, 1e-9) << "kdtree=" << useKdTree;
  }
}

// The closest prediction in Euclidean distance is very certain (huge d2),
// while a further one with a large covariance is compatible.
TEST(DataAssociation, KdTreeDoesNotSkipCompatiblePredictions)
{
  CMatrixDouble y(2, 2);
  CMatrixDouble z(1, 2);
  CMatrixDouble S(4, 4);
  y(0, 0) = 1;
  y(0, 1) = 0;
  y(1, 0) = 3;
  y(1, 1) = 0;
  z.setZero();
  S.setZero();
  S(0, 0) = S(1, 1) = 1e-4;
  S(2, 2) = S(3, 3) = 4.0;
  for (const auto method : {assocNN, assocJCBB})
  {
    for (const bool useKdTree : {false, true})
    {
      TDataAssociationResults r;
      data_association_full_covariance(z, y, S, r, method, metricMaha, 0.99, useKdTree);
      ASSERT_EQ(r.associations.size(), 1U) << "method=" << method << " kdtree=" << useKdTree;
      EXPECT_EQ(r.associations.begin()->second, 1U);
      EXPECT_NEAR(r.indiv_distances(1, 0), 9.0 / 4.0, 1e-9);
    }
  }
}

// The joint matching likelihood is the Gaussian density of the joint
// innovation, of dimension (number of pairings) x (observation length).
TEST(DataAssociation, JCBBMatchingLikelihoodIsTheJointDensity)
{
  CMatrixDouble y(2, 2);
  CMatrixDouble z(2, 2);
  CMatrixDouble S(4, 4);
  y.setZero();
  y(1, 0) = 10;
  z = y;
  z(0, 0) += 0.1;
  z(1, 1) += 0.1;
  S.setZero();
  for (int i = 0; i < 4; i++)
  {
    S(i, i) = 0.04;
  }
  TDataAssociationResults r;
  data_association_full_covariance(z, y, S, r, assocJCBB, metricML, 0.99, false);
  ASSERT_EQ(r.associations.size(), 2U);

  // Product of two independent 2D Gaussian densities:
  const double pdf1 = std::exp(-0.5 * 0.01 / 0.04) / (2 * M_PI * 0.04);
  EXPECT_NEAR(r.distance, pdf1 * pdf1, 1e-9 * pdf1 * pdf1);
}

// Bearings observed at +179.9 deg of a landmark predicted at -179.9 deg
TEST(DataAssociation, AngularComponentsAreWrapped)
{
  CMatrixDouble y(2, 2);
  CMatrixDouble z(2, 2);
  CMatrixDouble S(4, 4);
  y(0, 0) = 5.0;
  y(0, 1) = mrpt::DEG2RAD(-179.9);
  y(1, 0) = 3.0;
  y(1, 1) = mrpt::DEG2RAD(90.0);
  z(0, 0) = 5.01;
  z(0, 1) = mrpt::DEG2RAD(179.9);
  z(1, 0) = 3.01;
  z(1, 1) = mrpt::DEG2RAD(90.1);
  S.setZero();
  for (int i = 0; i < 2; i++)
  {
    S(2 * i, 2 * i) = mrpt::square(0.05);
    S(2 * i + 1, 2 * i + 1) = mrpt::square(mrpt::DEG2RAD(1.0));
  }
  const std::vector<bool> angular = {false, true};

  for (const auto method : {assocNN, assocJCBB})
  {
    TDataAssociationResults r;
    data_association_full_covariance(z, y, S, r, angular, method, metricMaha, 0.99, true);
    ASSERT_EQ(r.associations.size(), 2U) << "method=" << method;
    EXPECT_EQ(r.associations.at(0), 0U);
    EXPECT_EQ(r.associations.at(1), 1U);

    // Without the angular information, the first pairing is rejected:
    TDataAssociationResults r2;
    data_association_full_covariance(z, y, S, r2, method, metricMaha, 0.99, true);
    EXPECT_EQ(r2.associations.size(), 1U) << "method=" << method;
  }
}
