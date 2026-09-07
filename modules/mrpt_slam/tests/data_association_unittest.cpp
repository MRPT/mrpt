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
#include <mrpt/slam/data_association.h>

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

        EXPECT_EQ(r.associations.size(), 3U)
            << "method=" << static_cast<int>(method) << " metric=" << static_cast<int>(metric)
            << " kdtree=" << useKdTree;
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
  for (const auto& [obsIdx, predId] : r.associations)
  {
    (void)obsIdx;
    EXPECT_TRUE(predId == 100 || predId == 200 || predId == 300);
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
