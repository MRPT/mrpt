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

#include <mrpt/math/KDTreeCapable.h>  // For kd-tree's
#include <mrpt/math/data_utils.h>
#include <mrpt/math/distributions.h>  // for chi2inv
#include <mrpt/math/ops_matrices.h>   // extractSubmatrix
#include <mrpt/math/wrap2pi.h>
#include <mrpt/poses/CPoint2DPDFGaussian.h>
#include <mrpt/poses/CPointPDFGaussian.h>
#include <mrpt/slam/data_association.h>

#include <Eigen/Dense>
#include <algorithm>
#include <memory>         // unique_ptr
#include <nanoflann.hpp>  // For kd-tree's
#include <set>

/*
   For all data association algorithms, the individual compatibility is
   estabished by
   the chi2 test (Mahalanobis distance), but later on one of the potential
   pairings is
   picked by the metric given by the user (maha vs. match. lik.)

   Related papers:
  - Matching likelihood. See:  https://www.mrpt.org/Paper:Matching_Likelihood

  - JCBB: Joint Compatibility Branch & Bound \cite neira2001data

*/

using namespace std;
using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::slam;

namespace mrpt::slam
{
struct TAuxDataRecursiveJCBB
{
  /** Just to avoid recomputing them all the time. */
  size_t nPredictions, nObservations, length_O;
  std::map<size_t, size_t> currentAssociation;
  /** Quantile for the joint compatibility chi2 test */
  double chi2quantile = 0.99;
  /** Observation components which are angles (may be empty) */
  const std::vector<bool>* angular_components = nullptr;
};

namespace
{
/** Wraps to [-pi,pi] the innovation components which are angles */
template <class VECTOR>
void wrapAngularComponents(VECTOR& v, const std::vector<bool>* angular, size_t length_O)
{
  if (angular == nullptr || angular->empty())
  {
    return;
  }
  const size_t n = static_cast<size_t>(v.size());
  for (size_t i = 0; i < n; i++)
  {
    if ((*angular)[i % length_O])
    {
      v[i] = mrpt::math::wrapToPi(v[i]);
    }
  }
}
}  // namespace

/**  Computes the joint distance metric (squared Mahalanobis distance, or the
 * logarithm of the matching likelihood) of a set of associations.
 *
 * On "currentAssociation":  maps "ID_obs" -> "ID_pred"
 *  For each landmark ID in the observations (ID_obs), its association
 *  in the predictions, that is: ID_pred = associations[ID_obs]
 *
 */
template <typename T, TDataAssociationMetric METRIC>
double joint_pdf_metric(
    const CMatrixDynamic<T>& Z_observations_mean,
    const CMatrixDynamic<T>& Y_predictions_mean,
    const CMatrixDynamic<T>& Y_predictions_cov,
    const TAuxDataRecursiveJCBB& info,
    [[maybe_unused]] const TDataAssociationResults& aux_data)
{
  // Make a list of the indices of the predictions that appear in
  // "currentAssociation":
  const size_t N = info.currentAssociation.size();
  ASSERT_(N > 0);
  std::vector<size_t> indices_pred(N);  // Appearance order indices in the std::maps
  std::vector<size_t> indices_obs(N);

  {
    size_t i = 0;
    for (auto it : info.currentAssociation)
    {
      indices_obs[i] = it.first;
      indices_pred[i] = it.second;
      i++;
    }
  }

  // ----------------------------------------------------------------------
  // Extract submatrix of the covariances involved here:
  //  COV = PREDICTIONS_COV(INDX,INDX) + OBSERVATIONS_COV(INDX2,INDX2)
  // ----------------------------------------------------------------------
  CMatrixDynamic<T> COV;
  mrpt::math::extractSubmatrixSymmetricalBlocksDyn(
      Y_predictions_cov,
      info.length_O,  // dims of cov. submatrices
      indices_pred, COV);

  // ----------------------------------------------------------------------
  // Mean:
  // The same for the vector of "errors" or "innovation" between predictions
  // and observations:
  // ----------------------------------------------------------------------
  Eigen::Matrix<T, Eigen::Dynamic, 1> innovations(N * info.length_O);
  T* dst_ptr = &innovations[0];
  for (auto it = info.currentAssociation.begin(); it != info.currentAssociation.end(); ++it)
  {
    const T* pred_i_mean = &Y_predictions_mean(it->second, 0);
    const T* obs_i_mean = &Z_observations_mean(it->first, 0);

    for (unsigned int k = 0; k < info.length_O; k++)
    {
      *dst_ptr++ = pred_i_mean[k] - obs_i_mean[k];
    }
  }
  wrapAngularComponents(innovations, info.angular_components, info.length_O);

  // Compute mahalanobis distance squared:
  const CMatrixDynamic<T> COV_inv = COV.inverse_LLt();

  const double d2 = mrpt::math::multiply_HtCH_scalar(innovations, COV_inv);

  if (METRIC == metricMaha) return d2;

  ASSERT_(METRIC == metricML);

  // Matching likelihood: The evaluation at 0 of the PDF of the difference
  // between the two Gaussians, in log scale to avoid underflow. Its dimension
  // is that of the whole joint innovation vector:
  const double dim = static_cast<double>(innovations.size());
  const double log_ml = -0.5 * (d2 + dim * std::log(M_2PI) + std::log(COV.det()));
  return log_ml;
}

template <TDataAssociationMetric METRIC>
bool isCloser(const double v1, const double v2);

template <>
bool isCloser<metricMaha>(const double v1, const double v2)
{
  return v1 < v2;
}

template <>
bool isCloser<metricML>(const double v1, const double v2)
{
  return v1 > v2;
}

/* Based on MATLAB code by:
  University of Zaragoza
  Centro Politecnico Superior
  Robotics and Real Time Group
  Authors of the original MATLAB code:  J. Neira, J. Tardos
  C++ version: J.L. Blanco Claraco
*/
template <typename T, TDataAssociationMetric METRIC>
void JCBB_recursive(
    const mrpt::math::CMatrixDynamic<T>& Z_observations_mean,
    const mrpt::math::CMatrixDynamic<T>& Y_predictions_mean,
    const mrpt::math::CMatrixDynamic<T>& Y_predictions_cov,
    TDataAssociationResults& results,
    const TAuxDataRecursiveJCBB& info,
    const observation_index_t curObsIdx)
{
  // End of iteration?
  if (curObsIdx >= info.nObservations)
  {
    if (info.currentAssociation.size() > results.associations.size())
    {
      // It's a better choice since more features are matched.
      results.associations = info.currentAssociation;
      results.distance = joint_pdf_metric<T, METRIC>(
          Z_observations_mean, Y_predictions_mean, Y_predictions_cov, info, results);
    }
    else if (
        !info.currentAssociation.empty() &&
        info.currentAssociation.size() == results.associations.size())
    {
      // The same # of features matched than the previous best one...
      // decide by better distance:
      const double d2 = joint_pdf_metric<T, METRIC>(
          Z_observations_mean, Y_predictions_mean, Y_predictions_cov, info, results);

      if (isCloser<METRIC>(d2, results.distance))
      {
        results.associations = info.currentAssociation;
        results.distance = d2;
      }
    }
  }
  else  // A normal iteration:
  {
    // Iterate for all compatible landmarsk of "curObsIdx"
    const observation_index_t obsIdx = curObsIdx;

    const size_t nPreds = results.indiv_compatibility.rows();

    // Can we do it better than the current "results.associations"?
    // This can be checked by counting the potential new pairings+the so-far
    // established ones: the number of remaining observations with at least
    // one individually compatible prediction.
    //    Matlab: potentials  = pairings(compatibility.AL(i+1:end))
    const size_t potentials = static_cast<size_t>(std::count_if(
        results.indiv_compatibility_counts.begin() + (obsIdx + 1),
        results.indiv_compatibility_counts.end(), [](uint32_t c) { return c > 0; }));
    for (prediction_index_t predIdx = 0; predIdx < nPreds; predIdx++)
    {
      // Pairing this observation adds one to the potential pairings. Ties are
      // also explored, since they are decided by the joint distance:
      if ((info.currentAssociation.size() + 1 + potentials) >= results.associations.size())
      {
        // Only if predIdx is NOT already assigned:
        if (results.indiv_compatibility(predIdx, obsIdx))
        {
          // Only if predIdx is NOT already assigned:
          bool already_asigned = false;
          for (auto itS : info.currentAssociation)
          {
            if (itS.second == predIdx)
            {
              already_asigned = true;
              break;
            }
          }

          if (!already_asigned)
          {
            // Launch a new recursive line for this hypothesis, only if the
            // extended hypothesis is jointly compatible:
            TAuxDataRecursiveJCBB new_info = info;
            new_info.currentAssociation[curObsIdx] = predIdx;

            const double joint_d2 = joint_pdf_metric<T, metricMaha>(
                Z_observations_mean, Y_predictions_mean, Y_predictions_cov, new_info, results);
            const auto dof =
                static_cast<unsigned int>(info.length_O * new_info.currentAssociation.size());

            if (joint_d2 < mrpt::math::chi2inv(info.chi2quantile, dof))
            {
              results.nNodesExploredInJCBB++;

              JCBB_recursive<T, METRIC>(
                  Z_observations_mean, Y_predictions_mean, Y_predictions_cov, results, new_info,
                  curObsIdx + 1);
            }
          }
        }
      }
    }

    // Can we do it better than the current "results.associations"?
    if ((info.currentAssociation.size() + potentials) >= results.associations.size())
    {
      // Yes we can </obama>

      // star node: Ei not paired
      results.nNodesExploredInJCBB++;
      JCBB_recursive<T, METRIC>(
          Z_observations_mean, Y_predictions_mean, Y_predictions_cov, results, info, curObsIdx + 1);
    }
  }
}

}  // namespace mrpt::slam

// See the documentation of all the parameters in the header file.
void mrpt::slam::data_association_full_covariance(
    const mrpt::math::CMatrixDouble& Z_observations_mean,
    const mrpt::math::CMatrixDouble& Y_predictions_mean,
    const mrpt::math::CMatrixDouble& Y_predictions_cov,
    TDataAssociationResults& results,
    const TDataAssociationMethod method,
    const TDataAssociationMetric metric,
    const double chi2quantile,
    const bool DAT_ASOC_USE_KDTREE,
    const std::vector<prediction_index_t>& predictions_IDs,
    const TDataAssociationMetric compatibilityTestMetric,
    const double log_ML_compat_test_threshold)
{
  data_association_full_covariance(
      Z_observations_mean, Y_predictions_mean, Y_predictions_cov, results, std::vector<bool>(),
      method, metric, chi2quantile, DAT_ASOC_USE_KDTREE, predictions_IDs, compatibilityTestMetric,
      log_ML_compat_test_threshold);
}

void mrpt::slam::data_association_full_covariance(
    const mrpt::math::CMatrixDouble& Z_observations_mean,
    const mrpt::math::CMatrixDouble& Y_predictions_mean,
    const mrpt::math::CMatrixDouble& Y_predictions_cov,
    TDataAssociationResults& results,
    const std::vector<bool>& angular_components,
    const TDataAssociationMethod method,
    const TDataAssociationMetric metric,
    const double chi2quantile,
    const bool use_kd_tree,
    const std::vector<prediction_index_t>& predictions_IDs,
    const TDataAssociationMetric compatibilityTestMetric,
    const double log_ML_compat_test_threshold)
{
  // For details on the theory, see the papers cited at the beginning of this
  // file.

  using nanoflann::KDTreeEigenMatrixAdaptor;

  MRPT_START

  results.clear();

  const size_t nPredictions = Y_predictions_mean.rows();
  const size_t nObservations = Z_observations_mean.rows();

  const size_t length_O = Z_observations_mean.cols();

  ASSERT_(nPredictions != 0);
  ASSERT_(nObservations != 0);
  ASSERT_(length_O == static_cast<size_t>(Y_predictions_mean.cols()));
  ASSERT_(length_O * nPredictions == static_cast<size_t>(Y_predictions_cov.rows()));
  ASSERT_(Y_predictions_cov.isSquare());
  ASSERT_(chi2quantile > 0 && chi2quantile < 1);
  ASSERT_(metric == metricMaha || metric == metricML);
  ASSERT_(angular_components.empty() || angular_components.size() == length_O);
  const double chi2thres = mrpt::math::chi2inv(chi2quantile, static_cast<unsigned int>(length_O));

  // The kd-tree orders predictions by Euclidean distance, which is not
  // meaningful for angular components near the +-pi discontinuity:
  const bool hasAngles = std::find(angular_components.begin(), angular_components.end(), true) !=
                         angular_components.end();
  const bool DAT_ASOC_USE_KDTREE = use_kd_tree && !hasAngles;

  // ------------------------------------------------------------
  // Build a KD-tree of the predictions for quick look-up:
  // ------------------------------------------------------------
  using KDTreeMatrixPtr = std::unique_ptr<KDTreeEigenMatrixAdaptor<CMatrixDouble>>;
  KDTreeMatrixPtr kd_tree;
  const size_t N_KD_RESULTS = nPredictions;
  std::vector<double> kd_result_distances(DAT_ASOC_USE_KDTREE ? N_KD_RESULTS : 0);
  std::vector<CMatrixDouble::Index> kd_result_indices(DAT_ASOC_USE_KDTREE ? N_KD_RESULTS : 0);
  std::vector<double> kd_queryPoint(DAT_ASOC_USE_KDTREE ? length_O : 0);

  // Upper bound of the largest eigenvalue of all the prediction covariances,
  // such that d2 >= |z-y|^2 / max_trace for any prediction:
  double max_trace = 0;

  if (DAT_ASOC_USE_KDTREE)
  {
    // Construct kd-tree for the predictions:
    kd_tree =
        std::make_unique<KDTreeEigenMatrixAdaptor<CMatrixDouble>>(length_O, Y_predictions_mean);

    for (size_t i = 0; i < nPredictions; ++i)
    {
      const size_t idx = i * length_O;
      max_trace = std::max(
          max_trace, Y_predictions_cov.asEigen().block(idx, idx, length_O, length_O).trace());
    }
  }

  // Initialize with the worst possible distance:
  results.distance = (metric == metricML) ? 0 : std::numeric_limits<double>::max();

  //-------------------------------------------
  // Compute the individual compatibility:
  //-------------------------------------------
  results.indiv_distances.resize(nPredictions, nObservations);
  results.indiv_compatibility.setSize(nPredictions, nObservations);
  results.indiv_compatibility_counts.assign(nObservations, 0);

  results.indiv_distances.fill(
      metric == metricMaha ? 1000 /*A very large Sq. Maha. Dist. */
                           : -1000 /*A very small log-likelihoo   */);
  results.indiv_compatibility.fill(false);

  CMatrixDouble pred_i_cov(length_O, length_O);

  Eigen::VectorXd diff_means_i_j(length_O);

  for (size_t j = 0; j < nObservations; ++j)
  {
    if (!DAT_ASOC_USE_KDTREE)
    {
      // Compute all the distances w/o a KD-tree
      for (size_t i = 0; i < nPredictions; ++i)
      {
        // Evaluate sqr. mahalanobis distance of obs_j -> pred_i:
        // Extract the submatrix from the diagonal:
        const size_t pred_cov_idx = i * length_O;
        pred_i_cov =
            Y_predictions_cov.asEigen().block(pred_cov_idx, pred_cov_idx, length_O, length_O);

        for (size_t k = 0; k < length_O; k++)
        {
          diff_means_i_j[k] = Z_observations_mean(j, k) - Y_predictions_mean(i, k);
        }
        wrapAngularComponents(diff_means_i_j, &angular_components, length_O);

        double d2;
        double ml;
        mrpt::math::mahalanobisDistanceSqAndLogPDF(diff_means_i_j, pred_i_cov, d2, ml);

        // The distance according to the metric
        double val = (metric == metricMaha) ? d2 : ml;

        results.indiv_distances(i, j) = val;

        // Individual compatibility
        const bool IC = (compatibilityTestMetric == metricML) ? (ml > log_ML_compat_test_threshold)
                                                              : (d2 < chi2thres);
        results.indiv_compatibility(i, j) = IC;
        if (IC) results.indiv_compatibility_counts[j]++;
      }
    }
    else
    {
      // Use a kd-tree and compute only the N closest ones:
      for (size_t k = 0; k < length_O; k++) kd_queryPoint[k] = Z_observations_mean(j, k);

      kd_tree->query(
          &kd_queryPoint[0], N_KD_RESULTS, &kd_result_indices[0], &kd_result_distances[0]);

      // Only compute the distances for these ones:
      for (size_t w = 0; w < N_KD_RESULTS; w++)
      {
        const size_t i = kd_result_indices[w];  // This is the index of
        // the prediction in
        // "predictions_mean"

        // Build the PDF of the prediction:
        // Extract the submatrix from the diagonal:
        const size_t pred_cov_idx = i * length_O;
        pred_i_cov =
            Y_predictions_cov.asEigen().block(pred_cov_idx, pred_cov_idx, length_O, length_O);

        // Since the kd-tree returns the predictions sorted by Euclidean
        // distance, once it implies d2 > chi2thres for any covariance, no
        // further prediction can pass the chi2 test:
        if (compatibilityTestMetric == metricMaha && kd_result_distances[w] > chi2thres * max_trace)
        {
          break;
        }

        for (size_t k = 0; k < length_O; k++)
        {
          diff_means_i_j[k] = Z_observations_mean(j, k) - Y_predictions_mean(i, k);
        }

        double d2;
        double ml;
        mrpt::math::mahalanobisDistanceSqAndLogPDF(diff_means_i_j, pred_i_cov, d2, ml);

        // The distance according to the metric
        double val = (metric == metricMaha) ? d2 : ml;

        results.indiv_distances(i, j) = val;

        // Individual compatibility
        const bool IC = (compatibilityTestMetric == metricML) ? (ml > log_ML_compat_test_threshold)
                                                              : (d2 < chi2thres);
        results.indiv_compatibility(i, j) = IC;
        if (IC) results.indiv_compatibility_counts[j]++;
      }
    }  // end use KD-Tree
  }    // end for

#if 0
	cout << "Distances: " << endl << results.indiv_distances << "\n";
	//cout << "indiv compat: " << endl << results.indiv_compatibility << "\n";
#endif

  // Do associations:
  results.associations.clear();

  switch (method)
  {
    // --------------------------
    // Nearest-neighbor
    // --------------------------
    case assocNN:
    {
      // 1) For each observation "j", make a list of the indiv. compatible
      //     predictions and their distances, sorted first the best.
      //     NOTE: distances are saved so smaller is always better,
      //            hence "metricML" are made negative.
      // -------------------------------------------------------------------
      using TListAllICs =
          multimap<double, pair<observation_index_t, multimap<double, prediction_index_t>>>;
      TListAllICs lst_all_ICs;

      for (observation_index_t j = 0; j < nObservations; ++j)
      {
        multimap<double, prediction_index_t> ICs;

        for (prediction_index_t i = 0; i < nPredictions; ++i)
        {
          if (results.indiv_compatibility(i, j))
          {
            double d2 = results.indiv_distances(i, j);
            if (metric == metricML) d2 = -d2;
            ICs.insert(make_pair(d2, i));
          }
        }

        if (!ICs.empty())
        {
          const double best_dist = ICs.begin()->first;
          lst_all_ICs.insert(make_pair(best_dist, make_pair(j, ICs)));
        }
      }

      // 2) With that lists, start by the best one and make the
      // assignment.
      //    Remove the prediction from the list of available, and go on.
      // --------------------------------------------------------------------
      std::set<prediction_index_t> lst_already_taken_preds;

      for (auto it = lst_all_ICs.begin(); it != lst_all_ICs.end(); ++it)
      {
        const observation_index_t obs_id = it->second.first;
        const multimap<double, prediction_index_t>& lstCompats = it->second.second;

        for (auto lstCompat : lstCompats)
        {
          if (lst_already_taken_preds.find(lstCompat.second) == lst_already_taken_preds.end())
          {
            // It's free: make the association:
            results.associations[obs_id] = lstCompat.second;
            lst_already_taken_preds.insert(lstCompat.second);
            break;
          }
        }
      }
    }
    break;

    // ------------------------------------
    // Joint Compatibility Branch & Bound:
    // ------------------------------------
    case assocJCBB:
    {
      // Call to the recursive method:
      TAuxDataRecursiveJCBB info;
      info.nPredictions = nPredictions;
      info.nObservations = nObservations;
      info.length_O = length_O;
      info.chi2quantile = chi2quantile;
      info.angular_components = &angular_components;

      if (metric == metricMaha)
      {
        JCBB_recursive<CMatrixDouble::Scalar, metricMaha>(
            Z_observations_mean, Y_predictions_mean, Y_predictions_cov, results, info, 0);
      }
      else
      {
        JCBB_recursive<CMatrixDouble::Scalar, metricML>(
            Z_observations_mean, Y_predictions_mean, Y_predictions_cov, results, info, 0);

        // The joint metric was evaluated in log scale:
        if (!results.associations.empty())
        {
          results.distance = std::exp(results.distance);
        }
      }
    }
    break;

    default:
      THROW_EXCEPTION("Unknown value of 'method'");
  };

  // If a mapping of prediction indices to IDs was providen, apply it now:
  // ------------------------------------------------------------------------
  if (!predictions_IDs.empty())
  {
    ASSERT_(predictions_IDs.size() == nPredictions);
    for (auto& association : results.associations)
      association.second = predictions_IDs[association.second];
  }

  MRPT_END
}

/* ==================================================================================================
          data_association_independent_predictions
   ==================================================================================================
   */
void mrpt::slam::data_association_independent_predictions(
    const mrpt::math::CMatrixDouble& Z_observations_mean,
    const mrpt::math::CMatrixDouble& Y_predictions_mean,
    const mrpt::math::CMatrixDouble& Y_predictions_cov_stacked,
    TDataAssociationResults& results,
    const TDataAssociationMethod method,
    const TDataAssociationMetric metric,
    const double chi2quantile,
    const bool DAT_ASOC_USE_KDTREE,
    const std::vector<prediction_index_t>& predictions_IDs,
    const TDataAssociationMetric compatibilityTestMetric,
    const double log_ML_compat_test_threshold)
{
  MRPT_START

  results.clear();

  const size_t nPredictions = Y_predictions_mean.rows();
  const size_t nObservations = Z_observations_mean.rows();

  const size_t length_O = Z_observations_mean.cols();

  ASSERT_(nPredictions != 0);
  ASSERT_(nObservations != 0);
  ASSERT_(length_O == static_cast<size_t>(Y_predictions_mean.cols()));
  ASSERT_(length_O * nPredictions == static_cast<size_t>(Y_predictions_cov_stacked.rows()));
  ASSERT_(chi2quantile > 0 && chi2quantile < 1);
  ASSERT_(metric == metricMaha || metric == metricML);
  // const double chi2thres = mrpt::math::chi2inv( chi2quantile, length_O );

  CMatrixDouble Y_predictions_cov_full(length_O * nPredictions, length_O * nPredictions);
  CMatrixDouble COV_i(length_O, length_O);
  for (size_t i = 0; i < nPredictions; i++)
  {
    const size_t idx = i * length_O;
    COV_i = Y_predictions_cov_stacked.extractMatrix(length_O, length_O, idx, 0);
    Y_predictions_cov_full.insertMatrix(idx, idx, COV_i);
  }

  data_association_full_covariance(
      Z_observations_mean, Y_predictions_mean, Y_predictions_cov_full, results, method, metric,
      chi2quantile, DAT_ASOC_USE_KDTREE, predictions_IDs, compatibilityTestMetric,
      log_ML_compat_test_threshold);

  MRPT_END
}
