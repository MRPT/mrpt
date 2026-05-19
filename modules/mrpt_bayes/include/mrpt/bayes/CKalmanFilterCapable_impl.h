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
#pragma once

#ifndef CKalmanFilterCapable_H
#error Include this file only from 'CKalmanFilterCapable.h'
#endif
//
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/containers/stl_containers_utils.h>
#include <mrpt/math/ops_matrices.h>  // extractSubmatrixSymmetrical()

#include <Eigen/Dense>

namespace mrpt::bayes
{
// The main entry point in the Kalman Filter class:
template <size_t VEH_SIZE, size_t OBS_SIZE, size_t FEAT_SIZE, size_t ACT_SIZE, typename KFTYPE>
void CKalmanFilterCapable<VEH_SIZE, OBS_SIZE, FEAT_SIZE, ACT_SIZE, KFTYPE>::runOneKalmanIteration()
{
  using namespace std;
  MRPT_START

  m_timLogger.enable(KF_options.enable_profiler);
  m_timLogger.enter("KF:complete_step");

  ASSERT_(int(m_xkk.size()) == m_pkk.cols());
  ASSERT_(size_t(m_xkk.size()) >= VEH_SIZE);
  // =============================================================
  //  1. CREATE ACTION MATRIX u FROM ODOMETRY
  // =============================================================
  KFArray_ACT u;

  m_timLogger.enter("KF:1.OnGetAction");
  OnGetAction(u);
  m_timLogger.leave("KF:1.OnGetAction");

  // Sanity check:
  if (FEAT_SIZE)
  {
    ASSERTDEB_((((m_xkk.size() - VEH_SIZE) / FEAT_SIZE) * FEAT_SIZE) == (m_xkk.size() - VEH_SIZE));
  }

  // =============================================================
  //  2. PREDICTION OF NEW POSE xv_{k+1|k}
  // =============================================================
  m_timLogger.enter("KF:2.prediction stage");

  const size_t N_map = getNumberOfLandmarksInTheMap();

  // Vehicle pose
  KFArray_VEH xv(&m_xkk[0]);

  bool skipPrediction = false;  // Whether to skip the prediction step (in
  // SLAM this is desired for the first
  // iteration...)

  // Update mean: xv will have the updated pose until we update it in the
  // filterState later.
  //  This is to maintain a copy of the last robot pose in the state vector,
  //  required for the Jacobian computation.
  OnTransitionModel(u, xv, skipPrediction);

  if (!skipPrediction)
  {
    // =============================================================
    //  3. PREDICTION OF COVARIANCE P_{k+1|k}
    // =============================================================
    // First, we compute de Jacobian fv_by_xv  (derivative of f_vehicle wrt
    // x_vehicle):
    KFMatrix_VxV dfv_dxv;

    // Try closed-form Jacobian first:
    m_user_didnt_implement_jacobian = false;  // Set to true by the default
    // method if not reimplemented
    // in base class.
    if (KF_options.use_analytic_transition_jacobian) OnTransitionJacobian(dfv_dxv);

    if (m_user_didnt_implement_jacobian || !KF_options.use_analytic_transition_jacobian ||
        KF_options.debug_verify_analytic_jacobians)
    {                                      // Numeric approximation:
      KFArray_VEH xkk_vehicle(&m_xkk[0]);  // A copy of the vehicle part of the state vector.
      KFArray_VEH xkk_veh_increments;
      OnTransitionJacobianNumericGetIncrements(xkk_veh_increments);

      mrpt::math::estimateJacobian(
          xkk_vehicle,
          std::function<void(
              const KFArray_VEH& x, const std::pair<KFCLASS*, KFArray_ACT>& dat,
              KFArray_VEH& out_x)>(&KF_aux_estimate_trans_jacobian),
          xkk_veh_increments, std::pair<KFCLASS*, KFArray_ACT>(this, u), dfv_dxv);

      if (KF_options.debug_verify_analytic_jacobians)
      {
        KFMatrix_VxV dfv_dxv_gt(mrpt::math::UNINITIALIZED_MATRIX);
        OnTransitionJacobian(dfv_dxv_gt);
        if ((dfv_dxv - dfv_dxv_gt).sum_abs() > KF_options.debug_verify_analytic_jacobians_threshold)
        {
          std::cerr << "[KalmanFilter] ERROR: User analytical "
                       "transition Jacobians are wrong: \n"
                    << " Real dfv_dxv: \n"
                    << dfv_dxv << "\n Analytical dfv_dxv:\n"
                    << dfv_dxv_gt << "Diff:\n"
                    << (dfv_dxv - dfv_dxv_gt) << "\n";
          THROW_EXCEPTION(
              "ERROR: User analytical transition Jacobians are wrong "
              "(More details dumped to cerr)");
        }
      }
    }

    // Q is the process noise covariance matrix, is associated to the robot
    // movement and is necessary to calculate the prediction P(k+1|k)
    KFMatrix_VxV Q;
    OnTransitionNoise(Q);

    // ====================================
    //  3.1:  Pxx submatrix
    // ====================================
    // Replace old covariance:
    m_pkk.asEigen().template block<VEH_SIZE, VEH_SIZE>(0, 0) =
        Q.asEigen() + dfv_dxv.asEigen() * m_pkk.template block<VEH_SIZE, VEH_SIZE>(0, 0) *
                          dfv_dxv.asEigen().transpose();

    // ====================================
    //  3.2:  All Pxy_i
    // ====================================
    // Now, update the cov. of landmarks, if any:
    KFMatrix_VxF aux;
    for (size_t i = 0; i < N_map; i++)
    {
      aux = dfv_dxv.asEigen() *
            m_pkk.template block<VEH_SIZE, FEAT_SIZE>(0, VEH_SIZE + i * FEAT_SIZE);

      m_pkk.asEigen().template block<VEH_SIZE, FEAT_SIZE>(0, VEH_SIZE + i * FEAT_SIZE) =
          aux.asEigen();
      m_pkk.asEigen().template block<FEAT_SIZE, VEH_SIZE>(VEH_SIZE + i * FEAT_SIZE, 0) =
          aux.asEigen().transpose();
    }

    // =============================================================
    //  4. NOW WE CAN OVERWRITE THE NEW STATE VECTOR
    // =============================================================
    for (size_t i = 0; i < VEH_SIZE; i++) m_xkk[i] = xv[i];

    // Normalize, if necessary.
    OnNormalizeStateVector();

  }  // end if (!skipPrediction)

  const double tim_pred = m_timLogger.leave("KF:2.prediction stage");

  // =============================================================
  //  5. PREDICTION OF OBSERVATIONS AND COMPUTE JACOBIANS
  // =============================================================
  m_timLogger.enter("KF:3.predict all obs");

  KFMatrix_OxO R;  // Sensor uncertainty (covariance matrix): R
  OnGetObservationNoise(R);

  // Predict the observations for all the map LMs, so the user
  //  can decide if their covariances (more costly) must be computed as well:
  m_all_predictions.resize(N_map);
  OnObservationModel(mrpt::math::sequenceStdVec<size_t, 1>(0, N_map), m_all_predictions);

  const double tim_pred_obs = m_timLogger.leave("KF:3.predict all obs");

  m_timLogger.enter("KF:4.decide pred obs");

  // Decide if some of the covariances shouldn't be predicted.
  m_predictLMidxs.clear();
  if (FEAT_SIZE == 0)
    // In non-SLAM problems, just do one prediction, for the entire system
    // state:
    m_predictLMidxs.assign(1, 0);
  else
    // On normal SLAM problems:
    OnPreComputingPredictions(m_all_predictions, m_predictLMidxs);

  m_timLogger.leave("KF:4.decide pred obs");

  // =============================================================
  //  6. COMPUTE INNOVATION MATRIX "m_S"
  // =============================================================
  // Do the prediction of the observation covariances:
  // Compute m_S:  m_S = H P H' + R
  //
  // Build a big dh_dx Jacobian composed of the small block Jacobians.
  // , but: it's actually a subset of the full Jacobian, since the
  // non-predicted
  //  features do not appear.
  //
  //  dh_dx: O*M x V+M*F
  //      m_S: O*M x O*M
  //  M = |m_predictLMidxs|
  //  O=size of each observation.
  //  F=size of features in the map
  //
  //  Updated: Now we only keep the non-zero blocks of that Jacobian,
  //    in the vectors m_Hxs[] and m_Hys[].
  //

  m_timLogger.enter("KF:5.build Jacobians");

  size_t N_pred = FEAT_SIZE == 0
                      ? 1 /* In non-SLAM problems, there'll be only 1 fixed observation */
                      : m_predictLMidxs.size();

  std::vector<int> data_association;  // -1: New map feature.>=0: Indexes in the
  // state vector

  // The next loop will only do more than one iteration if the heuristic in
  // OnPreComputingPredictions() fails,
  //  which will be detected by the addition of extra landmarks to predict
  //  into "missing_predictions_to_add"
  std::vector<size_t> missing_predictions_to_add;

  m_Hxs.resize(N_pred);  // Lists of partial Jacobians
  m_Hys.resize(N_pred);

  size_t first_new_pred = 0;  // This will be >0 only if we perform multiple
  // loops due to failures in the prediction
  // heuristic.

  do
  {
    if (!missing_predictions_to_add.empty())
    {
      const size_t nNew = missing_predictions_to_add.size();
      MRPT_LOG_WARN_STREAM(
          "[KF] *Performance Warning*: " << nNew
                                         << " LMs were not correctly predicted by "
                                            "OnPreComputingPredictions().");

      ASSERTDEB_(FEAT_SIZE != 0);
      for (size_t j = 0; j < nNew; j++) m_predictLMidxs.push_back(missing_predictions_to_add[j]);

      N_pred = m_predictLMidxs.size();
      missing_predictions_to_add.clear();
    }

    m_Hxs.resize(N_pred);  // Append new entries, if needed.
    m_Hys.resize(N_pred);

    for (size_t i = first_new_pred; i < N_pred; ++i)
    {
      const size_t lm_idx = FEAT_SIZE == 0 ? 0 : m_predictLMidxs[i];
      KFMatrix_OxV& Hx = m_Hxs[i];
      KFMatrix_OxF& Hy = m_Hys[i];

      // Try the analitic Jacobian first:
      m_user_didnt_implement_jacobian = false;  // Set to true by the default method if not
      // reimplemented in base class.
      if (KF_options.use_analytic_observation_jacobian) OnObservationJacobians(lm_idx, Hx, Hy);

      if (m_user_didnt_implement_jacobian || !KF_options.use_analytic_observation_jacobian ||
          KF_options.debug_verify_analytic_jacobians)
      {  // Numeric approximation:
        const size_t lm_idx_in_statevector = VEH_SIZE + lm_idx * FEAT_SIZE;

        const KFArray_VEH x_vehicle(&m_xkk[0]);
        const KFArray_FEAT x_feat(&m_xkk[lm_idx_in_statevector]);

        KFArray_VEH xkk_veh_increments;
        KFArray_FEAT feat_increments;
        OnObservationJacobiansNumericGetIncrements(xkk_veh_increments, feat_increments);

        mrpt::math::estimateJacobian(
            x_vehicle,
            std::function<void(
                const KFArray_VEH& x, const std::pair<KFCLASS*, size_t>& dat, KFArray_OBS& out_x)>(
                &KF_aux_estimate_obs_Hx_jacobian),
            xkk_veh_increments, std::pair<KFCLASS*, size_t>(this, lm_idx), Hx);
        // The state vector was temporarily modified by
        // KF_aux_estimate_*, restore it:
        std::memcpy(&m_xkk[0], &x_vehicle[0], sizeof(m_xkk[0]) * VEH_SIZE);

        mrpt::math::estimateJacobian(
            x_feat,
            std::function<void(
                const KFArray_FEAT& x, const std::pair<KFCLASS*, size_t>& dat, KFArray_OBS& out_x)>(
                &KF_aux_estimate_obs_Hy_jacobian),
            feat_increments, std::pair<KFCLASS*, size_t>(this, lm_idx), Hy);
        // The state vector was temporarily modified by
        // KF_aux_estimate_*, restore it:
        std::memcpy(&m_xkk[lm_idx_in_statevector], &x_feat[0], sizeof(m_xkk[0]) * FEAT_SIZE);

        if (KF_options.debug_verify_analytic_jacobians)
        {
          KFMatrix_OxV Hx_gt(mrpt::math::UNINITIALIZED_MATRIX);
          KFMatrix_OxF Hy_gt(mrpt::math::UNINITIALIZED_MATRIX);
          OnObservationJacobians(lm_idx, Hx_gt, Hy_gt);
          if (KFMatrix(Hx - Hx_gt).sum_abs() > KF_options.debug_verify_analytic_jacobians_threshold)
          {
            std::cerr << "[KalmanFilter] ERROR: User analytical "
                         "observation Hx Jacobians are wrong: \n"
                      << " Real Hx: \n"
                      << Hx.asEigen() << "\n Analytical Hx:\n"
                      << Hx_gt.asEigen() << "Diff:\n"
                      << (Hx.asEigen() - Hx_gt.asEigen()) << "\n";
            THROW_EXCEPTION(
                "ERROR: User analytical observation Hx Jacobians "
                "are wrong (More details dumped to cerr)");
          }
          if (KFMatrix(Hy - Hy_gt).sum_abs() > KF_options.debug_verify_analytic_jacobians_threshold)
          {
            std::cerr << "[KalmanFilter] ERROR: User analytical "
                         "observation Hy Jacobians are wrong: \n"
                      << " Real Hy: \n"
                      << Hy.asEigen() << "\n Analytical Hx:\n"
                      << Hy_gt.asEigen() << "Diff:\n"
                      << Hy.asEigen() - Hy_gt.asEigen() << "\n";
            THROW_EXCEPTION(
                "ERROR: User analytical observation Hy Jacobians "
                "are wrong (More details dumped to cerr)");
          }
        }
      }
    }
    m_timLogger.leave("KF:5.build Jacobians");

    m_timLogger.enter("KF:6.build m_S");

    // Compute m_S:  m_S = H P H' + R  (R will be added below)
    //  exploiting the sparsity of H:
    // Each block in m_S is:
    //    Sij =
    // ------------------------------------------
    m_S.setSize(N_pred * OBS_SIZE, N_pred * OBS_SIZE);

    if (FEAT_SIZE > 0)
    {  // SLAM-like problem:
      // Covariance of the vehicle pose
      const auto Px = m_pkk.template block<VEH_SIZE, VEH_SIZE>(0, 0);

      for (size_t i = 0; i < N_pred; ++i)
      {
        const size_t lm_idx_i = m_predictLMidxs[i];
        // Pxyi^t
        const auto Pxyi_t =
            m_pkk.template block<FEAT_SIZE, VEH_SIZE>(VEH_SIZE + lm_idx_i * FEAT_SIZE, 0);

        // Only do j>=i (upper triangle), since m_S is symmetric:
        for (size_t j = i; j < N_pred; ++j)
        {
          const size_t lm_idx_j = m_predictLMidxs[j];
          // Sij block:
          mrpt::math::CMatrixFixed<KFTYPE, OBS_SIZE, OBS_SIZE> Sij;

          const auto Pxyj =
              m_pkk.template block<VEH_SIZE, FEAT_SIZE>(0, VEH_SIZE + lm_idx_j * FEAT_SIZE);
          const auto Pyiyj = m_pkk.template block<FEAT_SIZE, FEAT_SIZE>(
              VEH_SIZE + lm_idx_i * FEAT_SIZE, VEH_SIZE + lm_idx_j * FEAT_SIZE);

          // clang-format off
					Sij = m_Hxs[i].asEigen() * Px     * m_Hxs[j].asEigen().transpose() +
					      m_Hys[i].asEigen() * Pxyi_t * m_Hxs[j].asEigen().transpose() +
					      m_Hxs[i].asEigen() * Pxyj   * m_Hys[j].asEigen().transpose() +
					      m_Hys[i].asEigen() * Pyiyj  * m_Hys[j].asEigen().transpose();
          // clang-format on

          m_S.insertMatrix(OBS_SIZE * i, OBS_SIZE * j, Sij);

          // Copy transposed to the symmetric lower-triangular part:
          if (i != j) m_S.insertMatrixTransposed(OBS_SIZE * j, OBS_SIZE * i, Sij);
        }

        // Sum the "R" term to the diagonal blocks:
        const size_t obs_idx_off = i * OBS_SIZE;
        m_S.asEigen().template block<OBS_SIZE, OBS_SIZE>(obs_idx_off, obs_idx_off) += R.asEigen();
      }
    }
    else
    {  // Not SLAM-like problem: simply m_S=H*Pkk*H^t + R
      ASSERTDEB_(N_pred == 1);
      ASSERTDEB_(m_S.cols() == OBS_SIZE);

      m_S = m_Hxs[0].asEigen() * m_pkk.asEigen() * m_Hxs[0].asEigen().transpose() + R.asEigen();
    }

    m_timLogger.leave("KF:6.build m_S");

    m_Z.clear();  // Each entry is one observation:

    m_timLogger.enter("KF:7.get obs & DA");

    // Get observations and do data-association:
    OnGetObservationsAndDataAssociation(
        m_Z, data_association,  // Out
        m_all_predictions, m_S, m_predictLMidxs,
        R  // In
    );
    ASSERTDEB_(
        data_association.size() == m_Z.size() || (data_association.empty() && FEAT_SIZE == 0));

    // Check if an observation hasn't been predicted in
    // OnPreComputingPredictions() but has been actually
    //  observed. This may imply an error in the heuristic of
    //  OnPreComputingPredictions(), and forces us
    //  to rebuild the matrices
    missing_predictions_to_add.clear();
    if (FEAT_SIZE != 0)
    {
      for (int i : data_association)
      {
        if (i < 0) continue;
        const auto assoc_idx_in_map = static_cast<size_t>(i);
        const size_t assoc_idx_in_pred =
            mrpt::containers::find_in_vector(assoc_idx_in_map, m_predictLMidxs);
        if (assoc_idx_in_pred == std::string::npos)
          missing_predictions_to_add.push_back(assoc_idx_in_map);
      }
    }

    first_new_pred = N_pred;  // If we do another loop, start at the begin
                              // of new predictions

  } while (!missing_predictions_to_add.empty());

  const double tim_obs_DA = m_timLogger.leave("KF:7.get obs & DA");

  // =============================================================
  //  7. UPDATE USING THE KALMAN GAIN
  // =============================================================
  // Update, only if there are observations!
  if (!m_Z.empty())
  {
    m_timLogger.enter("KF:8.update stage");

    switch (KF_options.method)
    {
      // -----------------------
      //  FULL KF- METHOD
      // -----------------------
      case kfEKFNaive:
      case kfIKFFull:
      {
        // Build the whole Jacobian dh_dx matrix
        // ---------------------------------------------
        // Keep only those whose DA is not -1
        std::vector<int> mapIndicesForKFUpdate(data_association.size());
        mapIndicesForKFUpdate.resize(std::distance(
            mapIndicesForKFUpdate.begin(),
            std::remove_copy_if(
                data_association.begin(), data_association.end(), mapIndicesForKFUpdate.begin(),
                [](int i) { return i == -1; })));

        const size_t N_upd =
            (FEAT_SIZE == 0) ? 1 :             // Non-SLAM problems: Just one
                                               // observation for the entire
                                               // system.
                mapIndicesForKFUpdate.size();  // SLAM: # of observed known landmarks

        // Just one, or several update iterations??
        const size_t nKF_iterations =
            (KF_options.method == kfEKFNaive) ? 1 : KF_options.IKF_iterations;

        const KFVector xkk_0 = m_xkk;

        // For each IKF iteration (or 1 for EKF)
        if (N_upd > 0)  // Do not update if we have no observations!
        {
          for (size_t IKF_iteration = 0; IKF_iteration < nKF_iterations; IKF_iteration++)
          {
            // Compute ytilde = OBS - PREDICTION
            KFVector ytilde(OBS_SIZE * N_upd);
            size_t ytilde_idx = 0;

            // TODO: Use a Matrix view of "dh_dx_full" instead of
            // creating a copy into "m_dh_dx_full_obs"
            m_dh_dx_full_obs.setZero(
                N_upd * OBS_SIZE,
                VEH_SIZE + FEAT_SIZE * N_map);  // Init to zeros.
            KFMatrix S_observed;                // The KF "m_S" matrix: A
            // re-ordered, subset, version of
            // the prediction m_S:

            // (map_idx, pred_idx) for each update observation — used by sparse P update
            std::vector<std::pair<size_t, size_t>> upd_obs_info;

            if (FEAT_SIZE != 0)
            {  // SLAM problems:
              std::vector<size_t> S_idxs;
              S_idxs.reserve(OBS_SIZE * N_upd);
              upd_obs_info.reserve(N_upd);

              // const size_t row_len = VEH_SIZE + FEAT_SIZE *
              // N_map;

              for (size_t i = 0; i < data_association.size(); ++i)
              {
                if (data_association[i] < 0) continue;

                const auto assoc_idx_in_map = static_cast<size_t>(data_association[i]);
                const size_t assoc_idx_in_pred =
                    mrpt::containers::find_in_vector(assoc_idx_in_map, m_predictLMidxs);
                ASSERTMSG_(
                    assoc_idx_in_pred != string::npos,
                    "OnPreComputingPredictions() didn't "
                    "recommend the prediction of a landmark "
                    "which has been actually observed!");
                // TODO: In these cases, extend the prediction
                // right now instead of launching an
                // exception... or is this a bad idea??

                // Build the subset m_dh_dx_full_obs:
                //										m_dh_dx_full_obs.block(S_idxs.size()
                //,0, OBS_SIZE, row_len)
                //										=
                //										dh_dx_full.block
                //(assoc_idx_in_pred*OBS_SIZE,0, OBS_SIZE,
                // row_len);

                m_dh_dx_full_obs.template block<OBS_SIZE, VEH_SIZE>(S_idxs.size(), 0) =
                    m_Hxs[assoc_idx_in_pred].asEigen();
                m_dh_dx_full_obs.template block<OBS_SIZE, FEAT_SIZE>(
                    S_idxs.size(), VEH_SIZE + assoc_idx_in_map * FEAT_SIZE) =
                    m_Hys[assoc_idx_in_pred].asEigen();

                upd_obs_info.emplace_back(assoc_idx_in_map, assoc_idx_in_pred);

                // S_idxs.size() is used as counter for
                // "m_dh_dx_full_obs".
                for (size_t k = 0; k < OBS_SIZE; k++)
                  S_idxs.push_back(assoc_idx_in_pred * OBS_SIZE + k);

                // ytilde_i = Z[i] - m_all_predictions[i]
                KFArray_OBS ytilde_i = m_Z[i];
                OnSubtractObservationVectors(
                    ytilde_i, m_all_predictions[m_predictLMidxs[assoc_idx_in_pred]]);
                for (size_t k = 0; k < OBS_SIZE; k++) ytilde[ytilde_idx++] = ytilde_i[k];
              }
              // Extract the subset that is involved in this
              // observation:
              mrpt::math::extractSubmatrixSymmetrical(m_S, S_idxs, S_observed);
            }
            else
            {  // Non-SLAM problems:
              ASSERT_(m_Z.size() == 1 && m_all_predictions.size() == 1);
              ASSERT_(m_dh_dx_full_obs.rows() == OBS_SIZE && m_dh_dx_full_obs.cols() == VEH_SIZE);
              ASSERT_(m_Hxs.size() == 1);
              m_dh_dx_full_obs = m_Hxs[0];  // Was: dh_dx_full
              KFArray_OBS ytilde_i = m_Z[0];
              OnSubtractObservationVectors(ytilde_i, m_all_predictions[0]);
              for (size_t k = 0; k < OBS_SIZE; k++) ytilde[ytilde_idx++] = ytilde_i[k];
              // Extract the subset that is involved in this
              // observation:
              S_observed = m_S;
            }

            // Compute the full K matrix:
            // ------------------------------
            m_timLogger.enter("KF:8.update stage:1.FULLKF:build K");

            m_K.setSize(m_pkk.rows(), S_observed.cols());

            // K = m_pkk * (~dh_dx) * m_S.inverse_LLt() );
            m_K.asEigen() = m_pkk.asEigen() * m_dh_dx_full_obs.asEigen().transpose();

            // Inverse of S_observed -> m_S_1
            m_S_1 = S_observed.inverse_LLt();
            m_K.asEigen() *= m_S_1.asEigen();

            m_timLogger.leave("KF:8.update stage:1.FULLKF:build K");

            // Use the full K matrix to update the mean:
            if (nKF_iterations == 1)
            {
              m_timLogger.enter("KF:8.update stage:2.FULLKF:update xkk");
              m_xkk.asEigen() += (m_K * ytilde).eval();
              m_timLogger.leave("KF:8.update stage:2.FULLKF:update xkk");
            }
            else
            {
              m_timLogger.enter("KF:8.update stage:2.FULLKF:iter.update xkk");

              auto HAx_column = KFVector(m_dh_dx_full_obs * (m_xkk - xkk_0));

              m_xkk = xkk_0;
              m_xkk.asEigen() += m_K * (ytilde - HAx_column);

              m_timLogger.leave("KF:8.update stage:2.FULLKF:iter.update xkk");
            }

            // Update the covariance just at the end
            //  of iterations if we are in IKF, always in normal
            //  EKF.
            if (IKF_iteration == (nKF_iterations - 1))
            {
              m_timLogger.enter("KF:8.update stage:3.FULLKF:update Pkk");

              // Helper lambda: dense covariance update used for both the
              // non-SLAM path and the optional debug cross-check.
              // Returns updated P given the current P and K.
              auto dense_P_update = [&](const KFMatrix& pkk_in) -> KFMatrix
              {
                // Build (I - K*H) using the dense m_dh_dx_full_obs
                m_aux_K_dh_dx.matProductOf_AB(m_K, m_dh_dx_full_obs);
                const size_t stat_len = m_aux_K_dh_dx.cols();
                for (size_t r = 0; r < stat_len; r++)
                {
                  for (size_t c = 0; c < stat_len; c++)
                  {
                    m_aux_K_dh_dx(r, c) =
                        (r == c) ? kftype(1) - m_aux_K_dh_dx(r, c) : -m_aux_K_dh_dx(r, c);
                  }
                }
                KFMatrix out(pkk_in.rows(), pkk_in.cols());
                if (KF_options.use_joseph_form)
                {
                  const auto IKH = m_aux_K_dh_dx.asEigen();
                  const auto P = pkk_in.asEigen();
                  out.asEigen() = (IKH * P * IKH.transpose()).eval();
                  // K*S_observed*K^T (equivalent to K*R_block*K^T but cheaper)
                  out.asEigen() +=
                      (m_K.asEigen() * S_observed.asEigen() * m_K.asEigen().transpose()).eval();
                }
                else
                {
                  out.asEigen() = (m_aux_K_dh_dx.asEigen() * pkk_in.asEigen()).eval();
                }
                const auto Peval = out.asEigen().eval();
                out.asEigen() = 0.5 * (Peval + Peval.transpose());
                return out;
              };

              if constexpr (FEAT_SIZE != 0)
              {
                // -------------------------------------------------------
                // SLAM: sparse P update exploiting the block structure of H.
                // dP = K * H * P, computed as a sum over observed landmarks:
                //   dP += K_i * Hx_i * P[0:VEH, :]
                //       + K_i * Hy_i * P[idx_off:idx_off+FEAT, :]
                // Complexity: O(N_upd*(VEH+FEAT)*n) vs O(n^3) for dense.
                // -------------------------------------------------------

                // Optionally save P before modification for the debug cross-check.
                KFMatrix pkk_before_sparse;
                if (KF_options.debug_sparse_vs_dense_P_update)
                {
                  pkk_before_sparse = m_pkk;
                }

                const size_t n = m_pkk.cols();
                KFMatrix dP(n, n);
                dP.setZero();

                for (size_t i = 0; i < N_upd; i++)
                {
                  const size_t map_idx = upd_obs_info[i].first;
                  const size_t pred_idx = upd_obs_info[i].second;
                  const size_t idx_off = VEH_SIZE + map_idx * FEAT_SIZE;
                  const auto K_i =
                      m_K.asEigen().middleCols(i * OBS_SIZE, OBS_SIZE);  // n x OBS_SIZE
                  dP.asEigen() += K_i * m_Hxs[pred_idx].asEigen() *
                                  m_pkk.asEigen().template topRows<VEH_SIZE>();
                  dP.asEigen() += K_i * m_Hys[pred_idx].asEigen() *
                                  m_pkk.asEigen().middleRows(idx_off, FEAT_SIZE);
                }

                if (KF_options.use_joseph_form)
                {
                  // Joseph form via sparse dP (algebraically identical to
                  // (I-KH)*P*(I-KH)^T + K*R*K^T):
                  //   P' = P - dP - dP^T + K*S_observed*K^T
                  m_pkk.asEigen() -= (dP.asEigen() + dP.asEigen().transpose()).eval();
                  m_pkk.asEigen() +=
                      (m_K.asEigen() * S_observed.asEigen() * m_K.asEigen().transpose()).eval();
                }
                else
                {
                  m_pkk.asEigen() -= dP.asEigen();
                }
                // Symmetrize:
                {
                  const auto Peval = m_pkk.asEigen().eval();
                  m_pkk.asEigen() = 0.5 * (Peval + Peval.transpose());
                }

                if (KF_options.debug_sparse_vs_dense_P_update)
                {
                  const KFMatrix pkk_sparse_result = m_pkk;
                  // Compute dense result from the same pre-update P:
                  m_pkk = pkk_before_sparse;
                  const KFMatrix pkk_dense_result = dense_P_update(m_pkk);

                  const KFTYPE max_diff =
                      static_cast<KFTYPE>((pkk_sparse_result.asEigen() - pkk_dense_result.asEigen())
                                              .cwiseAbs()
                                              .maxCoeff());
                  if (max_diff > KFTYPE(1e-7))
                  {
                    MRPT_LOG_ERROR_FMT(
                        "[KF] debug_sparse_vs_dense_P_update MISMATCH: "
                        "max |sparse-dense| = %.3e (threshold 1e-7)",
                        static_cast<double>(max_diff));
                    THROW_EXCEPTION(
                        "Sparse and dense covariance updates disagree. "
                        "See stderr for details.");
                  }
                  // Use the sparse result (verified to match dense):
                  m_pkk = pkk_sparse_result;
                }
              }
              else
              {
                // Non-SLAM: H has no sparsity (VEH_SIZE x VEH_SIZE system),
                // use the existing dense code path.
                m_pkk = dense_P_update(m_pkk);
              }

              m_timLogger.leave("KF:8.update stage:3.FULLKF:update Pkk");
            }
          }  // end for each IKF iteration
        }
      }
      break;

      // --------------------------------------------------------------------
      // - EKF 'a la' Davison: One observation element at once
      // --------------------------------------------------------------------
      case kfEKFAlaDavison:
      {
        // For each observed landmark/whole system state:
        for (size_t obsIdx = 0; obsIdx < m_Z.size(); obsIdx++)
        {
          // Known & mapped landmark?
          bool doit;
          size_t idxInTheFilter = 0;

          if (data_association.empty())
          {
            doit = true;
          }
          else
          {
            doit = data_association[obsIdx] >= 0;
            if (doit) idxInTheFilter = data_association[obsIdx];
          }

          if (doit)
          {
            m_timLogger.enter("KF:8.update stage:1.ScalarAtOnce.prepare");

            // Already mapped: OK
            const size_t idx_off =
                VEH_SIZE + idxInTheFilter * FEAT_SIZE;  // The offset in m_xkk & Pkk.

            // Compute just the part of the Jacobian that we need
            // using the current updated m_xkk:
            vector_KFArray_OBS pred_obs;
            OnObservationModel(std::vector<size_t>(1, idxInTheFilter), pred_obs);
            ASSERTDEB_(pred_obs.size() == 1);

            // ytilde = observation - prediction
            KFArray_OBS ytilde = m_Z[obsIdx];
            OnSubtractObservationVectors(ytilde, pred_obs[0]);

            // Jacobians:
            // dh_dx: already is (N_pred*OBS_SIZE) x (VEH_SIZE +
            // FEAT_SIZE * N_pred )
            //         with N_pred = |m_predictLMidxs|

            const size_t i_idx_in_preds =
                mrpt::containers::find_in_vector(idxInTheFilter, m_predictLMidxs);
            ASSERTMSG_(
                i_idx_in_preds != string::npos,
                "OnPreComputingPredictions() didn't recommend the "
                "prediction of a landmark which has been actually "
                "observed!");

            const KFMatrix_OxV& Hx = m_Hxs[i_idx_in_preds];
            const KFMatrix_OxF& Hy = m_Hys[i_idx_in_preds];

            m_timLogger.leave("KF:8.update stage:1.ScalarAtOnce.prepare");

            // For each component of the observation:
            for (size_t j = 0; j < OBS_SIZE; j++)
            {
              m_timLogger.enter("KF:8.update stage:2.ScalarAtOnce.update");

              // Compute the scalar S_i for each component j of
              // the observation: Sij = dhij_dxv Pxx dhij_dxv^t +
              // 2 * dhij_dyi Pyix dhij_dxv + dhij_dyi Pyiyi
              // dhij_dyi^t + R
              //          ^^
              //         Hx:(O*LxSv)
              //       \----------------------/
              //       \--------------------------/
              //       \------------------------/ \-/
              //               TERM 1                   TERM 2
              //               TERM 3 R
              //
              // O: Observation size (3)
              // L: # landmarks
              // Sv: Vehicle state size (6)
              //

              // Require diagonal R for this scalar update algorithm:
              {
                KFMatrix_OxO Rdiag;
                Rdiag.setZero();
                for (size_t _d = 0; _d < OBS_SIZE; _d++) Rdiag(_d, _d) = R(_d, _d);
                ASSERTMSG_(
                    (R.asEigen() - Rdiag.asEigen()).cwiseAbs().sum() < KFTYPE(1e-12),
                    "kfEKFAlaDavison requires a diagonal observation noise matrix R. "
                    "Select another KF algorithm (e.g. kfEKFNaive) for correlated noise.");
              }

              // Innovation variance Sij for component j of the observation.
              // Sij = R(j,j) + Hx_j*Pxx*Hx_j^T + 2*Hx_j*Pxyi^T*Hy_j^T + Hy_j*Pyiyi*Hy_j^T
              const auto Hx_j = Hx.asEigen().row(j);
              const auto Hy_j = Hy.asEigen().row(j);
              const auto Pxx = m_pkk.asEigen().template block<VEH_SIZE, VEH_SIZE>(0, 0);
              const auto Pxyi = m_pkk.asEigen().template block<VEH_SIZE, FEAT_SIZE>(0, idx_off);
              const auto Pyiyi =
                  m_pkk.asEigen().template block<FEAT_SIZE, FEAT_SIZE>(idx_off, idx_off);

              KFTYPE Sij = R(j, j) + static_cast<KFTYPE>((Hx_j * Pxx * Hx_j.transpose()).value()) +
                           static_cast<KFTYPE>(2.0 * (Hx_j * Pxyi * Hy_j.transpose()).value()) +
                           static_cast<KFTYPE>((Hy_j * Pyiyi * Hy_j.transpose()).value());

              // Kalman gain vector: Kij = P * h_j^T / Sij
              // where h_j is the j-th row of the full Jacobian.
              const size_t N = m_pkk.cols();
              Eigen::Matrix<KFTYPE, Eigen::Dynamic, 1> h_j =
                  Eigen::Matrix<KFTYPE, Eigen::Dynamic, 1>::Zero(N);
              for (size_t q = 0; q < VEH_SIZE; q++) h_j[q] = Hx(j, q);
              for (size_t q = 0; q < FEAT_SIZE; q++) h_j[idx_off + q] = Hy(j, q);

              Eigen::Matrix<KFTYPE, Eigen::Dynamic, 1> Kij_vec = (m_pkk.asEigen() * h_j) / Sij;

              // Update state: x' = x + Kij * ytilde(j)
              m_xkk.asEigen() += Kij_vec * ytilde[j];

              // Update covariance: P' = P - Sij * Kij * Kij^T (rank-1 update)
              m_pkk.asEigen() -= Sij * (Kij_vec * Kij_vec.transpose());
              // Symmetrize to suppress floating-point drift:
              {
                const auto P_eval = m_pkk.asEigen().eval();
                m_pkk.asEigen() = 0.5 * (P_eval + P_eval.transpose());
              }

              m_timLogger.leave("KF:8.update stage:2.ScalarAtOnce.update");
            }  // end for j
          }    // end if mapped
        }      // end for each observed LM.
      }
      break;

      // --------------------------------------------------------------------
      // - IKF method, processing each observation scalar secuentially:
      // --------------------------------------------------------------------
      default:
        THROW_EXCEPTION("Invalid value of options.KF_method");
    }  // end switch method
  }

  const double tim_update = m_timLogger.leave("KF:8.update stage");

  m_timLogger.enter("KF:9.OnNormalizeStateVector");
  OnNormalizeStateVector();
  m_timLogger.leave("KF:9.OnNormalizeStateVector");

  // =============================================================
  //  8. INTRODUCE NEW LANDMARKS
  // =============================================================
  if (!data_association.empty())
  {
    m_timLogger.enter("KF:A.add new landmarks");
    detail::addNewLandmarks(*this, m_Z, data_association, R);
    m_timLogger.leave("KF:A.add new landmarks");
  }  // end if data_association!=empty

  // Post iteration user code:
  m_timLogger.enter("KF:B.OnPostIteration");
  OnPostIteration();
  m_timLogger.leave("KF:B.OnPostIteration");

  m_timLogger.leave("KF:complete_step");

  MRPT_LOG_DEBUG(mrpt::format(
      "[KF] %u LMs | Pr: %.2fms | Pr.Obs: %.2fms | Obs.DA: %.2fms | Upd: "
      "%.2fms\n",
      static_cast<unsigned int>(getNumberOfLandmarksInTheMap()), 1e3 * tim_pred, 1e3 * tim_pred_obs,
      1e3 * tim_obs_DA, 1e3 * tim_update));
  MRPT_END
}  // End of "runOneKalmanIteration"

template <size_t VEH_SIZE, size_t OBS_SIZE, size_t FEAT_SIZE, size_t ACT_SIZE, typename KFTYPE>
void CKalmanFilterCapable<VEH_SIZE, OBS_SIZE, FEAT_SIZE, ACT_SIZE, KFTYPE>::
    KF_aux_estimate_trans_jacobian(
        const KFArray_VEH& x, const std::pair<KFCLASS*, KFArray_ACT>& dat, KFArray_VEH& out_x)
{
  bool dumm;
  out_x = x;
  dat.first->OnTransitionModel(dat.second, out_x, dumm);
}

template <size_t VEH_SIZE, size_t OBS_SIZE, size_t FEAT_SIZE, size_t ACT_SIZE, typename KFTYPE>
void CKalmanFilterCapable<VEH_SIZE, OBS_SIZE, FEAT_SIZE, ACT_SIZE, KFTYPE>::
    KF_aux_estimate_obs_Hx_jacobian(
        const KFArray_VEH& x, const std::pair<KFCLASS*, size_t>& dat, KFArray_OBS& out_x)
{
  std::vector<size_t> idxs_to_predict(1, dat.second);
  vector_KFArray_OBS prediction;
  // Overwrite (temporarily!) the affected part of the state vector:
  std::memcpy(&dat.first->m_xkk[0], &x[0], sizeof(x[0]) * VEH_SIZE);
  dat.first->OnObservationModel(idxs_to_predict, prediction);
  ASSERTDEB_(prediction.size() == 1);
  out_x = prediction[0];
}

template <size_t VEH_SIZE, size_t OBS_SIZE, size_t FEAT_SIZE, size_t ACT_SIZE, typename KFTYPE>
void CKalmanFilterCapable<VEH_SIZE, OBS_SIZE, FEAT_SIZE, ACT_SIZE, KFTYPE>::
    KF_aux_estimate_obs_Hy_jacobian(
        const KFArray_FEAT& x, const std::pair<KFCLASS*, size_t>& dat, KFArray_OBS& out_x)
{
  std::vector<size_t> idxs_to_predict(1, dat.second);
  vector_KFArray_OBS prediction;
  const size_t lm_idx_in_statevector = VEH_SIZE + FEAT_SIZE * dat.second;
  // Overwrite (temporarily!) the affected part of the state vector:
  std::memcpy(&dat.first->m_xkk[lm_idx_in_statevector], &x[0], sizeof(x[0]) * FEAT_SIZE);
  dat.first->OnObservationModel(idxs_to_predict, prediction);
  ASSERTDEB_(prediction.size() == 1);
  out_x = prediction[0];
}

namespace detail
{
// generic version for SLAM. There is a speciation below for NON-SLAM problems.
template <size_t VEH_SIZE, size_t OBS_SIZE, size_t FEAT_SIZE, size_t ACT_SIZE, typename KFTYPE>
void addNewLandmarks(
    CKalmanFilterCapable<VEH_SIZE, OBS_SIZE, FEAT_SIZE, ACT_SIZE, KFTYPE>& obj,
    const typename CKalmanFilterCapable<VEH_SIZE, OBS_SIZE, FEAT_SIZE, ACT_SIZE, KFTYPE>::
        vector_KFArray_OBS& Z,
    const std::vector<int>& data_association,
    const typename CKalmanFilterCapable<VEH_SIZE, OBS_SIZE, FEAT_SIZE, ACT_SIZE, KFTYPE>::
        KFMatrix_OxO& R)
{
  using KF = CKalmanFilterCapable<VEH_SIZE, OBS_SIZE, FEAT_SIZE, ACT_SIZE, KFTYPE>;

  for (size_t idxObs = 0; idxObs < Z.size(); idxObs++)
  {
    // Is already in the map?
    if (data_association[idxObs] < 0)  // Not in the map yet!
    {
      obj.getProfiler().enter("KF:9.create new LMs");
      // Add it:

      // Append to map of IDs <-> position in the state vector:
      ASSERTDEB_(FEAT_SIZE > 0);
      ASSERTDEB_(0 == ((obj.internal_getXkk().size() - VEH_SIZE) % FEAT_SIZE));  // Sanity test

      const size_t newIndexInMap = (obj.internal_getXkk().size() - VEH_SIZE) / FEAT_SIZE;

      // Inverse sensor model:
      typename KF::KFArray_FEAT yn;
      typename KF::KFMatrix_FxV dyn_dxv;
      typename KF::KFMatrix_FxO dyn_dhn;
      typename KF::KFMatrix_FxF dyn_dhn_R_dyn_dhnT;
      bool use_dyn_dhn_jacobian = true;

      // Compute the inv. sensor model and its Jacobians:
      obj.OnInverseObservationModel(
          Z[idxObs], yn, dyn_dxv, dyn_dhn, dyn_dhn_R_dyn_dhnT, use_dyn_dhn_jacobian);

      // And let the application do any special handling of adding a new
      // feature to the map:
      obj.OnNewLandmarkAddedToMap(idxObs, newIndexInMap);

      ASSERTDEB_(yn.size() == FEAT_SIZE);

      // Append to m_xkk:
      size_t q;
      size_t idx = obj.internal_getXkk().size();
      obj.internal_getXkk().resize(obj.internal_getXkk().size() + FEAT_SIZE);

      for (q = 0; q < FEAT_SIZE; q++) obj.internal_getXkk()[idx + q] = yn[q];

      // --------------------
      // Append to Pkk:
      // --------------------
      ASSERTDEB_(
          obj.internal_getPkk().cols() == (int)idx && obj.internal_getPkk().rows() == (int)idx);

      obj.internal_getPkk().setSize(idx + FEAT_SIZE, idx + FEAT_SIZE);

      // Fill the Pxyn term:
      // --------------------
      auto Pxx =
          typename KF::KFMatrix_VxV(obj.internal_getPkk().template block<VEH_SIZE, VEH_SIZE>(0, 0));
      auto Pxyn = typename KF::KFMatrix_FxV(dyn_dxv * Pxx);

      obj.internal_getPkk().insertMatrix(idx, 0, Pxyn);
      obj.internal_getPkk().insertMatrix(0, idx, Pxyn.transpose());

      // Fill the Pyiyn terms:
      // --------------------
      const size_t nLMs = (idx - VEH_SIZE) / FEAT_SIZE;  // Number of previous landmarks:
      for (q = 0; q < nLMs; q++)
      {
        auto P_x_yq = typename KF::KFMatrix_VxF(
            obj.internal_getPkk().template block<VEH_SIZE, FEAT_SIZE>(0, VEH_SIZE + q * FEAT_SIZE));

        auto P_cross = typename KF::KFMatrix_FxF(dyn_dxv * P_x_yq);

        obj.internal_getPkk().insertMatrix(idx, VEH_SIZE + q * FEAT_SIZE, P_cross);
        obj.internal_getPkk().insertMatrix(VEH_SIZE + q * FEAT_SIZE, idx, P_cross.transpose());
      }  // end each previous LM(q)

      // Fill the Pynyn term:
      //  P_yn_yn =  (dyn_dxv * Pxx * ~dyn_dxv) + (dyn_dhn * R *
      //  ~dyn_dhn);
      // --------------------
      typename KF::KFMatrix_FxF P_yn_yn = mrpt::math::multiply_HCHt(dyn_dxv, Pxx);
      if (use_dyn_dhn_jacobian)
        // Accumulate in P_yn_yn
        P_yn_yn += mrpt::math::multiply_HCHt(dyn_dhn, R);
      else
        P_yn_yn += dyn_dhn_R_dyn_dhnT;

      obj.internal_getPkk().insertMatrix(idx, idx, P_yn_yn);

      obj.getProfiler().leave("KF:9.create new LMs");
    }
  }
}

template <size_t VEH_SIZE, size_t OBS_SIZE, size_t ACT_SIZE, typename KFTYPE>
void addNewLandmarks(
    CKalmanFilterCapable<VEH_SIZE, OBS_SIZE, 0 /* FEAT_SIZE=0 */, ACT_SIZE, KFTYPE>& obj,
    const typename CKalmanFilterCapable<VEH_SIZE, OBS_SIZE, 0 /* FEAT_SIZE=0 */, ACT_SIZE, KFTYPE>::
        vector_KFArray_OBS& Z,
    const std::vector<int>& data_association,
    const typename CKalmanFilterCapable<VEH_SIZE, OBS_SIZE, 0 /* FEAT_SIZE=0 */, ACT_SIZE, KFTYPE>::
        KFMatrix_OxO& R)
{
  // Do nothing: this is NOT a SLAM problem.
}

template <size_t VEH_SIZE, size_t OBS_SIZE, size_t FEAT_SIZE, size_t ACT_SIZE, typename KFTYPE>
size_t getNumberOfLandmarksInMap(
    const CKalmanFilterCapable<VEH_SIZE, OBS_SIZE, FEAT_SIZE, ACT_SIZE, KFTYPE>& obj)
{
  return (obj.getStateVectorLength() - VEH_SIZE) / FEAT_SIZE;
}
// Specialization for FEAT_SIZE=0
template <size_t VEH_SIZE, size_t OBS_SIZE, size_t ACT_SIZE, typename KFTYPE>
size_t getNumberOfLandmarksInMap(
    const CKalmanFilterCapable<VEH_SIZE, OBS_SIZE, 0 /*FEAT_SIZE*/, ACT_SIZE, KFTYPE>& obj)
{
  return 0;
}

template <size_t VEH_SIZE, size_t OBS_SIZE, size_t FEAT_SIZE, size_t ACT_SIZE, typename KFTYPE>
bool isMapEmpty(const CKalmanFilterCapable<VEH_SIZE, OBS_SIZE, FEAT_SIZE, ACT_SIZE, KFTYPE>& obj)
{
  return (obj.getStateVectorLength() == VEH_SIZE);
}
// Specialization for FEAT_SIZE=0
template <size_t VEH_SIZE, size_t OBS_SIZE, size_t ACT_SIZE, typename KFTYPE>
bool isMapEmpty(
    const CKalmanFilterCapable<VEH_SIZE, OBS_SIZE, 0 /*FEAT_SIZE*/, ACT_SIZE, KFTYPE>& obj)
{
  return true;
}
}  // namespace detail
}  // namespace mrpt::bayes
