/* +---------------------------------------------------------------------------+
|                     Mobile Robot Programming Toolkit (MRPT)               |
|                          http://www.mrpt.org/                             |
|                                                                           |
| Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
| See: http://www.mrpt.org/Authors - All rights reserved.                   |
| Released under BSD License. See details in http://www.mrpt.org/License    |
+---------------------------------------------------------------------------+ */
#ifndef CKalmanFilterCapable_IMPL_H
#define CKalmanFilterCapable_IMPL_H

#ifndef CKalmanFilterCapable_H
#error Include this file only from 'CKalmanFilterCapable.h'
#endif

namespace mrpt
{
	namespace bayes
	{
		// The main entry point in the Kalman Filter class:
		template <size_t VEH_SIZE, size_t OBS_SIZE, size_t FEAT_SIZE, size_t ACT_SIZE, typename KFTYPE>
		void CKalmanFilterCapable<VEH_SIZE,OBS_SIZE,FEAT_SIZE,ACT_SIZE,KFTYPE>::runOneKalmanIteration()
		{
			using namespace std;
			MRPT_START

			m_timLogger.enable(KF_options.enable_profiler);
			m_timLogger.enter("KF:complete_step");

			ASSERT_(size_t(m_xkk.size())==m_pkk.getColCount())
				ASSERT_(size_t(m_xkk.size())>=VEH_SIZE)

				// =============================================================
				//  1. CREATE ACTION MATRIX u FROM ODOMETRY
				// =============================================================
				KFArray_ACT  u;

			m_timLogger.enter("KF:1.OnGetAction");
			OnGetAction(u);
			m_timLogger.leave("KF:1.OnGetAction");

			// Sanity check:
			if (FEAT_SIZE) { ASSERTDEB_( (((m_xkk.size()-VEH_SIZE)/FEAT_SIZE)*FEAT_SIZE)== (m_xkk.size()-VEH_SIZE) ) }

			// =============================================================
			//  2. PREDICTION OF NEW POSE xv_{k+1|k}
			// =============================================================
			m_timLogger.enter("KF:2.prediction stage");

			const size_t N_map = getNumberOfLandmarksInTheMap();

			KFArray_VEH xv( &m_xkk[0] );  // Vehicle pose

			bool skipPrediction=false; // Whether to skip the prediction step (in SLAM this is desired for the first iteration...)

			// Update mean: xv will have the updated pose until we update it in the filterState later.
			//  This is to maintain a copy of the last robot pose in the state vector, required for the Jacobian computation.
			OnTransitionModel(u, xv, skipPrediction);

			if ( !skipPrediction )
			{
				// =============================================================
				//  3. PREDICTION OF COVARIANCE P_{k+1|k}
				// =============================================================
				// First, we compute de Jacobian fv_by_xv  (derivative of f_vehicle wrt x_vehicle):
				KFMatrix_VxV  dfv_dxv;

				// Try closed-form Jacobian first:
				m_user_didnt_implement_jacobian=false; // Set to true by the default method if not reimplemented in base class.
				if (KF_options.use_analytic_transition_jacobian)
					OnTransitionJacobian(dfv_dxv);

				if (m_user_didnt_implement_jacobian || !KF_options.use_analytic_transition_jacobian || KF_options.debug_verify_analytic_jacobians)
				{	// Numeric approximation:
					KFArray_VEH xkk_vehicle( &m_xkk[0] );  // A copy of the vehicle part of the state vector.
					KFArray_VEH xkk_veh_increments;
					OnTransitionJacobianNumericGetIncrements(xkk_veh_increments);

					mrpt::math::estimateJacobian(
						xkk_vehicle,
						&KF_aux_estimate_trans_jacobian, //(const VECTORLIKE &x,const USERPARAM &y, VECTORLIKE3  &out),
						xkk_veh_increments,
						std::pair<KFCLASS*,KFArray_ACT>(this,u),
						dfv_dxv);

					if (KF_options.debug_verify_analytic_jacobians)
					{
						KFMatrix_VxV dfv_dxv_gt(mrpt::math::UNINITIALIZED_MATRIX);
						OnTransitionJacobian(dfv_dxv_gt);
						if ((dfv_dxv-dfv_dxv_gt).array().abs().sum()>KF_options.debug_verify_analytic_jacobians_threshold)
						{
							std::cerr << "[KalmanFilter] ERROR: User analytical transition Jacobians are wrong: \n"
								<< " Real dfv_dxv: \n" << dfv_dxv << "\n Analytical dfv_dxv:\n" << dfv_dxv_gt << "Diff:\n" << (dfv_dxv-dfv_dxv_gt) << "\n";
							THROW_EXCEPTION("ERROR: User analytical transition Jacobians are wrong (More details dumped to cerr)")
						}
					}

				}

				// Q is the process noise covariance matrix, is associated to the robot movement and is necesary to calculate the prediction P(k+1|k)
				KFMatrix_VxV  Q;
				OnTransitionNoise(Q);

				// ====================================
				//  3.1:  Pxx submatrix
				// ====================================
				// Replace old covariance:
				Eigen::Block<typename KFMatrix::Base,VEH_SIZE,VEH_SIZE>(m_pkk,0,0) =
					Q +
					dfv_dxv * Eigen::Block<typename KFMatrix::Base,VEH_SIZE,VEH_SIZE>(m_pkk,0,0) * dfv_dxv.transpose();

				// ====================================
				//  3.2:  All Pxy_i
				// ====================================
				// Now, update the cov. of landmarks, if any:
				KFMatrix_VxF aux;
				for (size_t i=0 ; i<N_map ; i++)
				{
					aux = dfv_dxv * Eigen::Block<typename KFMatrix::Base,VEH_SIZE,FEAT_SIZE>(m_pkk,0,VEH_SIZE+i*FEAT_SIZE);

					Eigen::Block<typename KFMatrix::Base,VEH_SIZE,FEAT_SIZE>(m_pkk, 0                    , VEH_SIZE+i*FEAT_SIZE) = aux;
					Eigen::Block<typename KFMatrix::Base,FEAT_SIZE,VEH_SIZE>(m_pkk, VEH_SIZE+i*FEAT_SIZE , 0                   ) = aux.transpose();
				}

				// =============================================================
				//  4. NOW WE CAN OVERWRITE THE NEW STATE VECTOR
				// =============================================================
				for (size_t i=0;i<VEH_SIZE;i++)
					m_xkk[i]=xv[i];

				// Normalize, if neccesary.
				OnNormalizeStateVector();

			} // end if (!skipPrediction)

			const double tim_pred = m_timLogger.leave("KF:2.prediction stage");

			// =============================================================
			//  5. PREDICTION OF OBSERVATIONS AND COMPUTE JACOBIANS
			// =============================================================
			m_timLogger.enter("KF:3.predict all obs");

			KFMatrix_OxO  R;	// Sensor uncertainty (covariance matrix): R
			OnGetObservationNoise(R);

			// Predict the observations for all the map LMs, so the user
			//  can decide if their covariances (more costly) must be computed as well:
			all_predictions.resize(N_map);
			OnObservationModel(
				mrpt::math::sequenceStdVec<size_t,1>(0,N_map),
				all_predictions);

			const double tim_pred_obs = m_timLogger.leave("KF:3.predict all obs");

			m_timLogger.enter("KF:4.decide pred obs");

			// Decide if some of the covariances shouldn't be predicted.
			predictLMidxs.clear();
			if (FEAT_SIZE==0)
				// In non-SLAM problems, just do one prediction, for the entire system state:
					predictLMidxs.assign(1,0);
			else
				// On normal SLAM problems:
				OnPreComputingPredictions(all_predictions, predictLMidxs);


			m_timLogger.leave("KF:4.decide pred obs");

			// =============================================================
			//  6. COMPUTE INNOVATION MATRIX "S"
			// =============================================================
			// Do the prediction of the observation covariances:
			// Compute S:  S = H P ~H + R
			//
			// Build a big dh_dx Jacobian composed of the small block Jacobians.
			// , but: it's actually a subset of the full Jacobian, since the non-predicted
			//  features do not appear.
			//
			//  dh_dx: O*M x V+M*F
			//      S: O*M x O*M
			//  M = |predictLMidxs|
			//  O=size of each observation.
			//  F=size of features in the map
			//
			//  Updated: Now we only keep the non-zero blocks of that Jacobian,
			//    in the vectors Hxs[] and Hys[].
			//

			m_timLogger.enter("KF:5.build Jacobians");

			size_t N_pred = FEAT_SIZE==0 ?
				1 /* In non-SLAM problems, there'll be only 1 fixed observation */ :
			predictLMidxs.size();

			vector_int  data_association;  // -1: New map feature.>=0: Indexes in the state vector

			// The next loop will only do more than one iteration if the heuristic in OnPreComputingPredictions() fails,
			//  which will be detected by the addition of extra landmarks to predict into "missing_predictions_to_add"
			std::vector<size_t> missing_predictions_to_add;

			Hxs.resize(N_pred);  // Lists of partial Jacobians
			Hys.resize(N_pred);

			size_t first_new_pred = 0; // This will be >0 only if we perform multiple loops due to failures in the prediction heuristic.

			do
			{
				if (!missing_predictions_to_add.empty())
				{
					const size_t nNew = missing_predictions_to_add.size();
					MRPT_LOG_WARN_STREAM << "[KF] *Performance Warning*: " <<nNew <<  " LMs were not correctly predicted by OnPreComputingPredictions().";

					ASSERTDEB_(FEAT_SIZE!=0)
						for (size_t j=0;j<nNew;j++)
							predictLMidxs.push_back( missing_predictions_to_add[j] );

					N_pred = predictLMidxs.size();
					missing_predictions_to_add.clear();
				}

				Hxs.resize(N_pred);  // Append new entries, if needed.
				Hys.resize(N_pred);

				for (size_t i=first_new_pred;i<N_pred;++i)
				{
					const size_t lm_idx = FEAT_SIZE==0 ? 0 : predictLMidxs[i];
					KFMatrix_OxV &Hx = Hxs[i];
					KFMatrix_OxF &Hy = Hys[i];

					// Try the analitic Jacobian first:
					m_user_didnt_implement_jacobian=false; // Set to true by the default method if not reimplemented in base class.
					if (KF_options.use_analytic_observation_jacobian)
						OnObservationJacobians(lm_idx,Hx,Hy);

					if (m_user_didnt_implement_jacobian || !KF_options.use_analytic_observation_jacobian || KF_options.debug_verify_analytic_jacobians)
					{	// Numeric approximation:
						const size_t lm_idx_in_statevector = VEH_SIZE+lm_idx*FEAT_SIZE;

						const KFArray_VEH  x_vehicle( &m_xkk[0] );
						const KFArray_FEAT x_feat( &m_xkk[lm_idx_in_statevector] );

						KFArray_VEH  xkk_veh_increments;
						KFArray_FEAT feat_increments;
						OnObservationJacobiansNumericGetIncrements(xkk_veh_increments, feat_increments);

						mrpt::math::estimateJacobian(
							x_vehicle,
							&KF_aux_estimate_obs_Hx_jacobian,
							xkk_veh_increments,
							std::pair<KFCLASS*,size_t>(this,lm_idx),
							Hx);
						// The state vector was temporarily modified by KF_aux_estimate_*, restore it:
						::memcpy(&m_xkk[0],&x_vehicle[0],sizeof(m_xkk[0])*VEH_SIZE);

						mrpt::math::estimateJacobian(
							x_feat,
							&KF_aux_estimate_obs_Hy_jacobian,
							feat_increments,
							std::pair<KFCLASS*,size_t>(this,lm_idx),
							Hy);
						// The state vector was temporarily modified by KF_aux_estimate_*, restore it:
						::memcpy(&m_xkk[lm_idx_in_statevector],&x_feat[0],sizeof(m_xkk[0])*FEAT_SIZE);

						if (KF_options.debug_verify_analytic_jacobians)
						{
							KFMatrix_OxV Hx_gt(mrpt::math::UNINITIALIZED_MATRIX);
							KFMatrix_OxF Hy_gt(mrpt::math::UNINITIALIZED_MATRIX);
							OnObservationJacobians(lm_idx,Hx_gt,Hy_gt);
							if ((Hx-Hx_gt).array().abs().sum()>KF_options.debug_verify_analytic_jacobians_threshold) {
								std::cerr << "[KalmanFilter] ERROR: User analytical observation Hx Jacobians are wrong: \n"
									<< " Real Hx: \n" << Hx << "\n Analytical Hx:\n" << Hx_gt << "Diff:\n" << Hx-Hx_gt << "\n";
								THROW_EXCEPTION("ERROR: User analytical observation Hx Jacobians are wrong (More details dumped to cerr)")
							}
							if ((Hy-Hy_gt).array().abs().sum()>KF_options.debug_verify_analytic_jacobians_threshold) {
								std::cerr << "[KalmanFilter] ERROR: User analytical observation Hy Jacobians are wrong: \n"
									<< " Real Hy: \n" << Hy << "\n Analytical Hx:\n" << Hy_gt << "Diff:\n" << Hy-Hy_gt << "\n";
								THROW_EXCEPTION("ERROR: User analytical observation Hy Jacobians are wrong (More details dumped to cerr)")
							}
						}
					}
				}
				m_timLogger.leave("KF:5.build Jacobians");

				m_timLogger.enter("KF:6.build S");

				// Compute S:  S = H P ~H + R  (R will be added below)
				//  exploiting the sparsity of H:
				// Each block in S is:
				//    Sij =
				// ------------------------------------------
				S.setSize(N_pred*OBS_SIZE,N_pred*OBS_SIZE);

				if ( FEAT_SIZE>0 )
				{	// SLAM-like problem:
					const Eigen::Block<const typename KFMatrix::Base,VEH_SIZE,VEH_SIZE>  Px(m_pkk,0,0);  // Covariance of the vehicle pose

					for (size_t i=0;i<N_pred;++i)
					{
						const size_t lm_idx_i = predictLMidxs[i];
						const Eigen::Block<const typename KFMatrix::Base,FEAT_SIZE,VEH_SIZE>   Pxyi_t(m_pkk,VEH_SIZE+lm_idx_i*FEAT_SIZE,0);  // Pxyi^t

						// Only do j>=i (upper triangle), since S is symmetric:
						for (size_t j=i;j<N_pred;++j)
						{
							const size_t lm_idx_j = predictLMidxs[j];
							// Sij block:
							Eigen::Block<typename KFMatrix::Base, OBS_SIZE, OBS_SIZE> Sij(S,OBS_SIZE*i,OBS_SIZE*j);

							const Eigen::Block<const typename KFMatrix::Base,VEH_SIZE,FEAT_SIZE>   Pxyj(m_pkk,0, VEH_SIZE+lm_idx_j*FEAT_SIZE);
							const Eigen::Block<const typename KFMatrix::Base,FEAT_SIZE,FEAT_SIZE>  Pyiyj(m_pkk,VEH_SIZE+lm_idx_i*FEAT_SIZE,VEH_SIZE+lm_idx_j*FEAT_SIZE);

							Sij = Hxs[i] * Px * Hxs[j].transpose()
								+ Hys[i] * Pxyi_t * Hxs[j].transpose()
								+ Hxs[i] * Pxyj * Hys[j].transpose()
								+ Hys[i] * Pyiyj * Hys[j].transpose();

							// Copy transposed to the symmetric lower-triangular part:
							if (i!=j)
								Eigen::Block<typename KFMatrix::Base, OBS_SIZE, OBS_SIZE>(S,OBS_SIZE*j,OBS_SIZE*i) = Sij.transpose();
						}

						// Sum the "R" term to the diagonal blocks:
						const size_t obs_idx_off = i*OBS_SIZE;
						Eigen::Block<typename KFMatrix::Base, OBS_SIZE, OBS_SIZE>(S,obs_idx_off,obs_idx_off) += R;
					}
				}
				else
				{ // Not SLAM-like problem: simply S=H*Pkk*H^t + R
					ASSERTDEB_(N_pred==1)
						ASSERTDEB_(S.getColCount() == OBS_SIZE )

						S = Hxs[0] * m_pkk *Hxs[0].transpose() + R;
				}

				m_timLogger.leave("KF:6.build S");

				Z.clear();	// Each entry is one observation:

				m_timLogger.enter("KF:7.get obs & DA");

				// Get observations and do data-association:
				OnGetObservationsAndDataAssociation(
					Z, data_association, // Out
					all_predictions, S, predictLMidxs, R  // In
					);
				ASSERTDEB_(data_association.size()==Z.size() || (data_association.empty() && FEAT_SIZE==0));

				// Check if an observation hasn't been predicted in OnPreComputingPredictions() but has been actually
				//  observed. This may imply an error in the heuristic of OnPreComputingPredictions(), and forces us
				//  to rebuild the matrices
				missing_predictions_to_add.clear();
				if (FEAT_SIZE!=0)
				{
					for (size_t i=0;i<data_association.size();++i)
					{
						if (data_association[i]<0) continue;
						const size_t assoc_idx_in_map = static_cast<size_t>(data_association[i]);
						const size_t assoc_idx_in_pred = mrpt::utils::find_in_vector(assoc_idx_in_map, predictLMidxs);
						if (assoc_idx_in_pred==std::string::npos)
							missing_predictions_to_add.push_back(assoc_idx_in_map);
					}
				}

				first_new_pred = N_pred;  // If we do another loop, start at the begin of new predictions

			} while (!missing_predictions_to_add.empty());


			const double tim_obs_DA = m_timLogger.leave("KF:7.get obs & DA");

			// =============================================================
			//  7. UPDATE USING THE KALMAN GAIN
			// =============================================================
			// Update, only if there are observations!
			if ( !Z.empty() )
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
						vector_int  mapIndicesForKFUpdate(data_association.size());
						mapIndicesForKFUpdate.resize( std::distance(mapIndicesForKFUpdate.begin(),
							std::remove_copy_if(
							data_association.begin(),
							data_association.end(),
							mapIndicesForKFUpdate.begin(),
							bind1st(equal_to<int>(),-1) ) ) );

						const size_t N_upd = (FEAT_SIZE==0) ?
							1 : 	// Non-SLAM problems: Just one observation for the entire system.
						mapIndicesForKFUpdate.size(); // SLAM: # of observed known landmarks

						// Just one, or several update iterations??
						const size_t nKF_iterations = (KF_options.method==kfEKFNaive) ?  1 : KF_options.IKF_iterations;

						const KFVector xkk_0 = m_xkk;

						// For each IKF iteration (or 1 for EKF)
						if (N_upd>0) // Do not update if we have no observations!
						{
							for (size_t IKF_iteration=0;IKF_iteration<nKF_iterations;IKF_iteration++)
							{
								// Compute ytilde = OBS - PREDICTION
								KFVector  ytilde(OBS_SIZE*N_upd);
								size_t    ytilde_idx = 0;

								// TODO: Use a Matrix view of "dh_dx_full" instead of creating a copy into "dh_dx_full_obs"
								dh_dx_full_obs.zeros(N_upd*OBS_SIZE, VEH_SIZE + FEAT_SIZE * N_map ); // Init to zeros.
								KFMatrix S_observed;	// The KF "S" matrix: A re-ordered, subset, version of the prediction S:

								if (FEAT_SIZE!=0)
								{	// SLAM problems:
									vector_size_t S_idxs;
									S_idxs.reserve(OBS_SIZE*N_upd);

									//const size_t row_len = VEH_SIZE + FEAT_SIZE * N_map;

									for (size_t i=0;i<data_association.size();++i)
									{
										if (data_association[i]<0) continue;

										const size_t assoc_idx_in_map = static_cast<size_t>(data_association[i]);
										const size_t assoc_idx_in_pred = mrpt::utils::find_in_vector(assoc_idx_in_map, predictLMidxs);
										ASSERTMSG_(assoc_idx_in_pred!=string::npos, "OnPreComputingPredictions() didn't recommend the prediction of a landmark which has been actually observed!")
											// TODO: In these cases, extend the prediction right now instead of launching an exception... or is this a bad idea??

											// Build the subset dh_dx_full_obs:
											//										dh_dx_full_obs.block(S_idxs.size()             ,0, OBS_SIZE, row_len)
											//										=
											//										dh_dx_full.block    (assoc_idx_in_pred*OBS_SIZE,0, OBS_SIZE, row_len);

											Eigen::Block<typename KFMatrix::Base,OBS_SIZE,VEH_SIZE> (dh_dx_full_obs, S_idxs.size(),0) = Hxs[assoc_idx_in_pred];
										Eigen::Block<typename KFMatrix::Base,OBS_SIZE,FEAT_SIZE>(dh_dx_full_obs, S_idxs.size(), VEH_SIZE+assoc_idx_in_map*FEAT_SIZE ) = Hys[assoc_idx_in_pred];

										// S_idxs.size() is used as counter for "dh_dx_full_obs".
										for (size_t k=0;k<OBS_SIZE;k++)
											S_idxs.push_back(assoc_idx_in_pred*OBS_SIZE+k);

										// ytilde_i = Z[i] - all_predictions[i]
										KFArray_OBS ytilde_i = Z[i];
										OnSubstractObservationVectors(ytilde_i,all_predictions[predictLMidxs[assoc_idx_in_pred]]);
										for (size_t k=0;k<OBS_SIZE;k++)
											ytilde[ytilde_idx++] = ytilde_i[k];
									}
									// Extract the subset that is involved in this observation:
									S.extractSubmatrixSymmetrical(S_idxs,S_observed);
								}
								else
								{	// Non-SLAM problems:
									ASSERT_(Z.size()==1 && all_predictions.size()==1)
										ASSERT_(dh_dx_full_obs.getRowCount()==OBS_SIZE && dh_dx_full_obs.getColCount()==VEH_SIZE)
										ASSERT_(Hxs.size()==1)

										dh_dx_full_obs = Hxs[0];  // Was: dh_dx_full
									KFArray_OBS ytilde_i = Z[0];
									OnSubstractObservationVectors(ytilde_i,all_predictions[0]);
									for (size_t k=0;k<OBS_SIZE;k++)
										ytilde[ytilde_idx++] = ytilde_i[k];
									// Extract the subset that is involved in this observation:
									S_observed = S;
								}

								// Compute the full K matrix:
								// ------------------------------
								m_timLogger.enter("KF:8.update stage:1.FULLKF:build K");

								K.setSize(m_pkk.getRowCount(), S_observed.getColCount() );

								// K = m_pkk * (~dh_dx) * S.inv() );
								K.multiply_ABt(m_pkk, dh_dx_full_obs);

								// Inverse of S_observed -> S_1
								S_observed.inv(S_1);
								K*=S_1;

								m_timLogger.leave("KF:8.update stage:1.FULLKF:build K");

								// Use the full K matrix to update the mean:
								if (nKF_iterations==1)
								{
									m_timLogger.enter("KF:8.update stage:2.FULLKF:update xkk");
									m_xkk += K * ytilde;
									m_timLogger.leave("KF:8.update stage:2.FULLKF:update xkk");
								}
								else
								{
									m_timLogger.enter("KF:8.update stage:2.FULLKF:iter.update xkk");

									KFVector  HAx_column;
									dh_dx_full_obs.multiply_Ab( m_xkk - xkk_0, HAx_column);

									m_xkk = xkk_0;
									K.multiply_Ab(
										(ytilde-HAx_column),
										m_xkk,
										true /* Accumulate in output */
										);

									m_timLogger.leave("KF:8.update stage:2.FULLKF:iter.update xkk");
								}

								// Update the covariance just at the end
								//  of iterations if we are in IKF, always in normal EKF.
								if (IKF_iteration == (nKF_iterations-1) )
								{
									m_timLogger.enter("KF:8.update stage:3.FULLKF:update Pkk");

									// Use the full K matrix to update the covariance:
									//m_pkk = (I - K*dh_dx ) * m_pkk;
									// TODO: "Optimize this: sparsity!"

									// K * dh_dx_full_obs
									aux_K_dh_dx.multiply(K,dh_dx_full_obs);

									// aux_K_dh_dx  <-- I-aux_K_dh_dx
									const size_t stat_len = aux_K_dh_dx.getColCount();
									for (size_t r=0;r<stat_len;r++) {
										for (size_t c=0;c<stat_len;c++) {
											if (r==c)
												aux_K_dh_dx.get_unsafe(r,c)=-aux_K_dh_dx.get_unsafe(r,c) + kftype(1);
											else aux_K_dh_dx.get_unsafe(r,c)=-aux_K_dh_dx.get_unsafe(r,c);
										}
									}

									m_pkk.multiply_result_is_symmetric(aux_K_dh_dx, m_pkk );

									m_timLogger.leave("KF:8.update stage:3.FULLKF:update Pkk");
								}
							} // end for each IKF iteration
						}
					}
					break;

					// --------------------------------------------------------------------
					// - EKF 'a la' Davison: One observation element at once
					// --------------------------------------------------------------------
				case kfEKFAlaDavison:
					{
						// For each observed landmark/whole system state:
						for (size_t obsIdx=0;obsIdx<Z.size();obsIdx++)
						{
							// Known & mapped landmark?
							bool   doit;
							size_t idxInTheFilter=0;

							if (data_association.empty())
							{
								doit = true;
							}
							else
							{
								doit = data_association[obsIdx] >= 0;
								if (doit)
									idxInTheFilter = data_association[obsIdx];
							}

							if ( doit )
							{
								m_timLogger.enter("KF:8.update stage:1.ScalarAtOnce.prepare");

								// Already mapped: OK
								const size_t idx_off = VEH_SIZE + idxInTheFilter*FEAT_SIZE; // The offset in m_xkk & Pkk.

								// Compute just the part of the Jacobian that we need using the current updated m_xkk:
								vector_KFArray_OBS  pred_obs;
								OnObservationModel( vector_size_t(1,idxInTheFilter),pred_obs);
								ASSERTDEB_(pred_obs.size()==1);

								// ytilde = observation - prediction
								KFArray_OBS ytilde = Z[obsIdx];
								OnSubstractObservationVectors(ytilde, pred_obs[0]);

								// Jacobians:
								// dh_dx: already is (N_pred*OBS_SIZE) x (VEH_SIZE + FEAT_SIZE * N_pred )
								//         with N_pred = |predictLMidxs|

								const size_t i_idx_in_preds = mrpt::utils::find_in_vector(idxInTheFilter,predictLMidxs);
								ASSERTMSG_(i_idx_in_preds!=string::npos, "OnPreComputingPredictions() didn't recommend the prediction of a landmark which has been actually observed!")

									const KFMatrix_OxV & Hx = Hxs[i_idx_in_preds];
								const KFMatrix_OxF & Hy = Hys[i_idx_in_preds];

								m_timLogger.leave("KF:8.update stage:1.ScalarAtOnce.prepare");

								// For each component of the observation:
								for (size_t j=0;j<OBS_SIZE;j++)
								{
									m_timLogger.enter("KF:8.update stage:2.ScalarAtOnce.update");

									// Compute the scalar S_i for each component j of the observation:
									// Sij = dhij_dxv Pxx dhij_dxv^t + 2 * dhij_dyi Pyix dhij_dxv + dhij_dyi Pyiyi dhij_dyi^t + R
									//          ^^
									//         Hx:(O*LxSv)
									//       \----------------------/ \--------------------------/  \------------------------/ \-/
									//               TERM 1                   TERM 2                        TERM 3              R
									//
									// O: Observation size (3)
									// L: # landmarks
									// Sv: Vehicle state size (6)
									//

#if defined(_DEBUG)
									{
										// This algorithm is applicable only if the scalar component of the sensor noise are INDEPENDENT:
										for (size_t a=0;a<OBS_SIZE;a++)
											for (size_t b=0;b<OBS_SIZE;b++)
												if ( a!=b )
													if (R(a,b)!=0)
														THROW_EXCEPTION("This KF algorithm assumes independent noise components in the observation (matrix R). Select another KF algorithm.")
									}
#endif
									// R:
									KFTYPE Sij = R.get_unsafe(j,j);

									// TERM 1:
									for (size_t k=0;k<VEH_SIZE;k++)
									{
										KFTYPE accum = 0;
										for (size_t q=0;q<VEH_SIZE;q++)
											accum += Hx.get_unsafe(j,q) * m_pkk.get_unsafe(q,k);
										Sij+= Hx.get_unsafe(j,k) * accum;
									}

									// TERM 2:
									KFTYPE term2=0;
									for (size_t k=0;k<VEH_SIZE;k++)
									{
										KFTYPE accum = 0;
										for (size_t q=0;q<FEAT_SIZE;q++)
											accum += Hy.get_unsafe(j,q) * m_pkk.get_unsafe(idx_off+q,k);
										term2+= Hx.get_unsafe(j,k) * accum;
									}
									Sij += 2 * term2;

									// TERM 3:
									for (size_t k=0;k<FEAT_SIZE;k++)
									{
										KFTYPE accum = 0;
										for (size_t q=0;q<FEAT_SIZE;q++)
											accum += Hy.get_unsafe(j,q) * m_pkk.get_unsafe(idx_off+q,idx_off+k);
										Sij+= Hy.get_unsafe(j,k) * accum;
									}

									// Compute the Kalman gain "Kij" for this observation element:
									// -->  K = m_pkk * (~dh_dx) * S.inv() );
									size_t N = m_pkk.getColCount();
									vector<KFTYPE> Kij( N );

									for (size_t k=0;k<N;k++)
									{
										KFTYPE K_tmp = 0;

										// dhi_dxv term
										size_t q;
										for (q=0;q<VEH_SIZE;q++)
											K_tmp+= m_pkk.get_unsafe(k,q) * Hx.get_unsafe(j,q);

										// dhi_dyi term
										for (q=0;q<FEAT_SIZE;q++)
											K_tmp+= m_pkk.get_unsafe(k,idx_off+q) * Hy.get_unsafe(j,q);

										Kij[k] = K_tmp / Sij;
									} // end for k


									// Update the state vector m_xkk:
									//  x' = x + Kij * ytilde(ij)
									for (size_t k=0;k<N;k++)
										m_xkk[k] += Kij[k] * ytilde[j];

									// Update the covariance Pkk:
									// P' =  P - Kij * Sij * Kij^t
									{
										for (size_t k=0;k<N;k++)
										{
											for (size_t q=k;q<N;q++)  // Half matrix
											{
												m_pkk(k,q) -= Sij * Kij[k] * Kij[q];
												// It is symmetric
												m_pkk(q,k) = m_pkk(k,q);
											}

#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
											if (m_pkk(k,k)<0)
											{
												m_pkk.saveToTextFile("Pkk_err.txt");
												mrpt::system::vectorToTextFile(Kij,"Kij.txt");
												ASSERT_(m_pkk(k,k)>0)
											}
#endif
										}
									}


									m_timLogger.leave("KF:8.update stage:2.ScalarAtOnce.update");
								} // end for j
							} // end if mapped
						} // end for each observed LM.
					}
					break;

					// --------------------------------------------------------------------
					// - IKF method, processing each observation scalar secuentially:
					// --------------------------------------------------------------------
				case kfIKF:  // TODO !!
					{
						THROW_EXCEPTION("IKF scalar by scalar not implemented yet.");
#if 0
						KFMatrix h,Hx,Hy;

						// Just one, or several update iterations??
						size_t nKF_iterations = KF_options.IKF_iterations;

						// To avoid wasting time, if we are doing 1 iteration only, do not reserve memory for this matrix:
						KFMatrix		*saved_Pkk=NULL;
						if (nKF_iterations>1)
						{
							// Create a copy of Pkk for later restoring it at the beginning of each iteration:
							saved_Pkk = new KFMatrix( m_pkk );
						}

						KFVector	xkk_0 = m_xkk; // First state vector at the beginning of IKF
						KFVector	xkk_next_iter = m_xkk;  // for IKF only

						// For each IKF iteration (or 1 for EKF)
						for (size_t IKF_iteration=0;IKF_iteration<nKF_iterations;IKF_iteration++)
						{
							// Restore initial covariance matrix.
							// The updated mean is stored in m_xkk and is improved with each estimation,
							//  and so do the Jacobians which use that improving mean.
							if (IKF_iteration>0)
							{
								m_pkk = *saved_Pkk;
								xkk_next_iter = xkk_0;
							}

							// For each observed landmark/whole system state:
							for (size_t obsIdx=0;obsIdx<Z.getRowCount();obsIdx++)
							{
								// Known & mapped landmark?
								bool   doit;
								size_t idxInTheFilter=0;

								if (data_association.empty())
								{
									doit = true;
								}
								else
								{
									doit = data_association[obsIdx] >= 0;
									if (doit)
										idxInTheFilter = data_association[obsIdx];
								}

								if ( doit )
								{
									// Already mapped: OK
									const size_t idx_off      = VEH_SIZE + idxInTheFilter *FEAT_SIZE; // The offset in m_xkk & Pkk.
									const size_t R_row_offset = obsIdx*OBS_SIZE;  // Offset row in R

									// Compute just the part of the Jacobian that we need using the current updated m_xkk:
									KFVector   ytilde; // Diff. observation - prediction
									OnObservationModelAndJacobians(
										Z,
										data_association,
										false, 			// Only a partial Jacobian
										(int)obsIdx,  	// Which row from Z
										ytilde,
										Hx,
										Hy );

									ASSERTDEB_(ytilde.size() == OBS_SIZE )
										ASSERTDEB_(Hx.getRowCount() == OBS_SIZE )
										ASSERTDEB_(Hx.getColCount() == VEH_SIZE )

										if (FEAT_SIZE>0)
										{
											ASSERTDEB_(Hy.getRowCount() == OBS_SIZE )
												ASSERTDEB_(Hy.getColCount() == FEAT_SIZE )
										}

										// Compute the OxO matrix S_i for each observation:
										// Si  = TERM1 + TERM2 + TERM2^t + TERM3 + R
										//
										// TERM1: dhi_dxv Pxx dhi_dxv^t
										//          ^^
										//         Hx:(OxV)
										//
										// TERM2: dhi_dyi Pyix dhi_dxv
										//          ^^
										//         Hy:(OxF)
										//
										// TERM3: dhi_dyi Pyiyi dhi_dyi^t
										//
										// i: obsIdx
										// O: Observation size
										// F: Feature size
										// V: Vehicle state size
										//

										// R:
										KFMatrix Si(OBS_SIZE,OBS_SIZE);
										R.extractMatrix(R_row_offset,0, Si);

										size_t k;
										KFMatrix	term(OBS_SIZE,OBS_SIZE);

										// TERM1: dhi_dxv Pxx dhi_dxv^t
										Hx.multiply_HCHt(
											m_pkk, 		// The cov. matrix
											Si,			// The output
											true,		// Yes, submatrix of m_pkk only
											0, 			// Offset in m_pkk
											true		// Yes, accum results in output.
											);

										// TERM2: dhi_dyi Pyix dhi_dxv
										//  + its transpose:
										KFMatrix	Pyix( FEAT_SIZE, VEH_SIZE );
										m_pkk.extractMatrix(idx_off,0, Pyix);  // Extract cross cov. to Pyix

										term.multiply_ABCt( Hy, Pyix, Hx ); //term = Hy * Pyix * ~Hx;
										Si.add_AAt( term );  // Si += A + ~A

										// TERM3: dhi_dyi Pyiyi dhi_dyi^t
										Hy.multiply_HCHt(
											m_pkk, 		// The cov. matrix
											Si,			// The output
											true,		// Yes, submatrix of m_pkk only
											idx_off,    // Offset in m_pkk
											true		// Yes, accum results in output.
											);

										// Compute the inverse of Si:
										KFMatrix Si_1(OBS_SIZE,OBS_SIZE);

										// Compute the Kalman gain "Ki" for this i'th observation:
										// -->  Ki = m_pkk * (~dh_dx) * S.inv();
										size_t N = m_pkk.getColCount();

										KFMatrix Ki( N, OBS_SIZE );

										for (k=0;k<N;k++)  // for each row of K
										{
											size_t q;

											for (size_t c=0;c<OBS_SIZE;c++)  // for each column of K
											{
												KFTYPE	K_tmp = 0;

												// dhi_dxv term
												for (q=0;q<VEH_SIZE;q++)
													K_tmp+= m_pkk.get_unsafe(k,q) * Hx.get_unsafe(c,q);

												// dhi_dyi term
												for (q=0;q<FEAT_SIZE;q++)
													K_tmp+= m_pkk.get_unsafe(k,idx_off+q) * Hy.get_unsafe(c,q);

												Ki.set_unsafe(k,c, K_tmp);
											} // end for c
										} // end for k

										Ki.multiply(Ki, Si.inv() );  // K = (...) * S^-1


										// Update the state vector m_xkk:
										if (nKF_iterations==1)
										{
											// EKF:
											//  x' = x + Kij * ytilde(ij)
											for (k=0;k<N;k++)
												for (size_t q=0;q<OBS_SIZE;q++)
													m_xkk[k] += Ki.get_unsafe(k,q) * ytilde[q];
										}
										else
										{
											// IKF:
											Eigen::Matrix<KFTYPE,Eigen::Dynamic,1> HAx(OBS_SIZE);
											size_t o,q;
											// HAx = H*(x0-xi)
											for (o=0;o<OBS_SIZE;o++)
											{
												KFTYPE	tmp = 0;
												for (q=0;q<VEH_SIZE;q++)
													tmp += Hx.get_unsafe(o,q) * (xkk_0[q] - m_xkk[q]);

												for (q=0;q<FEAT_SIZE;q++)
													tmp += Hy.get_unsafe(o,q) * (xkk_0[idx_off+q] - m_xkk[idx_off+q]);

												HAx[o] = tmp;
											}

											// Compute only once (ytilde[j] - HAx)
											for (o=0;o<OBS_SIZE;o++)
												HAx[o] = ytilde[o] - HAx[o];

											//  x' = x_0 + Kij * ( ytilde(ij) - H*(x0-xi))   -> xi: i=this iteration
											//  xkk_next_iter is initialized to xkk_0 at each IKF iteration.
											for (k=0;k<N;k++)
											{
												KFTYPE	 tmp = xkk_next_iter[k];
												//m_xkk[k] = xkk_0[k] + Kij[k] * (ytilde[j] - HAx );
												for (o=0;o<OBS_SIZE;o++)
													tmp += Ki.get_unsafe(k,o) * HAx[o];

												xkk_next_iter[k] = tmp;
											}
										}

										// Update the covariance Pkk:
										// P' =  P - Kij * Sij * Kij^t
										//if (IKF_iteration==(nKF_iterations-1))   // Just for the last IKF iteration
										{
											// m_pkk -= Ki * Si * ~Ki;
											Ki.multiplyByMatrixAndByTransposeNonSymmetric(
												Si,
												m_pkk,   // Output
												true,    // Accumulate
												true);   // Substract instead of sum.

											m_pkk.force_symmetry();

											/*							for (k=0;k<N;k++)
											{
											for (size_t q=k;q<N;q++)  // Half matrix
											{
											m_pkk(k,q) -= Sij * Kij[k] * Kij[q];
											// It is symmetric
											m_pkk(q,k) = m_pkk(k,q);
											}

											#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
											if (m_pkk(k,k)<0)
											{
											m_pkk.saveToTextFile("Pkk_err.txt");
											mrpt::system::vectorToTextFile(Kij,"Kij.txt");
											ASSERT_(m_pkk(k,k)>0)
											}
											#endif
											}
											*/
										}

								} // end if doit

							} // end for each observed LM.

							// Now, save the new iterated mean:
							if (nKF_iterations>1)
							{
#if 0
								cout << "IKF iter: " << IKF_iteration << " -> " << xkk_next_iter << endl;
#endif
								m_xkk = xkk_next_iter;
							}

						} // end for each IKF_iteration

						// Copy of pkk not required anymore:
						if (saved_Pkk) delete saved_Pkk;

#endif
					}
					break;

				default:
					THROW_EXCEPTION("Invalid value of options.KF_method");
				} // end switch method

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
				detail::addNewLandmarks(*this, Z, data_association,R);
				m_timLogger.leave("KF:A.add new landmarks");
			} // end if data_association!=empty

			// Post iteration user code:
			m_timLogger.enter("KF:B.OnPostIteration");
			OnPostIteration();
			m_timLogger.leave("KF:B.OnPostIteration");

			m_timLogger.leave("KF:complete_step");

			MRPT_LOG_DEBUG( mrpt::format("[KF] %u LMs | Pr: %.2fms | Pr.Obs: %.2fms | Obs.DA: %.2fms | Upd: %.2fms\n",
				static_cast<unsigned int>(getNumberOfLandmarksInTheMap()),
				1e3*tim_pred,
				1e3*tim_pred_obs,
				1e3*tim_obs_DA,
				1e3*tim_update
				) );
			MRPT_END
		} // End of "runOneKalmanIteration"



		template <size_t VEH_SIZE, size_t OBS_SIZE, size_t FEAT_SIZE, size_t ACT_SIZE, typename KFTYPE>
		void CKalmanFilterCapable<VEH_SIZE,OBS_SIZE,FEAT_SIZE,ACT_SIZE,KFTYPE>::KF_aux_estimate_trans_jacobian(
			const KFArray_VEH &x,
			const std::pair<KFCLASS*,KFArray_ACT> &dat,
			KFArray_VEH &out_x)
		{
			bool dumm;
			out_x=x;
			dat.first->OnTransitionModel(dat.second,out_x, dumm);
		}

		template <size_t VEH_SIZE, size_t OBS_SIZE, size_t FEAT_SIZE, size_t ACT_SIZE, typename KFTYPE>
		void CKalmanFilterCapable<VEH_SIZE,OBS_SIZE,FEAT_SIZE,ACT_SIZE,KFTYPE>::KF_aux_estimate_obs_Hx_jacobian(
			const KFArray_VEH &x,
			const std::pair<KFCLASS*,size_t> &dat,
			KFArray_OBS &out_x)
		{
			vector_size_t  idxs_to_predict(1,dat.second);
			vector_KFArray_OBS prediction;
			// Overwrite (temporarily!) the affected part of the state vector:
			::memcpy(&dat.first->m_xkk[0],&x[0],sizeof(x[0])*VEH_SIZE);
			dat.first->OnObservationModel(idxs_to_predict,prediction);
			ASSERTDEB_(prediction.size()==1);
			out_x=prediction[0];
		}

		template <size_t VEH_SIZE, size_t OBS_SIZE, size_t FEAT_SIZE, size_t ACT_SIZE, typename KFTYPE>
		void CKalmanFilterCapable<VEH_SIZE,OBS_SIZE,FEAT_SIZE,ACT_SIZE,KFTYPE>::KF_aux_estimate_obs_Hy_jacobian(
			const KFArray_FEAT &x,
			const std::pair<KFCLASS*,size_t> &dat,
			KFArray_OBS &out_x)
		{
			vector_size_t  idxs_to_predict(1,dat.second);
			vector_KFArray_OBS  prediction;
			const size_t lm_idx_in_statevector = VEH_SIZE+FEAT_SIZE*dat.second;
			// Overwrite (temporarily!) the affected part of the state vector:
			::memcpy(&dat.first->m_xkk[lm_idx_in_statevector],&x[0],sizeof(x[0])*FEAT_SIZE);
			dat.first->OnObservationModel(idxs_to_predict,prediction);
			ASSERTDEB_(prediction.size()==1);
			out_x=prediction[0];
		}


		namespace detail
		{
			// generic version for SLAM. There is a speciation below for NON-SLAM problems.
			template <size_t VEH_SIZE, size_t OBS_SIZE, size_t FEAT_SIZE, size_t ACT_SIZE, typename KFTYPE>
			void addNewLandmarks(
				CKalmanFilterCapable<VEH_SIZE,OBS_SIZE,FEAT_SIZE,ACT_SIZE,KFTYPE> &obj,
				const typename CKalmanFilterCapable<VEH_SIZE,OBS_SIZE,FEAT_SIZE,ACT_SIZE,KFTYPE>::vector_KFArray_OBS & Z,
				const vector_int       &data_association,
				const typename CKalmanFilterCapable<VEH_SIZE,OBS_SIZE,FEAT_SIZE,ACT_SIZE,KFTYPE>::KFMatrix_OxO		&R
				)
			{
				typedef CKalmanFilterCapable<VEH_SIZE,OBS_SIZE,FEAT_SIZE,ACT_SIZE,KFTYPE> KF;

				for (size_t idxObs=0;idxObs<Z.size();idxObs++)
				{
					// Is already in the map?
					if ( data_association[idxObs] < 0 )  // Not in the map yet!
					{
						obj.getProfiler().enter("KF:9.create new LMs");
						// Add it:

						// Append to map of IDs <-> position in the state vector:
						ASSERTDEB_(FEAT_SIZE>0)
						ASSERTDEB_( 0 == ((obj.internal_getXkk().size() - VEH_SIZE) % FEAT_SIZE) ) // Sanity test

						const size_t  newIndexInMap = (obj.internal_getXkk().size() - VEH_SIZE) / FEAT_SIZE;

						// Inverse sensor model:
						typename KF::KFArray_FEAT yn;
						typename KF::KFMatrix_FxV dyn_dxv;
						typename KF::KFMatrix_FxO dyn_dhn;
						typename KF::KFMatrix_FxF dyn_dhn_R_dyn_dhnT;
						bool use_dyn_dhn_jacobian=true;

						// Compute the inv. sensor model and its Jacobians:
						obj.OnInverseObservationModel(
							Z[idxObs],
							yn,
							dyn_dxv,
							dyn_dhn,
							dyn_dhn_R_dyn_dhnT,
							use_dyn_dhn_jacobian );

						// And let the application do any special handling of adding a new feature to the map:
						obj.OnNewLandmarkAddedToMap(
							idxObs,
							newIndexInMap
							);

						ASSERTDEB_( yn.size() == FEAT_SIZE )

							// Append to m_xkk:
							size_t q;
						size_t idx = obj.internal_getXkk().size();
						obj.internal_getXkk().conservativeResize( obj.internal_getXkk().size() + FEAT_SIZE );

						for (q=0;q<FEAT_SIZE;q++)
							obj.internal_getXkk()[idx+q] = yn[q];

						// --------------------
						// Append to Pkk:
						// --------------------
						ASSERTDEB_( obj.internal_getPkk().getColCount()==idx && obj.internal_getPkk().getRowCount()==idx );

						obj.internal_getPkk().setSize( idx+FEAT_SIZE,idx+FEAT_SIZE );

						// Fill the Pxyn term:
						// --------------------
						typename KF::KFMatrix_VxV Pxx;
						obj.internal_getPkk().extractMatrix(0,0,Pxx);
						typename KF::KFMatrix_FxV Pxyn; // Pxyn = dyn_dxv * Pxx
						Pxyn.multiply( dyn_dxv, Pxx );

						obj.internal_getPkk().insertMatrix( idx,0, Pxyn );
						obj.internal_getPkk().insertMatrixTranspose( 0,idx, Pxyn );

						// Fill the Pyiyn terms:
						// --------------------
						const size_t nLMs = (idx-VEH_SIZE)/FEAT_SIZE; // Number of previous landmarks:
						for (q=0;q<nLMs;q++)
						{
							typename KF::KFMatrix_VxF  P_x_yq(mrpt::math::UNINITIALIZED_MATRIX);
							obj.internal_getPkk().extractMatrix(0,VEH_SIZE+q*FEAT_SIZE,P_x_yq) ;

							typename KF::KFMatrix_FxF P_cross(mrpt::math::UNINITIALIZED_MATRIX);
							P_cross.multiply(dyn_dxv, P_x_yq );

							obj.internal_getPkk().insertMatrix(idx,VEH_SIZE+q*FEAT_SIZE, P_cross );
							obj.internal_getPkk().insertMatrixTranspose(VEH_SIZE+q*FEAT_SIZE,idx, P_cross );
						} // end each previous LM(q)

						// Fill the Pynyn term:
						//  P_yn_yn =  (dyn_dxv * Pxx * ~dyn_dxv) + (dyn_dhn * R * ~dyn_dhn);
						// --------------------
						typename KF::KFMatrix_FxF P_yn_yn(mrpt::math::UNINITIALIZED_MATRIX);
						dyn_dxv.multiply_HCHt(Pxx,  P_yn_yn);
						if (use_dyn_dhn_jacobian)
							dyn_dhn.multiply_HCHt(R, P_yn_yn, true); // Accumulate in P_yn_yn
						else 	P_yn_yn+=dyn_dhn_R_dyn_dhnT;

						obj.internal_getPkk().insertMatrix(idx,idx, P_yn_yn );

						obj.getProfiler().leave("KF:9.create new LMs");
					}
				}
			}

			template <size_t VEH_SIZE, size_t OBS_SIZE, size_t ACT_SIZE, typename KFTYPE>
			void addNewLandmarks(
				CKalmanFilterCapable<VEH_SIZE,OBS_SIZE,0 /* FEAT_SIZE=0 */,ACT_SIZE,KFTYPE> &obj,
				const typename CKalmanFilterCapable<VEH_SIZE,OBS_SIZE,0 /* FEAT_SIZE=0 */,ACT_SIZE,KFTYPE>::vector_KFArray_OBS & Z,
				const vector_int       &data_association,
				const typename CKalmanFilterCapable<VEH_SIZE,OBS_SIZE,0 /* FEAT_SIZE=0 */,ACT_SIZE,KFTYPE>::KFMatrix_OxO		&R
				)
			{
				// Do nothing: this is NOT a SLAM problem.
			}

			template <size_t VEH_SIZE, size_t OBS_SIZE, size_t FEAT_SIZE, size_t ACT_SIZE, typename KFTYPE>
			inline size_t getNumberOfLandmarksInMap(const CKalmanFilterCapable<VEH_SIZE,OBS_SIZE,FEAT_SIZE,ACT_SIZE,KFTYPE> &obj)
			{
				return (obj.getStateVectorLength()-VEH_SIZE)/FEAT_SIZE;
			}
			// Specialization for FEAT_SIZE=0
			template <size_t VEH_SIZE, size_t OBS_SIZE, size_t ACT_SIZE, typename KFTYPE>
			inline size_t getNumberOfLandmarksInMap(const CKalmanFilterCapable<VEH_SIZE,OBS_SIZE,0 /*FEAT_SIZE*/,ACT_SIZE,KFTYPE> &obj)
			{
				return 0;
			}

			template <size_t VEH_SIZE, size_t OBS_SIZE, size_t FEAT_SIZE, size_t ACT_SIZE, typename KFTYPE>
			inline bool isMapEmpty(const CKalmanFilterCapable<VEH_SIZE,OBS_SIZE,FEAT_SIZE,ACT_SIZE,KFTYPE> &obj)
			{
				return (obj.getStateVectorLength()==VEH_SIZE);
			}
			// Specialization for FEAT_SIZE=0
			template <size_t VEH_SIZE, size_t OBS_SIZE, size_t ACT_SIZE, typename KFTYPE>
			inline bool isMapEmpty (const CKalmanFilterCapable<VEH_SIZE,OBS_SIZE,0 /*FEAT_SIZE*/,ACT_SIZE,KFTYPE> &obj)
			{
				return true;
			}
		} // end namespace "detail"
	} // ns 
} // ns

#endif
