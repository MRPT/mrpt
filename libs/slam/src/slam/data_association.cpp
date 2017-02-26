/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "slam-precomp.h"   // Precompiled headers

/*
   For all data association algorithms, the individual compatibility is estabished by
   the chi2 test (Mahalanobis distance), but later on one of the potential pairings is
   picked by the metric given by the user (maha vs. match. lik.)

   Related papers:
	- Matching likelihood. See:  http://www.mrpt.org/Paper:Matching_Likelihood

	- JCBB: Joint Compatibility Branch & Bound [Neira, Tardos 2001]

*/

#include <mrpt/slam/data_association.h>
#include <mrpt/math/distributions.h>  // for chi2inv
#include <mrpt/math/data_utils.h>
#include <mrpt/poses/CPointPDFGaussian.h>
#include <mrpt/poses/CPoint2DPDFGaussian.h>

#include <set>
#include <numeric>  // accumulate
#include <memory>   // auto_ptr, unique_ptr

#include <mrpt/otherlibs/nanoflann/nanoflann.hpp> // For kd-tree's
#include <mrpt/math/KDTreeCapable.h>   // For kd-tree's

using namespace std;
using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::slam;
using namespace mrpt::utils;


namespace mrpt
{
namespace slam
{
	struct TAuxDataRecursiveJCBB
	{
		size_t nPredictions, nObservations, length_O; //!< Just to avoid recomputing them all the time.
		std::map<size_t,size_t>	currentAssociation;
	};

/**  Computes the joint distance metric (mahalanobis or matching likelihood) between two  a set of associations
  *
  * On "currentAssociation":  maps "ID_obs" -> "ID_pred"
  *  For each landmark ID in the observations (ID_obs), its association
  *  in the predictions, that is: ID_pred = associations[ID_obs]
  *
  */
template <typename T, TDataAssociationMetric METRIC>
double joint_pdf_metric (
	const CMatrixTemplateNumeric<T>		&Z_observations_mean,
	const CMatrixTemplateNumeric<T>		&Y_predictions_mean,
	const CMatrixTemplateNumeric<T>		&Y_predictions_cov,
	const TAuxDataRecursiveJCBB			&info,
	const TDataAssociationResults	&aux_data)
{
	MRPT_UNUSED_PARAM(aux_data);
	// Make a list of the indices of the predictions that appear in "currentAssociation":
	const size_t  N = info.currentAssociation.size();
	ASSERT_(N>0)

	vector_size_t  indices_pred(N);  // Appearance order indices in the std::maps
	vector_size_t  indices_obs(N);

	{
		size_t i=0;
		for (map<size_t,size_t>::const_iterator it=info.currentAssociation.begin();it!=info.currentAssociation.end();++it)
		{
			indices_obs[i]  = it->first;
			indices_pred[i] = it->second;
			i++;
		}
	}

	// ----------------------------------------------------------------------
	// Extract submatrix of the covariances involved here:
	//  COV = PREDICTIONS_COV(INDX,INDX) + OBSERVATIONS_COV(INDX2,INDX2)
	// ----------------------------------------------------------------------
	Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> COV;
	Y_predictions_cov.extractSubmatrixSymmetricalBlocks(
		info.length_O, // dims of cov. submatrices
		indices_pred,
		COV );

	// ----------------------------------------------------------------------
	// Mean:
	// The same for the vector of "errors" or "innovation" between predictions and observations:
	// ----------------------------------------------------------------------
	Eigen::Matrix<T,Eigen::Dynamic,1>  innovations(N * info.length_O);
	T *dst_ptr= &innovations[0];
	for (map<size_t,size_t>::const_iterator it=info.currentAssociation.begin();it!=info.currentAssociation.end();++it)
	{
		const T *pred_i_mean = Y_predictions_mean.get_unsafe_row( it->second );
		const T *obs_i_mean  = Z_observations_mean.get_unsafe_row( it->first );

		for (unsigned int k=0;k<info.length_O;k++)
			*dst_ptr++ = pred_i_mean[k]-obs_i_mean[k];
	}

	// Compute mahalanobis distance squared:
	CMatrixTemplateNumeric<T>	COV_inv;
	COV.inv_fast(COV_inv);

	const double d2 = mrpt::math::multiply_HCHt_scalar(innovations, COV_inv);

	if (METRIC==metricMaha)
		return d2;

	ASSERT_(METRIC==metricML);

	// Matching likelihood: The evaluation at 0 of the PDF of the difference between the two Gaussians:
	const T cov_det = COV.det();
	const double ml = exp(-0.5*d2) / ( std::pow(M_2PI, info.length_O * 0.5) * std::sqrt(cov_det) );
	return ml;
}

template<TDataAssociationMetric METRIC>
bool isCloser(const double v1, const double v2);

template<>
bool isCloser<metricMaha>(const double v1, const double v2) { return v1<v2; }

template<>
bool isCloser<metricML>(const double v1, const double v2) { return v1>v2; }


/* Based on MATLAB code by:
  University of Zaragoza
  Centro Politecnico Superior
  Robotics and Real Time Group
  Authors of the original MATLAB code:  J. Neira, J. Tardos
  C++ version: J.L. Blanco Claraco
*/
template <typename T, TDataAssociationMetric METRIC>
void JCBB_recursive(
	const mrpt::math::CMatrixTemplateNumeric<T>		&Z_observations_mean,
	const mrpt::math::CMatrixTemplateNumeric<T>		&Y_predictions_mean,
	const mrpt::math::CMatrixTemplateNumeric<T>		&Y_predictions_cov,
	TDataAssociationResults			&results,
	const TAuxDataRecursiveJCBB		&info,
	const observation_index_t		curObsIdx
	)
{
	// End of iteration?
	if (curObsIdx>=info.nObservations)
	{
		if (info.currentAssociation.size()>results.associations.size())
		{
			// It's a better choice since more features are matched.
			results.associations = info.currentAssociation;
			results.distance = joint_pdf_metric<T,METRIC>(
				Z_observations_mean,
				Y_predictions_mean,  Y_predictions_cov,
				info,
				results);
		}
		else if ( !info.currentAssociation.empty() && info.currentAssociation.size()==results.associations.size() )
		{
			// The same # of features matched than the previous best one... decide by better distance:
			const double d2 = joint_pdf_metric<T,METRIC>(
				Z_observations_mean,
				Y_predictions_mean,  Y_predictions_cov,
				info,
				results);

			if (isCloser<METRIC>(d2,results.distance))
			{
				results.associations = info.currentAssociation;
				results.distance = d2;
			}
		}
	}
	else // A normal iteration:
	{
		// Iterate for all compatible landmarsk of "curObsIdx"
		const observation_index_t obsIdx = curObsIdx;

		const size_t nPreds = results.indiv_compatibility.getRowCount();

		// Can we do it better than the current "results.associations"?
		// This can be checked by counting the potential new pairings+the so-far established ones.
		//    Matlab: potentials  = pairings(compatibility.AL(i+1:end))
		// Moved up by Kasra Khosoussi
		const size_t potentials = std::accumulate( results.indiv_compatibility_counts.begin()+(obsIdx+1), results.indiv_compatibility_counts.end(),0 );
		for (prediction_index_t predIdx=0;predIdx<nPreds;predIdx++)
		{
			if ((info.currentAssociation.size() + potentials) >= results.associations.size())
			{
				// Only if predIdx is NOT already assigned:
				if ( results.indiv_compatibility(predIdx,obsIdx) )
				{
					// Only if predIdx is NOT already assigned:
					bool already_asigned = false;
					for (map<size_t,size_t>::const_iterator itS=info.currentAssociation.begin();itS!=info.currentAssociation.end();++itS)
					{
						if (itS->second==predIdx)
						{
							already_asigned = true;
							break;
						}
					}

					if (!already_asigned)
					{
						// Launch a new recursive line for this hipothesis:
						TAuxDataRecursiveJCBB new_info = info;
						new_info.currentAssociation[ curObsIdx ] = predIdx;

						results.nNodesExploredInJCBB++;

						JCBB_recursive<T,METRIC>(
							Z_observations_mean, Y_predictions_mean, Y_predictions_cov,
							results, new_info, curObsIdx+1);
					}
				}
			}
		}

		// Can we do it better than the current "results.associations"?
		if ((info.currentAssociation.size() + potentials) >= results.associations.size() )
		{
			// Yes we can </obama>

			// star node: Ei not paired
			results.nNodesExploredInJCBB++;
			JCBB_recursive<T,METRIC>(
				Z_observations_mean,  Y_predictions_mean, Y_predictions_cov,
				results, info, curObsIdx+1);
		}
	}
}



} // end namespace
} // end namespace


/* ==================================================================================================
Computes the data-association between the prediction of a set of landmarks and their observations, all of them with covariance matrices.
* Implemented methods include (see TDataAssociation)
*		- NN: Nearest-neighbor
*		- JCBB: Joint Compatibility Branch & Bound [Neira, Tardos 2001]
*
*  With both a Mahalanobis-distance or Matching-likelihood metric (See paper: http://www.mrpt.org/Paper:Matching_Likelihood )
*
* \param Z_observations_mean [IN] An MxO matrix with the M observations, each row containing the observation "mean".
* \param Y_predictions_mean [IN ] An NxO matrix with the N predictions, each row containing the mean of one prediction.
* \param Y_predictions_cov [IN ] An N·OxN·O matrix with the full covariance matrix of all the N predictions.

* \param predictions_mean [IN] The list of predicted locations of landmarks/features, indexed by their ID. The 2D/3D locations are in the same coordinate framework than "observations".
* \param predictions_cov [IN] The full covariance matrix of predictions, in blocks of 2x2 matrices. The order of the submatrices is the appearance order of lanmarks in "predictions_mean".
* \param results [OUT] The output data association hypothesis, and other useful information.
* \param method [IN, optional] The selected method to make the associations.
* \param chi2quantile [IN, optional] The threshold for considering a match between two close Gaussians for two landmarks, in the range [0,1]. It is used to call mrpt::math::chi2inv
* \param use_kd_tree [IN, optional] Build a KD-tree to speed-up the evaluation of individual compatibility (IC). It's perhaps more efficient to disable it for a small number of features. (default=true).
* \param predictions_IDs [IN, optional] (default:none) An N-vector. If provided, the resulting associations in "results.associations" will not contain prediction indices "i", but "predictions_IDs[i]".
*
 ==================================================================================================  */
void mrpt::slam::data_association_full_covariance(
	const mrpt::math::CMatrixDouble		&Z_observations_mean,
	const mrpt::math::CMatrixDouble		&Y_predictions_mean,
	const mrpt::math::CMatrixDouble		&Y_predictions_cov,
	TDataAssociationResults				&results,
	const TDataAssociationMethod		method,
	const TDataAssociationMetric		metric,
	const double						chi2quantile,
	const bool							DAT_ASOC_USE_KDTREE,
	const std::vector<prediction_index_t>		&predictions_IDs,
	const TDataAssociationMetric		compatibilityTestMetric,
	const double						log_ML_compat_test_threshold
	)
{
	// For details on the theory, see the papers cited at the beginning of this file.

	using nanoflann::KDTreeEigenMatrixAdaptor;

	MRPT_START

	results.clear();

	const size_t nPredictions  = size(Y_predictions_mean,1);
	const size_t nObservations = size(Z_observations_mean,1);

	const size_t length_O = size(Z_observations_mean,2);

	ASSERT_( nPredictions!=0 )
	ASSERT_( nObservations!=0 )
	ASSERT_( length_O == size(Y_predictions_mean,2))
	ASSERT_( length_O*nPredictions == size(Y_predictions_cov,1))
	ASSERT_( Y_predictions_cov.isSquare() )

	ASSERT_( chi2quantile>0 && chi2quantile<1 )
	ASSERT_( metric==metricMaha || metric==metricML )

	const double chi2thres = mrpt::math::chi2inv( chi2quantile, length_O );

	// ------------------------------------------------------------
	// Build a KD-tree of the predictions for quick look-up:
	// ------------------------------------------------------------
#if MRPT_HAS_CXX11
	typedef std::unique_ptr<KDTreeEigenMatrixAdaptor<CMatrixDouble> > KDTreeMatrixPtr;
#else
	typedef std::auto_ptr<KDTreeEigenMatrixAdaptor<CMatrixDouble> > KDTreeMatrixPtr;
#endif
	KDTreeMatrixPtr  kd_tree;
	const size_t N_KD_RESULTS = nPredictions;
	std::vector<double>	kd_result_distances(DAT_ASOC_USE_KDTREE ? N_KD_RESULTS : 0);
	std::vector<CMatrixDouble::Index> kd_result_indices(DAT_ASOC_USE_KDTREE ? N_KD_RESULTS : 0);
	std::vector<double>	kd_queryPoint(DAT_ASOC_USE_KDTREE ? length_O : 0);

	if (DAT_ASOC_USE_KDTREE)
	{
		// Construct kd-tree for the predictions:
		kd_tree = KDTreeMatrixPtr( new KDTreeEigenMatrixAdaptor<CMatrixDouble>(length_O, Y_predictions_mean) );
	}

	// Initialize with the worst possible distance:
	results.distance = (metric==metricML) ? 0 : std::numeric_limits<double>::max();

	//-------------------------------------------
	// Compute the individual compatibility:
	//-------------------------------------------
	results.indiv_distances.resize(nPredictions,nObservations);
	results.indiv_compatibility.setSize(nPredictions,nObservations);
	results.indiv_compatibility_counts.assign(nObservations,0);

	results.indiv_distances.fill(
		metric==metricMaha ?
			 1000 /*A very large Sq. Maha. Dist. */ :
			-1000 /*A very small log-likelihoo   */ );
	results.indiv_compatibility.fillAll(false);

	CMatrixDouble pred_i_cov(length_O,length_O);

	Eigen::VectorXd  diff_means_i_j(length_O);

	for (size_t j=0;j<nObservations;++j)
	{
		if (!DAT_ASOC_USE_KDTREE)
		{
			// Compute all the distances w/o a KD-tree
			for (size_t i=0;i<nPredictions;++i)
			{
				// Evaluate sqr. mahalanobis distance of obs_j -> pred_i:
				const size_t pred_cov_idx = i*length_O;  // Extract the submatrix from the diagonal:
				Y_predictions_cov.extractMatrix(pred_cov_idx,pred_cov_idx,length_O,length_O, pred_i_cov);

				for (size_t k=0;k<length_O;k++)
					diff_means_i_j[k] = Z_observations_mean.get_unsafe(j,k) - Y_predictions_mean.get_unsafe(i,k);

				double d2, ml;
				//mrpt::math::productIntegralAndMahalanobisTwoGaussians(diff_means_i_j,pred_i_cov,obs_j_cov, d2,ml);
				mrpt::math::mahalanobisDistance2AndLogPDF(diff_means_i_j,pred_i_cov, d2,ml);

				// The distance according to the metric
				double val =  (metric==metricMaha) ? d2 : ml;

				results.indiv_distances(i,j) = val;

				// Individual compatibility
				const bool IC =  (compatibilityTestMetric==metricML) ? (ml > log_ML_compat_test_threshold) : (d2 < chi2thres);
				results.indiv_compatibility(i,j)=IC;
				if (IC)
					results.indiv_compatibility_counts[j]++;
			}
		}
		else
		{
			// Use a kd-tree and compute only the N closest ones:
			for (size_t k=0;k<length_O;k++)
				kd_queryPoint[k] = Z_observations_mean.get_unsafe(j,k);

			kd_tree->query(&kd_queryPoint[0], N_KD_RESULTS, &kd_result_indices[0], &kd_result_distances[0] );

			// Only compute the distances for these ones:
			for (size_t w=0;w<N_KD_RESULTS;w++)
			{
				const size_t i = kd_result_indices[w];  // This is the index of the prediction in "predictions_mean"

				// Build the PDF of the prediction:
				const size_t pred_cov_idx = i*length_O;  // Extract the submatrix from the diagonal:
				Y_predictions_cov.extractMatrix(pred_cov_idx,pred_cov_idx,length_O,length_O, pred_i_cov);

				for (size_t k=0;k<length_O;k++)
					diff_means_i_j[k] = Z_observations_mean.get_unsafe(j,k) - Y_predictions_mean.get_unsafe(i,k);

				double d2, ml;
//				mrpt::math::productIntegralAndMahalanobisTwoGaussians(diff_means_i_j,pred_i_cov,obs_j_cov, d2,ml);
				mrpt::math::mahalanobisDistance2AndLogPDF(diff_means_i_j,pred_i_cov, d2,ml);

				if (d2>6*chi2thres) break; // Since kd-tree returns the landmarks by distance order, we can skip the rest

				// The distance according to the metric
				double val =  (metric==metricMaha) ? d2 : ml;

				results.indiv_distances(i,j) = val;

				// Individual compatibility
				const bool IC =  (compatibilityTestMetric==metricML) ? (ml > log_ML_compat_test_threshold) : (d2 < chi2thres);
				results.indiv_compatibility(i,j) = IC;
				if (IC)
					results.indiv_compatibility_counts[j]++;
			}
		} // end use KD-Tree
	} // end for

#if 0
	cout << "Distances: " << endl << results.indiv_distances << endl;
	//cout << "indiv compat: " << endl << results.indiv_compatibility << endl;
#endif

	// Do associations:
	results.associations.clear();

	switch(method)
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
			typedef multimap<double, pair<observation_index_t, multimap<double,prediction_index_t> > >  TListAllICs;
			TListAllICs lst_all_ICs;

			for (observation_index_t j=0;j<nObservations;++j)
			{
				multimap<double,prediction_index_t> ICs;

				for (prediction_index_t i=0;i<nPredictions;++i)
				{
					if (results.indiv_compatibility.get_unsafe(i,j))
					{
						double d2 = results.indiv_distances.get_unsafe(i,j);
						if (metric==metricML) d2=-d2;
						ICs.insert(make_pair(d2,i));
					}
				}

				if (!ICs.empty())
				{
					const double best_dist = ICs.begin()->first;
					lst_all_ICs.insert(make_pair( best_dist, make_pair(j,ICs) ) );
				}
			}

			// 2) With that lists, start by the best one and make the assignment.
			//    Remove the prediction from the list of available, and go on.
			// --------------------------------------------------------------------
			std::set<prediction_index_t> lst_already_taken_preds;

			for (TListAllICs::const_iterator it=lst_all_ICs.begin();it!=lst_all_ICs.end();++it)
			{
				const observation_index_t  obs_id = it->second.first;
				const multimap<double,prediction_index_t> &lstCompats = it->second.second;

				for (multimap<double,prediction_index_t>::const_iterator itP = lstCompats.begin(); itP!=lstCompats.end();++itP)
				{
					if (lst_already_taken_preds.find(itP->second) ==lst_already_taken_preds.end() )
					{
						// It's free: make the association:
						results.associations[obs_id] = itP->second;
						lst_already_taken_preds.insert(itP->second);
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
			TAuxDataRecursiveJCBB	info;
			info.nPredictions	= nPredictions;
			info.nObservations	= nObservations;
			info.length_O		= length_O;

			if (metric==metricMaha)
				JCBB_recursive<CMatrixDouble::Scalar,metricMaha>(Z_observations_mean,  Y_predictions_mean, Y_predictions_cov,results, info, 0 );
			else
				JCBB_recursive<CMatrixDouble::Scalar,metricML>(Z_observations_mean,  Y_predictions_mean, Y_predictions_cov,results, info, 0 );
		}
		break;

	default:
		THROW_EXCEPTION("Unknown value of 'method'")
	};

	// If a mapping of prediction indices to IDs was providen, apply it now:
	// ------------------------------------------------------------------------
	if (!predictions_IDs.empty())
	{
		ASSERT_( predictions_IDs.size()==nPredictions );
		for (std::map<size_t,size_t>::iterator itAssoc = results.associations.begin();itAssoc!=results.associations.end();++itAssoc)
			itAssoc->second = predictions_IDs[itAssoc->second];
	}

	MRPT_END
}



/* ==================================================================================================
					data_association_independent_predictions
   ================================================================================================== */
void mrpt::slam::data_association_independent_predictions(
	const mrpt::math::CMatrixDouble		&Z_observations_mean,
	const mrpt::math::CMatrixDouble		&Y_predictions_mean,
	const mrpt::math::CMatrixDouble		&Y_predictions_cov_stacked,
	TDataAssociationResults				&results,
	const TDataAssociationMethod		method,
	const TDataAssociationMetric		metric,
	const double						chi2quantile,
	const bool							DAT_ASOC_USE_KDTREE,
	const std::vector<prediction_index_t>		&predictions_IDs,
	const TDataAssociationMetric		compatibilityTestMetric,
	const double						log_ML_compat_test_threshold
	)
{
	MRPT_START

	results.clear();

	const size_t nPredictions  = size(Y_predictions_mean,1);
	const size_t nObservations = size(Z_observations_mean,1);

	const size_t length_O = size(Z_observations_mean,2);

	ASSERT_( nPredictions!=0 )
	ASSERT_( nObservations!=0 )
	ASSERT_( length_O == size(Y_predictions_mean,2))
	ASSERT_( length_O*nPredictions == size(Y_predictions_cov_stacked,1))

	ASSERT_( chi2quantile>0 && chi2quantile<1 )
	ASSERT_( metric==metricMaha || metric==metricML )

	//const double chi2thres = mrpt::math::chi2inv( chi2quantile, length_O );

	// TODO: Optimized version!!
	CMatrixDouble Y_predictions_cov_full(length_O*nPredictions,length_O*nPredictions);
	CMatrixDouble COV_i(length_O,length_O);
	for (size_t i=0;i<nPredictions;i++)
	{
		const size_t idx = i*length_O;
		Y_predictions_cov_stacked.extractSubmatrix(idx,idx+length_O-1,0,length_O-1, COV_i);
		Y_predictions_cov_full.insertMatrix(idx,idx,COV_i);
	}

	data_association_full_covariance(
		Z_observations_mean,
		Y_predictions_mean,Y_predictions_cov_full,
		results, method, metric, chi2quantile,
		DAT_ASOC_USE_KDTREE, predictions_IDs,
		compatibilityTestMetric, log_ML_compat_test_threshold );

	MRPT_END
}
