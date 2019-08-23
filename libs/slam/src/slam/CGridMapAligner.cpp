/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "slam-precomp.h"  // Precompiled headers

#include <mrpt/img/CEnhancedMetaFile.h>
#include <mrpt/maps/CLandmarksMap.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/distributions.h>
#include <mrpt/math/geometry.h>
#include <mrpt/math/ops_containers.h>
#include <mrpt/poses/CPoint2DPDFGaussian.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/random.h>
#include <mrpt/slam/CGridMapAligner.h>
#include <mrpt/slam/CICP.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/tfest/se2.h>
#include <Eigen/Dense>

using namespace mrpt::math;
using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::poses;
using namespace mrpt::random;
using namespace mrpt::img;
using namespace mrpt::system;
using namespace mrpt::vision;
using namespace std;

CPosePDF::Ptr CGridMapAligner::AlignPDF(
	const mrpt::maps::CMetricMap* mm1, const mrpt::maps::CMetricMap* mm2,
	const CPosePDFGaussian& initialEstimationPDF, float* runningTime,
	void* info)
{
	MRPT_START

	switch (options.methodSelection)
	{
		case CGridMapAligner::amCorrelation:
			return AlignPDF_correlation(
				mm1, mm2, initialEstimationPDF, runningTime, info);

		case CGridMapAligner::amModifiedRANSAC:
		case CGridMapAligner::amRobustMatch:
			// The same function has an internal switch for the specific method:
			return AlignPDF_robustMatch(
				mm1, mm2, initialEstimationPDF, runningTime, info);

		default:
			THROW_EXCEPTION("Wrong value found in 'options.methodSelection'!!");
	}

	MRPT_END
}

bool myVectorOrder(const pair<size_t, float>& o1, const pair<size_t, float>& o2)
{
	return o1.second < o2.second;
}

/*---------------------------------------------------------------
					AlignPDF_robustMatch
---------------------------------------------------------------*/
CPosePDF::Ptr CGridMapAligner::AlignPDF_robustMatch(
	const mrpt::maps::CMetricMap* mm1, const mrpt::maps::CMetricMap* mm2,
	const CPosePDFGaussian& initialEstimationPDF, float* runningTime,
	void* info)
{
	MRPT_START

	ASSERT_(
		options.methodSelection == CGridMapAligner::amRobustMatch ||
		options.methodSelection == CGridMapAligner::amModifiedRANSAC);

	TReturnInfo outInfo;
	mrpt::tfest::TMatchingPairList& correspondences =
		outInfo.correspondences;  // Use directly this placeholder to save 1
	// variable & 1 copy.
	mrpt::tfest::TMatchingPairList largestConsensusCorrs;

	CTicTac* tictac = nullptr;

	CPose2D grossEst = initialEstimationPDF.mean;

	CLandmarksMap::Ptr lm1(new CLandmarksMap());
	CLandmarksMap::Ptr lm2(new CLandmarksMap());

	std::vector<size_t> idxs1, idxs2;
	std::vector<size_t>::iterator it1, it2;

	// Asserts:
	// -----------------
	const CMultiMetricMap* multimap1 = nullptr;
	const CMultiMetricMap* multimap2 = nullptr;
	const COccupancyGridMap2D* m1 = nullptr;
	const COccupancyGridMap2D* m2 = nullptr;

	if (IS_CLASS(*mm1, CMultiMetricMap) && IS_CLASS(*mm2, CMultiMetricMap))
	{
		multimap1 = dynamic_cast<const CMultiMetricMap*>(mm1);
		multimap2 = dynamic_cast<const CMultiMetricMap*>(mm2);

		ASSERT_(multimap1->countMapsByClass<COccupancyGridMap2D>());
		ASSERT_(multimap2->countMapsByClass<COccupancyGridMap2D>());

		m1 = multimap1->mapByClass<COccupancyGridMap2D>().get();
		m2 = multimap2->mapByClass<COccupancyGridMap2D>().get();
	}
	else if (
		IS_CLASS(*mm1, COccupancyGridMap2D) &&
		IS_CLASS(*mm2, COccupancyGridMap2D))
	{
		m1 = dynamic_cast<const COccupancyGridMap2D*>(mm1);
		m2 = dynamic_cast<const COccupancyGridMap2D*>(mm2);
	}
	else
		THROW_EXCEPTION(
			"Metric maps must be of classes COccupancyGridMap2D or "
			"CMultiMetricMap");

	ASSERT_(m1);
	ASSERT_(m2);

	ASSERTMSG_(
		m1->getResolution() == m2->getResolution(),
		mrpt::format(
			"Different resolutions for m1, m2:\n"
			"\tres(m1) = %f\n\tres(m2) = %f\n",
			m1->getResolution(), m2->getResolution()));

	// The algorithm output auxiliar info:
	// -------------------------------------------------
	outInfo.goodness = 1.0f;

	outInfo.landmarks_map1 = lm1;
	outInfo.landmarks_map2 = lm2;

	// The PDF to estimate:
	// ------------------------------------------------------
	CPosePDFSOG::Ptr pdf_SOG = std::make_shared<CPosePDFSOG>();

	// Extract features from grid-maps:
	// ------------------------------------------------------
	const size_t N1 =
		std::max(40, mrpt::round(m1->getArea() * options.featsPerSquareMeter));
	const size_t N2 =
		std::max(40, mrpt::round(m2->getArea() * options.featsPerSquareMeter));

	m_grid_feat_extr.extractFeatures(
		*m1, *lm1, N1, options.feature_descriptor,
		options.feature_detector_options);
	m_grid_feat_extr.extractFeatures(
		*m2, *lm2, N2, options.feature_descriptor,
		options.feature_detector_options);

	if (runningTime)
	{
		tictac = new CTicTac();
		tictac->Tic();
	}

	const size_t nLM1 = lm1->size();
	const size_t nLM2 = lm2->size();

	//  At least two landmarks at each map!
	// ------------------------------------------------------
	if (nLM1 < 2 || nLM2 < 2)
	{
		outInfo.goodness = 0;
	}
	else
	{
		//#define METHOD_FFT
		//#define DEBUG_SHOW_CORRELATIONS

		// Compute correlation between landmarks:
		// ---------------------------------------------
		CMatrixFloat CORR(lm1->size(), lm2->size()), auxCorr;
		CImage im1, im2;  // Grayscale
		CVectorFloat corr;
		unsigned int corrsCount = 0;
		std::vector<bool> hasCorr(nLM1, false);

		const double thres_max = options.threshold_max;
		const double thres_delta = options.threshold_delta;

		// CDisplayWindowPlots::Ptr	auxWin;
		if (options.debug_show_corrs)
		{
			// auxWin = CDisplayWindowPlots::Ptr( new
			// CDisplayWindowPlots("Individual corr.") );
			std::cerr << "Warning: options.debug_show_corrs has no effect "
						 "since MRPT 0.9.1\n";
		}

		for (size_t idx1 = 0; idx1 < nLM1; idx1++)
		{
			// CVectorFloat  	corrs_indiv;
			vector<pair<size_t, float>> corrs_indiv;  // (index, distance);
			// Index is used to
			// recover the original
			// index after sorting.
			vector<float> corrs_indiv_only;
			corrs_indiv.reserve(nLM2);
			corrs_indiv_only.reserve(nLM2);

			for (size_t idx2 = 0; idx2 < nLM2; idx2++)
			{
				float minDist;
				minDist =
					lm1->landmarks.get(idx1)->features[0].descriptorDistanceTo(
						lm2->landmarks.get(idx2)->features[0]);

				corrs_indiv.emplace_back(idx2, minDist);
				corrs_indiv_only.push_back(minDist);
			}  // end for idx2

			// double corr_mean,corr_std;
			// mrpt::math::meanAndStd(corrs_indiv_only,corr_mean,corr_std);
			const double corr_best = mrpt::math::minimum(corrs_indiv_only);
			// cout << "M: " << corr_mean << " std: " << corr_std << " best: "
			// << corr_best << endl;

			// Sort the list and keep the N best features:
			std::sort(corrs_indiv.begin(), corrs_indiv.end(), myVectorOrder);

			// const size_t nBestToKeep = std::min( (size_t)30,
			// corrs_indiv.size() );
			const size_t nBestToKeep = corrs_indiv.size();

			for (size_t w = 0; w < nBestToKeep; w++)
			{
				if (corrs_indiv[w].second <= thres_max &&
					corrs_indiv[w].second <= (corr_best + thres_delta))
				{
					idxs1.push_back(idx1);
					idxs2.push_back(corrs_indiv[w].first);
					outInfo.correspondences_dists_maha.emplace_back(
						idx1, corrs_indiv[w].first, corrs_indiv[w].second);
				}
			}
		}  // end for idx1

		// Save an image with each feature and its matches:
		if (options.save_feat_coors)
		{
			mrpt::system::deleteFilesInDirectory("grid_feats");
			mrpt::system::createDirectory("grid_feats");

			std::map<size_t, std::set<size_t>> corrs_by_idx;
			for (size_t l = 0; l < idxs1.size(); l++)
				corrs_by_idx[idxs1[l]].insert(idxs2[l]);

			for (auto& it : corrs_by_idx)
			{
				CMatrixFloat descriptor1;
				lm1->landmarks.get(it.first)
					->features[0]
					.getFirstDescriptorAsMatrix(descriptor1);

				im1.setFromMatrix(descriptor1, true /*normalized*/);

				const size_t FEAT_W = im1.getWidth();
				const size_t FEAT_H = im1.getHeight();
				const size_t nF = it.second.size();

				CImage img_compose(FEAT_W * 2 + 15, 10 + (5 + FEAT_H) * nF);
				img_compose.filledRectangle(
					0, 0, img_compose.getWidth() - 1,
					img_compose.getHeight() - 1, TColor::black());

				img_compose.drawImage(5, 5, im1);

				size_t j;
				std::set<size_t>::iterator it_j;
				string fil =
					format("grid_feats/_FEAT_MATCH_%03i", (int)it.first);

				for (j = 0, it_j = it.second.begin(); j < nF; ++j, ++it_j)
				{
					fil += format("_%u", static_cast<unsigned int>(*it_j));

					CMatrixFloat descriptor2;
					lm2->landmarks.get(*it_j)
						->features[0]
						.getFirstDescriptorAsMatrix(descriptor2);
					im2.setFromMatrix(descriptor2, true);
					img_compose.drawImage(
						10 + FEAT_W, 5 + j * (FEAT_H + 5), im2);
				}
				fil += ".png";
				img_compose.saveToFile(fil);
			}  // end for map
		}

		// ------------------------------------------------------------------
		// Create the list of correspondences from the lists: idxs1 & idxs2
		// ------------------------------------------------------------------
		correspondences.clear();
		for (it1 = idxs1.begin(), it2 = idxs2.begin(); it1 != idxs1.end();
			 ++it1, ++it2)
		{
			mrpt::tfest::TMatchingPair mp;
			mp.this_idx = *it1;
			mp.this_x = lm1->landmarks.get(*it1)->pose_mean.x;
			mp.this_y = lm1->landmarks.get(*it1)->pose_mean.y;
			mp.this_z = lm1->landmarks.get(*it1)->pose_mean.z;

			mp.other_idx = *it2;
			mp.other_x = lm2->landmarks.get(*it2)->pose_mean.x;
			mp.other_y = lm2->landmarks.get(*it2)->pose_mean.y;
			mp.other_z = lm2->landmarks.get(*it2)->pose_mean.z;
			correspondences.push_back(mp);

			if (!hasCorr[*it1])
			{
				hasCorr[*it1] = true;
				corrsCount++;
			}
		}  // end for corres.

		outInfo.goodness = (2.0f * corrsCount) / (nLM1 + nLM2);

		// Compute the estimation using ALL the correspondences (NO ROBUST):
		// ----------------------------------------------------------------------
		mrpt::math::TPose2D noRobustEst;
		if (!mrpt::tfest::se2_l2(correspondences, noRobustEst))
		{
			// There's no way to match the maps! e.g. no correspondences
			outInfo.goodness = 0;
			pdf_SOG->clear();
			outInfo.sog1 = pdf_SOG;
			outInfo.sog2 = pdf_SOG;
			outInfo.sog3 = pdf_SOG;
		}
		else
		{
			outInfo.noRobustEstimation = mrpt::poses::CPose2D(noRobustEst);
			MRPT_LOG_INFO(mrpt::format(
				"[CGridMapAligner] Overall estimation(%u corrs, total: "
				"%u): (%.03f,%.03f,%.03fdeg)\n",
				corrsCount, (unsigned)correspondences.size(),
				outInfo.noRobustEstimation.x(), outInfo.noRobustEstimation.y(),
				RAD2DEG(outInfo.noRobustEstimation.phi())));

			// The list of SOG modes & their corresponding sub-sets of
			// matchings:
			using TMapMatchingsToPoseMode = std::map<
				mrpt::tfest::TMatchingPairList, CPosePDFSOG::TGaussianMode>;
			TMapMatchingsToPoseMode sog_modes;

			// ---------------------------------------------------------------
			// Now, we have to choose between the methods:
			//  - CGridMapAligner::amRobustMatch  ("normal" RANSAC)
			//  - CGridMapAligner::amModifiedRANSAC
			// ---------------------------------------------------------------
			if (options.methodSelection == CGridMapAligner::amRobustMatch)
			{
				// ====================================================
				//             METHOD: "Normal" RANSAC
				// ====================================================

				// RANSAC over the found correspondences:
				// -------------------------------------------------
				const unsigned int min_inliers =
					options.ransac_minSetSizeRatio * (nLM1 + nLM2) / 2;

				mrpt::tfest::TSE2RobustParams tfest_params;
				tfest_params.ransac_minSetSize = min_inliers;
				tfest_params.ransac_maxSetSize = nLM1 + nLM2;
				tfest_params.ransac_mahalanobisDistanceThreshold =
					options.ransac_mahalanobisDistanceThreshold;
				tfest_params.ransac_nSimulations = 0;  // 0=auto
				tfest_params.ransac_fuseByCorrsMatch = true;
				tfest_params.ransac_fuseMaxDiffXY = 0.01;
				tfest_params.ransac_fuseMaxDiffPhi = DEG2RAD(0.1);
				tfest_params.ransac_algorithmForLandmarks = true;
				tfest_params.probability_find_good_model =
					options.ransac_prob_good_inliers;
				tfest_params.verbose = false;

				mrpt::tfest::TSE2RobustResult tfest_result;
				mrpt::tfest::se2_l2_robust(
					correspondences, options.ransac_SOG_sigma_m, tfest_params,
					tfest_result);

				ASSERT_(pdf_SOG);
				*pdf_SOG = tfest_result.transformation;
				largestConsensusCorrs = tfest_result.largestSubSet;

				// Simplify the SOG by merging close modes:
				// -------------------------------------------------
				size_t nB = pdf_SOG->size();
				outInfo.sog1 = pdf_SOG;

				// Keep only the most-likely Gaussian mode:
				CPosePDFSOG::TGaussianMode best_mode;
				best_mode.log_w = -std::numeric_limits<double>::max();

				for (size_t n = 0; n < pdf_SOG->size(); n++)
				{
					const CPosePDFSOG::TGaussianMode& m = (*pdf_SOG)[n];

					if (m.log_w > best_mode.log_w) best_mode = m;
				}

				pdf_SOG->clear();
				pdf_SOG->push_back(best_mode);
				outInfo.sog2 = pdf_SOG;

				MRPT_LOG_INFO_STREAM(
					"[CGridMapAligner] amRobustMatch: "
					<< nB << " SOG modes reduced to " << pdf_SOG->size()
					<< " (most-likely) (min.inliers=" << min_inliers << ")\n");

			}  // end of amRobustMatch
			else
			{
				// ====================================================
				//             METHOD: "Modified" RANSAC
				// ====================================================
				mrpt::tfest::TMatchingPairList all_corrs = correspondences;

				const size_t nCorrs = all_corrs.size();
				ASSERT_(nCorrs >= 2);

				pdf_SOG->clear();  // Start with 0 Gaussian modes

				// Build a points-map for exploiting its KD-tree:
				// ----------------------------------------------------
				CSimplePointsMap lm1_pnts, lm2_pnts;
				lm1_pnts.reserve(nLM1);
				for (size_t i = 0; i < nLM1; i++)
					lm1_pnts.insertPoint(lm1->landmarks.get(i)->pose_mean);
				lm2_pnts.reserve(nLM2);
				for (size_t i = 0; i < nLM2; i++)
					lm2_pnts.insertPoint(lm2->landmarks.get(i)->pose_mean);

				// RANSAC loop
				// ---------------------
				const size_t minInliersTOaccept =
					round(options.ransac_minSetSizeRatio * 0.5 * (nLM1 + nLM2));
				// Set an initial # of iterations:
				const unsigned int ransac_min_nSimulations =
					2 * (nLM1 + nLM2);  // 1000;
				unsigned int ransac_nSimulations =
					10;  // It doesn't matter actually, since will be changed in
				// the first loop
				const double probability_find_good_model = 0.9999;

				const double chi2_thres_dim1 =
					mrpt::math::chi2inv(options.ransac_chi2_quantile, 1);
				const double chi2_thres_dim2 =
					mrpt::math::chi2inv(options.ransac_chi2_quantile, 2);

				// Generic 2x2 covariance matrix for all features in their local
				// coords:
				CMatrixDouble22 COV_pnt;
				COV_pnt(0, 0) = COV_pnt(1, 1) =
					square(options.ransac_SOG_sigma_m);

				// The absolute number of trials.
				// in practice it's only important for a reduced number of
				// correspondences, to avoid a dead-lock:
				//  It's the binomial coefficient:
				//   / n \      n!          n * (n-1)
				//   |   | = ----------- = -----------
				//   \ 2 /    2! (n-2)!         2
				//
				const unsigned int max_trials =
					(nCorrs * (nCorrs - 1) / 2) *
					5;  // "*5" is just for safety...

				unsigned int iter = 0;  // Valid iterations (those passing the
				// first mahalanobis test)
				unsigned int trials = 0;  // counter of all iterations,
				// including "iter" + failing ones.
				while (iter < ransac_nSimulations &&
					   trials <
						   max_trials)  // ransac_nSimulations can be dynamic
				{
					trials++;

					mrpt::tfest::TMatchingPairList tentativeSubSet;

					// Pick 2 random correspondences:
					uint32_t idx1, idx2;
					idx1 = getRandomGenerator().drawUniform32bit() % nCorrs;
					do
					{
						idx2 = getRandomGenerator().drawUniform32bit() % nCorrs;
					} while (idx1 == idx2);  // Avoid a degenerated case!

					// Uniqueness of features:
					if (all_corrs[idx1].this_idx == all_corrs[idx2].this_idx ||
						all_corrs[idx1].this_idx == all_corrs[idx2].other_idx)
						continue;
					if (all_corrs[idx1].other_idx == all_corrs[idx2].this_idx ||
						all_corrs[idx1].other_idx == all_corrs[idx2].other_idx)
						continue;

					// Check the feasibility of this pair "idx1"-"idx2":
					//  The distance between the pair of points in MAP1 must be
					//  very close
					//   to that of their correspondences in MAP2:
					const double corrs_dist1 =
						mrpt::math::distanceBetweenPoints(
							all_corrs[idx1].this_x, all_corrs[idx1].this_y,
							all_corrs[idx1].this_z, all_corrs[idx2].this_x,
							all_corrs[idx2].this_y, all_corrs[idx2].this_z);

					const double corrs_dist2 =
						mrpt::math::distanceBetweenPoints(
							all_corrs[idx1].other_x, all_corrs[idx1].other_y,
							all_corrs[idx1].other_z, all_corrs[idx2].other_x,
							all_corrs[idx2].other_y, all_corrs[idx2].other_z);

					// Is is a consistent possibility?
					//  We use a chi2 test (see paper for the derivation)
					const double corrs_dist_chi2 =
						square(square(corrs_dist1) - square(corrs_dist2)) /
						(8.0 * square(options.ransac_SOG_sigma_m) *
						 (square(corrs_dist1) + square(corrs_dist2)));

					if (corrs_dist_chi2 > chi2_thres_dim1) continue;  // Nope

					iter++;  // Do not count iterations if they fail the test
					// above.

					// before proceeding with this hypothesis, is it an old one?
					bool is_new_hyp = true;
					for (auto& sog_mode : sog_modes)
					{
						if (sog_mode.first.contains(all_corrs[idx1]) &&
							sog_mode.first.contains(all_corrs[idx2]))
						{
							// Increment weight:
							sog_mode.second.log_w =
								std::log(std::exp(sog_mode.second.log_w) + 1.0);
							is_new_hyp = false;
							break;
						}
					}
					if (!is_new_hyp) continue;

					// Ok, it's a new hypothesis:
					tentativeSubSet.push_back(all_corrs[idx1]);
					tentativeSubSet.push_back(all_corrs[idx2]);

					// Maintain a list of already used landmarks IDs in both
					// maps to avoid repetitions:
					std::vector<bool> used_landmarks1(nLM1, false);
					std::vector<bool> used_landmarks2(nLM2, false);

					used_landmarks1[all_corrs[idx1].this_idx] = true;
					used_landmarks1[all_corrs[idx2].this_idx] = true;
					used_landmarks2[all_corrs[idx1].other_idx] = true;
					used_landmarks2[all_corrs[idx2].other_idx] = true;

					// Build the transformation for these temptative
					// correspondences:
					bool keep_incorporating = true;
					CPosePDFGaussian temptPose;
					do  // Incremently incorporate inliers:
					{
						if (!mrpt::tfest::se2_l2(tentativeSubSet, temptPose))
							continue;  // Invalid matching...

						// The computed cov is "normalized", i.e. must be
						// multiplied by std^2_xy
						temptPose.cov *= square(options.ransac_SOG_sigma_m);

						// cout << "q: " << temptPose << endl;

						// Find the landmark in MAP2 with the best (maximum)
						// product-integral:
						//   (i^* , j^*) = arg max_(i,j) \int p_i()p_j()
						//----------------------------------------------------------------------
						const double ccos = cos(temptPose.mean.phi());
						const double ssin = sin(temptPose.mean.phi());

						CMatrixDouble22 Hc;  // Jacobian wrt point_j
						Hc(1, 1) = ccos;
						Hc(0, 0) = ccos;
						Hc(1, 0) = ssin;
						Hc(0, 1) = -ssin;

						CMatrixFixed<double, 2, 3>
							Hq;  // Jacobian wrt transformation q
						Hq(0, 0) = 1;
						Hq(1, 1) = 1;

						TPoint2D p2_j_local;
						vector<float> matches_dist;
						std::vector<size_t> matches_idx;

						CPoint2DPDFGaussian pdf_M2_j;
						CPoint2DPDFGaussian pdf_M1_i;

// Use integral-of-product vs. mahalanobis distances to match:
#define GRIDMAP_USE_PROD_INTEGRAL

#ifdef GRIDMAP_USE_PROD_INTEGRAL
						double best_pair_value = 0;
#else
						double best_pair_value =
							std::numeric_limits<double>::max();
#endif
						double best_pair_d2 =
							std::numeric_limits<double>::max();
						pair<size_t, size_t> best_pair_ij;

						//#define SHOW_CORRS

#ifdef SHOW_CORRS
						CDisplayWindowPlots win("Matches");
#endif
						for (size_t j = 0; j < nLM2; j++)
						{
							if (used_landmarks2[j]) continue;

							lm2_pnts.getPoint(
								j, p2_j_local);  // In local coords.
							pdf_M2_j.mean = mrpt::poses::CPoint2D(
								temptPose.mean +
								p2_j_local);  // In (temptative) global coords:
							pdf_M2_j.cov(0, 0) = pdf_M2_j.cov(1, 1) =
								square(options.ransac_SOG_sigma_m);

#ifdef SHOW_CORRS
							win.plotEllipse(
								pdf_M2_j.mean.x(), pdf_M2_j.mean.y(),
								pdf_M2_j.cov, 2, "b-",
								format("M2_%u", (unsigned)j), true);
#endif

							static const unsigned int N_KDTREE_SEARCHED = 3;

							// Look for a few close features which may be
							// potential matches:
							lm1_pnts.kdTreeNClosestPoint2DIdx(
								pdf_M2_j.mean.x(), pdf_M2_j.mean.y(),
								N_KDTREE_SEARCHED, matches_idx, matches_dist);

							// And for each one, compute the product-integral:
							for (unsigned long u : matches_idx)
							{
								if (used_landmarks1[u]) continue;

								// Jacobian wrt transformation q
								Hq(0, 2) =
									-p2_j_local.x * ssin - p2_j_local.y * ccos;
								Hq(1, 2) =
									p2_j_local.x * ccos - p2_j_local.y * ssin;

								// COV_j = Hq \Sigma_q Hq^t + Hc Cj Hc^t
								pdf_M1_i.cov =
									mrpt::math::multiply_HCHt(Hc, COV_pnt);
								pdf_M1_i.cov += mrpt::math::multiply_HCHt(
									Hq, temptPose.cov);

								float px, py;
								lm1_pnts.getPoint(u, px, py);
								pdf_M1_i.mean.x(px);
								pdf_M1_i.mean.y(py);

#ifdef SHOW_CORRS
								win.plotEllipse(
									pdf_M1_i.mean.x(), pdf_M1_i.mean.y(),
									pdf_M1_i.cov, 2, "r-",
									format("M1_%u", (unsigned)matches_idx[u]),
									true);
#endif

// And now compute the product integral:
#ifdef GRIDMAP_USE_PROD_INTEGRAL
								const double prod_ij =
									pdf_M1_i.productIntegralWith(pdf_M2_j);
								// const double prod_ij_d2 = square(
								// pdf_M1_i.mahalanobisDistanceTo(pdf_M2_j) );

								if (prod_ij > best_pair_value)
#else
								const double prod_ij =
									pdf_M1_i.mean.distanceTo(pdf_M2_j.mean);
								if (prod_ij < best_pair_value)
#endif
								{
									// const double prodint_ij =
									// pdf_M1_i.productIntegralWith2D(pdf_M2_j);

									best_pair_value = prod_ij;
									best_pair_ij.first = u;
									best_pair_ij.second = j;

									best_pair_d2 =
										square(pdf_M1_i.mahalanobisDistanceTo(
											pdf_M2_j));

									// cout << "P1: " << pdf_M1_i.mean << " C= "
									// << pdf_M1_i.cov.inMatlabFormat() << endl;
									// cout << "P2: " << pdf_M2_j.mean << " C= "
									// << pdf_M2_j.cov.inMatlabFormat() << endl;
									// cout << "  -> " << format("%e",prod_ij)
									// << " d2: " << best_pair_d2 << endl <<
									// endl;
								}
							}  // end for u (closest matches of LM2 in MAP 1)

#ifdef SHOW_CORRS
							win.axis_fit(true);
							win.waitForKey();
							win.clear();
#endif

						}  // end for each LM2

						// Stop when the best choice has a bad mahal. dist.
						keep_incorporating = false;

						// For the best (i,j), gate by the mahalanobis distance:
						if (best_pair_d2 < chi2_thres_dim2)
						{
							// AND, also, check if the descriptors have some
							// resemblance!!
							// ----------------------------------------------------------------
							// double feat_dist =
							// lm1->landmarks.get(best_pair_ij.first)->features[0]->descriptorDistanceTo(*lm1->landmarks.get(best_pair_ij.second)->features[0]);
							// if (feat_dist< options.threshold_max)
							{
								float p1_i_localx, p1_i_localy;
								float p2_j_localx, p2_j_localy;
								lm1_pnts.getPoint(
									best_pair_ij.first, p1_i_localx,
									p1_i_localy);
								lm2_pnts.getPoint(
									best_pair_ij.second, p2_j_localx,
									p2_j_localy);  // In local coords.

								used_landmarks1[best_pair_ij.first] = true;
								used_landmarks2[best_pair_ij.second] = true;

								tentativeSubSet.push_back(
									mrpt::tfest::TMatchingPair(
										best_pair_ij.first, best_pair_ij.second,
										p1_i_localx, p1_i_localy, 0,  // MAP1
										p2_j_localx, p2_j_localy, 0  // MAP2
										));

								keep_incorporating = true;
							}
						}

					} while (keep_incorporating);

					// Consider this pairing?
					const size_t ninliers = tentativeSubSet.size();
					if (ninliers > minInliersTOaccept)
					{
						CPosePDFSOG::TGaussianMode newGauss;
						newGauss.log_w = 0;  // log(1);  //
						// std::log(static_cast<double>(nCoincidences));
						newGauss.mean = temptPose.mean;
						newGauss.cov = temptPose.cov;

						sog_modes[tentativeSubSet] = newGauss;

						// cout << "ITER: " << iter << " #inliers: " << ninliers
						// << " q: " << temptPose.mean << endl;
					}

					// Keep the largest consensus & dynamic # of steps:
					if (largestConsensusCorrs.size() < ninliers)
					{
						largestConsensusCorrs = tentativeSubSet;

						// Update estimate of N, the number of trials to ensure
						// we pick,
						// with probability p, a data set with no outliers.
						const double fracinliers =
							ninliers /
							static_cast<double>(std::min(nLM1, nLM2));
						double pNoOutliers =
							1 - pow(fracinliers,
									static_cast<double>(
										2.0));  // minimumSizeSamplesToFit

						pNoOutliers = std::max(
							std::numeric_limits<double>::epsilon(),
							pNoOutliers);  // Avoid division by -Inf
						pNoOutliers = std::min(
							1.0 - std::numeric_limits<double>::epsilon(),
							pNoOutliers);  // Avoid division by 0.
						// Number of
						ransac_nSimulations =
							log(1 - probability_find_good_model) /
							log(pNoOutliers);

						if (ransac_nSimulations < ransac_min_nSimulations)
							ransac_nSimulations = ransac_min_nSimulations;

						// if (verbose)
						//	cout << "[Align] Iter #" << iter << " Est. #iters: "
						//<< ransac_nSimulations << "  pNoOutliers=" <<
						// pNoOutliers << " #inliers: " << ninliers << endl;
					}

				}  // end of RANSAC loop

				// Move SOG modes into pdf_SOG:
				pdf_SOG->clear();
				for (auto s = sog_modes.begin(); s != sog_modes.end(); ++s)
				{
					cout << "SOG mode: " << s->second.mean
						 << " inliers: " << s->first.size() << endl;
					pdf_SOG->push_back(s->second);
				}

				// Normalize:
				if (pdf_SOG->size() > 0) pdf_SOG->normalizeWeights();

				// Simplify the SOG by merging close modes:
				// -------------------------------------------------
				size_t nB = pdf_SOG->size();
				outInfo.sog1 = pdf_SOG;

				CTicTac merge_clock;
				pdf_SOG->mergeModes(options.maxKLd_for_merge, false);
				const double merge_clock_tim = merge_clock.Tac();

				outInfo.sog2 = pdf_SOG;
				size_t nA = pdf_SOG->size();
				MRPT_LOG_INFO(mrpt::format(
					"[CGridMapAligner] amModifiedRANSAC: %u SOG modes "
					"merged to %u in %.03fsec\n",
					(unsigned int)nB, (unsigned int)nA, merge_clock_tim));

			}  // end of amModifiedRANSAC

			// Save best corrs:
			if (options.debug_save_map_pairs)
			{
				static unsigned int NN = 0;
				static const COccupancyGridMap2D* lastM1 = nullptr;
				if (lastM1 != m1)
				{
					lastM1 = m1;
					NN = 0;
				}
				printf(
					"   Largest consensus: %u\n",
					static_cast<unsigned>(largestConsensusCorrs.size()));
				CEnhancedMetaFile::LINUX_IMG_WIDTH(
					m1->getSizeX() + m2->getSizeX() + 50);
				CEnhancedMetaFile::LINUX_IMG_HEIGHT(
					max(m1->getSizeY(), m2->getSizeY()) + 50);

				for (auto s = sog_modes.begin(); s != sog_modes.end(); ++s)
				{
					COccupancyGridMap2D::saveAsEMFTwoMapsWithCorrespondences(
						format("__debug_corrsGrid_%05u.emf", NN), m1, m2,
						s->first);
					COccupancyGridMap2D::saveAsBitmapTwoMapsWithCorrespondences(
						format("__debug_corrsGrid_%05u.png", NN), m1, m2,
						s->first);
					++NN;
				}
			}

			// --------------------------------------------------------------------
			// At this point:
			//   - "pdf_SOG": has the resulting PDF with the SOG (from whatever
			//   method)
			//   - "largestConsensusCorrs": The 'best' set of correspondences
			//
			//  Now: If we had a multi-metric map, use the points map to improve
			//        the estimation with ICP.
			// --------------------------------------------------------------------
			if (multimap1 && multimap2 &&
				multimap1->countMapsByClass<CSimplePointsMap>() != 0 &&
				multimap2->countMapsByClass<CSimplePointsMap>() != 0)
			{
				auto pnts1 = multimap1->mapByClass<CSimplePointsMap>();
				auto pnts2 = multimap2->mapByClass<CSimplePointsMap>();

				CICP icp;
				CICP::TReturnInfo icpInfo;

				icp.options.maxIterations = 20;
				icp.options.smallestThresholdDist = 0.05f;
				icp.options.thresholdDist = 0.75f;

				// Invoke ICP once for each mode in the SOG:
				size_t cnt = 0;
				outInfo.icp_goodness_all_sog_modes.clear();
				for (auto i = pdf_SOG->begin(); i != pdf_SOG->end(); ++cnt)
				{
					CPosePDF::Ptr icp_est = icp.Align(
						pnts1.get(), pnts2.get(), (i)->mean, nullptr, &icpInfo);

					//(i)->cov(0,0) += square( 0.05 );
					//(i)->cov(1,1) += square( 0.05 );
					//(i)->cov(2,2) += square( DEG2RAD(0.05) );

					CPosePDFGaussian i_gauss(i->mean, i->cov);
					CPosePDFGaussian icp_gauss(icp_est->getMeanVal(), i->cov);

					const double icp_maha_dist =
						i_gauss.mahalanobisDistanceTo(icp_gauss);

					cout << "ICP " << cnt << " log-w: " << i->log_w
						 << " Goodness: " << 100 * icpInfo.goodness
						 << "  D_M= " << icp_maha_dist << endl;
					cout << "  final pos: " << icp_est->getMeanVal() << endl;
					cout << "    org pos: " << i->mean << endl;

					outInfo.icp_goodness_all_sog_modes.push_back(
						icpInfo.goodness);

					// Discard?
					if (icpInfo.goodness > options.min_ICP_goodness &&
						icp_maha_dist < options.max_ICP_mahadist)
					{
						icp_est->getMean((i)->mean);
						++i;
					}
					else
					{
						// Delete:
						i = pdf_SOG->erase(i);
					}

				}  // end for i

				// Merge:
				outInfo.sog3 = pdf_SOG;

				pdf_SOG->mergeModes(options.maxKLd_for_merge, false);
				MRPT_LOG_DEBUG_STREAM(
					"[CGridMapAligner] " << pdf_SOG->size()
										 << " SOG modes merged after ICP.");

			}  // end multimapX

		}  // end of, yes, we have enough correspondences

	}  // end of: yes, there are landmarks in the grid maps!

	// Copy the output info if requested:
	// -------------------------------------------------
	MRPT_TODO(
		"Refactor `info` so it is polymorphic and can use dynamic_cast<> here");
	if (info)
	{
		auto* info_ = static_cast<TReturnInfo*>(info);
		*info_ = outInfo;
	}

	if (runningTime)
	{
		*runningTime = tictac->Tac();
		delete tictac;
	}

	return pdf_SOG;

	MRPT_END
}

/*---------------------------------------------------------------
					AlignPDF_correlation
---------------------------------------------------------------*/
CPosePDF::Ptr CGridMapAligner::AlignPDF_correlation(
	const mrpt::maps::CMetricMap* mm1, const mrpt::maps::CMetricMap* mm2,
	const CPosePDFGaussian& initialEstimationPDF, float* runningTime,
	void* info)
{
	MRPT_UNUSED_PARAM(initialEstimationPDF);
	MRPT_UNUSED_PARAM(info);

	MRPT_START

	//#define	CORRELATION_SHOW_DEBUG

	CTicTac* tictac = nullptr;

	// Asserts:
	// -----------------
	ASSERT_(mm1->GetRuntimeClass() == CLASS_ID(COccupancyGridMap2D));
	ASSERT_(mm2->GetRuntimeClass() == CLASS_ID(COccupancyGridMap2D));
	const auto* m1 = dynamic_cast<const COccupancyGridMap2D*>(mm1);
	const auto* m2 = dynamic_cast<const COccupancyGridMap2D*>(mm2);

	ASSERT_(m1->getResolution() == m2->getResolution());

	if (runningTime)
	{
		tictac = new CTicTac();
		tictac->Tic();
	}

	// The PDF to estimate:
	// ------------------------------------------------------
	CPosePDFGaussian::Ptr PDF = std::make_shared<CPosePDFGaussian>();

	// Determine the extension to compute the correlation into:
	// ----------------------------------------------------------
	float phiResolution = DEG2RAD(0.2f);
	float phiMin = -M_PIf + 0.5f * phiResolution;
	float phiMax = M_PIf;

	// Compute the difference between maps for each (u,v) pair!
	// --------------------------------------------------------------
	float phi;
	float pivotPt_x = 0.5f * (m1->getXMax() + m1->getXMin());
	float pivotPt_y = 0.5f * (m1->getYMax() + m1->getYMin());
	COccupancyGridMap2D map2_mod;
	CImage map1_img, map2_img;
	float currentMaxCorr = -1e6f;

	m1->getAsImage(map1_img);

	map2_mod.setSize(
		m1->getXMin(), m1->getXMax(), m1->getYMin(), m1->getYMax(),
		m1->getResolution());
	size_t map2_lx = map2_mod.getSizeX();
	size_t map2_ly = map2_mod.getSizeY();
	CMatrixF outCrossCorr, bestCrossCorr;
	float map2width_2 = 0.5f * (map2_mod.getXMax() - map2_mod.getXMin());

#ifdef CORRELATION_SHOW_DEBUG
	CDisplayWindow* win = new CDisplayWindow("corr");
	CDisplayWindow* win2 = new CDisplayWindow("map2_mod");
#endif

	// --------------------------------------------------------
	// Use FFT-based correlation for each orientation:
	// --------------------------------------------------------
	for (phi = phiMin; phi < phiMax; phi += phiResolution)
	{
		// Create the displaced map2 grid, for the size of map1:
		// --------------------------------------------------------
		CPose2D v2(
			pivotPt_x - cos(phi) * map2width_2,
			pivotPt_y - sin(phi) * map2width_2,
			phi);  // Rotation point: the centre of img1:
		CPoint2D v1, v3;
		v2 = CPose2D(0, 0, 0) - v2;  // Inverse

		for (size_t cy2 = 0; cy2 < map2_ly; cy2++)
		{
			COccupancyGridMap2D::cellType* row = map2_mod.getRow(cy2);
			for (size_t cx2 = 0; cx2 < map2_lx; cx2++)
			{
				v3 = v2 + CPoint2D(map2_mod.idx2x(cx2), map2_mod.idx2y(cy2));
				*row++ = m2->p2l(m2->getPos(v3.x(), v3.y()));
			}
		}

		map2_mod.getAsImage(map2_img);
		//		map2_img.saveToBMP("__debug_map2mod.bmp");

		// Compute the correlation:
		map1_img.cross_correlation_FFT(
			map2_img, outCrossCorr, -1, -1, -1, -1,
			127,  // Bias to be substracted
			127  // Bias to be substracted
		);

		float corrPeak = outCrossCorr.maxCoeff();
		printf("phi = %fdeg \tmax corr=%f\n", RAD2DEG(phi), corrPeak);

		// Keep the maximum:
		if (corrPeak > currentMaxCorr)
		{
			currentMaxCorr = corrPeak;
			bestCrossCorr = outCrossCorr;
			PDF->mean.phi(phi);
		}

#ifdef CORRELATION_SHOW_DEBUG
		outCrossCorr.adjustRange(0, 1);
		CImage aux(outCrossCorr, true);
		win->showImage(aux);
		win2->showImage(map2_img);
		std::this_thread::sleep_for(5ms);
#endif

	}  // end for phi

	if (runningTime)
	{
		*runningTime = tictac->Tac();
		delete tictac;
	}

	CImage aux;
	aux.setFromMatrix(bestCrossCorr, false /* do normalization [0,1]*/);
	aux.saveToFile("_debug_best_corr.png");

#ifdef CORRELATION_SHOW_DEBUG
	delete win;
	delete win2;
#endif

	// Transform the best corr matrix peak into coordinates:
	std::size_t uMax, vMax;
	currentMaxCorr = bestCrossCorr.maxCoeff(uMax, vMax);

	PDF->mean.x(m1->idx2x(uMax));
	PDF->mean.y(m1->idx2y(vMax));

	return PDF;

	// Done!
	MRPT_END
}

/*---------------------------------------------------------------
					TConfigParams
  ---------------------------------------------------------------*/
CGridMapAligner::TConfigParams::TConfigParams() : feature_detector_options() {}
/*---------------------------------------------------------------
					dumpToTextStream
  ---------------------------------------------------------------*/
void CGridMapAligner::TConfigParams::dumpToTextStream(std::ostream& out) const
{
	out << mrpt::format(
		"\n----------- [CGridMapAligner::TConfigParams] ------------ \n\n");

	LOADABLEOPTS_DUMP_VAR(methodSelection, int)
	LOADABLEOPTS_DUMP_VAR(featsPerSquareMeter, float)
	LOADABLEOPTS_DUMP_VAR(threshold_max, float)
	LOADABLEOPTS_DUMP_VAR(threshold_delta, float)
	LOADABLEOPTS_DUMP_VAR(min_ICP_goodness, float)
	LOADABLEOPTS_DUMP_VAR(max_ICP_mahadist, double)
	LOADABLEOPTS_DUMP_VAR(maxKLd_for_merge, float)
	LOADABLEOPTS_DUMP_VAR(ransac_minSetSizeRatio, float)
	LOADABLEOPTS_DUMP_VAR(ransac_mahalanobisDistanceThreshold, float)
	LOADABLEOPTS_DUMP_VAR(ransac_chi2_quantile, double)
	LOADABLEOPTS_DUMP_VAR(ransac_prob_good_inliers, double)
	LOADABLEOPTS_DUMP_VAR(ransac_SOG_sigma_m, float)
	LOADABLEOPTS_DUMP_VAR(save_feat_coors, bool)
	LOADABLEOPTS_DUMP_VAR(debug_show_corrs, bool)
	LOADABLEOPTS_DUMP_VAR(debug_save_map_pairs, bool)

	LOADABLEOPTS_DUMP_VAR(feature_descriptor, int)

	feature_detector_options.dumpToTextStream(out);

	out << mrpt::format("\n");
}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void CGridMapAligner::TConfigParams::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& iniFile, const std::string& section)
{
	methodSelection =
		iniFile.read_enum(section, "methodSelection", methodSelection);

	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(
		featsPerSquareMeter, float, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(ransac_SOG_sigma_m, float, iniFile, section)

	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(threshold_max, float, iniFile, section)
	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(threshold_delta, float, iniFile, section)

	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(min_ICP_goodness, float, iniFile, section)
	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(max_ICP_mahadist, double, iniFile, section)

	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(maxKLd_for_merge, float, iniFile, section)
	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(
		ransac_minSetSizeRatio, float, iniFile, section)
	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(
		ransac_mahalanobisDistanceThreshold, float, iniFile, section)
	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(
		ransac_chi2_quantile, double, iniFile, section)
	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(
		ransac_prob_good_inliers, double, iniFile, section)

	MRPT_LOAD_CONFIG_VAR(save_feat_coors, bool, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(debug_show_corrs, bool, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(debug_save_map_pairs, bool, iniFile, section)

	feature_descriptor = iniFile.read_enum(
		section, "feature_descriptor", feature_descriptor, true);
	feature_detector_options.loadFromConfigFile(iniFile, section);
}

CPose3DPDF::Ptr CGridMapAligner::Align3DPDF(
	const mrpt::maps::CMetricMap* m1, const mrpt::maps::CMetricMap* m2,
	const CPose3DPDFGaussian& initialEstimationPDF, float* runningTime,
	void* info)
{
	MRPT_UNUSED_PARAM(m1);
	MRPT_UNUSED_PARAM(m2);
	MRPT_UNUSED_PARAM(initialEstimationPDF);
	MRPT_UNUSED_PARAM(runningTime);
	MRPT_UNUSED_PARAM(info);
	THROW_EXCEPTION("Align3D method not applicable to gridmap-aligner");
}
