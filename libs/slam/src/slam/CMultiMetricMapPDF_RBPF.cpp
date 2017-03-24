/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "slam-precomp.h"   // Precompiled headers

#include <mrpt/random.h>
#include <mrpt/math/utils.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/CFileStream.h>

#include <mrpt/maps/CMultiMetricMapPDF.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFGrid.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CLandmarksMap.h>
#include <mrpt/math.h>

#include <mrpt/slam/PF_aux_structs.h>


using namespace mrpt;
using namespace mrpt::bayes;
using namespace mrpt::math;
using namespace mrpt::slam;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::poses;
using namespace mrpt::random;
using namespace mrpt::utils;
using namespace std;

namespace mrpt
{
	namespace slam
	{
		/** Fills out a "TPoseBin2D" variable, given a path hypotesis and (if not set to NULL) a new pose appended at the end, using the KLD params in "options".
			*/
		template <>
		void KLF_loadBinFromParticle(
			detail::TPoseBin2D	&outBin,
			const TKLDParams  	&opts,
			const mrpt::maps::CRBPFParticleData	*currentParticleValue,
			const TPose3D			*newPoseToBeInserted)
		{
			// 2D pose approx: Use the latest pose only:
			if (newPoseToBeInserted)
			{
				outBin.x 	= round( newPoseToBeInserted->x / opts.KLD_binSize_XY );
				outBin.y	= round( newPoseToBeInserted->y / opts.KLD_binSize_XY );
				outBin.phi	= round( newPoseToBeInserted->yaw / opts.KLD_binSize_PHI );
			}
			else
			{
				ASSERT_(currentParticleValue && !currentParticleValue->robotPath.empty())
				const TPose3D &p = *currentParticleValue->robotPath.rbegin();
				outBin.x 	= round( p.x / opts.KLD_binSize_XY );
				outBin.y	= round( p.y / opts.KLD_binSize_XY );
				outBin.phi	= round( p.yaw / opts.KLD_binSize_PHI );
			}
		}

		/** Fills out a "TPathBin2D" variable, given a path hypotesis and (if not set to NULL) a new pose appended at the end, using the KLD params in "options".
			*/
		template <>
		void KLF_loadBinFromParticle(
			detail::TPathBin2D	&outBin,
			const TKLDParams  	&opts,
			const mrpt::maps::CRBPFParticleData	*currentParticleValue,
			const TPose3D			*newPoseToBeInserted)
		{
			const size_t lenBinPath = (currentParticleValue!=NULL) ? currentParticleValue->robotPath.size() : 0;

			// Set the output bin dimensionality:
			outBin.bins.resize(lenBinPath + (newPoseToBeInserted!=NULL ? 1:0) );

			// Is a path provided??
			if (currentParticleValue!=NULL)
				for (size_t i=0;i<lenBinPath;++i)	// Fill the bin data:
				{
					outBin.bins[i].x   = round( currentParticleValue->robotPath[i].x / opts.KLD_binSize_XY );
					outBin.bins[i].y   = round( currentParticleValue->robotPath[i].y / opts.KLD_binSize_XY );
					outBin.bins[i].phi = round( currentParticleValue->robotPath[i].yaw / opts.KLD_binSize_PHI );
				}

			// Is a newPose provided??
			if (newPoseToBeInserted!=NULL)
			{
				// And append the last pose: the new one:
				outBin.bins[lenBinPath].x   = round( newPoseToBeInserted->x / opts.KLD_binSize_XY );
				outBin.bins[lenBinPath].y   = round( newPoseToBeInserted->y / opts.KLD_binSize_XY );
				outBin.bins[lenBinPath].phi = round( newPoseToBeInserted->yaw / opts.KLD_binSize_PHI );
			}
		}
	}
}

// Include this AFTER specializations:
#include <mrpt/slam/PF_implementations.h>

/** Auxiliary for optimal sampling in RO-SLAM */
struct TAuxRangeMeasInfo
{
	TAuxRangeMeasInfo() :
		sensorLocationOnRobot(),
		sensedDistance(0),
		beaconID(INVALID_BEACON_ID),
		nGaussiansInMap(0)
	{}

	CPoint3D		sensorLocationOnRobot;
	float			sensedDistance;
	int64_t			beaconID;
	size_t			nGaussiansInMap; // Number of Gaussian modes in the map representation

	/** Auxiliary for optimal sampling in RO-SLAM */
	static bool cmp_Asc(const TAuxRangeMeasInfo &a, const TAuxRangeMeasInfo &b)
	{
		return a.nGaussiansInMap < b.nGaussiansInMap;
	}
};



/*----------------------------------------------------------------------------------
			prediction_and_update_pfAuxiliaryPFOptimal

  See paper reference in "PF_SLAM_implementation_pfAuxiliaryPFOptimal"
 ----------------------------------------------------------------------------------*/
void  CMultiMetricMapPDF::prediction_and_update_pfAuxiliaryPFOptimal(
	const mrpt::obs::CActionCollection	* actions,
	const mrpt::obs::CSensoryFrame		* sf,
	const bayes::CParticleFilter::TParticleFilterOptions &PF_options )
{
	MRPT_START

	PF_SLAM_implementation_pfAuxiliaryPFOptimal<mrpt::slam::detail::TPoseBin2D>( actions, sf, PF_options,options.KLD_params);

	MRPT_END
}

/*----------------------------------------------------------------------------------
			PF_SLAM_implementation_pfAuxiliaryPFStandard
 ----------------------------------------------------------------------------------*/
void  CMultiMetricMapPDF::prediction_and_update_pfAuxiliaryPFStandard(
	const mrpt::obs::CActionCollection	* actions,
	const mrpt::obs::CSensoryFrame		* sf,
	const bayes::CParticleFilter::TParticleFilterOptions &PF_options )
{
	MRPT_START

	PF_SLAM_implementation_pfAuxiliaryPFStandard<mrpt::slam::detail::TPoseBin2D>( actions, sf, PF_options,options.KLD_params);

	MRPT_END
}


/*----------------------------------------------------------------------------------
			prediction_and_update_pfOptimalProposal

For grid-maps:
==============
 Approximation by Grissetti et al:  Use scan matching to approximate
   the observation model by a Gaussian:
  See: "Improved Grid-based SLAM with Rao-Blackwellized PF by Adaptive Proposals
	       and Selective Resampling" (G. Grisetti, C. Stachniss, W. Burgard)

For beacon maps:
===============
  (JLBC: Method under development)

 ----------------------------------------------------------------------------------*/
void  CMultiMetricMapPDF::prediction_and_update_pfOptimalProposal(
	const mrpt::obs::CActionCollection	* actions,
	const mrpt::obs::CSensoryFrame		* sf,
	const bayes::CParticleFilter::TParticleFilterOptions &PF_options )
{
	MRPT_START

	// ----------------------------------------------------------------------
	//						PREDICTION STAGE
	// ----------------------------------------------------------------------
	CVectorDouble				rndSamples;
	size_t						M = m_particles.size();
	bool						updateStageAlreadyDone = false;
	CPose3D						initialPose,incrPose, finalPose;

	// ICP used if "pfOptimalProposal_mapSelection" = 0 or 1
	CICP				icp (options.icp_params);  // Set our ICP params instead of default ones.
	CICP::TReturnInfo	icpInfo;

	CParticleList::iterator		partIt;

	ASSERT_(sf!=NULL)

	// Find a robot movement estimation:
	CPose3D						motionModelMeanIncr;	// The mean motion increment:
	CPoseRandomSampler			robotActionSampler;
	{
		CActionRobotMovement2DPtr	robotMovement2D = actions->getBestMovementEstimation();

		// If there is no 2D action, look for a 3D action:
		if (robotMovement2D.present())
		{
			robotActionSampler.setPosePDF( robotMovement2D->poseChange.get_ptr() );
			motionModelMeanIncr = robotMovement2D->poseChange->getMeanVal();
		}
		else
		{
			CActionRobotMovement3DPtr	robotMovement3D = actions->getActionByClass<CActionRobotMovement3D>();
			if (robotMovement3D)
			{
				robotActionSampler.setPosePDF( robotMovement3D->poseChange );
				robotMovement3D->poseChange.getMean( motionModelMeanIncr );
			}
			else
			{
				motionModelMeanIncr.setFromValues(0,0,0);
			}
		}
	}

	// Average map will need to be updated after this:
	averageMapIsUpdated = false;

	// --------------------------------------------------------------------------------------
	//  Prediction:
	//
	//  Compute a new mean and covariance by sampling around the mean of the input "action"
	// --------------------------------------------------------------------------------------
	printf(" 1) Prediction...");
	M = m_particles.size();

	// To be computed as an average from all m_particles:
	size_t particleWithHighestW = 0;
	for (size_t i=0;i<M;i++)
		if (getW(i)>getW(particleWithHighestW))
			particleWithHighestW = i;


	//   The paths MUST already contain the starting location for each particle:
	ASSERT_( !m_particles[0].d->robotPath.empty() )

	// Build the local map of points for ICP:
	CSimplePointsMap	localMapPoints;
	CLandmarksMap		localMapLandmarks;
	bool built_map_points = false;
	bool built_map_lms = false;

	// Update particle poses:
	size_t i;
	for (i=0,partIt = m_particles.begin(); partIt!=m_particles.end(); partIt++,i++)
	{
		double extra_log_lik = 0; // Used for the optimal_PF with ICP

		// Set initial robot pose estimation for this particle:
		const CPose3D ith_last_pose = CPose3D(*partIt->d->robotPath.rbegin()); // The last robot pose in the path

		CPose3D		initialPoseEstimation = ith_last_pose + motionModelMeanIncr;

		// Use ICP with the map associated to particle?
		if ( options.pfOptimalProposal_mapSelection==0 ||
			 options.pfOptimalProposal_mapSelection==1 ||
			 options.pfOptimalProposal_mapSelection==3 )
		{
			CPosePDFGaussian			icpEstimation;

			// Configure the matchings that will take place in the ICP process:
			if (partIt->d->mapTillNow.m_pointsMaps.size())
			{
				ASSERT_(partIt->d->mapTillNow.m_pointsMaps.size()==1);
				//partIt->d->mapTillNow.m_pointsMaps[0]->insertionOptions.matchStaticPointsOnly = false;
			}

			CMetricMap *map_to_align_to = NULL;

			if (options.pfOptimalProposal_mapSelection==0)  // Grid map
			{
				ASSERT_( !partIt->d->mapTillNow.m_gridMaps.empty() );

				// Build local map of points.
				if (!built_map_points)
				{
					built_map_points=true;

					localMapPoints.insertionOptions.minDistBetweenLaserPoints =  0.02f; //3.0f * m_particles[0].d->mapTillNow.m_gridMaps[0]->getResolution();;
					localMapPoints.insertionOptions.isPlanarMap = true;
					sf->insertObservationsInto( &localMapPoints );
				}

				map_to_align_to = partIt->d->mapTillNow.m_gridMaps[0].pointer();
			}
			else
			if (options.pfOptimalProposal_mapSelection==3)  // Map of points
			{
				ASSERT_( !partIt->d->mapTillNow.m_pointsMaps.empty() );

				// Build local map of points.
				if (!built_map_points)
				{
					built_map_points=true;

					localMapPoints.insertionOptions.minDistBetweenLaserPoints =  0.02f; //3.0f * m_particles[0].d->mapTillNow.m_gridMaps[0]->getResolution();;
					localMapPoints.insertionOptions.isPlanarMap = true;
					sf->insertObservationsInto( &localMapPoints );
				}

				map_to_align_to = partIt->d->mapTillNow.m_pointsMaps[0].pointer();
			}
			else
			{
				ASSERT_( partIt->d->mapTillNow.m_landmarksMap.present() );

				// Build local map of LMs.
				if (!built_map_lms)
				{
					built_map_lms=true;
					sf->insertObservationsInto( &localMapLandmarks );
				}

				map_to_align_to = partIt->d->mapTillNow.m_landmarksMap.pointer();
			}

			ASSERT_(map_to_align_to!=NULL);

			// Use ICP to align to each particle's map:
			{
				CPosePDFPtr alignEst =
				icp.Align(
					map_to_align_to,
					&localMapPoints,
					CPose2D(initialPoseEstimation),
					NULL,
					&icpInfo);
				icpEstimation.copyFrom( *alignEst );
			}


			if (i==particleWithHighestW)
			{
				newInfoIndex = 1 - icpInfo.goodness; //newStaticPointsRatio; //* icpInfo.goodness;
			}

			// Set the gaussian pose:
			CPose3DPDFGaussian finalEstimatedPoseGauss( icpEstimation );

			//printf("[rbpf-slam] gridICP[%u]: %.02f%%\n", i, 100*icpInfo.goodness);
			if (icpInfo.goodness<options.ICPGlobalAlign_MinQuality && SFs.size())
			{
				printf("[rbpf-slam] Warning: gridICP[%u]: %.02f%% -> Using odometry instead!\n", (unsigned int)i, 100*icpInfo.goodness);
				icpEstimation.mean = CPose2D(initialPoseEstimation);
			}

			// As a way to take into account the odometry / "prior", use
			//  a correcting factor in the likelihood from the mismatch prior<->icp_estimate:
//			const double prior_dist_lin = initialPoseEstimation.distanceTo(icpEstimation.mean);
//			const double prior_dist_ang = std::abs( mrpt::math::wrapToPi( initialPoseEstimation.yaw()-icpEstimation.mean.phi() ) );
////			if (prior_dist_lin>0.10 || prior_dist_ang>DEG2RAD(3))
////				printf(" >>>>>>>>>> %f %f\n",prior_dist_lin,RAD2DEG(prior_dist_ang));
//			extra_log_lik = -(prior_dist_lin/0.20) - (prior_dist_ang/DEG2RAD(20));

//				printf("gICP: %.02f%%, Iters=%u\n",icpInfo.goodness,icpInfo.nIterations);

#if 0  // Use hacked ICP covariance:
			CPose3D Ap = finalEstimatedPoseGauss.mean - ith_last_pose;
			const double  Ap_dist = Ap.norm();

			finalEstimatedPoseGauss.cov.zeros();
			finalEstimatedPoseGauss.cov(0,0) = square( fabs(Ap_dist)*0.01 );
			finalEstimatedPoseGauss.cov(1,1) = square( fabs(Ap_dist)*0.01 );
			finalEstimatedPoseGauss.cov(2,2) = square( fabs(Ap.yaw())*0.02 );
#else
		// Use real ICP covariance (with a minimum level):
		keep_max( finalEstimatedPoseGauss.cov(0,0), square(0.002));
		keep_max( finalEstimatedPoseGauss.cov(1,1), square(0.002));
		keep_max( finalEstimatedPoseGauss.cov(2,2), square(DEG2RAD(0.1)));

#endif

			// Generate gaussian-distributed 2D-pose increments according to "finalEstimatedPoseGauss":
			// -------------------------------------------------------------------------------------------
			finalPose = finalEstimatedPoseGauss.mean;					// Add to the new robot pose:
			randomGenerator.drawGaussianMultivariate(rndSamples, finalEstimatedPoseGauss.cov );
			// Add noise:
			finalPose.setFromValues(
				finalPose.x() + rndSamples[0],
				finalPose.y() + rndSamples[1],
				finalPose.z(),
				finalPose.yaw() + rndSamples[2],
				finalPose.pitch(),
				finalPose.roll() );
		}
		else
		if ( options.pfOptimalProposal_mapSelection==2)
		{
			// --------------------------------------------------------
			//     Perform optimal sampling with the beacon map:
			//  Described in paper...
			// --------------------------------------------------------
			/** \todo Add paper ref!
			  */
			ASSERT_( partIt->d->mapTillNow.m_beaconMap.present() );
			CBeaconMapPtr beacMap = partIt->d->mapTillNow.m_beaconMap;

			updateStageAlreadyDone = true; // We'll also update the weight of the particle here

			// =====================================================================
			// SUMMARY:
			// For each beacon measurement in the SF, stored in "lstObservedRanges",
			//  compute the SOG of the observation model, and multiply all of them
			//  (fuse) in "fusedObsModels". The result will hopefully be a very small
			//  PDF where to draw a sample from for the new robot pose.
			// =====================================================================
			bool 		methodSOGorGrid = false; // TRUE=SOG
			CPoint3D  	newDrawnPosition;
			float  		firstEstimateRobotHeading=std::numeric_limits<float>::max();

			// The parameters to discard too far gaussians:
			CPoint3D  	centerPositionPrior( ith_last_pose );
			float		centerPositionPriorRadius=2.0f;

			if ( !robotActionSampler.isPrepared() )
			{
				firstEstimateRobotHeading = ith_last_pose.yaw();
				// If the map is empty: There is no solution!:
				// THROW_EXCEPTION("There is no odometry & the initial beacon map is empty: RO-SLAM has no solution -> ABORTED!!!");

				if (!beacMap->size())
				{
					// First iteration only...
					cerr << "[RO-SLAM] Optimal filtering without map & odometry...this message should appear only the first iteration!!" << endl;
				}
				else
				{
					// To make RO-SLAM to have a solution, arbitrarily fix one of the beacons so
					//  unbiguity dissapears.
					if ( beacMap->get(0).m_typePDF==CBeacon::pdfSOG)
					{
						cerr << "[RO-SLAM] Optimal filtering without map & odometry->FIXING ONE BEACON!" << endl;
						ASSERT_(beacMap->get(0).m_locationSOG.size()>0)

						CPoint3D 	fixedBeacon( beacMap->get(0).m_locationSOG[0].val.mean );

						// Pass to gaussian without uncertainty:
						beacMap->get(0).m_typePDF=CBeacon::pdfGauss;
						beacMap->get(0).m_locationSOG.clear();
						beacMap->get(0).m_locationGauss.mean = fixedBeacon;
						beacMap->get(0).m_locationGauss.cov.unit(3, 1e-6);
					}
				}
			} // end if there is no odometry


			// 1. Make the list of beacon IDs-ranges:
			// -------------------------------------------
			deque<TAuxRangeMeasInfo>	lstObservedRanges;

			for (CSensoryFrame::const_iterator itObs = sf->begin();itObs!=sf->end();++itObs)
			{
				if ( IS_CLASS( (*itObs),CObservationBeaconRanges) )
				{
					const CObservationBeaconRanges *obs = static_cast<const CObservationBeaconRanges*> (itObs->pointer());
					deque<CObservationBeaconRanges::TMeasurement>::const_iterator itRanges;
					for (itRanges=obs->sensedData.begin();itRanges!=obs->sensedData.end();itRanges++)
					{
						ASSERT_( itRanges->beaconID != INVALID_BEACON_ID )
						// only add those in the map:
						for (CBeaconMap::iterator itBeacs=beacMap->begin();itBeacs!=beacMap->end();++itBeacs)
						{
							if ( (itBeacs)->m_ID == itRanges->beaconID)
							{
								TAuxRangeMeasInfo	newMeas;
								newMeas.beaconID = itRanges->beaconID;
								newMeas.sensedDistance = itRanges->sensedDistance;
								newMeas.sensorLocationOnRobot = itRanges->sensorLocationOnRobot;

								ASSERT_( (itBeacs)->m_typePDF==CBeacon::pdfGauss || (itBeacs)->m_typePDF==CBeacon::pdfSOG )
								newMeas.nGaussiansInMap = (itBeacs)->m_typePDF==CBeacon::pdfSOG ? (itBeacs)->m_locationSOG.size() : 1 /*pdfGauss*/;

								lstObservedRanges.push_back( newMeas );
								break; // Next observation
							}
						}
					}
				}
			}

			//ASSERT_( lstObservedRanges.size()>=2 );

			// Sort ascending ranges: Smallest ranges first -> the problem is easiest!
			sort( lstObservedRanges.begin(),lstObservedRanges.end(), &TAuxRangeMeasInfo::cmp_Asc);


			if ( methodSOGorGrid )
			{
				CPointPDFSOG	fusedObsModels; //<- p(z_t|x_t) for all the range measurements (fused)
				fusedObsModels.clear();

				// 0. OPTIONAL: Create a "prior" as a first mode in "fusedObsModels"
				//       using odometry. If there is no odometry, we *absolutely* need
				//       at least one well-localized beacon at the beginning, or the symmetry cannot be broken!
				// -----------------------------------------------------------------------------------------------
				if ( robotActionSampler.isPrepared() )
				{
					CPointPDFSOG::TGaussianMode	newMode;
					newMode.log_w = 0;

					CPose3D auxPose= ith_last_pose + motionModelMeanIncr; // CPose3D(robotMovement->poseChange->getEstimatedPose()));
					firstEstimateRobotHeading = auxPose.yaw();

					newMode.val.mean = CPoint3D(auxPose);

					// Uncertainty in z is null:
					//CMatrix poseCOV =  robotMovement->poseChange->getEstimatedCovariance();
					CMatrixD poseCOV;
					robotActionSampler.getOriginalPDFCov2D( poseCOV );

					poseCOV.setSize(2,2);
					poseCOV.setSize(3,3);
					newMode.val.cov =poseCOV;
					fusedObsModels.push_back(newMode);	// Add it:
				}


				// 2. Generate the optimal proposal by fusing obs models
				// -------------------------------------------------------------
				for (CBeaconMap::iterator itBeacs=beacMap->begin();itBeacs!=beacMap->end();++itBeacs)
				{
					// for each observed beacon (by its ID), generate observation model:
					for (deque<TAuxRangeMeasInfo>::iterator itObs=lstObservedRanges.begin();itObs!=lstObservedRanges.end();++itObs)
					{
						if ((itBeacs)->m_ID==itObs->beaconID)
						{
							// Match:
							float sensedRange = itObs->sensedDistance;

							CPointPDFSOG	newObsModel;
							(itBeacs)->generateObservationModelDistribution(
								sensedRange,
								newObsModel,
								beacMap.pointer(),        			 // The beacon map, for options
								itObs->sensorLocationOnRobot,// Sensor location on robot
								centerPositionPrior,
								centerPositionPriorRadius );

							if (! fusedObsModels.size() )
							{
								// This is the first one: Just copy the obs. model here:
								fusedObsModels = newObsModel;
							}
							else
							{
								// Fuse with existing:
								CPointPDFSOG	tempFused(0);
								tempFused.bayesianFusion(
									fusedObsModels,
									newObsModel,
									3   // minMahalanobisDistToDrop
									);
								fusedObsModels = tempFused;
							}

							// Remove modes with negligible weights:
							// -----------------------------------------------------------

							{
								cout << "#modes bef: " << fusedObsModels.size() << " ESS: " << fusedObsModels.ESS() << endl;
								double max_w=-1e100;
								//int idx;

								CPointPDFSOG::iterator it;

								for (it=fusedObsModels.begin();it!=fusedObsModels.end();it++)
									max_w = max(max_w,(it)->log_w);	// keep the maximum mode weight

								for (it=fusedObsModels.begin();it!=fusedObsModels.end(); )
								{
									if (max_w - (it)->log_w > 20 ) // Remove the mode:
										 it = fusedObsModels.erase( it  );
									else  it++;
								}

								cout << "#modes after: " << fusedObsModels.size() << endl;
							}

							// Shall we simplify the PDF?
							// -----------------------------------------------------------
							CMatrixDouble currentCov;
							fusedObsModels.getCovariance(currentCov);
							ASSERT_(currentCov(0,0)>0 && currentCov(1,1)>0)
							if ( sqrt(currentCov(0,0))< 0.10f &&
								 sqrt(currentCov(1,1))< 0.10f &&
								 sqrt(currentCov(2,2))< 0.10f )
							{
								// Approximate by a single gaussian!
								CPoint3D currentMean;
								fusedObsModels.getMean(currentMean);
								fusedObsModels[0].log_w = 0;
								fusedObsModels[0].val.mean = currentMean;
								fusedObsModels[0].val.cov = currentCov;

								fusedObsModels.resize(1);
							}


							{/*
								CMatrixD  evalGrid;
								fusedObsModels.evaluatePDFInArea(-3,3,-3,3,0.1,0,evalGrid, true);
								evalGrid *= 1.0/evalGrid.maxCoeff();
								CImage imgF(evalGrid, true);
								static int autoCount=0;
								imgF.saveToFile(format("debug_%04i.png",autoCount++));*/
							}
						}
					} // end for itObs (in lstObservedRanges)

				} // end for itBeacs


				/** /
				COpenGLScene	scene;
				opengl::CSetOfObjects *obj = new opengl::CSetOfObjects();
				fusedObsModels.getAs3DObject( *obj );
				scene.insert( obj );
				CFileStream("debug.3Dscene",fomWrite) << scene;
				cout << "fusedObsModels # of modes: " << fusedObsModels.m_modes.size() << endl;
				printf("ESS: %f\n",fusedObsModels.ESS() );
				cout << fusedObsModels.getEstimatedCovariance() << endl;
				mrpt::system::pause(); / **/

				if (beacMap->size())
					fusedObsModels.drawSingleSample( newDrawnPosition );
			}
			else
			{
				// =============================
				//         GRID METHOD
				// =============================

				float   grid_min_x = ith_last_pose.x() - 0.5f;
				float   grid_max_x = ith_last_pose.x() + 0.5f;
				float   grid_min_y = ith_last_pose.y() - 0.5f;
				float   grid_max_y = ith_last_pose.y() + 0.5f;
				float   grid_resXY = 0.02f;

				bool repeatGridCalculation=false;

				do
				{
					CPosePDFGrid	*pdfGrid = new CPosePDFGrid(
						grid_min_x,grid_max_x,
						grid_min_y,grid_max_y,
						grid_resXY, DEG2RAD(180), 0, 0 );

					pdfGrid->uniformDistribution();

					// Fuse all the observation models in the grid:
					// -----------------------------------------------------
					for (CBeaconMap::iterator itBeacs=beacMap->begin();itBeacs!=beacMap->end();++itBeacs)
					{
						// for each observed beacon (by its ID), generate observation model:
						for (deque<TAuxRangeMeasInfo>::iterator itObs=lstObservedRanges.begin();itObs!=lstObservedRanges.end();++itObs)
						{
							if ((itBeacs)->m_ID==itObs->beaconID)
							{
								// Match:
								float sensedRange = itObs->sensedDistance;

/** /
								CPointPDFSOG	newObsModel;
								(itBeacs)->generateObservationModelDistribution(
									sensedRange,
									newObsModel,
									beacMap,        			 // The beacon map, for options
									itObs->sensorLocationOnRobot,// Sensor location on robot
									centerPositionPrior,
									centerPositionPriorRadius );
/ **/
								for (size_t idxX=0;idxX<pdfGrid->getSizeX();idxX++)
								{
									float grid_x = pdfGrid->idx2x(idxX);
									for (size_t idxY=0;idxY<pdfGrid->getSizeY();idxY++)
									{
										double grid_y = pdfGrid->idx2y(idxY);

										// Evaluate obs model:
										double *cell = pdfGrid->getByIndex(idxX,idxY,0);

										//
										double lik = 1; //newObsModel.evaluatePDF(CPoint3D(grid_x,grid_y,0),true);
										switch ((itBeacs)->m_typePDF)
										{
											case CBeacon::pdfSOG:
											{
												CPointPDFSOG	*sog = & (itBeacs)->m_locationSOG;
												double sensorSTD2 = square(beacMap->likelihoodOptions.rangeStd);

												CPointPDFSOG::iterator	it;
												for (it = sog->begin();it!=sog->end();it++)
												{
													lik *= exp( -0.5*square( sensedRange -
														(it)->val.mean.distance2DTo( grid_x+itObs->sensorLocationOnRobot.x(), grid_y+itObs->sensorLocationOnRobot.y()))/sensorSTD2 );
												}
											}
											break;
											default: break; //THROW_EXCEPTION("NO")

										}

										(*cell) *= lik;

									} // for idxY
								} // for idxX
								pdfGrid->normalize();
							} // end if match
						} // end for itObs
					} // end for beacons in map

					// Draw the single pose from the grid:
					if (beacMap->size())
					{
						// Take the most likely cell:
						float maxW = -1e10f;
						float maxX=0,maxY=0;
						for (size_t idxX=0;idxX<pdfGrid->getSizeX();idxX++)
						{
							//float grid_x = pdfGrid->idx2x(idxX);
							for (size_t idxY=0;idxY<pdfGrid->getSizeY();idxY++)
							{
								//float grid_y = pdfGrid->idx2y(idxY);

								// Evaluate obs model:
								float c = *pdfGrid->getByIndex(idxX,idxY,0);
								if (c>maxW)
								{
									maxW=c;
									maxX=pdfGrid->idx2x(idxX);
									maxY=pdfGrid->idx2y(idxY);
								}
							}
						}
						newDrawnPosition.x( maxX );
						newDrawnPosition.y( maxY );

 #if 0
						{
							//cout << "Grid: " << pdfGaussApprox << endl;
							//pdfGrid->saveToTextFile("debug.txt");
							CMatrixDouble outMat;
							pdfGrid->getAsMatrix(0, outMat );
							outMat *= 1.0f/outMat.maxCoeff();
							CImage imgF(outMat, true);
							static int autocount=0;
							imgF.saveToFile(format("debug_grid_%f_%05i.png",grid_resXY,autocount++));
							printf("grid res: %f   MAX: %f,%f\n",grid_resXY,maxX,maxY);
							//mrpt::system::pause();
						}
#endif

						if (grid_resXY>0.01f)
						{
							grid_min_x = maxX - 0.03f;
							grid_max_x = maxX + 0.03f;
							grid_min_y = maxY - 0.03f;
							grid_max_y = maxY + 0.03f;
							grid_resXY=0.002f;
							repeatGridCalculation = true;
						}
						else
							repeatGridCalculation = false;


						/*
						// Approximate by a Gaussian:
						CPosePDFGaussian pdfGaussApprox(
							pdfGrid->getEstimatedPose(),
							pdfGrid->getEstimatedCovariance() );
						CPose2D newDrawnPose;
						//pdfGrid->drawSingleSample( newDrawnPose );
						pdfGaussApprox.drawSingleSample( newDrawnPose );
						newDrawnPosition = newDrawnPose;
						*/
					}
					delete pdfGrid; pdfGrid = NULL;

				} while (repeatGridCalculation);

				// newDrawnPosition has the pose:
			}

			// 3. Get the new "finalPose" of this particle as a random sample from the
			//  optimal proposal:
			// ---------------------------------------------------------------------------
			if (!beacMap->size())
			{
				// If we are in the first iteration (no map yet!) just stay still:
				finalPose = ith_last_pose;
			}
			else
			{
				cout << "Drawn: " << newDrawnPosition << endl;
				//cout << "Final cov was:\n" << fusedObsModels.getEstimatedCovariance() << endl << endl;

				ASSERT_(firstEstimateRobotHeading!=std::numeric_limits<float>::max()) // Make sure it was initialized

				finalPose.setFromValues(
					newDrawnPosition.x(),
					newDrawnPosition.y(),
					newDrawnPosition.z(),
					firstEstimateRobotHeading, 0,0);
			}
			/** \todo If there are 2+ sensors on the robot, compute phi?
			  */

			// 4. Update the weight:
			//     In optimal sampling this is the expectation of the
			//     observation likelihood: This is the observation likelihood averaged
			//     over the whole optimal proposal.
			// ---------------------------------------------------------------------------
			double weightUpdate=0;
			partIt->log_w += PF_options.powFactor * weightUpdate;

		}
		else
		{
			// By default:
			// Generate gaussian-distributed 2D-pose increments according to mean-cov:
			if ( !robotActionSampler.isPrepared() )
				THROW_EXCEPTION("Action list does not contain any CActionRobotMovement2D or CActionRobotMovement3D object!");

			robotActionSampler.drawSample( incrPose );

			finalPose = ith_last_pose + incrPose;
		}

		// Insert as the new pose in the path:
		partIt->d->robotPath.push_back( finalPose );

		// ----------------------------------------------------------------------
		//						UPDATE STAGE
		// ----------------------------------------------------------------------
		if (!updateStageAlreadyDone)
		{
			partIt->log_w +=
				PF_options.powFactor *
				(PF_SLAM_computeObservationLikelihoodForParticle(PF_options,i,*sf,finalPose)
				+ extra_log_lik);
		} // if update not already done...

	} // end of for each particle "i" & "partIt"

	printf("Ok\n");

	MRPT_END
}

/*---------------------------------------------------------------
			prediction_and_update_pfStandardProposal
 ---------------------------------------------------------------*/
void  CMultiMetricMapPDF::prediction_and_update_pfStandardProposal(
	const mrpt::obs::CActionCollection	* actions,
	const mrpt::obs::CSensoryFrame		* sf,
	const bayes::CParticleFilter::TParticleFilterOptions &PF_options )
{
	MRPT_START

	PF_SLAM_implementation_pfStandardProposal<mrpt::slam::detail::TPoseBin2D>(actions, sf, PF_options,options.KLD_params);

	// Average map will need to be updated after this:
	averageMapIsUpdated = false;

	MRPT_END
}


// Specialization for my kind of particles:
void CMultiMetricMapPDF::PF_SLAM_implementation_custom_update_particle_with_new_pose(
	CRBPFParticleData *particleData,
	const TPose3D &newPose) const
{
	particleData->robotPath.push_back( newPose );
}

// Specialization for RBPF maps:
bool CMultiMetricMapPDF::PF_SLAM_implementation_doWeHaveValidObservations(
	const CMultiMetricMapPDF::CParticleList	&particles,
	const CSensoryFrame *sf) const
{
	if (sf==NULL) return false;
	ASSERT_(!particles.empty())
	return particles.begin()->d.get()->mapTillNow.canComputeObservationsLikelihood( *sf );
}

/** Do not move the particles until the map is populated.  */
bool CMultiMetricMapPDF::PF_SLAM_implementation_skipRobotMovement() const
{
	return 0==getNumberOfObservationsInSimplemap();
}



/*---------------------------------------------------------------
 Evaluate the observation likelihood for one
   particle at a given location
 ---------------------------------------------------------------*/
double CMultiMetricMapPDF::PF_SLAM_computeObservationLikelihoodForParticle(
	const CParticleFilter::TParticleFilterOptions	&PF_options,
	const size_t			particleIndexForMap,
	const CSensoryFrame		&observation,
	const CPose3D			&x ) const
{
	MRPT_UNUSED_PARAM(PF_options);
	CMultiMetricMap *map = const_cast<CMultiMetricMap *>(&m_particles[particleIndexForMap].d->mapTillNow);
	double	ret = 0;
	for (CSensoryFrame::const_iterator it=observation.begin();it!=observation.end();++it)
		ret += map->computeObservationLikelihood( (CObservation*)it->pointer(), x );
	return ret;
}

