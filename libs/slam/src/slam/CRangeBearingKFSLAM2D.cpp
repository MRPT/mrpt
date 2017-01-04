/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "slam-precomp.h"   // Precompiled headers

#include <mrpt/slam/CRangeBearingKFSLAM2D.h>
#include <mrpt/slam/data_association.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/math/utils.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/system/os.h>

#include <mrpt/opengl/stock_objects.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CEllipsoid.h>

using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::math;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace mrpt;
using namespace std;

#define STATS_EXPERIMENT 0
#define STATS_EXPERIMENT_ALSO_NC 1

/*---------------------------------------------------------------
							Constructor
  ---------------------------------------------------------------*/
CRangeBearingKFSLAM2D::CRangeBearingKFSLAM2D( ) :
	options(),
	m_action(),m_SF(),
	m_IDs(),
	m_SFs()
{
	reset();
}

/*---------------------------------------------------------------
							reset
  ---------------------------------------------------------------*/
void CRangeBearingKFSLAM2D::reset()
{
	m_action.clear_unique();
	m_SF.clear_unique();
	m_IDs.clear();
	m_SFs.clear();

	// INIT KF STATE
	m_xkk.assign(3,0);	// State: 3D pose (x,y,phi)

	// Initial cov:
	m_pkk.setSize(3,3);
	m_pkk.zeros();
}

/*---------------------------------------------------------------
							Destructor
  ---------------------------------------------------------------*/
CRangeBearingKFSLAM2D::~CRangeBearingKFSLAM2D()
{
}

/*---------------------------------------------------------------
							getCurrentRobotPose
  ---------------------------------------------------------------*/
void CRangeBearingKFSLAM2D::getCurrentRobotPose(
	CPosePDFGaussian      &out_robotPose ) const
{
	MRPT_START

	ASSERT_(m_xkk.size()>=3);

	// Set 6D pose mean:
	out_robotPose.mean = CPose2D(m_xkk[0],m_xkk[1],m_xkk[2]);

	// and cov:
	CMatrixTemplateNumeric<kftype>  COV(3,3);
	m_pkk.extractMatrix(0,0,COV);
	out_robotPose.cov = COV;

	MRPT_END
}


/*---------------------------------------------------------------
							getCurrentState
  ---------------------------------------------------------------*/
void CRangeBearingKFSLAM2D::getCurrentState(
	CPosePDFGaussian      &out_robotPose,
	std::vector<TPoint2D>  &out_landmarksPositions,
	std::map<unsigned int,CLandmark::TLandmarkID> &out_landmarkIDs,
	CVectorDouble      &out_fullState,
	CMatrixDouble      &out_fullCovariance ) const
{
	MRPT_START

	ASSERT_(m_xkk.size()>=3);

	// Set 6D pose mean:
	out_robotPose.mean = CPose2D(m_xkk[0],m_xkk[1],m_xkk[2]);

	// and cov:
	CMatrixTemplateNumeric<kftype>  COV(3,3);
	m_pkk.extractMatrix(0,0,COV);
	out_robotPose.cov = COV;


	// Landmarks:
	ASSERT_( ((m_xkk.size() - 3) % 2)==0 );
	size_t i,nLMs = (m_xkk.size() - 3) / 2;
	out_landmarksPositions.resize(nLMs);
	for (i=0;i<nLMs;i++)
	{
		out_landmarksPositions[i].x = m_xkk[3+i*2+0];
		out_landmarksPositions[i].y = m_xkk[3+i*2+1];
	} // end for i

	// IDs:
	out_landmarkIDs = m_IDs.getInverseMap(); //m_IDs_inverse;

    // Full state:
    out_fullState.resize( m_xkk.size() );
	for (KFVector::Index i=0;i<m_xkk.size();i++)
		out_fullState[i] = m_xkk[i];

	// Full cov:
	out_fullCovariance = m_pkk;

	MRPT_END
}


/*---------------------------------------------------------------
						processActionObservation
  ---------------------------------------------------------------*/
void  CRangeBearingKFSLAM2D::processActionObservation(
    CActionCollectionPtr	&action,
    CSensoryFramePtr		&SF )
{
	MRPT_START

	m_action = action;
	m_SF = SF;

	// Sanity check:
	ASSERT_( m_IDs.size() == this->getNumberOfLandmarksInTheMap() );

	// ===================================================================================================================
	// Here's the meat!: Call the main method for the KF algorithm, which will call all the callback methods as required:
	// ===================================================================================================================
	runOneKalmanIteration();

	// =============================================================
	//  ADD TO SFs SEQUENCE
	// =============================================================
	if (options.create_simplemap)
	{
		CPosePDFGaussianPtr	auxPosePDF = CPosePDFGaussian::Create();
		getCurrentRobotPose( *auxPosePDF );
		m_SFs.insert( auxPosePDF , SF );
	}

    MRPT_END
}


/** Must return the action vector u.
  * \param out_u The action vector which will be passed to OnTransitionModel
  */
void CRangeBearingKFSLAM2D::OnGetAction(KFArray_ACT &u) const
{
	// Get odometry estimation:
	CActionRobotMovement2DPtr actMov2D = m_action->getBestMovementEstimation();
	CActionRobotMovement3DPtr actMov3D = m_action->getActionByClass<CActionRobotMovement3D>();
	if (actMov3D)
	{
		u[0]=actMov3D->poseChange.mean.x();
		u[1]=actMov3D->poseChange.mean.y();
		u[2]=actMov3D->poseChange.mean.yaw();
	}
	else
	if (actMov2D)
	{
		CPose2D estMovMean;
		actMov2D->poseChange->getMean(estMovMean);
		u[0]=estMovMean.x();
		u[1]=estMovMean.y();
		u[2]=estMovMean.phi();
	}
	else
	{
		// Left u as zeros
		for (size_t i=0;i<3;i++) u[i]=0;
	}
}


/** This virtual function musts implement the prediction model of the Kalman filter.
 */
void  CRangeBearingKFSLAM2D::OnTransitionModel(
	const KFArray_ACT &u,
	KFArray_VEH       &xv,
	bool &out_skipPrediction ) const
{
	MRPT_START

	// Do not update the vehicle pose & its covariance until we have some landmakrs in the map,
	// otherwise, we are imposing a lower bound to the best uncertainty from now on:
	if (size_t(m_xkk.size()) == get_vehicle_size() )
	{
		out_skipPrediction = true;
		return;
	}

	CPose2D  robotPose(xv[0],xv[1],xv[2]);
	CPose2D  odoIncrement(u[0],u[1],u[2]);

	// Pose composition:
	robotPose = robotPose + odoIncrement;

	xv[0]=robotPose.x();
	xv[1]=robotPose.y();
	xv[2]=robotPose.phi();

	MRPT_END
}

/** This virtual function musts calculate the Jacobian F of the prediction model.
 */
void  CRangeBearingKFSLAM2D::OnTransitionJacobian( KFMatrix_VxV  &F ) const
{
	MRPT_START

	// The jacobian of the transition function:  dfv_dxv
	CActionRobotMovement2DPtr act2D = m_action->getBestMovementEstimation();
	CActionRobotMovement3DPtr act3D = m_action->getActionByClass<CActionRobotMovement3D>();

	if (act3D && act2D)  THROW_EXCEPTION("Both 2D & 3D odometry are present!?!?")

	TPoint2D  Ap;

	if (act3D)
	{
		Ap = TPoint2D(CPose2D(act3D->poseChange.mean));
	}
	else if (act2D)
	{
		Ap = TPoint2D(act2D->poseChange->getMeanVal());
	}
	else
	{
		// No odometry at all:
		F.setIdentity(); // Unit diagonal
		return;
	}

	const kftype cy = cos(m_xkk[2]);
	const kftype sy = sin(m_xkk[2]);

	const kftype Ax = Ap.x;
	const kftype Ay = Ap.y;

	F.setIdentity(); // Unit diagonal

	F.get_unsafe(0,2) = -Ax*sy-Ay*cy;
	F.get_unsafe(1,2) =  Ax*cy-Ay*sy;

	MRPT_END
}

/** This virtual function musts calculate de noise matrix of the prediction model.
 */
void  CRangeBearingKFSLAM2D::OnTransitionNoise( KFMatrix_VxV &Q ) const
{
	MRPT_START

	// The uncertainty of the 2D odometry, projected from the current position:
	CActionRobotMovement2DPtr act2D = m_action->getBestMovementEstimation();
	CActionRobotMovement3DPtr act3D = m_action->getActionByClass<CActionRobotMovement3D>();

	if (act3D && act2D)  THROW_EXCEPTION("Both 2D & 3D odometry are present!?!?")

	CPosePDFGaussian odoIncr;

	if (!act3D && !act2D)
	{
		// Use constant Q:
		Q.zeros();
		ASSERT_(size_t(options.stds_Q_no_odo.size())==Q.getColCount())

		for (size_t i=0;i<3;i++)
			Q.get_unsafe(i,i) = square( options.stds_Q_no_odo[i]);
		return;
	}
	else
	{
		if (act2D)
		{
			// 2D odometry:
			odoIncr = CPosePDFGaussian(*act2D->poseChange);
		}
		else
		{
			// 3D odometry:
			odoIncr = CPosePDFGaussian(act3D->poseChange);
		}
	}

	odoIncr.rotateCov(m_xkk[2]);

	Q = odoIncr.cov;

	MRPT_END
}

void CRangeBearingKFSLAM2D::OnObservationModel(
	const vector_size_t       &idx_landmarks_to_predict,
	vector_KFArray_OBS  &out_predictions ) const
{
	MRPT_START

	// Get the sensor pose relative to the robot:
	CObservationBearingRangePtr obs = m_SF->getObservationByClass<CObservationBearingRange>();
	ASSERTMSG_(obs,"*ERROR*: This method requires an observation of type CObservationBearingRange")
	const CPose2D  sensorPoseOnRobot = CPose2D(obs->sensorLocationOnRobot);

	/* -------------------------------------------
       Equations, obtained using matlab, of the relative 2D position of a landmark (xi,yi), relative
          to a robot 2D pose (x0,y0,phi)
        Refer to technical report "6D EKF derivation...", 2008

		x0 y0 phi0         % Robot's 2D pose
        x0s y0s phis      % Sensor's 2D pose relative to robot
        xi yi             % Absolute 2D landmark coordinates:

        Hx : dh_dxv   -> Jacobian of the observation model wrt the robot pose
        Hy : dh_dyi   -> Jacobian of the observation model wrt each landmark mean position

        Sizes:
	     h:  1x2
         Hx: 2x3
         Hy: 2x2
	  ------------------------------------------- */
	// Mean of the prior of the robot pose:
	const CPose2D  robotPose( m_xkk[0],m_xkk[1],m_xkk[2] );

    const size_t  vehicle_size = get_vehicle_size();
    const size_t  feature_size = get_feature_size();

	const CPose2D sensorPoseAbs = robotPose + sensorPoseOnRobot;

	// ---------------------------------------------------
	// Observation prediction
	// ---------------------------------------------------
	const size_t N = idx_landmarks_to_predict.size();
	out_predictions.resize(N);
	for (size_t i=0;i<N;i++)
	{
		const size_t idx_lm = idx_landmarks_to_predict[i];
		ASSERTDEB_(idx_lm<this->getNumberOfLandmarksInTheMap());

		// Landmark absolute position in the map:
		const kftype xi = m_xkk[ vehicle_size + feature_size*idx_lm + 0 ];
		const kftype yi = m_xkk[ vehicle_size + feature_size*idx_lm + 1 ];

		const double Axi = xi-sensorPoseAbs.x();
		const double Ayi = yi-sensorPoseAbs.y();

		out_predictions[i][0] = sqrt( square(Axi)+square(Ayi) );   // Range
		out_predictions[i][1] = mrpt::math::wrapToPi( atan2(Ayi,Axi) - sensorPoseAbs.phi() ); // Yaw
	}

	MRPT_END
}

void CRangeBearingKFSLAM2D::OnObservationJacobians(
	const size_t &idx_landmark_to_predict,
	KFMatrix_OxV &Hx,
	KFMatrix_OxF &Hy ) const
{
	MRPT_START

	// Get the sensor pose relative to the robot:
	CObservationBearingRangePtr obs = m_SF->getObservationByClass<CObservationBearingRange>();
	ASSERTMSG_(obs,"*ERROR*: This method requires an observation of type CObservationBearingRange")
	const CPose2D  sensorPoseOnRobot = CPose2D(obs->sensorLocationOnRobot);

	/* -------------------------------------------
       Equations, obtained using matlab, of the relative 2D position of a landmark (xi,yi), relative
          to a robot 2D pose (x0,y0,phi)
        Refer to technical report "6D EKF derivation...", 2008

		x0 y0 phi0         % Robot's 2D pose
        x0s y0s phis      % Sensor's 2D pose relative to robot
        xi yi             % Absolute 2D landmark coordinates:

        Hx : dh_dxv   -> Jacobian of the observation model wrt the robot pose
        Hy : dh_dyi   -> Jacobian of the observation model wrt each landmark mean position

        Sizes:
	     h:  1x2
         Hx: 2x3
         Hy: 2x2
	  ------------------------------------------- */
	// Mean of the prior of the robot pose:
	const CPose2D  robotPose( m_xkk[0],m_xkk[1],m_xkk[2] );

    const size_t  vehicle_size = get_vehicle_size();
    const size_t  feature_size = get_feature_size();

    // Robot 6D pose:
    const kftype x0 = m_xkk[0];
    const kftype y0 = m_xkk[1];
    const kftype phi0 = m_xkk[2];

    const kftype cphi0 = cos(phi0);
    const kftype sphi0 = sin(phi0);

    // Sensor 2D pose on robot:
    const kftype x0s = sensorPoseOnRobot.x();
    const kftype y0s = sensorPoseOnRobot.y();
	const kftype phis = sensorPoseOnRobot.phi();

    const kftype cphi0s = cos(phi0+phis);
    const kftype sphi0s = sin(phi0+phis);

	const CPose2D sensorPoseAbs = robotPose + sensorPoseOnRobot;

	// Landmark absolute position in the map:
	const kftype xi = m_xkk[ vehicle_size + feature_size*idx_landmark_to_predict + 0 ];
	const kftype yi = m_xkk[ vehicle_size + feature_size*idx_landmark_to_predict + 1 ];

	// ---------------------------------------------------
	// Generate dhi_dxv: A 2x3 block
	// ---------------------------------------------------
	const kftype EXP1 = -2*yi*y0s*cphi0-2*yi*y0+2*xi*y0s*sphi0-2*xi*x0-2*xi*x0s*cphi0-2*yi*x0s*sphi0+2*y0s*y0*cphi0-2*y0s*x0*sphi0+2*y0*x0s*sphi0+square(x0)+2*x0s*x0*cphi0+square(x0s)+square(y0s)+square(xi)+square(yi)+square(y0);
	const kftype sqrtEXP1_1 = kftype(1)/sqrt(EXP1);

	const kftype EXP2 = cphi0s*xi+sphi0s*yi-sin(phis)*y0s-y0*sphi0s-x0s*cos(phis)-x0*cphi0s;
	const kftype EXP2sq = square(EXP2);

	const kftype EXP3 = -sphi0s*xi+cphi0s*yi-cos(phis)*y0s-y0*cphi0s+x0s*sin(phis)+x0*sphi0s;
	const kftype EXP3sq = square(EXP3);

	const kftype EXP4 = kftype(1)/(1+EXP3sq/EXP2sq);

	Hx.get_unsafe(0,0)= (-xi-sphi0*y0s+cphi0*x0s+x0)*sqrtEXP1_1;
	Hx.get_unsafe(0,1)= (-yi+cphi0*y0s+y0+sphi0*x0s)*sqrtEXP1_1;
	Hx.get_unsafe(0,2)= (y0s*xi*cphi0+y0s*yi*sphi0-y0*y0s*sphi0-x0*y0s*cphi0+x0s*xi*sphi0-x0s*yi*cphi0+y0*x0s*cphi0-x0s*x0*sphi0)*sqrtEXP1_1;

	Hx.get_unsafe(1,0)= (sphi0s/(EXP2)+(EXP3)/EXP2sq*cphi0s) * EXP4;
	Hx.get_unsafe(1,1)= (-cphi0s/(EXP2)+(EXP3)/EXP2sq*sphi0s) * EXP4;
	Hx.get_unsafe(1,2)= ((-cphi0s*xi-sphi0s*yi+y0*sphi0s+x0*cphi0s)/(EXP2)-(EXP3)/EXP2sq*(-sphi0s*xi+cphi0s*yi-y0*cphi0s+x0*sphi0s)) * EXP4;

	// ---------------------------------------------------
	// Generate dhi_dyi: A 2x2 block
	// ---------------------------------------------------
	Hy.get_unsafe(0,0)= (xi+sphi0*y0s-cphi0*x0s-x0)*sqrtEXP1_1;
	Hy.get_unsafe(0,1)= (yi-cphi0*y0s-y0-sphi0*x0s)*sqrtEXP1_1;

	Hy.get_unsafe(1,0)= (-sphi0s/(EXP2)-(EXP3)/EXP2sq*cphi0s)* EXP4;
	Hy.get_unsafe(1,1)= (cphi0s/(EXP2)-(EXP3)/EXP2sq*sphi0s) * EXP4;

	MRPT_END
}


/** This is called between the KF prediction step and the update step, and the application must return the observations and, when applicable, the data association between these observations and the current map.
  *
  * \param out_z N vectors, each for one "observation" of length OBS_SIZE, N being the number of "observations": how many observed landmarks for a map, or just one if not applicable.
  * \param out_data_association An empty vector or, where applicable, a vector where the i'th element corresponds to the position of the observation in the i'th row of out_z within the system state vector (in the range [0,getNumberOfLandmarksInTheMap()-1]), or -1 if it is a new map element and we want to insert it at the end of this KF iteration.
  * \param in_S The full covariance matrix of the observation predictions (i.e. the "innovation covariance matrix"). This is a M·O x M·O matrix with M=length of "in_lm_indices_in_S".
  * \param in_lm_indices_in_S The indices of the map landmarks (range [0,getNumberOfLandmarksInTheMap()-1]) that can be found in the matrix in_S.
  *
  *  This method will be called just once for each complete KF iteration.
  * \note It is assumed that the observations are independent, i.e. there are NO cross-covariances between them.
  */
void CRangeBearingKFSLAM2D::OnGetObservationsAndDataAssociation(
	vector_KFArray_OBS    &Z,
	vector_int                  &data_association,
	const vector_KFArray_OBS   &all_predictions,
	const KFMatrix              &S,
	const vector_size_t         &lm_indices_in_S,
	const KFMatrix_OxO          &R
	)
{
	MRPT_START

    //static const size_t vehicle_size = get_vehicle_size();
    static const size_t obs_size  = get_observation_size();

	// Z: Observations
	CObservationBearingRange::TMeasurementList::const_iterator itObs;

	CObservationBearingRangePtr obs = m_SF->getObservationByClass<CObservationBearingRange>();
	ASSERTMSG_(obs,"*ERROR*: This method requires an observation of type CObservationBearingRange")

	const size_t N = obs->sensedData.size();
	Z.resize(N);

	size_t row;
	for (row=0,itObs = obs->sensedData.begin();itObs!=obs->sensedData.end();++itObs,++row)
	{
		// Fill one row in Z:
		Z[row][0] =itObs->range;
		Z[row][1] =itObs->yaw;
		ASSERTMSG_(itObs->pitch==0,"ERROR: Observation contains pitch!=0 but this is 2D-SLAM!!!")
	}

	// Data association:
	// ---------------------
	data_association.assign(N,-1);  // Initially, all new landmarks

	// For each observed LM:
	vector_size_t   obs_idxs_needing_data_assoc;
	obs_idxs_needing_data_assoc.reserve(N);

	{
		vector_int::iterator itDA;
		CObservationBearingRange::TMeasurementList::const_iterator itObs;
		size_t row;
		for (row=0,itObs = obs->sensedData.begin(),itDA=data_association.begin();itObs!=obs->sensedData.end();++itObs,++itDA,++row)
		{
    		// Fill data asociation: Using IDs!
			if (itObs->landmarkID<0)
				obs_idxs_needing_data_assoc.push_back(row);
			else
			{
    			mrpt::utils::bimap<CLandmark::TLandmarkID,unsigned int>::iterator itID;
    			if ( (itID = m_IDs.find_key( itObs->landmarkID )) != m_IDs.end()  )
    				*itDA = itID->second; // This row in Z corresponds to the i'th map element in the state vector:
			}
		}
	}

	// ---- Perform data association ----
	//  Only for observation indices in "obs_idxs_needing_data_assoc"
	if (obs_idxs_needing_data_assoc.empty())
	{
		// We don't need to do DA:
		m_last_data_association = TDataAssocInfo();

		// Save them for the case the external user wants to access them:
		for (size_t idxObs=0;idxObs<data_association.size();idxObs++)
		{
			int da = data_association[idxObs];
			if (da>=0)
				m_last_data_association.results.associations[idxObs] = data_association[idxObs];
		}

#if STATS_EXPERIMENT	// DEBUG: Generate statistic info
		{
			static CFileOutputStream fC ("metric_stats_C.txt",true);
			#if STATS_EXPERIMENT_ALSO_NC
			static CFileOutputStream fNC("metric_stats_NC.txt",true);
			#endif

			vector_int::iterator itDA;
			size_t idx_obs = 0;
			for(itDA=data_association.begin();itDA!=data_association.end();++itDA,++idx_obs)
			{
				if (*itDA==-1) continue;
				const size_t lm_idx_in_map = *itDA;
				size_t valid_idx_pred = find_in_vector(lm_idx_in_map,lm_indices_in_S);

				const KFArray_OBS &obs_mu = Z[idx_obs];

				for (size_t idx_pred=0;idx_pred<lm_indices_in_S.size();idx_pred++)
				{
					const KFArray_OBS &lm_mu  = all_predictions[lm_indices_in_S[idx_pred]];

					const size_t base_idx_in_S = obs_size*idx_pred;
					KFMatrix_OxO  lm_cov;
					S.extractMatrix(base_idx_in_S,base_idx_in_S,lm_cov);

					double md,log_pdf;
					mahalanobisDistance2AndLogPDF(lm_mu-obs_mu, lm_cov, md,log_pdf);

					if (valid_idx_pred==idx_pred)
						fC.printf("%e %e\n",md,log_pdf);
					else
					{
					#if STATS_EXPERIMENT_ALSO_NC
						fNC.printf("%e %e\n",md,log_pdf);
					#endif
					}
				}
			}
		}
#endif

	}
	else
	{
		// Build a Z matrix with the observations that need dat.assoc:
		const size_t nObsDA = obs_idxs_needing_data_assoc.size();

		CMatrixTemplateNumeric<kftype>  Z_obs_means(nObsDA,obs_size);
		for (size_t i=0;i<nObsDA;i++)
		{
			const size_t idx = obs_idxs_needing_data_assoc[i];
			for (unsigned k=0;k<obs_size;k++)
				Z_obs_means.get_unsafe(i,k) = Z[idx][k];
		}

		// Vehicle uncertainty
		KFMatrix_VxV  Pxx(UNINITIALIZED_MATRIX  );
		m_pkk.extractMatrix(0,0, Pxx);

		// Build predictions:
		// ---------------------------
		const size_t nPredictions = lm_indices_in_S.size();
		m_last_data_association.clear();

		// S is the covariance of the predictions:
		m_last_data_association.Y_pred_covs = S;

		// The means:
		m_last_data_association.Y_pred_means.setSize(nPredictions, obs_size);
		for (size_t q=0;q<nPredictions;q++)
		{
			const size_t i = lm_indices_in_S[q];
			for (size_t w=0;w<obs_size;w++)
				m_last_data_association.Y_pred_means.get_unsafe(q,w) = all_predictions[i][w];
			m_last_data_association.predictions_IDs.push_back( i ); // for the conversion of indices...
		}

		// Do Dat. Assoc :
		// ---------------------------
		if (nPredictions)
		{
			CMatrixDouble  Z_obs_cov = CMatrixDouble(R);

			mrpt::slam::data_association_full_covariance(
				Z_obs_means, //Z_obs_cov,
				m_last_data_association.Y_pred_means,m_last_data_association.Y_pred_covs,
				m_last_data_association.results,
				options.data_assoc_method,
				options.data_assoc_metric,
				options.data_assoc_IC_chi2_thres,
				true,   // Use KD-tree
				m_last_data_association.predictions_IDs,
				options.data_assoc_IC_metric,
				options.data_assoc_IC_ml_threshold
				);

			// Return pairings to the main KF algorithm:
			for (map<size_t,size_t>::const_iterator it= m_last_data_association.results.associations.begin();it!=m_last_data_association.results.associations.end();++it)
				data_association[ it->first ] = it->second;
		}
	}
	// ---- End of data association ----

	MRPT_END
}


/** This virtual function musts normalize the state vector and covariance matrix (only if its necessary).
 */
void  CRangeBearingKFSLAM2D::OnNormalizeStateVector()
{
	// Check angles:
	mrpt::math::wrapToPiInPlace(m_xkk[2]);

}

/*---------------------------------------------------------------
						loadOptions
  ---------------------------------------------------------------*/
void CRangeBearingKFSLAM2D::loadOptions( const mrpt::utils::CConfigFileBase &ini )
{
	// Main
	options.loadFromConfigFile( ini, "RangeBearingKFSLAM" );
	KF_options.loadFromConfigFile( ini, "RangeBearingKFSLAM_KalmanFilter" );
}

/*---------------------------------------------------------------
						loadOptions
  ---------------------------------------------------------------*/
void CRangeBearingKFSLAM2D::TOptions::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase	&source,
	const std::string		&section)
{
	stds_Q_no_odo[2] = RAD2DEG(stds_Q_no_odo[2]);

	source.read_vector(section,"stds_Q_no_odo", stds_Q_no_odo, stds_Q_no_odo );
	ASSERT_(stds_Q_no_odo.size()==3)

	stds_Q_no_odo[2] = DEG2RAD(stds_Q_no_odo[2]);

	std_sensor_range	= source.read_float(section,"std_sensor_range", std_sensor_range);
	std_sensor_yaw		= DEG2RAD( source.read_float(section,"std_sensor_yaw_deg", RAD2DEG(std_sensor_yaw)));

	MRPT_LOAD_CONFIG_VAR(quantiles_3D_representation, float, source,section);
	MRPT_LOAD_CONFIG_VAR(create_simplemap, bool, source,section);

	data_assoc_method    = source.read_enum<TDataAssociationMethod>(section,"data_assoc_method",data_assoc_method);
	data_assoc_metric    = source.read_enum<TDataAssociationMetric>(section,"data_assoc_metric",data_assoc_metric);
	data_assoc_IC_metric = source.read_enum<TDataAssociationMetric>(section,"data_assoc_IC_metric",data_assoc_IC_metric);

	MRPT_LOAD_CONFIG_VAR(data_assoc_IC_chi2_thres,double,  source, section);
	MRPT_LOAD_CONFIG_VAR(data_assoc_IC_ml_threshold,double,  source, section);
}

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CRangeBearingKFSLAM2D::TOptions::TOptions() :
	stds_Q_no_odo(3,0),
	std_sensor_range ( 0.1f),
	std_sensor_yaw   ( DEG2RAD( 0.5f )),
	quantiles_3D_representation ( 3),
	create_simplemap			(false),
	data_assoc_method			(assocNN),
	data_assoc_metric			(metricMaha),
	data_assoc_IC_chi2_thres	(0.99),
	data_assoc_IC_metric		(metricMaha),
	data_assoc_IC_ml_threshold	(0.0)

{
	stds_Q_no_odo[0]=0.10f;
	stds_Q_no_odo[1]=0.10f;
	stds_Q_no_odo[2]=DEG2RAD(4.0f);
}

/*---------------------------------------------------------------
						dumpToTextStream
  ---------------------------------------------------------------*/
void CRangeBearingKFSLAM2D::TOptions::dumpToTextStream( CStream &out)const
{
	out.printf("\n----------- [CRangeBearingKFSLAM2D::TOptions] ------------ \n\n");

	out.printf("data_assoc_method                       = %s\n", TEnumType<TDataAssociationMethod>::value2name(data_assoc_method).c_str() );
	out.printf("data_assoc_metric                       = %s\n", TEnumType<TDataAssociationMetric>::value2name(data_assoc_metric).c_str() );
	out.printf("data_assoc_IC_chi2_thres                = %.06f\n", data_assoc_IC_chi2_thres );
	out.printf("data_assoc_IC_metric                    = %s\n", TEnumType<TDataAssociationMetric>::value2name(data_assoc_IC_metric).c_str() );
	out.printf("data_assoc_IC_ml_threshold              = %.06f\n", data_assoc_IC_ml_threshold );

	out.printf("\n");
}

void CRangeBearingKFSLAM2D::OnInverseObservationModel(
	const KFArray_OBS & in_z,
	KFArray_FEAT  & yn,
	KFMatrix_FxV  & dyn_dxv,
	KFMatrix_FxO  & dyn_dhn ) const
{
	MRPT_START

	/* -------------------------------------------
       Equations, obtained using matlab

        x0 y0 phi0      % Robot's 2D pose
        x0s y0s phis    % Sensor's 2D pose relative to robot
        hr ha         	% Observation hn: range, yaw

        xi yi           % Absolute 2D landmark coordinates:

        dyn_dxv   -> Jacobian of the inv. observation model wrt the robot pose
        dyn_dhn   -> Jacobian of the inv. observation model wrt each landmark observation

        Sizes:
	     hn:      1x2  <--
         yn:      1x2  -->
         dyn_dxv: 2x3  -->
         dyn_dhn: 2x2  -->
	  ------------------------------------------- */

	CObservationBearingRangePtr obs = m_SF->getObservationByClass<CObservationBearingRange>();
	ASSERTMSG_(obs,"*ERROR*: This method requires an observation of type CObservationBearingRange")
	const CPose2D   sensorPoseOnRobot = CPose2D( obs->sensorLocationOnRobot );

	// Mean of the prior of the robot pose:
	const TPose2D  robotPose( m_xkk[0],m_xkk[1],m_xkk[2] );

    // Robot 2D pose:
    const kftype x0 = m_xkk[0];
    const kftype y0 = m_xkk[1];
    const kftype phi0  = m_xkk[2];

    const kftype cphi0 = cos(phi0);
    const kftype sphi0 = sin(phi0);

    // Sensor 6D pose on robot:
    const kftype x0s = sensorPoseOnRobot.x();
    const kftype y0s = sensorPoseOnRobot.y();
    const kftype phis = sensorPoseOnRobot.phi();

    const kftype hr = in_z[0];
    const kftype ha = in_z[1];

    const kftype cphi_0sa = cos(phi0+phis+ha);
    const kftype sphi_0sa = sin(phi0+phis+ha);

    // Compute the mean 2D absolute coordinates:
    yn[0] = hr*cphi_0sa+cphi0*x0s-sphi0*y0s+x0;
    yn[1] = hr*sphi_0sa+sphi0*x0s+cphi0*y0s+y0;

    // Jacobian wrt xv:
    dyn_dxv.get_unsafe(0,0) = 1;
    dyn_dxv.get_unsafe(0,1) = 0;
    dyn_dxv.get_unsafe(0,2) = -hr*sphi_0sa-sphi0*x0s-cphi0*y0s;

    dyn_dxv.get_unsafe(1,0) = 0;
    dyn_dxv.get_unsafe(1,1) = 1;
    dyn_dxv.get_unsafe(1,2) = hr*cphi_0sa+cphi0*x0s-sphi0*y0s;

    // Jacobian wrt hn:
    dyn_dhn.get_unsafe(0,0) = cphi_0sa;
    dyn_dhn.get_unsafe(0,1) = -hr*sphi_0sa;

    dyn_dhn.get_unsafe(1,0) = sphi_0sa;
    dyn_dhn.get_unsafe(1,1) = hr*cphi_0sa;

	MRPT_END
}


/** If applicable to the given problem, do here any special handling of adding a new landmark to the map.
  * \param in_obsIndex The index of the observation whose inverse sensor is to be computed. It corresponds to the row in in_z where the observation can be found.
  * \param in_idxNewFeat The index that this new feature will have in the state vector (0:just after the vehicle state, 1: after that,...). Save this number so data association can be done according to these indices.
  * \sa OnInverseObservationModel
  */
void CRangeBearingKFSLAM2D::OnNewLandmarkAddedToMap(
	const size_t	in_obsIdx,
	const size_t	in_idxNewFeat )
{
	MRPT_START

	CObservationBearingRangePtr obs = m_SF->getObservationByClass<CObservationBearingRange>();
	ASSERTMSG_(obs,"*ERROR*: This method requires an observation of type CObservationBearingRange")

	// ----------------------------------------------
	// introduce in the lists of ID<->index in map:
	// ----------------------------------------------
	ASSERT_( in_obsIdx < obs->sensedData.size() )

	if (obs->sensedData[in_obsIdx].landmarkID>=0)
	{
		// The sensor provides us a LM ID... use it:
		m_IDs.insert(
			obs->sensedData[in_obsIdx].landmarkID,
			in_idxNewFeat );
	}
	else
	{
		// Features do not have IDs... use indices:
		m_IDs.insert( in_idxNewFeat,in_idxNewFeat);
	}

	m_last_data_association.newly_inserted_landmarks[in_obsIdx] = in_idxNewFeat; // Just for stats, etc...

    MRPT_END
}


/*---------------------------------------------------------------
						getAs3DObject
  ---------------------------------------------------------------*/
void  CRangeBearingKFSLAM2D::getAs3DObject( mrpt::opengl::CSetOfObjectsPtr	&outObj ) const
{
    outObj->clear();

	// ------------------------------------------------
	//  Add the XYZ corner for the current area:
	// ------------------------------------------------
	outObj->insert( opengl::stock_objects::CornerXYZ() );


	// 2D ellipsoid for robot pose:
	CPoint2DPDFGaussian pointGauss;
    pointGauss.mean.x( m_xkk[0] );
    pointGauss.mean.y( m_xkk[1] );
    CMatrixTemplateNumeric<kftype>  COV;
    m_pkk.extractMatrix(0,0,2,2, COV);
    pointGauss.cov = COV;

    {
		opengl::CEllipsoidPtr ellip = opengl::CEllipsoid::Create();

		ellip->setPose(pointGauss.mean);
		ellip->setCovMatrix(pointGauss.cov);
		ellip->enableDrawSolid3D(false);
		ellip->setQuantiles( options.quantiles_3D_representation );
		ellip->set3DsegmentsCount(10);
		ellip->setColor(1,0,0);

		outObj->insert( ellip );
    }


	// 2D ellipsoids for landmarks:
	const size_t nLMs = (m_xkk.size()-3)/2;
	for (size_t i=0;i<nLMs;i++)
	{
        pointGauss.mean.x( m_xkk[3+2*i+0] );
        pointGauss.mean.y( m_xkk[3+2*i+1] );
        m_pkk.extractMatrix(3+2*i,3+2*i,2,2, COV);
        pointGauss.cov = COV;

		opengl::CEllipsoidPtr ellip = opengl::CEllipsoid::Create();

		ellip->setName( format( "%u",static_cast<unsigned int>(i) ) );
		ellip->enableShowName(true);
		ellip->setPose( pointGauss.mean );
		ellip->setCovMatrix( pointGauss.cov );
		ellip->enableDrawSolid3D(false);
		ellip->setQuantiles( options.quantiles_3D_representation );
		ellip->set3DsegmentsCount(10);

		ellip->setColor(0,0,1);

		outObj->insert( ellip );
	}
}



/*---------------------------------------------------------------
              saveMapAndPathRepresentationAsMATLABFile
  ---------------------------------------------------------------*/
void CRangeBearingKFSLAM2D::saveMapAndPath2DRepresentationAsMATLABFile(
	const string &fil,
	float stdCount,
	const string &styleLandmarks,
	const string &stylePath,
	const string &styleRobot ) const
{
	FILE	*f= os::fopen(fil.c_str(),"wt");
	if (!f) return;

	CMatrixDouble cov(2,2);
	CVectorDouble mean(2);

	// Header:
	os::fprintf(f,"%%--------------------------------------------------------------------\n");
	os::fprintf(f,"%% File automatically generated using the MRPT method:\n");
	os::fprintf(f,"%% 'CRangeBearingKFSLAM2D::saveMapAndPath2DRepresentationAsMATLABFile'\n");
	os::fprintf(f,"%%\n");
	os::fprintf(f,"%%                        ~ MRPT ~\n");
	os::fprintf(f,"%%  Jose Luis Blanco Claraco, University of Malaga @ 2008\n");
	os::fprintf(f,"%%      http://www.mrpt.org/     \n");
	os::fprintf(f,"%%--------------------------------------------------------------------\n");

	// Main code:
	os::fprintf(f,"hold on;\n\n");

	size_t i, nLMs = (m_xkk.size()-get_vehicle_size())/get_feature_size();

	for (i=0;i<nLMs;i++)
	{
		size_t idx = get_vehicle_size()+i*get_feature_size();

		cov(0,0) = m_pkk(idx+0,idx+0);
		cov(1,1) = m_pkk(idx+1,idx+1);
		cov(0,1) = cov(1,0) = m_pkk(idx+0,idx+1);

		mean[0] = m_xkk[idx+0];
		mean[1] = m_xkk[idx+1];

		// Command to draw the 2D ellipse:
		os::fprintf(f, "%s", math::MATLAB_plotCovariance2D( cov, mean, stdCount,styleLandmarks ).c_str() );
	}


	// Now: the robot path:
	// ------------------------------
	if (m_SFs.size())
	{
		os::fprintf(f,"\nROB_PATH=[");
		for (i=0;i<m_SFs.size();i++)
		{
			CSensoryFramePtr dummySF;
			CPose3DPDFPtr pdf3D;
			m_SFs.get(i,pdf3D,dummySF);

			CPose3D p;
			pdf3D->getMean(p);
			CPoint3D pnt3D(p); // 6D -> 3D only

			os::fprintf(f,"%.04f %.04f", pnt3D.x(), pnt3D.y() );
			if (i<(m_SFs.size()-1))
				os::fprintf(f,";");
		}
		os::fprintf(f,"];\n");

		os::fprintf(f,"plot(ROB_PATH(:,1),ROB_PATH(:,2),'%s');\n",stylePath.c_str());
	}

	// The robot pose:
	cov(0,0) = m_pkk(0,0);
	cov(1,1) = m_pkk(1,1);
	cov(0,1) = cov(1,0) = m_pkk(0,1);

	mean[0] = m_xkk[0];
	mean[1] = m_xkk[1];

	os::fprintf(f, "%s", math::MATLAB_plotCovariance2D( cov, mean, stdCount, styleRobot ).c_str() );

	os::fprintf(f,"\naxis equal;\n");
	os::fclose(f);
}


/** Computes A=A-B, which may need to be re-implemented depending on the topology of the individual scalar components (eg, angles).
  */
void CRangeBearingKFSLAM2D::OnSubstractObservationVectors(KFArray_OBS &A, const KFArray_OBS &B) const
{
	A[0]-=B[0];
	A[1]-=B[1];
	mrpt::math::wrapToPiInPlace(A[1]);
}


/** Return the observation NOISE covariance matrix, that is, the model of the Gaussian additive noise of the sensor.
  * \param out_R The noise covariance matrix. It might be non diagonal, but it'll usually be.
  */
void CRangeBearingKFSLAM2D::OnGetObservationNoise(KFMatrix_OxO &out_R) const
{
	out_R(0,0)= square( options.std_sensor_range );
	out_R(1,1)= square( options.std_sensor_yaw );
}

/** This will be called before OnGetObservationsAndDataAssociation to allow the application to reduce the number of covariance landmark predictions to be made.
  *  For example, features which are known to be "out of sight" shouldn't be added to the output list to speed up the calculations.
  * \param in_all_prediction_means The mean of each landmark predictions; the computation or not of the corresponding covariances is what we're trying to determined with this method.
  * \param out_LM_indices_to_predict The list of landmark indices in the map [0,getNumberOfLandmarksInTheMap()-1] that should be predicted.
  * \note This is not a pure virtual method, so it should be implemented only if desired. The default implementation returns a vector with all the landmarks in the map.
  * \sa OnGetObservations, OnDataAssociation
  */
void CRangeBearingKFSLAM2D::OnPreComputingPredictions(
	const vector_KFArray_OBS	&prediction_means,
	vector_size_t				&out_LM_indices_to_predict ) const
{
	CObservationBearingRangePtr obs = m_SF->getObservationByClass<CObservationBearingRange>();
	ASSERTMSG_(obs,"*ERROR*: This method requires an observation of type CObservationBearingRange")

	const double sensor_max_range = obs->maxSensorDistance;
	const double fov_yaw   = obs->fieldOfView_yaw;

	const double max_vehicle_loc_uncertainty = 4 * std::sqrt( m_pkk.get_unsafe(0,0) + m_pkk.get_unsafe(1,1) );
	const double max_vehicle_ang_uncertainty = 4 * std::sqrt( m_pkk.get_unsafe(2,2) );

	out_LM_indices_to_predict.clear();
	for (size_t i=0;i<prediction_means.size();i++)
#if (!STATS_EXPERIMENT)  // In the experiment we force errors too far and some predictions are out of range, so just generate all of them:
		if (      prediction_means[i][0] < (       1.5  + sensor_max_range + max_vehicle_loc_uncertainty + 4*options.std_sensor_range) &&
		    fabs(prediction_means[i][1]) < (DEG2RAD(20) + 0.5*fov_yaw      + max_vehicle_ang_uncertainty + 4*options.std_sensor_yaw)
			)
#endif
		{
			out_LM_indices_to_predict.push_back(i);
		}
}


/** Only called if using a numeric approximation of the transition Jacobian, this method must return the increments in each dimension of the vehicle state vector while estimating the Jacobian.
  */
void CRangeBearingKFSLAM2D::OnTransitionJacobianNumericGetIncrements(KFArray_VEH &out_increments) const
{
	for (size_t i=0;i<get_vehicle_size();i++) out_increments[i] = 1e-6;
}


/** Only called if using a numeric approximation of the observation Jacobians, this method must return the increments in each dimension of the vehicle state vector while estimating the Jacobian.
  */
void CRangeBearingKFSLAM2D::OnObservationJacobiansNumericGetIncrements(
		KFArray_VEH  &out_veh_increments,
		KFArray_FEAT &out_feat_increments ) const
{
	for (size_t i=0;i<get_vehicle_size();i++) out_veh_increments[i] = 1e-6;
	for (size_t i=0;i<get_feature_size();i++) out_feat_increments[i] = 1e-6;
}
