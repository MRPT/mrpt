/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "slam-precomp.h"   // Precompiled headers

// ----------------------------------------------------------------------------------------
// For the theory behind this implementation, see the technical report in:
//
//            http://www.mrpt.org/6D-SLAM
// ----------------------------------------------------------------------------------------

#include <mrpt/slam/CRangeBearingKFSLAM.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPose3DQuatPDFGaussian.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/math/utils.h>
#include <mrpt/math/ops_containers.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/system/os.h>

#include <mrpt/opengl/stock_objects.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CEllipsoid.h>

using namespace mrpt::slam;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace mrpt;
using namespace std;


/*---------------------------------------------------------------
							Constructor
  ---------------------------------------------------------------*/
CRangeBearingKFSLAM::CRangeBearingKFSLAM( ) :
	options(),
	m_action(),m_SF(),
	m_IDs(),
	mapPartitioner(),
	m_SFs(),
	m_lastPartitionSet()
{
	reset();
}

/*---------------------------------------------------------------
							reset
  ---------------------------------------------------------------*/
void CRangeBearingKFSLAM::reset( )
{
	m_action.clear_unique();
	m_SF.clear_unique();
	m_IDs.clear();
	m_SFs.clear();
	mapPartitioner.clear();
	m_lastPartitionSet.clear();

	// -----------------------
	// INIT KF STATE
	m_xkk.assign(get_vehicle_size(),0);	// State: 6D pose (x,y,z)=(0,0,0)
	m_xkk[3]=1.0;  // (qr,qx,qy,qz)=(1,0,0,0)

	// Initial cov:  NULL diagonal -> perfect knowledge.
	m_pkk.setSize(get_vehicle_size(),get_vehicle_size());
	m_pkk.zeros();
	// -----------------------

	// Use SF-based matching (faster & easier for bearing-range observations with ID).
	mapPartitioner.options.useMapMatching  = false;
}

/*---------------------------------------------------------------
							Destructor
  ---------------------------------------------------------------*/
CRangeBearingKFSLAM::~CRangeBearingKFSLAM()
{

}

/*---------------------------------------------------------------
							getCurrentRobotPose
  ---------------------------------------------------------------*/
void CRangeBearingKFSLAM::getCurrentRobotPose(
	CPose3DQuatPDFGaussian      &out_robotPose ) const
{
	MRPT_START

	ASSERT_(m_xkk.size()>=7);

	// Copy xyz+quat: (explicitly unroll the loop)
	out_robotPose.mean.m_coords[0] = m_xkk[0];
	out_robotPose.mean.m_coords[1] = m_xkk[1];
	out_robotPose.mean.m_coords[2] = m_xkk[2];
	out_robotPose.mean.m_quat  [0] = m_xkk[3];
	out_robotPose.mean.m_quat  [1] = m_xkk[4];
	out_robotPose.mean.m_quat  [2] = m_xkk[5];
	out_robotPose.mean.m_quat  [3] = m_xkk[6];

	// and cov:
	m_pkk.extractMatrix(0,0,out_robotPose.cov);

	MRPT_END
}

/*---------------------------------------------------------------
							getCurrentRobotPoseMean
  ---------------------------------------------------------------*/
mrpt::poses::CPose3DQuat CRangeBearingKFSLAM::getCurrentRobotPoseMean() const
{
	CPose3DQuat  q(mrpt::math::UNINITIALIZED_QUATERNION);
	ASSERTDEB_(m_xkk.size()>=7)
	// Copy xyz+quat: (explicitly unroll the loop)
	q.m_coords[0] = m_xkk[0];
	q.m_coords[1] = m_xkk[1];
	q.m_coords[2] = m_xkk[2];
	q.m_quat  [0] = m_xkk[3];
	q.m_quat  [1] = m_xkk[4];
	q.m_quat  [2] = m_xkk[5];
	q.m_quat  [3] = m_xkk[6];

	return q;
}


/*---------------------------------------------------------------
							getCurrentState
  ---------------------------------------------------------------*/
void CRangeBearingKFSLAM::getCurrentState(
	CPose3DQuatPDFGaussian      &out_robotPose,
	std::vector<TPoint3D>  &out_landmarksPositions,
	std::map<unsigned int,CLandmark::TLandmarkID> &out_landmarkIDs,
	CVectorDouble      &out_fullState,
	CMatrixDouble      &out_fullCovariance ) const
{
	MRPT_START

	ASSERT_(size_t(m_xkk.size())>=get_vehicle_size());

	// Copy xyz+quat: (explicitly unroll the loop)
	out_robotPose.mean.m_coords[0] = m_xkk[0];
	out_robotPose.mean.m_coords[1] = m_xkk[1];
	out_robotPose.mean.m_coords[2] = m_xkk[2];
	out_robotPose.mean.m_quat  [0] = m_xkk[3];
	out_robotPose.mean.m_quat  [1] = m_xkk[4];
	out_robotPose.mean.m_quat  [2] = m_xkk[5];
	out_robotPose.mean.m_quat  [3] = m_xkk[6];

	// and cov:
	m_pkk.extractMatrix(0,0,out_robotPose.cov);

	// Landmarks:
	ASSERT_( ((m_xkk.size() - get_vehicle_size()) % get_feature_size())==0 );
	size_t i,nLMs = (m_xkk.size() - get_vehicle_size()) / get_feature_size();
	out_landmarksPositions.resize(nLMs);
	for (i=0;i<nLMs;i++)
	{
		out_landmarksPositions[i].x = m_xkk[get_vehicle_size()+i*get_feature_size()+0];
		out_landmarksPositions[i].y = m_xkk[get_vehicle_size()+i*get_feature_size()+1];
		out_landmarksPositions[i].z = m_xkk[get_vehicle_size()+i*get_feature_size()+2];
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
void  CRangeBearingKFSLAM::processActionObservation(
    CActionCollectionPtr	&action,
    CSensoryFramePtr		&SF )
{
	MRPT_START

	m_action = action;
	m_SF = SF;

	// Sanity check:
	ASSERT_( m_IDs.size() == (m_pkk.getColCount() - get_vehicle_size() )/get_feature_size() );

	// ===================================================================================================================
	// Here's the meat!: Call the main method for the KF algorithm, which will call all the callback methods as required:
	// ===================================================================================================================
	runOneKalmanIteration();


	// =============================================================
	//  ADD TO SFs SEQUENCE
	// =============================================================
	CPose3DQuatPDFGaussian  q(UNINITIALIZED_QUATERNION);
	this->getCurrentRobotPose(q);
	CPose3DPDFGaussianPtr	auxPosePDF = CPose3DPDFGaussianPtr(new CPose3DPDFGaussian(q));

	if (options.create_simplemap)
	{
		m_SFs.insert( CPose3DPDFPtr(auxPosePDF) , SF );
	}

	// =============================================================
	//  UPDATE THE PARTITION GRAPH EXPERIMENT
	// =============================================================
	if (options.doPartitioningExperiment)
	{
	    if (options.partitioningMethod==0)
	    {
	        // Use spectral-graph technique:
            mapPartitioner.addMapFrame( SF, auxPosePDF );

            vector<vector_uint>   partitions;
            mapPartitioner.updatePartitions( partitions );
            m_lastPartitionSet = partitions;
	    }
	    else
	    {
	        // Fixed partitions every K observations:
	        vector<vector_uint>   partitions;
	        vector_uint           tmpCluster;

	        ASSERT_(options.partitioningMethod>1);
	        size_t    K = options.partitioningMethod;

	        for (size_t i=0;i<m_SFs.size();i++)
	        {
	            tmpCluster.push_back(i);
	            if ( (i % K) == 0 )
	            {
	                partitions.push_back(tmpCluster);
	                tmpCluster.clear();
	                tmpCluster.push_back(i);  // This observation "i" is shared between both clusters
	            }
	        }
            m_lastPartitionSet = partitions;
	    }

		printf("Partitions: %u\n", static_cast<unsigned>(m_lastPartitionSet.size() ));
	}

    MRPT_END
}

/** Return the last odometry, as a pose increment.
  */
CPose3DQuat CRangeBearingKFSLAM::getIncrementFromOdometry() const
{
	CActionRobotMovement2DPtr actMov2D = m_action->getBestMovementEstimation();
	CActionRobotMovement3DPtr actMov3D = m_action->getActionByClass<CActionRobotMovement3D>();
	if (actMov3D && !options.force_ignore_odometry)
	{
		return CPose3DQuat( actMov3D->poseChange.mean );
	}
	else if (actMov2D && !options.force_ignore_odometry)
	{
		CPose2D estMovMean;
		actMov2D->poseChange->getMean(estMovMean);
		return CPose3DQuat( CPose3D(estMovMean) );
	}
	else
	{
		return CPose3DQuat();
	}
}

/** Must return the action vector u.
  * \param out_u The action vector which will be passed to OnTransitionModel
  */
void CRangeBearingKFSLAM::OnGetAction( KFArray_ACT &u ) const
{
	// Get odometry estimation:
	const CPose3DQuat theIncr = getIncrementFromOdometry();

	for (KFArray_ACT::Index i=0;i<u.size();i++)
		u[i] = theIncr[i];
}

/** This virtual function musts implement the prediction model of the Kalman filter.
 */
void  CRangeBearingKFSLAM::OnTransitionModel(
	const KFArray_ACT &u,
	KFArray_VEH       &xv,
	bool &out_skipPrediction ) const
{
	MRPT_START

	// Do not update the vehicle pose & its covariance until we have some landmarks in the map,
	// otherwise, we are imposing a lower bound to the best uncertainty from now on:
	if (size_t(m_xkk.size()) == get_vehicle_size() )
	{
		out_skipPrediction = true;
	}

	// Current pose: copy xyz+quat
	CPose3DQuat	robotPose = getCurrentRobotPoseMean();

	// Increment pose: copy xyz+quat (explicitly unroll the loop)
	CPose3DQuat	odoIncrement(UNINITIALIZED_QUATERNION);
	odoIncrement.m_coords[0] = u[0];
	odoIncrement.m_coords[1] = u[1];
	odoIncrement.m_coords[2] = u[2];
	odoIncrement.m_quat  [0] = u[3];
	odoIncrement.m_quat  [1] = u[4];
	odoIncrement.m_quat  [2] = u[5];
	odoIncrement.m_quat  [3] = u[6];

	// Pose composition:
	robotPose += odoIncrement;

	// Output:
	for (size_t i=0;i<xv.static_size;i++)
		xv[i] = robotPose[i];

	MRPT_END
}

/** This virtual function musts calculate the Jacobian F of the prediction model.
 */
void  CRangeBearingKFSLAM::OnTransitionJacobian( KFMatrix_VxV  &F ) const
{
	MRPT_START

	// Current pose: copy xyz+quat
	CPose3DQuat	robotPose = getCurrentRobotPoseMean();

	// Odometry:
	const CPose3DQuat theIncr = getIncrementFromOdometry();

	// Compute jacobians:
	CMatrixDouble77 df_du(UNINITIALIZED_MATRIX);

	CPose3DQuatPDF::jacobiansPoseComposition(
		robotPose,  // x
		theIncr,       // u
		F,   // df_dx,
		df_du);

	MRPT_END
}

/** This virtual function musts calculate de noise matrix of the prediction model.
 */
void  CRangeBearingKFSLAM::OnTransitionNoise( KFMatrix_VxV &Q ) const
{
	MRPT_START

	// The uncertainty of the 2D odometry, projected from the current position:
	CActionRobotMovement2DPtr act2D = m_action->getBestMovementEstimation();
	CActionRobotMovement3DPtr act3D = m_action->getActionByClass<CActionRobotMovement3D>();

	if (act3D && act2D)  THROW_EXCEPTION("Both 2D & 3D odometry are present!?!?")

	CPose3DQuatPDFGaussian odoIncr;

	if ((!act3D && !act2D) || options.force_ignore_odometry)
	{
		// Use constant Q:
		Q.zeros();
		ASSERT_(size_t(options.stds_Q_no_odo.size())==size_t(Q.getColCount()))

		for (size_t i=0;i<get_vehicle_size();i++)
			Q.set_unsafe(i,i, square( options.stds_Q_no_odo[i]));
		return;
	}
	else
	{
		if (act2D)
		{
			// 2D odometry:
			CPosePDFGaussian odoIncr2D;
			odoIncr2D.copyFrom( *act2D->poseChange );
			odoIncr = CPose3DQuatPDFGaussian( CPose3DPDFGaussian(odoIncr2D) );
		}
		else
		{
			// 3D odometry:
			odoIncr = CPose3DQuatPDFGaussian( act3D->poseChange );
		}
	}

	odoIncr.cov(2,2) +=  square(options.std_odo_z_additional);

	// Current pose: copy xyz+quat
	CPose3DQuat	robotPose = getCurrentRobotPoseMean();

	// Transform from odometry increment to "relative to the robot":
	odoIncr.changeCoordinatesReference( robotPose );

	Q = odoIncr.cov;

	MRPT_END
}


void CRangeBearingKFSLAM::OnObservationModel(
	const vector_size_t       &idx_landmarks_to_predict,
	vector_KFArray_OBS  &out_predictions ) const
{
	MRPT_START

	// Mean of the prior of the robot pose:
	CPose3DQuat	robotPose = getCurrentRobotPoseMean();

	// Get the sensor pose relative to the robot:
	CObservationBearingRangePtr obs = m_SF->getObservationByClass<CObservationBearingRange>();
	ASSERTMSG_(obs,"*ERROR*: This method requires an observation of type CObservationBearingRange")
	const CPose3DQuat sensorPoseOnRobot = CPose3DQuat(obs->sensorLocationOnRobot);

	/* -------------------------------------------
       Equations, obtained using matlab, of the relative 3D position of a landmark (xi,yi,zi), relative
          to a robot 6D pose (x0,y0,z0,y,p,r)
        Refer to technical report "6D EKF derivation...", 2008

        x0 y0 z0 y p r         % Robot's 6D pose
        x0s y0s z0s ys ps rs   % Sensor's 6D pose relative to robot
        xi yi zi % Absolute 3D landmark coordinates:

        Hx : dh_dxv   -> Jacobian of the observation model wrt the robot pose
        Hy : dh_dyi   -> Jacobian of the observation model wrt each landmark mean position

        Sizes:
	     h:  Lx3
         Hx: 3Lx6
         Hy: 3Lx3

          L=# of landmarks in the map (ALL OF THEM)
	  ------------------------------------------- */

    const size_t  vehicle_size = get_vehicle_size();
    //const size_t  obs_size  = get_observation_size();
    const size_t  feature_size = get_feature_size();

	const CPose3DQuat  sensorPoseAbs= robotPose + sensorPoseOnRobot;

	const size_t N = idx_landmarks_to_predict.size();
	out_predictions.resize(N);
	for (size_t i=0;i<N;i++)
	{
		const size_t row_in = feature_size*idx_landmarks_to_predict[i];

		// Landmark absolute 3D position in the map:
		const TPoint3D  mapEst(
			m_xkk[ vehicle_size + row_in + 0 ],
			m_xkk[ vehicle_size + row_in + 1 ],
			m_xkk[ vehicle_size + row_in + 2 ] );

		// Generate Range, yaw, pitch
		// ---------------------------------------------------
		sensorPoseAbs.sphericalCoordinates(
			mapEst,
			out_predictions[i][0], // range
			out_predictions[i][1], // yaw
			out_predictions[i][2]  // pitch
			);
	}

	MRPT_END
}

void CRangeBearingKFSLAM::OnObservationJacobians(
	const size_t &idx_landmark_to_predict,
	KFMatrix_OxV &Hx,
	KFMatrix_OxF &Hy ) const
{
	MRPT_START

	// Mean of the prior of the robot pose:
	const CPose3DQuat	robotPose = getCurrentRobotPoseMean();

	// Get the sensor pose relative to the robot:
	CObservationBearingRangePtr obs = m_SF->getObservationByClass<CObservationBearingRange>();
	ASSERTMSG_(obs,"*ERROR*: This method requires an observation of type CObservationBearingRange")
	const CPose3DQuat sensorPoseOnRobot = CPose3DQuat(obs->sensorLocationOnRobot);

    const size_t  vehicle_size = get_vehicle_size();
    //const size_t  obs_size  = get_observation_size();
    const size_t  feature_size= get_feature_size();

	// Compute the jacobians, needed below:
	//const CPose3DQuat  sensorPoseAbs= robotPose + sensorPoseOnRobot;
	CPose3DQuat  sensorPoseAbs(UNINITIALIZED_QUATERNION);
	CMatrixFixedNumeric<kftype,7,7>  H_senpose_vehpose(UNINITIALIZED_MATRIX);
	CMatrixFixedNumeric<kftype,7,7>  H_senpose_senrelpose(UNINITIALIZED_MATRIX);  // Not actually used

	CPose3DQuatPDF::jacobiansPoseComposition(
		robotPose,
		sensorPoseOnRobot,
		H_senpose_vehpose,
		H_senpose_senrelpose,
		&sensorPoseAbs);

	const size_t row_in = feature_size*idx_landmark_to_predict;

	// Landmark absolute 3D position in the map:
	const TPoint3D  mapEst(
		m_xkk[ vehicle_size + row_in + 0 ],
		m_xkk[ vehicle_size + row_in + 1 ],
		m_xkk[ vehicle_size + row_in + 2 ] );

	// The Jacobian wrt the sensor pose must be transformed later on:
	KFMatrix_OxV Hx_sensor;
	double obsData[3];
	sensorPoseAbs.sphericalCoordinates(
		mapEst,
		obsData[0], // range
		obsData[1], // yaw
		obsData[2],  // pitch
		&Hy,
		&Hx_sensor
		);

	// Chain rule: Hx = d sensorpose / d vehiclepose   * Hx_sensor
	Hx.multiply( Hx_sensor, H_senpose_vehpose );

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
void CRangeBearingKFSLAM::OnGetObservationsAndDataAssociation(
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
		Z[row][2] =itObs->pitch;
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
void  CRangeBearingKFSLAM::OnNormalizeStateVector()
{
	MRPT_START

	// m_xkk[3:6] must be a normalized quaternion:
	const double T = std::sqrt( square(m_xkk[3])+square(m_xkk[4])+square(m_xkk[5])+square(m_xkk[6]) );
	ASSERTMSG_(T>0,"Vehicle pose quaternion norm is not >0!!")

	const double T_ = (m_xkk[3]<0 ? -1.0:1.0)/T;  // qr>=0
	m_xkk[3]*=T_;
	m_xkk[4]*=T_;
	m_xkk[5]*=T_;
	m_xkk[6]*=T_;

	MRPT_END
}

/*---------------------------------------------------------------
						loadOptions
  ---------------------------------------------------------------*/
void CRangeBearingKFSLAM::loadOptions( const mrpt::utils::CConfigFileBase &ini )
{
	// Main
	options.loadFromConfigFile( ini, "RangeBearingKFSLAM" );
	KF_options.loadFromConfigFile( ini, "RangeBearingKFSLAM_KalmanFilter" );
	// partition algorithm:
	mapPartitioner.options.loadFromConfigFile(ini,"RangeBearingKFSLAM");
}

/*---------------------------------------------------------------
						loadOptions
  ---------------------------------------------------------------*/
void CRangeBearingKFSLAM::TOptions::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase	&source,
	const std::string		&section)
{
	source.read_vector(section,"stds_Q_no_odo", stds_Q_no_odo, stds_Q_no_odo );
	ASSERT_(stds_Q_no_odo.size()==7)

	std_sensor_range	= source.read_float(section,"std_sensor_range", std_sensor_range);
	std_sensor_yaw		= DEG2RAD( source.read_float(section,"std_sensor_yaw_deg", RAD2DEG(std_sensor_yaw)));
	std_sensor_pitch	= DEG2RAD( source.read_float(section,"std_sensor_pitch_deg", RAD2DEG(std_sensor_pitch) ));

	std_odo_z_additional = source.read_float(section,"std_odo_z_additional", std_odo_z_additional);

	MRPT_LOAD_CONFIG_VAR(doPartitioningExperiment,bool, source,section);
	MRPT_LOAD_CONFIG_VAR(partitioningMethod,int,  source,section);

	MRPT_LOAD_CONFIG_VAR(create_simplemap,bool, source,section);

	MRPT_LOAD_CONFIG_VAR(force_ignore_odometry,bool, source,section);

	data_assoc_method    = source.read_enum<TDataAssociationMethod>(section,"data_assoc_method",data_assoc_method);
	data_assoc_metric    = source.read_enum<TDataAssociationMetric>(section,"data_assoc_metric",data_assoc_metric);
	data_assoc_IC_metric = source.read_enum<TDataAssociationMetric>(section,"data_assoc_IC_metric",data_assoc_IC_metric);

	MRPT_LOAD_CONFIG_VAR(data_assoc_IC_chi2_thres,double,  source, section);
	MRPT_LOAD_CONFIG_VAR(data_assoc_IC_ml_threshold,double,  source, section);

	MRPT_LOAD_CONFIG_VAR(quantiles_3D_representation, float, source,section);

}

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CRangeBearingKFSLAM::TOptions::TOptions() :
	stds_Q_no_odo(get_vehicle_size(),0),
	std_sensor_range ( 0.01f),
	std_sensor_yaw   ( DEG2RAD( 0.2f )),
	std_sensor_pitch ( DEG2RAD( 0.2f )),
	std_odo_z_additional ( 0),
	doPartitioningExperiment(false),
	quantiles_3D_representation ( 3),
	partitioningMethod(0),
	data_assoc_method			(assocNN),
	data_assoc_metric			(metricMaha),
	data_assoc_IC_chi2_thres	(0.99),
	data_assoc_IC_metric		(metricMaha),
	data_assoc_IC_ml_threshold	(0.0),
	create_simplemap			(false),
	force_ignore_odometry		(false)
{
	stds_Q_no_odo[0]=
	stds_Q_no_odo[1]=
	stds_Q_no_odo[2]=0.10f;
	stds_Q_no_odo[3]=
	stds_Q_no_odo[4]=
	stds_Q_no_odo[5]=
	stds_Q_no_odo[6]=0.05f;
}

/*---------------------------------------------------------------
						dumpToTextStream
  ---------------------------------------------------------------*/
void CRangeBearingKFSLAM::TOptions::dumpToTextStream( CStream &out)const
{
	out.printf("\n----------- [CRangeBearingKFSLAM::TOptions] ------------ \n\n");

	out.printf("doPartitioningExperiment                = %c\n",	doPartitioningExperiment ? 'Y':'N' );
	out.printf("partitioningMethod                      = %i\n",	partitioningMethod );
	out.printf("data_assoc_method                       = %s\n", TEnumType<TDataAssociationMethod>::value2name(data_assoc_method).c_str() );
	out.printf("data_assoc_metric                       = %s\n", TEnumType<TDataAssociationMetric>::value2name(data_assoc_metric).c_str() );
	out.printf("data_assoc_IC_chi2_thres                = %.06f\n", data_assoc_IC_chi2_thres );
	out.printf("data_assoc_IC_metric                    = %s\n", TEnumType<TDataAssociationMetric>::value2name(data_assoc_IC_metric).c_str() );
	out.printf("data_assoc_IC_ml_threshold              = %.06f\n", data_assoc_IC_ml_threshold );

	out.printf("\n");
}

void CRangeBearingKFSLAM::OnInverseObservationModel(
	const KFArray_OBS & in_z,
	KFArray_FEAT  & yn,
	KFMatrix_FxV  & dyn_dxv,
	KFMatrix_FxO  & dyn_dhn ) const
{
	MRPT_START

	CObservationBearingRangePtr obs = m_SF->getObservationByClass<CObservationBearingRange>();
	ASSERTMSG_(obs,"*ERROR*: This method requires an observation of type CObservationBearingRange")
	const CPose3DQuat   sensorPoseOnRobot = CPose3DQuat( obs->sensorLocationOnRobot );

	// Mean of the prior of the robot pose:
	const CPose3DQuat	robotPose = getCurrentRobotPoseMean();

	//const CPose3DQuat  sensorPoseAbs= robotPose + sensorPoseOnRobot;
	CPose3DQuat  sensorPoseAbs(UNINITIALIZED_QUATERNION);
	CMatrixFixedNumeric<kftype,7,7>  dsensorabs_dvehpose(UNINITIALIZED_MATRIX);
	CMatrixFixedNumeric<kftype,7,7>  dsensorabs_dsenrelpose(UNINITIALIZED_MATRIX);  // Not actually used

	CPose3DQuatPDF::jacobiansPoseComposition(
		robotPose,
		sensorPoseOnRobot,
		dsensorabs_dvehpose,
		dsensorabs_dsenrelpose,
		&sensorPoseAbs);

    kftype hn_r = in_z[0];
    kftype hn_y = in_z[1];
    kftype hn_p = in_z[2];

    kftype chn_y = cos(hn_y);  kftype shn_y = sin(hn_y);
    kftype chn_p = cos(hn_p);  kftype shn_p = sin(hn_p);

	/* -------------------------------------------
		syms H h_range h_yaw h_pitch real;
		H=[ h_range ; h_yaw ; h_pitch ];

		syms X xi_ yi_ zi_ real;
		xi_ = h_range * cos(h_yaw) * cos(h_pitch);
		yi_ = h_range * sin(h_yaw) * cos(h_pitch);
		zi_ = -h_range * sin(h_pitch);

		X=[xi_ yi_ zi_];
		jacob_inv_mod=jacobian(X,H)
	  ------------------------------------------- */

	// The new point, relative to the sensor:
	const TPoint3D  yn_rel_sensor(
		hn_r * chn_y * chn_p,
		hn_r * shn_y * chn_p,
		-hn_r * shn_p );

	// The Jacobian of the 3D point in the coordinate system of the sensor:
	/*
		[ cos(h_pitch)*cos(h_yaw), -h_range*cos(h_pitch)*sin(h_yaw), -h_range*cos(h_yaw)*sin(h_pitch)]
		[ cos(h_pitch)*sin(h_yaw),  h_range*cos(h_pitch)*cos(h_yaw), -h_range*sin(h_pitch)*sin(h_yaw)]
		[           -sin(h_pitch),                                0,            -h_range*cos(h_pitch)]
	*/
	const kftype values_dynlocal_dhn[] = {
		chn_p*chn_y		, -hn_r*chn_p*shn_y		, -hn_r*chn_y*shn_p,
		chn_p*shn_y		,  hn_r*chn_p*chn_y		, -hn_r*shn_p*shn_y,
		-shn_p			,  0					, -hn_r*chn_p
		};
	const KFMatrix_FxO  dynlocal_dhn(values_dynlocal_dhn);

	KFMatrix_FxF jacob_dyn_dynrelsensor(UNINITIALIZED_MATRIX);
	KFMatrix_FxV jacob_dyn_dsensorabs(UNINITIALIZED_MATRIX);

	sensorPoseAbs.composePoint(
		yn_rel_sensor.x, yn_rel_sensor.y, yn_rel_sensor.z,  // yn rel. to the sensor
		yn[0], yn[1], yn[2],  // yn in global coords
		&jacob_dyn_dynrelsensor,
		&jacob_dyn_dsensorabs
		);

	//dyn_dhn = jacob_dyn_dynrelsensor * dynlocal_dhn;
	dyn_dhn.multiply_AB(jacob_dyn_dynrelsensor, dynlocal_dhn);

	//dyn_dxv =
	dyn_dxv.multiply_AB(jacob_dyn_dsensorabs, dsensorabs_dvehpose ); // dsensorabs_dsenrelpose);

    MRPT_END
}


/** If applicable to the given problem, do here any special handling of adding a new landmark to the map.
  * \param in_obsIndex The index of the observation whose inverse sensor is to be computed. It corresponds to the row in in_z where the observation can be found.
  * \param in_idxNewFeat The index that this new feature will have in the state vector (0:just after the vehicle state, 1: after that,...). Save this number so data association can be done according to these indices.
  * \sa OnInverseObservationModel
  */
void CRangeBearingKFSLAM::OnNewLandmarkAddedToMap(
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
		m_IDs.insert( in_idxNewFeat, in_idxNewFeat);
	}

    MRPT_END
}


/*---------------------------------------------------------------
						getAs3DObject
  ---------------------------------------------------------------*/
void  CRangeBearingKFSLAM::getAs3DObject( mrpt::opengl::CSetOfObjectsPtr	&outObj ) const
{
    outObj->clear();

	// ------------------------------------------------
	//  Add the XYZ corner for the current area:
	// ------------------------------------------------
	outObj->insert( opengl::stock_objects::CornerXYZ() );


	// 3D ellipsoid for robot pose:
	CPointPDFGaussian	pointGauss;
    pointGauss.mean.x( m_xkk[0] );
    pointGauss.mean.y( m_xkk[1] );
    pointGauss.mean.z( m_xkk[2] );
    CMatrixTemplateNumeric<kftype>  COV;
    m_pkk.extractMatrix(0,0,3,3, COV);
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


	// 3D ellipsoids for landmarks:
	const size_t nLMs = this->getNumberOfLandmarksInTheMap();
	for (size_t i=0;i<nLMs;i++)
	{
        pointGauss.mean.x( m_xkk[get_vehicle_size()+get_feature_size()*i+0] );
        pointGauss.mean.y( m_xkk[get_vehicle_size()+get_feature_size()*i+1] );
        pointGauss.mean.z( m_xkk[get_vehicle_size()+get_feature_size()*i+2] );
        m_pkk.extractMatrix(get_vehicle_size()+get_feature_size()*i,get_vehicle_size()+get_feature_size()*i,get_feature_size(),get_feature_size(), COV);
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

		// Set color depending on partitions?
		if (options.doPartitioningExperiment)
		{
		    // This is done for each landmark:
			map<int,bool>    belongToPartition;
			const CSimpleMap *SFs = &m_SFs; //mapPartitioner.getSequenceOfFrames();

			for (size_t p=0;p<m_lastPartitionSet.size();p++)
			{
				for (size_t w=0;w<m_lastPartitionSet[p].size();w++)
				{
					// Check if landmark #i is in the SF of m_lastPartitionSet[p][w]:
					CLandmark::TLandmarkID  i_th_ID = m_IDs.inverse(i);

					// Look for the lm_ID in the SF:
					CPose3DPDFPtr    pdf;
					CSensoryFramePtr SF_i;
					SFs->get( m_lastPartitionSet[p][w], pdf, SF_i);

					CObservationBearingRangePtr obs = SF_i->getObservationByClass<CObservationBearingRange>();

					for (size_t o=0;o<obs->sensedData.size();o++)
					{
						if ( obs->sensedData[o].landmarkID == i_th_ID )
						{
							belongToPartition[p]=true;
							break;
						}
					} // end for o
				} // end for w
			} // end for p

			// Build label:
			string strParts("[");

			for (map<int,bool>::iterator it=belongToPartition.begin();it!=belongToPartition.end();++it)
			{
				if (it!=belongToPartition.begin()) strParts += string(",");
				strParts += format("%i",it->first);
			}

			ellip->setName( ellip->getName() + strParts + string("]") );

		}

		outObj->insert( ellip );
	}
}


/*---------------------------------------------------------------
						getLastPartitionLandmarks
  ---------------------------------------------------------------*/
void CRangeBearingKFSLAM::getLastPartitionLandmarksAsIfFixedSubmaps( size_t K, std::vector<vector_uint>	&landmarksMembership )
{
    // temporary copy:
    std::vector<vector_uint> tmpParts = m_lastPartitionSet;

    // Fake "m_lastPartitionSet":

    // Fixed partitions every K observations:
    vector<vector_uint>   partitions;
    vector_uint           tmpCluster;

    for (size_t i=0;i<m_SFs.size();i++)
    {
        tmpCluster.push_back(i);
        if ( (i % K) == 0 )
        {
            partitions.push_back(tmpCluster);
            tmpCluster.clear();
            tmpCluster.push_back(i);  // This observation "i" is shared between both clusters
        }
    }
    m_lastPartitionSet = partitions;

    // Call the actual method:
    getLastPartitionLandmarks(landmarksMembership);

    // Replace copy:
    m_lastPartitionSet = tmpParts; //-V519
}

/*---------------------------------------------------------------
						getLastPartitionLandmarks
  ---------------------------------------------------------------*/
void CRangeBearingKFSLAM::getLastPartitionLandmarks(
    std::vector<vector_uint>	&landmarksMembership ) const
{
    landmarksMembership.clear();

    // All the computation is made based on "m_lastPartitionSet"

    if (!options.doPartitioningExperiment) return;

	const size_t nLMs = this->getNumberOfLandmarksInTheMap();
	for (size_t i=0;i<nLMs;i++)
	{
        map<int,bool>    belongToPartition;
        const CSimpleMap *SFs = &m_SFs; //mapPartitioner.getSequenceOfFrames();

        for (size_t p=0;p<m_lastPartitionSet.size();p++)
        {
            for (size_t w=0;w<m_lastPartitionSet[p].size();w++)
            {
                // Check if landmark #i is in the SF of m_lastPartitionSet[p][w]:
                CLandmark::TLandmarkID  i_th_ID = m_IDs.inverse(i);

                // Look for the lm_ID in the SF:
                CPose3DPDFPtr pdf;
                CSensoryFramePtr SF_i;
                SFs->get( m_lastPartitionSet[p][w], pdf, SF_i);

                CObservationBearingRangePtr obs = SF_i->getObservationByClass<CObservationBearingRange>();

                for (size_t o=0;o<obs->sensedData.size();o++)
                {
                    if ( obs->sensedData[o].landmarkID == i_th_ID )
                    {
                        belongToPartition[p]=true;
                        break;
                    }
                } // end for o
            } // end for w
        } // end for p

        // Build membership list:
        vector_uint membershipOfThisLM;

        for (map<int,bool>::iterator it=belongToPartition.begin();it!=belongToPartition.end();++it)
            membershipOfThisLM.push_back(it->first);

        landmarksMembership.push_back( membershipOfThisLM );
	} // end for i
}


/*---------------------------------------------------------------
            computeOffDiagonalBlocksApproximationError
  ---------------------------------------------------------------*/
double CRangeBearingKFSLAM::computeOffDiagonalBlocksApproximationError(
    const std::vector<vector_uint>	&landmarksMembership ) const
{
    MRPT_START

    // Compute the information matrix:
    CMatrixTemplateNumeric<kftype> fullCov( m_pkk );
	size_t i;
    for (i=0;i<get_vehicle_size();i++)
        fullCov(i,i) = max(fullCov(i,i), 1e-6);

    CMatrixTemplateNumeric<kftype>		H( fullCov.inv() );
    H.array().abs();        // Replace by absolute values:

    double sumOffBlocks = 0;
    unsigned int nLMs = landmarksMembership.size();

    ASSERT_((get_vehicle_size()+nLMs*get_feature_size())==fullCov.getColCount());

    for (i=0;i<nLMs;i++)
    {
        for (size_t j=i+1;j<nLMs;j++)
        {
            // Sum the cross cov. between LM(i) and LM(j)??
            // --> Only if there is no common cluster:
            if ( 0==math::countCommonElements( landmarksMembership[i],landmarksMembership[j] ) )
            {
                size_t col = get_vehicle_size()+i*get_feature_size();
                size_t row = get_vehicle_size()+j*get_feature_size();
                sumOffBlocks += 2 * H.block( row,col,2,2).sum();
            }
        }
    }

    return sumOffBlocks / H.block(get_vehicle_size(),get_vehicle_size(),H.rows()-get_vehicle_size(),H.cols()-get_vehicle_size()).sum();  // Starting (7,7)-end
    MRPT_END
}

/*---------------------------------------------------------------
              reconsiderPartitionsNow
  ---------------------------------------------------------------*/
void CRangeBearingKFSLAM::reconsiderPartitionsNow()
{
	mapPartitioner.markAllNodesForReconsideration();

    vector<vector_uint>   partitions; // A different buffer for making this thread-safe some day...

    mapPartitioner.updatePartitions( partitions );

    m_lastPartitionSet = partitions;
}

/*---------------------------------------------------------------
              saveMapAndPathRepresentationAsMATLABFile
  ---------------------------------------------------------------*/
void CRangeBearingKFSLAM::saveMapAndPath2DRepresentationAsMATLABFile(
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
	os::fprintf(f,"%% 'CRangeBearingKFSLAM::saveMapAndPath2DRepresentationAsMATLABFile'\n");
	os::fprintf(f,"%%\n");
	os::fprintf(f,"%%                        ~ MRPT ~\n");
	os::fprintf(f,"%%  Jose Luis Blanco Claraco, University of Malaga @ 2008\n");
	os::fprintf(f,"%%      http://www.mrpt.org/     \n");
	os::fprintf(f,"%%--------------------------------------------------------------------\n");

	// Main code:
	os::fprintf(f,"hold on;\n\n");

	const size_t nLMs = this->getNumberOfLandmarksInTheMap();

	for (size_t i=0;i<nLMs;i++)
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
		for (size_t i=0;i<m_SFs.size();i++)
		{
			CSensoryFramePtr dummySF;
			CPose3DPDFPtr pdf3D;
			m_SFs.get(i,pdf3D,dummySF);

			CPose3D p;
			pdf3D->getMean(p);

			os::fprintf(f,"%.04f %.04f", p.x(), p.y() );
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
void CRangeBearingKFSLAM::OnSubstractObservationVectors(KFArray_OBS &A, const KFArray_OBS &B) const
{
	A-=B;
	mrpt::math::wrapToPiInPlace(A[1]);
	mrpt::math::wrapToPiInPlace(A[2]);
}


/** Return the observation NOISE covariance matrix, that is, the model of the Gaussian additive noise of the sensor.
  * \param out_R The noise covariance matrix. It might be non diagonal, but it'll usually be.
  */
void CRangeBearingKFSLAM::OnGetObservationNoise(KFMatrix_OxO &out_R) const
{
	out_R(0,0)= square( options.std_sensor_range );
	out_R(1,1)= square( options.std_sensor_yaw );
	out_R(2,2)= square( options.std_sensor_pitch );
}

/** This will be called before OnGetObservationsAndDataAssociation to allow the application to reduce the number of covariance landmark predictions to be made.
  *  For example, features which are known to be "out of sight" shouldn't be added to the output list to speed up the calculations.
  * \param in_all_prediction_means The mean of each landmark predictions; the computation or not of the corresponding covariances is what we're trying to determined with this method.
  * \param out_LM_indices_to_predict The list of landmark indices in the map [0,getNumberOfLandmarksInTheMap()-1] that should be predicted.
  * \note This is not a pure virtual method, so it should be implemented only if desired. The default implementation returns a vector with all the landmarks in the map.
  * \sa OnGetObservations, OnDataAssociation
  */
void CRangeBearingKFSLAM::OnPreComputingPredictions(
	const vector_KFArray_OBS	&prediction_means,
	vector_size_t				&out_LM_indices_to_predict ) const
{
	CObservationBearingRangePtr obs = m_SF->getObservationByClass<CObservationBearingRange>();
	ASSERTMSG_(obs,"*ERROR*: This method requires an observation of type CObservationBearingRange")

#define USE_HEURISTIC_PREDICTION

#ifdef USE_HEURISTIC_PREDICTION
	const double sensor_max_range = obs->maxSensorDistance;
	const double fov_yaw   = obs->fieldOfView_yaw;
	const double fov_pitch = obs->fieldOfView_pitch;

	const double max_vehicle_loc_uncertainty = 4 * std::sqrt( m_pkk.get_unsafe(0,0) + m_pkk.get_unsafe(1,1)+m_pkk.get_unsafe(2,2) );
#endif

	out_LM_indices_to_predict.clear();
	for (size_t i=0;i<prediction_means.size();i++)
	{
#ifndef USE_HEURISTIC_PREDICTION
		out_LM_indices_to_predict.push_back(i);
#else
	// Heuristic: faster but doesn't work always!
		if (      prediction_means[i][0] < (       15   + sensor_max_range + max_vehicle_loc_uncertainty + 4*options.std_sensor_range) &&
		    fabs(prediction_means[i][1]) < (DEG2RAD(30) + 0.5*fov_yaw      + 4*options.std_sensor_yaw)   &&
			fabs(prediction_means[i][2]) < (DEG2RAD(30) + 0.5*fov_pitch    + 4*options.std_sensor_pitch)
			)
		{
			out_LM_indices_to_predict.push_back(i);
		}
#endif
	}
}
