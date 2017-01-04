/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CRangeBearingKFSLAM2D_H
#define CRangeBearingKFSLAM2D_H

#include <mrpt/math/CMatrixTemplateNumeric.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/opengl/opengl_frwds.h>
#include <mrpt/bayes/CKalmanFilterCapable.h>

#include <mrpt/utils/safe_pointers.h>
#include <mrpt/utils/bimap.h>

#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/maps/CLandmark.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/slam/CIncrementalMapPartitioner.h>
#include <mrpt/slam/data_association.h>

#include <mrpt/slam/link_pragmas.h>

namespace mrpt
{
	namespace slam
	{
		/** An implementation of EKF-based SLAM with range-bearing sensors, odometry, and a 2D (+heading) robot pose, and 2D landmarks.
		  *  The main method is "processActionObservation" which processes pairs of action/observation.
		  *
		  *   The following pages describe front-end applications based on this class:
		  *		- http://www.mrpt.org/Application:2d-slam-demo
		  *		- http://www.mrpt.org/Application:kf-slam
		  *
		  * \sa CRangeBearingKFSLAM  \ingroup metric_slam_grp
		  */
		class SLAM_IMPEXP  CRangeBearingKFSLAM2D :
			public bayes::CKalmanFilterCapable<3 /* x y yaw */, 2 /* range yaw */, 2 /* x y */,      3 /* Ax Ay Ayaw */>
			                               // <size_t VEH_SIZE, size_t OBS_SIZE,   size_t FEAT_SIZE, size_t ACT_SIZE, size typename kftype = double>
		{
		 public:
			typedef mrpt::math::TPoint2D landmark_point_t;  //!< Either mrpt::math::TPoint2D or mrpt::math::TPoint3D

			 CRangeBearingKFSLAM2D( ); //!< Default constructor
			 virtual ~CRangeBearingKFSLAM2D();	//!< Destructor
			 void reset(); //!< Reset the state of the SLAM filter: The map is emptied and the robot put back to (0,0,0).

			/** Process one new action and observations to update the map and robot pose estimate. See the description of the class at the top of this page.
			 *  \param action May contain odometry
			 *	\param SF The set of observations, must contain at least one CObservationBearingRange
			 */
			void  processActionObservation(
				mrpt::obs::CActionCollectionPtr &action,
				mrpt::obs::CSensoryFramePtr     &SF );

			/** Returns the complete mean and cov.
			  *  \param out_robotPose The mean & 3x3 covariance matrix of the robot 2D pose
			  *  \param out_landmarksPositions One entry for each of the M landmark positions (2D).
			  *  \param out_landmarkIDs Each element[index] (for indices of out_landmarksPositions) gives the corresponding landmark ID.
			  *  \param out_fullState The complete state vector (3+2M).
			  *  \param out_fullCovariance The full (3+2M)x(3+2M) covariance matrix of the filter.
			  * \sa getCurrentRobotPose
			  */
			void  getCurrentState(
				mrpt::poses::CPosePDFGaussian &out_robotPose,
				std::vector<mrpt::math::TPoint2D>  &out_landmarksPositions,
				std::map<unsigned int,mrpt::maps::CLandmark::TLandmarkID> &out_landmarkIDs,
				mrpt::math::CVectorDouble      &out_fullState,
				mrpt::math::CMatrixDouble      &out_fullCovariance
				) const;

			/** Returns the mean & 3x3 covariance matrix of the robot 2D pose.
			  * \sa getCurrentState
			  */
			void  getCurrentRobotPose(
				mrpt::poses::CPosePDFGaussian &out_robotPose ) const;

			/** Returns a 3D representation of the landmarks in the map and the robot 3D position according to the current filter state.
			  *  \param out_objects
			  */
			void  getAs3DObject( mrpt::opengl::CSetOfObjectsPtr	&outObj ) const;

			/** Load options from a ini-like file/text
			  */
			void loadOptions( const mrpt::utils::CConfigFileBase &ini );

			/** The options for the algorithm
			  */
			struct SLAM_IMPEXP TOptions : utils::CLoadableOptions
			{
				/** Default values */
				TOptions();

				void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section) MRPT_OVERRIDE; // See base docs
				void dumpToTextStream(mrpt::utils::CStream &out) const MRPT_OVERRIDE; // See base docs

				mrpt::math::CVectorFloat 	stds_Q_no_odo;	//!< A 3-length vector with the std. deviation of the transition model in (x,y,phi) used only when there is no odometry (if there is odo, its uncertainty values will be used instead); x y: In meters, phi: radians (but in degrees when loading from a configuration ini-file!)
				float 			std_sensor_range, std_sensor_yaw;	//!< The std. deviation of the sensor (for the matrix R in the kalman filters), in meters and radians.
				float 			quantiles_3D_representation;	//!< Default = 3
				bool			create_simplemap; //!< Whether to fill m_SFs (default=false)

				// Data association:
				TDataAssociationMethod  data_assoc_method;
				TDataAssociationMetric  data_assoc_metric;
				double					data_assoc_IC_chi2_thres;  //!< Threshold in [0,1] for the chi2square test for individual compatibility between predictions and observations (default: 0.99)
				TDataAssociationMetric  data_assoc_IC_metric;	   //!< Whether to use mahalanobis (->chi2 criterion) vs. Matching likelihood.
				double					data_assoc_IC_ml_threshold;//!< Only if data_assoc_IC_metric==ML, the log-ML threshold (Default=0.0)

			};

			TOptions options; //!< The options for the algorithm


			/** Save the current state of the filter (robot pose & map) to a MATLAB script which displays all the elements in 2D
			  */
			void saveMapAndPath2DRepresentationAsMATLABFile(
				const std::string &fil,
				float              stdCount=3.0f,
				const std::string &styleLandmarks = std::string("b"),
				const std::string &stylePath = std::string("r"),
				const std::string &styleRobot = std::string("r") ) const;


			/** Information for data-association:
			  * \sa getLastDataAssociation
			  */
			struct SLAM_IMPEXP TDataAssocInfo
			{
				TDataAssocInfo() :
					Y_pred_means(0,0),
					Y_pred_covs(0,0)
				{
				}

				void clear() {
					results.clear();
					predictions_IDs.clear();
					newly_inserted_landmarks.clear();
				}

				// Predictions from the map:
				mrpt::math::CMatrixTemplateNumeric<kftype>	Y_pred_means,Y_pred_covs;
				mrpt::vector_size_t				predictions_IDs;

				/** Map from the 0-based index within the last observation and the landmark 0-based index in the map (the robot-map state vector)
				    Only used for stats and so. */
				std::map<size_t,size_t>  newly_inserted_landmarks;

				// DA results:
				TDataAssociationResults			results;
			};

			/** Returns a read-only reference to the information on the last data-association */
			const TDataAssocInfo & getLastDataAssociation() const {
				return m_last_data_association;
			}

		 protected:

			/** @name Virtual methods for Kalman Filter implementation
				@{
			 */

			/** Must return the action vector u.
			  * \param out_u The action vector which will be passed to OnTransitionModel
			  */
			void OnGetAction( KFArray_ACT &out_u ) const;

			/** Implements the transition model \f$ \hat{x}_{k|k-1} = f( \hat{x}_{k-1|k-1}, u_k ) \f$
			  * \param in_u The vector returned by OnGetAction.
			  * \param inout_x At input has \f[ \hat{x}_{k-1|k-1} \f] , at output must have \f$ \hat{x}_{k|k-1} \f$ .
			  * \param out_skip Set this to true if for some reason you want to skip the prediction step (to do not modify either the vector or the covariance). Default:false
			  */
			void OnTransitionModel(
				const KFArray_ACT &in_u,
				KFArray_VEH       &inout_x,
				bool &out_skipPrediction
				) const;

			/** Implements the transition Jacobian \f$ \frac{\partial f}{\partial x} \f$
			  * \param out_F Must return the Jacobian.
			  *  The returned matrix must be \f$V \times V\f$ with V being either the size of the whole state vector (for non-SLAM problems) or VEH_SIZE (for SLAM problems).
			  */
			void OnTransitionJacobian( KFMatrix_VxV  &out_F ) const;

			/** Only called if using a numeric approximation of the transition Jacobian, this method must return the increments in each dimension of the vehicle state vector while estimating the Jacobian.
			  */
			void OnTransitionJacobianNumericGetIncrements(KFArray_VEH &out_increments) const;


			/** Implements the transition noise covariance \f$ Q_k \f$
			  * \param out_Q Must return the covariance matrix.
			  *  The returned matrix must be of the same size than the jacobian from OnTransitionJacobian
			  */
			void OnTransitionNoise( KFMatrix_VxV &out_Q ) const;

			/** This is called between the KF prediction step and the update step, and the application must return the observations and, when applicable, the data association between these observations and the current map.
			  *
			  * \param out_z N vectors, each for one "observation" of length OBS_SIZE, N being the number of "observations": how many observed landmarks for a map, or just one if not applicable.
			  * \param out_data_association An empty vector or, where applicable, a vector where the i'th element corresponds to the position of the observation in the i'th row of out_z within the system state vector (in the range [0,getNumberOfLandmarksInTheMap()-1]), or -1 if it is a new map element and we want to insert it at the end of this KF iteration.
			  * \param in_S The full covariance matrix of the observation predictions (i.e. the "innovation covariance matrix"). This is a M*O x M*O matrix with M=length of "in_lm_indices_in_S".
			  * \param in_lm_indices_in_S The indices of the map landmarks (range [0,getNumberOfLandmarksInTheMap()-1]) that can be found in the matrix in_S.
			  *
			  *  This method will be called just once for each complete KF iteration.
			  * \note It is assumed that the observations are independent, i.e. there are NO cross-covariances between them.
			  */
			void OnGetObservationsAndDataAssociation(
				vector_KFArray_OBS    &out_z,
				vector_int                  &out_data_association,
				const vector_KFArray_OBS   &in_all_predictions,
				const KFMatrix              &in_S,
				const vector_size_t         &in_lm_indices_in_S,
				const KFMatrix_OxO          &in_R
				);

			void OnObservationModel(
				const vector_size_t       &idx_landmarks_to_predict,
				vector_KFArray_OBS  &out_predictions
				) const;

			/** Implements the observation Jacobians \f$ \frac{\partial h_i}{\partial x} \f$ and (when applicable) \f$ \frac{\partial h_i}{\partial y_i} \f$.
			  * \param idx_landmark_to_predict The index of the landmark in the map whose prediction is expected as output. For non SLAM-like problems, this will be zero and the expected output is for the whole state vector.
			  * \param Hx  The output Jacobian \f$ \frac{\partial h_i}{\partial x} \f$.
			  * \param Hy  The output Jacobian \f$ \frac{\partial h_i}{\partial y_i} \f$.
			  */
			void OnObservationJacobians(
				const size_t &idx_landmark_to_predict,
				KFMatrix_OxV &Hx,
				KFMatrix_OxF &Hy
				) const;

			/** Only called if using a numeric approximation of the observation Jacobians, this method must return the increments in each dimension of the vehicle state vector while estimating the Jacobian.
			  */
			void OnObservationJacobiansNumericGetIncrements(
					KFArray_VEH  &out_veh_increments,
					KFArray_FEAT &out_feat_increments ) const;


			/** Computes A=A-B, which may need to be re-implemented depending on the topology of the individual scalar components (eg, angles).
			  */
			void OnSubstractObservationVectors(KFArray_OBS &A, const KFArray_OBS &B) const;

			/** Return the observation NOISE covariance matrix, that is, the model of the Gaussian additive noise of the sensor.
			  * \param out_R The noise covariance matrix. It might be non diagonal, but it'll usually be.
			  */
			void OnGetObservationNoise(KFMatrix_OxO &out_R) const;

			/** This will be called before OnGetObservationsAndDataAssociation to allow the application to reduce the number of covariance landmark predictions to be made.
			  *  For example, features which are known to be "out of sight" shouldn't be added to the output list to speed up the calculations.
			  * \param in_all_prediction_means The mean of each landmark predictions; the computation or not of the corresponding covariances is what we're trying to determined with this method.
			  * \param out_LM_indices_to_predict The list of landmark indices in the map [0,getNumberOfLandmarksInTheMap()-1] that should be predicted.
			  * \note This is not a pure virtual method, so it should be implemented only if desired. The default implementation returns a vector with all the landmarks in the map.
			  * \sa OnGetObservations, OnDataAssociation
			  */
			void OnPreComputingPredictions(
				const vector_KFArray_OBS	&in_all_prediction_means,
				vector_size_t				&out_LM_indices_to_predict ) const;

			/** If applicable to the given problem, this method implements the inverse observation model needed to extend the "map" with a new "element".
			  * \param in_z The observation vector whose inverse sensor model is to be computed. This is actually one of the vector<> returned by OnGetObservations().
			  * \param out_yn The F-length vector with the inverse observation model \f$ y_n=y(x,z_n) \f$.
			  * \param out_dyn_dxv The \f$F \times V\f$ Jacobian of the inv. sensor model wrt the robot pose \f$ \frac{\partial y_n}{\partial x_v} \f$.
			  * \param out_dyn_dhn The \f$F \times O\f$ Jacobian of the inv. sensor model wrt the observation vector \f$ \frac{\partial y_n}{\partial h_n} \f$.
			  *
			  *  - O: OBS_SIZE
			  *  - V: VEH_SIZE
			  *  - F: FEAT_SIZE
			  *
			  * \note OnNewLandmarkAddedToMap will be also called after calling this method if a landmark is actually being added to the map.
			  */
			void  OnInverseObservationModel(
				const KFArray_OBS & in_z,
				KFArray_FEAT  & out_yn,
				KFMatrix_FxV  & out_dyn_dxv,
				KFMatrix_FxO  & out_dyn_dhn ) const;

			/** If applicable to the given problem, do here any special handling of adding a new landmark to the map.
			  * \param in_obsIndex The index of the observation whose inverse sensor is to be computed. It corresponds to the row in in_z where the observation can be found.
			  * \param in_idxNewFeat The index that this new feature will have in the state vector (0:just after the vehicle state, 1: after that,...). Save this number so data association can be done according to these indices.
			  * \sa OnInverseObservationModel
			  */
			void OnNewLandmarkAddedToMap(
				const size_t	in_obsIdx,
				const size_t	in_idxNewFeat );


			/** This method is called after the prediction and after the update, to give the user an opportunity to normalize the state vector (eg, keep angles within -pi,pi range) if the application requires it.
			  */
			void OnNormalizeStateVector();

			/** @}
			 */


			void getLandmarkIDsFromIndexInStateVector(std::map<unsigned int,mrpt::maps::CLandmark::TLandmarkID>  &out_id2index) const
			{
				out_id2index = m_IDs.getInverseMap();
			}

		protected:

			/** Set up by processActionObservation */
			mrpt::obs::CActionCollectionPtr	m_action;

			/** Set up by processActionObservation */
			mrpt::obs::CSensoryFramePtr		m_SF;

			/** The mapping between landmark IDs and indexes in the Pkk cov. matrix: */
			mrpt::utils::bimap<mrpt::maps::CLandmark::TLandmarkID,unsigned int>	m_IDs;

			/** The sequence of all the observations and the robot path (kept for debugging, statistics,etc) */
			mrpt::maps::CSimpleMap      m_SFs;

			TDataAssocInfo m_last_data_association; //!< Last data association
		}; // end class
	} // End of namespace
} // End of namespace

#endif
