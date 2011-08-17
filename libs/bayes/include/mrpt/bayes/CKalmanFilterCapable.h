/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */
#ifndef CKalmanFilterCapable_H
#define CKalmanFilterCapable_H

#include <mrpt/math/CMatrixFixedNumeric.h>
#include <mrpt/math/CMatrixTemplateNumeric.h>
#include <mrpt/math/CArray.h>
#include <mrpt/math/utils.h>

#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/CDebugOutputCapable.h>
#include <mrpt/utils/stl_extensions.h>
#include <mrpt/system/os.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/TEnumType.h>

#include <mrpt/bayes/link_pragmas.h>


namespace mrpt
{
	namespace bayes
	{
		using namespace mrpt::utils;
		using namespace mrpt::math;
		using namespace mrpt;
		using namespace std;

		/** The Kalman Filter algorithm to employ in bayes::CKalmanFilterCapable
		  *  For further details on each algorithm see the tutorial: http://www.mrpt.org/Kalman_Filters
		  * \sa bayes::CKalmanFilterCapable::KF_options
	 	  * \ingroup mrpt_bayes_grp
		  */
		enum TKFMethod {
			kfEKFNaive = 0,
			kfEKFAlaDavison,
			kfIKFFull,
			kfIKF
		};

		// Forward declaration:
		template <size_t VEH_SIZE, size_t OBS_SIZE, size_t FEAT_SIZE, size_t ACT_SIZE, typename KFTYPE> class CKalmanFilterCapable;

		namespace detail {
		struct CRunOneKalmanIteration_addNewLandmarks;
		}

		/** Generic options for the Kalman Filter algorithm in itself.
	 	  * \ingroup mrpt_bayes_grp
		  */
		struct BAYES_IMPEXP TKF_options : public utils::CLoadableOptions
		{
			TKF_options();

			void  loadFromConfigFile(
				const mrpt::utils::CConfigFileBase	&source,
				const std::string		&section);

			/** This method must display clearly all the contents of the structure in textual form, sending it to a CStream. */
			void  dumpToTextStream(CStream	&out) const;

			TKFMethod	method;				//!< The method to employ (default: kfEKFNaive)
			bool		verbose; 		//!< If set to true timing and other information will be dumped during the execution (default=false)
			int 		IKF_iterations;	//!< Number of refinement iterations, only for the IKF method.
			bool		enable_profiler;//!< If enabled (default=false), detailed timing information will be dumped to the console thru a CTimerLog at the end of the execution.
			bool		use_analytic_transition_jacobian;	//!< (default=true) If true, OnTransitionJacobian will be called; otherwise, the Jacobian will be estimated from a numeric approximation by calling several times to OnTransitionModel.
			bool		use_analytic_observation_jacobian;	//!< (default=true) If true, OnObservationJacobians will be called; otherwise, the Jacobian will be estimated from a numeric approximation by calling several times to OnObservationModel.
			bool		debug_verify_analytic_jacobians; //!< (default=false) If true, will compute all the Jacobians numerically and compare them to the analytical ones, throwing an exception on mismatch.
			double		debug_verify_analytic_jacobians_threshold; //!< (default-1e-2) Sets the threshold for the difference between the analytic and the numerical jacobians
		};

		/** Auxiliary functions, for internal usage of MRPT classes */
		namespace detail
		{
			// Auxiliary functions.
			template <size_t VEH_SIZE, size_t OBS_SIZE, size_t FEAT_SIZE, size_t ACT_SIZE, typename KFTYPE>
			inline size_t getNumberOfLandmarksInMap(const CKalmanFilterCapable<VEH_SIZE,OBS_SIZE,FEAT_SIZE,ACT_SIZE,KFTYPE> &obj);
			// Specialization:
			template <size_t VEH_SIZE, size_t OBS_SIZE, size_t ACT_SIZE, typename KFTYPE>
			inline size_t getNumberOfLandmarksInMap(const CKalmanFilterCapable<VEH_SIZE,OBS_SIZE,0 /*FEAT_SIZE*/,ACT_SIZE,KFTYPE> &obj);

			template <size_t VEH_SIZE, size_t OBS_SIZE, size_t FEAT_SIZE, size_t ACT_SIZE, typename KFTYPE>
			inline bool isMapEmpty(const CKalmanFilterCapable<VEH_SIZE,OBS_SIZE,FEAT_SIZE,ACT_SIZE,KFTYPE> &obj);
			// Specialization:
			template <size_t VEH_SIZE, size_t OBS_SIZE, size_t ACT_SIZE, typename KFTYPE>
			inline bool isMapEmpty(const CKalmanFilterCapable<VEH_SIZE,OBS_SIZE,0 /*FEAT_SIZE*/,ACT_SIZE,KFTYPE> &obj);
		}


		/** Virtual base for Kalman Filter (EKF,IEKF,UKF) implementations.
		 *   This base class stores the state vector and covariance matrix of the system. It has virtual methods that must be completed
		 *    by derived classes to address a given filtering problem. The main entry point of the algorithm is CKalmanFilterCapable::runOneKalmanIteration, which
		 *    should be called AFTER setting the desired filter options in KF_options, as well as any options in the derived class.
		 *   Note that the main entry point is protected, so derived classes must offer another method more specific to a given problem which, internally, calls runOneKalmanIteration.
		 *
		 *  For further details and examples, check out the tutorial: http://www.mrpt.org/Kalman_Filters
		 *
		 *  The Kalman filter algorithms are generic, but this implementation is biased to ease the implementation
		 *  of SLAM-like problems. However, it can be also applied to many generic problems not related to robotics or SLAM.
		 *
		 *  The meaning of the template parameters is:
		 *	- VEH_SIZE: The dimension of the "vehicle state": either the full state vector or the "vehicle" part if applicable.
		 *	- OBS_SIZE: The dimension of each observation (eg, 2 for pixel coordinates, 3 for 3D coordinates,etc).
		 *	- FEAT_SIZE: The dimension of the features in the system state (the "map"), or 0 if not applicable (the default if not implemented).
		 *	- ACT_SIZE: The dimension of each "action" u_k (or 0 if not applicable).
		 *	- KFTYPE: The numeric type of the matrices (default: double)
		 *
		 * Revisions:
		 *	- 2007: Antonio J. Ortiz de Galisteo (AJOGD)
		 *	- 2008/FEB: All KF classes corrected, reorganized, and rewritten (JLBC).
		 *	- 2008/MAR: Implemented IKF (JLBC).
		 *	- 2009/DEC: Totally rewritten as a generic template using fixed-size matrices where possible (JLBC).
		 *
		 *  \sa mrpt::slam::CRangeBearingKFSLAM, mrpt::slam::CRangeBearingKFSLAM2D
	 	 * \ingroup mrpt_bayes_grp
		 */
		template <size_t VEH_SIZE, size_t OBS_SIZE, size_t FEAT_SIZE, size_t ACT_SIZE, typename KFTYPE = double>
		class CKalmanFilterCapable : public mrpt::utils::CDebugOutputCapable
		{
		public:
			static inline size_t get_vehicle_size() { return VEH_SIZE; }
			static inline size_t get_observation_size() { return OBS_SIZE; }
			static inline size_t get_feature_size() { return FEAT_SIZE; }
			static inline size_t get_action_size() { return ACT_SIZE; }
			inline size_t getNumberOfLandmarksInTheMap() const { return detail::getNumberOfLandmarksInMap(*this); }
			inline bool   isMapEmpty() const { return detail::isMapEmpty(*this); }


			typedef KFTYPE kftype; //!< The numeric type used in the Kalman Filter (default=double)
			typedef CKalmanFilterCapable<VEH_SIZE,OBS_SIZE,FEAT_SIZE,ACT_SIZE,KFTYPE>  KFCLASS;  //!< My class, in a shorter name!

			// ---------- Many useful typedefs to short the notation a bit... --------
			typedef mrpt::dynamicsize_vector<KFTYPE> KFVector;
			typedef CMatrixTemplateNumeric<KFTYPE>   KFMatrix;

			typedef CMatrixFixedNumeric<KFTYPE,VEH_SIZE,VEH_SIZE>   KFMatrix_VxV;
			typedef CMatrixFixedNumeric<KFTYPE,OBS_SIZE,OBS_SIZE>   KFMatrix_OxO;
			typedef CMatrixFixedNumeric<KFTYPE,FEAT_SIZE,FEAT_SIZE> KFMatrix_FxF;
			typedef CMatrixFixedNumeric<KFTYPE,ACT_SIZE,ACT_SIZE>   KFMatrix_AxA;

			typedef CMatrixFixedNumeric<KFTYPE,VEH_SIZE,OBS_SIZE>   KFMatrix_VxO;
			typedef CMatrixFixedNumeric<KFTYPE,VEH_SIZE,FEAT_SIZE>  KFMatrix_VxF;

			typedef CMatrixFixedNumeric<KFTYPE,FEAT_SIZE,VEH_SIZE>  KFMatrix_FxV;
			typedef CMatrixFixedNumeric<KFTYPE,FEAT_SIZE,OBS_SIZE>  KFMatrix_FxO;

			typedef CMatrixFixedNumeric<KFTYPE,OBS_SIZE,FEAT_SIZE>  KFMatrix_OxF;
			typedef CMatrixFixedNumeric<KFTYPE,OBS_SIZE,VEH_SIZE>   KFMatrix_OxV;

			typedef CArrayNumeric<KFTYPE,VEH_SIZE>  KFArray_VEH;
			typedef CArrayNumeric<KFTYPE,ACT_SIZE>  KFArray_ACT;
			typedef CArrayNumeric<KFTYPE,OBS_SIZE>  KFArray_OBS;
			typedef typename mrpt::aligned_containers<KFArray_OBS>::vector_t  vector_KFArray_OBS;
			typedef CArrayNumeric<KFTYPE,FEAT_SIZE> KFArray_FEAT;

			inline size_t getStateVectorLength() const { return m_xkk.size(); }

			/** Returns the mean of the estimated value of the idx'th landmark (not applicable to non-SLAM problems).
			  * \exception std::exception On idx>= getNumberOfLandmarksInTheMap()
			  */
			inline void getLandmarkMean(size_t idx, KFArray_FEAT &feat ) const {
				ASSERT_(idx<getNumberOfLandmarksInTheMap())
				::memcpy(&feat[0], &m_xkk[VEH_SIZE+idx*FEAT_SIZE], FEAT_SIZE*sizeof(m_xkk[0]));
			}
			/** Returns the covariance of the idx'th landmark (not applicable to non-SLAM problems).
			  * \exception std::exception On idx>= getNumberOfLandmarksInTheMap()
			  */
			inline void getLandmarkCov(size_t idx, KFMatrix_FxF &feat_cov ) const {
				m_pkk.extractMatrix(VEH_SIZE+idx*FEAT_SIZE,VEH_SIZE+idx*FEAT_SIZE,feat_cov);
			}

		protected:
			/** @name Kalman filter state
				@{ */

			KFVector  m_xkk;  //!< The system state vector.
			KFMatrix  m_pkk;  //!< The system full covariance matrix.

			/** @} */

			mrpt::utils::CTimeLogger  m_timLogger;

			/** @name Virtual methods for Kalman Filter implementation
				@{
			 */

			/** Must return the action vector u.
			  * \param out_u The action vector which will be passed to OnTransitionModel
			  */
			virtual void OnGetAction( KFArray_ACT &out_u ) const = 0;

			/** Implements the transition model \f$ \hat{x}_{k|k-1} = f( \hat{x}_{k-1|k-1}, u_k ) \f$
			  * \param in_u The vector returned by OnGetAction.
			  * \param inout_x At input has \f[ \hat{x}_{k-1|k-1} \f] , at output must have \f$ \hat{x}_{k|k-1} \f$ .
			  * \param out_skip Set this to true if for some reason you want to skip the prediction step (to do not modify either the vector or the covariance). Default:false
			  * \note Even if you return "out_skip=true", the value of "inout_x" MUST be updated as usual (this is to allow numeric approximation of Jacobians).
			  */
			virtual void OnTransitionModel(
				const KFArray_ACT &in_u,
				KFArray_VEH       &inout_x,
				bool &out_skipPrediction
				)  const = 0;

			/** Implements the transition Jacobian \f$ \frac{\partial f}{\partial x} \f$
			  * \param out_F Must return the Jacobian.
			  *  The returned matrix must be \f$V \times V\f$ with V being either the size of the whole state vector (for non-SLAM problems) or VEH_SIZE (for SLAM problems).
			  */
			virtual void OnTransitionJacobian( KFMatrix_VxV  &out_F ) const
			{
				m_user_didnt_implement_jacobian=true;
			}

			/** Only called if using a numeric approximation of the transition Jacobian, this method must return the increments in each dimension of the vehicle state vector while estimating the Jacobian.
			  */
			virtual void OnTransitionJacobianNumericGetIncrements(KFArray_VEH &out_increments) const
			{
				for (size_t i=0;i<VEH_SIZE;i++) out_increments[i] = 1e-6;
			}

			/** Implements the transition noise covariance \f$ Q_k \f$
			  * \param out_Q Must return the covariance matrix.
			  *  The returned matrix must be of the same size than the jacobian from OnTransitionJacobian
			  */
			virtual void OnTransitionNoise( KFMatrix_VxV &out_Q )  const = 0;

			/** This will be called before OnGetObservationsAndDataAssociation to allow the application to reduce the number of covariance landmark predictions to be made.
			  *  For example, features which are known to be "out of sight" shouldn't be added to the output list to speed up the calculations.
			  * \param in_all_prediction_means The mean of each landmark predictions; the computation or not of the corresponding covariances is what we're trying to determined with this method.
			  * \param out_LM_indices_to_predict The list of landmark indices in the map [0,getNumberOfLandmarksInTheMap()-1] that should be predicted.
			  * \note This is not a pure virtual method, so it should be implemented only if desired. The default implementation returns a vector with all the landmarks in the map.
			  * \sa OnGetObservations, OnDataAssociation
			  */
			virtual void OnPreComputingPredictions(
				const vector_KFArray_OBS &in_all_prediction_means,
				mrpt::vector_size_t				&out_LM_indices_to_predict ) const
			{
				// Default: all of them:
				const size_t N = this->getNumberOfLandmarksInTheMap();
				out_LM_indices_to_predict.resize(N);
				for (size_t i=0;i<N;i++) out_LM_indices_to_predict[i]=i;
			}

			/** Return the observation NOISE covariance matrix, that is, the model of the Gaussian additive noise of the sensor.
			  * \param out_R The noise covariance matrix. It might be non diagonal, but it'll usually be.
			  * \note Upon call, it can be assumed that the previous contents of out_R are all zeros.
			  */
			virtual void OnGetObservationNoise(KFMatrix_OxO &out_R)  const = 0;

			/** This is called between the KF prediction step and the update step, and the application must return the observations and, when applicable, the data association between these observations and the current map.
			  *
			  * \param out_z N vectors, each for one "observation" of length OBS_SIZE, N being the number of "observations": how many observed landmarks for a map, or just one if not applicable.
			  * \param out_data_association An empty vector or, where applicable, a vector where the i'th element corresponds to the position of the observation in the i'th row of out_z within the system state vector (in the range [0,getNumberOfLandmarksInTheMap()-1]), or -1 if it is a new map element and we want to insert it at the end of this KF iteration.
			  * \param in_all_predictions A vector with the prediction of ALL the landmarks in the map. Note that, in contrast, in_S only comprises a subset of all the landmarks.
			  * \param in_S The full covariance matrix of the observation predictions (i.e. the "innovation covariance matrix"). This is a M·O x M·O matrix with M=length of "in_lm_indices_in_S".
			  * \param in_lm_indices_in_S The indices of the map landmarks (range [0,getNumberOfLandmarksInTheMap()-1]) that can be found in the matrix in_S.
			  *
			  *  This method will be called just once for each complete KF iteration.
			  * \note It is assumed that the observations are independent, i.e. there are NO cross-covariances between them.
			  */
			virtual void OnGetObservationsAndDataAssociation(
				vector_KFArray_OBS			&out_z,
				mrpt::vector_int            &out_data_association,
				const vector_KFArray_OBS	&in_all_predictions,
				const KFMatrix              &in_S,
				const vector_size_t         &in_lm_indices_in_S,
				const KFMatrix_OxO          &in_R
				) = 0;

			/** Implements the observation prediction \f$ h_i(x) \f$.
			  * \param idx_landmark_to_predict The indices of the landmarks in the map whose predictions are expected as output. For non SLAM-like problems, this input value is undefined and the application should just generate one observation for the given problem.
			  * \param out_predictions The predicted observations.
			  */
			virtual void OnObservationModel(
				const mrpt::vector_size_t       &idx_landmarks_to_predict,
				vector_KFArray_OBS  &out_predictions
				) const = 0;

			/** Implements the observation Jacobians \f$ \frac{\partial h_i}{\partial x} \f$ and (when applicable) \f$ \frac{\partial h_i}{\partial y_i} \f$.
			  * \param idx_landmark_to_predict The index of the landmark in the map whose prediction is expected as output. For non SLAM-like problems, this will be zero and the expected output is for the whole state vector.
			  * \param Hx  The output Jacobian \f$ \frac{\partial h_i}{\partial x} \f$.
			  * \param Hy  The output Jacobian \f$ \frac{\partial h_i}{\partial y_i} \f$.
			  */
			virtual void OnObservationJacobians(
				const size_t &idx_landmark_to_predict,
				KFMatrix_OxV &Hx,
				KFMatrix_OxF &Hy
				) const
			{
				m_user_didnt_implement_jacobian=true;
			}

			/** Only called if using a numeric approximation of the observation Jacobians, this method must return the increments in each dimension of the vehicle state vector while estimating the Jacobian.
			  */
			virtual void OnObservationJacobiansNumericGetIncrements(
					KFArray_VEH  &out_veh_increments,
					KFArray_FEAT &out_feat_increments ) const
			{
				for (size_t i=0;i<VEH_SIZE;i++) out_veh_increments[i] = 1e-6;
				for (size_t i=0;i<FEAT_SIZE;i++) out_feat_increments[i] = 1e-6;
			}

			/** Computes A=A-B, which may need to be re-implemented depending on the topology of the individual scalar components (eg, angles).
			  */
			virtual void OnSubstractObservationVectors(KFArray_OBS &A, const KFArray_OBS &B) const
			{
				A -= B;
			}

			/** If applicable to the given problem, this method implements the inverse observation model needed to extend the "map" with a new "element".
			  * \param in_z The observation vector whose inverse sensor model is to be computed. This is actually one of the vector<> returned by OnGetObservationsAndDataAssociation().
			  * \param out_yn The F-length vector with the inverse observation model \f$ y_n=y(x,z_n) \f$.
			  * \param out_dyn_dxv The \f$F \times V\f$ Jacobian of the inv. sensor model wrt the robot pose \f$ \frac{\partial y_n}{\partial x_v} \f$.
			  * \param out_dyn_dhn The \f$F \times O\f$ Jacobian of the inv. sensor model wrt the observation vector \f$ \frac{\partial y_n}{\partial h_n} \f$.
			  *
			  *  - O: OBS_SIZE
			  *  - V: VEH_SIZE
			  *  - F: FEAT_SIZE
			  *
			  * \note OnNewLandmarkAddedToMap will be also called after calling this method if a landmark is actually being added to the map.
			  * \deprecated This version of the method is deprecated. The alternative method is preferred to allow a greater flexibility.
			  */
			virtual void  OnInverseObservationModel(
				const KFArray_OBS & in_z,
				KFArray_FEAT  & out_yn,
				KFMatrix_FxV  & out_dyn_dxv,
				KFMatrix_FxO  & out_dyn_dhn ) const
			{
				MRPT_START
				THROW_EXCEPTION("Inverse sensor model required but not implemented in derived class.")
				MRPT_END
			}

			/** If applicable to the given problem, this method implements the inverse observation model needed to extend the "map" with a new "element".
			  *  The uncertainty in the new map feature comes from two parts: one from the vehicle uncertainty (through the out_dyn_dxv Jacobian),
			  *   and another from the uncertainty in the observation itself. By default, out_use_dyn_dhn_jacobian=true on call, and if it's left at "true",
			  *   the base KalmanFilter class will compute the uncertainty of the landmark relative position from out_dyn_dhn.
			  *  Only in some problems (e.g. MonoSLAM), it'll be needed for the application to directly return the covariance matrix \a out_dyn_dhn_R_dyn_dhnT, which is the equivalent to:
			  *
			  *         \f$ \frac{\partial y_n}{\partial h_n} R \frac{\partial y_n}{\partial h_n}^\top \f$.
			  *
			  *  but may be computed from additional terms, or whatever needed by the user.
			  *
			  * \param in_z The observation vector whose inverse sensor model is to be computed. This is actually one of the vector<> returned by OnGetObservationsAndDataAssociation().
			  * \param out_yn The F-length vector with the inverse observation model \f$ y_n=y(x,z_n) \f$.
			  * \param out_dyn_dxv The \f$F \times V\f$ Jacobian of the inv. sensor model wrt the robot pose \f$ \frac{\partial y_n}{\partial x_v} \f$.
			  * \param out_dyn_dhn The \f$F \times O\f$ Jacobian of the inv. sensor model wrt the observation vector \f$ \frac{\partial y_n}{\partial h_n} \f$.
			  * \param out_dyn_dhn_R_dyn_dhnT See the discussion above.
			  *
			  *  - O: OBS_SIZE
			  *  - V: VEH_SIZE
			  *  - F: FEAT_SIZE
			  *
			  * \note OnNewLandmarkAddedToMap will be also called after calling this method if a landmark is actually being added to the map.
			  */
			virtual void  OnInverseObservationModel(
				const KFArray_OBS & in_z,
				KFArray_FEAT  & out_yn,
				KFMatrix_FxV  & out_dyn_dxv,
				KFMatrix_FxO  & out_dyn_dhn,
				KFMatrix_FxF  & out_dyn_dhn_R_dyn_dhnT,
				bool          & out_use_dyn_dhn_jacobian
				 ) const
			{
				MRPT_START
				OnInverseObservationModel(in_z,out_yn,out_dyn_dxv,out_dyn_dhn);
				out_use_dyn_dhn_jacobian=true;
				MRPT_END
			}

			/** If applicable to the given problem, do here any special handling of adding a new landmark to the map.
			  * \param in_obsIndex The index of the observation whose inverse sensor is to be computed. It corresponds to the row in in_z where the observation can be found.
			  * \param in_idxNewFeat The index that this new feature will have in the state vector (0:just after the vehicle state, 1: after that,...). Save this number so data association can be done according to these indices.
			  * \sa OnInverseObservationModel
			  */
			virtual void OnNewLandmarkAddedToMap(
				const size_t	in_obsIdx,
				const size_t	in_idxNewFeat )
			{
				// Do nothing in this base class.
			}

			/** This method is called after the prediction and after the update, to give the user an opportunity to normalize the state vector (eg, keep angles within -pi,pi range) if the application requires it.
			  */
			virtual void OnNormalizeStateVector()
			{
				// Do nothing in this base class.
			}

			/** This method is called after finishing one KF iteration and before returning from runOneKalmanIteration().
			  */
			virtual void OnPostIteration()
			{
				// Do nothing in this base class.
			}

			/** @}
			 */

		public:
			CKalmanFilterCapable() {} //!< Default constructor
			virtual ~CKalmanFilterCapable() {}  //!< Destructor

			mrpt::utils::CTimeLogger &getProfiler() { return m_timLogger; }

			TKF_options KF_options;	//!< Generic options for the Kalman Filter algorithm itself.


		private:
			//  "Local" variables to runOneKalmanIteration, declared here to avoid
			//   allocating them over and over again with each call.
			//  (The variables that go into the stack remains in the function body)
			vector_KFArray_OBS		all_predictions;
			vector_size_t  			predictLMidxs;
			KFMatrix 				dh_dx;
			KFMatrix 				dh_dx_full;
			vector_size_t 			idxs;
			KFMatrix  				S;
			KFMatrix 				Pkk_subset;
			vector_KFArray_OBS 		Z;		// Each entry is one observation:
			KFMatrix 				K; 		// Kalman gain
			KFMatrix 				S_1; 	// Inverse of S
			KFMatrix 				dh_dx_full_obs;
			KFMatrix				aux_K_dh_dx;

		protected:

			/** The main entry point, executes one complete step: prediction + update.
			  *  It is protected since derived classes must provide a problem-specific entry point for users.
			  *  The exact order in which this method calls the virtual method is explained in http://www.mrpt.org/Kalman_Filters
			  */
			void runOneKalmanIteration()
			{
				MRPT_START

				m_timLogger.enable(KF_options.enable_profiler || KF_options.verbose);
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

				bool skipPrediction=false; // Wether to skip the prediction step (in SLAM this is desired for the first iteration...)

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
							std::make_pair<KFCLASS*,KFArray_ACT>(this,u),
							dfv_dxv);

						if (KF_options.debug_verify_analytic_jacobians)
						{
							KFMatrix_VxV dfv_dxv_gt(UNINITIALIZED_MATRIX);
							OnTransitionJacobian(dfv_dxv_gt);
							if ((dfv_dxv-dfv_dxv_gt).Abs().sumAll()>KF_options.debug_verify_analytic_jacobians_threshold)
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
					m_pkk.block(0,0,VEH_SIZE,VEH_SIZE) =
						Q +
						dfv_dxv * m_pkk.block(0,0,VEH_SIZE,VEH_SIZE) * dfv_dxv.transpose();

					// ====================================
					//  3.2:  All Pxy_i
					// ====================================
					// Now, update the cov. of landmarks, if any:
					KFMatrix_VxF aux;
					for (size_t i=0 ; i<N_map ; i++)
					{
						// aux = dfv_dxv(...) * m_pkk(...)
						dfv_dxv.multiply_subMatrix(
							m_pkk,
							aux,  // Output
							VEH_SIZE+i*FEAT_SIZE,   // Offset col
							0,				                 // Offset row
							FEAT_SIZE                        // Number of columns desired in output
						);

						m_pkk.insertMatrix         (0,                VEH_SIZE+i*FEAT_SIZE,  aux );
						m_pkk.insertMatrixTranspose(VEH_SIZE+i*FEAT_SIZE, 0               ,  aux );
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
				//  dh_dx: O·M x V+M·F
				//      S: O·M x O·M
				//  M = |predictLMidxs|
				//  O=size of each observation.
				//  F=size of features in the map
				//

				// This is the beginning of a loop if we are doing sequential
				//  data association, where after each incorporation of an observation
				//  into the filter we recompute the vehicle state, then reconsider the
				//  data associations:
				// TKFFusionMethod fusion_strategy


				m_timLogger.enter("KF:5.build Jacobians");

				size_t N_pred = FEAT_SIZE==0 ?
					1 /* In non-SLAM problems, there'll be only 1 fixed observation */ :
					predictLMidxs.size();

				vector_int  data_association;  // -1: New map feature.>=0: Indexes in the state vector

				// The next loop will only do more than one iteration if the heuristic in OnPreComputingPredictions() fails, 
				//  which will be detected by the addition of extra landmarks to predict into "missing_predictions_to_add"
				std::vector<size_t> missing_predictions_to_add;

				// Indices in xkk (& Pkk) that are involved in this observation (used below).
				idxs.clear();
				idxs.reserve(VEH_SIZE+N_pred*FEAT_SIZE);
				for (size_t i=0;i<VEH_SIZE;i++) idxs.push_back(i);

				dh_dx.zeros(N_pred*OBS_SIZE, VEH_SIZE + FEAT_SIZE * N_pred ); // Init to zeros.
				dh_dx_full.zeros(N_pred*OBS_SIZE, VEH_SIZE + FEAT_SIZE * N_map ); // Init to zeros.

				size_t first_new_pred = 0; // This will be >0 only if we perform multiple loops due to failures in the prediction heuristic.

				do  
				{
					if (!missing_predictions_to_add.empty())
					{
						const size_t nNew = missing_predictions_to_add.size();
						printf_debug("[KF] *Performance Warning*: %u LMs were not correctly predicted by OnPreComputingPredictions()\n",static_cast<unsigned int>(nNew));

						ASSERTDEB_(FEAT_SIZE!=0)
						for (size_t j=0;j<nNew;j++)
							predictLMidxs.push_back( missing_predictions_to_add[j] );

						N_pred = predictLMidxs.size();
						missing_predictions_to_add.clear();
					}

					dh_dx.setSize(N_pred*OBS_SIZE, VEH_SIZE + FEAT_SIZE * N_pred ); // Pad with zeros.
					dh_dx_full.setSize(N_pred*OBS_SIZE, VEH_SIZE + FEAT_SIZE * N_map ); // Pad with zeros.

					for (size_t i=first_new_pred;i<N_pred;++i)
					{
						const size_t lm_idx = FEAT_SIZE==0 ? 0 : predictLMidxs[i];
						KFMatrix_OxV Hx(UNINITIALIZED_MATRIX);
						KFMatrix_OxF Hy(UNINITIALIZED_MATRIX);

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
								std::make_pair<KFCLASS*,size_t>(this,lm_idx),
								Hx);
							// The state vector was temporarily modified by KF_aux_estimate_*, restore it:
							::memcpy(&m_xkk[0],&x_vehicle[0],sizeof(m_xkk[0])*VEH_SIZE);

							mrpt::math::estimateJacobian(
								x_feat,
								&KF_aux_estimate_obs_Hy_jacobian,
								feat_increments,
								std::make_pair<KFCLASS*,size_t>(this,lm_idx),
								Hy);
							// The state vector was temporarily modified by KF_aux_estimate_*, restore it:
							::memcpy(&m_xkk[lm_idx_in_statevector],&x_feat[0],sizeof(m_xkk[0])*FEAT_SIZE);

							if (KF_options.debug_verify_analytic_jacobians)
							{
								KFMatrix_OxV Hx_gt(UNINITIALIZED_MATRIX);
								KFMatrix_OxF Hy_gt(UNINITIALIZED_MATRIX);
								OnObservationJacobians(lm_idx,Hx_gt,Hy_gt);
								if ((Hx-Hx_gt).Abs().sumAll()>KF_options.debug_verify_analytic_jacobians_threshold) {
									std::cerr << "[KalmanFilter] ERROR: User analytical observation Hx Jacobians are wrong: \n"
										<< " Real Hx: \n" << Hx << "\n Analytical Hx:\n" << Hx_gt << "Diff:\n" << Hx-Hx_gt << "\n";
									THROW_EXCEPTION("ERROR: User analytical observation Hx Jacobians are wrong (More details dumped to cerr)")
								}
								if ((Hy-Hy_gt).Abs().sumAll()>KF_options.debug_verify_analytic_jacobians_threshold) {
									std::cerr << "[KalmanFilter] ERROR: User analytical observation Hy Jacobians are wrong: \n"
										<< " Real Hy: \n" << Hy << "\n Analytical Hx:\n" << Hy_gt << "Diff:\n" << Hy-Hy_gt << "\n";
									THROW_EXCEPTION("ERROR: User analytical observation Hy Jacobians are wrong (More details dumped to cerr)")
								}
							}
						}

						dh_dx.insertMatrix(i*OBS_SIZE,0, Hx);
						if (FEAT_SIZE!=0)
							dh_dx.insertMatrix(i*OBS_SIZE,VEH_SIZE+i*FEAT_SIZE, Hy);

						dh_dx_full.insertMatrix(i*OBS_SIZE,0, Hx);
						if (FEAT_SIZE!=0)
						{
							dh_dx_full.insertMatrix(i*OBS_SIZE,VEH_SIZE+lm_idx*FEAT_SIZE, Hy);

							for (size_t k=0;k<FEAT_SIZE;k++)
								idxs.push_back(k+VEH_SIZE+FEAT_SIZE*lm_idx);
						}
					}
					m_timLogger.leave("KF:5.build Jacobians");

					// Compute S:  S = H P ~H + R
					// *TODO*: This can be accelerated by exploiting the sparsity of dh_dx!!!
					// ------------------------------------
					m_timLogger.enter("KF:6.build S");

					S.setSize(N_pred*OBS_SIZE,N_pred*OBS_SIZE);

					// (TODO: Implement multiply_HCHt for a subset of COV directly.)
					// Extract the subset of m_Pkk that is involved in this observation:
					m_pkk.extractSubmatrixSymmetrical(idxs,Pkk_subset);

					// S = dh_dx * m_pkk(subset) * (~dh_dx);
					dh_dx.multiply_HCHt(Pkk_subset,S);

					// Sum the "R" term:
					if ( FEAT_SIZE>0 )
					{
						for (size_t i=0;i<N_pred;++i)
						{
							const size_t obs_idx_off = i*OBS_SIZE;
							for (size_t j=0;j<OBS_SIZE;j++)
								for (size_t k=0;k<OBS_SIZE;k++)
									S.get_unsafe(obs_idx_off+j,obs_idx_off+k) += R.get_unsafe(j,k);
						}
					}
					else
					{	// Not a SLAM-like EKF problem:
						ASSERTDEB_(S.getColCount() == OBS_SIZE );
						S+=R;
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
								binder1st<equal_to<int> >(equal_to<int>(),-1) ) ) );

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

									const size_t row_len = VEH_SIZE + FEAT_SIZE * N_map;

									for (size_t i=0;i<data_association.size();++i)
									{
										if (data_association[i]<0) continue;

										const size_t assoc_idx_in_map = static_cast<size_t>(data_association[i]);
										const size_t assoc_idx_in_pred = mrpt::utils::find_in_vector(assoc_idx_in_map, predictLMidxs);
										ASSERTMSG_(assoc_idx_in_pred!=string::npos, "OnPreComputingPredictions() didn't recommend the prediction of a landmark which has been actually observed!")
										// TODO: In these cases, extend the prediction right now instead of launching an exception... or is this a bad idea??

										// Build the subset dh_dx_full_obs:
										dh_dx_full_obs.block(S_idxs.size()             ,0, OBS_SIZE, row_len)
										=
										dh_dx_full.block    (assoc_idx_in_pred*OBS_SIZE,0, OBS_SIZE, row_len);
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

									dh_dx_full_obs = dh_dx_full;
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

								//KFMatrix S_1( S_observed.getRowCount(), S_observed.getColCount() );
								S_observed.inv(S_1); // Do NOT call inv_fast since it destroys S
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

									// K * dh_dx_full
									aux_K_dh_dx.multiply(K,dh_dx_full_obs);

									// aux_K_dh_dx  <-- I-aux_K_dh_dx
									const size_t stat_len = aux_K_dh_dx.getColCount();
									for (size_t r=0;r<stat_len;r++)
										for (size_t c=0;c<stat_len;c++)
											if (r==c)
											     aux_K_dh_dx.get_unsafe(r,c)=-aux_K_dh_dx.get_unsafe(r,c) + kftype(1);
											else aux_K_dh_dx.get_unsafe(r,c)=-aux_K_dh_dx.get_unsafe(r,c);

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

								KFMatrix_OxV Hx(UNINITIALIZED_MATRIX);
								KFMatrix_OxF Hy(UNINITIALIZED_MATRIX);
								const size_t i_idx_in_preds = mrpt::utils::find_in_vector(idxInTheFilter,predictLMidxs);
								ASSERTMSG_(i_idx_in_preds!=string::npos, "OnPreComputingPredictions() didn't recommend the prediction of a landmark which has been actually observed!")
								dh_dx.extractMatrix(i_idx_in_preds*OBS_SIZE,0, Hx);
								dh_dx.extractMatrix(i_idx_in_preds*OBS_SIZE,VEH_SIZE+i_idx_in_preds*OBS_SIZE, Hy);


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
										mrpt::dynamicsize_vector<KFTYPE> HAx(OBS_SIZE);
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

					OnNormalizeStateVector();

					// =============================================================
					//  8. INTRODUCE NEW LANDMARKS
					// =============================================================
					if (!data_association.empty())
					{
 						detail::CRunOneKalmanIteration_addNewLandmarks()(*this, Z, data_association,R);
					} // end if data_association!=empty

					if (KF_options.verbose)
					{
						printf_debug("[KF] %u LMs | Pr: %.2fms | Pr.Obs: %.2fms | Obs.DA: %.2fms | Upd: %.2fms\n",
							static_cast<unsigned int>(getNumberOfLandmarksInTheMap()),
							1e3*tim_pred,
							1e3*tim_pred_obs,
							1e3*tim_obs_DA,
							1e3*tim_update
							);
					}

					// Post iteration user code:
					OnPostIteration();

					m_timLogger.leave("KF:complete_step");

					MRPT_END

				} // End of "runOneKalmanIteration"



		private:
			mutable bool m_user_didnt_implement_jacobian;

			/** Auxiliary functions for Jacobian numeric estimation */
			static void KF_aux_estimate_trans_jacobian(
				const KFArray_VEH &x,
				const std::pair<KFCLASS*,KFArray_ACT> &dat,
				KFArray_VEH &out_x)
			{
				bool dumm;
				out_x=x;
				dat.first->OnTransitionModel(dat.second,out_x, dumm);
			}
			static void KF_aux_estimate_obs_Hx_jacobian(
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
			static void KF_aux_estimate_obs_Hy_jacobian(
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


 			friend struct detail::CRunOneKalmanIteration_addNewLandmarks;

		}; // end class

		namespace detail
		{
 			// generic version for SLAM. There is a speciation below for NON-SLAM problems.
 			struct CRunOneKalmanIteration_addNewLandmarks {
 				template <size_t VEH_SIZE, size_t OBS_SIZE, size_t FEAT_SIZE, size_t ACT_SIZE, typename KFTYPE>
 				void operator()(
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
							obj.m_timLogger.enter("KF:9.create new LMs");
							// Add it:

							// Append to map of IDs <-> position in the state vector:
							ASSERTDEB_(FEAT_SIZE>0)
							ASSERTDEB_( 0 == ((obj.m_xkk.size() - VEH_SIZE) % FEAT_SIZE) ) // Sanity test

							const size_t  newIndexInMap = (obj.m_xkk.size() - VEH_SIZE) / FEAT_SIZE;

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
							size_t idx = obj.m_xkk.size();
							obj.m_xkk.resize( obj.m_xkk.size() + FEAT_SIZE );

							for (q=0;q<FEAT_SIZE;q++)
								obj.m_xkk[idx+q] = yn[q];

							// --------------------
							// Append to Pkk:
							// --------------------
							ASSERTDEB_( obj.m_pkk.getColCount()==idx && obj.m_pkk.getRowCount()==idx );

							obj.m_pkk.setSize( idx+FEAT_SIZE,idx+FEAT_SIZE );

							// Fill the Pxyn term:
							// --------------------
							typename KF::KFMatrix_VxV Pxx;
							obj.m_pkk.extractMatrix(0,0,Pxx);
							typename KF::KFMatrix_FxV Pxyn; // Pxyn = dyn_dxv * Pxx
							Pxyn.multiply( dyn_dxv, Pxx );

							obj.m_pkk.insertMatrix( idx,0, Pxyn );
							obj.m_pkk.insertMatrixTranspose( 0,idx, Pxyn );

							// Fill the Pyiyn terms:
							// --------------------
							const size_t nLMs = (idx-VEH_SIZE)/FEAT_SIZE; // Number of previous landmarks:
							for (q=0;q<nLMs;q++)
							{
								typename KF::KFMatrix_VxF  P_x_yq(UNINITIALIZED_MATRIX);
								obj.m_pkk.extractMatrix(0,VEH_SIZE+q*FEAT_SIZE,P_x_yq) ;

								typename KF::KFMatrix_FxF P_cross(UNINITIALIZED_MATRIX);
								P_cross.multiply(dyn_dxv, P_x_yq );

								obj.m_pkk.insertMatrix(idx,VEH_SIZE+q*FEAT_SIZE, P_cross );
								obj.m_pkk.insertMatrixTranspose(VEH_SIZE+q*FEAT_SIZE,idx, P_cross );
							} // end each previous LM(q)

							// Fill the Pynyn term:
							//  P_yn_yn =  (dyn_dxv * Pxx * ~dyn_dxv) + (dyn_dhn * R * ~dyn_dhn);
							// --------------------
							typename KF::KFMatrix_FxF P_yn_yn(UNINITIALIZED_MATRIX);
							dyn_dxv.multiply_HCHt(Pxx,  P_yn_yn);
							if (use_dyn_dhn_jacobian)
									dyn_dhn.multiply_HCHt(R, P_yn_yn, true); // Accumulate in P_yn_yn
							else 	P_yn_yn+=dyn_dhn_R_dyn_dhnT;

							obj.m_pkk.insertMatrix(idx,idx, P_yn_yn );

							obj.m_timLogger.leave("KF:9.create new LMs");
						}
					}
				}

 				template <size_t VEH_SIZE, size_t OBS_SIZE, size_t ACT_SIZE, typename KFTYPE>
 				void operator()(
 					CKalmanFilterCapable<VEH_SIZE,OBS_SIZE,0 /* FEAT_SIZE=0 */,ACT_SIZE,KFTYPE> &obj,
 					const typename CKalmanFilterCapable<VEH_SIZE,OBS_SIZE,0 /* FEAT_SIZE=0 */,ACT_SIZE,KFTYPE>::vector_KFArray_OBS & Z,
 					const vector_int       &data_association,
 					const typename CKalmanFilterCapable<VEH_SIZE,OBS_SIZE,0 /* FEAT_SIZE=0 */,ACT_SIZE,KFTYPE>::KFMatrix_OxO		&R
  				)
 				{
 					// Do nothing: this is NOT a SLAM problem.
 				}

 			}; // end runOneKalmanIteration_addNewLandmarks

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

	} // end namespace

	// Specializations MUST occur at the same namespace:
	namespace utils
	{
		template <>
		struct TEnumTypeFiller<bayes::TKFMethod>
		{
			typedef bayes::TKFMethod enum_t;
			static void fill(bimap<enum_t,std::string>  &m_map)
			{
				m_map.insert(bayes::kfEKFNaive,          "kfEKFNaive");
				m_map.insert(bayes::kfEKFAlaDavison,     "kfEKFAlaDavison");
				m_map.insert(bayes::kfIKFFull,           "kfIKFFull");
				m_map.insert(bayes::kfIKF,               "kfIKF");
			}
		};
	} // End of namespace

} // end namespace

#endif
