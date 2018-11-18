/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/CMatrixFixedNumeric.h>
#include <mrpt/math/CMatrixTemplateNumeric.h>
#include <mrpt/math/CArrayNumeric.h>
#include <mrpt/math/num_jacobian.h>
#include <mrpt/math/utils.h>
#include <mrpt/math/num_jacobian.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/core/aligned_std_vector.h>
#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/containers/stl_containers_utils.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/containers/stl_containers_utils.h>  // find_in_vector
#include <mrpt/system/CTicTac.h>
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/typemeta/TEnumType.h>
#include <mrpt/system/vector_loadsave.h>

namespace mrpt
{
namespace bayes
{
/** The Kalman Filter algorithm to employ in bayes::CKalmanFilterCapable
 *  For further details on each algorithm see the tutorial:
 * http://www.mrpt.org/Kalman_Filters
 * \sa bayes::CKalmanFilterCapable::KF_options
 * \ingroup mrpt_bayes_grp
 */
enum TKFMethod
{
	kfEKFNaive = 0,
	kfEKFAlaDavison,
	kfIKFFull,
	kfIKF
};

// Forward declaration:
template <
	size_t VEH_SIZE, size_t OBS_SIZE, size_t FEAT_SIZE, size_t ACT_SIZE,
	typename KFTYPE>
class CKalmanFilterCapable;

/** Generic options for the Kalman Filter algorithm in itself.
 * \ingroup mrpt_bayes_grp
 */
struct TKF_options : public mrpt::config::CLoadableOptions
{
	TKF_options(mrpt::system::VerbosityLevel& verb_level_ref)
		: verbosity_level(verb_level_ref)
	{
	}

	void loadFromConfigFile(
		const mrpt::config::CConfigFileBase& iniFile,
		const std::string& section) override
	{
		method = iniFile.read_enum<TKFMethod>(section, "method", method);
		verbosity_level = iniFile.read_enum<mrpt::system::VerbosityLevel>(
			section, "verbosity_level", verbosity_level);
		MRPT_LOAD_CONFIG_VAR(IKF_iterations, int, iniFile, section);
		MRPT_LOAD_CONFIG_VAR(enable_profiler, bool, iniFile, section);
		MRPT_LOAD_CONFIG_VAR(
			use_analytic_transition_jacobian, bool, iniFile, section);
		MRPT_LOAD_CONFIG_VAR(
			use_analytic_observation_jacobian, bool, iniFile, section);
		MRPT_LOAD_CONFIG_VAR(
			debug_verify_analytic_jacobians, bool, iniFile, section);
		MRPT_LOAD_CONFIG_VAR(
			debug_verify_analytic_jacobians_threshold, double, iniFile,
			section);
	}

	/** This method must display clearly all the contents of the structure in
	 * textual form, sending it to a CStream. */
	void dumpToTextStream(std::ostream& out) const override
	{
		out << mrpt::format("\n----------- [TKF_options] ------------ \n\n");
		out << mrpt::format(
			"method                                  = %s\n",
			mrpt::typemeta::TEnumType<TKFMethod>::value2name(method).c_str());
		out << mrpt::format(
			"verbosity_level                         = %s\n",
			mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>::value2name(
				verbosity_level)
				.c_str());
		out << mrpt::format(
			"IKF_iterations                          = %i\n", IKF_iterations);
		out << mrpt::format(
			"enable_profiler                         = %c\n",
			enable_profiler ? 'Y' : 'N');
		out << mrpt::format("\n");
	}

	/** The method to employ (default: kfEKFNaive) */
	TKFMethod method{kfEKFNaive};
	mrpt::system::VerbosityLevel& verbosity_level;
	/** Number of refinement iterations, only for the IKF method. */
	int IKF_iterations{5};
	/** If enabled (default=false), detailed timing information will be dumped
	 * to the console thru a CTimerLog at the end of the execution. */
	bool enable_profiler{false};
	/** (default=true) If true, OnTransitionJacobian will be called; otherwise,
	 * the Jacobian will be estimated from a numeric approximation by calling
	 * several times to OnTransitionModel. */
	bool use_analytic_transition_jacobian{true};
	/** (default=true) If true, OnObservationJacobians will be called;
	 * otherwise, the Jacobian will be estimated from a numeric approximation by
	 * calling several times to OnObservationModel. */
	bool use_analytic_observation_jacobian{true};
	/** (default=false) If true, will compute all the Jacobians numerically and
	 * compare them to the analytical ones, throwing an exception on mismatch.
	 */
	bool debug_verify_analytic_jacobians{false};
	/** (default-1e-2) Sets the threshold for the difference between the
	 * analytic and the numerical jacobians */
	double debug_verify_analytic_jacobians_threshold{1e-2};
};

/** Auxiliary functions, for internal usage of MRPT classes */
namespace detail
{
// Auxiliary functions.
template <
	size_t VEH_SIZE, size_t OBS_SIZE, size_t FEAT_SIZE, size_t ACT_SIZE,
	typename KFTYPE>
inline size_t getNumberOfLandmarksInMap(
	const CKalmanFilterCapable<VEH_SIZE, OBS_SIZE, FEAT_SIZE, ACT_SIZE, KFTYPE>&
		obj);
// Specialization:
template <size_t VEH_SIZE, size_t OBS_SIZE, size_t ACT_SIZE, typename KFTYPE>
inline size_t getNumberOfLandmarksInMap(
	const CKalmanFilterCapable<
		VEH_SIZE, OBS_SIZE, 0 /*FEAT_SIZE*/, ACT_SIZE, KFTYPE>& obj);

template <
	size_t VEH_SIZE, size_t OBS_SIZE, size_t FEAT_SIZE, size_t ACT_SIZE,
	typename KFTYPE>
inline bool isMapEmpty(
	const CKalmanFilterCapable<VEH_SIZE, OBS_SIZE, FEAT_SIZE, ACT_SIZE, KFTYPE>&
		obj);
// Specialization:
template <size_t VEH_SIZE, size_t OBS_SIZE, size_t ACT_SIZE, typename KFTYPE>
inline bool isMapEmpty(
	const CKalmanFilterCapable<
		VEH_SIZE, OBS_SIZE, 0 /*FEAT_SIZE*/, ACT_SIZE, KFTYPE>& obj);

template <
	size_t VEH_SIZE, size_t OBS_SIZE, size_t FEAT_SIZE, size_t ACT_SIZE,
	typename KFTYPE>
void addNewLandmarks(
	CKalmanFilterCapable<VEH_SIZE, OBS_SIZE, FEAT_SIZE, ACT_SIZE, KFTYPE>& obj,
	const typename CKalmanFilterCapable<
		VEH_SIZE, OBS_SIZE, FEAT_SIZE, ACT_SIZE, KFTYPE>::vector_KFArray_OBS& Z,
	const std::vector<int>& data_association,
	const typename CKalmanFilterCapable<
		VEH_SIZE, OBS_SIZE, FEAT_SIZE, ACT_SIZE, KFTYPE>::KFMatrix_OxO& R);
template <size_t VEH_SIZE, size_t OBS_SIZE, size_t ACT_SIZE, typename KFTYPE>
void addNewLandmarks(
	CKalmanFilterCapable<
		VEH_SIZE, OBS_SIZE, 0 /* FEAT_SIZE=0 */, ACT_SIZE, KFTYPE>& obj,
	const typename CKalmanFilterCapable<
		VEH_SIZE, OBS_SIZE, 0 /* FEAT_SIZE=0 */, ACT_SIZE,
		KFTYPE>::vector_KFArray_OBS& Z,
	const std::vector<int>& data_association,
	const typename CKalmanFilterCapable<
		VEH_SIZE, OBS_SIZE, 0 /* FEAT_SIZE=0 */, ACT_SIZE,
		KFTYPE>::KFMatrix_OxO& R);
}  // namespace detail

/** Virtual base for Kalman Filter (EKF,IEKF,UKF) implementations.
 *   This base class stores the state vector and covariance matrix of the
 *system. It has virtual methods that must be completed
 *    by derived classes to address a given filtering problem. The main entry
 *point of the algorithm is CKalmanFilterCapable::runOneKalmanIteration, which
 *    should be called AFTER setting the desired filter options in KF_options,
 *as well as any options in the derived class.
 *   Note that the main entry point is protected, so derived classes must offer
 *another method more specific to a given problem which, internally, calls
 *runOneKalmanIteration.
 *
 *  For further details and examples, check out the tutorial:
 *http://www.mrpt.org/Kalman_Filters
 *
 *  The Kalman filter algorithms are generic, but this implementation is biased
 *to ease the implementation
 *  of SLAM-like problems. However, it can be also applied to many generic
 *problems not related to robotics or SLAM.
 *
 *  The meaning of the template parameters is:
 *	- VEH_SIZE: The dimension of the "vehicle state": either the full state
 *vector or the "vehicle" part if applicable.
 *	- OBS_SIZE: The dimension of each observation (eg, 2 for pixel coordinates,
 *3 for 3D coordinates,etc).
 *	- FEAT_SIZE: The dimension of the features in the system state (the "map"),
 *or 0 if not applicable (the default if not implemented).
 *	- ACT_SIZE: The dimension of each "action" u_k (or 0 if not applicable).
 *	- KFTYPE: The numeric type of the matrices (default: double)
 *
 * Revisions:
 *	- 2007: Antonio J. Ortiz de Galisteo (AJOGD)
 *	- 2008/FEB: All KF classes corrected, reorganized, and rewritten (JLBC).
 *	- 2008/MAR: Implemented IKF (JLBC).
 *	- 2009/DEC: Totally rewritten as a generic template using fixed-size
 *matrices where possible (JLBC).
 *
 *  \sa mrpt::slam::CRangeBearingKFSLAM, mrpt::slam::CRangeBearingKFSLAM2D
 * \ingroup mrpt_bayes_grp
 */
template <
	size_t VEH_SIZE, size_t OBS_SIZE, size_t FEAT_SIZE, size_t ACT_SIZE,
	typename KFTYPE = double>
class CKalmanFilterCapable : public mrpt::system::COutputLogger
{
   public:
	static inline size_t get_vehicle_size() { return VEH_SIZE; }
	static inline size_t get_observation_size() { return OBS_SIZE; }
	static inline size_t get_feature_size() { return FEAT_SIZE; }
	static inline size_t get_action_size() { return ACT_SIZE; }
	inline size_t getNumberOfLandmarksInTheMap() const
	{
		return detail::getNumberOfLandmarksInMap(*this);
	}
	inline bool isMapEmpty() const { return detail::isMapEmpty(*this); }
	/** The numeric type used in the Kalman Filter (default=double) */
	using kftype = KFTYPE;
	/** My class, in a shorter name! */
	using KFCLASS =
		CKalmanFilterCapable<VEH_SIZE, OBS_SIZE, FEAT_SIZE, ACT_SIZE, KFTYPE>;

	// ---------- Many useful typedefs to short the notation a bit... --------
	using KFVector = Eigen::Matrix<KFTYPE, Eigen::Dynamic, 1>;
	using KFMatrix = mrpt::math::CMatrixTemplateNumeric<KFTYPE>;

	using KFMatrix_VxV =
		mrpt::math::CMatrixFixedNumeric<KFTYPE, VEH_SIZE, VEH_SIZE>;
	using KFMatrix_OxO =
		mrpt::math::CMatrixFixedNumeric<KFTYPE, OBS_SIZE, OBS_SIZE>;
	using KFMatrix_FxF =
		mrpt::math::CMatrixFixedNumeric<KFTYPE, FEAT_SIZE, FEAT_SIZE>;
	using KFMatrix_AxA =
		mrpt::math::CMatrixFixedNumeric<KFTYPE, ACT_SIZE, ACT_SIZE>;

	using KFMatrix_VxO =
		mrpt::math::CMatrixFixedNumeric<KFTYPE, VEH_SIZE, OBS_SIZE>;
	using KFMatrix_VxF =
		mrpt::math::CMatrixFixedNumeric<KFTYPE, VEH_SIZE, FEAT_SIZE>;
	using KFMatrix_FxV =
		mrpt::math::CMatrixFixedNumeric<KFTYPE, FEAT_SIZE, VEH_SIZE>;
	using KFMatrix_FxO =
		mrpt::math::CMatrixFixedNumeric<KFTYPE, FEAT_SIZE, OBS_SIZE>;
	using KFMatrix_OxF =
		mrpt::math::CMatrixFixedNumeric<KFTYPE, OBS_SIZE, FEAT_SIZE>;
	using KFMatrix_OxV =
		mrpt::math::CMatrixFixedNumeric<KFTYPE, OBS_SIZE, VEH_SIZE>;

	using KFArray_VEH = mrpt::math::CArrayNumeric<KFTYPE, VEH_SIZE>;
	using KFArray_ACT = mrpt::math::CArrayNumeric<KFTYPE, ACT_SIZE>;
	using KFArray_OBS = mrpt::math::CArrayNumeric<KFTYPE, OBS_SIZE>;
	using vector_KFArray_OBS = mrpt::aligned_std_vector<KFArray_OBS>;
	using KFArray_FEAT = mrpt::math::CArrayNumeric<KFTYPE, FEAT_SIZE>;

	inline size_t getStateVectorLength() const { return m_xkk.size(); }
	inline KFVector& internal_getXkk() { return m_xkk; }
	inline KFMatrix& internal_getPkk() { return m_pkk; }
	/** Returns the mean of the estimated value of the idx'th landmark (not
	 * applicable to non-SLAM problems).
	 * \exception std::exception On idx>= getNumberOfLandmarksInTheMap()
	 */
	inline void getLandmarkMean(size_t idx, KFArray_FEAT& feat) const
	{
		ASSERT_(idx < getNumberOfLandmarksInTheMap());
		::memcpy(
			&feat[0], &m_xkk[VEH_SIZE + idx * FEAT_SIZE],
			FEAT_SIZE * sizeof(m_xkk[0]));
	}
	/** Returns the covariance of the idx'th landmark (not applicable to
	 * non-SLAM problems).
	 * \exception std::exception On idx>= getNumberOfLandmarksInTheMap()
	 */
	inline void getLandmarkCov(size_t idx, KFMatrix_FxF& feat_cov) const
	{
		m_pkk.extractMatrix(
			VEH_SIZE + idx * FEAT_SIZE, VEH_SIZE + idx * FEAT_SIZE, feat_cov);
	}

   protected:
	/** @name Kalman filter state
		@{ */

	/** The system state vector. */
	KFVector m_xkk;
	/** The system full covariance matrix. */
	KFMatrix m_pkk;

	/** @} */

	mrpt::system::CTimeLogger m_timLogger;

	/** @name Virtual methods for Kalman Filter implementation
		@{
	 */

	/** Must return the action vector u.
	 * \param out_u The action vector which will be passed to OnTransitionModel
	 */
	virtual void OnGetAction(KFArray_ACT& out_u) const = 0;

	/** Implements the transition model \f$ \hat{x}_{k|k-1} = f(
	 * \hat{x}_{k-1|k-1}, u_k ) \f$
	 * \param in_u The vector returned by OnGetAction.
	 * \param inout_x At input has \f[ \hat{x}_{k-1|k-1} \f] , at output must
	 * have \f$ \hat{x}_{k|k-1} \f$ .
	 * \param out_skip Set this to true if for some reason you want to skip the
	 * prediction step (to do not modify either the vector or the covariance).
	 * Default:false
	 * \note Even if you return "out_skip=true", the value of "inout_x" MUST be
	 * updated as usual (this is to allow numeric approximation of Jacobians).
	 */
	virtual void OnTransitionModel(
		const KFArray_ACT& in_u, KFArray_VEH& inout_x,
		bool& out_skipPrediction) const = 0;

	/** Implements the transition Jacobian \f$ \frac{\partial f}{\partial x} \f$
	 * \param out_F Must return the Jacobian.
	 *  The returned matrix must be \f$V \times V\f$ with V being either the
	 * size of the whole state vector (for non-SLAM problems) or VEH_SIZE (for
	 * SLAM problems).
	 */
	virtual void OnTransitionJacobian(KFMatrix_VxV& out_F) const
	{
		MRPT_UNUSED_PARAM(out_F);
		m_user_didnt_implement_jacobian = true;
	}

	/** Only called if using a numeric approximation of the transition Jacobian,
	 * this method must return the increments in each dimension of the vehicle
	 * state vector while estimating the Jacobian.
	 */
	virtual void OnTransitionJacobianNumericGetIncrements(
		KFArray_VEH& out_increments) const
	{
		for (size_t i = 0; i < VEH_SIZE; i++) out_increments[i] = 1e-6;
	}

	/** Implements the transition noise covariance \f$ Q_k \f$
	 * \param out_Q Must return the covariance matrix.
	 *  The returned matrix must be of the same size than the jacobian from
	 * OnTransitionJacobian
	 */
	virtual void OnTransitionNoise(KFMatrix_VxV& out_Q) const = 0;

	/** This will be called before OnGetObservationsAndDataAssociation to allow
	 * the application to reduce the number of covariance landmark predictions
	 * to be made.
	 *  For example, features which are known to be "out of sight" shouldn't be
	 * added to the output list to speed up the calculations.
	 * \param in_all_prediction_means The mean of each landmark predictions;
	 * the computation or not of the corresponding covariances is what we're
	 * trying to determined with this method.
	 * \param out_LM_indices_to_predict The list of landmark indices in the map
	 * [0,getNumberOfLandmarksInTheMap()-1] that should be predicted.
	 * \note This is not a pure virtual method, so it should be implemented
	 * only if desired. The default implementation returns a vector with all the
	 * landmarks in the map.
	 * \sa OnGetObservations, OnDataAssociation
	 */
	virtual void OnPreComputingPredictions(
		const vector_KFArray_OBS& in_all_prediction_means,
		std::vector<size_t>& out_LM_indices_to_predict) const
	{
		MRPT_UNUSED_PARAM(in_all_prediction_means);
		// Default: all of them:
		const size_t N = this->getNumberOfLandmarksInTheMap();
		out_LM_indices_to_predict.resize(N);
		for (size_t i = 0; i < N; i++) out_LM_indices_to_predict[i] = i;
	}

	/** Return the observation NOISE covariance matrix, that is, the model of
	 * the Gaussian additive noise of the sensor.
	 * \param out_R The noise covariance matrix. It might be non diagonal, but
	 * it'll usually be.
	 * \note Upon call, it can be assumed that the previous contents of out_R
	 * are all zeros.
	 */
	virtual void OnGetObservationNoise(KFMatrix_OxO& out_R) const = 0;

	/** This is called between the KF prediction step and the update step, and
	 * the application must return the observations and, when applicable, the
	 * data association between these observations and the current map.
	 *
	 * \param out_z N vectors, each for one "observation" of length OBS_SIZE, N
	 * being the number of "observations": how many observed landmarks for a
	 * map, or just one if not applicable.
	 * \param out_data_association An empty vector or, where applicable, a
	 * vector where the i'th element corresponds to the position of the
	 * observation in the i'th row of out_z within the system state vector (in
	 * the range [0,getNumberOfLandmarksInTheMap()-1]), or -1 if it is a new map
	 * element and we want to insert it at the end of this KF iteration.
	 * \param in_all_predictions A vector with the prediction of ALL the
	 * landmarks in the map. Note that, in contrast, in_S only comprises a
	 * subset of all the landmarks.
	 * \param in_S The full covariance matrix of the observation predictions
	 * (i.e. the "innovation covariance matrix"). This is a M*O x M*O matrix
	 * with M=length of "in_lm_indices_in_S".
	 * \param in_lm_indices_in_S The indices of the map landmarks (range
	 * [0,getNumberOfLandmarksInTheMap()-1]) that can be found in the matrix
	 * in_S.
	 *
	 *  This method will be called just once for each complete KF iteration.
	 * \note It is assumed that the observations are independent, i.e. there
	 * are NO cross-covariances between them.
	 */
	virtual void OnGetObservationsAndDataAssociation(
		vector_KFArray_OBS& out_z, std::vector<int>& out_data_association,
		const vector_KFArray_OBS& in_all_predictions, const KFMatrix& in_S,
		const std::vector<size_t>& in_lm_indices_in_S,
		const KFMatrix_OxO& in_R) = 0;

	/** Implements the observation prediction \f$ h_i(x) \f$.
	 * \param idx_landmark_to_predict The indices of the landmarks in the map
	 * whose predictions are expected as output. For non SLAM-like problems,
	 * this input value is undefined and the application should just generate
	 * one observation for the given problem.
	 * \param out_predictions The predicted observations.
	 */
	virtual void OnObservationModel(
		const std::vector<size_t>& idx_landmarks_to_predict,
		vector_KFArray_OBS& out_predictions) const = 0;

	/** Implements the observation Jacobians \f$ \frac{\partial h_i}{\partial x}
	 * \f$ and (when applicable) \f$ \frac{\partial h_i}{\partial y_i} \f$.
	 * \param idx_landmark_to_predict The index of the landmark in the map
	 * whose prediction is expected as output. For non SLAM-like problems, this
	 * will be zero and the expected output is for the whole state vector.
	 * \param Hx  The output Jacobian \f$ \frac{\partial h_i}{\partial x} \f$.
	 * \param Hy  The output Jacobian \f$ \frac{\partial h_i}{\partial y_i}
	 * \f$.
	 */
	virtual void OnObservationJacobians(
		const size_t& idx_landmark_to_predict, KFMatrix_OxV& Hx,
		KFMatrix_OxF& Hy) const
	{
		MRPT_UNUSED_PARAM(idx_landmark_to_predict);
		MRPT_UNUSED_PARAM(Hx);
		MRPT_UNUSED_PARAM(Hy);
		m_user_didnt_implement_jacobian = true;
	}

	/** Only called if using a numeric approximation of the observation
	 * Jacobians, this method must return the increments in each dimension of
	 * the vehicle state vector while estimating the Jacobian.
	 */
	virtual void OnObservationJacobiansNumericGetIncrements(
		KFArray_VEH& out_veh_increments,
		KFArray_FEAT& out_feat_increments) const
	{
		for (size_t i = 0; i < VEH_SIZE; i++) out_veh_increments[i] = 1e-6;
		for (size_t i = 0; i < FEAT_SIZE; i++) out_feat_increments[i] = 1e-6;
	}

	/** Computes A=A-B, which may need to be re-implemented depending on the
	 * topology of the individual scalar components (eg, angles).
	 */
	virtual void OnSubstractObservationVectors(
		KFArray_OBS& A, const KFArray_OBS& B) const
	{
		A -= B;
	}

   public:
	/** If applicable to the given problem, this method implements the inverse
	 * observation model needed to extend the "map" with a new "element".
	 * \param in_z The observation vector whose inverse sensor model is to be
	 * computed. This is actually one of the vector<> returned by
	 * OnGetObservationsAndDataAssociation().
	 * \param out_yn The F-length vector with the inverse observation model \f$
	 * y_n=y(x,z_n) \f$.
	 * \param out_dyn_dxv The \f$F \times V\f$ Jacobian of the inv. sensor
	 * model wrt the robot pose \f$ \frac{\partial y_n}{\partial x_v} \f$.
	 * \param out_dyn_dhn The \f$F \times O\f$ Jacobian of the inv. sensor
	 * model wrt the observation vector \f$ \frac{\partial y_n}{\partial h_n}
	 * \f$.
	 *
	 *  - O: OBS_SIZE
	 *  - V: VEH_SIZE
	 *  - F: FEAT_SIZE
	 *
	 * \note OnNewLandmarkAddedToMap will be also called after calling this
	 * method if a landmark is actually being added to the map.
	 * \deprecated This version of the method is deprecated. The alternative
	 * method is preferred to allow a greater flexibility.
	 */
	virtual void OnInverseObservationModel(
		const KFArray_OBS& in_z, KFArray_FEAT& out_yn,
		KFMatrix_FxV& out_dyn_dxv, KFMatrix_FxO& out_dyn_dhn) const
	{
		MRPT_UNUSED_PARAM(in_z);
		MRPT_UNUSED_PARAM(out_yn);
		MRPT_UNUSED_PARAM(out_dyn_dxv);
		MRPT_UNUSED_PARAM(out_dyn_dhn);
		MRPT_START
		THROW_EXCEPTION(
			"Inverse sensor model required but not implemented in derived "
			"class.")
		MRPT_END
	}

	/** If applicable to the given problem, this method implements the inverse
	 * observation model needed to extend the "map" with a new "element".
	 *  The uncertainty in the new map feature comes from two parts: one from
	 * the vehicle uncertainty (through the out_dyn_dxv Jacobian),
	 *   and another from the uncertainty in the observation itself. By
	 * default, out_use_dyn_dhn_jacobian=true on call, and if it's left at
	 * "true",
	 *   the base KalmanFilter class will compute the uncertainty of the
	 * landmark relative position from out_dyn_dhn.
	 *  Only in some problems (e.g. MonoSLAM), it'll be needed for the
	 * application to directly return the covariance matrix \a
	 * out_dyn_dhn_R_dyn_dhnT, which is the equivalent to:
	 *
	 *         \f$ \frac{\partial y_n}{\partial h_n} R \frac{\partial
	 * y_n}{\partial h_n}^\top \f$.
	 *
	 *  but may be computed from additional terms, or whatever needed by the
	 * user.
	 *
	 * \param in_z The observation vector whose inverse sensor model is to be
	 * computed. This is actually one of the vector<> returned by
	 * OnGetObservationsAndDataAssociation().
	 * \param out_yn The F-length vector with the inverse observation model \f$
	 * y_n=y(x,z_n) \f$.
	 * \param out_dyn_dxv The \f$F \times V\f$ Jacobian of the inv. sensor
	 * model wrt the robot pose \f$ \frac{\partial y_n}{\partial x_v} \f$.
	 * \param out_dyn_dhn The \f$F \times O\f$ Jacobian of the inv. sensor
	 * model wrt the observation vector \f$ \frac{\partial y_n}{\partial h_n}
	 * \f$.
	 * \param out_dyn_dhn_R_dyn_dhnT See the discussion above.
	 *
	 *  - O: OBS_SIZE
	 *  - V: VEH_SIZE
	 *  - F: FEAT_SIZE
	 *
	 * \note OnNewLandmarkAddedToMap will be also called after calling this
	 * method if a landmark is actually being added to the map.
	 */
	virtual void OnInverseObservationModel(
		const KFArray_OBS& in_z, KFArray_FEAT& out_yn,
		KFMatrix_FxV& out_dyn_dxv, KFMatrix_FxO& out_dyn_dhn,
		KFMatrix_FxF& out_dyn_dhn_R_dyn_dhnT,
		bool& out_use_dyn_dhn_jacobian) const
	{
		MRPT_UNUSED_PARAM(out_dyn_dhn_R_dyn_dhnT);
		MRPT_START
		OnInverseObservationModel(in_z, out_yn, out_dyn_dxv, out_dyn_dhn);
		out_use_dyn_dhn_jacobian = true;
		MRPT_END
	}

	/** If applicable to the given problem, do here any special handling of
	 * adding a new landmark to the map.
	 * \param in_obsIndex The index of the observation whose inverse sensor is
	 * to be computed. It corresponds to the row in in_z where the observation
	 * can be found.
	 * \param in_idxNewFeat The index that this new feature will have in the
	 * state vector (0:just after the vehicle state, 1: after that,...). Save
	 * this number so data association can be done according to these indices.
	 * \sa OnInverseObservationModel
	 */
	virtual void OnNewLandmarkAddedToMap(
		const size_t in_obsIdx, const size_t in_idxNewFeat)
	{
		MRPT_UNUSED_PARAM(in_obsIdx);
		MRPT_UNUSED_PARAM(in_idxNewFeat);
		// Do nothing in this base class.
	}

	/** This method is called after the prediction and after the update, to give
	 * the user an opportunity to normalize the state vector (eg, keep angles
	 * within -pi,pi range) if the application requires it.
	 */
	virtual void OnNormalizeStateVector()
	{
		// Do nothing in this base class.
	}

	/** This method is called after finishing one KF iteration and before
	 * returning from runOneKalmanIteration().
	 */
	virtual void OnPostIteration()
	{
		// Do nothing in this base class.
	}

	/** @}
	 */

   public:
	CKalmanFilterCapable()
		: mrpt::system::COutputLogger("CKalmanFilterCapable"),
		  KF_options(this->m_min_verbosity_level)
	{
	}
	/** Destructor */
	~CKalmanFilterCapable() override = default;
	mrpt::system::CTimeLogger& getProfiler() { return m_timLogger; }
	/** Generic options for the Kalman Filter algorithm itself. */
	TKF_options KF_options;

   private:
	//  "Local" variables to runOneKalmanIteration, declared here to avoid
	//   allocating them over and over again with each call.
	//  (The variables that go into the stack remains in the function body)
	vector_KFArray_OBS m_all_predictions;
	std::vector<size_t> m_predictLMidxs;
	/** The vector of all partial Jacobians dh[i]_dx for each prediction */
	mrpt::aligned_std_vector<KFMatrix_OxV> m_Hxs;
	/** The vector of all partial Jacobians dh[i]_dy[i] for each prediction */
	mrpt::aligned_std_vector<KFMatrix_OxF> m_Hys;
	KFMatrix m_S;
	vector_KFArray_OBS m_Z;  // Each entry is one observation:
	KFMatrix m_K;  // Kalman gain
	KFMatrix m_S_1;  // Inverse of m_S
	KFMatrix m_dh_dx_full_obs;
	KFMatrix m_aux_K_dh_dx;

   protected:
	/** The main entry point, executes one complete step: prediction + update.
	 *  It is protected since derived classes must provide a problem-specific
	 * entry point for users.
	 *  The exact order in which this method calls the virtual method is
	 * explained in http://www.mrpt.org/Kalman_Filters
	 */
	void runOneKalmanIteration();

   private:
	mutable bool m_user_didnt_implement_jacobian{true};

	/** Auxiliary functions for Jacobian numeric estimation */
	static void KF_aux_estimate_trans_jacobian(
		const KFArray_VEH& x, const std::pair<KFCLASS*, KFArray_ACT>& dat,
		KFArray_VEH& out_x);
	static void KF_aux_estimate_obs_Hx_jacobian(
		const KFArray_VEH& x, const std::pair<KFCLASS*, size_t>& dat,
		KFArray_OBS& out_x);
	static void KF_aux_estimate_obs_Hy_jacobian(
		const KFArray_FEAT& x, const std::pair<KFCLASS*, size_t>& dat,
		KFArray_OBS& out_x);

	template <
		size_t VEH_SIZEb, size_t OBS_SIZEb, size_t FEAT_SIZEb, size_t ACT_SIZEb,
		typename KFTYPEb>
	friend void detail::addNewLandmarks(
		CKalmanFilterCapable<
			VEH_SIZEb, OBS_SIZEb, FEAT_SIZEb, ACT_SIZEb, KFTYPEb>& obj,
		const typename CKalmanFilterCapable<
			VEH_SIZEb, OBS_SIZEb, FEAT_SIZEb, ACT_SIZEb,
			KFTYPEb>::vector_KFArray_OBS& Z,
		const std::vector<int>& data_association,
		const typename CKalmanFilterCapable<
			VEH_SIZEb, OBS_SIZEb, FEAT_SIZEb, ACT_SIZEb, KFTYPEb>::KFMatrix_OxO&
			R);
};  // end class

}  // namespace bayes
}  // namespace mrpt

MRPT_ENUM_TYPE_BEGIN(mrpt::bayes::TKFMethod)
using namespace mrpt::bayes;
MRPT_FILL_ENUM(kfEKFNaive);
MRPT_FILL_ENUM(kfEKFAlaDavison);
MRPT_FILL_ENUM(kfIKFFull);
MRPT_FILL_ENUM(kfIKF);
MRPT_ENUM_TYPE_END()

// Template implementation:
#define CKalmanFilterCapable_H
#include "CKalmanFilterCapable_impl.h"
