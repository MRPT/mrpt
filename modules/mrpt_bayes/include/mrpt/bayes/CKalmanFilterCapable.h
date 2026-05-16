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

#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/containers/stl_containers_utils.h>
#include <mrpt/containers/stl_containers_utils.h>  // find_in_vector
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CVectorFixed.h>
#include <mrpt/math/num_jacobian.h>
#include <mrpt/math/utils.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/typemeta/TEnumType.h>

#include <cstring>  // memcpy
#include <vector>

#if defined(_DEBUG)
#include <mrpt/io/vector_loadsave.h>
#endif

namespace mrpt
{
namespace bayes
{
/** The Kalman Filter algorithm to employ in bayes::CKalmanFilterCapable
 *  For further details on each algorithm see the tutorial:
 * https://www.mrpt.org/Kalman_Filters
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
template <size_t VEH_SIZE, size_t OBS_SIZE, size_t FEAT_SIZE, size_t ACT_SIZE, typename KFTYPE>
class CKalmanFilterCapable;

/** Generic options for the Kalman Filter algorithm in itself.
 * \ingroup mrpt_bayes_grp
 */
struct TKF_options : public mrpt::config::CLoadableOptions
{
  TKF_options(mrpt::system::VerbosityLevel& verb_level_ref) : verbosity_level(verb_level_ref) {}

  void loadFromConfigFile(
      const mrpt::config::CConfigFileBase& iniFile, const std::string& section) override
  {
    method = iniFile.read_enum<TKFMethod>(section, "method", method);
    verbosity_level = iniFile.read_enum<mrpt::system::VerbosityLevel>(
        section, "verbosity_level", verbosity_level);
    MRPT_LOAD_CONFIG_VAR(IKF_iterations, int, iniFile, section);
    MRPT_LOAD_CONFIG_VAR(enable_profiler, bool, iniFile, section);
    MRPT_LOAD_CONFIG_VAR(use_analytic_transition_jacobian, bool, iniFile, section);
    MRPT_LOAD_CONFIG_VAR(use_analytic_observation_jacobian, bool, iniFile, section);
    MRPT_LOAD_CONFIG_VAR(debug_verify_analytic_jacobians, bool, iniFile, section);
    MRPT_LOAD_CONFIG_VAR(debug_verify_analytic_jacobians_threshold, double, iniFile, section);
  }

  /** This method must display clearly all the contents of the structure in
   * textual form, sending it to a CStream. */
  void dumpToTextStream(std::ostream& out) const override
  {
    out << "\n----------- [TKF_options] ------------ \n\n";
    out << mrpt::format(
        "method                                  = %s\n",
        mrpt::typemeta::TEnumType<TKFMethod>::value2name(method).c_str());
    out << mrpt::format(
        "verbosity_level                         = %s\n",
        mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>::value2name(verbosity_level)
            .c_str());
    out << mrpt::format("IKF_iterations                          = %i\n", IKF_iterations);
    out << mrpt::format(
        "enable_profiler                         = %c\n", enable_profiler ? 'Y' : 'N');
    out << "\n";
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
template <size_t VEH_SIZE, size_t OBS_SIZE, size_t FEAT_SIZE, size_t ACT_SIZE, typename KFTYPE>
size_t getNumberOfLandmarksInMap(
    const CKalmanFilterCapable<VEH_SIZE, OBS_SIZE, FEAT_SIZE, ACT_SIZE, KFTYPE>& obj);
// Specialization:
template <size_t VEH_SIZE, size_t OBS_SIZE, size_t ACT_SIZE, typename KFTYPE>
size_t getNumberOfLandmarksInMap(
    const CKalmanFilterCapable<VEH_SIZE, OBS_SIZE, 0 /*FEAT_SIZE*/, ACT_SIZE, KFTYPE>& obj);

template <size_t VEH_SIZE, size_t OBS_SIZE, size_t FEAT_SIZE, size_t ACT_SIZE, typename KFTYPE>
bool isMapEmpty(const CKalmanFilterCapable<VEH_SIZE, OBS_SIZE, FEAT_SIZE, ACT_SIZE, KFTYPE>& obj);
// Specialization:
template <size_t VEH_SIZE, size_t OBS_SIZE, size_t ACT_SIZE, typename KFTYPE>
bool isMapEmpty(
    const CKalmanFilterCapable<VEH_SIZE, OBS_SIZE, 0 /*FEAT_SIZE*/, ACT_SIZE, KFTYPE>& obj);

template <size_t VEH_SIZE, size_t OBS_SIZE, size_t FEAT_SIZE, size_t ACT_SIZE, typename KFTYPE>
void addNewLandmarks(
    CKalmanFilterCapable<VEH_SIZE, OBS_SIZE, FEAT_SIZE, ACT_SIZE, KFTYPE>& obj,
    const typename CKalmanFilterCapable<VEH_SIZE, OBS_SIZE, FEAT_SIZE, ACT_SIZE, KFTYPE>::
        vector_KFArray_OBS& Z,
    const std::vector<int>& data_association,
    const typename CKalmanFilterCapable<VEH_SIZE, OBS_SIZE, FEAT_SIZE, ACT_SIZE, KFTYPE>::
        KFMatrix_OxO& R);
template <size_t VEH_SIZE, size_t OBS_SIZE, size_t ACT_SIZE, typename KFTYPE>
void addNewLandmarks(
    CKalmanFilterCapable<VEH_SIZE, OBS_SIZE, 0 /* FEAT_SIZE=0 */, ACT_SIZE, KFTYPE>& obj,
    const typename CKalmanFilterCapable<VEH_SIZE, OBS_SIZE, 0 /* FEAT_SIZE=0 */, ACT_SIZE, KFTYPE>::
        vector_KFArray_OBS& Z,
    const std::vector<int>& data_association,
    const typename CKalmanFilterCapable<VEH_SIZE, OBS_SIZE, 0 /* FEAT_SIZE=0 */, ACT_SIZE, KFTYPE>::
        KFMatrix_OxO& R);
}  // namespace detail

/** \brief Virtual base template for EKF, IEKF, and IKF Kalman filter
 * implementations using the CRTP-like callback pattern.
 *
 * Stores the state mean vector \f$ \hat{x}_{k|k} \f$ (m_xkk) and the full
 * covariance matrix \f$ P_{k|k} \f$ (m_pkk). Derived classes implement the
 * pure-virtual "On*" callback methods that describe the specific filtering
 * problem (motion model, observation model, noise matrices, data association).
 * The main entry point runOneKalmanIteration() then calls those callbacks in
 * the correct order to carry out one predict-update cycle.
 *
 * Supported filter variants (selected via KF_options.method):
 *  - kfEKFNaive: Standard Extended Kalman Filter.
 *  - kfEKFAlaDavison: EKF with efficient feature-by-feature updates (SLAM).
 *  - kfIKFFull: Iterated EKF (full re-linearization per iteration).
 *  - kfIKF: Iterated EKF (efficient feature-based variant).
 *
 * \tparam VEH_SIZE  Dimension of the vehicle (robot) state sub-vector.
 * \tparam OBS_SIZE  Dimension of a single observation vector.
 * \tparam FEAT_SIZE Dimension of each map feature state, or 0 if no map.
 * \tparam ACT_SIZE  Dimension of the action (control input) vector, or 0.
 * \tparam KFTYPE    Scalar type for all matrix/vector arithmetic (default: double).
 *
 * \note runOneKalmanIteration() is protected; derived classes must expose a
 * problem-specific public method that calls it.
 *
 * For further details and examples see: https://www.mrpt.org/Kalman_Filters
 *
 * \sa mrpt::slam::CRangeBearingKFSLAM, mrpt::slam::CRangeBearingKFSLAM2D
 * \ingroup mrpt_bayes_grp
 *
 * Virtual base for Kalman Filter (EKF,IEKF,UKF) implementations.
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
    size_t VEH_SIZE,
    size_t OBS_SIZE,
    size_t FEAT_SIZE,
    size_t ACT_SIZE,
    typename KFTYPE = double>
class CKalmanFilterCapable : public mrpt::system::COutputLogger
{
 public:
  static constexpr size_t get_vehicle_size() { return VEH_SIZE; }
  static constexpr size_t get_observation_size() { return OBS_SIZE; }
  static constexpr size_t get_feature_size() { return FEAT_SIZE; }
  static constexpr size_t get_action_size() { return ACT_SIZE; }
  size_t getNumberOfLandmarksInTheMap() const { return detail::getNumberOfLandmarksInMap(*this); }
  bool isMapEmpty() const { return detail::isMapEmpty(*this); }
  /** The numeric type used in the Kalman Filter (default=double) */
  using kftype = KFTYPE;
  /** My class, in a shorter name! */
  using KFCLASS = CKalmanFilterCapable<VEH_SIZE, OBS_SIZE, FEAT_SIZE, ACT_SIZE, KFTYPE>;

  // ---------- Many useful typedefs to short the notation a bit... --------
  using KFVector = mrpt::math::CVectorDynamic<KFTYPE>;
  using KFMatrix = mrpt::math::CMatrixDynamic<KFTYPE>;

  using KFMatrix_VxV = mrpt::math::CMatrixFixed<KFTYPE, VEH_SIZE, VEH_SIZE>;
  using KFMatrix_OxO = mrpt::math::CMatrixFixed<KFTYPE, OBS_SIZE, OBS_SIZE>;
  using KFMatrix_FxF = mrpt::math::CMatrixFixed<KFTYPE, FEAT_SIZE, FEAT_SIZE>;
  using KFMatrix_AxA = mrpt::math::CMatrixFixed<KFTYPE, ACT_SIZE, ACT_SIZE>;

  using KFMatrix_VxO = mrpt::math::CMatrixFixed<KFTYPE, VEH_SIZE, OBS_SIZE>;
  using KFMatrix_VxF = mrpt::math::CMatrixFixed<KFTYPE, VEH_SIZE, FEAT_SIZE>;
  using KFMatrix_FxV = mrpt::math::CMatrixFixed<KFTYPE, FEAT_SIZE, VEH_SIZE>;
  using KFMatrix_FxO = mrpt::math::CMatrixFixed<KFTYPE, FEAT_SIZE, OBS_SIZE>;
  using KFMatrix_OxF = mrpt::math::CMatrixFixed<KFTYPE, OBS_SIZE, FEAT_SIZE>;
  using KFMatrix_OxV = mrpt::math::CMatrixFixed<KFTYPE, OBS_SIZE, VEH_SIZE>;

  using KFArray_VEH = mrpt::math::CVectorFixed<KFTYPE, VEH_SIZE>;
  using KFArray_ACT = mrpt::math::CVectorFixed<KFTYPE, ACT_SIZE>;
  using KFArray_OBS = mrpt::math::CVectorFixed<KFTYPE, OBS_SIZE>;
  using vector_KFArray_OBS = std::vector<KFArray_OBS>;
  using KFArray_FEAT = mrpt::math::CVectorFixed<KFTYPE, FEAT_SIZE>;

  size_t getStateVectorLength() const { return m_xkk.size(); }
  KFVector& internal_getXkk() { return m_xkk; }
  KFMatrix& internal_getPkk() { return m_pkk; }
  /** Returns the mean of the estimated value of the idx'th landmark (not
   * applicable to non-SLAM problems).
   * \exception std::exception On idx>= getNumberOfLandmarksInTheMap()
   */
  void getLandmarkMean(size_t idx, KFArray_FEAT& feat) const
  {
    ASSERT_(idx < getNumberOfLandmarksInTheMap());
    std::memcpy(&feat[0], &m_xkk[VEH_SIZE + idx * FEAT_SIZE], FEAT_SIZE * sizeof(m_xkk[0]));
  }
  /** Returns the covariance of the idx'th landmark (not applicable to
   * non-SLAM problems).
   * \exception std::exception On idx>= getNumberOfLandmarksInTheMap()
   */
  void getLandmarkCov(size_t idx, KFMatrix_FxF& feat_cov) const
  {
    feat_cov = m_pkk.template blockCopy<FEAT_SIZE, FEAT_SIZE>(
        VEH_SIZE + idx * FEAT_SIZE, VEH_SIZE + idx * FEAT_SIZE);
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

  /** \brief Returns the control action vector \f$ u_k \f$ for the current step.
   *
   * Called once per KF iteration before OnTransitionModel().
   * \param[out] out_u The action vector to be passed to OnTransitionModel().
   */
  virtual void OnGetAction(KFArray_ACT& out_u) const = 0;

  /** \brief Implements the transition (prediction) model
   * \f$ \hat{x}_{k|k-1} = f( \hat{x}_{k-1|k-1}, u_k ) \f$.
   *
   * \param[in]    in_u          The action vector returned by OnGetAction().
   * \param[inout] inout_x       On input: \f$ \hat{x}_{k-1|k-1} \f$. On
   *               output: must hold \f$ \hat{x}_{k|k-1} \f$.
   * \param[out]   out_skipPrediction Set to true to skip updating the state
   *               vector and covariance for this step. Even when true, the
   *               output value of \a inout_x must be set correctly because
   *               it may be used for numeric Jacobian approximation.
   */
  virtual void OnTransitionModel(
      const KFArray_ACT& in_u, KFArray_VEH& inout_x, bool& out_skipPrediction) const = 0;

  /** \brief Computes the transition Jacobian
   * \f$ F = \frac{\partial f}{\partial x} \f$.
   *
   * Only called when KF_options.use_analytic_transition_jacobian is true.
   * \param[out] out_F The \f$ V \times V \f$ Jacobian of the transition
   *             function with respect to the vehicle state (VEH_SIZE x VEH_SIZE
   *             for SLAM problems, or full state size for non-SLAM).
   * \note The default implementation sets m_user_didnt_implement_jacobian=true,
   *       causing a numeric approximation to be used instead.
   */
  virtual void OnTransitionJacobian([[maybe_unused]] KFMatrix_VxV& out_F) const
  {
    m_user_didnt_implement_jacobian = true;
  }

  /** \brief Returns the numeric perturbation sizes for transition Jacobian
   * approximation.
   *
   * Called only when use_analytic_transition_jacobian is false.
   * \param[out] out_increments Per-dimension step sizes (default: 1e-6).
   */
  virtual void OnTransitionJacobianNumericGetIncrements(KFArray_VEH& out_increments) const
  {
    for (size_t i = 0; i < VEH_SIZE; i++) out_increments[i] = 1e-6;
  }

  /** \brief Returns the process (transition) noise covariance matrix
   * \f$ Q_k \f$.
   *
   * \param[out] out_Q The \f$ V \times V \f$ process noise covariance. Must
   *             match the size of the Jacobian returned by OnTransitionJacobian().
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
      [[maybe_unused]] const vector_KFArray_OBS& in_all_prediction_means,
      std::vector<size_t>& out_LM_indices_to_predict) const
  {
    // Default: all of them:
    const size_t N = this->getNumberOfLandmarksInTheMap();
    out_LM_indices_to_predict.resize(N);
    for (size_t i = 0; i < N; i++) out_LM_indices_to_predict[i] = i;
  }

  /** \brief Returns the observation (sensor) noise covariance matrix
   * \f$ R \f$.
   *
   * \param[out] out_R The \f$ O \times O \f$ sensor noise covariance. On
   *             entry all elements are guaranteed to be zero; the
   *             implementation should fill only the non-zero entries.
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
      vector_KFArray_OBS& out_z,
      std::vector<int>& out_data_association,
      const vector_KFArray_OBS& in_all_predictions,
      const KFMatrix& in_S,
      const std::vector<size_t>& in_lm_indices_in_S,
      const KFMatrix_OxO& in_R) = 0;

  /** \brief Computes predicted observations \f$ h_i(x) \f$ for selected
   * landmarks (or the whole state for non-SLAM problems).
   *
   * \param[in]  idx_landmarks_to_predict Indices (0-based) of the map
   *             landmarks whose predictions are requested. For non-SLAM
   *             problems this parameter is undefined; produce exactly one
   *             prediction for the whole state.
   * \param[out] out_predictions One OBS_SIZE-length vector per requested
   *             landmark, in the same order as \a idx_landmarks_to_predict.
   */
  virtual void OnObservationModel(
      const std::vector<size_t>& idx_landmarks_to_predict,
      vector_KFArray_OBS& out_predictions) const = 0;

  /** \brief Computes observation Jacobians for one landmark.
   *
   * Provides \f$ H_x = \frac{\partial h_i}{\partial x_v} \f$ and, for SLAM
   * problems, \f$ H_y = \frac{\partial h_i}{\partial y_i} \f$.
   *
   * Only called when KF_options.use_analytic_observation_jacobian is true.
   *
   * \param[in]  idx_landmark_to_predict Index of the landmark whose Jacobians
   *             are requested. For non-SLAM problems this is always 0.
   * \param[out] Hx \f$ O \times V \f$ Jacobian of the observation model wrt
   *             the vehicle state.
   * \param[out] Hy \f$ O \times F \f$ Jacobian of the observation model wrt
   *             the landmark state (zero matrix for non-SLAM problems).
   * \note The default implementation sets m_user_didnt_implement_jacobian=true
   *       so that a numeric approximation is used instead.
   */
  virtual void OnObservationJacobians(
      [[maybe_unused]] size_t idx_landmark_to_predict,
      [[maybe_unused]] KFMatrix_OxV& Hx,
      [[maybe_unused]] KFMatrix_OxF& Hy) const
  {
    m_user_didnt_implement_jacobian = true;
  }

  /** \brief Returns numeric perturbation sizes for observation Jacobian
   * approximation.
   *
   * Called only when use_analytic_observation_jacobian is false.
   * \param[out] out_veh_increments  Per-dimension vehicle-state step sizes.
   * \param[out] out_feat_increments Per-dimension feature-state step sizes.
   */
  virtual void OnObservationJacobiansNumericGetIncrements(
      KFArray_VEH& out_veh_increments, KFArray_FEAT& out_feat_increments) const
  {
    for (size_t i = 0; i < VEH_SIZE; i++) out_veh_increments[i] = 1e-6;
    for (size_t i = 0; i < FEAT_SIZE; i++) out_feat_increments[i] = 1e-6;
  }

  /** Computes A=A-B, which may need to be re-implemented depending on the
   * topology of the individual scalar components (eg, angles).
   */
  virtual void OnSubstractObservationVectors(KFArray_OBS& A, const KFArray_OBS& B) const { A -= B; }

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
      [[maybe_unused]] const KFArray_OBS& in_z,
      [[maybe_unused]] KFArray_FEAT& out_yn,
      [[maybe_unused]] KFMatrix_FxV& out_dyn_dxv,
      [[maybe_unused]] KFMatrix_FxO& out_dyn_dhn) const
  {
    MRPT_START
    THROW_EXCEPTION(
        "Inverse sensor model required but not implemented in derived "
        "class.");
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
      const KFArray_OBS& in_z,
      KFArray_FEAT& out_yn,
      KFMatrix_FxV& out_dyn_dxv,
      KFMatrix_FxO& out_dyn_dhn,
      [[maybe_unused]] KFMatrix_FxF& out_dyn_dhn_R_dyn_dhnT,
      bool& out_use_dyn_dhn_jacobian) const
  {
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
      [[maybe_unused]] size_t in_obsIdx, [[maybe_unused]] size_t in_idxNewFeat)
  {
    // Do nothing in this base class.
  }

  /** \brief Optional post-update state normalization hook.
   *
   * Called after each predict-update cycle to allow derived classes to
   * normalize the state vector (e.g. wrap angles to [-pi, pi]).
   * The default implementation does nothing.
   */
  virtual void OnNormalizeStateVector()
  {
    // Do nothing in this base class.
  }

  /** \brief Optional post-iteration hook called at the end of
   * runOneKalmanIteration(), before returning.
   *
   * The default implementation does nothing.
   */
  virtual void OnPostIteration()
  {
    // Do nothing in this base class.
  }

  /** @}
   */

 public:
  CKalmanFilterCapable() :
      mrpt::system::COutputLogger("CKalmanFilterCapable"), KF_options(this->m_min_verbosity_level)
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
  std::vector<KFMatrix_OxV> m_Hxs;
  /** The vector of all partial Jacobians dh[i]_dy[i] for each prediction */
  std::vector<KFMatrix_OxF> m_Hys;
  KFMatrix m_S;
  vector_KFArray_OBS m_Z;  // Each entry is one observation:
  KFMatrix m_K;            // Kalman gain
  KFMatrix m_S_1;          // Inverse of m_S
  KFMatrix m_dh_dx_full_obs;
  KFMatrix m_aux_K_dh_dx;

 protected:
  /** The main entry point, executes one complete step: prediction + update.
   *  It is protected since derived classes must provide a problem-specific
   * entry point for users.
   *  The exact order in which this method calls the virtual method is
   * explained in https://www.mrpt.org/Kalman_Filters
   */
  void runOneKalmanIteration();

 private:
  mutable bool m_user_didnt_implement_jacobian{true};

  /** Auxiliary functions for Jacobian numeric estimation */
  static void KF_aux_estimate_trans_jacobian(
      const KFArray_VEH& x, const std::pair<KFCLASS*, KFArray_ACT>& dat, KFArray_VEH& out_x);
  static void KF_aux_estimate_obs_Hx_jacobian(
      const KFArray_VEH& x, const std::pair<KFCLASS*, size_t>& dat, KFArray_OBS& out_x);
  static void KF_aux_estimate_obs_Hy_jacobian(
      const KFArray_FEAT& x, const std::pair<KFCLASS*, size_t>& dat, KFArray_OBS& out_x);

  template <
      size_t VEH_SIZEb,
      size_t OBS_SIZEb,
      size_t FEAT_SIZEb,
      size_t ACT_SIZEb,
      typename KFTYPEb>
  friend void detail::addNewLandmarks(
      CKalmanFilterCapable<VEH_SIZEb, OBS_SIZEb, FEAT_SIZEb, ACT_SIZEb, KFTYPEb>& obj,
      const typename CKalmanFilterCapable<VEH_SIZEb, OBS_SIZEb, FEAT_SIZEb, ACT_SIZEb, KFTYPEb>::
          vector_KFArray_OBS& Z,
      const std::vector<int>& data_association,
      const typename CKalmanFilterCapable<VEH_SIZEb, OBS_SIZEb, FEAT_SIZEb, ACT_SIZEb, KFTYPEb>::
          KFMatrix_OxO& R);
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
