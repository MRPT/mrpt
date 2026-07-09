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

#include <gtest/gtest.h>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/bayes/CProbabilityParticle.h>
#include <mrpt/config/CConfigFileMemory.h>

#include <sstream>

using namespace mrpt::bayes;

namespace
{
// ===========================================================================
// Non-SLAM (FEAT_SIZE=0) 2D linear KF: x' = x + u ; z = x
// (VEH_SIZE/OBS_SIZE/ACT_SIZE=2, since mrpt_math only explicitly instantiates
// fixed-size matrices/vectors of dimension 2,3,4,6,7,12 -- NOT 1.)
// ===========================================================================
class ScalarKF : public CKalmanFilterCapable<2 /*VEH*/, 2 /*OBS*/, 0 /*FEAT*/, 2 /*ACT*/, double>
{
 public:
  double action = 0.0;
  double obsValue = 0.0;
  bool haveObs = true;
  double processNoise = 0.01;
  double obsNoise = 0.01;
  bool skipPredictionFlag = false;

  ScalarKF()
  {
    m_xkk.resize(2);
    m_xkk[0] = 0.0;
    m_xkk[1] = 0.0;
    m_pkk.setSize(2, 2);
    m_pkk(0, 0) = 1.0;
    m_pkk(1, 1) = 1.0;
  }

  void step() { runOneKalmanIteration(); }

  double state() const { return m_xkk[0]; }
  double cov() const { return m_pkk(0, 0); }

  // Exercises the deprecated OnSubstractObservationVectors() wrapper.
  void callDeprecatedSubtract(KFArray_OBS& a, const KFArray_OBS& b)
  {
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
    OnSubstractObservationVectors(a, b);
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
  }

 protected:
  void OnGetAction(KFArray_ACT& out_u) const override
  {
    out_u[0] = action;
    out_u[1] = action;
  }
  void OnTransitionModel(
      const KFArray_ACT& in_u, KFArray_VEH& inout_x, bool& out_skipPrediction) const override
  {
    out_skipPrediction = skipPredictionFlag;
    inout_x[0] += in_u[0];
    inout_x[1] += in_u[1];
  }
  void OnTransitionNoise(KFMatrix_VxV& out_Q) const override
  {
    out_Q(0, 0) = processNoise;
    out_Q(1, 1) = processNoise;
  }
  void OnGetObservationNoise(KFMatrix_OxO& out_R) const override
  {
    out_R(0, 0) = obsNoise;
    out_R(1, 1) = obsNoise;
  }
  void OnGetObservationsAndDataAssociation(
      vector_KFArray_OBS& out_z,
      std::vector<int>& out_data_association,
      const vector_KFArray_OBS&,
      const KFMatrix&,
      const std::vector<size_t>&,
      const KFMatrix_OxO&) override
  {
    out_data_association.clear();
    if (haveObs)
    {
      out_z.resize(1);
      out_z[0][0] = obsValue;
      out_z[0][1] = obsValue;
    }
    else
    {
      out_z.clear();
    }
  }
  void OnObservationModel(
      const std::vector<size_t>&, vector_KFArray_OBS& out_predictions) const override
  {
    out_predictions.resize(1);
    out_predictions[0][0] = m_xkk[0];
    out_predictions[0][1] = m_xkk[1];
  }
};

// Analytic (correct) Jacobians: F=I, Hx=I
class ScalarKF_AnalyticJac : public ScalarKF
{
 protected:
  void OnTransitionJacobian(KFMatrix_VxV& out_F) const override { out_F.setIdentity(); }
  void OnObservationJacobians(size_t, KFMatrix_OxV& Hx, KFMatrix_OxF&) const override
  {
    Hx.setIdentity();
  }
};

// Deliberately wrong transition Jacobian, to exercise the debug-verify mismatch throw.
class ScalarKF_WrongTransitionJac : public ScalarKF
{
 protected:
  void OnTransitionJacobian(KFMatrix_VxV& out_F) const override
  {
    out_F.setIdentity();
    out_F(0, 0) = 42.0;
  }
};

// Deliberately wrong observation Jacobian, to exercise the debug-verify mismatch throw.
class ScalarKF_WrongObservationJac : public ScalarKF
{
 protected:
  void OnTransitionJacobian(KFMatrix_VxV& out_F) const override { out_F.setIdentity(); }
  void OnObservationJacobians(size_t, KFMatrix_OxV& Hx, KFMatrix_OxF&) const override
  {
    Hx.setIdentity();
    Hx(0, 0) = -99.0;
  }
};

}  // namespace

TEST(CKalmanFilterCapable, NonSLAM_NumericJacobians_NaiveEKF)
{
  ScalarKF kf;
  kf.action = 1.0;
  kf.obsValue = 1.0;
  kf.step();
  EXPECT_NEAR(kf.state(), 1.0, 0.05);
}

TEST(CKalmanFilterCapable, NonSLAM_NumericJacobians_ExplicitNonAnalytic)
{
  ScalarKF_AnalyticJac kf;
  kf.KF_options.use_analytic_transition_jacobian = false;
  kf.KF_options.use_analytic_observation_jacobian = false;
  kf.action = 2.0;
  kf.obsValue = 2.0;
  kf.step();
  EXPECT_NEAR(kf.state(), 2.0, 0.05);
}

TEST(CKalmanFilterCapable, NonSLAM_AnalyticJacobians_NoVerify)
{
  ScalarKF_AnalyticJac kf;
  kf.action = 1.0;
  kf.obsValue = 1.0;
  kf.step();
  EXPECT_NEAR(kf.state(), 1.0, 0.05);
}

TEST(CKalmanFilterCapable, NonSLAM_AnalyticJacobians_VerifyMatches)
{
  ScalarKF_AnalyticJac kf;
  kf.KF_options.debug_verify_analytic_jacobians = true;
  kf.action = 1.0;
  kf.obsValue = 1.0;
  EXPECT_NO_THROW(kf.step());
}

TEST(CKalmanFilterCapable, NonSLAM_WrongTransitionJacobian_Throws)
{
  ScalarKF_WrongTransitionJac kf;
  kf.KF_options.debug_verify_analytic_jacobians = true;
  kf.action = 1.0;
  EXPECT_THROW(kf.step(), std::exception);
}

TEST(CKalmanFilterCapable, NonSLAM_WrongObservationJacobian_Throws)
{
  ScalarKF_WrongObservationJac kf;
  kf.KF_options.debug_verify_analytic_jacobians = true;
  kf.action = 1.0;
  EXPECT_THROW(kf.step(), std::exception);
}

TEST(CKalmanFilterCapable, NonSLAM_IKF_MultipleIterations)
{
  ScalarKF_AnalyticJac kf;
  kf.KF_options.method = kfIKFFull;
  kf.KF_options.IKF_iterations = 3;
  kf.action = 1.0;
  kf.obsValue = 1.0;
  kf.step();
  EXPECT_NEAR(kf.state(), 1.0, 0.05);
}

TEST(CKalmanFilterCapable, NonSLAM_AlaDavison)
{
  ScalarKF_AnalyticJac kf;
  kf.KF_options.method = kfEKFAlaDavison;
  kf.action = 1.0;
  kf.obsValue = 1.0;
  kf.step();
  EXPECT_NEAR(kf.state(), 1.0, 0.05);
}

TEST(CKalmanFilterCapable, NonSLAM_InvalidMethod_Throws)
{
  ScalarKF_AnalyticJac kf;
  kf.KF_options.method = static_cast<TKFMethod>(99);
  kf.action = 1.0;
  kf.obsValue = 1.0;
  EXPECT_THROW(kf.step(), std::exception);
}

TEST(CKalmanFilterCapable, NonSLAM_NoObservation_SkipsUpdate)
{
  ScalarKF kf;
  kf.action = 1.0;
  kf.haveObs = false;
  const double covBefore = kf.cov();
  kf.step();
  // No update was applied: covariance should have only grown (prediction), not shrunk.
  EXPECT_GE(kf.cov(), covBefore);
}

TEST(CKalmanFilterCapable, NonSLAM_SkipPrediction)
{
  ScalarKF kf;
  kf.action = 5.0;
  kf.skipPredictionFlag = true;
  kf.obsValue = 0.0;
  kf.step();
  // Since prediction was skipped, the action must not have affected the
  // state: it should have stayed near the observed value (0), not been
  // shifted by the (skipped) action.
  EXPECT_NEAR(kf.state(), 0.0, 0.5);
}

TEST(CKalmanFilterCapable, NonSLAM_JosephFormDisabled)
{
  ScalarKF_AnalyticJac kf;
  kf.KF_options.use_joseph_form = false;
  kf.action = 1.0;
  kf.obsValue = 1.0;
  EXPECT_NO_THROW(kf.step());
}

TEST(CKalmanFilterCapable, DeprecatedSubtractObservationVectors)
{
  ScalarKF kf;
  ScalarKF::KFArray_OBS a;
  ScalarKF::KFArray_OBS b;
  a[0] = 5.0;
  b[0] = 2.0;
  kf.callDeprecatedSubtract(a, b);
  EXPECT_NEAR(a[0], 3.0, 1e-12);
}

TEST(CKalmanFilterCapable, ProbabilityParticleValueConstructor)
{
  CProbabilityParticle<double, particle_storage_mode::VALUE> p(3.5, -1.0);
  EXPECT_NEAR(p.d, 3.5, 1e-12);
  EXPECT_NEAR(p.log_w, -1.0, 1e-12);
}

namespace
{
// ===========================================================================
// SLAM-like (FEAT_SIZE=2) 2D landmark KF:
//   Vehicle: 2D position. Action: position delta. Landmarks: 2D positions.
//   Observation: relative landmark position z = landmark - vehicle.
// ===========================================================================
class Landmark2DKF :
    public CKalmanFilterCapable<2 /*VEH*/, 2 /*OBS*/, 2 /*FEAT*/, 2 /*ACT*/, double>
{
 public:
  double actionX = 0.0;
  double actionY = 0.0;

  std::vector<KFArray_OBS> nextZ;
  std::vector<int> nextDA;
  std::vector<size_t> restrictPredictionsTo;  // empty => predict all (default behavior)
  bool useDynDhnJacobian = true;

  Landmark2DKF()
  {
    m_xkk.resize(2);
    m_xkk[0] = 0.0;
    m_xkk[1] = 0.0;
    m_pkk.setSize(2, 2);
    m_pkk(0, 0) = 0.1;
    m_pkk(1, 1) = 0.1;
  }

  void step() { runOneKalmanIteration(); }

  size_t numLandmarks() const { return getNumberOfLandmarksInTheMap(); }

  KFArray_FEAT landmarkMean(size_t idx) const
  {
    KFArray_FEAT out;
    getLandmarkMean(idx, out);
    return out;
  }

 protected:
  void OnGetAction(KFArray_ACT& out_u) const override
  {
    out_u[0] = actionX;
    out_u[1] = actionY;
  }
  void OnTransitionModel(
      const KFArray_ACT& in_u, KFArray_VEH& inout_x, bool& out_skipPrediction) const override
  {
    out_skipPrediction = false;
    inout_x[0] += in_u[0];
    inout_x[1] += in_u[1];
  }
  void OnTransitionJacobian(KFMatrix_VxV& out_F) const override { out_F.setIdentity(); }
  void OnTransitionNoise(KFMatrix_VxV& out_Q) const override
  {
    out_Q(0, 0) = 1e-4;
    out_Q(1, 1) = 1e-4;
  }
  void OnGetObservationNoise(KFMatrix_OxO& out_R) const override
  {
    out_R(0, 0) = 1e-2;
    out_R(1, 1) = 1e-2;
  }
  void OnPreComputingPredictions(
      const vector_KFArray_OBS&, std::vector<size_t>& out_LM_indices_to_predict) const override
  {
    if (restrictPredictionsTo.empty())
    {
      const size_t N = getNumberOfLandmarksInTheMap();
      out_LM_indices_to_predict.resize(N);
      for (size_t i = 0; i < N; i++) out_LM_indices_to_predict[i] = i;
    }
    else
    {
      out_LM_indices_to_predict = restrictPredictionsTo;
    }
  }
  void OnGetObservationsAndDataAssociation(
      vector_KFArray_OBS& out_z,
      std::vector<int>& out_data_association,
      const vector_KFArray_OBS&,
      const KFMatrix&,
      const std::vector<size_t>&,
      const KFMatrix_OxO&) override
  {
    out_z = nextZ;
    out_data_association = nextDA;
  }
  void OnObservationModel(
      const std::vector<size_t>& idx_landmarks_to_predict,
      vector_KFArray_OBS& out_predictions) const override
  {
    out_predictions.resize(idx_landmarks_to_predict.size());
    for (size_t i = 0; i < idx_landmarks_to_predict.size(); i++)
    {
      KFArray_FEAT lm;
      getLandmarkMean(idx_landmarks_to_predict[i], lm);
      out_predictions[i][0] = lm[0] - m_xkk[0];
      out_predictions[i][1] = lm[1] - m_xkk[1];
    }
  }
  void OnObservationJacobians(size_t, KFMatrix_OxV& Hx, KFMatrix_OxF& Hy) const override
  {
    Hx.setZero();
    Hx(0, 0) = -1.0;
    Hx(1, 1) = -1.0;
    Hy.setIdentity();
  }
  void OnInverseObservationModel(
      const KFArray_OBS& in_z,
      KFArray_FEAT& out_yn,
      KFMatrix_FxV& out_dyn_dxv,
      KFMatrix_FxO& out_dyn_dhn,
      KFMatrix_FxF& out_dyn_dhn_R_dyn_dhnT,
      bool& out_use_dyn_dhn_jacobian) const override
  {
    out_yn[0] = m_xkk[0] + in_z[0];
    out_yn[1] = m_xkk[1] + in_z[1];
    out_dyn_dxv.setIdentity();
    out_dyn_dhn.setIdentity();
    out_use_dyn_dhn_jacobian = useDynDhnJacobian;
    if (!useDynDhnJacobian)
    {
      // R is diagonal 1e-2 here (mirrors OnGetObservationNoise): dyn_dhn * R * dyn_dhn^T = R.
      out_dyn_dhn_R_dyn_dhnT.setZero();
      out_dyn_dhn_R_dyn_dhnT(0, 0) = 1e-2;
      out_dyn_dhn_R_dyn_dhnT(1, 1) = 1e-2;
    }
  }
};

Landmark2DKF::KFArray_OBS mkObs(double a, double b)
{
  Landmark2DKF::KFArray_OBS out;
  out[0] = a;
  out[1] = b;
  return out;
}

}  // namespace

TEST(CKalmanFilterCapable, SLAM_AddTwoNewLandmarks)
{
  Landmark2DKF kf;
  const auto z0 = mkObs(5.0, 0.0);
  const auto z1 = mkObs(0.0, 5.0);
  kf.nextZ = {z0, z1};
  kf.nextDA = {-1, -1};  // both new landmarks
  kf.step();

  ASSERT_EQ(kf.numLandmarks(), 2u);
  EXPECT_NEAR(kf.landmarkMean(0)[0], 5.0, 0.5);
  EXPECT_NEAR(kf.landmarkMean(1)[1], 5.0, 0.5);
}

TEST(CKalmanFilterCapable, SLAM_UpdateExistingLandmarks_JosephForm)
{
  Landmark2DKF kf;
  const auto z0 = mkObs(5.0, 0.0);
  const auto z1 = mkObs(0.0, 5.0);
  kf.nextZ = {z0, z1};
  kf.nextDA = {-1, -1};
  kf.step();
  ASSERT_EQ(kf.numLandmarks(), 2u);

  // Second step: re-observe both landmarks (now known), forces the SLAM Sij
  // block computation (both diagonal and off-diagonal blocks).
  kf.nextDA = {0, 1};
  kf.KF_options.use_joseph_form = true;
  EXPECT_NO_THROW(kf.step());
}

TEST(CKalmanFilterCapable, SLAM_UpdateExistingLandmarks_NoJosephForm)
{
  Landmark2DKF kf;
  const auto z0 = mkObs(5.0, 0.0);
  const auto z1 = mkObs(0.0, 5.0);
  kf.nextZ = {z0, z1};
  kf.nextDA = {-1, -1};
  kf.step();

  kf.nextDA = {0, 1};
  kf.KF_options.use_joseph_form = false;
  EXPECT_NO_THROW(kf.step());
}

TEST(CKalmanFilterCapable, SLAM_DebugSparseVsDenseMatches)
{
  Landmark2DKF kf;
  const auto z0 = mkObs(5.0, 0.0);
  const auto z1 = mkObs(0.0, 5.0);
  kf.nextZ = {z0, z1};
  kf.nextDA = {-1, -1};
  kf.step();

  kf.nextDA = {0, 1};
  kf.KF_options.debug_sparse_vs_dense_P_update = true;
  EXPECT_NO_THROW(kf.step());
}

TEST(CKalmanFilterCapable, SLAM_IteratedKF)
{
  Landmark2DKF kf;
  const auto z0 = mkObs(5.0, 0.0);
  kf.nextZ = {z0};
  kf.nextDA = {-1};
  kf.step();

  kf.nextDA = {0};
  kf.KF_options.method = kfIKFFull;
  kf.KF_options.IKF_iterations = 3;
  EXPECT_NO_THROW(kf.step());
}

TEST(CKalmanFilterCapable, SLAM_AlaDavison_MixedKnownAndNewLandmarks)
{
  Landmark2DKF kf;
  const auto z0 = mkObs(5.0, 0.0);
  kf.nextZ = {z0};
  kf.nextDA = {-1};
  kf.step();
  ASSERT_EQ(kf.numLandmarks(), 1u);

  // Observe the known landmark plus a brand-new one:
  const auto z1 = mkObs(0.0, 3.0);
  kf.nextZ = {z0, z1};
  kf.nextDA = {0, -1};
  kf.KF_options.method = kfEKFAlaDavison;
  EXPECT_NO_THROW(kf.step());
  EXPECT_EQ(kf.numLandmarks(), 2u);
}

TEST(CKalmanFilterCapable, SLAM_MissingPredictionRetriggersJacobianBuild)
{
  Landmark2DKF kf;
  const auto z0 = mkObs(5.0, 0.0);
  const auto z1 = mkObs(0.0, 5.0);
  kf.nextZ = {z0, z1};
  kf.nextDA = {-1, -1};
  kf.step();
  ASSERT_EQ(kf.numLandmarks(), 2u);

  // OnPreComputingPredictions() deliberately omits landmark #1, but the
  // data association below references it: this forces the
  // "missing_predictions_to_add" retry loop in runOneKalmanIteration().
  kf.restrictPredictionsTo = {0};
  kf.nextDA = {1};
  kf.nextZ = {z1};
  EXPECT_NO_THROW(kf.step());
}

TEST(CKalmanFilterCapable, SLAM_InverseObservationModel_WithoutDynDhnJacobian)
{
  Landmark2DKF kf;
  kf.useDynDhnJacobian = false;
  const auto z0 = mkObs(5.0, 0.0);
  kf.nextZ = {z0};
  kf.nextDA = {-1};
  EXPECT_NO_THROW(kf.step());
  ASSERT_EQ(kf.numLandmarks(), 1u);
}

TEST(CKalmanFilterCapable, SLAM_NumericObservationJacobian)
{
  // With use_analytic_observation_jacobian=false, both the vehicle (Hx) and
  // feature (Hy) parts of the observation Jacobian are estimated
  // numerically for a SLAM (FEAT_SIZE!=0) problem.
  Landmark2DKF kf;
  kf.KF_options.use_analytic_observation_jacobian = false;
  const auto z0 = mkObs(5.0, 0.0);
  const auto z1 = mkObs(0.0, 5.0);
  kf.nextZ = {z0, z1};
  kf.nextDA = {-1, -1};
  kf.step();
  ASSERT_EQ(kf.numLandmarks(), 2u);

  kf.nextDA = {0, 1};
  EXPECT_NO_THROW(kf.step());
}

TEST(CKalmanFilterCapable, SLAM_DebugVerifyObservationJacobians_Matches)
{
  Landmark2DKF kf;
  kf.KF_options.debug_verify_analytic_jacobians = true;
  const auto z0 = mkObs(5.0, 0.0);
  kf.nextZ = {z0};
  kf.nextDA = {-1};
  kf.step();
  ASSERT_EQ(kf.numLandmarks(), 1u);

  kf.nextDA = {0};
  EXPECT_NO_THROW(kf.step());
}

namespace
{
// A SLAM-shaped filter that never implements the inverse observation model,
// used to exercise the default (throwing) 4-argument overload.
class LandmarkNoInverseModelKF :
    public CKalmanFilterCapable<2 /*VEH*/, 2 /*OBS*/, 2 /*FEAT*/, 2 /*ACT*/, double>
{
 public:
  LandmarkNoInverseModelKF()
  {
    m_xkk.resize(2);
    m_pkk.setSize(2, 2);
  }
  void step() { runOneKalmanIteration(); }

 protected:
  void OnGetAction(KFArray_ACT& out_u) const override
  {
    out_u[0] = 0;
    out_u[1] = 0;
  }
  void OnTransitionModel(const KFArray_ACT&, KFArray_VEH&, bool& out_skipPrediction) const override
  {
    out_skipPrediction = true;
  }
  void OnTransitionNoise(KFMatrix_VxV& out_Q) const override { out_Q.setZero(); }
  void OnGetObservationNoise(KFMatrix_OxO& out_R) const override { out_R.setIdentity(); }
  void OnGetObservationsAndDataAssociation(
      vector_KFArray_OBS& out_z,
      std::vector<int>& out_data_association,
      const vector_KFArray_OBS&,
      const KFMatrix&,
      const std::vector<size_t>&,
      const KFMatrix_OxO&) override
  {
    out_z.resize(1);
    out_z[0][0] = 1.0;
    out_z[0][1] = 1.0;
    out_data_association = {-1};
  }
  void OnObservationModel(
      const std::vector<size_t>& idxs, vector_KFArray_OBS& out_predictions) const override
  {
    out_predictions.assign(idxs.size(), KFArray_OBS());
  }
};
}  // namespace

TEST(CKalmanFilterCapable, SLAM_DefaultInverseObservationModel_Throws)
{
  LandmarkNoInverseModelKF kf;
  EXPECT_THROW(kf.step(), std::exception);
}

namespace
{
// Overrides only the deprecated 4-argument OnInverseObservationModel(),
// relying on the base class' 6-argument overload to forward to it.
class Landmark2DKF_LegacyInverseModel : public Landmark2DKF
{
 protected:
  void OnInverseObservationModel(
      const KFArray_OBS& in_z,
      KFArray_FEAT& out_yn,
      KFMatrix_FxV& out_dyn_dxv,
      KFMatrix_FxO& out_dyn_dhn) const override
  {
    out_yn[0] = m_xkk[0] + in_z[0];
    out_yn[1] = m_xkk[1] + in_z[1];
    out_dyn_dxv.setIdentity();
    out_dyn_dhn.setIdentity();
  }
};

// Deliberately wrong Hy (feature) observation Jacobian, to exercise the
// debug-verify mismatch throw for the SLAM (FEAT_SIZE!=0) case.
class Landmark2DKF_WrongHyJacobian : public Landmark2DKF
{
 protected:
  void OnObservationJacobians(size_t, KFMatrix_OxV& Hx, KFMatrix_OxF& Hy) const override
  {
    Hx.setZero();
    Hx(0, 0) = -1.0;
    Hx(1, 1) = -1.0;
    Hy.setIdentity();
    Hy(0, 0) = -99.0;
  }
};
}  // namespace

TEST(CKalmanFilterCapable, SLAM_LegacyInverseObservationModel_Overload)
{
  Landmark2DKF_LegacyInverseModel kf;
  const auto z0 = mkObs(5.0, 0.0);
  kf.nextZ = {z0};
  kf.nextDA = {-1};
  EXPECT_NO_THROW(kf.step());
  ASSERT_EQ(kf.numLandmarks(), 1u);
  EXPECT_NEAR(kf.landmarkMean(0)[0], 5.0, 0.5);
}

TEST(CKalmanFilterCapable, SLAM_WrongObservationHyJacobian_Throws)
{
  Landmark2DKF_WrongHyJacobian kf;
  kf.KF_options.debug_verify_analytic_jacobians = true;
  const auto z0 = mkObs(5.0, 0.0);
  kf.nextZ = {z0};
  kf.nextDA = {-1};
  kf.step();
  ASSERT_EQ(kf.numLandmarks(), 1u);

  kf.nextDA = {0};
  EXPECT_THROW(kf.step(), std::exception);
}

TEST(CKalmanFilterCapable, TKFOptions_LoadConfigFileAndDump)
{
  // TKF_options only overrides loadFromConfigFile()/dumpToTextStream(), not
  // saveToConfigFile() (which would throw the CLoadableOptions default), so
  // build the input content by hand instead of round-tripping through save.
  const std::string iniContent =
      "[KF]\n"
      "method = kfIKFFull\n"
      "IKF_iterations = 7\n"
      "enable_profiler = true\n"
      "use_analytic_transition_jacobian = false\n"
      "use_analytic_observation_jacobian = false\n"
      "debug_verify_analytic_jacobians = true\n"
      "debug_verify_analytic_jacobians_threshold = 0.5\n"
      "use_joseph_form = false\n"
      "debug_sparse_vs_dense_P_update = true\n";
  mrpt::config::CConfigFileMemory cfg(iniContent);

  mrpt::system::VerbosityLevel verb = mrpt::system::LVL_INFO;
  TKF_options opts(verb);
  opts.loadFromConfigFile(cfg, "KF");

  EXPECT_EQ(opts.method, kfIKFFull);
  EXPECT_EQ(opts.IKF_iterations, 7);
  EXPECT_TRUE(opts.enable_profiler);
  EXPECT_FALSE(opts.use_analytic_transition_jacobian);
  EXPECT_FALSE(opts.use_analytic_observation_jacobian);
  EXPECT_TRUE(opts.debug_verify_analytic_jacobians);
  EXPECT_NEAR(opts.debug_verify_analytic_jacobians_threshold, 0.5, 1e-12);
  EXPECT_FALSE(opts.use_joseph_form);
  EXPECT_TRUE(opts.debug_sparse_vs_dense_P_update);

  std::ostringstream ss;
  opts.dumpToTextStream(ss);
  EXPECT_FALSE(ss.str().empty());
}
