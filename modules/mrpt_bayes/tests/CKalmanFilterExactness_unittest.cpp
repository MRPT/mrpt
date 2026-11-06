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

// Checks CKalmanFilterCapable against an independent dense Eigen
// implementation of the (extended / iterated) Kalman filter.

#include <gtest/gtest.h>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/math/wrap2pi.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <vector>

using namespace mrpt::bayes;

namespace
{
using Mat = Eigen::MatrixXd;
using Vec = Eigen::VectorXd;

// ---------------------------------------------------------------------------
// Problem definitions shared by MRPT filters and the dense references
// ---------------------------------------------------------------------------
Mat linF()
{
  Mat F(2, 2);
  F << 1.0, 0.1, 0.0, 1.0;
  return F;
}
Mat linH()
{
  // Non-diagonal, so the two observation components are correlated.
  Mat H(2, 2);
  H << 1.0, 0.5, 0.2, 1.0;
  return H;
}
Mat procQ()
{
  Mat Q(2, 2);
  Q << 0.01, 0.002, 0.002, 0.02;
  return Q;
}
Mat obsR()
{
  // Diagonal, so that kfEKFAlaDavison is applicable.
  Mat R(2, 2);
  R << 0.04, 0.0, 0.0, 0.09;
  return R;
}
Vec initX()
{
  Vec x(2);
  x << 1.0, -0.5;
  return x;
}
Mat initP()
{
  Mat P(2, 2);
  P << 1.0, 0.3, 0.3, 0.5;
  return P;
}
Vec actionU()
{
  Vec u(2);
  u << 0.2, -0.1;
  return u;
}
Vec measZ(int step)
{
  Vec z(2);
  z << 1.3 + 0.1 * step, -0.2 - 0.05 * step;
  return z;
}

// Nonlinear model: beacon range/bearing observation.
double BX = 5.0;
double BY = 3.0;
double MEAS_R = 4.2;
double MEAS_BEARING = 0.65;

Vec nlTransition(const Vec& x, const Vec& u)
{
  Vec o(2);
  o << x[0] + u[0] * std::cos(x[1]), x[1] + u[1];
  return o;
}
Mat nlTransitionJac(const Vec& x, const Vec& u)
{
  Mat F(2, 2);
  F << 1.0, -u[0] * std::sin(x[1]), 0.0, 1.0;
  return F;
}
Vec nlObs(const Vec& x)
{
  const double dx = BX - x[0];
  const double dy = BY - x[1];
  Vec o(2);
  o << std::sqrt(dx * dx + dy * dy), std::atan2(dy, dx);
  return o;
}
Mat nlObsJac(const Vec& x)
{
  const double dx = BX - x[0];
  const double dy = BY - x[1];
  const double r2 = dx * dx + dy * dy;
  const double r = std::sqrt(r2);
  Mat H(2, 2);
  H << -dx / r, -dy / r, dy / r2, -dx / r2;
  return H;
}
Vec nlMeasZ()
{
  Vec z(2);
  z << MEAS_R, MEAS_BEARING;
  return z;
}

// ---------------------------------------------------------------------------
// Dense references
// ---------------------------------------------------------------------------
void denseLinearKFStep(Vec& x, Mat& P, int step)
{
  const Mat F = linF();
  const Mat H = linH();
  x = F * x + actionU();
  P = F * P * F.transpose() + procQ();
  const Mat S = H * P * H.transpose() + obsR();
  const Mat K = P * H.transpose() * S.inverse();
  x = x + K * (measZ(step) - H * x);
  const Mat IKH = Mat::Identity(2, 2) - K * H;
  P = IKH * P * IKH.transpose() + K * obsR() * K.transpose();
}

// Gauss-Newton iterated EKF (Bell & Cathey 1993) with `iters` iterations.
void denseNonlinearIEKFStep(Vec& x, Mat& P, int iters)
{
  const Vec u = actionU();
  const Mat F = nlTransitionJac(x, u);
  x = nlTransition(x, u);
  P = F * P * F.transpose() + procQ();
  const Vec xpred = x;
  Vec xi = xpred;
  Mat K;
  Mat H;
  for (int it = 0; it < iters; it++)
  {
    H = nlObsJac(xi);
    const Mat S = H * P * H.transpose() + obsR();
    K = P * H.transpose() * S.inverse();
    xi = xpred + K * (nlMeasZ() - nlObs(xi) - H * (xpred - xi));
  }
  x = xi;
  const Mat IKH = Mat::Identity(2, 2) - K * H;
  P = IKH * P * IKH.transpose() + K * obsR() * K.transpose();
}

// ---------------------------------------------------------------------------
// MRPT filters
// ---------------------------------------------------------------------------
class LinearKF : public CKalmanFilterCapable<2, 2, 0, 2, double>
{
 public:
  int step_idx = 0;
  bool analytic = true;

  LinearKF()
  {
    const Vec x0 = initX();
    const Mat P0 = initP();
    m_xkk.resize(2);
    m_pkk.setSize(2, 2);
    for (int r = 0; r < 2; r++)
    {
      m_xkk[r] = x0[r];
      for (int c = 0; c < 2; c++)
      {
        m_pkk(r, c) = P0(r, c);
      }
    }
  }
  void step() { runOneKalmanIteration(); }

 protected:
  void OnGetAction(KFArray_ACT& u) const override
  {
    u[0] = actionU()[0];
    u[1] = actionU()[1];
  }
  void OnTransitionModel(const KFArray_ACT& u, KFArray_VEH& x, bool& skip) const override
  {
    skip = false;
    const Mat F = linF();
    const double x0 = x[0];
    const double x1 = x[1];
    x[0] = F(0, 0) * x0 + F(0, 1) * x1 + u[0];
    x[1] = F(1, 0) * x0 + F(1, 1) * x1 + u[1];
  }
  void OnTransitionJacobian(KFMatrix_VxV& F) const override
  {
    if (!analytic)
    {
      CKalmanFilterCapable::OnTransitionJacobian(F);
      return;
    }
    const Mat Fd = linF();
    for (int r = 0; r < 2; r++)
    {
      for (int c = 0; c < 2; c++)
      {
        F(r, c) = Fd(r, c);
      }
    }
  }
  void OnTransitionNoise(KFMatrix_VxV& Q) const override
  {
    const Mat Qd = procQ();
    for (int r = 0; r < 2; r++)
    {
      for (int c = 0; c < 2; c++)
      {
        Q(r, c) = Qd(r, c);
      }
    }
  }
  void OnGetObservationNoise(KFMatrix_OxO& R) const override
  {
    const Mat Rd = obsR();
    for (int r = 0; r < 2; r++)
    {
      for (int c = 0; c < 2; c++)
      {
        R(r, c) = Rd(r, c);
      }
    }
  }
  void OnGetObservationsAndDataAssociation(
      vector_KFArray_OBS& out_z,
      std::vector<int>& out_da,
      const vector_KFArray_OBS&,
      const KFMatrix&,
      const std::vector<size_t>&,
      const KFMatrix_OxO&) override
  {
    out_da.clear();
    out_z.resize(1);
    const Vec z = measZ(step_idx);
    out_z[0][0] = z[0];
    out_z[0][1] = z[1];
  }
  void OnObservationModel(const std::vector<size_t>&, vector_KFArray_OBS& out) const override
  {
    const Mat H = linH();
    out.resize(1);
    out[0][0] = H(0, 0) * m_xkk[0] + H(0, 1) * m_xkk[1];
    out[0][1] = H(1, 0) * m_xkk[0] + H(1, 1) * m_xkk[1];
  }
  void OnObservationJacobians(size_t idx, KFMatrix_OxV& Hx, KFMatrix_OxF& Hy) const override
  {
    if (!analytic)
    {
      CKalmanFilterCapable::OnObservationJacobians(idx, Hx, Hy);
      return;
    }
    const Mat H = linH();
    for (int r = 0; r < 2; r++)
    {
      for (int c = 0; c < 2; c++)
      {
        Hx(r, c) = H(r, c);
      }
    }
  }
};

class NonlinearKF : public CKalmanFilterCapable<2, 2, 0, 2, double>
{
 public:
  bool analytic = true;

  NonlinearKF()
  {
    const Vec x0 = initX();
    const Mat P0 = initP();
    m_xkk.resize(2);
    m_pkk.setSize(2, 2);
    for (int r = 0; r < 2; r++)
    {
      m_xkk[r] = x0[r];
      for (int c = 0; c < 2; c++)
      {
        m_pkk(r, c) = P0(r, c);
      }
    }
  }
  void step() { runOneKalmanIteration(); }

 protected:
  void OnSubtractObservationVectors(KFArray_OBS& A, const KFArray_OBS& B) const override
  {
    A[0] -= B[0];
    A[1] = mrpt::math::wrapToPi(A[1] - B[1]);
  }
  void OnGetAction(KFArray_ACT& u) const override
  {
    u[0] = actionU()[0];
    u[1] = actionU()[1];
  }
  void OnTransitionModel(const KFArray_ACT& u, KFArray_VEH& x, bool& skip) const override
  {
    skip = false;
    Vec xv(2);
    xv << x[0], x[1];
    Vec uv(2);
    uv << u[0], u[1];
    const Vec o = nlTransition(xv, uv);
    x[0] = o[0];
    x[1] = o[1];
  }
  void OnTransitionJacobian(KFMatrix_VxV& F) const override
  {
    if (!analytic)
    {
      CKalmanFilterCapable::OnTransitionJacobian(F);
      return;
    }
    Vec xv(2);
    xv << m_xkk[0], m_xkk[1];
    const Mat Fd = nlTransitionJac(xv, actionU());
    for (int r = 0; r < 2; r++)
    {
      for (int c = 0; c < 2; c++)
      {
        F(r, c) = Fd(r, c);
      }
    }
  }
  void OnTransitionNoise(KFMatrix_VxV& Q) const override
  {
    const Mat Qd = procQ();
    for (int r = 0; r < 2; r++)
    {
      for (int c = 0; c < 2; c++)
      {
        Q(r, c) = Qd(r, c);
      }
    }
  }
  void OnGetObservationNoise(KFMatrix_OxO& R) const override
  {
    const Mat Rd = obsR();
    for (int r = 0; r < 2; r++)
    {
      for (int c = 0; c < 2; c++)
      {
        R(r, c) = Rd(r, c);
      }
    }
  }
  void OnGetObservationsAndDataAssociation(
      vector_KFArray_OBS& out_z,
      std::vector<int>& out_da,
      const vector_KFArray_OBS&,
      const KFMatrix&,
      const std::vector<size_t>&,
      const KFMatrix_OxO&) override
  {
    out_da.clear();
    out_z.resize(1);
    out_z[0][0] = nlMeasZ()[0];
    out_z[0][1] = nlMeasZ()[1];
  }
  void OnObservationModel(const std::vector<size_t>&, vector_KFArray_OBS& out) const override
  {
    Vec xv(2);
    xv << m_xkk[0], m_xkk[1];
    const Vec o = nlObs(xv);
    out.resize(1);
    out[0][0] = o[0];
    out[0][1] = o[1];
  }
  void OnObservationJacobians(size_t idx, KFMatrix_OxV& Hx, KFMatrix_OxF& Hy) const override
  {
    if (!analytic)
    {
      CKalmanFilterCapable::OnObservationJacobians(idx, Hx, Hy);
      return;
    }
    Vec xv(2);
    xv << m_xkk[0], m_xkk[1];
    const Mat H = nlObsJac(xv);
    for (int r = 0; r < 2; r++)
    {
      for (int c = 0; c < 2; c++)
      {
        Hx(r, c) = H(r, c);
      }
    }
  }
};

// Landmark-dependent observation matrix of the linear SLAM model.
Mat lmA(size_t lm)
{
  const double k = static_cast<double>(lm);
  Mat A(2, 2);
  A << 1.0 + 0.1 * k, 0.2, -0.1 * k, 1.0;
  return A;
}

// SLAM: 2D vehicle position, 2D point landmarks, z_i = A_i * (y_i - x)
// (linear, with a different A_i for each landmark).
class LinearSLAM : public CKalmanFilterCapable<2, 2, 2, 2, double>
{
 public:
  // Observations for the next step, and their data association.
  std::vector<Vec> next_z;
  std::vector<int> next_da;
  // Innovation covariance passed to the data association callback.
  Mat last_S;
  std::vector<size_t> last_S_idxs;

  LinearSLAM()
  {
    const Vec x0 = initX();
    const Mat P0 = initP();
    m_xkk.resize(2);
    m_pkk.setSize(2, 2);
    for (int r = 0; r < 2; r++)
    {
      m_xkk[r] = x0[r];
      for (int c = 0; c < 2; c++)
      {
        m_pkk(r, c) = P0(r, c);
      }
    }
  }
  void step() { runOneKalmanIteration(); }

  Vec stateEigen() const
  {
    Vec v(m_xkk.size());
    for (int i = 0; i < static_cast<int>(m_xkk.size()); i++)
    {
      v[i] = m_xkk[i];
    }
    return v;
  }
  Mat covEigen() const
  {
    const int n = static_cast<int>(m_pkk.rows());
    Mat P(n, n);
    for (int r = 0; r < n; r++)
    {
      for (int c = 0; c < n; c++)
      {
        P(r, c) = m_pkk(r, c);
      }
    }
    return P;
  }

 protected:
  void OnGetAction(KFArray_ACT& u) const override
  {
    u[0] = actionU()[0];
    u[1] = actionU()[1];
  }
  void OnTransitionModel(const KFArray_ACT& u, KFArray_VEH& x, bool& skip) const override
  {
    skip = false;
    x[0] += u[0];
    x[1] += u[1];
  }
  void OnTransitionJacobian(KFMatrix_VxV& F) const override { F.setIdentity(); }
  void OnTransitionNoise(KFMatrix_VxV& Q) const override
  {
    const Mat Qd = procQ();
    for (int r = 0; r < 2; r++)
    {
      for (int c = 0; c < 2; c++)
      {
        Q(r, c) = Qd(r, c);
      }
    }
  }
  void OnGetObservationNoise(KFMatrix_OxO& R) const override
  {
    const Mat Rd = obsR();
    for (int r = 0; r < 2; r++)
    {
      for (int c = 0; c < 2; c++)
      {
        R(r, c) = Rd(r, c);
      }
    }
  }
  void OnGetObservationsAndDataAssociation(
      vector_KFArray_OBS& out_z,
      std::vector<int>& out_da,
      const vector_KFArray_OBS&,
      const KFMatrix& in_S,
      const std::vector<size_t>& in_lm_indices_in_S,
      const KFMatrix_OxO&) override
  {
    const int n = static_cast<int>(in_S.rows());
    last_S.resize(n, n);
    for (int r = 0; r < n; r++)
    {
      for (int c = 0; c < n; c++)
      {
        last_S(r, c) = in_S(r, c);
      }
    }
    last_S_idxs = in_lm_indices_in_S;
    out_z.resize(next_z.size());
    for (size_t i = 0; i < next_z.size(); i++)
    {
      out_z[i][0] = next_z[i][0];
      out_z[i][1] = next_z[i][1];
    }
    out_da = next_da;
  }
  void OnObservationModel(const std::vector<size_t>& idxs, vector_KFArray_OBS& out) const override
  {
    out.resize(idxs.size());
    for (size_t i = 0; i < idxs.size(); i++)
    {
      const size_t off = 2 + 2 * idxs[i];
      const Mat A = lmA(idxs[i]);
      const double dx = m_xkk[off] - m_xkk[0];
      const double dy = m_xkk[off + 1] - m_xkk[1];
      out[i][0] = A(0, 0) * dx + A(0, 1) * dy;
      out[i][1] = A(1, 0) * dx + A(1, 1) * dy;
    }
  }
  void OnObservationJacobians(size_t lm, KFMatrix_OxV& Hx, KFMatrix_OxF& Hy) const override
  {
    const Mat A = lmA(lm);
    for (int r = 0; r < 2; r++)
    {
      for (int c = 0; c < 2; c++)
      {
        Hx(r, c) = -A(r, c);
        Hy(r, c) = A(r, c);
      }
    }
  }

 public:
  using CKalmanFilterCapable::OnInverseObservationModel;
  void OnInverseObservationModel(
      const KFArray_OBS& z,
      KFArray_FEAT& yn,
      KFMatrix_FxV& dyn_dxv,
      KFMatrix_FxO& dyn_dhn) const override
  {
    // Index that the new landmark will get:
    const size_t lm = (static_cast<size_t>(m_xkk.size()) - 2) / 2;
    const Mat Ainv = lmA(lm).inverse();
    yn[0] = m_xkk[0] + Ainv(0, 0) * z[0] + Ainv(0, 1) * z[1];
    yn[1] = m_xkk[1] + Ainv(1, 0) * z[0] + Ainv(1, 1) * z[1];
    dyn_dxv.setIdentity();
    for (int r = 0; r < 2; r++)
    {
      for (int c = 0; c < 2; c++)
      {
        dyn_dhn(r, c) = Ainv(r, c);
      }
    }
  }
};

// Dense reference for LinearSLAM: prediction, update with the known
// landmarks (in the order given by `da`), then augmentation.
void denseSLAMStep(Vec& x, Mat& P, const std::vector<Vec>& zs, const std::vector<int>& da)
{
  const int n = static_cast<int>(x.size());
  // Prediction (only the vehicle moves, F = I):
  x.head(2) += actionU();
  P.topLeftCorner(2, 2) += procQ();

  // Update:
  std::vector<int> known;
  for (size_t i = 0; i < da.size(); i++)
  {
    if (da[i] >= 0)
    {
      known.push_back(static_cast<int>(i));
    }
  }
  if (!known.empty())
  {
    const int m = static_cast<int>(known.size());
    Mat H = Mat::Zero(2 * m, n);
    Mat R = Mat::Zero(2 * m, 2 * m);
    Vec y(2 * m);
    for (int k = 0; k < m; k++)
    {
      const int lm = da[known[k]];
      const Mat A = lmA(static_cast<size_t>(lm));
      H.block(2 * k, 0, 2, 2) = -A;
      H.block(2 * k, 2 + 2 * lm, 2, 2) = A;
      R.block(2 * k, 2 * k, 2, 2) = obsR();
      y.segment(2 * k, 2) = zs[known[k]] - A * (x.segment(2 + 2 * lm, 2) - x.head(2));
    }
    const Mat S = H * P * H.transpose() + R;
    const Mat K = P * H.transpose() * S.inverse();
    x = x + K * y;
    const Mat IKH = Mat::Identity(n, n) - K * H;
    P = IKH * P * IKH.transpose() + K * R * K.transpose();
  }

  // Augmentation, y_new = x_v + A^-1 * z:
  for (size_t i = 0; i < da.size(); i++)
  {
    if (da[i] >= 0)
    {
      continue;
    }
    const int nc = static_cast<int>(x.size());
    const Mat Ainv = lmA(static_cast<size_t>((nc - 2) / 2)).inverse();
    Vec xa(nc + 2);
    xa.head(nc) = x;
    xa.tail(2) = x.head(2) + Ainv * zs[i];
    Mat J = Mat::Zero(nc + 2, nc);
    J.topRows(nc) = Mat::Identity(nc, nc);
    J.block(nc, 0, 2, 2) = Mat::Identity(2, 2);
    Mat G = Mat::Zero(nc + 2, 2);
    G.block(nc, 0, 2, 2) = Ainv;
    P = J * P * J.transpose() + G * obsR() * G.transpose();
    x = xa;
  }
}

template <class KF>
Vec getX(KF& kf)
{
  const auto& xk = kf.internal_getXkk();
  Vec v(xk.size());
  for (int i = 0; i < static_cast<int>(xk.size()); i++)
  {
    v[i] = xk[i];
  }
  return v;
}
template <class KF>
Mat getP(KF& kf)
{
  const auto& Pk = kf.internal_getPkk();
  const int n = static_cast<int>(Pk.rows());
  Mat P(n, n);
  for (int r = 0; r < n; r++)
  {
    for (int c = 0; c < n; c++)
    {
      P(r, c) = Pk(r, c);
    }
  }
  return P;
}

void runLinearVsDense(TKFMethod method, bool joseph, bool analytic, double tol)
{
  LinearKF kf;
  kf.KF_options.method = method;
  kf.KF_options.use_joseph_form = joseph;
  kf.analytic = analytic;
  Vec x = initX();
  Mat P = initP();
  for (int s = 0; s < 3; s++)
  {
    kf.step_idx = s;
    kf.step();
    denseLinearKFStep(x, P, s);
    const Vec xk = getX(kf);
    const Mat Pk = getP(kf);
    EXPECT_LT((xk - x).norm(), tol) << "step " << s << "\nmrpt:\n"
                                    << xk.transpose() << "\nref:\n"
                                    << x.transpose();
    EXPECT_LT((Pk - P).norm(), tol) << "step " << s << "\nmrpt:\n" << Pk << "\nref:\n" << P;
  }
}

// Five scripted SLAM steps: new landmarks, mixed known/new, reordering.
struct SLAMScript
{
  std::vector<std::vector<Vec>> zs;
  std::vector<std::vector<int>> das;
};
Vec v2(double a, double b)
{
  Vec v(2);
  v << a, b;
  return v;
}
SLAMScript slamScript()
{
  SLAMScript s;
  s.zs.push_back({v2(3.0, 1.0), v2(-1.0, 2.5)});
  s.das.push_back({-1, -1});
  s.zs.push_back({v2(-1.1, 2.3), v2(0.5, -2.0), v2(2.7, 1.1)});
  s.das.push_back({1, -1, 0});
  s.zs.push_back({v2(0.6, -2.1), v2(2.6, 0.9)});
  s.das.push_back({2, 0});
  s.zs.push_back({});
  s.das.push_back({});
  s.zs.push_back({v2(-1.4, 2.2), v2(2.4, 1.0), v2(0.3, -2.3)});
  s.das.push_back({1, 0, 2});
  return s;
}

void runSLAMVsDense(TKFMethod method, bool joseph, bool debugSparse)
{
  LinearSLAM kf;
  kf.KF_options.method = method;
  kf.KF_options.use_joseph_form = joseph;
  kf.KF_options.debug_sparse_vs_dense_P_update = debugSparse;
  Vec x = initX();
  Mat P = initP();
  const SLAMScript s = slamScript();
  for (size_t k = 0; k < s.zs.size(); k++)
  {
    kf.next_z = s.zs[k];
    kf.next_da = s.das[k];
    kf.step();
    denseSLAMStep(x, P, s.zs[k], s.das[k]);
    const Vec xk = kf.stateEigen();
    const Mat Pk = kf.covEigen();
    ASSERT_EQ(xk.size(), x.size()) << "step " << k;
    EXPECT_LT((xk - x).norm(), 1e-9) << "step " << k << "\nmrpt:\n"
                                     << xk.transpose() << "\nref:\n"
                                     << x.transpose();
    EXPECT_LT((Pk - P).norm(), 1e-9) << "step " << k << "\nmrpt:\n" << Pk << "\nref:\n" << P;
    EXPECT_LT((Pk - Pk.transpose()).norm(), 1e-12);
  }
}

}  // namespace

TEST(KalmanFilterExactness, LinearNaiveJosephMatchesKF)
{
  runLinearVsDense(kfEKFNaive, true, true, 1e-10);
}

TEST(KalmanFilterExactness, LinearNaiveClassicMatchesKF)
{
  runLinearVsDense(kfEKFNaive, false, true, 1e-10);
}

TEST(KalmanFilterExactness, LinearNumericJacobiansMatchKF)
{
  runLinearVsDense(kfEKFNaive, true, false, 1e-7);
}

// Each scalar update must use the innovation at the current state.
TEST(KalmanFilterExactness, LinearAlaDavisonMatchesKF)
{
  runLinearVsDense(kfEKFAlaDavison, true, true, 1e-9);
}

// The iterated EKF must reduce to the KF on a linear system.
TEST(KalmanFilterExactness, LinearIKFMatchesKF) { runLinearVsDense(kfIKFFull, true, true, 1e-9); }

TEST(KalmanFilterExactness, NonlinearEKFMatchesDenseEKF)
{
  for (const bool analytic : {true, false})
  {
    NonlinearKF kf;
    kf.KF_options.method = kfEKFNaive;
    kf.analytic = analytic;
    kf.step();
    Vec x = initX();
    Mat P = initP();
    denseNonlinearIEKFStep(x, P, 1);
    const double tol = analytic ? 1e-10 : 1e-7;
    EXPECT_LT((getX(kf) - x).norm(), tol) << "analytic=" << analytic;
    EXPECT_LT((getP(kf) - P).norm(), tol) << "analytic=" << analytic;
  }
}

// The iterated EKF must re-linearize the observation model at each iterate.
TEST(KalmanFilterExactness, NonlinearIKFMatchesGaussNewtonIEKF)
{
  NonlinearKF kf;
  kf.KF_options.method = kfIKFFull;
  kf.KF_options.IKF_iterations = 5;
  kf.step();
  Vec x = initX();
  Mat P = initP();
  denseNonlinearIEKFStep(x, P, 5);
  EXPECT_LT((getX(kf) - x).norm(), 1e-6)
      << "mrpt: " << getX(kf).transpose() << "\nref:  " << x.transpose();
}

TEST(KalmanFilterExactness, SLAMNaiveJosephMatchesKF) { runSLAMVsDense(kfEKFNaive, true, false); }

TEST(KalmanFilterExactness, SLAMNaiveClassicMatchesKF) { runSLAMVsDense(kfEKFNaive, false, false); }

// The iterated EKF and the scalar-at-once update must reduce to the KF on a
// linear SLAM problem.
TEST(KalmanFilterExactness, SLAMIKFMatchesKF) { runSLAMVsDense(kfIKFFull, true, false); }

TEST(KalmanFilterExactness, SLAMAlaDavisonMatchesKF)
{
  runSLAMVsDense(kfEKFAlaDavison, true, false);
}

TEST(KalmanFilterExactness, SLAMSparseVsDenseDebugCheckPasses)
{
  runSLAMVsDense(kfEKFNaive, true, true);
  runSLAMVsDense(kfEKFNaive, false, true);
}

TEST(KalmanFilterExactness, SLAMInnovationCovarianceMatchesDense)
{
  LinearSLAM kf;
  Vec x = initX();
  Mat P = initP();
  const SLAMScript s = slamScript();
  // Build a map with three landmarks:
  for (size_t k = 0; k < 2; k++)
  {
    kf.next_z = s.zs[k];
    kf.next_da = s.das[k];
    kf.step();
    denseSLAMStep(x, P, s.zs[k], s.das[k]);
  }
  kf.next_z = {};
  kf.next_da = {};
  kf.step();
  // Reference: predicted covariance and S = H P H^T + R for all landmarks.
  P.topLeftCorner(2, 2) += procQ();
  const int nLM = 3;
  ASSERT_EQ(kf.last_S_idxs.size(), static_cast<size_t>(nLM));
  const int n = static_cast<int>(P.rows());
  Mat H = Mat::Zero(2 * nLM, n);
  Mat R = Mat::Zero(2 * nLM, 2 * nLM);
  for (int k = 0; k < nLM; k++)
  {
    const int lm = static_cast<int>(kf.last_S_idxs[k]);
    const Mat A = lmA(kf.last_S_idxs[k]);
    H.block(2 * k, 0, 2, 2) = -A;
    H.block(2 * k, 2 + 2 * lm, 2, 2) = A;
    R.block(2 * k, 2 * k, 2, 2) = obsR();
  }
  const Mat S = H * P * H.transpose() + R;
  EXPECT_LT((kf.last_S - S).norm(), 1e-10) << "mrpt:\n" << kf.last_S << "\nref:\n" << S;
}

namespace
{
// SLAM filter that only predicts a subset of the landmarks, so observed
// landmarks out of that subset must be added by the filter afterwards.
class SubsetSLAM : public LinearSLAM
{
 public:
  std::vector<size_t> to_predict;

 protected:
  void OnPreComputingPredictions(
      const vector_KFArray_OBS&, std::vector<size_t>& out_LM_indices_to_predict) const override
  {
    out_LM_indices_to_predict.clear();
    for (const size_t i : to_predict)
    {
      if (i < getNumberOfLandmarksInTheMap())
      {
        out_LM_indices_to_predict.push_back(i);
      }
    }
  }
};
}  // namespace

// A subset of predicted landmarks (smaller than, or not including, the
// observed ones) must not change the result, for all KF methods.
TEST(KalmanFilterExactness, SLAMPredictionSubsetMatchesKF)
{
  for (const auto method : {kfEKFNaive, kfEKFAlaDavison, kfIKFFull})
  {
    for (const auto& subset : std::vector<std::vector<size_t>>{
             {},
             { 2},
             { 2,0}
    })
    {
      SubsetSLAM kf;
      kf.KF_options.method = method;
      kf.to_predict = subset;
      Vec x = initX();
      Mat P = initP();
      const SLAMScript s = slamScript();
      for (size_t k = 0; k < s.zs.size(); k++)
      {
        kf.next_z = s.zs[k];
        kf.next_da = s.das[k];
        kf.step();
        denseSLAMStep(x, P, s.zs[k], s.das[k]);
        const Vec xk = kf.stateEigen();
        const Mat Pk = kf.covEigen();
        ASSERT_EQ(xk.size(), x.size()) << "step " << k;
        EXPECT_LT((xk - x).norm(), 1e-9) << "method " << static_cast<int>(method) << " step " << k;
        EXPECT_LT((Pk - P).norm(), 1e-9) << "method " << static_cast<int>(method) << " step " << k;
        // All the associated landmarks end up predicted:
        for (const int lm : s.das[k])
        {
          if (lm >= 0)
          {
            EXPECT_NE(
                std::find(kf.last_S_idxs.begin(), kf.last_S_idxs.end(), static_cast<size_t>(lm)),
                kf.last_S_idxs.end());
          }
        }
      }
    }
  }
}

// Numeric observation Jacobians must honor OnSubtractObservationVectors(),
// e.g. for angular observations close to +-pi.
TEST(KalmanFilterExactness, NumericJacobianNearBearingDiscontinuity)
{
  // Beacon right behind the predicted position: bearing close to +pi.
  const double oldBX = BX;
  const double oldBY = BY;
  const double oldR = MEAS_R;
  const double oldB = MEAS_BEARING;
  BX = -5.0;
  BY = -0.6 + 1e-7;
  MEAS_R = 6.3;
  MEAS_BEARING = -M_PI + 0.02;
  NonlinearKF kfA;
  kfA.analytic = true;
  kfA.step();
  NonlinearKF kfN;
  kfN.analytic = false;
  kfN.step();
  BX = oldBX;
  BY = oldBY;
  MEAS_R = oldR;
  MEAS_BEARING = oldB;
  EXPECT_LT((getX(kfA) - getX(kfN)).norm(), 1e-6)
      << "analytic: " << getX(kfA).transpose() << "\nnumeric:  " << getX(kfN).transpose();
  EXPECT_LT((getP(kfA) - getP(kfN)).norm(), 1e-6);
}
