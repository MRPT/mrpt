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
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/ops_matrices.h>
#include <mrpt/math/transform_gaussian.h>
#include <mrpt/random/RandomGenerators.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <vector>

using mrpt::math::CMatrixDouble;
using mrpt::math::CMatrixDouble33;
using mrpt::math::CMatrixDouble44;
using mrpt::math::CMatrixDouble66;
using mrpt::math::CVectorDouble;

namespace
{
CMatrixDouble randomMatrix(int rows, int cols)
{
  auto& rng = mrpt::random::getRandomGenerator();
  CMatrixDouble m(rows, cols);
  for (int r = 0; r < rows; r++)
  {
    for (int c = 0; c < cols; c++)
    {
      m(r, c) = rng.drawUniform(-1.0, 1.0);
    }
  }
  return m;
}

// Random symmetric positive-definite matrix: A*A^T + n*I
CMatrixDouble randomSPD(int n)
{
  const CMatrixDouble a = randomMatrix(n, n);
  Eigen::MatrixXd s = a.asEigen() * a.asEigen().transpose();
  s += n * Eigen::MatrixXd::Identity(n, n);
  return CMatrixDouble(s);
}

double maxAbsDiff(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b)
{
  return (a - b).cwiseAbs().maxCoeff();
}

// y = A*x, a linear function for which the unscented transform is exact
void linearMap23(
    const mrpt::math::CVectorFixedDouble<3>& x,
    const double& /*unused*/,
    mrpt::math::CVectorFixedDouble<2>& y)
{
  y[0] = 1.0 * x[0] - 2.0 * x[1] + 0.5 * x[2];
  y[1] = 0.3 * x[0] + 1.0 * x[1] - 1.0 * x[2];
}

CMatrixDouble randomSymmetric(int n)
{
  const CMatrixDouble a = randomMatrix(n, n);
  return CMatrixDouble((a.asEigen() + a.asEigen().transpose()).eval());
}
}  // namespace

TEST(MatrixBaseDecomp, cholReconstructsAndIgnoresUpperHalf)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  for (int n = 1; n <= 7; n++)
  {
    const CMatrixDouble A = randomSPD(n);
    // Corrupt the strict upper triangle: it must be ignored.
    CMatrixDouble Acorrupt = A;
    for (int r = 0; r < n; r++)
    {
      for (int c = r + 1; c < n; c++)
      {
        Acorrupt(r, c) = 1e3;
      }
    }
    CMatrixDouble U;
    ASSERT_TRUE(Acorrupt.chol(U));
    ASSERT_EQ(U.rows(), n);
    ASSERT_EQ(U.cols(), n);
    for (int r = 0; r < n; r++)
    {
      for (int c = 0; c < r; c++)
      {
        EXPECT_EQ(U(r, c), 0.0);
      }
    }
    const Eigen::MatrixXd UtU = U.asEigen().transpose() * U.asEigen();
    EXPECT_LT(maxAbsDiff(UtU, A.asEigen()), 1e-12 * n);
  }
}

// chol() must report failure for non positive-definite matrices
TEST(MatrixBaseDecomp, cholReturnsFalseForNonPositiveDefinite)
{
  CMatrixDouble33 A;
  A.setZero();
  A(0, 0) = -1.0;
  A(1, 1) = 2.0;
  A(2, 2) = 3.0;
  CMatrixDouble33 U;
  EXPECT_FALSE(A.chol(U));

  CMatrixDouble B(2, 2);
  B(0, 0) = 1.0;
  B(0, 1) = 2.0;
  B(1, 0) = 2.0;
  B(1, 1) = 1.0;
  CMatrixDouble UB;
  EXPECT_FALSE(B.chol(UB));
}

TEST(MatrixBaseDecomp, eigSymmetricReconstructsSortedOrthonormal)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  for (int n = 2; n <= 7; n++)
  {
    const CMatrixDouble A = randomSPD(n);
    CMatrixDouble V;
    std::vector<double> D;
    ASSERT_TRUE(A.eig_symmetric(V, D));
    ASSERT_EQ(D.size(), static_cast<size_t>(n));
    EXPECT_TRUE(std::is_sorted(D.begin(), D.end()));
    const Eigen::MatrixXd VtV = V.asEigen().transpose() * V.asEigen();
    EXPECT_LT(maxAbsDiff(VtV, Eigen::MatrixXd::Identity(n, n)), 1e-12);
    Eigen::VectorXd d(n);
    for (int i = 0; i < n; i++)
    {
      d[i] = D[static_cast<size_t>(i)];
    }
    const Eigen::MatrixXd R = V.asEigen() * d.asDiagonal() * V.asEigen().transpose();
    EXPECT_LT(maxAbsDiff(R, A.asEigen()), 1e-11 * n);
  }
}

// Only round-off negative eigenvalues may be reset to zero
TEST(MatrixBaseDecomp, eigSymmetricKeepsNegativeEigenvalues)
{
  CMatrixDouble33 A;
  A.setZero();
  A(0, 0) = -5.0;
  A(1, 1) = -3.0;
  A(2, 2) = 2.0;
  CMatrixDouble33 V;
  std::vector<double> D;
  ASSERT_TRUE(A.eig_symmetric(V, D));
  ASSERT_EQ(D.size(), 3U);
  EXPECT_NEAR(D[0], -5.0, 1e-12);
  EXPECT_NEAR(D[1], -3.0, 1e-12);
  EXPECT_NEAR(D[2], 2.0, 1e-12);
}

TEST(MatrixBaseDecomp, eigGeneralRealSpectrum)
{
  // Upper triangular, non-symmetric, with distinct real eigenvalues 1,2,3
  CMatrixDouble33 A;
  A.setZero();
  A(0, 0) = 3.0;
  A(0, 1) = 1.0;
  A(0, 2) = -2.0;
  A(1, 1) = 1.0;
  A(1, 2) = 4.0;
  A(2, 2) = 2.0;
  CMatrixDouble33 V;
  std::vector<double> D;
  ASSERT_TRUE(A.eig(V, D, true));
  ASSERT_EQ(D.size(), 3U);
  EXPECT_NEAR(D[0], 1.0, 1e-12);
  EXPECT_NEAR(D[1], 2.0, 1e-12);
  EXPECT_NEAR(D[2], 3.0, 1e-12);
  for (int i = 0; i < 3; i++)
  {
    const Eigen::Vector3d v = V.asEigen().col(i);
    EXPECT_NEAR(v.norm(), 1.0, 1e-12);
    EXPECT_LT((A.asEigen() * v - D[static_cast<size_t>(i)] * v).norm(), 1e-12);
  }
}

TEST(MatrixBaseDecomp, inverseFixedAndDynamic)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  {
    const CMatrixDouble A = randomSPD(5);
    const CMatrixDouble Ai = A.inverse();
    const Eigen::MatrixXd I = A.asEigen() * Ai.asEigen();
    EXPECT_LT(maxAbsDiff(I, Eigen::MatrixXd::Identity(5, 5)), 1e-12);
    const CMatrixDouble Ail = A.inverse_LLt();
    EXPECT_LT(maxAbsDiff(Ail.asEigen(), Ai.asEigen()), 1e-12);
  }
  {
    const CMatrixDouble44 A(randomSPD(4).asEigen());
    const CMatrixDouble44 Ai = A.inverse();
    const Eigen::Matrix4d I = A.asEigen() * Ai.asEigen();
    EXPECT_LT((I - Eigen::Matrix4d::Identity()).cwiseAbs().maxCoeff(), 1e-12);
    const CMatrixDouble44 Ail = A.inverse_LLt();
    EXPECT_LT((Ail.asEigen() - Ai.asEigen()).cwiseAbs().maxCoeff(), 1e-12);
  }
  {
    // Non-symmetric matrix: only inverse() applies
    const CMatrixDouble66 A(
        (randomMatrix(6, 6).asEigen() + 3.0 * Eigen::MatrixXd::Identity(6, 6)).eval());
    const CMatrixDouble66 Ai = A.inverse();
    const Eigen::Matrix<double, 6, 6> I = Ai.asEigen() * A.asEigen();
    EXPECT_LT((I - Eigen::Matrix<double, 6, 6>::Identity()).cwiseAbs().maxCoeff(), 1e-12);
  }
}

// inverse_LLt() throws for non positive-definite matrices
TEST(MatrixBaseDecomp, inverseLLtThrowsForNonPositiveDefinite)
{
  CMatrixDouble A(2, 2);
  A(0, 0) = 1.0;
  A(0, 1) = 2.0;
  A(1, 0) = 2.0;
  A(1, 1) = 1.0;
  EXPECT_ANY_THROW(A.inverse_LLt());
  // Singular (positive semi-definite) matrix:
  mrpt::math::CMatrixDouble22 S;
  S(0, 0) = 1.0;
  S(0, 1) = 2.0;
  S(1, 0) = 2.0;
  S(1, 1) = 4.0;
  EXPECT_ANY_THROW(S.inverse_LLt());
}

TEST(MatrixBaseDecomp, detAndRank)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  // det of a product equals the product of dets
  const CMatrixDouble33 A(randomMatrix(3, 3).asEigen());
  const CMatrixDouble33 B(randomMatrix(3, 3).asEigen());
  const CMatrixDouble33 AB(A.asEigen() * B.asEigen());
  EXPECT_NEAR(AB.det(), A.det() * B.det(), 1e-12);

  // Rank of an outer-product sum
  const CMatrixDouble u = randomMatrix(5, 2);
  const CMatrixDouble v = randomMatrix(2, 6);
  const CMatrixDouble R(u.asEigen() * v.asEigen());
  EXPECT_EQ(R.rank(), 2);
  CMatrixDouble Rn = R;
  Rn(0, 0) += 1e-9;
  EXPECT_EQ(Rn.rank(), 3);
  // A large relative threshold must ignore that tiny perturbation
  EXPECT_EQ(Rn.rank(1e-6), 2);
}

TEST(MatrixBaseDecomp, productsAAtAtAMatchNaive)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const CMatrixDouble A = randomMatrix(4, 7);
  CMatrixDouble AAt;
  AAt.matProductOf_AAt(A);
  CMatrixDouble AtA;
  AtA.matProductOf_AtA(A);
  EXPECT_LT(maxAbsDiff(AAt.asEigen(), A.asEigen() * A.asEigen().transpose()), 1e-14);
  EXPECT_LT(maxAbsDiff(AtA.asEigen(), A.asEigen().transpose() * A.asEigen()), 1e-14);

  const CMatrixDouble B = randomMatrix(7, 3);
  CMatrixDouble Asq = randomMatrix(7, 7);
  CMatrixDouble Bsq = randomMatrix(7, 7);
  CMatrixDouble C;
  C.matProductOf_AB(Asq, Bsq);
  EXPECT_LT(maxAbsDiff(C.asEigen(), Asq.asEigen() * Bsq.asEigen()), 1e-14);
  // Aliasing: C = C * B must use the old value of C
  const Eigen::MatrixXd expected = (C.asEigen() * Bsq.asEigen()).eval();
  C.matProductOf_AB(C, Bsq);
  EXPECT_LT(maxAbsDiff(C.asEigen(), expected), 1e-13);
}

// M.sum_At(M) must compute M + M^T
TEST(MatrixBaseDecomp, sumAtSelfAliasing)
{
  CMatrixDouble33 M;
  int k = 1;
  for (int r = 0; r < 3; r++)
  {
    for (int c = 0; c < 3; c++)
    {
      M(r, c) = k++;
    }
  }
  const Eigen::Matrix3d expected = M.asEigen() + M.asEigen().transpose();
  M.sum_At(M);
  EXPECT_LT((M.asEigen() - expected).cwiseAbs().maxCoeff(), 1e-15);
}

TEST(MatrixBaseDecomp, removeRowsColumnsUnsortedDuplicated)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const CMatrixDouble A = randomMatrix(6, 7);
  CMatrixDouble B = A;
  B.removeColumns({5, 1, 5, 3});
  B.removeRows({0, 4, 0});
  const std::vector<int> keptRows = {1, 2, 3, 5};
  const std::vector<int> keptCols = {0, 2, 4, 6};
  ASSERT_EQ(B.rows(), 4);
  ASSERT_EQ(B.cols(), 4);
  for (size_t r = 0; r < keptRows.size(); r++)
  {
    for (size_t c = 0; c < keptCols.size(); c++)
    {
      EXPECT_EQ(B(static_cast<int>(r), static_cast<int>(c)), A(keptRows[r], keptCols[c]));
    }
  }
  CMatrixDouble Cc = A;
  EXPECT_ANY_THROW(Cc.removeColumns({7}));
  EXPECT_ANY_THROW(Cc.removeRows({6}));
}

TEST(MatrixBaseDecomp, blockExtractInsertRoundTripAndBounds)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const CMatrixDouble A = randomMatrix(5, 6);
  const auto Bf = A.extractMatrix<2, 3>(1, 2);
  const CMatrixDouble Bd = A.extractMatrix(2, 3, 1, 2);
  const CMatrixDouble Bc = A.blockCopy(1, 2, 2, 3);
  for (int r = 0; r < 2; r++)
  {
    for (int c = 0; c < 3; c++)
    {
      EXPECT_EQ(Bf(r, c), A(r + 1, c + 2));
      EXPECT_EQ(Bd(r, c), A(r + 1, c + 2));
      EXPECT_EQ(Bc(r, c), A(r + 1, c + 2));
    }
  }
  CMatrixDouble Z(5, 6);
  Z.setZero();
  Z.insertMatrix(1, 2, Bd);
  Z.insertMatrixTransposed(2, 0, Bd);  // 3x2 block at (2,0)
  for (int r = 0; r < 3; r++)
  {
    for (int c = 0; c < 2; c++)
    {
      EXPECT_EQ(Z(r + 2, c), Bd(c, r));
    }
  }
  EXPECT_EQ(Z(1, 3), Bd(0, 1));
  EXPECT_ANY_THROW(A.extractMatrix(2, 3, 4, 0));
  EXPECT_ANY_THROW(A.extractMatrix(2, 3, 0, 4));
  EXPECT_ANY_THROW(Z.insertMatrix(4, 0, Bd));
  EXPECT_ANY_THROW(Z.insertMatrixTransposed(0, 5, Bd));
}

TEST(MatrixBaseDecomp, resizeKeepsContentsAndZeroesNewCells)
{
  CMatrixDouble A(2, 3);
  int k = 1;
  for (int r = 0; r < 2; r++)
  {
    for (int c = 0; c < 3; c++)
    {
      A(r, c) = k++;
    }
  }
  A.resize(3, 4);
  EXPECT_EQ(A(0, 0), 1.0);
  EXPECT_EQ(A(0, 2), 3.0);
  EXPECT_EQ(A(1, 2), 6.0);
  EXPECT_EQ(A(0, 3), 0.0);
  EXPECT_EQ(A(2, 0), 0.0);
  A.resize(1, 2);
  EXPECT_EQ(A(0, 0), 1.0);
  EXPECT_EQ(A(0, 1), 2.0);
}

// appendRow() on a matrix without columns gives a 1xN matrix
TEST(MatrixBaseDecomp, appendRowOnMatrixWithoutColumns)
{
  CMatrixDouble M(3, 0);
  const std::vector<double> v = {1.0, 2.0, 3.0, 4.0};
  M.appendRow(v);
  EXPECT_EQ(M.rows(), 1);
  EXPECT_EQ(M.cols(), 4);
}

TEST(MatrixBaseDecomp, emptyForZeroRowsMatrix)
{
  const CMatrixDouble M(0, 5);
  EXPECT_TRUE(M.empty());
}

TEST(OpsMatricesQuadForms, multiplyHCHtVariantsMatchNaive)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const CMatrixDouble H = randomMatrix(3, 5);
  const CMatrixDouble C = randomSymmetric(5);
  const Eigen::MatrixXd ref = H.asEigen() * C.asEigen() * H.asEigen().transpose();

  CMatrixDouble R;
  mrpt::math::multiply_HCHt(H, C, R);
  EXPECT_LT((R.asEigen() - ref).cwiseAbs().maxCoeff(), 1e-14);

  // Accumulate variant: R += H*C*H^T
  CMatrixDouble Racc = randomSymmetric(3);
  const Eigen::MatrixXd ref2 = Racc.asEigen() + ref;
  mrpt::math::multiply_HCHt(H, C, Racc, true);
  EXPECT_LT((Racc.asEigen() - ref2).cwiseAbs().maxCoeff(), 1e-14);

  // Fixed-size overload
  const mrpt::math::CMatrixFixed<double, 2, 3> Hf(randomMatrix(2, 3).asEigen());
  const mrpt::math::CMatrixFixed<double, 3, 3> Cf(randomSymmetric(3).asEigen());
  const auto Rf = mrpt::math::multiply_HCHt(Hf, Cf);
  const Eigen::Matrix2d reff = Hf.asEigen() * Cf.asEigen() * Hf.asEigen().transpose();
  EXPECT_LT((Rf.asEigen() - reff).cwiseAbs().maxCoeff(), 1e-14);

  // Aliasing: output is also the input covariance (square H)
  const CMatrixDouble Hs = randomMatrix(4, 4);
  CMatrixDouble Cs = randomSymmetric(4);
  const Eigen::MatrixXd ref3 = Hs.asEigen() * Cs.asEigen() * Hs.asEigen().transpose();
  mrpt::math::multiply_HCHt(Hs, Cs, Cs);
  EXPECT_LT((Cs.asEigen() - ref3).cwiseAbs().maxCoeff(), 1e-14);
}

TEST(OpsMatricesQuadForms, scalarQuadraticForms)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const CMatrixDouble C = randomSymmetric(4);
  const CMatrixDouble hcol = randomMatrix(4, 1);
  const CMatrixDouble hrow(hcol.asEigen().transpose().eval());
  const double ref = (hcol.asEigen().transpose() * C.asEigen() * hcol.asEigen())(0, 0);
  EXPECT_NEAR(mrpt::math::multiply_HtCH_scalar(hcol, C), ref, 1e-14);
  EXPECT_NEAR(mrpt::math::multiply_HCHt_scalar(hrow, C), ref, 1e-14);
  EXPECT_ANY_THROW(mrpt::math::multiply_HtCH_scalar(hrow, C));
}

TEST(OpsMatricesQuadForms, meanAndCovMatIsMaximumLikelihood)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const CMatrixDouble X = randomMatrix(50, 3);
  CVectorDouble mean;
  CMatrixDouble cov;
  mrpt::math::meanAndCovMat(X, mean, cov);
  const Eigen::RowVector3d m = X.asEigen().colwise().mean();
  const Eigen::MatrixXd Xc = X.asEigen().rowwise() - m;
  const Eigen::Matrix3d refCov = (Xc.transpose() * Xc) / 50.0;  // 1/N normalization
  for (int i = 0; i < 3; i++)
  {
    EXPECT_NEAR(mean[i], m[i], 1e-14);
  }
  EXPECT_LT((cov.asEigen() - refCov).cwiseAbs().maxCoeff(), 1e-14);

  CVectorDouble meanCols;
  CVectorDouble stdCols;
  mrpt::math::meanAndStdColumns(X, meanCols, stdCols, true);
  for (int i = 0; i < 3; i++)
  {
    EXPECT_NEAR(meanCols[i], m[i], 1e-14);
    EXPECT_NEAR(stdCols[i], std::sqrt(refCov(i, i) * 50.0 / 49.0), 1e-14);
  }
}

TEST(OpsMatricesQuadForms, extractSubmatrixSymmetricalBlocksUsesBlockIndices)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const CMatrixDouble M = randomSymmetric(8);
  const std::vector<size_t> blocks = {3, 1};
  CMatrixDouble E;
  mrpt::math::extractSubmatrixSymmetricalBlocks<2>(M, blocks, E);
  CMatrixDouble Ed;
  mrpt::math::extractSubmatrixSymmetricalBlocksDyn(M, 2, blocks, Ed);
  CMatrixDouble Es;
  mrpt::math::extractSubmatrixSymmetrical(M, {6, 7, 2, 3}, Es);
  ASSERT_EQ(E.rows(), 4);
  EXPECT_TRUE(E == Es);
  EXPECT_TRUE(Ed == Es);
}

TEST(OpsMatricesQuadForms, laplacianRowsSumToZero)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  CMatrixDouble W = randomMatrix(5, 5);
  W.asEigen() = W.asEigen().cwiseAbs();
  W.asEigen() = (W.asEigen() + W.asEigen().transpose()).eval();
  CMatrixDouble L;
  mrpt::math::laplacian(W, L);
  for (int r = 0; r < 5; r++)
  {
    EXPECT_NEAR(L.asEigen().row(r).sum(), 0.0, 1e-14);
    for (int c = 0; c < 5; c++)
    {
      if (r != c)
      {
        EXPECT_EQ(L(r, c), -W(r, c));
      }
    }
  }
}

TEST(MatrixBaseDecomp, eigOfEmptyMatrix)
{
  const CMatrixDouble A(0, 0);
  CMatrixDouble V;
  std::vector<double> D = {1.0};
  EXPECT_TRUE(A.eig(V, D));
  EXPECT_TRUE(D.empty());
  D = {1.0};
  EXPECT_TRUE(A.eig_symmetric(V, D));
  EXPECT_TRUE(D.empty());
  EXPECT_EQ(V.rows(), 0);
}

TEST(MatrixBaseDecomp, eigSymmetricRoundOffEigenvalueIsReset)
{
  // Rank-1 PSD matrix: one eigenvalue is exactly zero, up to round-off
  mrpt::math::CMatrixDouble22 A;
  A(0, 0) = 1.0;
  A(0, 1) = 0.5;
  A(1, 0) = 0.5;
  A(1, 1) = 0.25;
  mrpt::math::CMatrixDouble22 V;
  std::vector<double> D;
  ASSERT_TRUE(A.eig_symmetric(V, D));
  EXPECT_GE(D[0], 0.0);
  EXPECT_NEAR(D[0], 0.0, 1e-15);
  EXPECT_NEAR(D[1], 1.25, 1e-14);
}

TEST(MatrixBaseDecomp, fixedSizeCtorZeroesElements)
{
  const mrpt::math::CMatrixDouble33 m(3, 3);
  for (int r = 0; r < 3; r++)
  {
    for (int c = 0; c < 3; c++)
    {
      EXPECT_EQ(m(r, c), 0.0);
    }
  }
  EXPECT_ANY_THROW(mrpt::math::CMatrixDouble33(2, 3));
}

// The unscented transform must also accept singular (semi-definite) inputs
TEST(MatrixBaseDecomp, unscentedTransformSingularCovariance)
{
  mrpt::math::CVectorFixedDouble<3> xMean;
  xMean[0] = 1.0;
  xMean[1] = -2.0;
  xMean[2] = 0.5;
  // Rank-2 covariance:
  CMatrixDouble33 xCov;
  xCov.setZero();
  xCov(0, 0) = 1.0;
  xCov(0, 1) = 1.0;
  xCov(1, 0) = 1.0;
  xCov(1, 1) = 1.0;
  xCov(2, 2) = 2.0;
  Eigen::Matrix<double, 2, 3> A;
  A << 1.0, -2.0, 0.5, 0.3, 1.0, -1.0;

  mrpt::math::CVectorFixedDouble<2> yMean;
  mrpt::math::CMatrixDouble22 yCov;
  const double dummy = 0;
  mrpt::math::transform_gaussian_unscented(xMean, xCov, &linearMap23, dummy, yMean, yCov);
  const Eigen::Vector2d refMean = A * xMean.asEigen();
  const Eigen::Matrix2d refCov = A * xCov.asEigen() * A.transpose();
  EXPECT_NEAR((yMean.asEigen() - refMean).norm(), 0.0, 1e-8);
  EXPECT_NEAR((yCov.asEigen() - refCov).cwiseAbs().maxCoeff(), 0.0, 1e-6);

  // Indefinite matrices are rejected:
  xCov(2, 2) = -2.0;
  EXPECT_ANY_THROW(
      mrpt::math::transform_gaussian_unscented(xMean, xCov, &linearMap23, dummy, yMean, yCov));
}
