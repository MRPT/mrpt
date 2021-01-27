/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/exceptions.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/MatrixBase.h>

#include <Eigen/Eigenvalues>  // EigenSolver
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace mrpt::math
{
template <typename Scalar, class Derived>
void MatrixBase<Scalar, Derived>::unsafeRemoveColumns(
	const std::vector<std::size_t>& idxs)
{
	std::size_t k = 1;
	const auto nR = mbDerived().rows();
	for (auto it = idxs.rbegin(); it != idxs.rend(); ++it, ++k)
	{
		const auto nC = mbDerived().cols() - *it - k;
		if (nC > 0)
			mbDerived().asEigen().block(0, *it, nR, nC) =
				mbDerived().asEigen().block(0, *it + 1, nR, nC).eval();
	}
	mbDerived().setSize(nR, mbDerived().cols() - idxs.size());
}

template <typename Scalar, class Derived>
void MatrixBase<Scalar, Derived>::removeColumns(
	const std::vector<std::size_t>& idxsToRemove)
{
	std::vector<std::size_t> idxs = idxsToRemove;
	std::sort(idxs.begin(), idxs.end());
	auto itEnd = std::unique(idxs.begin(), idxs.end());
	idxs.resize(itEnd - idxs.begin());
	unsafeRemoveColumns(idxs);
}

template <typename Scalar, class Derived>
void MatrixBase<Scalar, Derived>::unsafeRemoveRows(
	const std::vector<size_t>& idxs)
{
	std::size_t k = 1;
	const auto nC = mbDerived().cols();
	for (auto it = idxs.rbegin(); it != idxs.rend(); ++it, ++k)
	{
		const auto nR = mbDerived().rows() - *it - k;
		if (nR > 0)
			mbDerived().asEigen().block(*it, 0, nR, nC) =
				mbDerived().asEigen().block(*it + 1, 0, nR, nC).eval();
	}
	mbDerived().setSize(mbDerived().rows() - idxs.size(), nC);
}

template <typename Scalar, class Derived>
void MatrixBase<Scalar, Derived>::removeRows(
	const std::vector<size_t>& idxsToRemove)
{
	std::vector<std::size_t> idxs = idxsToRemove;
	std::sort(idxs.begin(), idxs.end());
	auto itEnd = std::unique(idxs.begin(), idxs.end());
	idxs.resize(itEnd - idxs.begin());
	unsafeRemoveRows(idxs);
}

template <typename Scalar, class Derived>
Scalar MatrixBase<Scalar, Derived>::det() const
{
	return mbDerived().asEigen().eval().determinant();
}

namespace detail
{
// Aux func to sort by ascending eigenvalues:
template <typename Scalar, typename VEC1, typename MATRIX1, typename MATRIX2>
void sortEigResults(
	const VEC1& eVals, const MATRIX1& eVecs, std::vector<Scalar>& sorted_eVals,
	MATRIX2& sorted_eVecs)
{
	const int64_t N = static_cast<int64_t>(eVals.size());
	std::vector<std::pair<Scalar, int64_t>> D;
	D.reserve(N);
	for (int64_t i = 0; i < N; i++)
		D.emplace_back(eVals[i], i);
	std::sort(D.begin(), D.end());

	// store:
	sorted_eVecs.resize(eVecs.rows(), eVecs.cols());
	sorted_eVals.resize(N);
	for (int64_t i = 0; i < N; i++)
	{
		sorted_eVals[i] = D[i].first;
		sorted_eVecs.col(i) = eVecs.col(D[i].second);
	}
}
}  // namespace detail

template <typename Scalar, class Derived>
bool MatrixBase<Scalar, Derived>::eig(
	Derived& eVecs, std::vector<Scalar>& eVals, bool sorted) const
{
	Eigen::EigenSolver<typename Derived::eigen_t> es(mbDerived().asEigen());
	if (es.info() != Eigen::Success) return false;
	const auto eigenVal = es.eigenvalues().real();
	ASSERT_EQUAL_(eigenVal.rows(), mbDerived().rows());
	const auto N = eigenVal.rows();

	if (sorted)
	{
		detail::sortEigResults(
			eigenVal, es.eigenvectors().real(), eVals, eVecs);
	}
	else
	{
		eVals.resize(N);
		eVecs = es.eigenvectors().real();
		for (int i = 0; i < N; i++)
			eVals[i] = eigenVal[i];
	}
	return true;
}

template <typename Scalar, class Derived>
bool MatrixBase<Scalar, Derived>::eig_symmetric(
	Derived& eVecs, std::vector<Scalar>& eVals, bool sorted) const
{
	Eigen::SelfAdjointEigenSolver<typename Derived::eigen_t> es(
		mbDerived().asEigen());
	if (es.info() != Eigen::Success) return false;
	const auto eigenVal = es.eigenvalues().real();
	ASSERT_EQUAL_(eigenVal.rows(), mbDerived().rows());
	const auto N = eigenVal.rows();

	if (sorted)
	{
		detail::sortEigResults(
			eigenVal, es.eigenvectors().real(), eVals, eVecs);
	}
	else
	{
		eVals.resize(N);
		eVecs = es.eigenvectors().real();
		for (int i = 0; i < N; i++)
			eVals[i] = eigenVal[i];
	}
	return true;
}

template <typename Scalar, class Derived>
int MatrixBase<Scalar, Derived>::rank(Scalar threshold) const
{
	Eigen::FullPivLU<typename Derived::eigen_t> lu(
		mbDerived().asEigen().eval());
	if (threshold > 0) lu.setThreshold(threshold);
	return lu.rank();
}

template <typename Scalar, class Derived>
bool MatrixBase<Scalar, Derived>::chol(Derived& U) const
{
	Eigen::LLT<typename Derived::eigen_t> Chol =
		mbDerived().asEigen().template selfadjointView<Eigen::Lower>().llt();
	if (Chol.info() == Eigen::NoConvergence) return false;
	U = typename Derived::eigen_t(Chol.matrixU());
	return true;
}

template <typename Scalar, class Derived>
void MatrixBase<Scalar, Derived>::matProductOf_AB(
	const Derived& A, const Derived& B)
{
	mbDerived().resize(A.rows(), B.cols());
	mbDerived().asEigen() = (A.asEigen() * B.asEigen()).eval();
}

template <typename Scalar, class Derived>
Derived MatrixBase<Scalar, Derived>::inverse() const
{
	ASSERT_EQUAL_(mbDerived().cols(), mbDerived().rows());
	const auto N = mbDerived().cols();
	const Derived I = Derived::Identity(N);
	Derived inv(mrpt::math::UNINITIALIZED_MATRIX);
	inv.resize(N, N);
	inv.asEigen() = mbDerived().asEigen().lu().solve(I.asEigen()).eval();
	return inv;
}

template <typename Scalar, class Derived>
Derived MatrixBase<Scalar, Derived>::inverse_LLt() const
{
	ASSERT_EQUAL_(mbDerived().cols(), mbDerived().rows());
	const auto N = mbDerived().cols();
	const Derived I = Derived::Identity(N);
	Derived inv(mrpt::math::UNINITIALIZED_MATRIX);
	inv.resize(N, N);
	inv.asEigen() = mbDerived().asEigen().llt().solve(I.asEigen()).eval();
	return inv;
}

template <typename Scalar, class Derived>
Scalar MatrixBase<Scalar, Derived>::maximumDiagonal() const
{
	return mbDerived().asEigen().diagonal().maxCoeff();
}

template <typename Scalar, class Derived>
Scalar MatrixBase<Scalar, Derived>::minimumDiagonal() const
{
	return mbDerived().asEigen().diagonal().minCoeff();
}

template <typename Scalar, class Derived>
Scalar MatrixBase<Scalar, Derived>::trace() const
{
	return mbDerived().asEigen().trace();
}

}  // namespace mrpt::math
