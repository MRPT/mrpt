/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/mat2eig.h>
#include <mrpt/math/math_frwds.h>  // forward declarations
#include <mrpt/math/ops_containers.h>  // Many generic operations

/** \file ops_matrices.h
 * This file implements miscelaneous matrix and matrix/vector operations, and
 * internal functions in mrpt::math::detail
 */
namespace mrpt
{
namespace math
{
/** R = H * C * H^t */
template <typename MAT_H, typename MAT_C, typename MAT_R>
inline void multiply_HCHt(
	const MAT_H& H, const MAT_C& C, MAT_R& R, bool accumResultInOutput = false)
{
	auto res = (mat2eig(H) * mat2eig(C) * mat2eig(H).transpose()).eval();
	if (accumResultInOutput) R.asEigen() += res;
	else
	{
		R.resize(res.rows(), res.cols());
		R.asEigen() = res;
	}
}

/** return a fixed-size matrix with the result of: H * C * H^t */
template <std::size_t H_ROWS, std::size_t H_COLS, typename Scalar>
mrpt::math::CMatrixFixed<Scalar, H_ROWS, H_ROWS> multiply_HCHt(
	const mrpt::math::CMatrixFixed<Scalar, H_ROWS, H_COLS>& H,
	const mrpt::math::CMatrixFixed<Scalar, H_COLS, H_COLS>& C)
{
	mrpt::math::CMatrixFixed<Scalar, H_ROWS, H_ROWS> R;
	R.asEigen() = (mat2eig(H) * mat2eig(C) * mat2eig(H).transpose()).eval();
	return R;
}

/** r (scalar) = H^t*C*H (H: column vector, C: symmetric matrix) */
template <typename VECTOR_H, typename MAT_C>
typename MAT_C::Scalar multiply_HtCH_scalar(const VECTOR_H& H, const MAT_C& C)
{
	ASSERT_EQUAL_(H.rows(), C.rows());
	ASSERT_EQUAL_(C.rows(), C.cols());
	return (mat2eig(H).transpose() * mat2eig(C) * mat2eig(H)).eval()(0, 0);
}

/** r (scalar) = H*C*H^t (H: row vector, C: symmetric matrix) */
template <typename VECTOR_H, typename MAT_C>
typename MAT_C::Scalar multiply_HCHt_scalar(const VECTOR_H& H, const MAT_C& C)
{
	ASSERT_EQUAL_(H.cols(), C.rows());
	ASSERT_EQUAL_(C.rows(), C.cols());
	return (mat2eig(H) * mat2eig(C) * mat2eig(H).transpose()).eval()(0, 0);
}

/** Computes the mean vector and covariance from a list of samples in an NxM
 * matrix, where each row is a sample, so the covariance is MxM.
 * \param v The set of data as a NxM matrix, of types: CMatrixDynamic
 * or CMatrixFixed
 * \param out_mean The output M-vector for the estimated mean.
 * \param out_cov The output MxM matrix for the estimated covariance matrix,
 * this can also be either a fixed-size of dynamic size matrix.
 * \sa mrpt::math::meanAndCovVec, math::mean,math::stddev, math::cov
 */
template <class MAT_IN, class VECTOR, class MAT_OUT>
void meanAndCovMat(const MAT_IN& v, VECTOR& out_mean, MAT_OUT& out_cov)
{
	const size_t N = v.rows();
	ASSERTMSG_(N > 0, "The input matrix contains no elements");
	const double N_inv = 1.0 / N;

	const size_t M = v.cols();
	ASSERTMSG_(M > 0, "The input matrix contains rows of length 0");

	// First: Compute the mean
	out_mean.assign(M, 0);
	for (size_t i = 0; i < N; i++)
		for (size_t j = 0; j < M; j++)
			out_mean[j] += v.coeff(i, j);
	out_mean *= N_inv;

	// Second: Compute the covariance
	//  Save only the above-diagonal part, then after averaging
	//  duplicate that part to the other half.
	out_cov.setZero(M, M);
	for (size_t i = 0; i < N; i++)
	{
		for (size_t j = 0; j < M; j++)
			out_cov(j, j) += square(v(i, j) - out_mean[j]);

		for (size_t j = 0; j < M; j++)
			for (size_t k = j + 1; k < M; k++)
				out_cov(j, k) +=
					(v(i, j) - out_mean[j]) * (v(i, k) - out_mean[k]);
	}
	for (size_t j = 0; j < M; j++)
		for (size_t k = j + 1; k < M; k++)
			out_cov(k, j) = out_cov(j, k);
	out_cov *= N_inv;
}

/** Computes a row with the mean values of each column in the matrix and the
 * associated vector with the standard deviation of each column.
 * \sa mean,meanAndStdAll \exception std::exception If the matrix/vector is
 * empty.
 * \param unbiased_variance Standard deviation is sum(vals-mean)/K, with
 * K=N-1 or N for unbiased_variance=true or false, respectively.
 */
template <class MAT_IN, class VEC>
void meanAndStdColumns(
	const MAT_IN& m, VEC& outMeanVector, VEC& outStdVector,
	const bool unbiased_variance = true)
{
	const auto N = m.rows(), M = m.cols();
	if (N == 0) throw std::runtime_error("meanAndStdColumns: Empty container.");
	const double N_inv = 1.0 / N;
	const double N_ =
		unbiased_variance ? (N > 1 ? 1.0 / (N - 1) : 1.0) : 1.0 / N;
	outMeanVector.resize(M);
	outStdVector.resize(M);
	for (decltype(m.cols()) i = 0; i < M; i++)
	{
		outMeanVector[i] = m.asEigen().col(i).array().sum() * N_inv;
		outStdVector[i] = std::sqrt(
			(m.asEigen().col(i).array() - outMeanVector[i]).square().sum() *
			N_);
	}
}

/** Computes the covariance matrix from a list of samples in an NxM matrix,
 * where each row is a sample, so the covariance is MxM.
 * \param v The set of data, as a NxM matrix.
 * \param out_cov The output MxM matrix for the estimated covariance matrix.
 * \sa math::mean,math::stddev, math::cov
 */
template <class MATRIX>
CMatrixDouble cov(const MATRIX& v)
{
	CVectorDouble m;
	CMatrixDouble C;
	meanAndCovMat(v, m, C);
	return C;
}

/** A useful macro for saving matrixes to a file while debugging. */
#define SAVE_MATRIX(M) M.saveToTextFile(#M ".txt");

/** Only for vectors/arrays "v" of length3, compute out = A * Skew(v), where
 * Skew(v) is the skew symmetric matric generated from \a v (see
 * mrpt::math::skew_symmetric3)
 */
template <class MAT_A, class SKEW_3VECTOR, class MAT_OUT>
void multiply_A_skew3(const MAT_A& A, const SKEW_3VECTOR& v, MAT_OUT& out)
{
	MRPT_START
	ASSERT_EQUAL_(A.cols(), 3);
	ASSERT_EQUAL_(v.size(), 3);
	const size_t N = A.rows();
	out.setSize(N, 3);
	for (size_t i = 0; i < N; i++)
	{
		out(i, 0) = A(i, 1) * v[2] - A(i, 2) * v[1];
		out(i, 1) = -A(i, 0) * v[2] + A(i, 2) * v[0];
		out(i, 2) = A(i, 0) * v[1] - A(i, 1) * v[0];
	}
	MRPT_END
}

/** Only for vectors/arrays "v" of length3, compute out = Skew(v) * A, where
 * Skew(v) is the skew symmetric matric generated from \a v (see
 * mrpt::math::skew_symmetric3)
 */
template <class SKEW_3VECTOR, class MAT_A, class MAT_OUT>
void multiply_skew3_A(const SKEW_3VECTOR& v, const MAT_A& A, MAT_OUT& out)
{
	MRPT_START
	ASSERT_EQUAL_(A.rows(), 3);
	ASSERT_EQUAL_(v.size(), 3);
	const size_t N = A.cols();
	out.setSize(3, N);
	for (size_t i = 0; i < N; i++)
	{
		out(0, i) = -A(1, i) * v[2] + A(2, i) * v[1];
		out(1, i) = A(0, i) * v[2] - A(2, i) * v[0];
		out(2, i) = -A(0, i) * v[1] + A(1, i) * v[0];
	}
	MRPT_END
}

/** Computes the Laplacian of a square graph weight matrix.
 *  The laplacian matrix is L = D - W, with D a diagonal matrix with the
 * degree of each node, W the edge weights.
 */
template <typename MATIN, typename MATOUT>
void laplacian(const MATIN& g, MATOUT& ret)
{
	if (g.rows() != g.cols())
		throw std::runtime_error("laplacian: Defined for square matrixes only");
	const auto N = g.rows();
	ret = g;
	ret *= -1;
	for (typename MATIN::Index i = 0; i < N; i++)
	{
		typename MATIN::Scalar deg = 0;
		for (typename MATIN::Index j = 0; j < N; j++)
			deg += g(j, i);
		ret(i, i) += deg;
	}
}

/** Get a submatrix from a square matrix, by collecting the elements
 * M(idxs,idxs), where idxs is a sequence
 * {block_indices(i):block_indices(i)+BLOCKSIZE-1} for all "i" up to the
 * size of block_indices. A perfect application of this method is in
 * extracting covariance matrices of a subset of variables from the full
 * covariance matrix. \sa extractSubmatrix, extractSubmatrixSymmetrical
 */
template <std::size_t BLOCKSIZE, typename MAT, typename MATRIX>
void extractSubmatrixSymmetricalBlocks(
	const MAT& m, const std::vector<size_t>& block_indices, MATRIX& out)
{
	if (BLOCKSIZE < 1)
		throw std::runtime_error(
			"extractSubmatrixSymmetricalBlocks: BLOCKSIZE must be >=1");
	if (m.cols() != m.rows())
		throw std::runtime_error(
			"extractSubmatrixSymmetricalBlocks: Matrix is not square.");

	const size_t N = block_indices.size();
	const size_t nrows_out = N * BLOCKSIZE;
	out.resize(nrows_out, nrows_out);
	if (!N) return;	 // Done
	for (size_t dst_row_blk = 0; dst_row_blk < N; ++dst_row_blk)
	{
		for (size_t dst_col_blk = 0; dst_col_blk < N; ++dst_col_blk)
		{
#if defined(_DEBUG)
			if (block_indices[dst_col_blk] * BLOCKSIZE + BLOCKSIZE - 1 >=
				size_t(m.cols()))
				throw std::runtime_error(
					"extractSubmatrixSymmetricalBlocks: Indices out of "
					"range!");
#endif
			out.asEigen().template block<BLOCKSIZE, BLOCKSIZE>(
				dst_row_blk * BLOCKSIZE, dst_col_blk * BLOCKSIZE) =
				m.asEigen().template block<BLOCKSIZE, BLOCKSIZE>(
					block_indices[dst_row_blk] * BLOCKSIZE,
					block_indices[dst_col_blk] * BLOCKSIZE);
		}
	}
}

//! \overload for BLOCKSIZE determined at runtime
template <typename MAT, typename MATRIX>
void extractSubmatrixSymmetricalBlocksDyn(
	const MAT& m, const std::size_t BLOCKSIZE,
	const std::vector<size_t>& block_indices, MATRIX& out)
{
	if (BLOCKSIZE < 1)
		throw std::runtime_error(
			"extractSubmatrixSymmetricalBlocks: BLOCKSIZE must be >=1");
	if (m.cols() != m.rows())
		throw std::runtime_error(
			"extractSubmatrixSymmetricalBlocks: Matrix is not square.");

	const size_t N = block_indices.size();
	const size_t nrows_out = N * BLOCKSIZE;
	out.resize(nrows_out, nrows_out);
	if (!N) return;	 // Done
	for (size_t dst_row_blk = 0; dst_row_blk < N; ++dst_row_blk)
	{
		for (size_t dst_col_blk = 0; dst_col_blk < N; ++dst_col_blk)
		{
#if defined(_DEBUG)
			if (block_indices[dst_col_blk] * BLOCKSIZE + BLOCKSIZE - 1 >=
				size_t(m.cols()))
				throw std::runtime_error(
					"extractSubmatrixSymmetricalBlocks: Indices out of "
					"range!");
#endif
			out.block(
				dst_row_blk * BLOCKSIZE, dst_col_blk * BLOCKSIZE, BLOCKSIZE,
				BLOCKSIZE) =
				m.block(
					block_indices[dst_row_blk] * BLOCKSIZE,
					block_indices[dst_col_blk] * BLOCKSIZE, BLOCKSIZE,
					BLOCKSIZE);
		}
	}
}

/** Get a submatrix from a square matrix, by collecting the elements
 * M(idxs,idxs), where idxs is the sequence of indices passed as argument. A
 * perfect application of this method is in extracting covariance matrices
 * of a subset of variables from the full covariance matrix. \sa
 * extractSubmatrix, extractSubmatrixSymmetricalBlocks
 */
template <typename MAT, typename MATRIX>
void extractSubmatrixSymmetrical(
	const MAT& m, const std::vector<size_t>& indices, MATRIX& out)
{
	if (m.cols() != m.rows())
		throw std::runtime_error(
			"extractSubmatrixSymmetrical: Matrix is not square.");

	const size_t N = indices.size();
	const size_t nrows_out = N;
	out.resize(nrows_out, nrows_out);
	if (!N) return;	 // Done
	for (size_t dst_row_blk = 0; dst_row_blk < N; ++dst_row_blk)
		for (size_t dst_col_blk = 0; dst_col_blk < N; ++dst_col_blk)
			out(dst_row_blk, dst_col_blk) =
				m(indices[dst_row_blk], indices[dst_col_blk]);
}

/**  @} */	// end of grouping

}  // namespace math
}  // namespace mrpt
