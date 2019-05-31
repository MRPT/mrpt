/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/MatrixVectorBase.h>

namespace mrpt::math
{
/**  Base CRTP class for all MRPT matrices.
 *
 * See MatrixVectorBase
 *
 * \sa CMatrixFixed
 * \ingroup mrpt_math_grp
 */
template <typename Scalar, class Derived>
class MatrixBase : public MatrixVectorBase<Scalar, Derived>
{
   public:
	Derived& mbDerived() { return static_cast<Derived&>(*this); }
	const Derived& mbDerived() const
	{
		return static_cast<const Derived&>(*this);
	}

	/** Resize to NxN, set all entries to zero, except the main diagonal which
	 * is set to `value` */
	void setDiagonal(const std::size_t N, const Scalar value)
	{
		mbDerived().resize(N, N);
		for (typename Derived::Index r = 0; r < mbDerived().rows(); r++)
			for (typename Derived::Index c = 0; c < mbDerived().cols(); c++)
				mbDerived()(r, c) = (r == c) ? value : 0;
	}
	/** Set all entries to zero, except the main diagonal which is set to
	 * `value` */
	void setDiagonal(const Scalar value)
	{
		ASSERT_EQUAL_(mbDerived().cols(), mbDerived().rows());
		setDiagonal(mbDerived().cols(), value);
	}
	/** Resizes to NxN, with N the length of the input vector, set all entries
	 * to zero, except the main diagonal which is set to values in the vector.
	 */
	void setDiagonal(const std::vector<Scalar>& diags)
	{
		const std::size_t N = diags.size();
		mbDerived().setZero(N, N);
		for (std::size_t i = 0; i < N; i++) mbDerived()(i, i) = diags[i];
	}
	void setIdentity()
	{
		ASSERT_EQUAL_(mbDerived().rows(), mbDerived().cols());
		setDiagonal(mbDerived().cols(), 1);
	}
	void setIdentity(const std::size_t N) { setDiagonal(N, 1); }

	static Derived Identity()
	{
		ASSERTMSG_(
			Derived::RowsAtCompileTime > 0 && Derived::ColsAtCompileTime > 0,
			"Identity() without arguments can be used only for fixed-size "
			"matrices/vectors");
		Derived m;
		m.setIdentity();
		return m;
	}
	static Derived Identity(const std::size_t N)
	{
		Derived m;
		m.setIdentity(N);
		return m;
	}

	/** this = A*B, with A & B of the same type of this.
	 * For products of different matrix types, use the regular * operator (which
	 * requires the `<Eigen/Dense>` header) */
	void matProductOf_AB(const Derived& A, const Derived& B);

	/** @name Operations that DO require `#include <Eigen/Dense>` in user code
	 * @{ */

	auto col(int colIdx)
	{
		internalAssertEigenDefined<Derived>();
		return mbDerived().asEigen().col(colIdx);
	}
	auto col(int colIdx) const
	{
		internalAssertEigenDefined<Derived>();
		return mbDerived().asEigen().col(colIdx);
	}

	auto row(int rowIdx)
	{
		internalAssertEigenDefined<Derived>();
		return mbDerived().asEigen().row(rowIdx);
	}
	auto row(int rowIdx) const
	{
		internalAssertEigenDefined<Derived>();
		return mbDerived().asEigen().row(rowIdx);
	}

	template <typename VECTOR_LIKE>
	void extractRow(int rowIdx, VECTOR_LIKE& v) const
	{
		ASSERT_BELOW_(rowIdx, mbDerived().rows());
		v.resize(mbDerived().cols());
		for (typename Derived::Index i = 0; i < mbDerived().cols(); i++)
			v[i] = mbDerived().coeff(rowIdx, i);
	}

	template <typename VECTOR_LIKE>
	void extractColumn(int colIdx, VECTOR_LIKE& v) const
	{
		ASSERT_BELOW_(colIdx, mbDerived().cols());
		v.resize(mbDerived().rows());
		for (typename Derived::Index i = 0; i < mbDerived().rows(); i++)
			v[i] = mbDerived().coeff(i, colIdx);
	}

	/** @} */

	/** @name Standalone operations (do NOT require `#include <Eigen/Dense>`)
	 * @{ */
	/** Determinant of matrix. */
	Scalar det() const;

	/** Returns the inverse of a general matrix using LU */
	Derived inverse() const;

	/** Returns the inverse of a symmetric matrix using LLt */
	Derived inverse_LLt() const;

	/** Finds the rank of the matrix via LU decomposition.
	 * Uses Eigen's default threshold unless `threshold>0`. */
	int rank(Scalar threshold = 0) const;

	/** Cholesky M=U<sup>T</sup> * U decomposition for symmetric matrix
	 * (upper-half of the matrix is actually ignored.
	 * \return false if Cholesky fails
	 */
	bool chol(Derived& U) const;

	/** Computes the eigenvectors and eigenvalues for a square, general matrix.
	 * Use eig_symmetric() for symmetric matrices for better accuracy and
	 * performance.
	 * Eigenvectors are the columns of the returned matrix, and their order
	 * matches that of returned eigenvalues.
	 * \param[in] sorted If true, eigenvalues (and eigenvectors) will be sorted
	 * in ascending order.
	 * \param[out] eVecs The container where eigenvectors will be stored.
	 * \param[out] eVals The container where eigenvalues will be stored.
	 * \return false if eigenvalues could not be determined.
	 */
	bool eig(
		Derived& eVecs, std::vector<Scalar>& eVals, bool sorted = true) const;

	/** Read: eig()
	 * \note This only uses the **lower-triangular** part of the matrix */
	bool eig_symmetric(
		Derived& eVecs, std::vector<Scalar>& eVals, bool sorted = true) const;

	/** Returns the maximum value in the diagonal. */
	Scalar maximumDiagonal() const;
	/** Returns the minimum value in the diagonal. */
	Scalar minimumDiagonal() const;

	/** Returns the trace of the matrix (not necessarily square). */
	Scalar trace() const;

	/** Removes columns of the matrix.
	 * This "unsafe" version assumes indices sorted in ascending order. */
	void unsafeRemoveColumns(const std::vector<std::size_t>& idxs);

	/** Removes columns of the matrix. Indices may be unsorted and duplicated */
	void removeColumns(const std::vector<std::size_t>& idxsToRemove);

	/** Removes rows of the matrix.
	 * This "unsafe" version assumes indices sorted in ascending order. */
	void unsafeRemoveRows(const std::vector<std::size_t>& idxs);

	/** Removes rows of the matrix. Indices may be unsorted and duplicated */
	void removeRows(const std::vector<std::size_t>& idxsToRemove);

	/** Copies the given input submatrix/vector into this matrix/vector,
	 * starting at the given top-left coordinates. */
	template <typename OTHERMATVEC>
	void insertMatrix(
		const int row_start, const int col_start, const OTHERMATVEC& submat)
	{
		ASSERT_BELOWEQ_(row_start + submat.rows(), mbDerived().rows());
		ASSERT_BELOWEQ_(col_start + submat.cols(), mbDerived().cols());
		for (int r = 0; r < submat.rows(); r++)
			for (int c = 0; c < submat.cols(); c++)
				mbDerived()(r + row_start, c + col_start) = submat(r, c);
	}
	/** Like insertMatrix(), but inserts `submat'` (transposed) */
	template <typename OTHERMATVEC>
	void insertMatrixTransposed(
		const int row_start, const int col_start, const OTHERMATVEC& submat)
	{
		ASSERT_BELOWEQ_(row_start + submat.cols(), mbDerived().rows());
		ASSERT_BELOWEQ_(col_start + submat.rows(), mbDerived().cols());
		for (int r = 0; r < submat.cols(); r++)
			for (int c = 0; c < submat.rows(); c++)
				mbDerived()(r + row_start, c + col_start) = submat(c, r);
	}

	template <int BLOCK_ROWS, int BLOCK_COLS>
	CMatrixFixed<Scalar, BLOCK_ROWS, BLOCK_COLS> extractMatrix(
		const int start_row = 0, const int start_col = 0) const
	{
		ASSERT_BELOWEQ_(start_row + BLOCK_ROWS, mbDerived().rows());
		ASSERT_BELOWEQ_(start_col + BLOCK_COLS, mbDerived().cols());

		CMatrixFixed<Scalar, BLOCK_ROWS, BLOCK_COLS> ret;
		for (int r = 0; r < BLOCK_ROWS; r++)
			for (int c = 0; c < BLOCK_COLS; c++)
				ret(r, c) = mbDerived()(r + start_row, c + start_col);
		return ret;
	}

	CMatrixDynamic<Scalar> extractMatrix(
		const int BLOCK_ROWS, const int BLOCK_COLS, const int start_row,
		const int start_col) const
	{
		ASSERT_BELOWEQ_(start_row + BLOCK_ROWS, mbDerived().rows());
		ASSERT_BELOWEQ_(start_col + BLOCK_COLS, mbDerived().cols());

		CMatrixDynamic<Scalar> ret(BLOCK_ROWS, BLOCK_COLS);
		for (int r = 0; r < BLOCK_ROWS; r++)
			for (int c = 0; c < BLOCK_COLS; c++)
				ret(r, c) = mbDerived()(r + start_row, c + start_col);
		return ret;
	}

	/** this = A * A<sup>T</sup> */
	template <typename MAT_A>
	void matProductOf_AAt(const MAT_A& A)
	{
		using Index = typename Derived::Index;
		const auto N = A.rows(), Ninner = A.cols();
		mbDerived().resize(N, N);
		for (Index r = 0; r < N; r++)
		{
			// Only 1/2 of computations required:
			for (Index c = r; c < N; c++)
			{
				typename Derived::Scalar s = 0;
				for (Index i = 0; i < Ninner; i++) s += A(r, i) * A(c, i);
				mbDerived()(r, c) = s;
				mbDerived()(c, r) = s;
			}
		}
	}
	/** this = A<sup>T</sup> * A */
	template <typename MAT_A>
	void matProductOf_AtA(const MAT_A& A)
	{
		using Index = typename Derived::Index;
		const auto N = A.cols(), Ninner = A.rows();
		mbDerived().resize(N, N);
		for (Index r = 0; r < N; r++)
		{
			// Only 1/2 of computations required:
			for (Index c = r; c < N; c++)
			{
				typename Derived::Scalar s = 0;
				for (Index i = 0; i < Ninner; i++) s += A(i, r) * A(i, c);
				mbDerived()(r, c) = s;
				mbDerived()(c, r) = s;
			}
		}
	}

	/** @} */
};

}  // namespace mrpt::math
