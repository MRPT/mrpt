/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/alignment_req.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/math/MatrixBase.h>
#include <mrpt/math/math_frwds.h>  // Forward declarations
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/math/point_poses2vectors.h>  // MRPT_MATRIX_CONSTRUCTORS_FROM_POSES()
#include <mrpt/typemeta/TTypeName.h>
#include <mrpt/typemeta/num_to_string.h>
#include <array>
#include <cstddef>  // std::size_t

namespace mrpt::math
{
/** A compile-time fixed-size numeric matrix container.
 * It uses a RowMajor element memory layout.
 *
 * \sa CMatrixDynamic (for dynamic-size matrices)
 * \note For a complete introduction to Matrices and vectors in MRPT, see:
 * https://www.mrpt.org/Matrices_vectors_arrays_and_Linear_Algebra_MRPT_and_Eigen_classes
 * \ingroup mrpt_math_grp
 */
template <typename T, std::size_t ROWS, std::size_t COLS>
class CMatrixFixed : public MatrixBase<T, CMatrixFixed<T, ROWS, COLS>>
{
   private:
	/** RowMajor matrix data */
	using vec_t = std::array<T, ROWS * COLS>;
	alignas(MRPT_MAX_ALIGN_BYTES) vec_t m_data;

   public:
	/** @name Matrix type definitions
	 * @{ */
	/** The type of the matrix elements */
	using value_type = T;
	using Scalar = T;
	using Index = int;
	using reference = T&;
	using const_reference = const T&;
	using size_type = int;
	using difference_type = std::ptrdiff_t;
	constexpr static int RowsAtCompileTime = ROWS;
	constexpr static int ColsAtCompileTime = COLS;
	constexpr static int SizeAtCompileTime = ROWS * COLS;
	constexpr static int is_mrpt_type = 1;
	constexpr static int StorageOrder =
		(ROWS != 1 && COLS == 1) ? 0 /*colMajor*/ : 1 /*rowMajor*/;
	using eigen_t = Eigen::Matrix<T, ROWS, COLS, StorageOrder, ROWS, COLS>;
	/** @} */

	/** @name Iterators interface
	 * @{ */
	using iterator = typename vec_t::iterator;
	using const_iterator = typename vec_t::const_iterator;
	iterator begin() { return m_data.begin(); }
	iterator end() { return m_data.end(); }
	const_iterator begin() const { return m_data.begin(); }
	const_iterator end() const { return m_data.end(); }
	const_iterator cbegin() const { return m_data.begin(); }
	const_iterator cend() const { return m_data.end(); }
	/** @} */

	/** @name Constructors, assignment operators, initializers
	 *  @{ */

	/** Default constructor, initializes all elements to zero */
	inline CMatrixFixed() { m_data.fill(0); }

	/** Constructor which leaves the matrix uninitialized.
	 *  Example of usage: CMatrixFixed<double,3,2>
	 * M(mrpt::math::UNINITIALIZED_MATRIX);
	 */
	inline CMatrixFixed(TConstructorFlags_Matrices) {}

	/** Initializes from a C array with RowMajor values */
	template <size_t N>
	explicit CMatrixFixed(const T (&vals)[N])
	{
		this->loadFromArray(vals);
	}

	/** Initializes from a plain buffer with RowMajor values */
	explicit CMatrixFixed(const T* data) { this->loadFromRawPointer(data); }

	/** Convert from Eigen matrix */
	template <class Derived>
	explicit CMatrixFixed(const Eigen::MatrixBase<Derived>& m)
	{
		*this = m;
	}
	/** Convert from Eigen product */
	template <typename _Lhs, typename _Rhs, int Option>
	explicit CMatrixFixed(const Eigen::Product<_Lhs, _Rhs, Option>& p)
	{
		*this = p.eval();
	}
	/** Convert from Eigen binary op */
	template <typename Op, typename Lhs, typename Rhs>
	explicit CMatrixFixed(const Eigen::CwiseBinaryOp<Op, Lhs, Rhs>& p)
	{
		*this = p.eval();
	}

	/** Convert from Eigen block */
	template <typename VectorType, int Size>
	explicit CMatrixFixed(const Eigen::VectorBlock<VectorType, Size>& m)
	{
		*this = m;
	}

	/** Convenient ctor from size: in this class, it throws if size does not
	 * match compile-time size. It is provided for the sake of offering a
	 * uniform API with CMatrixDynamic. */
	CMatrixFixed(const size_type rows, const size_type cols)
	{
		ASSERT_EQUAL_(cols, static_cast<size_type>(COLS));
		ASSERT_EQUAL_(rows, static_cast<size_type>(ROWS));
	}

	MRPT_MATRIX_CONSTRUCTORS_FROM_POSES(CMatrixFixed)

	template <class MAT>
	void setFromMatrixLike(const MAT& m)
	{
		MRPT_START
		setSize(m.rows(), m.cols());
		for (Index r = 0; r < rows(); r++)
			for (Index c = 0; c < cols(); c++) (*this)(r, c) = m(r, c);
		MRPT_END
	}

	/** Assignment from an Eigen matrix */
	template <class Derived>
	CMatrixFixed& operator=(const Eigen::MatrixBase<Derived>& m)
	{
		MRPT_START
		setFromMatrixLike(m);
		return *this;
		MRPT_END
	}
	/** Assignment from an Eigen vector block */
	template <typename VectorType, int Size>
	CMatrixFixed& operator=(const Eigen::VectorBlock<VectorType, Size>& m)
	{
		MRPT_START
		setFromMatrixLike(m.eval());
		return *this;
		MRPT_END
	}

	/** Assignment from a Dynamic matrix */
	template <typename U>
	CMatrixFixed& operator=(const CMatrixDynamic<U>& m)
	{
		MRPT_START
		setFromMatrixLike(m);
		return *this;
		MRPT_END
	}

	template <typename VECTOR>
	void loadFromArray(const VECTOR& vals)
	{
		MRPT_START
		const auto LEN = std::size(vals);
		ASSERT_EQUAL_(LEN, ROWS * COLS);
		for (size_t r = 0, i = 0; r < ROWS; r++)
			for (size_t c = 0; c < COLS; c++) m_data[r * COLS + c] = vals[i++];
		MRPT_END
	}

	/** Initializes from a plain buffer with RowMajor values. Unsafe, prefer
	 * loadFromArray() wherever possible, to ensure buffer length checks. */
	void loadFromRawPointer(const T* data)
	{
		for (size_t r = 0, i = 0; r < ROWS; r++)
			for (size_t c = 0; c < COLS; c++) m_data[r * COLS + c] = data[i++];
	}

	/** Throws if size does not match with the fixed matrix size */
	void setSize(
		size_t row, size_t col, [[maybe_unused]] bool zeroNewElements = false)
	{
		ASSERT_EQUAL_(row, ROWS);
		ASSERT_EQUAL_(col, COLS);
	}

	void swap(CMatrixFixed& o) { m_data.swap(o.m_data); }

	// These ones are to make template code compatible with Eigen & mrpt:
	CMatrixFixed& derived() { return *this; }
	const CMatrixFixed& derived() const { return *this; }
	void conservativeResize(size_t row, size_t col) { setSize(row, col); }

	void resize(size_t n)
	{
		if (ROWS == 1)
			ASSERT_EQUAL_(COLS, n);
		else if (COLS == 1)
			ASSERT_EQUAL_(ROWS, n);
		else
			THROW_EXCEPTION("resize() can be invoked on 1xN or Nx1 only");
	}

	/** Throws if size does not match with the fixed matrix size */
	inline void resize(
		const matrix_size_t& siz, [[maybe_unused]] bool zeroNewElements = false)
	{
		resize(siz[0], siz[1]);
	}
	void resize(size_t row, size_t col)
	{
		ASSERT_EQUAL_(row, ROWS);
		ASSERT_EQUAL_(col, COLS);
	}

	/** Number of rows in the matrix \sa rows() */
	constexpr size_type rows() const { return ROWS; }

	/** Number of columns in the matrix \sa rows() */
	constexpr size_type cols() const { return COLS; }

	/** Get a 2-vector with [NROWS NCOLS] (as in MATLAB command size(x)) */
	constexpr matrix_size_t size() const
	{
		matrix_size_t dims;
		dims[0] = ROWS;
		dims[1] = COLS;
		return dims;
	}

	/** @} */

	/** @name Matrix element access & modifiers
	 *  @{ */

	/** Get as an Eigen-compatible Eigen::Map object  */
	template <
		typename EIGEN_MATRIX = eigen_t,
		typename EIGEN_MAP = Eigen::Map<
			EIGEN_MATRIX, MRPT_MAX_ALIGN_BYTES, Eigen::InnerStride<1>>>
	EIGEN_MAP asEigen()
	{
		static_assert(
			std::is_same_v<EIGEN_MATRIX, eigen_t>,
			"Please, do not override the default template arguments of this "
			"method.");
		return EIGEN_MAP(&m_data[0], ROWS, COLS);
	}

	/** \overload (const version) */
	template <
		typename EIGEN_MATRIX = eigen_t,
		typename EIGEN_MAP = Eigen::Map<
			const EIGEN_MATRIX, MRPT_MAX_ALIGN_BYTES, Eigen::InnerStride<1>>>
	EIGEN_MAP asEigen() const
	{
		static_assert(
			std::is_same_v<EIGEN_MATRIX, eigen_t>,
			"Please, do not override the default template arguments of this "
			"method.");
		return EIGEN_MAP(&m_data[0], ROWS, COLS);
	}

	/** Access (row,col), without out-of-bounds check (except in Debug builds)
	 */
	inline T& operator()(int row, int col)
	{
		ASSERTDEB_(static_cast<std::size_t>(row) < ROWS);
		ASSERTDEB_(static_cast<std::size_t>(col) < COLS);
		return m_data[row * COLS + col];
	}
	inline const T& operator()(int row, int col) const
	{
		ASSERTDEB_(static_cast<std::size_t>(row) < ROWS);
		ASSERTDEB_(static_cast<std::size_t>(col) < COLS);
		return m_data[row * COLS + col];
	}

	/** Access the i-th element, Row-Major order, without out-of-bounds check
	 * (except in Debug builds)
	 */
	inline T& operator()(int i)
	{
		ASSERTDEB_(static_cast<std::size_t>(i) < ROWS * COLS);
		return m_data[i];
	}
	inline const T& operator()(int i) const
	{
		ASSERTDEB_(static_cast<std::size_t>(i) < ROWS * COLS);
		return m_data[i];
	}

	/** Access the [i-th] element (for 1xN or Nx1 matrices) */
	inline T& operator[](int i)
	{
		ASSERT_(ROWS == 1 || COLS == 1);
		ASSERTDEB_(static_cast<std::size_t>(i) < ROWS * COLS);
		return m_data[i];
	}
	inline const T& operator[](int i) const
	{
		ASSERT_(ROWS == 1 || COLS == 1);
		ASSERTDEB_(static_cast<std::size_t>(i) < ROWS * COLS);
		return m_data[i];
	}

	CMatrixFixed<float, ROWS, COLS> cast_float() const;
	CMatrixFixed<double, ROWS, COLS> cast_double() const;

	/** Solves the linear system Ax=b, returns x, with A this **symmetric**
	 * matrix. \sa lu_solve() */
	CMatrixFixed<T, ROWS, 1> llt_solve(const CMatrixFixed<T, ROWS, 1>& b) const;

	/** Solves the linear system Ax=b, returns x, with A this **asymmetric**
	 * matrix. \sa llt_solve() */
	CMatrixFixed<T, ROWS, 1> lu_solve(const CMatrixFixed<T, ROWS, 1>& b) const;

	/** this += A<sup>T</sup> */
	void sum_At(const CMatrixFixed<Scalar, ROWS, COLS>& A)
	{
		if constexpr (ROWS == COLS)
		{
			for (Index r = 0; r < static_cast<Index>(ROWS); r++)
				for (Index c = 0; c < static_cast<Index>(COLS); c++)
					(*this)(r, c) += A(c, r);
		}
		else
		{
			throw std::runtime_error("sum_At(): matrix must be square.");
		}
	}

	/** @} */
};

/** @name Typedefs for common sizes
	@{ */
using CMatrixDouble22 = CMatrixFixed<double, 2, 2>;
using CMatrixDouble23 = CMatrixFixed<double, 2, 3>;
using CMatrixDouble32 = CMatrixFixed<double, 3, 2>;
using CMatrixDouble33 = CMatrixFixed<double, 3, 3>;
using CMatrixDouble44 = CMatrixFixed<double, 4, 4>;
using CMatrixDouble66 = CMatrixFixed<double, 6, 6>;
using CMatrixDouble77 = CMatrixFixed<double, 7, 7>;
using CMatrixDouble13 = CMatrixFixed<double, 1, 3>;
using CMatrixDouble31 = CMatrixFixed<double, 3, 1>;
using CMatrixDouble12 = CMatrixFixed<double, 1, 2>;
using CMatrixDouble21 = CMatrixFixed<double, 2, 1>;
using CMatrixDouble61 = CMatrixFixed<double, 6, 1>;
using CMatrixDouble16 = CMatrixFixed<double, 1, 6>;
using CMatrixDouble71 = CMatrixFixed<double, 7, 1>;
using CMatrixDouble17 = CMatrixFixed<double, 1, 7>;
using CMatrixDouble51 = CMatrixFixed<double, 5, 1>;
using CMatrixDouble15 = CMatrixFixed<double, 1, 5>;
using CMatrixDouble41 = CMatrixFixed<double, 4, 1>;
using CMatrixDouble6_12 = CMatrixFixed<double, 6, 12>;
using CMatrixDouble12_6 = CMatrixFixed<double, 12, 6>;
using CMatrixDouble39 = CMatrixFixed<double, 3, 9>;
using CMatrixDouble93 = CMatrixFixed<double, 9, 3>;

using CMatrixFloat22 = CMatrixFixed<float, 2, 2>;
using CMatrixFloat23 = CMatrixFixed<float, 2, 3>;
using CMatrixFloat32 = CMatrixFixed<float, 3, 2>;
using CMatrixFloat33 = CMatrixFixed<float, 3, 3>;
using CMatrixFloat44 = CMatrixFixed<float, 4, 4>;
using CMatrixFloat66 = CMatrixFixed<float, 6, 6>;
using CMatrixFloat77 = CMatrixFixed<float, 7, 7>;
using CMatrixFloat13 = CMatrixFixed<float, 1, 3>;
using CMatrixFloat31 = CMatrixFixed<float, 3, 1>;
using CMatrixFloat12 = CMatrixFixed<float, 1, 2>;
using CMatrixFloat21 = CMatrixFixed<float, 2, 1>;
using CMatrixFloat61 = CMatrixFixed<float, 6, 1>;
using CMatrixFloat16 = CMatrixFixed<float, 1, 6>;
using CMatrixFloat71 = CMatrixFixed<float, 7, 1>;
using CMatrixFloat17 = CMatrixFixed<float, 1, 7>;
using CMatrixFloat51 = CMatrixFixed<float, 5, 1>;
using CMatrixFloat15 = CMatrixFixed<float, 1, 5>;
/**  @} */

}  // namespace mrpt::math

namespace mrpt::typemeta
{
template <typename T, std::size_t N, std::size_t M>
struct TTypeName<mrpt::math::CMatrixFixed<T, N, M>>
{
	constexpr static auto get()
	{
		return literal("CMatrixFixed<") + TTypeName<T>::get() + literal(",") +
			   literal(num_to_string<N>::value) + literal(",") +
			   literal(num_to_string<M>::value) + literal(">");
	}
};
}  // namespace mrpt::typemeta
