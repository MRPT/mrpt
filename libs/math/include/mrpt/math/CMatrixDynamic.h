/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/containers/vector_with_small_size_optimization.h>
#include <mrpt/core/exceptions.h>  // ASSERT_()
#include <mrpt/core/format.h>
#include <mrpt/math/MatrixBase.h>
#include <mrpt/math/math_frwds.h>  // forward declarations
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/typemeta/TTypeName.h>
#include <algorithm>  // swap()
#include <cstring>  // memset()
#include <type_traits>

namespace mrpt::math
{
/**  This template class provides the basic functionality for a general 2D
 *any-size, resizable container of numerical or non-numerical elements.
 * NOTES:
 * - This class is not serializable since it is a template. For using
 *serialization, see mrpt::math::CMatrixD.
 * - First row or column index is "0".
 * - This class includes range checks with ASSERT_() if compiling with "_DEBUG"
 *or "MRPT_ALWAYS_CHECKS_DEBUG_MATRICES=1".
 * - Use asEigen() to get an `Eigen::Map<>` object and to access full Algebra
 *functionality.
 *
 * \sa CMatrixFixed
 * \ingroup mrpt_math_grp
 */
template <class T>
class CMatrixDynamic : public MatrixBase<T, CMatrixDynamic<T>>
{
   private:
	static constexpr size_t small_size = 16;
	using vec_t = mrpt::containers::vector_with_small_size_optimization<
		T, small_size, MRPT_MAX_STATIC_ALIGN_BYTES>;

	/** RowMajor matrix data */
	vec_t m_data;
	size_t m_Rows{0}, m_Cols{0};

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
	constexpr static int RowsAtCompileTime = -1;
	constexpr static int ColsAtCompileTime = -1;
	constexpr static int SizeAtCompileTime = -1;
	constexpr static int is_mrpt_type = 1;
	constexpr static int StorageOrder = 1 /*rowMajor*/;
	using eigen_t = Eigen::Matrix<
		T, RowsAtCompileTime, ColsAtCompileTime, StorageOrder,
		RowsAtCompileTime, ColsAtCompileTime>;
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

   private:
	/** Internal use only: It reallocs the memory for the 2D matrix, maintaining
	 * the previous contents if posible.
	 */
	void realloc(size_t row, size_t col, bool newElementsToZero = false)
	{
		if (row == m_Rows && col == m_Cols) return;
		const auto old_rows = m_Rows, old_cols = m_Cols;
		m_Rows = row;
		m_Cols = col;

		// New buffer:
		decltype(m_data) newData(m_Rows * m_Cols);
		newData.fill(typename decltype(m_data)::value_type());
		// Copy old content:
		const auto nRowsToCopy = m_Rows >= old_rows ? old_rows : m_Rows;
		const auto nColsToCopy = m_Cols >= old_cols ? old_cols : m_Cols;
		for (size_t r = 0; r < nRowsToCopy; r++)
		{
			if constexpr (std::is_trivial_v<T>)
				std::memcpy(
					&newData[r * m_Cols], &m_data[r * old_cols],
					sizeof(T) * nColsToCopy);
			else
				for (size_t c = 0; c < nColsToCopy; c++)
					newData[r * m_Cols + c] = m_data[r * old_cols + c];
		}
		// New rows to zero?
		if (newElementsToZero && m_Rows > old_rows)
		{
			if constexpr (std::is_trivial_v<T>)
				::memset(
					&newData[old_rows * m_Cols], 0,
					sizeof(T) * (m_Rows - old_rows));
			else
				for (size_t r = old_rows; r < m_Rows; r++)
					for (size_t c = 0; c < m_Cols; c++)
						newData[r * m_Cols + c] = T();
		}
		// New cols to zero?
		if (newElementsToZero && m_Cols > old_cols)
		{
			for (size_t r = 0; r < old_rows; r++)
				if constexpr (std::is_trivial_v<T>)
					::memset(
						&newData[r * m_Cols + old_cols], 0,
						sizeof(T) * (m_Cols - old_cols));
				else
					for (size_t c = old_cols; c < m_Cols; c++)
						newData[r * m_Cols + c] = T();
		}
		// Swap:
		m_data.swap(newData);
	}

   public:
	/** Swap with another matrix very efficiently (just swaps a pointer and two
	 * integer values). */
	inline void swap(CMatrixDynamic<T>& o)
	{
		m_data.swap(o.m_data);
		std::swap(m_Rows, o.m_Rows);
		std::swap(m_Cols, o.m_Cols);
	}

	/** Constructors */
	CMatrixDynamic(const CMatrixDynamic& m) { (*this) = m; }

	CMatrixDynamic(size_t row = 0, size_t col = 0) { realloc(row, col); }

	/** Copy (casting from if needed) from another matrix  */
	template <typename U>
	explicit CMatrixDynamic(const CMatrixDynamic<U>& m)
	{
		(*this) = m;
	}

	/** Convert from Eigen matrix */
	template <class Derived>
	explicit CMatrixDynamic(const Eigen::MatrixBase<Derived>& m)
	{
		*this = m;
	}

	/** Convert from Eigen product */
	template <typename _Lhs, typename _Rhs, int Option>
	explicit CMatrixDynamic(const Eigen::Product<_Lhs, _Rhs, Option>& p)
	{
		*this = p.eval();
	}
	/** Convert from Eigen binary op */
	template <typename Op, typename Lhs, typename Rhs>
	explicit CMatrixDynamic(const Eigen::CwiseBinaryOp<Op, Lhs, Rhs>& p)
	{
		*this = p.eval();
	}

	/** Copy constructor & crop from another matrix
	 */
	CMatrixDynamic(
		const CMatrixDynamic& m, const size_t cropRowCount,
		const size_t cropColCount)
	{
		ASSERT_(m.m_Rows >= cropRowCount);
		ASSERT_(m.m_Cols >= cropColCount);
		realloc(cropRowCount, cropColCount);
		for (size_t i = 0; i < m_Rows; i++)
			for (size_t j = 0; j < m_Cols; j++) (*this)(i, j) = m(i, j);
	}

	/** Constructor from fixed-size matrix: */
	template <std::size_t ROWS, std::size_t COLS>
	explicit CMatrixDynamic(const CMatrixFixed<T, ROWS, COLS>& o)
	{
		*this = o;
	}

	/** Constructor from a given size and a C array. The array length must match
	 *cols x row.
	 * \code
	 *  const double numbers[] = {
	 *    1,2,3,
	 *    4,5,6 };
	 *  CMatrixDouble   M(3,2, numbers);
	 * \endcode
	 */
	template <typename V, size_t N>
	CMatrixDynamic(size_t row, size_t col, V (&theArray)[N])
	{
		static_assert(N != 0, "Empty array!");
		realloc(row, col);
		if (m_Rows * m_Cols != N)
			THROW_EXCEPTION(format(
				"Mismatch between matrix size %lu x %lu and array of "
				"length %lu",
				static_cast<long unsigned>(m_Rows),
				static_cast<long unsigned>(m_Cols),
				static_cast<long unsigned>(N)));
		size_t idx = 0;
		for (size_t i = 0; i < m_Rows; i++)
			for (size_t j = 0; j < m_Cols; j++)
				(*this)(i, j) = static_cast<T>(theArray[idx++]);
	}

	/** Constructor from a given size and a STL container (std::vector,
	 * std::list,...) with the initial values. The vector length must match cols
	 * x row.
	 */
	template <typename V>
	CMatrixDynamic(size_t row, size_t col, const V& theVector)
	{
		const size_t N = theVector.size();
		realloc(row, col);
		if (m_Rows * m_Cols != N)
			THROW_EXCEPTION(format(
				"Mismatch between matrix size %lu x %lu and array of "
				"length %lu",
				static_cast<long unsigned>(m_Rows),
				static_cast<long unsigned>(m_Cols),
				static_cast<long unsigned>(N)));
		typename V::const_iterator it = theVector.begin();
		for (size_t i = 0; i < m_Rows; i++)
			for (size_t j = 0; j < m_Cols; j++)
				(*this)(i, j) = static_cast<T>(*(it++));
	}

	virtual ~CMatrixDynamic() = default;

	template <class MAT>
	void setFromMatrixLike(const MAT& m)
	{
		MRPT_START
		setSize(m.rows(), m.cols());
		for (Index r = 0; r < rows(); r++)
			for (Index c = 0; c < cols(); c++)
				(*this)(r, c) = static_cast<T>(m(r, c));
		MRPT_END
	}

	CMatrixDynamic& operator=(const CMatrixDynamic<T>& m) = default;

	/** Assignment operator from another matrix (possibly of a different type)
	 */
	template <typename U>
	CMatrixDynamic& operator=(const CMatrixDynamic<U>& m)
	{
		MRPT_START
		setFromMatrixLike(m);
		return *this;
		MRPT_END
	}

	/** Assignment from an Eigen matrix */
	template <class Derived>
	CMatrixDynamic& operator=(const Eigen::MatrixBase<Derived>& m)
	{
		MRPT_START
		setFromMatrixLike(m);
		return *this;
		MRPT_END
	}
	/** Assignment from a fixed matrix */
	template <std::size_t ROWS, std::size_t COLS>
	CMatrixDynamic& operator=(const CMatrixFixed<T, ROWS, COLS>& m)
	{
		MRPT_START
		setFromMatrixLike(m);
		return *this;
		MRPT_END
	}

	/** Assignment operator for initializing from a C array (The matrix must be
	 *set to the correct size before invoking this asignament)
	 * \code
	 *	 CMatrixDouble   M(3,2);
	 *  const double numbers[] = {
	 *    1,2,3,
	 *    4,5,6 };
	 *  M = numbers;
	 * \endcode
	 *  Refer also to the constructor with initialization data
	 *CMatrixDynamic::CMatrixDynamic
	 */
	template <typename V, size_t N>
	CMatrixDynamic& operator=(V (&theArray)[N])
	{
		static_assert(N != 0, "Empty array!");
		if (m_Rows * m_Cols != N)
		{
			THROW_EXCEPTION(format(
				"Mismatch between matrix size %lu x %lu and array of "
				"length %lu",
				m_Rows, m_Cols, N));
		}
		size_t idx = 0;
		for (size_t i = 0; i < m_Rows; i++)
			for (size_t j = 0; j < m_Cols; j++)
				(*this)(i, j) = static_cast<T>(theArray[idx++]);
		return *this;
	}

	/** Move ctor */
	CMatrixDynamic(CMatrixDynamic&& m) { (*this) = std::move(m); }

	/** Move operator */
	CMatrixDynamic& operator=(CMatrixDynamic&& m)
	{
		m_data = std::move(m.m_data);
		m_Cols = m.m_Cols;
		m_Rows = m.m_Rows;
		return *this;
	}

	/** Number of rows in the matrix \sa rows() */
	inline size_type rows() const { return m_Rows; }

	/** Number of columns in the matrix \sa rows() */
	inline size_type cols() const { return m_Cols; }

	/** Get a 2-vector with [NROWS NCOLS] (as in MATLAB command size(x)) */
	inline matrix_size_t size() const
	{
		matrix_size_t dims;
		dims[0] = m_Rows;
		dims[1] = m_Cols;
		return dims;
	}

	/** Changes the size of matrix, maintaining the previous contents. */
	void setSize(size_t row, size_t col, bool zeroNewElements = false)
	{
		realloc(row, col, zeroNewElements);
	}
	void resize(size_t row, size_t col) { setSize(row, col); }

	/** Resizes as a Nx1 vector */
	void resize(size_t vectorLen) { setSize(vectorLen, 1); }

	/** Resize the matrix */
	inline void resize(const matrix_size_t& siz, bool zeroNewElements = false)
	{
		setSize(siz[0], siz[1], zeroNewElements);
	}

	// These ones are to make template code compatible with Eigen & mrpt:
	CMatrixDynamic& derived() { return *this; }
	const CMatrixDynamic& derived() const { return *this; }
	void conservativeResize(size_t row, size_t col) { setSize(row, col); }

	/** Return raw pointer to row-major data buffer. All matrix cells can be
	 * assumed to be stored contiguously in memory, i.e. row stride = column
	 * count. */
	const T* data() const
	{
		ASSERT_(!m_data.empty());
		return &m_data[0];
	}
	/// \overload
	T* data()
	{
		ASSERT_(!m_data.empty());
		return &m_data[0];
	}

	/** Subscript operator to get/set individual elements
	 */
	inline T& operator()(size_t row, size_t col)
	{
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
		if (row >= m_Rows || col >= m_Cols)
			THROW_EXCEPTION(format(
				"Indexes (%lu,%lu) out of range. Matrix is %lux%lu",
				static_cast<unsigned long>(row),
				static_cast<unsigned long>(col),
				static_cast<unsigned long>(m_Rows),
				static_cast<unsigned long>(m_Cols)));
#endif
		return m_data[row * m_Cols + col];
	}

	/** Subscript operator to get individual elements
	 */
	inline const T& operator()(size_t row, size_t col) const
	{
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
		if (row >= m_Rows || col >= m_Cols)
			THROW_EXCEPTION(format(
				"Indexes (%lu,%lu) out of range. Matrix is %lux%lu",
				static_cast<unsigned long>(row),
				static_cast<unsigned long>(col),
				static_cast<unsigned long>(m_Rows),
				static_cast<unsigned long>(m_Cols)));
#endif
		return m_data[row * m_Cols + col];
	}

	/** Subscript operator to get/set an individual element from a row or column
	 * matrix.
	 * \exception std::exception If the object is not a column or row matrix.
	 */
	inline T& operator[](size_t ith)
	{
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
		ASSERT_(m_Rows == 1 || m_Cols == 1);
#endif
		if (m_Rows == 1)
		{
// A row matrix:
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
			if (ith >= m_Cols)
				THROW_EXCEPTION_FMT(
					"Index %u out of range!", static_cast<unsigned>(ith));
#endif
		}
		else
		{
// A columns matrix:
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
			if (ith >= m_Rows)
				THROW_EXCEPTION_FMT(
					"Index %u out of range!", static_cast<unsigned>(ith));
#endif
		}
		return m_data[ith];
	}

	/** Subscript operator to get/set an individual element from a row or column
	 * matrix. For non-vectors (NxM matrices), it returns the i-th matrix
	 * element, in RowMajor order.
	 * \exception std::exception If the object is not a column or row matrix.
	 */
	inline const T& operator[](size_t ith) const
	{
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
		ASSERT_BELOW_(ith, m_Rows * m_Cols);
#endif
		return m_data[ith];
	}

	/** Appends a new row to the MxN matrix from a 1xN vector.
	 *  The lenght of the vector must match the width of the matrix, unless
	 * it's empty: in that case the matrix is resized to 1xN.
	 *  \code
	 *    CMatrixDouble  M(0,0);
	 *    CVectorDouble  v(7),w(7);
	 *    // ...
	 *    M.appendRow(v);
	 *    M.appendRow(w);
	 *  \endcode
	 * \exception std::exception On incorrect vector length.
	 * \sa extractRow
	 * \sa appendCol
	 */
	template <typename VECTOR>
	void appendRow(const VECTOR& in)
	{
		if (m_Cols == 0 || m_Rows == 0)
			ASSERT_(!in.empty());
		else
			ASSERT_(in.size() == m_Cols);
		const auto row = m_Rows;
		realloc(row + 1, m_Cols = in.size());
		for (size_t i = 0; i < m_Cols; i++) (*this)(row, i) = in[i];
	}

	template <typename VECTOR>
	void setRow(const Index row, const VECTOR& v)
	{
		ASSERT_EQUAL_(cols(), static_cast<size_type>(v.size()));
		for (Index c = 0; c < cols(); c++) (*this)(row, c) = v[c];
	}

	template <typename VECTOR>
	void setCol(const Index col, const VECTOR& v)
	{
		ASSERT_EQUAL_(rows(), static_cast<size_type>(v.size()));
		for (Index r = 0; r < rows(); r++) (*this)(r, col) = v[r];
	}

	/** Appends a new column to the matrix from a vector.
	 * The length of the vector must match the number of rows of the matrix,
	 * unless it is (0,0).
	 * \exception std::exception On size mismatch.
	 * \sa extractCol
	 * \sa appendRow
	 */
	template <typename VECTOR>
	void appendCol(const VECTOR& in)
	{
		size_t r = m_Rows, c = m_Cols;
		if (m_Cols == 0 || m_Rows == 0)
		{
			ASSERT_(!in.empty());
			r = in.size();
			c = 0;
		}
		else
			ASSERT_EQUAL_(static_cast<decltype(m_Rows)>(in.size()), m_Rows);
		realloc(r, c + 1);
		for (size_t i = 0; i < m_Rows; i++) (*this)(i, m_Cols - 1) = in[i];
	}

	/** Returns a vector containing the matrix's values.
	 */
	template <typename VECTOR>
	void asVector(VECTOR& out) const
	{
		out.clear();
		out.reserve(m_Rows * m_Cols);
		for (const auto& d : m_data) out.push_back(d);
	}

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
		return EIGEN_MAP(&m_data[0], m_Rows, m_Cols);
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
		return EIGEN_MAP(&m_data[0], m_Rows, m_Cols);
	}

	CMatrixDynamic<float> cast_float() const;
	CMatrixDynamic<double> cast_double() const;

	/** Solves the linear system Ax=b, returns x, with A this **symmetric**
	 * matrix. \sa lu_solve() */
	CVectorDynamic<Scalar> llt_solve(const CVectorDynamic<Scalar>& b) const;

	/** Solves the linear system Ax=b, returns x, with A this **asymmetric**
	 * matrix. \sa llt_solve() */
	CVectorDynamic<Scalar> lu_solve(const CVectorDynamic<Scalar>& b) const;

};  // end of class CMatrixDynamic

/** Declares a matrix of booleans (non serializable).
 *  \sa CMatrixDouble, CMatrixFloat, CMatrixB */
using CMatrixBool = CMatrixDynamic<bool>;

/** Declares a matrix of float numbers (non serializable).
 *  For a serializable version, use math::CMatrixF
 *  \sa CMatrixDouble, CMatrixF, CMatrixD
 */
using CMatrixFloat = CMatrixDynamic<float>;

/** Declares a matrix of double numbers (non serializable).
 *  For a serializable version, use math::CMatrixD
 *  \sa CMatrixFloat, CMatrixF, CMatrixD
 */
using CMatrixDouble = CMatrixDynamic<double>;

/** Declares a matrix of unsigned ints (non serializable).
 *  \sa CMatrixDouble, CMatrixFloat
 */
using CMatrixUInt = CMatrixDynamic<unsigned int>;

/** matrix of uint8_t (non serializable). \sa CMatrixDouble */
using CMatrix_u8 = CMatrixDynamic<uint8_t>;

/** matrix of uint16_t (non serializable). \sa CMatrixDouble */
using CMatrix_u16 = CMatrixDynamic<uint16_t>;

#ifdef HAVE_LONG_DOUBLE
/** Declares a matrix of "long doubles" (non serializable), or of "doubles" if
 * the compiler does not support "long double".
 *  \sa CMatrixDouble, CMatrixFloat
 */
using CMatrixLongDouble = CMatrixDynamic<long double>;
#else
/** Declares a matrix of "long doubles" (non serializable), or of "doubles" if
 * the compiler does not support "long double".
 *  \sa CMatrixDouble, CMatrixFloat
 */
using CMatrixLongDouble = CMatrixDynamic<double>;
#endif
}  // namespace mrpt::math

namespace mrpt::typemeta
{
// Extensions to mrpt::typemeta::TTypeName for matrices:
template <typename T>
struct TTypeName<mrpt::math::CMatrixDynamic<T>>
{
	static auto get()
	{
		return literal("CMatrixDynamic<") + TTypeName<T>::get() + literal(">");
	}
};
}  // namespace mrpt::typemeta
