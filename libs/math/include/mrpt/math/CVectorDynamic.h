/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/containers/vector_with_small_size_optimization.h>
#include <mrpt/core/exceptions.h>  // ASSERT_()
#include <mrpt/math/MatrixVectorBase.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/serialization/serialization_frwds.h>
#include <mrpt/typemeta/TTypeName.h>
#include <cstring>  // memset()
#include <type_traits>

namespace mrpt::math
{
/** Template for column vectors of dynamic size, compatible with Eigen.
 *
 * \note For a complete introduction to Matrices and vectors in MRPT, see:
 *http://www.mrpt.org/Matrices_vectors_arrays_and_Linear_Algebra_MRPT_and_Eigen_classes
 * \sa CVectorDynamic, CMatrixFixed, CVectorFixed
 * \ingroup mrpt_math_grp
 */
template <class T>
class CVectorDynamic : public MatrixVectorBase<T, CVectorDynamic<T>>
{
   protected:
	static constexpr size_t small_size = 16;
	using vec_t = mrpt::containers::vector_with_small_size_optimization<
		T, small_size, MRPT_MAX_STATIC_ALIGN_BYTES>;

	vec_t m_data;

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
	constexpr static int ColsAtCompileTime = 1;
	constexpr static int is_mrpt_type = 1;
	using eigen_t = Eigen::Matrix<T, -1, 1, 0, -1, 1>;
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

	/** Internal use only: It reallocs the memory for the 2D matrix, maintaining
	 * the previous contents if posible.
	 */
	void realloc(const size_t new_len, bool newElementsToZero = false)
	{
		const auto old_len = m_data.size();
		if (new_len == old_len) return;
		m_data.resize(new_len);
		if (newElementsToZero && new_len > old_len)
		{
			if constexpr (std::is_trivial_v<T>)
				::memset(&m_data[old_len], 0, sizeof(T) * (new_len - old_len));
			else
				for (size_t k = old_len; k < new_len; k++) m_data[k] = T();
		}
	}

   public:
	void swap(CVectorDynamic<T>& o) { m_data.swap(o.m_data); }

	CVectorDynamic() = default;

	/** Initializes to a vector of "N" zeros */
	CVectorDynamic(size_t N, bool initZero = true) { realloc(N, initZero); }

	/** Copy (casting from if needed) from another matrix  */
	template <typename U>
	explicit CVectorDynamic(const CVectorDynamic<U>& m)
	{
		(*this) = m;
	}

	/** Ctor from a fixed-size vector */
	template <std::size_t ROWS>
	explicit CVectorDynamic(const CMatrixFixed<T, ROWS, 1>& v)
	{
		(*this) = v;
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
	template <
		typename ARRAY, typename = std::enable_if_t<std::is_array_v<ARRAY>>>
	CVectorDynamic(const ARRAY& data)
	{
		std::size_t N = std::size(data);
		ASSERTMSG_(N != 0, "CVectorDynamic ctor: Empty array!");
		realloc(N);
		for (size_t i = 0; i < N; i++) m_data[i] = static_cast<T>(data[i]);
	}

	/** Convert from Eigen matrix */
	template <class Derived>
	explicit CVectorDynamic(const Eigen::MatrixBase<Derived>& m)
	{
		*this = m;
	}

	/** Number of rows in the vector */
	size_type rows() const { return m_data.size(); }

	/** Number of columns in the matrix (always 1) */
	size_type cols() const { return 1; }

	/** Get a 2-vector with [NROWS NCOLS] (as in MATLAB command size(x)) */
	size_type size() const { return m_data.size(); }

	bool empty() const { return m_data.empty(); }

	/** Changes the size of matrix, maintaining the previous contents. */
	void setSize(size_t row, size_t col, bool zeroNewElements = false)
	{
		ASSERT_(col == 1);
		realloc(row, zeroNewElements);
	}
	void resize(std::size_t N, bool zeroNewElements = false)
	{
		setSize(N, 1, zeroNewElements);
	}

	template <class MAT>
	void fromVectorLike(const MAT& m)
	{
		MRPT_START
		ASSERT_EQUAL_(m.cols(), 1U);
		setSize(m.rows(), 1);
		for (Index r = 0; r < rows(); r++) (*this)[r] = m(r, 0);
		MRPT_END
	}

	/** Assignment operator from another matrix (possibly of a different type)
	 */
	template <typename U>
	CVectorDynamic& operator=(const CMatrixDynamic<U>& m)
	{
		MRPT_START
		fromVectorLike(m);
		return *this;
		MRPT_END
	}

	/** Assignment from an Eigen matrix */
	template <class Derived>
	CVectorDynamic& operator=(const Eigen::MatrixBase<Derived>& m)
	{
		MRPT_START
		fromVectorLike(m);
		return *this;
		MRPT_END
	}

	/** Assignment from a fixed-size vector */
	template <std::size_t ROWS>
	CVectorDynamic& operator=(const CMatrixFixed<T, ROWS, 1>& v)
	{
		MRPT_START
		fromVectorLike(v);
		return *this;
		MRPT_END
	}

	void push_back(const T& val)
	{
		m_data.resize(m_data.size() + 1);
		m_data.back() = val;
	}

	/** const segmentCopy(): Returns a *copy* of the given vector segment */
	template <int LEN>
	CMatrixFixed<Scalar, LEN, 1> segmentCopy(int start = 0) const
	{
		CMatrixFixed<Scalar, LEN, 1> v;
		for (int i = 0; i < LEN; i++) v[i] = m_data[start + i];
		return v;
	}

	/** const segmentCopy(): Returns a *copy* of the given vector segment (non
	 * templated version, dynamic length) */
	CVectorDynamic<Scalar> segmentCopy(int start, int LEN) const
	{
		CVectorDynamic<Scalar> v;
		v.resize(LEN);
		for (int i = 0; i < LEN; i++) v[i] = m_data[start + i];
		return v;
	}

	/** Subscript operator to get/set individual elements
	 */
	inline T& operator()(size_t row, size_t col)
	{
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
		if (row >= m_data.size() || col > 0)
			THROW_EXCEPTION(format(
				"Indexes (%lu,%lu) out of range. Vector is %lux%lu",
				static_cast<unsigned long>(row),
				static_cast<unsigned long>(col),
				static_cast<unsigned long>(m_data.size()),
				static_cast<unsigned long>(1)));
#endif
		return m_data[row];
	}

	/** Subscript operator to get individual elements
	 */
	inline const T& operator()(size_t row, size_t col) const
	{
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
		if (row >= m_data.size() || col > 0)
			THROW_EXCEPTION(format(
				"Indexes (%lu,%lu) out of range. Vector is %lux%lu",
				static_cast<unsigned long>(row),
				static_cast<unsigned long>(col),
				static_cast<unsigned long>(m_data.size()),
				static_cast<unsigned long>(1)));
#endif
		return m_data[row];
	}

	/** Subscript operator to get/set an individual element from a row or column
	 * matrix.
	 * \exception std::exception If the object is not a column or row matrix.
	 */
	inline T& operator[](size_t ith)
	{
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
		if (ith >= m_data.size())
			THROW_EXCEPTION_FMT(
				"Index %u out of range!", static_cast<unsigned>(ith));
#endif
		return m_data[ith];
	}

	/// \overload
	inline const T& operator[](size_t ith) const
	{
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
		if (ith >= m_data.size())
			THROW_EXCEPTION_FMT(
				"Index %u out of range!", static_cast<unsigned>(ith));
#endif
		return m_data[ith];
	}

	/** Get as an Eigen-compatible Eigen::Map object  */
	template <
		typename EIGEN_VECTOR = eigen_t,
		typename EIGEN_MAP = Eigen::Map<
			EIGEN_VECTOR, MRPT_MAX_ALIGN_BYTES, Eigen::InnerStride<1>>>
	EIGEN_MAP asEigen()
	{
		return EIGEN_MAP(&m_data[0], m_data.size());
	}
	/** \overload (const version) */
	template <
		typename EIGEN_VECTOR = Eigen::Matrix<T, -1, 1, 0, -1, 1>,
		typename EIGEN_MAP = Eigen::Map<
			const EIGEN_VECTOR, MRPT_MAX_ALIGN_BYTES, Eigen::InnerStride<1>>>
	EIGEN_MAP asEigen() const
	{
		return EIGEN_MAP(&m_data[0], m_data.size());
	}

	template <typename T2>
	CVectorDynamic<T2> cast() const;
};

using CVectorFloat = CVectorDynamic<float>;
using CVectorDouble = CVectorDynamic<double>;

mrpt::serialization::CArchive& operator<<(
	mrpt::serialization::CArchive& s, const CVectorFloat& a);
mrpt::serialization::CArchive& operator<<(
	mrpt::serialization::CArchive& s, const CVectorDouble& a);
mrpt::serialization::CArchive& operator>>(
	mrpt::serialization::CArchive& in, CVectorDouble& a);
mrpt::serialization::CArchive& operator>>(
	mrpt::serialization::CArchive& in, CVectorFloat& a);

}  // namespace mrpt::math

namespace mrpt::typemeta
{
// Extensions to mrpt::typemeta::TTypeName for matrices:
template <typename T>
struct TTypeName<mrpt::math::CVectorDynamic<T>>
{
	static auto get()
	{
		return literal("CVectorDynamic<") + TTypeName<T>::get() + literal(">");
	}
};
}  // namespace mrpt::typemeta
