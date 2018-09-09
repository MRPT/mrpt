/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/types_math.h>
#include <Eigen/Dense>
#include <mrpt/typemeta/TTypeName.h>
#include <mrpt/math/point_poses2vectors.h>  // MRPT_MATRIX_CONSTRUCTORS_FROM_POSES()
#include <mrpt/core/exceptions.h>

namespace mrpt
{
namespace math
{
/**  A matrix of dynamic size.
 *   Basically, this class is a wrapper on Eigen::Matrix<T,Dynamic,Dynamic>,
 * but
 *   with a RowMajor element memory layout (except for column vectors).
 *
 * \note This class exists for backward compatibility of ancient times when
 * MRPT didn't rely on Eigen, feel free to directly use Eigen::Matrix<> types
 * instead.
 *
 * \sa CMatrixTemplate (a non Eigen lib-based  class, which can hold arbitrary
 * objects, not only numerical types).
 * \note For a complete introduction to Matrices and vectors in MRPT, see:
 * http://www.mrpt.org/Matrices_vectors_arrays_and_Linear_Algebra_MRPT_and_Eigen_classes
 * \ingroup mrpt_math_grp
 */
template <class T>
class CMatrixTemplateNumeric : public Eigen::Matrix<
								   T, Eigen::Dynamic, Eigen::Dynamic,
								   // Use row major storage for backward
								   // compatibility with MRPT matrices in all
								   // cases (even in column vectors!)
								   Eigen::AutoAlign | Eigen::RowMajor>
{
   public:
	using Base = Eigen::Matrix<
		T, Eigen::Dynamic, Eigen::Dynamic, Eigen::AutoAlign | Eigen::RowMajor>;
	using mrpt_autotype = CMatrixTemplateNumeric<T>;

	/** Default constructor, builds a 1x1 matrix */
	inline CMatrixTemplateNumeric() : Base(1, 1) { Base::setZero(); }
	/** Constructor that builds a 0x0 matrix (that is, uninitialized), for usage
	 * in places where efficiency is a priority.
	 *  Use as:
	 *   \code
	 *     CMatrixTemplateNumeric<double>  M( UNINITIALIZED_MATRIX);
	 *   \endcode
	 */
	inline CMatrixTemplateNumeric(TConstructorFlags_Matrices) : Base(0, 0) {}
	/** Constructor, creates a matrix of the given size, filled with zeros. */
	inline CMatrixTemplateNumeric(size_t row, size_t col) : Base(row, col)
	{
		Base::setZero();
	}
	MRPT_MATRIX_CONSTRUCTORS_FROM_POSES(CMatrixTemplateNumeric)
	MRPT_EIGEN_DERIVED_CLASS_CTOR_OPERATOR_EQUAL(
		CMatrixTemplateNumeric)  // Implements ctor and "operator =" for any
	// other Eigen class

	/** Assignment operator of other types
	 */
	template <class R>
	inline CMatrixTemplateNumeric<T>& operator=(const CMatrixTemplate<R>& m)
	{
		Base::resize(m.rows(), m.cols());

		for (size_t i = 0; i < CMatrixTemplate<T>::rows(); i++)
			for (size_t j = 0; j < CMatrixTemplate<T>::cols(); j++)
				Base::coeffRef(i, j) = static_cast<T>(m.get_unsafe(i, j));
		return *this;
	}

	/** Assignment from any Eigen matrix/vector */
	template <typename Derived>
	inline CMatrixTemplateNumeric<T>& operator=(
		const Eigen::MatrixBase<Derived>& m) const
	{
		Base::operator=(m);
		return *this;
	}

	/** Constructor from a given size and a C array. The array length must match
	 *cols x row.
	 * \code
	 *  const double numbers[] = {
	 *    1,2,3,
	 *    4,5,6 };
	 *	 CMatrixDouble   M(3,2, numbers);
	 * \endcode
	 */
	template <typename V, size_t N>
	inline CMatrixTemplateNumeric(size_t row, size_t col, V (&theArray)[N])
		: Base(row, col)
	{
		ASSERT_EQUAL_(row * col, N);
		ASSERT_EQUAL_(sizeof(theArray[0]), sizeof(T));
		::memcpy(
			Base::data(), &theArray[0],
			sizeof(T) * N);  // Remember, row-major order!
	}

	/** Destructor
	 */
	inline ~CMatrixTemplateNumeric() = default;
	/** == comparison of two matrices; it differs from default Eigen operator in
	 * that returns false if matrices are of different sizes instead of raising
	 * an assert. */
	template <typename Derived>
	inline bool operator==(const Eigen::MatrixBase<Derived>& m2) const
	{
		return Base::cols() == m2.cols() && Base::rows() == m2.rows() &&
			   Base::cwiseEqual(m2).all();
	}
	/** != comparison of two matrices; it differs from default Eigen operator in
	 * that returns true if matrices are of different sizes instead of raising
	 * an assert. */
	template <typename Derived>
	inline bool operator!=(const Eigen::MatrixBase<Derived>& m2) const
	{
		return !((*this) == m2);
	}

};  // end of class definition

/** Declares a matrix of float numbers (non serializable).
 *  For a serializable version, use math::CMatrix
 *  \sa CMatrixDouble, CMatrix, CMatrixD
 */
using CMatrixFloat = CMatrixTemplateNumeric<float>;

/** Declares a matrix of double numbers (non serializable).
 *  For a serializable version, use math::CMatrixD
 *  \sa CMatrixFloat, CMatrix, CMatrixD
 */
using CMatrixDouble = CMatrixTemplateNumeric<double>;

/** Declares a matrix of unsigned ints (non serializable).
 *  \sa CMatrixDouble, CMatrixFloat
 */
using CMatrixUInt = CMatrixTemplateNumeric<unsigned int>;

#ifdef HAVE_LONG_DOUBLE
/** Declares a matrix of "long doubles" (non serializable), or of "doubles" if
 * the compiler does not support "long double".
 *  \sa CMatrixDouble, CMatrixFloat
 */
using CMatrixLongDouble = CMatrixTemplateNumeric<long double>;
#else
/** Declares a matrix of "long doubles" (non serializable), or of "doubles" if
 * the compiler does not support "long double".
 *  \sa CMatrixDouble, CMatrixFloat
 */
using CMatrixLongDouble = CMatrixTemplateNumeric<double>;
#endif

namespace detail
{
/**
 * Vicinity traits class specialization for fixed size matrices.
 */
template <typename T>
class VicinityTraits<CMatrixTemplateNumeric<T>>
{
   public:
	inline static void initialize(CMatrixTemplateNumeric<T>& mat, size_t N)
	{
		mat.setSize(N, N);
		mat.fill(0);
	}
	inline static void insertInContainer(
		CMatrixTemplateNumeric<T>& mat, size_t r, size_t c, const T& t)
	{
		mat.get_unsafe(r, c) = t;
	}
};
}  // namespace detail
}  // namespace math

namespace typemeta
{
// Extensions to mrpt::typemeta::TTypeName for matrices:
template <typename T>
struct TTypeName<mrpt::math::CMatrixTemplateNumeric<T>>
{
	static auto get()
	{
		return literal("CMatrixTemplateNumeric<") + TTypeName<T>::get() +
			   literal(">");
	}
};
}  // namespace typemeta

}  // namespace mrpt
