/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/config.h>
#include <string>
#include <type_traits>

/*! \file math_frwds.h
 * Forward declarations of all mrpt::math classes related to vectors, arrays
 * and matrices.
 * Many of the function implementations are in ops_matrices.h, others in
 * ops_containers.h
 */

namespace mrpt
{
namespace system
{
std::string MRPT_getVersion();
}
namespace math
{
struct TPoseOrPoint;
class CMatrix;  // mrpt-binary-serializable matrix
class CMatrixD;  // mrpt-binary-serializable matrix

namespace detail
{
/** Internal resize which compiles to nothing on fixed-size matrices. */
template <typename MAT, int TypeSizeAtCompileTime>
struct TAuxResizer
{
	static inline void internal_resize(MAT&, size_t, size_t) {}
	static inline void internal_resize(MAT&, size_t) {}
};
template <typename MAT>
struct TAuxResizer<MAT, -1>
{
	static inline void internal_resize(MAT& obj, size_t row, size_t col)
	{
		obj.derived().conservativeResize(row, col);
	}
	static inline void internal_resize(MAT& obj, size_t nsize)
	{
		obj.derived().conservativeResize(nsize);
	}
};
}  // namespace detail

/*! Selection of the number format in CMatrixTemplate::saveToTextFile
 */
enum TMatrixTextFileFormat
{
	/** engineering format '%e' */
	MATRIX_FORMAT_ENG = 0,
	/** fixed floating point '%f' */
	MATRIX_FORMAT_FIXED = 1,
	/** intergers '%i' */
	MATRIX_FORMAT_INT = 2
};

/** For usage in one of the constructors of CMatrixFixedNumeric or
   CMatrixTemplate (and derived classes), if it's not required
	 to fill it with zeros at the constructor to save time. */
enum TConstructorFlags_Matrices
{
	UNINITIALIZED_MATRIX = 0
};

// ---------------- Forward declarations: Classes ----------------
template <class T>
class CMatrixTemplate;
template <class T>
class CMatrixTemplateObjects;
template <class T>
class CQuaternion;

/** ContainerType<T>::element_t exposes the value of any STL or Eigen container.
 *  Default specialization works for STL and MRPT containers, there is another
 * one for Eigen in <mrpt/math/eigen_frwds.h> */
template <typename CONTAINER>
struct ContainerType
{
	using element_t = typename CONTAINER::value_type;
};

#define MRPT_MATRIX_CONSTRUCTORS_FROM_POSES(_CLASS_)                          \
	template <                                                                \
		class TPOSE, typename = std::enable_if_t<                             \
						 std::is_base_of_v<mrpt::math::TPoseOrPoint, TPOSE>>> \
	explicit inline _CLASS_(const TPOSE& p)                                   \
	{                                                                         \
		mrpt::math::containerFromPoseOrPoint(*this, p);                       \
	}                                                                         \
	template <class CPOSE, int = CPOSE::is_3D_val>                            \
	explicit inline _CLASS_(const CPOSE& p)                                   \
	{                                                                         \
		mrpt::math::containerFromPoseOrPoint(*this, p);                       \
	}

template <class CONTAINER1, class CONTAINER2>
void cumsum(const CONTAINER1& in_data, CONTAINER2& out_cumsum);

template <class CONTAINER>
inline typename CONTAINER::Scalar norm(const CONTAINER& v);
template <class CONTAINER>
inline typename CONTAINER::Scalar norm_inf(const CONTAINER& v);

template <class MAT_A, class SKEW_3VECTOR, class MAT_OUT>
void multiply_A_skew3(const MAT_A& A, const SKEW_3VECTOR& v, MAT_OUT& out);
template <class SKEW_3VECTOR, class MAT_A, class MAT_OUT>
void multiply_skew3_A(const SKEW_3VECTOR& v, const MAT_A& A, MAT_OUT& out);

namespace detail
{
template <class MATORG, class MATDEST>
void extractMatrix(
	const MATORG& M, const size_t first_row, const size_t first_col,
	MATDEST& outMat);
}

/** Conversion of poses (TPose2D,TPoint2D,...,
 * mrpt::poses::CPoint2D,CPose3D,...) to MRPT containers (vector/matrix) */
template <class CONTAINER, class POINT_OR_POSE>
CONTAINER& containerFromPoseOrPoint(CONTAINER& C, const POINT_OR_POSE& p);

// Vicinity classes ----------------------------------------------------
namespace detail
{
/**
 * The purpose of this class is to model traits for containers, so that they
 * can be used as return values for the function CMatrixTemplate::getVicinity.
 * This class is NOT defined for any base container, because correctness would
 * not be guaranteed. Instead, each class must define its own specialization
 * of the template, containing two functions:
 * - static void initialize(container<T>,size_t N): must reserve space to allow
 * at least the insertion of N*N elements, in a square fashion when appliable.
 * - static void insertInContainer(container<T>,size_t r,size_t c,const T &):
 * must insert the given element in the container. Whenever it's possible, it
 * must insert it in the (r,c) coordinates.
 * For linear containers, the vicinity functions are guaranteed to insert
 * elements in order, i.e., starting from the top and reading from left to
 * right.
 */
template <typename T>
class VicinityTraits;

/**
 * This huge template encapsulates a function to get the vicinity of an
 * element, with maximum genericity. Although it's not meant to be called
 * directly,
 * every type defining the get_unsafe method
 * will work. The assert checks if the boundaries (r-N,r+N,c-N,c+N) fit in
 * the matrix.
 * The template parameters are the following:
 * - MatrixType: the matrix or container base type, from which the vicinity is
 * required.
 * - T: the base type of the matrix or container.
 * - ReturnType: the returning container type. The class
 * VicinityTraits<ReturnType> must be completely defined.
 * - D: the dimension of the vicinity. Current implementations are 4, 5, 8, 9,
 * 12, 13, 20, 21, 24 and 25, although it's easy to implement new variants.
 */
template <typename MatrixType, typename T, typename ReturnType, size_t D>
struct getVicinity;
}  // namespace detail

// Other forward decls:
template <class T>
T wrapTo2Pi(T a);

}  // namespace math
}  // namespace mrpt
