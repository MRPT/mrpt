/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <cstdarg>
#include <cstdio>
#include <cmath>
#include <iosfwd>
#include <vector>
#include <limits>  // numeric_limits
#include <mrpt/core/exceptions.h>
#include <mrpt/math/eigen_frwds.h>

namespace mrpt
{
/** This base provides a set of functions for maths stuff.
 * \ingroup mrpt_math_grp
 */
namespace math
{
/**\brief Compare 2 floats and determine whether they are equal
 * \return True if equal, false otherwise
 * \param a Fist num
 * \param b Second num
 * \param epsilon Difference below which a, b are considered equal
 */
template <class T1, class T2>
bool approximatelyEqual(T1 a, T1 b, T2 epsilon)
{
	return fabs(a - b) <= ((fabs(a) > fabs(b) ? fabs(b) : fabs(a)) * epsilon);
}

/**\brief Compare 2 floats and determine whether they are equal
 * \return True if equal, false otherwise
 * \param a Fist num
 * \param b Second num
 */
template <class T>
bool approximatelyEqual(T a, T b)
{
	return approximatelyEqual(a, b, std::numeric_limits<T>::epsilon());
}

/**\brief Absolute difference between two numbers.
 *
 */
template <class T>
T absDiff(const T& lhs, const T& rhs)
{
	return lhs > rhs ? lhs - rhs : rhs - lhs;
}

/** \addtogroup container_ops_grp
 * @{ */

/** Loads one row of a text file as a numerical std::vector.
 * \return false on EOF or invalid format.
 * The body of the function is implemented in MATH.cpp
 */
bool loadVector(std::istream& f, std::vector<int>& d);

/** Loads one row of a text file as a numerical std::vector.
 * \return false on EOF or invalid format.
 * The body of the function is implemented in MATH.cpp
 */
bool loadVector(std::istream& f, std::vector<double>& d);

void medianFilter(
	const std::vector<double>& inV, std::vector<double>& outV,
	const int& winSize, const int& numberOfSigmas = 2);

/** Generates an equidistant sequence of numbers given the first one, the last
  one and the desired number of points.
  \sa sequence */
template <typename T, typename VECTOR>
void linspace(T first, T last, size_t count, VECTOR& out_vector)
{
	if (count < 2)
	{
		out_vector.assign(count, last);
		return;
	}
	else
	{
		out_vector.resize(count);
		const T incr = (last - first) / T(count - 1);
		T c = first;
		for (size_t i = 0; i < count; i++, c += incr) out_vector[i] = c;
	}
}

/** Generates a sequence of values [first,first+STEP,first+2*STEP,...]   \sa
 * linspace, sequence */
template <class T, T STEP>
inline std::vector<T> sequenceStdVec(T first, size_t length)
{
	std::vector<T> ret(length);
	if (!length) return ret;
	size_t i = 0;
	while (length--)
	{
		ret[i++] = first;
		first += STEP;
	}
	return ret;
}

/** Normalize a vector, such as its norm is the unity.
 *  If the vector has a null norm, the output is a null vector.
 */
template <class VEC1, class VEC2>
void normalize(const VEC1& v, VEC2& out_v)
{
	typename VEC1::Scalar total = 0;
	const size_t N = v.size();
	for (size_t i = 0; i < N; i++) total += square(v[i]);
	total = std::sqrt(total);
	if (total)
	{
		out_v = v * (1.0 / total);
	}
	else
		out_v.assign(v.size(), 0);
}

/** Extract a column from a vector of vectors, and store it in another vector.
 *  - Input data can be: std::vector<mrpt::math::CVectorDouble>,
 * std::deque<std::list<double> >, std::list<CArrayDouble<5> >, etc. etc.
 *  - Output is the sequence:  data[0][idx],data[1][idx],data[2][idx], etc..
 *
 *  For the sake of generality, this function does NOT check the limits in
 * the number of column, unless it's implemented in the [] operator of each of
 * the "rows".
 */
template <class VECTOR_OF_VECTORS, class VECTORLIKE>
inline void extractColumnFromVectorOfVectors(
	const size_t colIndex, const VECTOR_OF_VECTORS& data,
	VECTORLIKE& out_column)
{
	const size_t N = data.size();
	out_column.resize(N);
	for (size_t i = 0; i < N; i++) out_column[i] = data[i][colIndex];
}

/** Computes the factorial of an integer number and returns it as a 64-bit
 * integer number.
 */
uint64_t factorial64(unsigned int n);

/** Computes the factorial of an integer number and returns it as a double value
 * (internally it uses logarithms for avoiding overflow).
 */
double factorial(unsigned int n);

/** Generates a string with the MATLAB commands required to plot an confidence
 * interval (ellipse) for a 2D Gaussian ('float' version)..
 *  \param cov22 The 2x2 covariance matrix
 *  \param mean  The 2-length vector with the mean
 *  \param stdCount How many "quantiles" to get into the area of the ellipse:
 * 2: 95%, 3:99.97%,...
 *  \param style A matlab style string, for colors, line styles,...
 *  \param nEllipsePoints The number of points in the ellipse to generate
 * \ingroup stats_grp
 */
std::string MATLAB_plotCovariance2D(
	const CMatrixFloat& cov22, const CVectorFloat& mean, const float& stdCount,
	const std::string& style = std::string("b"),
	const size_t& nEllipsePoints = 30);

/** Generates a string with the MATLAB commands required to plot an confidence
 * interval (ellipse) for a 2D Gaussian ('double' version).
 *  \param cov22 The 2x2 covariance matrix
 *  \param mean  The 2-length vector with the mean
 *  \param stdCount How many "quantiles" to get into the area of the ellipse:
 * 2: 95%, 3:99.97%,...
 *  \param style A matlab style string, for colors, line styles,...
 *  \param nEllipsePoints The number of points in the ellipse to generate
 * \ingroup stats_grp
 */
std::string MATLAB_plotCovariance2D(
	const CMatrixDouble& cov22, const CVectorDouble& mean,
	const float& stdCount, const std::string& style = std::string("b"),
	const size_t& nEllipsePoints = 30);

/** Assignment operator for initializing a std::vector from a C array (The
 *vector will be automatically set to the correct size).
 * \code
 *	 CVectorDouble  v;
 *  const double numbers[] = { 1,2,3,5,6,7,8,9,10 };
 *  loadVector( v, numbers );
 * \endcode
 * \note This operator performs the appropiate type castings, if required.
 */
template <typename EIGEN_VECTOR, typename At, size_t N>
EIGEN_VECTOR& loadVector(EIGEN_VECTOR& v, At (&theArray)[N])
{
	static_assert(N != 0, "N!=0");
	v.derived().resize(N);
	for (size_t i = 0; i < N; i++)
		(v.derived())[i] =
			static_cast<typename EIGEN_VECTOR::Scalar>(theArray[i]);
	return v;
}
//! \overload
template <typename T, typename At, size_t N>
std::vector<T>& loadVector(std::vector<T>& v, At (&theArray)[N])
{
	static_assert(N != 0, "N!=0");
	v.resize(N);
	for (size_t i = 0; i < N; i++) v[i] = static_cast<T>(theArray[i]);
	return v;
}

/**  @} */  // end of grouping container_ops_grp

/** \defgroup mrpt_math_io Custom I/O for math containers
 * \ingroup mrpt_math_grp */
/** \addtogroup mrpt_math_io
 * @{ */

/** Saves to a plain-text file the nonzero entries of a Eigen sparse matrix,
 * represented as a vector of triplets.
 *  Output format is one line per entry with the format: "i j val", i:row,
 * j:col, val:value.
 * \tparam TRIPLET should be Eigen::Triplet<T>
 */
template <class TRIPLET>
bool saveEigenSparseTripletsToFile(
	const std::string& sFile, std::vector<TRIPLET>& tri)
{
#if defined(_MSC_VER) && \
	(_MSC_VER >= 1400)  // Use a secure version in Visual Studio 2005+
	FILE* f;
	if (0 != ::fopen_s(&f, sFile.c_str(), "wt")) f = nullptr;
#else
	FILE* f = ::fopen(sFile.c_str(), "wt");
#endif

	if (!f) return false;

	for (size_t i = 0; i < tri.size(); i++)
		fprintf(
			f, "%u %u %e\n", 1 + tri[i].row(), 1 + tri[i].col(),
			tri[i].value());

	fclose(f);
	return true;
}

/** @} */  // End of mrpt_math_io

}  // namespace math

}  // namespace mrpt
