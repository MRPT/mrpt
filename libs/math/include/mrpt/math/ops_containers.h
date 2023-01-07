/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/bits_math.h>  // keep_max(),...
#include <mrpt/math/CHistogram.h>
#include <mrpt/math/math_frwds.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <numeric>
#include <type_traits>

#include "ops_vectors.h"

/** \addtogroup container_ops_grp Vector and matrices mathematical operations
 * and other utilities
 *  \ingroup mrpt_math_grp
 *  @{ */

/** \file ops_containers.h
 * This file implements several operations that operate element-wise on
 * individual or pairs of containers.
 *  Containers here means any of: mrpt::math::CVectorTemplace,
 * mrpt::math::CArray, mrpt::math::CMatrixFixed,
 * mrpt::math::CMatrixDynamic.
 *
 */

namespace mrpt::math
{
/** ContainerType<T>::element_t exposes the value of any STL or Eigen container
 */
template <typename CONTAINER>
struct ContainerType;

/** Specialization for Eigen containers */
template <typename Derived>
struct ContainerType<Eigen::EigenBase<Derived>>
{
	using element_t = typename Derived::Scalar;
};
/** Specialization for MRPT containers */
template <typename Scalar, typename Derived>
struct ContainerType<mrpt::math::MatrixVectorBase<Scalar, Derived>>
{
	using element_t = Scalar;
};

/** Computes the normalized or normal histogram of a sequence of numbers given
 * the number of bins and the limits.
 *  In any case this is a "linear" histogram, i.e. for matrices, all the
 * elements are taken as if they were a plain sequence, not taking into account
 * they were in columns or rows.
 *  If desired, out_bin_centers can be set to receive the bins centers.
 */
template <class CONTAINER>
std::vector<double> histogram(
	const CONTAINER& v, double limit_min, double limit_max, size_t number_bins,
	bool do_normalization = false,
	std::vector<double>* out_bin_centers = nullptr)
{
	mrpt::math::CHistogram H(limit_min, limit_max, number_bins);
	std::vector<double> ret(number_bins);
	std::vector<double> dummy_ret_bins;
	H.add(v);
	if (do_normalization)
		H.getHistogramNormalized(
			out_bin_centers ? *out_bin_centers : dummy_ret_bins, ret);
	else
		H.getHistogram(
			out_bin_centers ? *out_bin_centers : dummy_ret_bins, ret);
	return ret;
}

template <class EIGEN_CONTAINER>
void resizeLike(EIGEN_CONTAINER& trg, const EIGEN_CONTAINER& src)
{
	trg.resizeLike(src);
}
template <typename T>
void resizeLike(std::vector<T>& trg, const std::vector<T>& src)
{
	trg.resize(src.size());
}

/** Computes the cumulative sum of all the elements, saving the result in
 * another container.
 *  This works for both matrices (even mixing their types) and vectores/arrays
 * (even mixing types),
 *  and even to store the cumsum of any matrix into any vector/array, but not
 * in opposite direction.
 * \sa sum */
template <class CONTAINER1, class CONTAINER2>
inline void cumsum_tmpl(const CONTAINER1& in_data, CONTAINER2& out_cumsum)
{
	resizeLike(out_cumsum, in_data);
	using T =
		std::remove_const_t<std::remove_reference_t<decltype(in_data[0])>>;
	T last = 0;
	const size_t N = in_data.size();
	for (size_t i = 0; i < N; i++)
		last = out_cumsum[i] = last + in_data[i];
}

template <class CONTAINER1, class CONTAINER2>
inline void cumsum(const CONTAINER1& in_data, CONTAINER2& out_cumsum)
{
	cumsum_tmpl<CONTAINER1, CONTAINER2>(in_data, out_cumsum);
}

/** Computes the cumulative sum of all the elements
 * \sa sum  */
template <class CONTAINER>
inline CONTAINER cumsum(const CONTAINER& in_data)
{
	CONTAINER ret;
	cumsum(in_data, ret);
	return ret;
}

template <class CONTAINER>
inline typename CONTAINER::Scalar norm_inf(const CONTAINER& v)
{
	return v.norm_inf();
}
template <class CONTAINER>
inline typename CONTAINER::Scalar norm(const CONTAINER& v)
{
	return v.norm();
}
template <class CONTAINER, int = CONTAINER::is_mrpt_type>
inline typename CONTAINER::Scalar maximum(const CONTAINER& v)
{
	return v.maxCoeff();
}
template <class CONTAINER, int = CONTAINER::is_mrpt_type>
inline typename CONTAINER::Scalar minimum(const CONTAINER& v)
{
	return v.minCoeff();
}

template <class Derived>
inline typename Derived::Scalar maximum(const Eigen::MatrixBase<Derived>& v)
{
	return v.maxCoeff();
}
template <class Derived>
inline typename Derived::Scalar minimum(const Eigen::MatrixBase<Derived>& v)
{
	return v.minCoeff();
}

template <typename T>
inline T maximum(const std::vector<T>& v)
{
	ASSERT_(!v.empty());
	T m = v[0];
	for (size_t i = 0; i < v.size(); i++)
		mrpt::keep_max(m, v[i]);
	return m;
}
template <typename T>
inline T minimum(const std::vector<T>& v)
{
	ASSERT_(!v.empty());
	T m = v[0];
	for (size_t i = 0; i < v.size(); i++)
		mrpt::keep_min(m, v[i]);
	return m;
}

/** \name Generic container element-wise operations - Miscelaneous
 * @{
 */

/** Accumulate the squared-norm of a vector/array/matrix into "total" (this
 * function is compatible with std::accumulate). */
template <class CONTAINER, typename VALUE>
VALUE squareNorm_accum(const VALUE total, const CONTAINER& v)
{
	return total + v.squaredNorm();
}

/** Compute the square norm of anything implementing [].
  \sa norm */
template <size_t N, class T, class U>
inline T squareNorm(const U& v)
{
	T res = 0;
	for (size_t i = 0; i < N; i++)
		res += square(v[i]);
	return res;
}

/** v1*v2: The dot product of two containers (vectors/arrays/matrices) */
template <class CONTAINER1, class CONTAINER2>
inline typename CONTAINER1::Scalar dotProduct(
	const CONTAINER1& v1, const CONTAINER1& v2)
{
	return v1.dot(v2);
}

/** v1*v2: The dot product of any two objects supporting []  */
template <size_t N, class T, class U, class V>
inline T dotProduct(const U& v1, const V& v2)
{
	T res = 0;
	for (size_t i = 0; i < N; i++)
		res += v1[i] * v2[i];
	return res;
}

/** Computes the sum of all the elements.
  * \note If used with containers of integer types (uint8_t, int, etc...) this
  could overflow. In those cases, use sumRetType the second argument RET to
  specify a larger type to hold the sum.
   \sa cumsum  */
template <class CONTAINER>
inline typename CONTAINER::Scalar sum(const CONTAINER& v)
{
	return v.sum();
}

/// \overload
template <typename T>
inline T sum(const std::vector<T>& v)
{
	return std::accumulate(v.begin(), v.end(), T(0));
}

/** Computes the sum of all the elements, with a custom return type.
   \sa sum, cumsum  */
template <class CONTAINER, typename RET>
inline RET sumRetType(const CONTAINER& v)
{
	return v.template sumRetType<RET>();
}

/** Computes the mean value of a vector  \return The mean, as a double number.
 * \sa math::stddev,math::meanAndStd  */
template <class CONTAINER>
inline double mean(const CONTAINER& v)
{
	if (v.empty()) return 0;
	else
		return sum(v) / static_cast<double>(v.size());
}

/** Return the maximum and minimum values of a std::vector */
template <typename T>
inline void minimum_maximum(const std::vector<T>& V, T& curMin, T& curMax)
{
	ASSERT_(V.size() != 0);
	const size_t N = V.size();
	curMin = curMax = V[0];
	for (size_t i = 1; i < N; i++)
	{
		mrpt::keep_min(curMin, V[i]);
		mrpt::keep_max(curMax, V[i]);
	}
}

/** Return the maximum and minimum values of a Eigen-based vector or matrix */
template <class Derived>
inline void minimum_maximum(
	const Eigen::MatrixBase<Derived>& V,
	typename Eigen::MatrixBase<Derived>::Scalar& curMin,
	typename Eigen::MatrixBase<Derived>::Scalar& curMax)
{
	V.minimum_maximum(curMin, curMax);
}

/** Scales all elements such as the minimum & maximum values are shifted to the
 * given values */
template <class CONTAINER, typename Scalar>
void normalize(CONTAINER& c, Scalar valMin, Scalar valMax)
{
	if (!c.size()) return;	// empty() is not defined for Eigen classes
	const Scalar curMin = c.minCoeff();
	const Scalar curMax = c.maxCoeff();
	Scalar minMaxDelta = curMax - curMin;
	if (minMaxDelta == 0) minMaxDelta = 1;
	const Scalar minMaxDelta_ = (valMax - valMin) / minMaxDelta;
	c.array() = (c.array() - curMin) * minMaxDelta_ + valMin;
}

/** Counts the number of elements that appear in both STL-like containers
 * (comparison through the == operator)
 *  It is assumed that no repeated elements appear within each of the
 * containers.  */
template <class CONTAINER1, class CONTAINER2>
size_t countCommonElements(const CONTAINER1& a, const CONTAINER2& b)
{
	size_t ret = 0;
	for (auto it1 = a.begin(); it1 != a.end(); ++it1)
		for (auto it2 = b.begin(); it2 != b.end(); ++it2)
			if ((*it1) == (*it2)) ret++;
	return ret;
}

/** Adjusts the range of all the elements such as the minimum and maximum values
 * being those supplied by the user.  */
template <class CONTAINER>
void adjustRange(
	CONTAINER& m, const typename CONTAINER::Scalar minVal,
	const typename CONTAINER::Scalar maxVal)
{
	if (size_t(m.size()) == 0) return;
	typename CONTAINER::Scalar curMin, curMax;
	minimum_maximum(m, curMin, curMax);
	const typename CONTAINER::Scalar curRan = curMax - curMin;
	m -= (curMin + minVal);
	if (curRan != 0) m *= (maxVal - minVal) / curRan;
}

/** Computes the standard deviation of a vector (or all elements of a matrix)
 * \param v The set of data, either as a vector, or a matrix (arrangement of
 * data is ignored in this function).
 * \param out_mean The output for the estimated mean
 * \param out_std The output for the estimated standard deviation
 * \param unbiased If set to true or false the std is normalized by "N-1" or
 * "N", respectively.
 * \sa math::mean,math::stddev
 */
template <class VECTORLIKE>
void meanAndStd(
	const VECTORLIKE& v, double& out_mean, double& out_std,
	bool unbiased = true)
{
	if (v.size() < 2)
	{
		out_std = 0;
		out_mean = (v.size() == 1) ? *v.begin() : 0;
	}
	else
	{
		// Compute the mean:
		const size_t N = v.size();
		out_mean = mrpt::math::sum(v) / static_cast<double>(N);
		// Compute the std:
		double vector_std = 0;
		for (size_t i = 0; i < N; i++)
			vector_std += mrpt::square(v[i] - out_mean);
		out_std =
			std::sqrt(vector_std / static_cast<double>(N - (unbiased ? 1 : 0)));
	}
}

/** Computes the standard deviation of a vector
 * \param v The set of data
 * \param unbiased If set to true or false the std is normalized by "N-1" or
 * "N", respectively.
 * \sa math::mean,math::meanAndStd
 */
template <class VECTORLIKE>
inline double stddev(const VECTORLIKE& v, bool unbiased = true)
{
	double m, s;
	meanAndStd(v, m, s, unbiased);
	return s;
}

/** Computes the mean vector and covariance from a list of values given as a
 * vector of vectors, where each row is a sample.
 * \param v The set of data, as a vector of N vectors of M elements.
 * \param out_mean The output M-vector for the estimated mean.
 * \param out_cov The output MxM matrix for the estimated covariance matrix.
 * \sa mrpt::math::meanAndCovMat, math::mean,math::stddev, math::cov
 */
template <class VECTOR_OF_VECTOR, class VECTORLIKE, class MATRIXLIKE>
void meanAndCovVec(
	const VECTOR_OF_VECTOR& v, VECTORLIKE& out_mean, MATRIXLIKE& out_cov)
{
	const size_t N = v.size();
	ASSERTMSG_(N > 0, "The input vector contains no elements");
	const double N_inv = 1.0 / N;

	const size_t M = v[0].size();
	ASSERTMSG_(M > 0, "The input vector contains rows of length 0");

	// First: Compute the mean
	out_mean.assign(M, 0);
	for (size_t i = 0; i < N; i++)
		for (size_t j = 0; j < M; j++)
			out_mean[j] += v[i][j];

	for (size_t j = 0; j < M; j++)
		out_mean[j] *= N_inv;

	// Second: Compute the covariance
	//  Save only the above-diagonal part, then after averaging
	//  duplicate that part to the other half.
	out_cov.setZero(M, M);
	for (size_t i = 0; i < N; i++)
	{
		for (size_t j = 0; j < M; j++)
			out_cov(j, j) += square(v[i][j] - out_mean[j]);

		for (size_t j = 0; j < M; j++)
			for (size_t k = j + 1; k < M; k++)
				out_cov(j, k) +=
					(v[i][j] - out_mean[j]) * (v[i][k] - out_mean[k]);
	}
	for (size_t j = 0; j < M; j++)
		for (size_t k = j + 1; k < M; k++)
			out_cov(k, j) = out_cov(j, k);
	out_cov *= N_inv;
}

/** Computes the covariance matrix from a list of values given as a vector of
 * vectors, where each row is a sample.
 * \param v The set of data, as a vector of N vectors of M elements.
 * \param out_cov The output MxM matrix for the estimated covariance matrix.
 * \tparam RETURN_MATRIX The type of the returned matrix, e.g. Eigen::MatrixXd
 * \sa math::mean,math::stddev, math::cov, meanAndCovVec
 */
template <class VECTOR_OF_VECTOR, class RETURN_MATRIX>
inline RETURN_MATRIX covVector(const VECTOR_OF_VECTOR& v)
{
	std::vector<double> m;
	RETURN_MATRIX C;
	meanAndCovVec(v, m, C);
	return C;
}

/** Normalized Cross Correlation coefficient between two 1-D vectors, returning
 * a single scalar between [-1, 1].
 *
 * It is equivalent to the following Matlab code:
 * \code
 * a = a - mean2(a);
 * b = b - mean2(b);
 * r = sum(sum(a.*b))/sqrt(sum(sum(a.*a))*sum(sum(b.*b)));
 * \endcode
 * \tparam CONT1 A std::vector<double>, Eigen or mrpt::math vectors.
 * \tparam CONT2 A std::vector<double>, Eigen or mrpt::math vectors.
 *
 * \sa xcorr
 */
template <class CONT1, class CONT2>
double ncc_vector(const CONT1& a, const CONT2& b)
{
	ASSERT_EQUAL_(a.size(), b.size());

	double numerator = 0, sum_a = 0, sum_b = 0, result, a_mean, b_mean;

	a_mean = mrpt::math::mean(a);
	b_mean = mrpt::math::mean(b);

	const size_t N = a.size();
	for (size_t i = 0; i < N; ++i)
	{
		numerator += (a[i] - a_mean) * (b[i] - b_mean);
		sum_a += mrpt::square(a[i] - a_mean);
		sum_b += mrpt::square(b[i] - b_mean);
	}
	ASSERTMSG_(sum_a * sum_b != 0, "Divide by zero when normalizing.");
	result = numerator / std::sqrt(sum_a * sum_b);
	return result;
}

/** Normalized Cross Correlation between two 1-D vectors, returning a vector
 *  of scalars, with a peak at the position revealing the offset of "b"
 *  that makes the two signals most similar:
 * \code
 *  r = xcorr(a, b, maxLag);
 *  lags = mrpt::math::linspace(-maxLag, maxLag);
 * \endcode
 *
 * Where:
 *  - `a` and `b` are the input signals.
 *  - `r` is the output cross correlation vector. Its length is `maxLag*2+1`.
 *  - `maxLag` is the maximum lag to search for.
 *  - `lags`: If needed, it can be generated with `linspace()`: it will hold the
 * "delay counts" for each corresponding entry in `r`, the sequence of
 * integers [-maxLag, maxLag].
 *
 * \tparam VECTOR A std::vector<double>, Eigen or mrpt::math vectors.
 *
 * \sa ncc_vector
 * \note (New in MRPT 2.3.3)
 */
template <class VECTOR>
VECTOR xcorr(
	const VECTOR& a, const VECTOR& b, const size_t maxLag,
	bool normalized = true)
{
	MRPT_START

	const signed int na = a.size(), nb = b.size();
	ASSERT_(na > 0);
	ASSERT_(nb > 0);
	ASSERTMSG_(
		!normalized || na == nb,
		"normalized=true is only possible for input sequences of identical "
		"lengths.");

	const auto a_mean = mrpt::math::mean(a);
	const auto b_mean = mrpt::math::mean(b);

	// Cache "a" and "b" demeaned and squared to faster repeated access later:
	auto az = a, asq = a;
	for (int i = 0; i < na; i++)
	{
		az[i] -= a_mean;
		asq[i] = mrpt::square(az[i]);
	}
	auto bz = b, bsq = b;
	for (int i = 0; i < nb; i++)
	{
		bz[i] -= b_mean;
		bsq[i] = mrpt::square(bz[i]);
	}

	VECTOR result;
	result.resize(maxLag * 2 + 1);

	const signed int maxLag_i = static_cast<signed int>(maxLag);
	for (int lag = -maxLag_i, idx = 0; lag <= maxLag_i; ++lag, ++idx)
	{
		double numerator = 0, sum_a = 0, sum_b = 0;
		for (int i_a = 0; i_a < na; ++i_a)
		{
			if (i_a + lag >= nb || i_a + lag < 0) continue;

			numerator += az[i_a] * bz[i_a + lag];
			if (normalized)
			{
				sum_a += asq[i_a];
				sum_b += bsq[i_a + lag];
			}
		}
		const auto sasb = sum_a * sum_b;
		const auto r = sasb != 0 ? numerator / std::sqrt(sasb) : numerator;
		result[idx] = r;
	}

	return result;
	MRPT_END
}

/** @} Misc ops */

}  // namespace mrpt::math
/**  @} */	// end of grouping
