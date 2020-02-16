/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/random/random_shuffle.h>
#include <algorithm>
#include <cstddef>
#include <limits>
#include <limits>  // numeric_limits
#include <random>
#include <stdexcept>
#include <type_traits>  // remove_reference
#include <vector>

// Frwd decl:
namespace Eigen
{
template <typename _MatrixType>
class SelfAdjointEigenSolver;
}

namespace mrpt
{
/** A namespace of pseudo-random numbers generators of diferent distributions.
 * The central class in this namespace is mrpt::random::CRandomGenerator
 * \ingroup mrpt_random_grp
 */
namespace random
{
/** Portable MT19937 random generator, C++11 UniformRandomBitGenerator
 * compliant.
 *
 * It is ensured to generate the same numbers on any compiler and system.
 * \ingroup mrpt_random_grp
 */
class Generator_MT19937
{
   public:
	using result_type = uint32_t;
	static constexpr result_type min()
	{
		return std::numeric_limits<result_type>::min();
	}
	static constexpr result_type max()
	{
		return std::numeric_limits<result_type>::max();
	}

	result_type operator()();

	void seed(const uint32_t seed);

   private:
	uint32_t m_MT[624];
	uint32_t m_index{0};
	bool m_seed_initialized{false};

	void generateNumbers();
};

/** A thred-safe pseudo random number generator, based on an internal MT19937
 * randomness generator.
 * The base algorithm for randomness is platform-independent. See
 * http://en.wikipedia.org/wiki/Mersenne_twister
 *
 * For real thread-safety, each thread must create and use its own instance of
 * this class.
 *
 * Single-thread programs can use the static object
 * mrpt::random::randomGenerator
 * \ingroup mrpt_random_grp
 */
class CRandomGenerator
{
   protected:
	/** Data used internally by the MT19937 PRNG algorithm. */
	Generator_MT19937 m_MT19937;

	std::normal_distribution<double> m_normdistribution;
	std::uniform_int_distribution<uint32_t> m_uint32;
	std::uniform_int_distribution<uint64_t> m_uint64;

   public:
	/** @name Initialization
	 @{ */

	/** Default constructor: initialize random seed based on current time */
	CRandomGenerator() { randomize(); }
	/** Constructor for providing a custom random seed to initialize the PRNG */
	CRandomGenerator(const uint32_t seed) { randomize(seed); }
	/** Initialize the PRNG from the given random seed */
	void randomize(const uint32_t seed);
	/** Randomize the generators, based on std::random_device */
	void randomize();

	/** @} */

	/** @name Uniform pdf
	 @{ */

	/** Generate a uniformly distributed pseudo-random number using the MT19937
	 * algorithm, in the whole range of 32-bit integers.
	 *  See: http://en.wikipedia.org/wiki/Mersenne_twister */
	uint32_t drawUniform32bit();

	/** Returns a uniformly distributed pseudo-random number by joining two
	 * 32bit numbers from \a drawUniform32bit() */
	uint64_t drawUniform64bit();

	/** You can call this overloaded method with either 32 or 64bit unsigned
	 * ints for the sake of general coding. */
	void drawUniformUnsignedInt(uint32_t& ret_number)
	{
		ret_number = m_uint32(m_MT19937);
	}
	void drawUniformUnsignedInt(uint64_t& ret_number)
	{
		ret_number = m_uint64(m_MT19937);
	}

	/** Return a uniform unsigned integer in the range [min_val,max_val] (both
	 * inclusive) */
	template <typename T, typename U, typename V>
	void drawUniformUnsignedIntRange(
		T& ret_number, const U min_val, const V max_val)
	{
		const T range = max_val - min_val + 1;
		T rnd;
		drawUniformUnsignedInt(rnd);
		ret_number = min_val + (rnd % range);
	}

	/** Generate a uniformly distributed pseudo-random number using the MT19937
	 * algorithm, scaled to the selected range. */
	template <typename return_t = double>
	return_t drawUniform(const double Min, const double Max)
	{
		double k = 2.3283064370807973754314699618685e-10;  // 0xFFFFFFFF ^ -1
		return static_cast<return_t>(
			Min + (Max - Min) * drawUniform32bit() * k);
	}

	/** Fills the given matrix with independent, uniformly distributed samples.
	 * Matrix classes can be mrpt::math::CMatrixDynamic or
	 * mrpt::math::CMatrixFixed
	 * \sa drawUniform
	 */
	template <class MAT>
	void drawUniformMatrix(
		MAT& matrix, const double unif_min = 0, const double unif_max = 1)
	{
		for (size_t r = 0; r < matrix.rows(); r++)
			for (size_t c = 0; c < matrix.cols(); c++)
				matrix(r, c) = static_cast<typename MAT::Scalar>(
					drawUniform(unif_min, unif_max));
	}

	/** Fills the given vector with independent, uniformly distributed samples.
	 * \sa drawUniform
	 */
	template <class VEC>
	void drawUniformVector(
		VEC& v, const double unif_min = 0, const double unif_max = 1)
	{
		const size_t N = v.size();
		for (size_t c = 0; c < N; c++)
			v[c] = static_cast<typename std::decay<decltype(v[c])>::type>(
				drawUniform(unif_min, unif_max));
	}

	/** @} */

	/** @name Normal/Gaussian pdf
	 @{ */

	/** Generate a normalized (mean=0, std=1) normally distributed sample.
	 *  \param likelihood If desired, pass a pointer to a double which will
	 * receive the likelihood of the given sample to have been obtained, that
	 * is, the value of the normal pdf at the sample value.
	 */
	double drawGaussian1D_normalized();

	/** Generate a normally distributed pseudo-random number.
	 * \param mean The mean value of desired normal distribution
	 * \param std  The standard deviation value of desired normal distribution
	 */
	double drawGaussian1D(const double mean, const double std)
	{
		return mean + std * drawGaussian1D_normalized();
	}

	/** Fills the given matrix with independent, 1D-normally distributed
	 * samples.
	 * Matrix classes can be mrpt::math::CMatrixDynamic or
	 * mrpt::math::CMatrixFixed
	 * \sa drawGaussian1D
	 */
	template <class MAT>
	void drawGaussian1DMatrix(
		MAT& matrix, const double mean = 0, const double std = 1)
	{
		for (decltype(matrix.rows()) r = 0; r < matrix.rows(); r++)
			for (decltype(matrix.cols()) c = 0; c < matrix.cols(); c++)
				matrix(r, c) = static_cast<typename MAT::Scalar>(
					drawGaussian1D(mean, std));
	}

	/** Generates a random definite-positive matrix of the given size, using the
	 * formula C = v*v^t + epsilon*I, with "v" being a vector of gaussian random
	 * samples.
	 */
	template <class MATRIX, class AUXVECTOR_T = MATRIX>
	MATRIX drawDefinitePositiveMatrix(
		const size_t dim, const double std_scale = 1.0,
		const double diagonal_epsilon = 1e-8)
	{
		AUXVECTOR_T r(dim, 1);
		drawGaussian1DMatrix(r, 0, std_scale);
		MATRIX cov;
		cov.resize(dim, dim);
		cov.matProductOf_AAt(r);  // random semi-definite positive matrix:
		for (size_t i = 0; i < dim; i++)
			cov(i, i) += diagonal_epsilon;  // make sure it's definite-positive
		return cov;
	}

	/** Fills the given vector with independent, 1D-normally distributed
	 * samples.
	 * \sa drawGaussian1D
	 */
	template <class VEC>
	void drawGaussian1DVector(
		VEC& v, const double mean = 0, const double std = 1)
	{
		const size_t N = v.size();
		for (size_t c = 0; c < N; c++)
			v[c] = static_cast<std::remove_reference_t<decltype(v[c])>>(
				drawGaussian1D(mean, std));
	}

	/** Generate multidimensional random samples according to a given covariance
	 * matrix.
	 *  Mean is assumed to be zero if mean==nullptr.
	 * \exception std::exception On invalid covariance matrix
	 * \sa drawGaussianMultivariateMany
	 */
	template <typename T, typename MATRIX>
	void drawGaussianMultivariate(
		std::vector<T>& out_result, const MATRIX& cov,
		const std::vector<T>* mean = nullptr)
	{
		const size_t dim = cov.cols();
		if (cov.rows() != cov.cols())
			throw std::runtime_error(
				"drawGaussianMultivariate(): cov is not square.");
		if (mean && mean->size() != dim)
			throw std::runtime_error(
				"drawGaussianMultivariate(): mean and cov sizes ");
		MATRIX Z, D;
		// Set size of output vector:
		out_result.clear();
		out_result.resize(dim, 0);
		/** Computes the eigenvalues/eigenvector decomposition of this matrix,
		 *    so that: M = Z * D * Z<sup>T</sup>, where columns in Z are the
		 *	  eigenvectors and the diagonal matrix D contains the eigenvalues
		 *    as diagonal elements, sorted in <i>ascending</i> order.
		 */
		cov.eigenVectors(Z, D);
		// Scale eigenvectors with eigenvalues:
		D = D.array().sqrt().matrix();
		Z.matProductOf_AB(Z, D);
		for (size_t i = 0; i < dim; i++)
		{
			T rnd = this->drawGaussian1D_normalized();
			for (size_t d = 0; d < dim; d++) out_result[d] += (Z(d, i) * rnd);
		}
		if (mean)
			for (size_t d = 0; d < dim; d++) out_result[d] += (*mean)[d];
	}

	/** Generate multidimensional random samples according to a given covariance
	 * matrix.
	 *  Mean is assumed to be zero if mean==nullptr.
	 * \exception std::exception On invalid covariance matrix
	 * \sa drawGaussianMultivariateMany
	 */
	template <class VECTORLIKE, class COVMATRIX>
	void drawGaussianMultivariate(
		VECTORLIKE& out_result, const COVMATRIX& cov,
		const VECTORLIKE* mean = nullptr)
	{
		const size_t N = cov.rows();
		if (cov.rows() != cov.cols())
			throw std::runtime_error(
				"drawGaussianMultivariate(): cov is not square.");
		if (mean && size_t(mean->size()) != N)
			throw std::runtime_error(
				"drawGaussianMultivariate(): mean and cov sizes ");

		// Compute eigenvalues/eigenvectors of cov:
		COVMATRIX eigVecs;
		std::vector<typename COVMATRIX::Scalar> eigVals;
		cov.eig_symmetric(eigVecs, eigVals, false /*sorted*/);

		// Scale eigenvectors with eigenvalues:
		// D.Sqrt(); Z = Z * D; (for each column)
		for (typename COVMATRIX::Index c = 0; c < eigVecs.cols(); c++)
		{
			const auto s = std::sqrt(eigVals[c]);
			for (typename COVMATRIX::Index r = 0; r < eigVecs.rows(); r++)
				eigVecs(c, r) *= s;
		}

		// Set size of output vector:
		out_result.assign(N, 0);

		for (size_t i = 0; i < N; i++)
		{
			typename COVMATRIX::Scalar rnd = drawGaussian1D_normalized();
			for (size_t d = 0; d < N; d++)
				out_result[d] += eigVecs.coeff(d, i) * rnd;
		}
		if (mean)
			for (size_t d = 0; d < N; d++) out_result[d] += (*mean)[d];
	}

	/** Generate a given number of multidimensional random samples according to
	 * a given covariance matrix.
	 * \param cov The covariance matrix where to draw the samples from.
	 * \param desiredSamples The number of samples to generate.
	 * \param ret The output list of samples
	 * \param mean The mean, or zeros if mean==nullptr.
	 */
	template <typename VECTOR_OF_VECTORS, typename COVMATRIX>
	void drawGaussianMultivariateMany(
		VECTOR_OF_VECTORS& ret, size_t desiredSamples, const COVMATRIX& cov,
		const typename VECTOR_OF_VECTORS::value_type* mean = nullptr)
	{
		const size_t N = cov.rows();
		if (cov.rows() != cov.cols())
			throw std::runtime_error(
				"drawGaussianMultivariateMany(): cov is not square.");
		if (mean && size_t(mean->size()) != N)
			throw std::runtime_error(
				"drawGaussianMultivariateMany(): mean and cov sizes ");

		// Compute eigenvalues/eigenvectors of cov:
		COVMATRIX eigVecs;
		std::vector<typename COVMATRIX::Scalar> eigVals;
		cov.eig_symmetric(eigVecs, eigVals, false /*sorted*/);

		// Scale eigenvectors with eigenvalues:
		// D.Sqrt(); Z = Z * D; (for each column)
		for (typename COVMATRIX::Index c = 0; c < eigVecs.cols(); c++)
		{
			const auto s = std::sqrt(eigVals[c]);
			for (typename COVMATRIX::Index r = 0; r < eigVecs.rows(); r++)
				eigVecs(c, r) *= s;
		}

		// Set size of output vector:
		ret.resize(desiredSamples);
		for (size_t k = 0; k < desiredSamples; k++)
		{
			ret[k].assign(N, 0);
			for (size_t i = 0; i < N; i++)
			{
				typename COVMATRIX::Scalar rnd = drawGaussian1D_normalized();
				for (size_t d = 0; d < N; d++)
					ret[k][d] += eigVecs.coeff(d, i) * rnd;
			}
			if (mean)
				for (size_t d = 0; d < N; d++) ret[k][d] += (*mean)[d];
		}
	}

	/** @} */

	/** @name Miscellaneous
	 @{ */

	/** Returns a random permutation of a vector: all the elements of the input
	 * vector are in the output but at random positions.
	 */
	template <class VEC>
	void permuteVector(const VEC& in_vector, VEC& out_result)
	{
		out_result = in_vector;
		const size_t N = out_result.size();
		if (N > 1) mrpt::random::shuffle(&out_result[0], &out_result[N - 1]);
	}

	/** @} */

};  // end of CRandomGenerator
// --------------------------------------------------------------

/** A static instance of a CRandomGenerator class, for use in single-thread
 * applications */
CRandomGenerator& getRandomGenerator();

/** A random number generator for usage in STL algorithms expecting a function
 * like this (eg, random_shuffle):
 */
inline ptrdiff_t random_generator_for_STL(ptrdiff_t i)
{
	return getRandomGenerator().drawUniform32bit() % i;
}

/** Fills the given matrix with independent, uniformly distributed samples.
 * Matrix classes can be mrpt::math::CMatrixDynamic or
 * mrpt::math::CMatrixFixed
 * \sa matrixRandomNormal
 */
template <class MAT>
void matrixRandomUni(
	MAT& matrix, const double unif_min = 0, const double unif_max = 1)
{
	for (typename MAT::Index r = 0; r < matrix.rows(); r++)
		for (typename MAT::Index c = 0; c < matrix.cols(); c++)
			matrix(r, c) = static_cast<typename MAT::Scalar>(
				getRandomGenerator().drawUniform(unif_min, unif_max));
}

/** Fills the given matrix with independent, uniformly distributed samples.
 * \sa vectorRandomNormal
 */
template <class T>
void vectorRandomUni(
	std::vector<T>& v_out, const T& unif_min = 0, const T& unif_max = 1)
{
	size_t n = v_out.size();
	for (size_t r = 0; r < n; r++)
		v_out[r] = getRandomGenerator().drawUniform(unif_min, unif_max);
}

/** Fills the given matrix with independent, normally distributed samples.
 * Matrix classes can be mrpt::math::CMatrixDynamic or
 * mrpt::math::CMatrixFixed
 * \sa matrixRandomUni
 */
template <class MAT>
void matrixRandomNormal(
	MAT& matrix, const double mean = 0, const double std = 1)
{
	for (typename MAT::Index r = 0; r < matrix.rows(); r++)
		for (typename MAT::Index c = 0; c < matrix.cols(); c++)
			matrix(r, c) = static_cast<typename MAT::Scalar>(
				mean + std * getRandomGenerator().drawGaussian1D_normalized());
}

/** Generates a random vector with independent, normally distributed samples.
 * \sa matrixRandomUni
 */
template <class T>
void vectorRandomNormal(
	std::vector<T>& v_out, const T& mean = 0, const T& std = 1)
{
	size_t n = v_out.size();
	for (size_t r = 0; r < n; r++)
		v_out[r] =
			mean + std * getRandomGenerator().drawGaussian1D_normalized();
}

/** Randomize the generators.
 *   A seed can be providen, or a current-time based seed can be used (default)
 */
inline void Randomize(const uint32_t seed)
{
	getRandomGenerator().randomize(seed);
}
inline void Randomize() { getRandomGenerator().randomize(); }
/** Returns a random permutation of a vector: all the elements of the input
 * vector are in the output but at random positions.
 */
template <class T>
void randomPermutation(
	const std::vector<T>& in_vector, std::vector<T>& out_result)
{
	getRandomGenerator().permuteVector(in_vector, out_result);
}

/** Generate a given number of multidimensional random samples according to a
 * given covariance matrix.
 * \param cov The covariance matrix where to draw the samples from.
 * \param desiredSamples The number of samples to generate.
 * \param samplesLikelihoods If desired, set to a valid pointer to a vector,
 * where it will be stored the likelihoods of having obtained each sample: the
 * product of the gaussian-pdf for each independent variable.
 * \param ret The output list of samples
 *
 * \exception std::exception On invalid covariance matrix
 *
 * \sa randomNormalMultiDimensional
 */
template <typename T, typename MATRIX>
void randomNormalMultiDimensionalMany(
	const MATRIX& cov, size_t desiredSamples, std::vector<std::vector<T>>& ret,
	std::vector<T>* samplesLikelihoods = nullptr)
{
	getRandomGenerator().drawGaussianMultivariateMany(
		ret, desiredSamples, cov, static_cast<const std::vector<T>*>(nullptr),
		samplesLikelihoods);
}

/** Generate multidimensional random samples according to a given covariance
 * matrix.
 * \exception std::exception On invalid covariance matrix
 * \sa randomNormalMultiDimensional
 */
template <typename T, typename MATRIXLIKE>
void randomNormalMultiDimensionalMany(
	const MATRIXLIKE& cov, size_t desiredSamples,
	std::vector<std::vector<T>>& ret)
{
	getRandomGenerator().drawGaussianMultivariateMany(ret, desiredSamples, cov);
}

/** Generate multidimensional random samples according to a given covariance
 * matrix.
 * \exception std::exception On invalid covariance matrix
 * \sa randomNormalMultiDimensionalMany
 */
template <typename T, typename MATRIX>
void randomNormalMultiDimensional(const MATRIX& cov, std::vector<T>& out_result)
{
	getRandomGenerator().drawGaussianMultivariate(out_result, cov);
}

}  // namespace random
}  // namespace mrpt
