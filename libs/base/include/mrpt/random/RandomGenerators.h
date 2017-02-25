/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef RandomGenerator_H
#define RandomGenerator_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/math/CMatrixTemplateNumeric.h>

namespace mrpt
{
	namespace math { template <class T> class CMatrixTemplateNumeric; }  // Frwd. decl.

	/** A namespace of pseudo-random numbers genrators of diferent distributions. The central class in this namespace is mrpt::random::CRandomGenerator
	 * \ingroup mrpt_base_grp
	 */
	namespace random
	{
		/** A thred-safe pseudo random number generator, based on an internal MT19937 randomness generator.
		  * The base algorithm for randomness is platform-independent. See http://en.wikipedia.org/wiki/Mersenne_twister
		  *
		  * For real thread-safety, each thread must create and use its own instance of this class.
		  *
		  * Single-thread programs can use the static object mrpt::random::randomGenerator
		 * \ingroup mrpt_base_grp
		  */
		class BASE_IMPEXP CRandomGenerator
		{
		protected:
			/** Data used internally by the MT19937 PRNG algorithm. */
			struct  TMT19937_data
			{
				TMT19937_data() : index(0), seed_initialized(false)
				{}
				uint32_t	MT[624];
				uint32_t	index;
				bool		seed_initialized;
			} m_MT19937_data;

			bool   m_std_gauss_set;
			double m_std_gauss_next;

			void MT19937_generateNumbers();
			void MT19937_initializeGenerator(const uint32_t &seed);

		public:

			/** @name Initialization
			 @{ */

				/** Default constructor: initialize random seed based on current time */
				CRandomGenerator() : m_MT19937_data(),m_std_gauss_set(false) { randomize(); }

				/** Constructor for providing a custom random seed to initialize the PRNG */
				CRandomGenerator(const uint32_t seed) : m_MT19937_data() { randomize(seed); }

				void randomize(const uint32_t seed);  //!< Initialize the PRNG from the given random seed
				void randomize();	//!< Randomize the generators, based on current time

			/** @} */

			/** @name Uniform pdf
			 @{ */

				/** Generate a uniformly distributed pseudo-random number using the MT19937 algorithm, in the whole range of 32-bit integers.
				  *  See: http://en.wikipedia.org/wiki/Mersenne_twister */
				uint32_t drawUniform32bit();

				/** Returns a uniformly distributed pseudo-random number by joining two 32bit numbers from \a drawUniform32bit() */
				uint64_t drawUniform64bit();

				/** You can call this overloaded method with either 32 or 64bit unsigned ints for the sake of general coding. */
				void drawUniformUnsignedInt(uint32_t &ret_number) { ret_number=drawUniform32bit(); }
				void drawUniformUnsignedInt(uint64_t &ret_number) { ret_number=drawUniform64bit(); }

				/** Return a uniform unsigned integer in the range [min_val,max_val] (both inclusive) */
				template <typename T, typename U,typename V>
				void drawUniformUnsignedIntRange(T &ret_number,const U min_val,const V max_val)
				{
					const T range = max_val-min_val+1;
					T rnd;
					drawUniformUnsignedInt(rnd);
					ret_number=min_val+ (rnd%range);
				}

				/** Generate a uniformly distributed pseudo-random number using the MT19937 algorithm, scaled to the selected range. */
				double drawUniform( const double Min, const double Max) {
					return Min + (Max-Min)* drawUniform32bit() * 2.3283064370807973754314699618685e-10; // 0xFFFFFFFF ^ -1
				}

				/** Fills the given matrix with independent, uniformly distributed samples.
				  * Matrix classes can be mrpt::math::CMatrixTemplateNumeric or mrpt::math::CMatrixFixedNumeric
				  * \sa drawUniform
  				  */
				template <class MAT>
				void drawUniformMatrix(
					MAT &matrix,
					const  double unif_min = 0,
					const  double unif_max = 1 )
				{
					for (size_t r=0;r<matrix.getRowCount();r++)
						for (size_t c=0;c<matrix.getColCount();c++)
							matrix.get_unsafe(r,c) = static_cast<typename MAT::Scalar>( drawUniform(unif_min,unif_max) );
				}

				/** Fills the given vector with independent, uniformly distributed samples.
				  * \sa drawUniform
  				  */
				template <class VEC>
				void drawUniformVector(
					VEC & v,
					const  double unif_min = 0,
					const  double unif_max = 1 )
				{
					const size_t N = v.size();
					for (size_t c=0;c<N;c++)
						v[c] = static_cast<typename mrpt::math::ContainerType<VEC>::element_t>( drawUniform(unif_min,unif_max) );
				}

			/** @} */

			/** @name Normal/Gaussian pdf
			 @{ */

				/** Generate a normalized (mean=0, std=1) normally distributed sample.
				 *  \param likelihood If desired, pass a pointer to a double which will receive the likelihood of the given sample to have been obtained, that is, the value of the normal pdf at the sample value.
				 */
				double drawGaussian1D_normalized( double *likelihood = NULL);

				/** Generate a normally distributed pseudo-random number.
				 * \param mean The mean value of desired normal distribution
				 * \param std  The standard deviation value of desired normal distribution
				 */
				double drawGaussian1D( const double mean, const double std ) {
					return mean+std*drawGaussian1D_normalized();
				}

				/** Fills the given matrix with independent, 1D-normally distributed samples.
				  * Matrix classes can be mrpt::math::CMatrixTemplateNumeric or mrpt::math::CMatrixFixedNumeric
				  * \sa drawGaussian1D
  				  */
				template <class MAT>
				void drawGaussian1DMatrix(
					MAT &matrix,
					const double mean = 0,
					const double std = 1 )
				{
					for (size_t r=0;r<matrix.getRowCount();r++)
						for (size_t c=0;c<matrix.getColCount();c++)
							matrix.get_unsafe(r,c) = static_cast<typename MAT::Scalar>( drawGaussian1D(mean,std) );
				}

				/** Generates a random definite-positive matrix of the given size, using the formula C = v*v^t + epsilon*I, with "v" being a vector of gaussian random samples.
				  */
				mrpt::math::CMatrixDouble drawDefinitePositiveMatrix(const size_t dim, const double std_scale = 1.0, const double diagonal_epsilon = 1e-8);

				/** Fills the given vector with independent, 1D-normally distributed samples.
				  * \sa drawGaussian1D
  				  */
				template <class VEC>
				void drawGaussian1DVector(
					VEC & v,
					const double mean = 0,
					const double std = 1 )
				{
					const size_t N = v.size();
					for (size_t c=0;c<N;c++)
						v[c] = static_cast<typename mrpt::math::ContainerType<VEC>::element_t>( drawGaussian1D(mean,std) );
				}

				/** Generate multidimensional random samples according to a given covariance matrix.
				 *  Mean is assumed to be zero if mean==NULL.
				 * \exception std::exception On invalid covariance matrix
				 * \sa drawGaussianMultivariateMany
				 */
				 template <typename T>
				 void  drawGaussianMultivariate(
					std::vector<T>		&out_result,
					const mrpt::math::CMatrixTemplateNumeric<T>	&cov,
					const std::vector<T>*  mean = NULL
					);


				/** Generate multidimensional random samples according to a given covariance matrix.
				 *  Mean is assumed to be zero if mean==NULL.
				 * \exception std::exception On invalid covariance matrix
				 * \sa drawGaussianMultivariateMany
				 */
				 template <class VECTORLIKE,class COVMATRIX>
				 void  drawGaussianMultivariate(
					VECTORLIKE	&out_result,
					const COVMATRIX &cov,
					const VECTORLIKE* mean = NULL
					)
				{
					const size_t N = cov.rows();
					ASSERT_(cov.rows()==cov.cols())
					if (mean) ASSERT_EQUAL_(size_t(mean->size()),N)

					// Compute eigenvalues/eigenvectors of cov:
					Eigen::SelfAdjointEigenSolver<typename COVMATRIX::PlainObject> eigensolver(cov);

					typename Eigen::SelfAdjointEigenSolver<typename COVMATRIX::PlainObject>::MatrixType eigVecs = eigensolver.eigenvectors();
					typename Eigen::SelfAdjointEigenSolver<typename COVMATRIX::PlainObject>::RealVectorType eigVals = eigensolver.eigenvalues();

					// Scale eigenvectors with eigenvalues:
					// D.Sqrt(); Z = Z * D; (for each column)
					eigVals = eigVals.array().sqrt();
					for (typename COVMATRIX::Index i=0;i<eigVecs.cols();i++)
						eigVecs.col(i) *= eigVals[i];

					// Set size of output vector:
					out_result.assign(N,0);

					for (size_t i=0;i<N;i++)
					{
						typename COVMATRIX::Scalar rnd = drawGaussian1D_normalized();
						for (size_t d=0;d<N;d++)
							out_result[d]+= eigVecs.coeff(d,i) * rnd;
					}
					if (mean)
						for (size_t d=0;d<N;d++)
							out_result[d]+= (*mean)[d];
				}

				/** Generate a given number of multidimensional random samples according to a given covariance matrix.
				 * \param cov The covariance matrix where to draw the samples from.
				 * \param desiredSamples The number of samples to generate.
				 * \param ret The output list of samples
				 * \param mean The mean, or zeros if mean==NULL.
				 */
				 template <typename VECTOR_OF_VECTORS,typename COVMATRIX>
				 void  drawGaussianMultivariateMany(
					VECTOR_OF_VECTORS	&ret,
					size_t               desiredSamples,
					const COVMATRIX     &cov,
					const typename VECTOR_OF_VECTORS::value_type *mean = NULL )
				{
					ASSERT_EQUAL_(cov.cols(),cov.rows())
					if (mean) ASSERT_EQUAL_(size_t(mean->size()),size_t(cov.cols()))

					// Compute eigenvalues/eigenvectors of cov:
					Eigen::SelfAdjointEigenSolver<typename COVMATRIX::PlainObject> eigensolver(cov);

					typename Eigen::SelfAdjointEigenSolver<typename COVMATRIX::PlainObject>::MatrixType eigVecs = eigensolver.eigenvectors();
					typename Eigen::SelfAdjointEigenSolver<typename COVMATRIX::PlainObject>::RealVectorType eigVals = eigensolver.eigenvalues();

					// Scale eigenvectors with eigenvalues:
					// D.Sqrt(); Z = Z * D; (for each column)
					eigVals = eigVals.array().sqrt();
					for (typename COVMATRIX::Index i=0;i<eigVecs.cols();i++)
						eigVecs.col(i) *= eigVals[i];

					// Set size of output vector:
					ret.resize(desiredSamples);
					const size_t N = cov.cols();
					for (size_t k=0;k<desiredSamples;k++)
					{
						ret[k].assign(N,0);
						for (size_t i=0;i<N;i++)
						{
							typename COVMATRIX::Scalar rnd = drawGaussian1D_normalized();
							for (size_t d=0;d<N;d++)
								ret[k][d]+= eigVecs.coeff(d,i) * rnd;
						}
						if (mean)
							for (size_t d=0;d<N;d++)
								ret[k][d]+= (*mean)[d];
					}
				}


			/** @} */


			/** @name Miscellaneous
			 @{ */

				/** Returns a random permutation of a vector: all the elements of the input vector are in the output but at random positions.
				  */
				template <class VEC>
				void  permuteVector(const VEC &in_vector, VEC &out_result)
				{
					out_result = in_vector;
					const size_t N = out_result.size();
					if (N>1)
						std::random_shuffle( &out_result[0],&out_result[N-1] );
				}

			/** @} */

		}; // end of CRandomGenerator --------------------------------------------------------------


		/** A static instance of a CRandomGenerator class, for use in single-thread applications */
		extern BASE_IMPEXP CRandomGenerator randomGenerator;


		/** A random number generator for usage in STL algorithms expecting a function like this (eg, random_shuffle):
		  */
		inline ptrdiff_t random_generator_for_STL(ptrdiff_t i)
		{
			return randomGenerator.drawUniform32bit() % i;
		}

		/** Fills the given matrix with independent, uniformly distributed samples.
		  * Matrix classes can be mrpt::math::CMatrixTemplateNumeric or mrpt::math::CMatrixFixedNumeric
		  * \sa matrixRandomNormal
		  */
		template <class MAT>
		void matrixRandomUni(
			MAT &matrix,
			const  double unif_min = 0,
			const  double unif_max = 1 )
		{
			for (size_t r=0;r<matrix.getRowCount();r++)
				for (size_t c=0;c<matrix.getColCount();c++)
					matrix.get_unsafe(r,c) = static_cast<typename MAT::Scalar>( randomGenerator.drawUniform(unif_min,unif_max) );
		}

		/** Fills the given matrix with independent, uniformly distributed samples.
		  * \sa vectorRandomNormal
		  */
		template <class T>
		void vectorRandomUni(
			std::vector<T> &v_out,
			const  T& unif_min = 0,
			const  T& unif_max = 1 )
		{
			size_t n = v_out.size();
			for (size_t r=0;r<n;r++)
				v_out[r] = randomGenerator.drawUniform(unif_min,unif_max);
		}

		/** Fills the given matrix with independent, normally distributed samples.
		  * Matrix classes can be mrpt::math::CMatrixTemplateNumeric or mrpt::math::CMatrixFixedNumeric
		  * \sa matrixRandomUni
		  */
		template <class MAT>
		void matrixRandomNormal(
			MAT &matrix,
			const double mean = 0,
			const double std = 1 )
		{
			for (size_t r=0;r<matrix.getRowCount();r++)
				for (size_t c=0;c<matrix.getColCount();c++)
					matrix.get_unsafe(r,c) = static_cast<typename MAT::Scalar>( mean + std*randomGenerator.drawGaussian1D_normalized() );
		}

		/** Generates a random vector with independent, normally distributed samples.
		  * \sa matrixRandomUni
		  */
		template <class T>
		void vectorRandomNormal(
			std::vector<T> &v_out,
			const  T& mean = 0,
			const  T& std = 1 )
		{
			size_t n = v_out.size();
			for (size_t r=0;r<n;r++)
				v_out[r] = mean + std*randomGenerator.drawGaussian1D_normalized();
		}

		/** Randomize the generators.
		 *   A seed can be providen, or a current-time based seed can be used (default)
		 */
		inline void Randomize(const uint32_t seed)  {
			randomGenerator.randomize(seed);
		}
		inline void Randomize()  {
			randomGenerator.randomize();
		}

		/** Returns a random permutation of a vector: all the elements of the input vector are in the output but at random positions.
		  */
		template <class T>
		void  randomPermutation(
			const std::vector<T> &in_vector,
			std::vector<T>       &out_result)
		{
			randomGenerator.permuteVector(in_vector,out_result);
		}


		/** Generate multidimensional random samples according to a given covariance matrix.
		 * \exception std::exception On invalid covariance matrix
		 * \sa randomNormalMultiDimensionalMany
		 */
		template <typename T>
		void  randomNormalMultiDimensional(
			const mrpt::math::CMatrixTemplateNumeric<T>	&cov,
			std::vector<T>		&out_result)
		 {
			randomGenerator.drawGaussianMultivariate(out_result,cov);
		 }

		 /** Generate a given number of multidimensional random samples according to a given covariance matrix.
		 * \param cov The covariance matrix where to draw the samples from.
		 * \param desiredSamples The number of samples to generate.
		 * \param samplesLikelihoods If desired, set to a valid pointer to a vector, where it will be stored the likelihoods of having obtained each sample: the product of the gaussian-pdf for each independent variable.
		 * \param ret The output list of samples
		 *
		 * \exception std::exception On invalid covariance matrix
		 *
		 * \sa randomNormalMultiDimensional
		 */
		 template <typename T>
		 void  randomNormalMultiDimensionalMany(
			const mrpt::math::CMatrixTemplateNumeric<T>	&cov,
			size_t							desiredSamples,
			std::vector< std::vector<T> >	&ret,
			std::vector<T>					*samplesLikelihoods = NULL)
		{
			randomGenerator.drawGaussianMultivariateMany(ret,desiredSamples,cov,static_cast<const std::vector<T>*>(NULL),samplesLikelihoods);
		}

		/** Generate multidimensional random samples according to a given covariance matrix.
		 * \exception std::exception On invalid covariance matrix
		 * \sa randomNormalMultiDimensional
		 */
		 template <typename T,typename MATRIXLIKE>
		 void  randomNormalMultiDimensionalMany(
			const MATRIXLIKE &cov,
			size_t           desiredSamples,
			std::vector< std::vector<T> >	&ret )
		 {
			 randomGenerator.drawGaussianMultivariateMany(ret,desiredSamples,cov);
		 }

		/** Generate multidimensional random samples according to a given covariance matrix.
		 * \exception std::exception On invalid covariance matrix
		 * \sa randomNormalMultiDimensionalMany
		 */
		 template <typename T,typename MATRIXLIKE>
		 void  randomNormalMultiDimensional(
			const MATRIXLIKE &cov,
			std::vector<T>   &out_result)
		{
			randomGenerator.drawGaussianMultivariate(out_result,cov);
		}


	}// End of namespace

} // End of namespace

#endif
