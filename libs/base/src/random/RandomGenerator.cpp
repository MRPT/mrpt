/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/random/RandomGenerators.h>
#include <mrpt/system/os.h>
#include <mrpt/system/datetime.h>

using namespace mrpt;
using namespace mrpt::random;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace std;

// The global instance of CRandomGenerator for single-thread programs:
CRandomGenerator mrpt::random::randomGenerator;

// MT19937 algorithm
// http://en.wikipedia.org/wiki/Mersenne_twister
// Initialize the generator from a seed
void CRandomGenerator::MT19937_initializeGenerator(const uint32_t &seed)
{
	m_MT19937_data.seed_initialized = true;
	m_MT19937_data.MT[0] = seed;
	for (uint32_t i=1;i<624;i++)
		m_MT19937_data.MT[i] = static_cast<uint32_t>( 1812433253 * (m_MT19937_data.MT[i-1] ^ ( m_MT19937_data.MT[i-1] >> 30 )) + i); // 0x6c078965
}

#if defined(_MSC_VER)
	#pragma warning(push)
	#pragma warning(disable:4146)
#endif

// Code from the implementation by Rick Wagner
//  http://www-personal.umich.edu/~wagnerr/MersenneTwister.html
inline uint32_t hiBit( const uint32_t u ) { return u & 0x80000000UL; }
inline uint32_t loBit( const uint32_t u ) { return u & 0x00000001UL; }
inline uint32_t loBits( const uint32_t u ) { return u & 0x7fffffffUL; }
inline uint32_t mixBits( const uint32_t u, const uint32_t v ) { return hiBit(u) | loBits(v); }
inline uint32_t twist( const uint32_t m, const uint32_t s0, const uint32_t s1 ) { return m ^ (mixBits(s0,s1)>>1) ^ (-loBit(s1) & 0x9908b0dfUL); }

#if defined(_MSC_VER)
	#pragma warning(pop)
#endif


// Generate an array of 624 untempered numbers
void CRandomGenerator::MT19937_generateNumbers()
{
	if (!m_MT19937_data.seed_initialized)	this->randomize();

	// Code from the implementation by Rick Wagner
	//  http://www-personal.umich.edu/~wagnerr/MersenneTwister.html
	// and:
	//  http://www.math.sci.hiroshima-u.ac.jp/~m-mat/MT/MT2002/CODES/mt19937ar.c
	//
	const int N = 624;	// length of state vector
	const int M = 397;	// period parameter

	uint32_t *p = m_MT19937_data.MT;
	for( int i = N - M; i--; ++p )
		*p = twist( p[M], p[0], p[1] );
	for( int i = M; --i; ++p )
		*p = twist( p[M-N], p[0], p[1] );
	*p = twist( p[M-N], p[0], m_MT19937_data.MT[0] );
}

uint64_t CRandomGenerator::drawUniform64bit()
{
	uint32_t n1 = drawUniform32bit();
	uint32_t n2 = drawUniform32bit();
	return static_cast<uint64_t>(n1) | (static_cast<uint64_t>(n2)<<32);
}

// MT19937 algorithm
// http://en.wikipedia.org/wiki/Mersenne_twister
uint32_t CRandomGenerator::drawUniform32bit()
{
	if (!m_MT19937_data.index)
		MT19937_generateNumbers();

	uint32_t	y = m_MT19937_data.MT[m_MT19937_data.index];
	y ^= y >> 11;
	y ^= (y << 7) & 2636928640U; // 0x9d2c5680
	y ^= (y << 15) & 4022730752U; // 0xefc60000
	y ^= (y >> 18);

	// Wrap index to [0,623].
	m_MT19937_data.index++;
	if (m_MT19937_data.index>=624) m_MT19937_data.index=0;

	return y;
}

/*---------------------------------------------------------------
						Randomize
  ---------------------------------------------------------------*/
void CRandomGenerator::randomize(const uint32_t seed)
{
	MT19937_initializeGenerator(seed);
	m_MT19937_data.index = 0;
}

/*---------------------------------------------------------------
						Randomize
  ---------------------------------------------------------------*/
void CRandomGenerator::randomize()
{
	MT19937_initializeGenerator( static_cast<uint32_t>(mrpt::system::getCurrentTime()) );
	m_MT19937_data.index = 0;
}

/*---------------------------------------------------------------
					normalizedGaussian
  This is a routine which has been extracted from page 217 in
      the Numerical Recipes in C, William H. Press
  ---------------------------------------------------------------*/
double CRandomGenerator::drawGaussian1D_normalized( double *likelihood  )
{
   if (!m_std_gauss_set)
   {   /* We don't have an extra deviate handy so */
	   double v1,v2,r;
       do
	   {
		   v1 = this->drawUniform(-1.0,1.0);   /* pick two uniform numbers in */
		   v2 = this->drawUniform(-1.0,1.0);   /* square extending from -1 to +1*/
										 /* in each direction, */
			r = v1 * v1 + v2 * v2;        /* see if they are in the unitcircle*/
       } while (r >= 1.0 || r==0.0);               /* and if they are not, try again. */
	   const double fac = std::sqrt(-2.0*log(r)/r);
       /* Now make the Box-Muller transformation to get two normal deviates.
	  Return one and save the other for next time. */
       m_std_gauss_next = v1 * fac;
       m_std_gauss_set = true;

	   if (likelihood)
	   {
		   const double x = v2*fac;
		   *likelihood = 0.39894228040143267794 * exp( -0.5*x*x );
		   return x;
	   }
	   else
	   {
	       return v2*fac;
	   }
   }
   else
   {      /* We have an extra deviate handy, */
       m_std_gauss_set = false; /* so unset the flag,              */

	   if (likelihood)
	   {
		   const double x = m_std_gauss_next;
		   *likelihood = 0.39894228040143267794 * exp( -0.5*x*x );
		   return x;
	   }
	   else
	   {
	       return m_std_gauss_next;
	   }
   }
}

/** Generates a random definite-positive matrix of the given size, using the formula C = v*v^t + epsilon*I, with "v" being a vector of gaussian random samples.
*/
CMatrixDouble CRandomGenerator::drawDefinitePositiveMatrix(
	const size_t dim,
	const double std_scale,
	const double diagonal_epsilon)
{
	CMatrixDouble   r(dim,1);
	drawGaussian1DMatrix(r, 0,std_scale);
	CMatrixDouble   cov(dim,dim);
	cov.multiply_AAt(r);  // random semi-definite positive matrix:
	for (size_t i=0;i<dim;i++) cov(i,i)+=diagonal_epsilon;  // make sure it's definite-positive
	return cov;
}


/*---------------------------------------------------------------
				drawGaussianMultivariate
  ---------------------------------------------------------------*/
template <typename T>
void  CRandomGenerator::drawGaussianMultivariate(
	std::vector<T>		&out_result,
	const CMatrixTemplateNumeric<T>	&cov,
	const std::vector<T>*  mean )
{
	ASSERT_(cov.getRowCount() == cov.getColCount() );
	const size_t	dim = cov.getColCount();

	if (mean) ASSERT_(mean->size()==dim)

	CMatrixTemplateNumeric<T>	Z,D;

	MRPT_START

	// Set size of output vector:
	out_result.clear();
	out_result.resize(dim,0);

	/** Computes the eigenvalues/eigenvector decomposition of this matrix,
	*    so that: M = Z · D · Z<sup>T</sup>, where columns in Z are the
	*	  eigenvectors and the diagonal matrix D contains the eigenvalues
	*    as diagonal elements, sorted in <i>ascending</i> order.
	*/
	cov.eigenVectors( Z, D );

	// Scale eigenvectors with eigenvalues:
	D = D.array().sqrt().matrix();
	Z.multiply(Z,D);

	for (size_t i=0;i<dim;i++)
	{
		T rnd = this->drawGaussian1D_normalized();
		for (size_t d=0;d<dim;d++)
			out_result[d]+= ( Z.get_unsafe(d,i)* rnd );
	}
	if (mean)
		for (size_t d=0;d<dim;d++) out_result[d]+= (*mean)[d];

	MRPT_END_WITH_CLEAN_UP( \
		printf("\nEXCEPTION: Dumping variables for debuging:\n"); \
		std::cout << "Z:\n" << Z << "D:\n" << D << "Cov:\n" << cov; \
		try \
		{ \
			cov.eigenVectors(Z,D); \
			std::cout << "Original Z:" << Z << "Original D:" << D; \
		} \
		catch(...)  {}; \
		);
}

// Instantiations:
template BASE_IMPEXP void  CRandomGenerator::drawGaussianMultivariate<double>(std::vector<double> &out_result,const CMatrixTemplateNumeric<double> &cov, const std::vector<double>* mean );
template BASE_IMPEXP void  CRandomGenerator::drawGaussianMultivariate<float>(std::vector<float> &out_result,const CMatrixTemplateNumeric<float> &cov, const std::vector<float>* mean );





namespace mrpt
{

	namespace random
	{

		double normalizedGaussian( double *likelihood ) {
			return randomGenerator.drawGaussian1D_normalized(likelihood);
		}

		double RandomNormal( double mean , double std )  {
			return randomGenerator.drawGaussian1D(mean,std);
		}

		uint32_t RandomUniInt()  {
			return randomGenerator.drawUniform32bit();
		}

		double RandomUni( const double min, const double max)
		{
			return min + (max-min)* randomGenerator.drawUniform32bit() * 2.3283064370807973754314699618685e-10; // 0xFFFFFFFF ^ -1
		}

	}
} // end of namespace
