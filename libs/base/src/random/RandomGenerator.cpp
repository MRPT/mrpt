/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/base.h>  // Precompiled headers



#include <mrpt/config.h>

#include <mrpt/math/CMatrix.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/system/os.h>

#include <mrpt/math/ops_matrices.h>

using namespace mrpt;
using namespace mrpt::random;
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
uint32_t hiBit( const uint32_t& u ) { return u & 0x80000000UL; }
uint32_t loBit( const uint32_t& u ) { return u & 0x00000001UL; }
uint32_t loBits( const uint32_t& u ) { return u & 0x7fffffffUL; }
uint32_t mixBits( const uint32_t& u, const uint32_t& v ) { return hiBit(u) | loBits(v); }
uint32_t twist( const uint32_t& m, const uint32_t& s0, const uint32_t& s1 ) { return m ^ (mixBits(s0,s1)>>1) ^ (-loBit(s1) & 0x9908b0dfUL); }

#if defined(_MSC_VER)
	#pragma warning(pop)
#endif


// Generate an array of 624 untempered numbers
void CRandomGenerator::MT19937_generateNumbers()
{
	if (!m_MT19937_data.seed_initialized)	this->randomize();

#if 0
	// My first code (slighly slower...)
	for (uint32_t i=0;i<624;i++)
	{
		uint32_t	y = (0x80000000 & m_MT19937_data.MT[i]) + (0x7FFFFFFF & m_MT19937_data.MT[(i+1) % 624]);
		m_MT19937_data.MT[i] = m_MT19937_data.MT[(i + 397) % 624] ^ (y >> 1);
		if ((y & 0x01)!=0) // Odd
			m_MT19937_data.MT[i] ^= 2567483615U; // 0x9908b0df
	}
#else
	// Code from the implementation by Rick Wagner
	//  http://www-personal.umich.edu/~wagnerr/MersenneTwister.html
	static const int N = 624;	// length of state vector
	static const int M = 397;	// period parameter

	register uint32_t *p = m_MT19937_data.MT;
	for( int i = N - M; i--; ++p )
		*p = twist( p[M], p[0], p[1] );
	for( int i = M; --i; ++p )
		*p = twist( p[M-N], p[0], p[1] );
	*p = twist( p[M-N], p[0], m_MT19937_data.MT[0] );
#endif
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
   static int iset = 0;
   static double gset;
   double fac,r,v1,v2,x;

   if (iset == 0) {   /* We don't have an extra deviate handy so */
       do {
	   v1 = this->drawUniform(-1.0,1.0);   /* pick two uniform numbers in */
	   v2 = this->drawUniform(-1.0,1.0);   /* square extending from -1 to +1*/
	                                 /* in each direction, */

	   r = v1 * v1 + v2 * v2;        /* see if they are in the unitcircle*/
       } while (r >= 1.0 && r!=0);               /* and if they are not, try again. */
       fac = sqrt(-2.0*log(r)/r);
       /* Now make the Box-Muller transformation to get two normal deviates.
	  Return one and save the other for next time. */
       gset = v1 * fac;
       iset = 1;

	   if (likelihood)
	   {
		   x = v2*fac;
		   *likelihood = 0.39894228040143267794 * exp( -0.5*x*x );
		   return x;
	   }
	   else
	   {
	       return v2*fac;
	   }

   } else
   {      /* We have an extra deviate handy, */
       iset = 0; /* so unset the flag,              */

	   if (likelihood)
	   {
		   x = (float)gset;
		   *likelihood = 0.39894228040143267794 * exp( -0.5*x*x );
		   return x;
	   }
	   else
	   {
	       return (float)gset;
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
	D.Sqrt();
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
