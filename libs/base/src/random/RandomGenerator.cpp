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
	m_MT19937.seed(seed);
}

uint64_t CRandomGenerator::drawUniform64bit()
{
	return m_uint64(m_MT19937);
}

// MT19937 algorithm
// http://en.wikipedia.org/wiki/Mersenne_twister
uint32_t CRandomGenerator::drawUniform32bit()
{
	return m_uint32(m_MT19937);
}

/*---------------------------------------------------------------
						Randomize
  ---------------------------------------------------------------*/
void CRandomGenerator::randomize(const uint32_t seed)
{
	MT19937_initializeGenerator(seed);
}

/*---------------------------------------------------------------
						Randomize
  ---------------------------------------------------------------*/
void CRandomGenerator::randomize()
{
	MT19937_initializeGenerator( static_cast<uint32_t>(mrpt::system::getCurrentTime()) );
}

/*---------------------------------------------------------------
					normalizedGaussian
  ---------------------------------------------------------------*/
double CRandomGenerator::drawGaussian1D_normalized()
{
	return m_normdistribution(m_MT19937);
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
		printf("\nEXCEPTION: Dumping variables for debugging:\n"); \
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

		double normalizedGaussian() {
			return randomGenerator.drawGaussian1D_normalized();
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
