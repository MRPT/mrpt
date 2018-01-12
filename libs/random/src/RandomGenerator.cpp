/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "random-precomp.h"  // Precompiled headers

#include <mrpt/random/RandomGenerators.h>

using namespace mrpt::random;

// The global instance of CRandomGenerator for single-thread programs:
static CRandomGenerator randomGenerator;

CRandomGenerator& mrpt::random::getRandomGenerator() { return randomGenerator; }
// MT19937 algorithm
// http://en.wikipedia.org/wiki/Mersenne_twister
// Initialize the generator from a seed
void CRandomGenerator::MT19937_initializeGenerator(const uint32_t& seed)
{
	m_MT19937.seed(seed);
}

uint64_t CRandomGenerator::drawUniform64bit() { return m_uint64(m_MT19937); }
// MT19937 algorithm
// http://en.wikipedia.org/wiki/Mersenne_twister
uint32_t CRandomGenerator::drawUniform32bit() { return m_uint32(m_MT19937); }
void CRandomGenerator::randomize(const uint32_t seed)
{
	MT19937_initializeGenerator(seed);
}

void CRandomGenerator::randomize()
{
	MT19937_initializeGenerator(std::random_device{}());
}

double CRandomGenerator::drawGaussian1D_normalized()
{
	return m_normdistribution(m_MT19937);
}
