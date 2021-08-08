/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "random-precomp.h"	 // Precompiled headers
//
#include <mrpt/random/RandomGenerators.h>

using namespace mrpt::random;

// The global instance of CRandomGenerator for single-thread programs:
static thread_local CRandomGenerator randomGenerator;

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4146)
#endif

// Code from the implementation by Rick Wagner
//  http://www-personal.umich.edu/~wagnerr/MersenneTwister.html
inline uint32_t hiBit(const uint32_t u) { return u & 0x80000000UL; }
inline uint32_t loBit(const uint32_t u) { return u & 0x00000001UL; }
inline uint32_t loBits(const uint32_t u) { return u & 0x7fffffffUL; }
inline uint32_t mixBits(const uint32_t u, const uint32_t v)
{
	return hiBit(u) | loBits(v);
}
inline uint32_t twist(const uint32_t m, const uint32_t s0, const uint32_t s1)
{
	return m ^ (mixBits(s0, s1) >> 1) ^ (-loBit(s1) & 0x9908b0dfUL);
}

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

Generator_MT19937::result_type Generator_MT19937::operator()()
{
	if (!m_index) generateNumbers();

	uint32_t y = m_MT[m_index];
	y ^= y >> 11;
	y ^= (y << 7) & 2636928640U;  // 0x9d2c5680
	y ^= (y << 15) & 4022730752U;  // 0xefc60000
	y ^= (y >> 18);

	// Wrap index to [0,623].
	m_index++;
	if (m_index >= 624) m_index = 0;

	return y;
}

void Generator_MT19937::generateNumbers()
{
	if (!m_seed_initialized) this->seed(std::random_device{}());

	// Code from the implementation by Rick Wagner
	//  http://www-personal.umich.edu/~wagnerr/MersenneTwister.html
	// and:
	//  http://www.math.sci.hiroshima-u.ac.jp/~m-mat/MT/MT2002/CODES/mt19937ar.c
	//
	const int N = 624;	// length of state vector
	const int M = 397;	// period parameter

	uint32_t* p = m_MT;
	for (int i = N - M; i--; ++p)
		*p = twist(p[M], p[0], p[1]);
	for (int i = M; --i; ++p)
		*p = twist(p[M - N], p[0], p[1]);
	*p = twist(p[M - N], p[0], m_MT[0]);
}

void Generator_MT19937::seed(const uint32_t seed)
{
	m_seed_initialized = true;
	m_MT[0] = seed;
	// 0x6c078965
	for (uint32_t i = 1; i < 624; i++)
		m_MT[i] = static_cast<uint32_t>(
			1812433253 * (m_MT[i - 1] ^ (m_MT[i - 1] >> 30)) + i);

	m_index = 0;
}

CRandomGenerator& mrpt::random::getRandomGenerator() { return randomGenerator; }
uint64_t CRandomGenerator::drawUniform64bit() { return m_uint64(m_MT19937); }
uint32_t CRandomGenerator::drawUniform32bit() { return m_uint32(m_MT19937); }
void CRandomGenerator::randomize(const uint32_t seed) { m_MT19937.seed(seed); }

void CRandomGenerator::randomize() { m_MT19937.seed(std::random_device{}()); }

double CRandomGenerator::drawGaussian1D_normalized()
{
	return m_normdistribution(m_MT19937);
}
