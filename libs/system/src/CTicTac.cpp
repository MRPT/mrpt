/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "system-precomp.h"  // Precompiled headers

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

#include <mrpt/core/exceptions.h>
#include <mrpt/system/CTicTac.h>
#include <cassert>
#include <cstring>

#include <type_traits>

// For Windows: get the common code out of CTicTac so it's only run once!
#ifdef _WIN32
struct AuxWindowsTicTac
{
	static AuxWindowsTicTac& GetInstance() noexcept
	{
		static AuxWindowsTicTac obj;
		return obj;
	}

	double dbl_period;

   private:
	LARGE_INTEGER m_freq;

	AuxWindowsTicTac() noexcept : dbl_period(.0)
	{
		QueryPerformanceFrequency(&m_freq);
		assert(m_freq.QuadPart != 0);
		dbl_period = 1.0 / static_cast<double>(m_freq.QuadPart);
	}
};

#endif

using namespace mrpt::system;

// Macros for easy access to memory with the correct types:
#ifdef _WIN32
#define LARGE_INTEGER_NUMS reinterpret_cast<LARGE_INTEGER*>(largeInts)
#else
#define TIMEVAL_NUMS reinterpret_cast<struct timeval*>(largeInts)
#endif

CTicTac::CTicTac() noexcept
{
	::memset(largeInts, 0, sizeof(largeInts));

#ifdef _WIN32
	static_assert(
		sizeof(largeInts) >= 2 * sizeof(LARGE_INTEGER),
		"sizeof(LARGE_INTEGER) failed!");
#else
	static_assert(
		sizeof(largeInts) >= 2 * sizeof(struct timeval),
		"sizeof(struct timeval) failed!");
#endif
	Tic();
}

void CTicTac::Tic() noexcept
{
#ifdef _WIN32
	LARGE_INTEGER* l = LARGE_INTEGER_NUMS;
	QueryPerformanceCounter(&l[0]);
#else
	auto* ts = TIMEVAL_NUMS;
	gettimeofday(&ts[0], nullptr);
#endif
}

double CTicTac::Tac() noexcept
{
#ifdef _WIN32
	LARGE_INTEGER* l = LARGE_INTEGER_NUMS;
	QueryPerformanceCounter(&l[1]);
	return (l[1].QuadPart - l[0].QuadPart) *
		   AuxWindowsTicTac::GetInstance().dbl_period;
#else
	auto* ts = TIMEVAL_NUMS;
	gettimeofday(&ts[1], nullptr);
	return static_cast<double>(ts[1].tv_sec - ts[0].tv_sec) +
		   1e-6 * static_cast<double>(ts[1].tv_usec - ts[0].tv_usec);
#endif
}
