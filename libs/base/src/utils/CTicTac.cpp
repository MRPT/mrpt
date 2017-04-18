/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/config.h>

#ifdef MRPT_OS_WINDOWS
    #include <windows.h>
    #include <mrpt/utils/utils_defs.h>
#else
    #include <sys/time.h>
#endif

#include <mrpt/utils/CTicTac.h>
#include <cstring>

// For Windows: get the common code out of CTicTac so it's only run once!
#ifdef MRPT_OS_WINDOWS
struct AuxWindowsTicTac
{
	static AuxWindowsTicTac & GetInstance()
	{
		static AuxWindowsTicTac obj;
		return obj;
	}

	double dbl_period;

private:
	LARGE_INTEGER m_freq;

	AuxWindowsTicTac() : dbl_period(.0)
	{
		QueryPerformanceFrequency(&m_freq);
		ASSERTMSG_(m_freq.QuadPart != 0, "Error getting QueryPerformanceFrequency()");
		dbl_period = 1.0 / static_cast<double>(m_freq.QuadPart);
	}
};

#endif

using namespace mrpt::utils;

// Macros for easy access to memory with the correct types:
#ifdef MRPT_OS_WINDOWS
#	define	LARGE_INTEGER_NUMS	reinterpret_cast<LARGE_INTEGER*>(largeInts)
#else
#	define	TIMEVAL_NUMS			reinterpret_cast<struct timeval*>(largeInts)
#endif

/*---------------------------------------------------------------
						Constructor
 ---------------------------------------------------------------*/
CTicTac::CTicTac()
{
	::memset( largeInts, 0, sizeof(largeInts) );

#ifdef MRPT_OS_WINDOWS
	static_assert( sizeof( largeInts ) >= 2*sizeof(LARGE_INTEGER), "sizeof(LARGE_INTEGER) failed!");
#else
	static_assert( sizeof( largeInts ) > 2*sizeof(struct timeval), "sizeof(struct timeval) failed!");
#endif
	Tic();
}

/*---------------------------------------------------------------
						Tic
	Starts the stopwatch
 ---------------------------------------------------------------*/
void	CTicTac::Tic()
{
#ifdef MRPT_OS_WINDOWS
	LARGE_INTEGER *l= LARGE_INTEGER_NUMS;
	QueryPerformanceCounter(&l[0]);
#else
	struct timeval* ts = TIMEVAL_NUMS;
	gettimeofday( &ts[0], NULL);
#endif
}

/*---------------------------------------------------------------
						Tac
   Stop. Returns ellapsed time in seconds
 ---------------------------------------------------------------*/
double	CTicTac::Tac()
{
#ifdef MRPT_OS_WINDOWS
	LARGE_INTEGER *l= LARGE_INTEGER_NUMS;
	QueryPerformanceCounter( &l[1] );
	return (l[1].QuadPart-l[0].QuadPart) * AuxWindowsTicTac::GetInstance().dbl_period;
#else
	struct timeval* ts = TIMEVAL_NUMS;
	gettimeofday( &ts[1], NULL);
	return ( ts[1].tv_sec - ts[0].tv_sec) + 1e-6*(  ts[1].tv_usec - ts[0].tv_usec );
#endif
}
