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
#include <cassert>

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
	memset( largeInts, 0, sizeof(largeInts) );

#ifdef MRPT_OS_WINDOWS
	ASSERT_( sizeof( largeInts ) > 3*sizeof(LARGE_INTEGER) );
	LARGE_INTEGER *l= LARGE_INTEGER_NUMS;
	QueryPerformanceFrequency(&l[0]);
#else
	assert( sizeof( largeInts ) > 2*sizeof(struct timeval) );
#endif
	Tic();
}

/*---------------------------------------------------------------
						Destructor
 ---------------------------------------------------------------*/
CTicTac::~CTicTac()
{
}

/*---------------------------------------------------------------
						Tic
	Starts the stopwatch
 ---------------------------------------------------------------*/
void	CTicTac::Tic()
{
#ifdef MRPT_OS_WINDOWS
	LARGE_INTEGER *l= LARGE_INTEGER_NUMS;
	QueryPerformanceCounter(&l[1]);
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
	QueryPerformanceCounter( &l[2] );
	return (l[2].QuadPart-l[1].QuadPart)/static_cast<double>(l[0].QuadPart);
#else
	struct timeval* ts = TIMEVAL_NUMS;
    gettimeofday( &ts[1], NULL);

    return ( ts[1].tv_sec - ts[0].tv_sec) +
           1e-6*(  ts[1].tv_usec - ts[0].tv_usec );
#endif
}
