/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#include <mrpt/base.h>  // Precompiled headers



#include <mrpt/config.h>

#ifdef MRPT_OS_WINDOWS
    #include <windows.h>
#else
    #include <sys/time.h>
#endif

#include <mrpt/utils/CTicTac.h>
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
	ASSERT_( sizeof( largeInts ) > 2*sizeof(struct timeval) );
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
