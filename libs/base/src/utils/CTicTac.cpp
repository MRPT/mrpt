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
