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

#include <mrpt/synch/atomic_incr.h>

using namespace mrpt::synch;
using namespace std;

#ifdef MRPT_OS_WINDOWS
	#include <windows.h>
#elif defined( __GNUC__ )
	#if ( __GNUC__ * 100 + __GNUC_MINOR__ >= 402 )
	#  include <ext/atomicity.h>
	#else
	#  include <bits/atomicity.h>
	#endif
#else
	#error This is not Windows and compiler is not GCC: Non implemented atomic_incr for this case
#endif


#ifdef MRPT_OS_WINDOWS
	void CAtomicCounter::operator++()
	{
		InterlockedIncrement(&m_value);
	}

	CAtomicCounter::atomic_num_t CAtomicCounter::operator--()
	{
		return InterlockedDecrement(&m_value);
	}

	CAtomicCounter::operator CAtomicCounter::atomic_num_t() const
	{
		return static_cast<long const volatile &>( m_value );
	}

#else
	// This should be GCC:
	#if ( defined( __i386__ ) || defined( __x86_64__ ) )
		void CAtomicCounter::operator++()
		{
			__asm__
			(
				"lock\n\t"
				"incl %0":
				"+m"( m_value ): // output (%0)
				: // inputs
				"cc" // clobbers
			);
		}

		int my_atomic_exchange_and_add( int * pw, int dv )
		{
			// int r = *pw;
			// *pw += dv;
			// return r;
			int r;
			__asm__ __volatile__
			(
				"lock\n\t"
				"xadd %1, %0":
				"+m"( *pw ), "=r"( r ): // outputs (%0, %1)
				"1"( dv ): // inputs (%2 == %1)
				"memory", "cc" // clobbers
			);
			return r;
		}

		CAtomicCounter::atomic_num_t CAtomicCounter::operator--()
		{
			return my_atomic_exchange_and_add( &m_value, -1 ) - 1;
		}

		CAtomicCounter::operator CAtomicCounter::atomic_num_t() const
		{
			return my_atomic_exchange_and_add( &m_value, 0 );
		}

	#else
		#if defined(__GLIBCXX__) // g++ 3.4+
		using __gnu_cxx::__atomic_add;
		using __gnu_cxx::__exchange_and_add;
		#endif

		void CAtomicCounter::operator++()
		{
			__atomic_add(&m_value, 1);
		}

		CAtomicCounter::atomic_num_t CAtomicCounter::operator--()
		{
			return __exchange_and_add(&m_value, -1) - 1;
		}

		CAtomicCounter::operator CAtomicCounter::atomic_num_t() const
		{
			return __exchange_and_add(&m_value, 0);
		}
	#endif
#endif


