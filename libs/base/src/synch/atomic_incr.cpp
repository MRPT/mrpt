/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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


