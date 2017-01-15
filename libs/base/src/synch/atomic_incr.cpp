/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/synch/atomic_incr.h>

using namespace mrpt::synch;

#ifdef MRPT_OS_WINDOWS
	#include <windows.h>
#elif defined( __clang__ ) && defined( MRPT_OS_APPLE )
	//no include
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

