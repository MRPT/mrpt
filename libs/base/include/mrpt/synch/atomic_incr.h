/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  mrpt_synch_atomicincr_H
#define  mrpt_synch_atomicincr_H

#include <mrpt/config.h>
#include <mrpt/utils/compiler_fixes.h>
#include <mrpt/base/link_pragmas.h>  // DLL import/export definitions

#if defined( __clang__ ) && defined( MRPT_OS_APPLE )
   //no include
#elif defined( __GNUC__ )
#if ( __GNUC__ * 100 + __GNUC_MINOR__ >= 402 )
#  include <ext/atomicity.h>
#else
#  include <bits/atomicity.h>
#endif
#endif


namespace mrpt
{
namespace synch
{

/** This class acts exactly as an int (or long) variable, but with atomic increment and decrement operators.
  *   This is a useful component of thread-safe smart pointers.
  * \note Based on code from the Boost library.
  * \ingroup synch_grp
  */
class BASE_IMPEXP CAtomicCounter
{
public:
#if defined( __GNUC__ ) && !defined( MRPT_OS_APPLE )
	typedef _Atomic_word atomic_num_t;
#elif defined(_WIN32) // mostly for MSVC in Windows
	typedef long atomic_num_t;
#else
	typedef int atomic_num_t;
#endif

	explicit CAtomicCounter( long v ): m_value( static_cast<atomic_num_t>(v) )
	{ }

	void operator++();  //!< Atomic increment of value.
	atomic_num_t operator--(); 	//!< Atomic decrement of value and return new value.
	operator atomic_num_t() const; //!< Get current value

private:
	mutable atomic_num_t m_value;

	CAtomicCounter( CAtomicCounter const & );	//!< Forbidden method
	CAtomicCounter & operator=( CAtomicCounter const & ); 	//!< Forbidden method
}; // end of CAtomicCounter


} // End of namespace
} // End of namespace

#endif
