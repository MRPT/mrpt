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
#ifndef  mrpt_synch_atomicincr_H
#define  mrpt_synch_atomicincr_H

// This file CANNOT include "utils_defs.h" to avoid a double include:
#include <mrpt/config.h>
#include <mrpt/base/link_pragmas.h>  // DLL import/export definitions

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
#ifdef MRPT_OS_WINDOWS
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
