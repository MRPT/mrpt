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
#ifndef  mrpt_synch_threadsafevar_H
#define  mrpt_synch_threadsafevar_H

#include <mrpt/synch/CCriticalSection.h>

namespace mrpt
{
namespace synch
{

	/** A template for created thread-safe variables with an internal critical section controlled each read or write.
	  * Example:
	  *  \code
	  *    CThreadSafeVariable<double>   var1;
	  *	   ...
	  *    var.set(2.3);    // Sets the value
	  *    double x = var.get();  // Reads the variable
	  *    ...
	  *    double foo = var;  // Also reads the variable
	  *    var = 2.3;       // ERROR: Not allowed, use ".set()" instead.
	  *  \endcode
	  *
	  * \sa CCriticalSection
	  * \ingroup synch_grp
	  */
	template <typename T>
	class CThreadSafeVariable
	{
	private:
		CCriticalSection  m_cs;
		T	m_val;
	public:
		CThreadSafeVariable() : m_cs(), m_val()  {  }
		CThreadSafeVariable(const T& init_val) : m_cs(), m_val(init_val)  {  }

		virtual ~CThreadSafeVariable() { }

		/** Return a copy of the hold variable */
		T get() const
		{
			T ret;
			{
				CCriticalSectionLocker l(&m_cs);
				ret = m_val;
			}
			return ret;
		}

		/** Return a copy of the hold variable */
		void get(T &out_val) const
		{
			CCriticalSectionLocker l(&m_cs);
			out_val = m_val;
		}

		/** Return a copy of the hold variable */
		operator T(void) const
		{
			CCriticalSectionLocker l(&m_cs);
			return m_val;
		}

		/** Return a copy of the hold variable */
		void set(const T &new_val)
		{
			CCriticalSectionLocker l(&m_cs);
			m_val = new_val;
		}

		/** Swap the current value of the hold variable and the passed one, as one atomic operation. */
		void swap(T &in_out_var)
		{
			CCriticalSectionLocker l(&m_cs);
			std::swap(in_out_var,m_val);
		}
	};

} // End of namespace
} // End of namespace

#endif
