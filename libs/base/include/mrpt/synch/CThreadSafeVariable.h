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
