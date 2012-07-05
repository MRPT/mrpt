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
#ifndef  mrpt_synch_mt_buffer_H
#define  mrpt_synch_mt_buffer_H

#include <mrpt/synch/CCriticalSection.h>

namespace mrpt
{
namespace synch
{

/** This class is a bulk sequence of bytes with MultiThread (MT)-safe read and write operations.
  * \ingroup synch_grp
  */
class MT_buffer
{
private:
	vector_byte       m_data;
	CCriticalSection  m_cs;

public:
	MT_buffer()  //!< Default constructor
	{}
	virtual ~MT_buffer()  //!< Destructor
	{}

	void clear()  //!< Empty the buffer
	{
		m_cs.enter();
		m_data.clear();
		m_cs.leave();
	}

	size_t size()  //!< Return the number of available bytes at this moment.
	{
		size_t s;
		m_cs.enter();
		s = m_data.size();
		m_cs.leave();
		return s;
	}

	void appendData(const vector_byte &d)  //!< Append new data to the stream
	{
		m_cs.enter();
		m_data.insert( m_data.begin(), d.begin(),d.end() );
		m_cs.leave();
	}

	void readAndClear(vector_byte &d)  //!< Read the whole buffer and empty it.
	{
		m_cs.enter();
		d.clear();
		m_data.swap(d);
		m_cs.leave();
	}

	void read(vector_byte &d)  //!< Read the whole buffer.
	{
		m_cs.enter();
		d = m_data;
		m_cs.leave();
	}

}; // end of MT_buffer


} // End of namespace
} // End of namespace

#endif
