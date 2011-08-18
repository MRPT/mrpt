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
