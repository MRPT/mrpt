/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  mrpt_synch_mt_buffer_H
#define  mrpt_synch_mt_buffer_H

#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/utils/types_simple.h>

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
