/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef  mrpt_synch_mt_buffer_H
#define  mrpt_synch_mt_buffer_H

#include <mrpt/utils/types_simple.h>

#include <thread>
#include <mutex>

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
	std::mutex  m_cs;

public:
	MT_buffer()  //!< Default constructor
	{}
	virtual ~MT_buffer()  //!< Destructor
	{}

	void clear()  //!< Empty the buffer
	{
		m_cs.lock();
		m_data.clear();
		m_cs.unlock();
	}

	size_t size()  //!< Return the number of available bytes at this moment.
	{
		size_t s;
		m_cs.lock();
		s = m_data.size();
		m_cs.unlock();
		return s;
	}

	void appendData(const vector_byte &d)  //!< Append new data to the stream
	{
		m_cs.lock();
		m_data.insert( m_data.begin(), d.begin(),d.end() );
		m_cs.unlock();
	}

	void readAndClear(vector_byte &d)  //!< Read the whole buffer and empty it.
	{
		m_cs.lock();
		d.clear();
		m_data.swap(d);
		m_cs.unlock();
	}

	void read(vector_byte &d)  //!< Read the whole buffer.
	{
		m_cs.lock();
		d = m_data;
		m_cs.unlock();
	}

}; // end of MT_buffer


} // End of namespace
} // End of namespace

#endif
