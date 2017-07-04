/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef mrpt_synch_mt_buffer_H
#define mrpt_synch_mt_buffer_H

#include <mrpt/utils/types_simple.h>

#include <thread>
#include <mutex>

namespace mrpt
{
namespace synch
{
/** This class is a bulk sequence of bytes with MultiThread (MT)-safe read and
 * write operations.
  * \ingroup synch_grp
  */
class MT_buffer
{
   private:
	vector_byte m_data;
	std::mutex m_cs;

   public:
	/** Default constructor */
	MT_buffer() {}
	/** Destructor */
	virtual ~MT_buffer() {}
	/** Empty the buffer */
	void clear()
	{
		m_cs.lock();
		m_data.clear();
		m_cs.unlock();
	}

	/** Return the number of available bytes at this moment. */
	size_t size()
	{
		size_t s;
		m_cs.lock();
		s = m_data.size();
		m_cs.unlock();
		return s;
	}

	/** Append new data to the stream */
	void appendData(const vector_byte& d)
	{
		m_cs.lock();
		m_data.insert(m_data.begin(), d.begin(), d.end());
		m_cs.unlock();
	}

	/** Read the whole buffer and empty it. */
	void readAndClear(vector_byte& d)
	{
		m_cs.lock();
		d.clear();
		m_data.swap(d);
		m_cs.unlock();
	}

	/** Read the whole buffer. */
	void read(vector_byte& d)
	{
		m_cs.lock();
		d = m_data;
		m_cs.unlock();
	}

};  // end of MT_buffer

}  // End of namespace
}  // End of namespace

#endif
