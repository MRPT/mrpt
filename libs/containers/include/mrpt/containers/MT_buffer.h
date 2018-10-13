/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <vector>
#include <cstdint>
#include <thread>
#include <mutex>

namespace mrpt::containers
{
/** This class is a bulk sequence of bytes with MultiThread (MT)-safe read and
 * write operations.
 * \ingroup stlext_grp
 */
class MT_buffer
{
   private:
	std::vector<uint8_t> m_data;
	std::mutex m_cs;

   public:
	/** Default constructor */
	MT_buffer() = default;
	/** Destructor */
	virtual ~MT_buffer() = default;
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
	void appendData(const std::vector<uint8_t>& d)
	{
		m_cs.lock();
		m_data.insert(m_data.begin(), d.begin(), d.end());
		m_cs.unlock();
	}

	/** Read the whole buffer and empty it. */
	void readAndClear(std::vector<uint8_t>& d)
	{
		m_cs.lock();
		d.clear();
		m_data.swap(d);
		m_cs.unlock();
	}

	/** Read the whole buffer. */
	void read(std::vector<uint8_t>& d)
	{
		m_cs.lock();
		d = m_data;
		m_cs.unlock();
	}

};  // end of MT_buffer

}  // namespace mrpt::containers
