/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <cstdlib>
#include <map>
#include <mutex>
#include <thread>

namespace mrpt::containers
{
/** Creates an instance of the data type T per requesting thread.
 *
 * \ingroup mrpt_containers_grp
 */
template <class T>
class PerThreadDataHolder
{
   public:
	PerThreadDataHolder() = default;

	T& get()
	{
		m_dataMtx.lock();
		T& d = m_data[std::this_thread::get_id()];
		m_dataMtx.unlock();
		return d;
	}

	void clear()
	{
		m_dataMtx.lock();
		m_data.clear();
		m_dataMtx.unlock();
	}

   private:
	std::map<std::thread::id, T> m_data;
	std::mutex m_dataMtx;
};

}  // namespace mrpt::containers
