/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/synch/CSemaphore.h>

using namespace mrpt;
using namespace mrpt::synch;

CSemaphore::CSemaphore(
	unsigned int    initialCount,
	unsigned int    maxCount)
{
	m_count = initialCount;
}

bool CSemaphore::waitForSignal( unsigned int timeout_ms )
{
	MRPT_START

	std::unique_lock<decltype(m_mutex)> lck(m_mutex);
	while (m_count==0)
	{
		if (std::cv_status::timeout==m_condition.wait_for(lck, std::chrono::milliseconds(timeout_ms)))
			return false;
	}
	--m_count;

	return true;

	MRPT_END
}

void CSemaphore::release(unsigned int increaseCount )
{
	MRPT_START

	std::unique_lock<decltype(m_mutex)> lck(m_mutex);
	++m_count;
	m_condition.notify_one();

	MRPT_END
}
