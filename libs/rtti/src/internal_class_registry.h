/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <atomic>
#include <mutex>
#include <queue>

namespace mrpt::rtti
{
using TRegisterFunction = void (*)();  // A void(void) function

struct queue_register_functions_t
{
	std::queue<TRegisterFunction> funcs;
	mutable std::mutex funcs_cs;

	/** Retrieve the next message in the queue, or nullptr if there is no
	 * message. The user MUST call "delete" with the returned object after use.
	 */
	inline bool get(TRegisterFunction& ret)
	{
		std::lock_guard<std::mutex> lock(funcs_cs);
		if (funcs.empty())
			return false;
		else
		{
			ret = funcs.front();
			funcs.pop();
			return true;
		}
	}
};

// Use a queue for the pending register issues, but also an atomic counter,
// which is much faster to check than a CS.
std::atomic<int>& pending_class_registers_count();
queue_register_functions_t& pending_class_registers();
/** Set to true if pending_class_registers() has been called after
 * registerAllPendingClasses(). Startup value is false. */
extern bool pending_class_registers_modified;

}  // namespace mrpt::rtti
