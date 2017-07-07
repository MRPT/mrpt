/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef  internal_class_registry_H
#define  internal_class_registry_H

#include <mrpt/utils/CThreadSafeQueue.h>
#include <atomic>

namespace mrpt
{
	namespace utils
	{
		typedef void (*TRegisterFunction)(); // A void(void) function

		// Use a queue for the pending register issues, but also an atomic counter, which is much faster to check than a CS.
		std::atomic<int>	BASE_IMPEXP &           pending_class_registers_count();
		CThreadSafeQueue<TRegisterFunction> BASE_IMPEXP &   pending_class_registers();
		/** Set to true if pending_class_registers() has been called after registerAllPendingClasses(). Startup value is false. */
		extern volatile bool BASE_IMPEXP                           pending_class_registers_modified; 

	} // End of namespace
} // End of namespace

#endif
