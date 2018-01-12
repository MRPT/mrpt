/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

// Wrappers for platform-specific thread/synchronization objects:
// * Thread
// * Mutex  (Critical Section)
// * Events (Conditional Variables)

#ifdef WIN32
// stick to the Windows scheme for now, but this structure could be easily
// replaced by pthread for portability; however if real-time priority turns
// out to be a requirement on that platform, the pthread implementation may
// not have support for such scheduling.
// for a much more lightweight run-time, this could be replaced by dummy
// objects, provided that the library client is careful enough to avoid any
// sort of race-conditions or dead-locks...
#include "libusbemu_threads_win32.h"
#else
#error LIBUSBEMU PTHREAD WRAPPER NOT YET IMPLEMENTED!
// #include "libusbemu_threads_pthread.h"
#endif

namespace libusbemu
{
struct RAIIMutex
{
	QuickMutex& m_mutex;
	RAIIMutex(QuickMutex& mutex) : m_mutex(mutex) { m_mutex.Enter(); }
	~RAIIMutex() { m_mutex.Leave(); }
};
}
