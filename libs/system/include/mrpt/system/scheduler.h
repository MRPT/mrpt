/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

namespace mrpt::system
{
/** \addtogroup mrpt_scheduler Scheduler helpers
 * (in #include <mrpt/system/scheduler.h>)
 *  \ingroup mrpt_base_grp
 * @{ */

/**  The type for cross-platform process (application) priorities.
 * \sa changeCurrentProcessPriority
 */
enum TProcessPriority
{
	ppIdle = 0,
	ppNormal,
	ppHigh,
	ppVeryHigh
};

/**  The type for cross-platform thread priorities.
 * \sa changeThreadPriority
 */
enum TThreadPriority
{
	tpLowests = -15,  //!< Win32: THREAD_PRIORITY_IDLE
	tpLower = -2,  //!< Win32: THREAD_PRIORITY_LOWEST
	tpLow = -1,  //!< Win32: THREAD_PRIORITY_BELOW_NORMAL
	tpNormal = 0,  //!< Win32: THREAD_PRIORITY_NORMAL
	tpHigh = 1,  //!< Win32: THREAD_PRIORITY_ABOVE_NORMAL
	tpHigher = 2,  //!< Win32: THREAD_PRIORITY_HIGHEST
	tpHighest = 15  //!< Win32: THREAD_PRIORITY_TIME_CRITICAL
};

/** Change the priority of the current thread - for Windows, see also
 * changeCurrentProcessPriority()
 * - Windows: This is equivalent to
 * [SetThreadPriority()](https://msdn.microsoft.com/en-us/library/windows/desktop/ms686277(v=vs.85).aspx)
 * (read the docs there)
 * - Linux (pthreads): May require `root` permissions! This sets the Round Robin
 * scheduler with the given priority level. Read
 * [sched_setscheduler](http://linux.die.net/man/2/sched_setscheduler). \sa
 * createThread, changeCurrentProcessPriority, changeCurrentThreadPriority
 */
void changeCurrentThreadPriority(TThreadPriority priority);

/** Change the priority of the given process (it applies to all the threads,
  plus independent modifiers for each thread).
  *  - Windows: See
  [SetPriorityClass](https://msdn.microsoft.com/es-es/library/windows/desktop/ms686219(v=vs.85).aspx)
  *  - Linux (pthreads): Requires `root` permissions to increase process
  priority! Internally it calls [nice()](http://linux.die.net/man/3/nice), so it
  has no effect if
  () was called and a SCHED_RR is already active.
  * \sa createThread, changeThreadPriority
  */
void changeCurrentProcessPriority(TProcessPriority priority);

/**  @} */

}  // namespace mrpt::system
