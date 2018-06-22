/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  MRPT_SYSTEM_THREADS_H
#define  MRPT_SYSTEM_THREADS_H

#include <mrpt/utils/core_defs.h>
#include <thread>

namespace mrpt
{
	namespace system
	{
		/** \addtogroup mrpt_thread Threads (in #include <mrpt/system/threads.h>)
		  *  \ingroup mrpt_base_grp
		  * @{ */

		/** This structure contains the information needed to interface the threads API on each platform:
		  * \sa createThread
		  */
		struct TThreadHandle
		{
			std::thread m_thread;

			/** Mark the handle as invalid.
			  * \sa isClear
			  */
			void clear()
			{
				if (m_thread.joinable())
					m_thread.detach();
				m_thread = std::thread();
			}
			/** Returns true if the handle is uninitialized */
			bool isClear() const { return !m_thread.joinable(); }
		};

		/**  The type for cross-platform process (application) priorities.
		  * \sa changeCurrentProcessPriority
		  */
		enum TProcessPriority {
			ppIdle = 0,
			ppNormal,
			ppHigh,
			ppVeryHigh
		};

		/**  The type for cross-platform thread priorities.
		  * \sa changeThreadPriority
		  */
		enum TThreadPriority {
			tpLowests =-15,	// Win32: THREAD_PRIORITY_IDLE
			tpLower = -2,	// Win32: THREAD_PRIORITY_LOWEST
			tpLow = -1,		// Win32: THREAD_PRIORITY_BELOW_NORMAL
			tpNormal = 0,   // Win32: THREAD_PRIORITY_NORMAL
			tpHigh = 1, 	// Win32: THREAD_PRIORITY_ABOVE_NORMAL
			tpHigher = 2, 	// Win32: THREAD_PRIORITY_HIGHEST
			tpHighest = 15	// Win32: THREAD_PRIORITY_TIME_CRITICAL
		};

		/** Creates a new thread from a function (or static method) with one generic parameter.
		  *  This function creates, and starts a new thread running some code given by a function.
		  *  The thread function should end by returning as normal.
		  * \param func The function with the code to run in the thread.
		  * \param param The parameter to be passed to the new thread function.
		  * \return A structure that represents the thread (it contains its ID and, in Windows, its HANDLE).
		  * \exception std::exception If the operation fails
		  * \sa createThreadFromObjectMethod, joinThread, changeThreadPriority
		  */
		template<typename T> inline TThreadHandle createThread(void (*func)(T),T param)	{
			TThreadHandle h;
			h.m_thread = std::thread(func, param);
			return h;
		}
		//! \overload
		template<typename T> inline TThreadHandle createThreadRef(void (*func)(T&),T& param)	{
			TThreadHandle h;
			h.m_thread = std::thread(func, std::ref<T>(param));
			return h;
		}
		//! \overload
		inline TThreadHandle createThread(void (*func)(void))	{
			TThreadHandle h;
			h.m_thread = std::thread(func);
			return h;
		}

		/** Creates a new thread running a non-static method (so it will have access to "this") from another method of the same class - with one generic parameter.
		  *  This function creates, and starts a new thread running some code given by a function.
		  *  The thread function should end by returning as normal.
		  *  Example of usage:
		  *
		  *  \code
		  *    class MyClass {
		  *    public:
		  *      void myThread(int n);
		  *      void someMethod() {
		  *         createThreadFromObjectMethod(this, &MyClass::myThread, 123 );
		  *         ....
		  *      }
		  *    };
		  *  \endcode
		  *
		  * \param func The function with the code to run in the thread.
		  * \param param The parameter to be passed to the new thread function.
		  * \return A structure that represents the thread (it contains its ID and, in Windows, its HANDLE).
		  * \exception std::exception If the operation fails
		  * \sa createThread, joinThread, changeThreadPriority
		  */
		template <typename CLASS,typename PARAM>
		inline TThreadHandle createThreadFromObjectMethod(CLASS *obj, void (CLASS::*func)(PARAM), PARAM param)	{
			TThreadHandle h;
			h.m_thread = std::thread(func, obj,param);
			return h;
		}
		//! \overload
		template <typename CLASS,typename PARAM>
		inline TThreadHandle createThreadFromObjectMethodRef(CLASS *obj, void (CLASS::*func)(PARAM&), PARAM &param)	{
			TThreadHandle h;
			h.m_thread = std::thread(func, obj, param);
			return h;
		}
		//! \overload
		template <typename CLASS>
		inline TThreadHandle createThreadFromObjectMethod(CLASS *obj, void (CLASS::*func)(void))	{
			TThreadHandle h;
			h.m_thread = std::thread(func, obj);
			return h;
		}


		/** Waits until the given thread ends.
		  * \sa createThread
		  */
		void BASE_IMPEXP joinThread( TThreadHandle &threadHandle );

		/** Returns the ID of the current thread.
		  */
		unsigned long BASE_IMPEXP getCurrentThreadId() MRPT_NO_THROWS;

		/** Explicit close of the current (running) thread.
		  *  Do not use normally, it's better just to return from the running thread function.
		  * \sa createThread
		  */
		void BASE_IMPEXP exitThread() MRPT_NO_THROWS;

			/** Returns the creation and exit times of the current thread and its CPU time consumed.
			  * \param creationTime The creation time of the thread.
			  * \param exitTime The exit time of the thread, or undefined if it is still running.
			  * \param cpuTime The CPU time consumed by the thread, in seconds.
		  * \exception std::exception If the operation fails
		  * \sa getCurrentThreadId, createThread
		  */
			void BASE_IMPEXP getCurrentThreadTimes(
				time_t			&creationTime,
				time_t			&exitTime,
				double			&cpuTime );

		/** Change the priority of the given thread - for Windows, see also changeCurrentProcessPriority()
		  * - Windows: This is equivalent to [SetThreadPriority()](https://msdn.microsoft.com/en-us/library/windows/desktop/ms686277(v=vs.85).aspx) (read the docs there)
		  * - Linux (pthreads): May require `root` permissions! This sets the Round Robin scheduler with the given priority level. Read [sched_setscheduler](http://linux.die.net/man/2/sched_setscheduler).
		  * \sa createThread, changeCurrentProcessPriority
		  */
		void BASE_IMPEXP changeThreadPriority( TThreadHandle &threadHandle, TThreadPriority priority );

		/** Terminate a thread, giving it no choice to delete objects, etc (use only as a last resource) */
		void BASE_IMPEXP terminateThread( TThreadHandle &threadHandle) MRPT_NO_THROWS;

		/** Change the priority of the given process (it applies to all the threads, plus independent modifiers for each thread).
		  *  - Windows: See [SetPriorityClass](https://msdn.microsoft.com/es-es/library/windows/desktop/ms686219(v=vs.85).aspx)
		  *  - Linux (pthreads): Requires `root` permissions to increase process priority! Internally it calls [nice()](http://linux.die.net/man/3/nice), so it has no effect if changeThreadPriority() was called and a SCHED_RR is already active.
		  * \sa createThread, changeThreadPriority
		  */
		void BASE_IMPEXP changeCurrentProcessPriority( TProcessPriority priority );

		/** Return the number of processors ("cores"), or 1 if it cannot be determined.
		  */
		unsigned int BASE_IMPEXP getNumberOfProcessors();

		/** An OS-independent method for sending the current thread to "sleep" for a given period of time.
		  * \param time_ms The sleep period, in miliseconds.
		  */
		void BASE_IMPEXP sleep( int time_ms ) MRPT_NO_THROWS;

		/** Executes the given command (which may contain a program + arguments), and waits until it finishes.
		  * \return false on any error, true otherwise
		  */
		bool BASE_IMPEXP  launchProcess( const std::string & command );

		/**  @} */

	} // End of namespace

} // End of namespace

#endif
