
//  Copyright (c) 2003-2019 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#ifndef XSENS_MUTEX_H
#define XSENS_MUTEX_H

#include <cstdint>
#include <atomic>
#include <xstypes/xsthread.h>
#include <xstypes/xstimestamp.h>

#include <deque>

#ifndef __GNUC__
#pragma warning(disable: 4127)
#endif

namespace xsens {
	class Lock;
	class LockReadWrite;
	class LockSuspendable;
	class WaitCondition;
	enum LockState { LS_Unlocked, LS_Read, LS_Write, LS_SuspendedWrite };

	/*! \class Mutex
		\brief A base mutex class
	*/
	class Mutex {
	private:
#if !XSENS_USE_POSIX_LOCKING
		CRITICAL_SECTION m_mutex;
#else
		pthread_mutexattr_t m_attr;
		pthread_mutex_t m_mutex;
#endif
		XsThreadId m_lockedBy;
		volatile std::atomic_int m_lockCount;
		friend class Lock;
		friend class WaitCondition;
	public:
		Mutex()
			: m_lockedBy(0)
			, m_lockCount(0)
		{
			#if !XSENS_USE_POSIX_LOCKING
				::InitializeCriticalSection(&m_mutex);
			#else
				// this is required to get the same behaviour we have in windows
				pthread_mutexattr_init(&m_attr);
				pthread_mutexattr_settype(&m_attr, PTHREAD_MUTEX_RECURSIVE);
				pthread_mutex_init(&m_mutex, &m_attr);
			#endif
		}
		~Mutex()
		{
			assert(m_lockCount == 0);
			assert(m_lockedBy == 0);
			#if !XSENS_USE_POSIX_LOCKING
				::DeleteCriticalSection(&m_mutex);
			#else
				pthread_mutex_destroy(&m_mutex);
				pthread_mutexattr_destroy(&m_attr);
			#endif
		}
		Mutex(Mutex const&) = delete;
		Mutex& operator=(Mutex const&) = delete;
		Mutex(Mutex &&) = delete;
		Mutex& operator=(Mutex &&) = delete;

		/*! \brief Claims (locks) a mutex
			\returns True if successful
		*/
		inline bool claimMutex()
		{
		#if !XSENS_USE_POSIX_LOCKING
			::EnterCriticalSection(&m_mutex);
		#else
			pthread_mutex_lock(&m_mutex);
		#endif
			++m_lockCount;
			m_lockedBy = xsGetCurrentThreadId();
			return true;
		}

		/*! \brief Releases (unlocks) a mutex
			\returns True if successful
		*/
		inline bool releaseMutex()
		{
			if (!--m_lockCount)
				m_lockedBy = 0;
		#if !XSENS_USE_POSIX_LOCKING
			::LeaveCriticalSection(&m_mutex);
		#else
			pthread_mutex_unlock(&m_mutex);
		#endif
			return true;
		}

		/*! \brief Tries to claim (lock) a mutex
			\returns True if successful
		*/
		inline bool tryClaimMutex()
		{
			// *CriticalSection returns true on success, pthread_* returns 0 on success.
		#if !XSENS_USE_POSIX_LOCKING
			if (::TryEnterCriticalSection(&m_mutex))
		#else
			if (pthread_mutex_trylock(&m_mutex) == 0)
		#endif
			{
				++m_lockCount;
				m_lockedBy = xsGetCurrentThreadId();
				return true;
			}
			return false;
		}
		#ifdef XSENS_DEBUG
		// this function is only intended as a debug function for use in assert statements
		inline bool haveLock() const
		{
			return m_lockedBy == xsGetCurrentThreadId();
		}
		#endif

		/*! \returns True if a mutex is locked
		*/
		inline bool isLocked() const volatile
		{
			return m_lockCount > 0;
		}
	};


	/*! \class MutexReadWrite
		\brief A readers-writer mutex class.
		\details Any number of readers can claim the mutex at the same time, which blocks a writer from
		claiming it. If a writer claims the mutex, all subsequent read claims will be blocked until
		no more read claims exist and the write claim has been completed (released). Recursive read claims
		within a thread that already has a read or write claim for this mutex will be granted to prevent
		deadlocks.
	*/
	class MutexReadWrite {
	private:
		Mutex m_access;
		volatile std::atomic_int m_writeRef;
		volatile std::atomic<XsThreadId> m_writeLocked;
		volatile std::atomic_int m_writeLocksPending;
		XsThreadId* m_readLocked;

		int m_readLockMax;
		int m_readLockCount;
		friend class LockReadWrite;
		MutexReadWrite(MutexReadWrite const&) = delete;
		MutexReadWrite& operator=(MutexReadWrite const&) = delete;
		MutexReadWrite(MutexReadWrite &&) = delete;
		MutexReadWrite& operator=(MutexReadWrite &&) = delete;

	public:
		inline MutexReadWrite()
			: m_writeRef(0)
			, m_writeLocked(0)
			, m_writeLocksPending(0)
		{
			m_readLockMax = 32;
			m_readLocked = new XsThreadId[m_readLockMax];
			m_readLockCount = 0;
		}
		inline ~MutexReadWrite()
		{
			delete[] m_readLocked;
		}
	private:
		inline void addReadLock(XsThreadId cid)
		{
			if (m_readLockCount == m_readLockMax)
			{
				m_readLockMax = m_readLockMax*2;
				XsThreadId* tmp = new XsThreadId[m_readLockMax];
				memcpy(tmp, m_readLocked, ((unsigned int) m_readLockCount)*sizeof(XsThreadId));
				delete[] m_readLocked;
				m_readLocked = tmp;
			}
			m_readLocked[m_readLockCount++] = cid;
		}

		inline void enterAtomic()
		{
			m_access.claimMutex();
		}

		inline void leaveAtomic()
		{
			m_access.releaseMutex();
		}

		inline bool claimMutex(bool write)
		{
			XsThreadId cid = xsGetCurrentThreadId();
			// write lock
			if (write)
			{
				enterAtomic();
				++m_writeLocksPending;
				while(1)
				{
					bool readFail = false;
					if (m_writeLocked == 0)
					{
						for (int i = 0; i < m_readLockCount; ++i)
						{
							if (m_readLocked[i] != cid)
							{
								readFail = true;
								break;
							}
						}
					}

					if (m_writeLocked != cid && (m_writeLocked != 0 || readFail))
					{
						leaveAtomic();
						xsYield();	// wait for other locks to end
						enterAtomic();
					}
					else
					{
						++m_writeRef;
						--m_writeLocksPending;
						m_writeLocked = cid;
						leaveAtomic();
						return true;
					}
				}
			}
			// read lock
			else
			while(1)
			{
				enterAtomic();
				bool readGot = false;
				if ((m_writeLocked == 0 && m_writeLocksPending == 0) || m_writeLocked == cid)
					readGot = true;
				else
					for (int i = 0; i < m_readLockCount; ++i)
						if (m_readLocked[i] == cid)
						{
							readGot = true;
							break;
						}

				if (readGot)
				{
					addReadLock(cid);
					leaveAtomic();
					return true;
				}
				leaveAtomic();
				// wait for write lock to end
				xsYield();
			}
		}

		/*! \brief Downgrade an existing write lock to a read lock */
		inline bool downgradeToRead()
		{
			XsThreadId cid = xsGetCurrentThreadId();
			enterAtomic();
			if (m_writeLocked == cid)
			{
				if (--m_writeRef == 0)
					m_writeLocked = 0;
				addReadLock(cid);
				leaveAtomic();
				return true;
			}
			leaveAtomic();
			return false;
		}

		inline bool releaseMutex(bool write)
		{
			XsThreadId cid = xsGetCurrentThreadId();
			enterAtomic();
			if (write)
			{
				if (m_writeLocked == cid)
				{
					if (--m_writeRef == 0)
						m_writeLocked = 0;
				}
				else
				{
					leaveAtomic();
					return false;
				}
			}
			else
			{
				if (m_readLockCount == 0)
				{
					leaveAtomic();
					return false;
				}
				for (int i = m_readLockCount-1; i >= 0; --i)
					if (m_readLocked[i] == cid)
					{
						for (int j = i+1; j < m_readLockCount; ++j)
							m_readLocked[j-1] = m_readLocked[j];
						--m_readLockCount;
						break;
					}
			}
			leaveAtomic();
			return true;
		}

		inline bool tryClaimMutex(bool write, uint32_t timeout)
		{
			XsThreadId cid = xsGetCurrentThreadId();
			long long timeToEnd = XsTimeStamp_now(0) + timeout;

			// write lock
			if (write)
			{
				enterAtomic();
				++m_writeLocksPending;
				leaveAtomic();
				do
				{
					enterAtomic();
					bool readFail = false;
					if (m_writeLocked == 0)
					{
						for (int i = 0; i < m_readLockCount; ++i)
							if (m_readLocked[i] != cid)
							{
								readFail = true;
								break;
							}
					}

					if (m_writeLocked != cid && (m_writeLocked != 0 || readFail))
					{
						leaveAtomic();
						// wait for other locks to end
						xsYield();
					}
					else
					{
						++m_writeRef;
						--m_writeLocksPending;
						m_writeLocked = cid;
						leaveAtomic();
						return true;
					}
				} while(XsTimeStamp_now(0) >= timeToEnd);
				enterAtomic();
				--m_writeLocksPending;
				leaveAtomic();
				return false;
			}
			// read lock
			// else
			do {
				enterAtomic();

				bool readGot = false;
				if ((m_writeLocked == 0 && m_writeLocksPending == 0) || m_writeLocked == cid)
					readGot = true;
				else
				{
					for (int i = 0; i < m_readLockCount; ++i)
						if (m_readLocked[i] == cid)
						{
							readGot = true;
							break;
						}
				}

				if (readGot)
				{
					addReadLock(cid);
					leaveAtomic();
					return true;
				}
				leaveAtomic();
				// wait for write lock to end
				xsYield();
			} while (XsTimeStamp_now(0) <= timeToEnd);
			return false;
		}

#ifdef XSENS_DEBUG
		public:	// these functions should only be used internally and in assert statements
#endif
		inline bool haveWriteClaim() const
		{
			bool rv = false;
			XsThreadId cid = xsGetCurrentThreadId();

			const_cast<MutexReadWrite*>(this)->enterAtomic();
			if (m_writeLocked == cid)
				rv = true;
			const_cast<MutexReadWrite*>(this)->leaveAtomic();
			return rv;
		}
		// this function is only intended as a debug function for use in assert statements
		inline bool haveReadClaim() const
		{
			bool rv = false;
			XsThreadId cid = xsGetCurrentThreadId();

			const_cast<MutexReadWrite*>(this)->enterAtomic();
			if (m_writeLocked == cid)
				rv = true;
			else
			{
				for (int i = 0; i < m_readLockCount; ++i)
					if (m_readLocked[i] == cid)
					{
						rv = true;
						break;
					}
			}
			const_cast<MutexReadWrite*>(this)->leaveAtomic();
			return rv;
		}
	};

	/*! \brief An enum that hold mutex status flags
	*/
	enum MutexStatusFlags : int {
		MSF_HaveReadLock = 1,
		MSF_HaveWriteLock = 2,
		MSF_WriteLockSuspended = 4,
		MSF_WriteLocked = 8,
		MSF_ReadLocked = 16,
		MSF_WriteLocksPending = 32,
		MSF_GuardedLocked = 64,
	};

	/*! \class MutexReadWriteSuspendable
		\brief A readers-writer mutex class that is able to be suspended
	*/
	class MutexReadWriteSuspendable
	{
	private:
		Mutex m_access;
		volatile std::atomic_int m_writeRef;
		volatile std::atomic<XsThreadId> m_writeLocked;
		volatile std::atomic_int m_writeLocksPending;
		volatile std::atomic_int m_writeLockSuspendCount;
		XsThreadId* m_readLocked;

		int m_readLockMax;
		int m_readLockCount;
		friend class LockSuspendable;
		MutexReadWriteSuspendable(MutexReadWriteSuspendable const&) = delete;
		MutexReadWriteSuspendable& operator=(MutexReadWriteSuspendable const&) = delete;
		MutexReadWriteSuspendable(MutexReadWriteSuspendable &&) = delete;
		MutexReadWriteSuspendable& operator=(MutexReadWriteSuspendable &&) = delete;
	public:

		/*! \brief An enum class that holds a three different modes of a suspendable mutex
		*/
		enum class Mode
		{
			Read,
			Write,
			SuspendedWrite
		};

		inline MutexReadWriteSuspendable()
			: m_writeRef(0)
			, m_writeLocked(0)
			, m_writeLocksPending(0)
			, m_writeLockSuspendCount(0)
			, m_readLockMax(32)
			, m_readLockCount(0)
		{
			m_readLocked = new XsThreadId[m_readLockMax];
		}

		inline ~MutexReadWriteSuspendable()
		{
			delete[] m_readLocked;
		}
	private:
		inline void addReadLock(XsThreadId cid)
		{
			if (m_readLockCount == m_readLockMax)
			{
				m_readLockMax = m_readLockMax*2;
				XsThreadId* tmp = new XsThreadId[m_readLockMax];
				memcpy(tmp, m_readLocked, ((unsigned int)m_readLockCount)*sizeof(XsThreadId));
				delete[] m_readLocked;
				m_readLocked = tmp;
			}
			m_readLocked[m_readLockCount++] = cid;
		}

		inline void enterAtomic()
		{
			m_access.claimMutex();
		}

		inline void leaveAtomic()
		{
			m_access.releaseMutex();
		}

	protected:

		/*! \brief Claims (locks) a mutex using a given mode
			\param mode The mode to use
			\returns True if successful
		*/
		inline bool claimMutex(Mode mode)
		{
			XsThreadId cid = xsGetCurrentThreadId();
			// write lock
			switch (mode)
			{
			case Mode::Write:
				enterAtomic();
				++m_writeLocksPending;
				while(1)
				{
					bool readFail = false;
					if (m_writeLocked == 0 || (m_writeLockSuspendCount && m_writeLockSuspendCount == m_writeRef))
					{
						for (int i = 0; i < m_readLockCount; ++i)
						{
							if (m_readLocked[i] != cid)
							{
								readFail = true;
								break;
							}
						}
					}

					if ((m_writeLocked != cid && (m_writeLocked != 0 || readFail))
						|| (readFail && m_writeLockSuspendCount && m_writeLockSuspendCount == m_writeRef))
					{
						leaveAtomic();
						xsYield();	// wait for other locks to end
						enterAtomic();
					}
					else
					{
						++m_writeRef;
						--m_writeLocksPending;
						m_writeLocked = cid;
						leaveAtomic();
						return true;
					}
				}
				break;

			case Mode::Read:
				while (1)
				{
					enterAtomic();
					bool readGot = false;
					if ((m_writeLocked == 0 && m_writeLocksPending == 0)
						|| m_writeLocked == cid
						|| (m_writeLockSuspendCount && m_writeLockSuspendCount == m_writeRef))
					{
						readGot = true;
					}
					else
					{
						for (int i = 0; i < m_readLockCount; ++i)
						{
							if (m_readLocked[i] == cid)
							{
								readGot = true;
								break;
							}
						}
					}

					if (readGot)
					{
						addReadLock(cid);
						leaveAtomic();
						return true;
					}
					leaveAtomic();
					// wait for write lock to end
					xsYield();
				}
				break;

			case Mode::SuspendedWrite:
				enterAtomic();
				++m_writeLocksPending;
				while(1)
				{
					if (m_writeLocked != cid && m_writeLocked != 0)
					{
						leaveAtomic();
						xsYield();	// wait for other write locks to end
						enterAtomic();
					}
					else
					{
						++m_writeRef;
						--m_writeLocksPending;
						m_writeLocked = cid;
						++m_writeLockSuspendCount;
						leaveAtomic();
						return true;
					}
				}
				break;

			default:
				break;
			}

			assert(0);
			return false;
		}

		/*! \brief Claims (locks) a mutex for read or write modes
			\param write If set to true write mode is used, otherwise read mode
			\returns True if successful
			\note Made for backwards compatibility
		*/
		inline bool claimMutex(bool write)
		{
			return claimMutex(write ? Mode::Write : Mode::Read);
		}

		/*! \brief Downgrade an existing write lock to a read lock
		*/
		inline bool downgradeToRead(bool decreaseSuspend)
		{
			XsThreadId cid = xsGetCurrentThreadId();
			enterAtomic();
			if (m_writeLocked == cid)
			{
				if (decreaseSuspend)
					--m_writeLockSuspendCount;
				if (--m_writeRef == 0)
				{
					m_writeLocked = 0;
					m_writeLockSuspendCount = 0;
				}
				addReadLock(cid);
				leaveAtomic();
				return true;
			}
			leaveAtomic();
			return false;
		}

		/*! \brief Suspend an existing write lock so read locks are allowed, but do not allow other write locks
		*/
		inline bool suspendWriteLock()
		{
			XsThreadId cid = xsGetCurrentThreadId();
			enterAtomic();
			if (m_writeLocked == cid)
			{
				++m_writeLockSuspendCount;
				leaveAtomic();
				return true;
			}
			leaveAtomic();
			return false;
		}

		/*! \brief Resumes an existing write lock so read locks are no longer allowed
		*/
		inline bool resumeWriteLock()
		{
			XsThreadId cid = xsGetCurrentThreadId();
			enterAtomic();
			if (m_writeLocked == cid)
			{
				if (m_writeLockSuspendCount-- == m_writeRef)
				{
					// wait for all read locks to end
					while(1)
					{
						bool stillReading = false;
						for (int i = 0; i < m_readLockCount; ++i)
						{
							if (m_readLocked[i] != cid)
							{
								stillReading = true;
								break;
							}
						}

						if (!stillReading)
						{
							leaveAtomic();
							return true;
						}
						leaveAtomic();
						xsYield();
						enterAtomic();
					}
				}
				leaveAtomic();
				return true;
			}
			leaveAtomic();
			return false;
		}

		/*! \brief Releases (unlocks) a mutex for read or write modes
			\param write If set to true, write mode is used, otherwise read mode
			\param decreaseSuspendCount If set to true, it will decrease a suspend count
			\returns True if successful
		*/
		inline bool releaseMutex(bool write, bool decreaseSuspendCount)
		{
			XsThreadId cid = xsGetCurrentThreadId();
			enterAtomic();
			if (write)
			{
				if (m_writeLocked == cid)
				{
					if (decreaseSuspendCount)
						--m_writeLockSuspendCount;

					if (--m_writeRef == 0)
					{
						m_writeLockSuspendCount = 0;
						m_writeLocked = 0;
					}
				}
				else
				{
					leaveAtomic();
					return false;
				}
			}
			else
			{
				if (m_readLockCount == 0)
				{
					leaveAtomic();
					return false;
				}
				for (int i = m_readLockCount-1; i >= 0; --i)
					if (m_readLocked[i] == cid)
					{
						for (int j = i+1; j < m_readLockCount; ++j)
							m_readLocked[j-1] = m_readLocked[j];
						--m_readLockCount;
						break;
					}
			}
			leaveAtomic();
			return true;
		}

		/*! \brief Releases (unlocks) a mutex using a given mode
			\param mode The mode to use
			\param decreaseSuspendCount If set to true, it will decrease a suspend count
			\returns True if successful
		*/
		inline bool releaseMutex(Mode mode, bool decreaseSuspendCount)
		{
			return releaseMutex(mode != Mode::Read, decreaseSuspendCount);
		}
	public:

		/*! \returns The status of the current thread
			\sa MutexStatusFlags
		*/
		int status() const volatile
		{
			int rv = 0;
			XsThreadId cid = xsGetCurrentThreadId();
			const_cast<MutexReadWriteSuspendable*>(this)->enterAtomic();
			if (m_writeRef)
			{
				rv |= MSF_WriteLocked;
				if (m_writeLocked == cid)
					rv |= MSF_HaveWriteLock;
				if (m_writeLockSuspendCount == m_writeRef)
					rv |= MSF_WriteLockSuspended;
			}
			if (m_writeLocksPending)
				rv |= MSF_WriteLocksPending;
			if (m_readLockCount)
				rv |= MSF_ReadLocked;
			for (int i = 0; i < m_readLockCount; ++i)
			{
				if (m_readLocked[i] == cid)
				{
					rv |= MSF_HaveReadLock;
					break;
				}
			}
			const_cast<MutexReadWriteSuspendable*>(this)->leaveAtomic();
			return rv;
		}

#ifndef XSENS_DEBUG
		protected:
		// these functions should only be used internally and in assert statements
#endif
		/*! \returns True if is has an write lock
		*/
		inline bool haveWriteClaim() const volatile
		{
			return (status() & MSF_HaveWriteLock) != 0;
		}

		/*! \returns True if is has a read lock
		*/
		inline bool haveReadClaim() const volatile
		{
			return (status() & (MSF_HaveWriteLock | MSF_HaveReadLock)) != 0;
		}
	};

	/*! \class Lock
		\brief A base class for a ock
	*/
	class Lock {
	private:
		Mutex* m_mutex;
		bool m_locked;

		Lock(Lock const&) = delete;
		Lock& operator=(Lock const&) = delete;
		Lock(Lock &&) = delete;
		Lock& operator=(Lock &&) = delete;

	public:

		/*! \brief Constructs a lock a given mutex
			\param mutex The pointer to a mutex to lock
		*/
		inline Lock(Mutex* mutex) : m_mutex(mutex), m_locked(false)
		{
			m_locked = m_mutex->claimMutex();
		}

		/*! \brief Constructs a lock a given mutex
			\param mutex The pointer to a mutex to lock
			\param lockit If set to true then it will lock a mutex
		*/
		inline Lock(Mutex* mutex, bool lockit) : m_mutex(mutex), m_locked(false)
		{
			if (lockit)
				m_locked = m_mutex->claimMutex();
		}

		/*! \brief Constructs a lock a given mutex
			\param mutex The pointer to a mutex to lock
			\param lockit If is not set to an unlocked state then it will lock it
		*/
		inline Lock(Mutex* mutex, LockState lockit) : m_mutex(mutex), m_locked(false)
		{
			if (lockit != LS_Unlocked)
				m_locked = m_mutex->claimMutex();
		}
		inline ~Lock()
		{
			unlock();
		}

		/*!	\brief Locks the unlocked mutex
			\returns True if the mutex is successfully locked
		*/
		inline bool lock()
		{
			if (!m_locked)
				return (m_locked = m_mutex->claimMutex()) != 0;
			return true;
		}

		/*!	\brief Unlocks the locked mutex
			\returns True if the mutex is successfully unlocked
		*/
		inline bool unlock() noexcept
		{
			if (m_locked)
			{
#ifndef __GNUC__
	#pragma warning(suppress: 4706)
#endif
				return !(m_locked = !m_mutex->releaseMutex());
			}
			return true;
		}

		/*!	\brief Tries to lock the unlocked mutex
			\returns True if the mutex is successfully locked
		*/
		inline bool tryLock()
		{
			if (!m_locked)
				return (m_locked = m_mutex->tryClaimMutex());
			return true;
		}

		/*! \returns True if the mutex is locked*/
		inline bool isLocked() const
		{
			return m_locked;
		}
	};

	/*! \class LockReadWrite
		\brief A readers-writer lock class
	*/
	class LockReadWrite {
	private:
		MutexReadWrite* m_mutex;
		bool m_lockedR;
		bool m_lockedW;

		LockReadWrite(LockReadWrite const&) = delete;
		LockReadWrite& operator=(LockReadWrite const&) = delete;
		LockReadWrite(LockReadWrite &&) = delete;
		LockReadWrite& operator=(LockReadWrite &&) = delete;
	public:

		/*! \brief Consturctor
		*/
		inline LockReadWrite(MutexReadWrite* mutex, LockState lockState = LS_Unlocked) : m_mutex(mutex), m_lockedR(false), m_lockedW(false)
		{
			#if defined(XSENS_DEBUG)
				assert(m_mutex != NULL);
			#endif

			if (lockState == LS_Read)
				lock(false);
			else if (lockState == LS_Write)
				lock(true);
			else
				assert(lockState != LS_SuspendedWrite);
		}
		inline ~LockReadWrite()
		{
			unlock();
		}

		/*! \brief Make sure that the lock has exactly the given lock state
			\details The lock will be upgraded/downgraded as required, possibly unlocking in between
			\param write If set to true, write mode is used, otherwise read mode
			\returns True if successfully locked
		*/
		inline bool lock(bool write)
		{
			if (write)
			{
				if (m_lockedW)
					return true;
				if (m_lockedR)
					unlock();
				m_lockedW = m_mutex->claimMutex(true);
				assert(m_lockedW);
				return m_lockedW;
			}
			else
			{
				if (m_lockedW)
				{
					m_lockedR = m_mutex->downgradeToRead();
					m_lockedW = false;
					assert(m_lockedR);
				}
				if (!m_lockedR)
					m_lockedR = m_mutex->claimMutex(false);
				assert(m_lockedR);
				return m_lockedR;
			}
		}

		/*! \brief Convenience function that accepts the generic LockState, forwards to lock(bool)
			\param ls If set not to read lock state then it will lock the mutex
			\returns True if successfully locked
		*/
		inline bool lock(LockState ls)
		{
			assert(ls == LS_Read || ls == LS_Write);
			return lock(ls != LS_Read);
		}
		/*! \brief Convenience function for lock(false);
			\return True if successful
		*/
		inline bool lockRead() { return lock(false); }

		/*! \brief Convenience function for lock(true);
			\return True if successful
		*/
		inline bool lockWrite() { return lock(true); }

		/*! \brief Unlocks the write or read locked mutex
			\returns True if the mutex is successfully unlocked. False if it was not locked.
		*/
		inline bool unlock() noexcept
		{
			if (m_lockedW)
			{
				assert(!m_lockedR);
				m_lockedW = false;
				return m_mutex->releaseMutex(true);
			}
			if (m_lockedR)
			{
				m_lockedR = false;
				return m_mutex->releaseMutex(false);
			}
			return false;
		}

		/*!	\brief Tries to lock the write/read unlocked mutex before a given time runs out
			\param write If set to true, write mode is used, otherwise read mode
			\param timeout The timeout value in ms
			\returns True if the mutex is successfully locked
		*/
		inline bool tryLock(bool write, uint32_t timeout)
		{
			if (write)
			{
				if (m_lockedW)
					return true;
				if (m_lockedR)
					unlock();
				m_lockedW = m_mutex->tryClaimMutex(true, timeout);
				assert(m_lockedW);
				return m_lockedW;
			}
			else
			{
				if (m_lockedW)
				{
					m_lockedR = m_mutex->downgradeToRead();
					m_lockedW = false;
					assert(m_lockedR);
				}
				if (!m_lockedR)
					m_lockedR = m_mutex->tryClaimMutex(false, timeout);
				assert(m_lockedR);
				return m_lockedR;
			}
		}

		/*! \returns true if the lock has at least the given state
			\param write If set to true, write mode is used, otherwise read mode
		*/
		inline bool isLocked(bool write) const volatile
		{
			return (!write && m_lockedR) || m_lockedW;
		}

		/*! \returns True if the mutex has a write lock
			\note Do not use this function internally! The mutex may have a write claim, while the lock does not, but this function will return true.
		*/
		inline bool haveWriteClaim() const volatile
		{
			return m_mutex->haveWriteClaim();
		}
	};

	/*! \brief A two-layer mutex, typically used for status+data protection
		\details The class provides a main suspendable mutex, which can be used for all kinds of status updates and a
		deeper guarded mutex, which will enforce the main mux to be locked before allowing itself to be locked.

		\note The main purpose of this is for multi-threaded data handling in which the mian mux guards the state of the device,
		and the guarded mux guards data access.
	*/
	class GuardedMutex : private MutexReadWriteSuspendable
	{
	private:
		Mutex m_guarded;
		friend class LockGuarded;
		friend class LockSuspendable;

		GuardedMutex(GuardedMutex const&) = delete;
		GuardedMutex& operator=(GuardedMutex const&) = delete;
		GuardedMutex(GuardedMutex &&) = delete;
		GuardedMutex& operator=(GuardedMutex &&) = delete;

	public:

		/*! \brief Constructor
		*/
		inline GuardedMutex()
		{
		}

		/*! \brief Destructor
		*/
		inline ~GuardedMutex()
		{
		}

		/*! \returns true if the mutex is claimed successfully
		*/
		inline bool claimMutex()
		{
			MutexReadWriteSuspendable::claimMutex(Mode::Read);
			return m_guarded.claimMutex();
		}

		/*! \returns true if the mutex is released successfully
		*/
		inline bool releaseMutex()
		{
			m_guarded.releaseMutex();
			MutexReadWriteSuspendable::releaseMutex(false, false);
			return true;
		}

		/*! \returns The reference to this mutex
			\note Give access to the base class for directly checking the state of the mux
		*/
		MutexReadWriteSuspendable& suspendable() { return *this; }

		/*! \brief Check if the guarded mutex is equal to the supplied mutex
			\param mutex The supplied mutex
			\returns True if it is equal
		*/
		inline bool isUsing(Mutex const* mutex) const
		{
			return &m_guarded == mutex;
		}

		/*! \returns The status of this mutex*/
		int status() const volatile
		{
			return MutexReadWriteSuspendable::status() | (m_guarded.isLocked() ? MSF_GuardedLocked : 0);
		}

#ifdef XSENS_DEBUG
		// this function is only intended as a debug function for use in assert statements
		inline bool haveGuardedLock() const
		{
			return m_guarded.haveLock();
		}
#endif

#ifndef XSENS_DEBUG
		protected:
		// these functions should only be used internally and in assert statements
#endif
		using MutexReadWriteSuspendable::haveWriteClaim;
		using MutexReadWriteSuspendable::haveReadClaim;
	};

	/*! \class LockSuspendable
		\brief A readers-writer lock class that is able to be suspended
	*/
	class LockSuspendable {
	private:
		MutexReadWriteSuspendable* m_mutex;
		bool m_lockedR;
		bool m_lockedW;
		bool m_iSuspended;	// Read: I suspended the write lock, this does not mean that it actually IS suspended

		LockSuspendable(LockSuspendable const&) = delete;
		LockSuspendable& operator=(LockSuspendable const&) = delete;
		LockSuspendable(LockSuspendable &&) = delete;
		LockSuspendable& operator=(LockSuspendable &&) = delete;

	public:

		/*! \brief Constructs a lock using a suspendable readers-writer mutex
		*/
		inline LockSuspendable(MutexReadWriteSuspendable* mutex, LockState lockState) : m_mutex(mutex), m_lockedR(false), m_lockedW(false), m_iSuspended(false)
		{
			#if defined(XSENS_DEBUG)
				assert(m_mutex != NULL);
			#endif

			if (lockState != LS_Unlocked)
				lock(lockState);
		}

		/*! \brief Constructs a lock using a guarded mutex
		*/
		inline LockSuspendable(GuardedMutex* mutex, LockState lockState) : m_mutex(mutex), m_lockedR(false), m_lockedW(false), m_iSuspended(false)
		{
			#if defined(XSENS_DEBUG)
				assert(m_mutex != NULL);
			#endif

			if (lockState != LS_Unlocked)
				lock(lockState);
		}

		inline ~LockSuspendable()
		{
			unlock();
		}

		/*! \brief Make sure that the lock has exactly the given lock state
			\details The lock will be upgraded/downgraded as required, possibly unlocking in between
			\param desiredLockState If set not to read lock state then it will lock the mutex
			\returns True if successfully locked
		*/
		inline bool lock(LockState desiredLockState)
		{
			switch (desiredLockState)
			{
			case LS_Write:
				if (m_lockedW)
					return resume();
				if (m_lockedR)
					unlock();
				m_lockedW = m_mutex->claimMutex(MutexReadWriteSuspendable::Mode::Write);
				assert(m_lockedW);
				return m_lockedW;

			case LS_SuspendedWrite:
				if (m_lockedW)
					return suspend();
				if (m_lockedR)
					unlock();
				m_lockedW = m_mutex->claimMutex(MutexReadWriteSuspendable::Mode::SuspendedWrite);
				m_iSuspended = m_lockedW;
				assert(m_lockedW);
				return m_lockedW;

			case LS_Read:
				if (m_lockedW)
				{
					m_lockedR = m_mutex->downgradeToRead(m_iSuspended);
					m_iSuspended = false;
					m_lockedW = false;
					assert(m_lockedR);
				}
				if (!m_lockedR)
					m_lockedR = m_mutex->claimMutex(MutexReadWriteSuspendable::Mode::Read);
				assert(m_lockedR);
				return m_lockedR;

			case LS_Unlocked:
				return unlock();

			default:
				assert(0);
				return false;
			}
		}

		/*! \brief Convenience function that accepts the boolean write value, forwards to lock(LockState)
			\param write If set to true, write mode is used, otherwise read mode
			\returns True if successfully locked
		*/
		inline bool lock(bool write)
		{
			return lock(write ? LS_Write : LS_Read);
		}

		/*! \brief Convenience function for lock(LS_Read);
			\return True if successful
		*/
		inline bool lockRead() { return lock(LS_Read); }

		/*! \brief Convenience function for lock(LS_Write);
			\return True if successful
		*/
		inline bool lockWrite() { return lock(LS_Write); }

		/*! \brief Convenience function for lock(LS_SuspendedWrite);
			\return True if successful
		*/
		inline bool lockSuspendedWrite() { return lock(LS_SuspendedWrite); }

		/*! \brief Suspend a write lock, allowing readers access, but preventing others from getting a write lock
			\returns True if successful or if already suspended
		*/
		inline bool suspend()
		{
			assert(m_lockedW);
			if (m_iSuspended)
				return true;
			m_iSuspended = true;
			return m_mutex->suspendWriteLock();
		}

		/*! \brief Resume a write lock
			\return True if successful or if already resuming
		*/
		inline bool resume()
		{
			assert(m_lockedW);
			if (!m_iSuspended)
				return true;
			m_iSuspended = false;
			return m_mutex->resumeWriteLock();
		}

		/*! \returns True if mutex is suspended
		*/
		inline bool isSuspended() const
		{
			return m_iSuspended;
		}

		/*! \returns True if mutex is suspended
			\note This function check the current status of the mutex to tell whether it is actually suspended
		*/
		inline bool mutexIsSuspended() const volatile
		{
			return (m_mutex->status() & (MSF_WriteLockSuspended | MSF_WriteLocked)) == (MSF_WriteLockSuspended | MSF_WriteLocked);
		}

		/*! \brief Unlocks the write or read locked mutex
			\returns True if the mutex is successfully unlocked. False if it was not locked.
		*/
		inline bool unlock() noexcept
		{
			if (m_lockedW)
			{
				assert(!m_lockedR);
				m_lockedW = false;
				bool s = m_iSuspended;
				m_iSuspended = false;
				return m_mutex->releaseMutex(true, s);
			}
			if (m_lockedR)
			{
				m_lockedR = false;
				return m_mutex->releaseMutex(false, false);
			}
			return false;
		}

		/*! \returns True if the lock has at least the given state. A suspended write lock is treated as a read lock.
			\param minimumState The minimum lock state to check with
		*/
		inline bool isLocked(LockState minimumState) const
		{
			bool sus = isSuspended();
			if (minimumState == LS_Write)
				return m_lockedW && !sus;
			if (minimumState == LS_SuspendedWrite)
				return m_lockedW && sus;
			if (minimumState == LS_Read)
				return m_lockedW || m_lockedR;
			assert(minimumState == LS_Unlocked);
			return true;
		}

		/*! \brief Check if the lock is using the supplied mutex
			\param mutex The supplied suspendable readers-writer mutex
			\returns True if it is equal
		*/
		bool isUsing(MutexReadWriteSuspendable const* mutex) const
		{
			return m_mutex == mutex;
		}

		/*! \brief Check if the lock is using the supplied mutex
		\param mutex The supplied guarded mutex
		\returns True if it is equal
		*/
		bool isUsing(GuardedMutex const* mutex) const
		{
			return m_mutex == mutex;
		}
	};

	/*! \class LockGuarded
		\brief A guarded lock class
	*/
	class LockGuarded {
	private:
		GuardedMutex* m_mutex;
		bool m_locked;

		LockGuarded(LockGuarded const&) = delete;
		LockGuarded& operator=(LockGuarded const&) = delete;
		LockGuarded(LockGuarded &&) = delete;
		LockGuarded& operator=(LockGuarded &&) = delete;

	public:

		/*! \brief Constructs a guarded lock using a guarded mutex
		*/
		inline LockGuarded(GuardedMutex* mutex, bool lockit = true) : m_mutex(mutex), m_locked(false)
		{
			if (lockit)
				m_locked = m_mutex->claimMutex();
		}

		/*! \brief Constructs a guarded lock using a guarded mutex and a lock state
		*/
		inline LockGuarded(GuardedMutex* mutex, LockState lockit) : m_mutex(mutex), m_locked(false)
		{
			if (lockit != LS_Unlocked)
				m_locked = m_mutex->claimMutex();
		}

		/*! \brief Destructor
		*/
		inline ~LockGuarded()
		{
			unlock();
		}

		/*!	\brief Locks the unlocked mutex
			\returns True if the mutex is successfully locked
		*/
		inline bool lock()
		{
			if (!m_locked)
				return (m_locked = m_mutex->claimMutex()) != 0;
			return true;
		}

		/*!	\brief Unlocks the locked mutex
			\returns True if the mutex is successfully unlocked
		*/
		inline bool unlock() noexcept
		{
			if (m_locked)
			{
#ifndef __GNUC__
	#pragma warning(suppress: 4706)
#endif
				return !(m_locked = !m_mutex->releaseMutex());
			}
			return true;
		}

		/*! \returns True if the mutex is locked by this lock
		*/
		inline bool isLocked() const
		{
			return m_locked;
		}

		/*! \brief Check if the lock is using the supplied mutex
			\param mutex The supplied guarded mutex
			\returns True if it is equal
		*/
		bool isUsing(GuardedMutex const* mutex) const
		{
			return m_mutex == mutex;
		}

		/*! \brief Check if the lock is using the supplied mutex
			\param mutex The supplied suspendable readers-writer mutex
			\returns True if it is equal
		*/
		bool isUsing(MutexReadWriteSuspendable const* mutex) const
		{
			return m_mutex == mutex;
		}

		/*! \brief Check if the lock is using the supplied mutex
			\param mutex The supplied generic mutex
			\returns True if it is equal
		*/
		bool isUsing(Mutex const* mutex) const
		{
			return m_mutex->isUsing(mutex);
		}
	};

	/*! \class ProtectedValue
		\brief A protected value class
	*/
	template <typename K>
	class ProtectedValue {
	private:
		mutable LockReadWrite* m_lock;
	public:

		/*! \brief Constructs a protected value using a readers-writer mutex
		*/
		ProtectedValue(MutexReadWrite* mutex, bool writeLock = false)
		{
			m_lock = new LockReadWrite(mutex);
			m_lock->lock(writeLock);
		}

		/*! \brief Constructs a protected value using a reference of another protected value
		*/
		ProtectedValue(const ProtectedValue &pv)
		{
			m_lock = pv.m_lock;
			m_value = pv.m_value;
			pv.m_lock = NULL;
		}

		/*! \brief Destructor
		*/
		~ProtectedValue()
		{
			if (m_lock)
				delete m_lock;
		}

		/*! \brief Assignment operator, copies contents from the \a pv protected value
			\param pv : The protected value to copy from
		*/
		void operator = (const ProtectedValue& pv)
		{
			if (&pv == this)
				return;

			assert(pv.m_lock);
			m_lock = pv.m_lock;
			m_value = pv.m_value;
			pv.m_lock = NULL;
		}

		K m_value; //!< A type name of this class
	};

	/*! \class Semaphore
		\brief A semaphore class
	*/
	class Semaphore {
	protected:
#ifdef _WIN32
		HANDLE *m_handleList;	//!< The list of handles on the Semaphore
#else
		char *m_semname;		//!< A name of the semaphore
		sem_t *m_handle;		//!< A semaphore's handle
#endif
		uint32_t m_nofHandles;	//!< A number of the semaphore's handles

		Semaphore(Semaphore const&) = delete;
		Semaphore& operator=(Semaphore const&) = delete;
		Semaphore(Semaphore &&) = delete;
		Semaphore& operator=(Semaphore &&) = delete;

	public:

		/*! \brief Waits for an infinite time or until the semaphore is released
			\returns True if the semaphore is released
		*/
		bool wait1();

		/*! \brief Waits until time-out interval elapses or until the semaphore is released
			\param timeout The timeout value to use
			\returns True if the semaphore is released
		*/
		bool wait1(uint32_t timeout);

		/*! \brief Increases the count of the specified semaphore object by a specified amount
			\param increment The amount by which the semaphore object's current count is to be increased
			\returns The amount of the semaphore object's previous count
		*/
		int32_t post(int32_t increment = 1) noexcept;
#ifdef _WIN32
		/*! \brief Constructor
		*/
		Semaphore(int32_t initVal = 0, uint32_t nofOtherHandles = 0, HANDLE *otherHandles = NULL);
#else
		/*! \brief Constructor
		*/
		Semaphore(int32_t initVal = 0, uint32_t nofOtherHandles = 0, sem_t *otherHandles = NULL);
#endif
		/*! \brief Destructor
		*/
		~Semaphore();
	};

	/*! \class WaitCondition
	 *	\brief A platform independent wait condition implementation
	 *
	 * Wait conditions are used to halt consumer threads until changes are made to data they operate on.
	 *
	 * Creating the wait condition:
	 * \code
	 * Mutex mut;
	 * WaitCondition wc(&mut);
	 * \endcode
	 *
	 * Consumer thread:
	 * \code
	 * void func()
	 * {
	 *     Lock locker(&mut);
	 *
	 *     if (!wc.wait()) // unlocks the mutex and waits for a signal
	 *         // the mutex is claimed
	 *         return;
	 *
	 *     // mutex is claimed
	 *     // copy data
	 *     lock.unlock();
	 *     // work with data
	 * }
	 * \endcode
	 *
	 * Feeder thread:
	 * \code
	 * void func()
	 * {
	 *     Lock locker(&mut);
	 *
	 *     // update data
	 *     wc.signal(); // signal that changes have been made
	 *     // alternatively, use wc.broadcast() to allow multiple consumers to handle the data
	 * }
	 * \endcode
	 */
	class WaitCondition
	{
		WaitCondition(WaitCondition const&) = delete;
		WaitCondition& operator=(WaitCondition const&) = delete;
		WaitCondition(WaitCondition &&) = delete;
		WaitCondition& operator=(WaitCondition &&) = delete;

	public:
		explicit WaitCondition(Mutex &m);
		~WaitCondition();

		void signal();
		void broadcast();
		bool wait();
		bool wait(uint32_t timeout);
	private:
#ifdef _WIN32
		CONDITION_VARIABLE m_cond;
#else
		pthread_cond_t m_cond;
		pthread_condattr_t m_condattr;
#ifdef __APPLE__
		int m_clockId;
#else
		clockid_t m_clockId;
#endif
#endif
		Mutex &m_mutex;
	};

	/*! \brief An event that can be set/reset and that can be waited for
		\details The WaitEvent is intended for 1-1 thread synchronization, not 1-N thread synchronization
	*/
	class WaitEvent
	{
		WaitEvent(WaitEvent const&) = delete;
		WaitEvent& operator=(WaitEvent const&) = delete;
		WaitEvent(WaitEvent &&) = delete;
		WaitEvent& operator=(WaitEvent &&) = delete;

	public:
		WaitEvent();
		~WaitEvent();

		void terminate();
		bool wait();
		void set();
		void reset();

	private:
#if defined(_WIN32)
		HANDLE m_event;
#else
		pthread_mutex_t m_mutex;
		pthread_cond_t m_cond;
		bool m_triggered;
#endif
		volatile std::atomic_int m_waiterCount;
		bool m_terminating;
	};
} // namespace xsens

#ifndef __GNUC__
#pragma warning(default: 4127)
#endif

#endif
