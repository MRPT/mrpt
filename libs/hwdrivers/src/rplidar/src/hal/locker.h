/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#pragma once
namespace rp::hal
{
class Locker
{
   public:
	enum LOCK_STATUS
	{
		LOCK_OK = 1,
		LOCK_TIMEOUT = -1,
		LOCK_FAILED = 0
	};

	Locker()
	{
#ifdef _WIN32
		_lock = nullptr;
#endif
		init();
	}

	~Locker() { release(); }
	Locker::LOCK_STATUS lock(unsigned long timeout = 0xFFFFFFFF)
	{
#ifdef _WIN32
		switch (WaitForSingleObject(
			_lock, timeout == 0xFFFFFFF ? INFINITE : (DWORD)timeout))
		{
			case WAIT_ABANDONED:
				return LOCK_FAILED;
			case WAIT_OBJECT_0:
				return LOCK_OK;
			case WAIT_TIMEOUT:
				return LOCK_TIMEOUT;
		}

#else
#ifdef _MACOS
		if (timeout != 0)
		{
			if (pthread_mutex_lock(&_lock) == 0) return LOCK_OK;
		}
#else
		if (timeout == 0xFFFFFFFF)
		{
			if (pthread_mutex_lock(&_lock) == 0) return LOCK_OK;
		}
#endif
		else if (timeout == 0)
		{
			if (pthread_mutex_trylock(&_lock) == 0) return LOCK_OK;
		}
#ifndef _MACOS
		else
		{
			timespec wait_time;
			timeval now;
			gettimeofday(&now, nullptr);

			wait_time.tv_sec = timeout / 1000 + now.tv_sec;
			wait_time.tv_nsec = (timeout % 1000) * 1000000 + now.tv_usec * 1000;

			if (wait_time.tv_nsec >= 1000000000)
			{
				++wait_time.tv_sec;
				wait_time.tv_nsec -= 1000000000;
			}
			switch (pthread_mutex_timedlock(&_lock, &wait_time))
			{
				case 0:
					return LOCK_OK;
				case ETIMEDOUT:
					return LOCK_TIMEOUT;
			}
		}
#endif
#endif

		return LOCK_FAILED;
	}

	void unlock()
	{
#ifdef _WIN32
		ReleaseMutex(_lock);
#else
		pthread_mutex_unlock(&_lock);
#endif
	}

#ifdef _WIN32
	HANDLE getLockHandle() { return _lock; }
#else
	pthread_mutex_t* getLockHandle() { return &_lock; }
#endif

   protected:
	void init()
	{
#ifdef _WIN32
		_lock = CreateMutex(nullptr, FALSE, nullptr);
#else
		pthread_mutex_init(&_lock, nullptr);
#endif
	}

	void release()
	{
		unlock();  // force unlock before release
#ifdef _WIN32

		if (_lock) CloseHandle(_lock);
		_lock = nullptr;
#else
		pthread_mutex_destroy(&_lock);
#endif
	}

#ifdef _WIN32
	HANDLE _lock;
#else
	pthread_mutex_t _lock;
#endif
};

class AutoLocker
{
   public:
	AutoLocker(Locker& l) : _binded(l) { _binded.lock(); }
	void forceUnlock() { _binded.unlock(); }
	~AutoLocker() { _binded.unlock(); }
	Locker& _binded;
};
}  // namespace rp::hal
