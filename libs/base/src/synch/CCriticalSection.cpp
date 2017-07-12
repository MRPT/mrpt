/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/system/threads.h>
#include <mutex>

using namespace mrpt::utils;
using namespace mrpt::synch;

// CAbstractMutex
CAbstractMutex::~CAbstractMutex()
{
}

// CCriticalSection
CCriticalSection::CCriticalSection(const char *name) :
	m_data(new std::mutex()),
	m_name(name ? name : "Unnamed"),
	m_debugOut(nullptr)
{
}

CCriticalSection::~CCriticalSection()
{
	std::mutex *mut = reinterpret_cast<std::mutex*>(m_data);
	delete mut;
	m_data = nullptr;
}

void  CCriticalSection::enter() const
{
	const unsigned long threadid = mrpt::system::getCurrentThreadId();
	if (m_debugOut) m_debugOut->printf("[CCriticalSection:%s] Entering Thread ID:%lu\n", m_name.c_str(), threadid);

	std::mutex *mut = reinterpret_cast<std::mutex*>(m_data);
	mut->lock();

	if (m_debugOut) m_debugOut->printf("[CCriticalSection:%s] Entering DONE Thread ID:%lu\n", m_name.c_str(), threadid);
}

void  CCriticalSection::leave() const
{
	const unsigned long threadid = mrpt::system::getCurrentThreadId();
	if (m_debugOut) m_debugOut->printf("[CCriticalSection:%s] Leaving Thread ID:%lu\n", m_name.c_str(), threadid);

	std::mutex *mut = reinterpret_cast<std::mutex*>(m_data);
	mut->unlock();
}

bool CCriticalSection::try_enter() const
{
	std::mutex *mut = reinterpret_cast<std::mutex*>(m_data);
	return mut->try_lock();
}

// CCriticalSectionRecursive
CCriticalSectionRecursive::CCriticalSectionRecursive() :
	m_data( new std::recursive_mutex() )
{
}
CCriticalSectionRecursive::~CCriticalSectionRecursive()
{
	std::recursive_mutex *mut = reinterpret_cast<std::recursive_mutex*>(m_data);
	delete mut;
	m_data=nullptr;
}

void  CCriticalSectionRecursive::enter() const
{
	std::recursive_mutex *mut = reinterpret_cast<std::recursive_mutex*>(m_data);
	mut->lock();
}

void  CCriticalSectionRecursive::leave() const
{
	std::recursive_mutex *mut = reinterpret_cast<std::recursive_mutex*>(m_data);
	mut->unlock();
}

bool CCriticalSectionRecursive::try_enter() const
{
	std::recursive_mutex *mut = reinterpret_cast<std::recursive_mutex*>(m_data);
	return mut->try_lock();
}




