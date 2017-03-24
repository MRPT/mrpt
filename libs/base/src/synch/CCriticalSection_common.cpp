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
#include <mutex>

using namespace mrpt::utils;
using namespace mrpt::synch;

CAbstractMutex::~CAbstractMutex()
{
}

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



