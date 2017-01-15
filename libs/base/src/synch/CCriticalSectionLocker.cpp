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
#include <mrpt/utils/CStream.h>

#include <iostream>

using namespace mrpt::utils;
using namespace mrpt::synch;
using namespace std;

#define CS_LOCKER_VERBOSE  0

/*---------------------------------------------------------------
				CCriticalSectionLocker
---------------------------------------------------------------*/
CCriticalSectionLocker::CCriticalSectionLocker( const CAbstractMutex * cs)
	: m_cs(cs)
{
	if (m_cs)
	{
#if CS_LOCKER_VERBOSE
		cout << "[CCriticalSectionLocker] Locking " << static_cast<const void*>(m_cs) << ": " << m_cs->getName() << endl;
#endif
		m_cs->enter();
	}
}

/*---------------------------------------------------------------
				~CCriticalSectionLocker
---------------------------------------------------------------*/
CCriticalSectionLocker::~CCriticalSectionLocker()
{
	if (m_cs)
	{
#if CS_LOCKER_VERBOSE
		cout << "[CCriticalSectionLocker] Unlocking " << static_cast<const void*>(m_cs) << ": " << m_cs->getName() << endl;
#endif
		m_cs->leave();
	}
}
