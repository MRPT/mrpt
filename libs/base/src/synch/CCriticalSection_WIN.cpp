/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/base.h>  // Precompiled headers

#include <mrpt/config.h>

#ifdef MRPT_OS_WINDOWS

#include <windows.h>

#include <mrpt/system/threads.h>
#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt::utils;
using namespace mrpt::synch;

struct CRIT_SECT_WIN
{
	CRITICAL_SECTION	cs;
	unsigned long		currentThreadOwner;
};

/*---------------------------------------------------------------
						CCriticalSection
---------------------------------------------------------------*/
CCriticalSection::CCriticalSection( const char *name )
{
	m_debugOut = NULL; //&utils::stdOut;

	m_data.resize( sizeof(CRIT_SECT_WIN) + 30 );

	InitializeCriticalSection( & m_data.getAs<CRIT_SECT_WIN*>()->cs );

	if (name!=NULL)
		m_name = name;
	else
		m_name = "Unnamed";

}

/*---------------------------------------------------------------
						Destructor
---------------------------------------------------------------*/
CCriticalSection::~CCriticalSection()
{
	if (m_data.alias_count()==1)
	{
		unsigned long	cur_own = m_data.getAs<CRIT_SECT_WIN*>()->currentThreadOwner;
		// JL (mar/2011): Disabled to avoid weird errors when suddenly closing a pogram with running mrpt::gui windows.
//		if ( cur_own != 0 )
//			THROW_EXCEPTION(format("Destroying a critical section ('%s') currently locked by thread %lu", m_name.c_str(), cur_own ) )

		DeleteCriticalSection( & m_data.getAs<CRIT_SECT_WIN*>()->cs );

		m_data.clear();
	}
}

/*---------------------------------------------------------------
						enter
---------------------------------------------------------------*/
void  CCriticalSection::enter() const
{
	const unsigned long threadid = mrpt::system::getCurrentThreadId();

	if (m_debugOut) m_debugOut->printf("[CCriticalSection:%s] Entering Thread ID:%lu\n", m_name.c_str(), threadid  );

	CRIT_SECT_WIN *myCS = const_cast<CRIT_SECT_WIN  *>(  m_data.getAs<const CRIT_SECT_WIN*>() );

	if( myCS->currentThreadOwner == threadid )
		THROW_EXCEPTION(format("Detected recursive lock on critical section ('%s') by the same thread: %lu",m_name.c_str(),threadid ) )

	EnterCriticalSection( & myCS->cs );

	ASSERT_( myCS->currentThreadOwner == 0 );
	myCS->currentThreadOwner = threadid;

	if (m_debugOut) m_debugOut->printf("[CCriticalSection:%s] Entering DONE Thread ID:%lu\n",m_name.c_str(),threadid );
}

/*---------------------------------------------------------------
						leave
---------------------------------------------------------------*/
void  CCriticalSection::leave() const
{
	const unsigned long threadid =  mrpt::system::getCurrentThreadId();

	if (m_debugOut) m_debugOut->printf("[CCriticalSection:%s] Leaving Thread ID:%lu\n",m_name.c_str(),threadid );

	CRIT_SECT_WIN *myCS = const_cast<CRIT_SECT_WIN  *>(  m_data.getAs<const CRIT_SECT_WIN*>() );

	if ( myCS->currentThreadOwner!=threadid )
		THROW_EXCEPTION(format("Trying to release a critical section  ('%s') locked by a different thread.",m_name.c_str()));

	myCS->currentThreadOwner = 0;

	LeaveCriticalSection( & myCS->cs );
}



#endif // Windows
