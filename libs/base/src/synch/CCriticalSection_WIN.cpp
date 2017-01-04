/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

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

	InitializeCriticalSection( & m_data.getAsPtr<CRIT_SECT_WIN>()->cs );

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
		//unsigned long	cur_own = m_data.getAsPtr<CRIT_SECT_WIN>()->currentThreadOwner;
		// JL (mar/2011): Disabled to avoid weird errors when suddenly closing a pogram with running mrpt::gui windows.
//		if ( cur_own != 0 )
//			THROW_EXCEPTION(format("Destroying a critical section ('%s') currently locked by thread %lu", m_name.c_str(), cur_own ) )

		DeleteCriticalSection( & m_data.getAsPtr<CRIT_SECT_WIN>()->cs );

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

	CRIT_SECT_WIN *myCS = const_cast<CRIT_SECT_WIN  *>(  m_data.getAsPtr<CRIT_SECT_WIN>() );

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

	CRIT_SECT_WIN *myCS = const_cast<CRIT_SECT_WIN  *>(  m_data.getAsPtr<CRIT_SECT_WIN>() );

	if ( myCS->currentThreadOwner!=threadid )
		THROW_EXCEPTION(format("Trying to release a critical section  ('%s') locked by a different thread.",m_name.c_str()));

	myCS->currentThreadOwner = 0;

	LeaveCriticalSection( & myCS->cs );
}



#endif // Windows
