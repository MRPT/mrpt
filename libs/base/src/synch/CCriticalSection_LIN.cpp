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
#if defined(MRPT_OS_LINUX) || defined(MRPT_OS_APPLE)

#include <mrpt/system/threads.h>
#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/utils/CStdOutStream.h>

#include <iostream>
#include <pthread.h>
#include <errno.h>
#include <semaphore.h>
#include <sys/timeb.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::synch;
using namespace std;

struct CRIT_SECT_LIN
{
	pthread_mutex_t		cs;
	unsigned long		currentThreadOwner;
};

/*---------------------------------------------------------------
						CCriticalSection
---------------------------------------------------------------*/
CCriticalSection::CCriticalSection( const char *name )
{
	m_debugOut = NULL; //&UTILS::stdOut;

	m_data.resize( sizeof( CRIT_SECT_LIN ) + 10 );

	pthread_mutex_t cs = PTHREAD_MUTEX_INITIALIZER;
	m_data.getAsPtr<CRIT_SECT_LIN>()->cs = cs;

	if (name!=NULL)
            m_name = name;
	else    m_name = "Unnamed";
}

/*---------------------------------------------------------------
						Destructor
---------------------------------------------------------------*/
CCriticalSection::~CCriticalSection()
{
	if (m_data.alias_count()==1)
	{
		// JL (mar/2011): Disabled to avoid weird errors when suddenly closing a pogram with running mrpt::gui windows.
//		if ( m_data.getAsPtr<CRIT_SECT_LIN>()->currentThreadOwner != 0 )
//			THROW_EXCEPTION(format("Destroying a critical section ('%s') currently locked by thread 0x%08lX", m_name.c_str(), m_data.getAsPtr<CRIT_SECT_LIN>()->currentThreadOwner ) );
	}
}

/*---------------------------------------------------------------
						enter
---------------------------------------------------------------*/
void  CCriticalSection::enter() const
{
	const unsigned long threadid = mrpt::system::getCurrentThreadId();

	if (m_debugOut)	m_debugOut->printf("[CCriticalSection:%s] Entering Thread ID:0x%08lX\n",m_name.c_str(),threadid );

	CRIT_SECT_LIN *myCS = const_cast<CRIT_SECT_LIN *>( m_data.getAsPtr<CRIT_SECT_LIN>() );

	if( myCS->currentThreadOwner == threadid )
		THROW_EXCEPTION(format("Detected recursive lock on critical section ('%s') by the same thread: 0x%08lX",m_name.c_str(),threadid))

    pthread_mutex_lock( & myCS->cs );

	if (m_debugOut)	m_debugOut->printf("[CCriticalSection:%s] Entering DONE Thread ID:0x%08lX\n",m_name.c_str(),threadid );

	ASSERT_( myCS->currentThreadOwner == 0 );
	myCS->currentThreadOwner = threadid;

}

/*---------------------------------------------------------------
						leave
---------------------------------------------------------------*/
void  CCriticalSection::leave() const
{
	const unsigned long threadid =  mrpt::system::getCurrentThreadId();

	if (m_debugOut)	m_debugOut->printf("[CCriticalSection:%s] Leaving Thread ID:0x%08lX\n",m_name.c_str(),threadid );

	CRIT_SECT_LIN *myCS = const_cast<CRIT_SECT_LIN *>( m_data.getAsPtr<CRIT_SECT_LIN>() );

	if ( myCS->currentThreadOwner!=threadid )
		THROW_EXCEPTION(format("Trying to release a critical section  ('%s') locked by a different thread.",m_name.c_str()));

	myCS->currentThreadOwner = 0;

    pthread_mutex_unlock( & myCS->cs );
}



#endif // Linux
