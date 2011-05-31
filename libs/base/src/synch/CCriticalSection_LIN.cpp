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

#include <mrpt/base.h>  // Only for precomp. headers, include all libmrpt-core headers.


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
	m_data.getAs<CRIT_SECT_LIN*>()->cs = cs;

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
//		if ( m_data.getAs<CRIT_SECT_LIN*>()->currentThreadOwner != 0 )
//			THROW_EXCEPTION(format("Destroying a critical section ('%s') currently locked by thread 0x%08lX", m_name.c_str(), m_data.getAs<CRIT_SECT_LIN*>()->currentThreadOwner ) );
	}
}

/*---------------------------------------------------------------
						enter
---------------------------------------------------------------*/
void  CCriticalSection::enter() const
{
	const unsigned long threadid = mrpt::system::getCurrentThreadId();

	if (m_debugOut)	m_debugOut->printf("[CCriticalSection:%s] Entering Thread ID:0x%08lX\n",m_name.c_str(),threadid );

	CRIT_SECT_LIN *myCS = const_cast<CRIT_SECT_LIN *>( m_data.getAs<const CRIT_SECT_LIN*>() );

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

	CRIT_SECT_LIN *myCS = const_cast<CRIT_SECT_LIN *>( m_data.getAs<const CRIT_SECT_LIN*>() );

	if ( myCS->currentThreadOwner!=threadid )
		THROW_EXCEPTION(format("Trying to release a critical section  ('%s') locked by a different thread.",m_name.c_str()));

	myCS->currentThreadOwner = 0;

    pthread_mutex_unlock( & myCS->cs );
}



#endif // Linux
