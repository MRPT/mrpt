/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
		//unsigned long	cur_own = m_data.getAs<CRIT_SECT_WIN*>()->currentThreadOwner;
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
