/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
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


#include <mrpt/synch.h>
#include <mrpt/utils/CStream.h>

#include <iostream>

using namespace mrpt::utils;
using namespace mrpt::synch;
using namespace std;

#define CS_LOCKER_VERBOSE  0

/*---------------------------------------------------------------
				CCriticalSectionLocker
---------------------------------------------------------------*/
CCriticalSectionLocker::CCriticalSectionLocker( const CCriticalSection * cs)
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
