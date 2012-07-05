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


#include <mrpt/utils/CObserver.h>
#include <mrpt/utils/CObservable.h>

using namespace mrpt::utils;
using namespace std;

CObservable::CObservable()
{
}

CObservable::~CObservable()
{
	MRPT_START
	// Notify my destruction:
	this->publishEvent( mrptEventOnDestroy(this) );

	// Tell observers to unsubscribe:
	while (!m_subscribers.empty())
		(*m_subscribers.begin())->observeEnd(*this);
	MRPT_END
}

void CObservable::internal_observer_begin(CObserver*o)
{
	m_subscribers.insert(o);
}

void CObservable::internal_observer_end(CObserver*o)
{
	MRPT_START
	set<CObserver*>::iterator it = m_subscribers.find(o);
	ASSERTMSG_(it!=m_subscribers.end(), "Ending subscription from an observer not subscribed to this object!")
	m_subscribers.erase(it);
	MRPT_END
}

/** Called when you want this object to emit an event to all the observers currently subscribed to this object.
*/
void CObservable::publishEvent(const mrptEvent &e) const
{
	MRPT_START
	for (set<CObserver*>::const_iterator it=m_subscribers.begin();it!=m_subscribers.end();++it)
		(*it)->internal_on_event(e);
	MRPT_END
}
