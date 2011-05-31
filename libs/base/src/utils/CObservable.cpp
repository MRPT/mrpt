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
