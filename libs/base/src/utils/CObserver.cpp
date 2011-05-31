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

CObserver::CObserver()
{
}

CObserver::~CObserver()
{
	while (!m_subscribed.empty())
		this->observeEnd(** m_subscribed.begin() );
}

/** Starts the subscription of this observer to the given object.  \sa observeEnd  */
void CObserver::observeBegin(CObservable &obj)
{
	m_subscribed.insert(&obj);
	obj.internal_observer_begin(this);
}

/** Ends the subscription of this observer to the given object (note that there is no need to call this method, since the destruction of the first of observer/observed will put an end to the process
\sa observeBegin  */
void CObserver::observeEnd(CObservable &obj)
{
	set<CObservable*>::iterator it = m_subscribed.find(&obj);
	if (it!=m_subscribed.end())
	{
		(*it)->internal_observer_end(this);
		m_subscribed.erase(it);
	}
}

// Redirect the notification to the user virtual method:
void CObserver::internal_on_event(const mrptEvent &e)
{
	this->OnEvent(e);
}
