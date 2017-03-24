/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers 


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
