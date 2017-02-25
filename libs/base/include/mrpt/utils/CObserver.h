/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  CObserver_H
#define  CObserver_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/mrptEvent.h>
#include <set>

namespace mrpt
{
	namespace utils
	{
		class CObservable;

		/** Inherit from this class to get notified about events from any CObservable object after subscribing to it.
		  *
		  *  The main methods in this class are:
		  *   - observeBegin(): To be called to start listening at a given object.
		  *   - OnEvent(): Virtual functions to be implemented in your child class to receive all the notifications. 
		  *
		  *  Note that if custom (child) mrptEvent classes are used, you can tell between them in runtime with "dynamic_cast<>()". 
		  *
		  * \note The pairs CObservable / CObserver automatically notify each other the destruction of any of them, effectively ending the subscription of events.
		  * \ingroup mrpt_base_grp
		  *  \sa CObservable, mrptEvent
		  */
		class BASE_IMPEXP CObserver
		{
			friend class CObservable;

		public:
			CObserver();
			virtual ~CObserver();

			/** Starts the subscription of this observer to the given object.  \sa observeEnd  */
			void observeBegin(CObservable &obj);

			/** Ends the subscription of this observer to the given object (note that there is no need to call this method, since the destruction of the first of observer/observed will put an end to the process
			    \sa observeBegin  */
			void observeEnd(CObservable &obj);

		private:
			std::set<CObservable*>  m_subscribed; 
			void internal_on_event(const mrptEvent &e);

		protected:
			/** This virtual function will be called upon receive of any event after starting listening at any CObservable object.
			  */
			virtual void OnEvent(const mrptEvent &e) = 0;

		}; // End of class def.

	} // End of namespace
} // end of namespace
#endif
