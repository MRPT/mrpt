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
#ifndef  CObserver_H
#define  CObserver_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/mrptEvent.h>

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
