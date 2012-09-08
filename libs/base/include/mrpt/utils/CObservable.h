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
#ifndef  CObservable_H
#define  CObservable_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/mrptEvent.h>

namespace mrpt
{
	namespace utils
	{
		class CObserver;

		/** Inherit from this class for those objects capable of being observed by a CObserver class.
		  *
		  *  The only thing to do in your child class is to call CObservable::publishEvent() whenever needed and all the
		  *   observer classes will be notified.
		  *
		  * \note The pairs CObservable / CObserver automatically notify each other the destruction of any of them, effectively ending the subscription of events.
		  *
		  *  \sa CObserver, mrptEvent
		 * \ingroup mrpt_base_grp
		  */
		class BASE_IMPEXP CObservable
		{
			friend class CObserver;

		public:
			CObservable();
			virtual ~CObservable();

		private:
			std::set<CObserver*>  m_subscribers;
			void internal_observer_begin(CObserver*);
			void internal_observer_end(CObserver*);

		protected:
			/** Called when you want this object to emit an event to all the observers currently subscribed to this object. */
			void publishEvent(const mrptEvent &e) const;

			/** Can be called by a derived class before preparing an event for publishing with \a publishEvent to determine if there is no one
			  *  subscribed, so it can save the wasted time preparing an event that will be not read. */
			inline bool hasSubscribers() const { return !m_subscribers.empty(); }

		}; // End of class def.

	} // End of namespace
} // end of namespace
#endif
