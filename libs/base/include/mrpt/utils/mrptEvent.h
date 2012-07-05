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
#ifndef  mrptEvent_H
#define  mrptEvent_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/system/datetime.h>

namespace mrpt
{
	namespace utils
	{
		class CObservable;

		/**  The basic event type for the observer-observable pattern in MRPT.
		  *   You can sub-class this base class to create custom event types, then
		  *    tell between them in runtime with isOfType<T>(), for example:
		  * \code
		  *   if (e.isOfType<mrptEventOnDestroy>())
		  *   {
		  *     const mrptEventOnDestroy* ev = e.getAs<mrptEventOnDestroy>();
		  *     ev-> ...
		  *   }
		  * \endcode
		  *
		  * \sa CObserver, CObservable
		 * \ingroup mrpt_base_grp
		  */
		class BASE_IMPEXP mrptEvent
		{
		protected:
			virtual void do_nothing() { } //!< Just to allow this class to be polymorphic
		public:
			/** Default ctor */
			inline mrptEvent() : timestamp(mrpt::system::now()) { }

			template <class EVENTTYPE>
			inline bool isOfType() const { return dynamic_cast<const EVENTTYPE*>(this)!=NULL; }

			template <class EVENTTYPE>
			inline const EVENTTYPE* getAs() const { return dynamic_cast<const EVENTTYPE*>(this); }
			
			template <class EVENTTYPE>
			inline EVENTTYPE* getAsNonConst() const { return const_cast<EVENTTYPE*>(dynamic_cast<const EVENTTYPE*>(this)); }

			mrpt::system::TTimeStamp  timestamp;

		}; // End of class def.

		/**  An event sent by any CObservable object (automatically) just before being destroyed and telling its observers to unsubscribe.
		 * \ingroup mrpt_base_grp
		  */
		class BASE_IMPEXP mrptEventOnDestroy : public mrptEvent
		{
		protected:
			virtual void do_nothing() { } //!< Just to allow this class to be polymorphic
		public:
			inline mrptEventOnDestroy(const CObservable *obj) : source_object(obj) { }

			const CObservable *source_object;

		}; // End of class def.

	} // End of namespace
} // end of namespace
#endif
