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
