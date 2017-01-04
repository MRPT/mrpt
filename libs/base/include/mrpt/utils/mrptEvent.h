/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  mrptEvent_H
#define  mrptEvent_H

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
			void do_nothing()  MRPT_OVERRIDE { } //!< Just to allow this class to be polymorphic
		public:
			inline mrptEventOnDestroy(const CObservable *obj) : source_object(obj) { }

			const CObservable *source_object;

		}; // End of class def.

	} // End of namespace
} // end of namespace
#endif
