/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <mrpt/core/Clock.h>

namespace mrpt::system
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
 * \ingroup mrpt_system_grp
 */
class mrptEvent
{
 protected:
  /** Just to allow this class to be polymorphic */
  virtual void do_nothing() {}

 public:
  mrptEvent() = default;
  virtual ~mrptEvent() = default;

  template <class EVENTTYPE>
  bool isOfType() const
  {
    return dynamic_cast<const EVENTTYPE*>(this) != nullptr;
  }

  template <class EVENTTYPE>
  const EVENTTYPE* getAs() const
  {
    return dynamic_cast<const EVENTTYPE*>(this);
  }

  template <class EVENTTYPE>
  EVENTTYPE* getAsNonConst() const
  {
    return const_cast<EVENTTYPE*>(dynamic_cast<const EVENTTYPE*>(this));
  }

  mrpt::Clock::time_point timestamp{mrpt::Clock::now()};

};  // End of class def.

/**  An event sent by any CObservable object (automatically) just before being
 * destroyed and telling its observers to unsubscribe.
 * \ingroup mrpt_system_grp
 */
class mrptEventOnDestroy : public mrptEvent
{
 protected:
  /** Just to allow this class to be polymorphic */
  void do_nothing() override {}

 public:
  mrptEventOnDestroy(const CObservable* obj) : source_object(obj) {}
  const CObservable* source_object;

};  // End of class def.

}  // namespace mrpt::system
