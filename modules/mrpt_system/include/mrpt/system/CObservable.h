/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <set>

namespace mrpt::system
{
class CObserver;
class mrptEvent;

/** Inherit from this class for those objects capable of being observed by a
 * CObserver class.
 *
 *  The only thing to do in your child class is to call
 * CObservable::publishEvent() whenever needed and all the
 *   observer classes will be notified.
 *
 * \note The pairs CObservable / CObserver automatically notify each other the
 * destruction of any of them, effectively ending the subscription of events.
 *
 *  \sa CObserver, mrptEvent
 * \ingroup mrpt_system_grp
 */
class CObservable
{
  friend class CObserver;

 public:
  CObservable();
  virtual ~CObservable();

 private:
  std::set<CObserver*> m_subscribers;
  void internal_observer_begin(CObserver*);
  void internal_observer_end(CObserver*);

 protected:
  /** Called when you want this object to emit an event to all the observers
   * currently subscribed to this object. */
  void publishEvent(const mrptEvent& e) const;

  /** Can be called by a derived class before preparing an event for
   * publishing with \a publishEvent to determine if there is no one
   *  subscribed, so it can save the wasted time preparing an event that will
   * be not read. */
  inline bool hasSubscribers() const { return !m_subscribers.empty(); }
};  // End of class def.

}  // namespace mrpt::system
