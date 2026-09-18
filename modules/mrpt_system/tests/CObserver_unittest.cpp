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

#include <gtest/gtest.h>
#include <mrpt/system/CObservable.h>
#include <mrpt/system/CObserver.h>

using mrpt::system::CObservable;
using mrpt::system::CObserver;
using mrpt::system::mrptEvent;
using mrpt::system::mrptEventOnDestroy;

namespace
{
/** An event type of its own, to check the dynamic dispatch in OnEvent(). */
class TestEvent : public mrptEvent
{
 public:
  explicit TestEvent(int v) : value(v) {}
  int value;

 protected:
  void do_nothing() override {}
};

class CountingObserver : public CObserver
{
 public:
  int numEvents = 0;
  int lastValue = 0;
  int numDestroyEvents = 0;

 protected:
  void OnEvent(const mrptEvent& e) override
  {
    numEvents++;
    if (e.isOfType<TestEvent>())
    {
      lastValue = e.getAs<TestEvent>()->value;
    }
    if (e.isOfType<mrptEventOnDestroy>())
    {
      numDestroyEvents++;
    }
  }
};

/** A CObservable that can publish on demand. */
class TestObservable : public CObservable
{
 public:
  using CObservable::publishEvent;
};
}  // namespace

TEST(CObserver, publishReachesEverySubscriber)
{
  TestObservable obj;
  CountingObserver a;
  CountingObserver b;

  a.observeBegin(obj);
  b.observeBegin(obj);

  obj.publishEvent(TestEvent(42));

  EXPECT_EQ(a.numEvents, 1);
  EXPECT_EQ(a.lastValue, 42);
  EXPECT_EQ(b.numEvents, 1);
  EXPECT_EQ(b.lastValue, 42);
}

TEST(CObserver, observeEndStopsTheNotifications)
{
  TestObservable obj;
  CountingObserver a;

  a.observeBegin(obj);
  obj.publishEvent(TestEvent(1));
  EXPECT_EQ(a.numEvents, 1);

  a.observeEnd(obj);
  obj.publishEvent(TestEvent(2));
  EXPECT_EQ(a.numEvents, 1);

  // Ending a subscription that is not there is a no-op:
  EXPECT_NO_THROW(a.observeEnd(obj));
}

TEST(CObserver, destroyingTheObserverUnsubscribesIt)
{
  TestObservable obj;
  {
    CountingObserver a;
    a.observeBegin(obj);
    obj.publishEvent(TestEvent(1));
    EXPECT_EQ(a.numEvents, 1);
  }
  // The observer is gone; publishing must not touch freed memory:
  EXPECT_NO_THROW(obj.publishEvent(TestEvent(2)));
}

TEST(CObserver, destroyingTheObservableNotifiesAndUnsubscribes)
{
  CountingObserver a;
  {
    TestObservable obj;
    a.observeBegin(obj);
    obj.publishEvent(TestEvent(1));
    EXPECT_EQ(a.numEvents, 1);
  }
  // The destructor publishes mrptEventOnDestroy and then drops everyone:
  EXPECT_EQ(a.numDestroyEvents, 1);

  // ...so the observer no longer holds a dangling subscription:
  TestObservable other;
  EXPECT_NO_THROW(a.observeBegin(other));
}

TEST(CObserver, oneObserverOnSeveralObservables)
{
  TestObservable objA;
  TestObservable objB;
  CountingObserver obs;

  obs.observeBegin(objA);
  obs.observeBegin(objB);

  objA.publishEvent(TestEvent(1));
  objB.publishEvent(TestEvent(2));

  EXPECT_EQ(obs.numEvents, 2);
  EXPECT_EQ(obs.lastValue, 2);
}

TEST(CObserver, publishOnAnObservableWithNoSubscribers)
{
  TestObservable obj;
  EXPECT_NO_THROW(obj.publishEvent(TestEvent(1)));
}

TEST(mrptEvent, typeIntrospection)
{
  const TestEvent e(7);
  const mrptEvent& base = e;

  EXPECT_TRUE(base.isOfType<TestEvent>());
  EXPECT_FALSE(base.isOfType<mrptEventOnDestroy>());
  EXPECT_EQ(base.getAs<TestEvent>()->value, 7);

  // The non-const accessor is available too:
  const TestEvent e2(9);
  const mrptEvent& base2 = e2;
  EXPECT_EQ(base2.getAsNonConst<TestEvent>()->value, 9);
}
