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
#include <mrpt/comms/nodelets.h>

#include <memory>
#include <string>

using namespace mrpt::comms;

TEST(nodelets, sameTopicNameYieldsSameTopic)
{
  auto dir = TopicDirectory::create();

  auto a = dir->getTopic("/robot/pose");
  auto b = dir->getTopic("/robot/pose");
  auto c = dir->getTopic("/robot/odometry");

  EXPECT_EQ(a.get(), b.get());
  EXPECT_NE(a.get(), c.get());
}

TEST(nodelets, topicIsRecreatedAfterItsLastReferenceIsDropped)
{
  auto dir = TopicDirectory::create();

  std::weak_ptr<Topic> firstTopic;
  {
    auto t = dir->getTopic("/transient");
    ASSERT_TRUE(t);
    firstTopic = t;
  }
  // The directory only holds weak references, so the entry is gone by now...
  EXPECT_TRUE(firstTopic.expired());

  // ...and asking again builds a fresh Topic:
  auto t2 = dir->getTopic("/transient");
  EXPECT_TRUE(t2);
}

TEST(nodelets, publishReachesEverySubscriber)
{
  auto dir = TopicDirectory::create();
  auto topic = dir->getTopic("/values");

  int received1 = 0;
  int received2 = 0;

  auto sub1 = topic->createSubscriber<int>([&](int v) { received1 += v; });
  auto sub2 = topic->createSubscriber<int>([&](int v) { received2 += 2 * v; });

  topic->publish(5);

  EXPECT_EQ(received1, 5);
  EXPECT_EQ(received2, 10);
}

TEST(nodelets, unsubscribedSubscribersStopReceiving)
{
  auto dir = TopicDirectory::create();
  auto topic = dir->getTopic("/values");

  int received = 0;
  {
    auto sub = topic->createSubscriber<int>([&](int v) { received += v; });
    topic->publish(1);
    EXPECT_EQ(received, 1);
  }
  // Destroying the subscriber removes it from the topic's list:
  topic->publish(1);
  EXPECT_EQ(received, 1);
}

TEST(nodelets, publishOfAWrongTypeIsIgnored)
{
  auto dir = TopicDirectory::create();
  auto topic = dir->getTopic("/values");

  bool called = false;
  auto sub = topic->createSubscriber<int>([&](int) { called = true; });

  // A std::string where the subscriber expects an int: the bad_any_cast is
  // caught and reported, and the callback is never run.
  topic->publish(std::string("not an int"));

  EXPECT_FALSE(called);
}

TEST(nodelets, publishOnATopicWithNoSubscribers)
{
  auto dir = TopicDirectory::create();
  auto topic = dir->getTopic("/nobody/listens");

  EXPECT_NO_THROW(topic->publish(42));
}

TEST(nodelets, subscribersOfDifferentTypesOnDifferentTopics)
{
  auto dir = TopicDirectory::create();

  std::string lastString;
  double lastDouble = 0;

  auto tStr = dir->getTopic("/strings");
  auto tNum = dir->getTopic("/numbers");

  auto s1 = tStr->createSubscriber<std::string>([&](const std::string& v) { lastString = v; });
  auto s2 = tNum->createSubscriber<double>([&](double v) { lastDouble = v; });

  tStr->publish(std::string("hello"));
  tNum->publish(3.5);

  EXPECT_EQ(lastString, "hello");
  EXPECT_EQ(lastDouble, 3.5);
}
