/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "comms-precomp.h"  // Precompiled headers

#include <mrpt/comms/nodelets.h>

using namespace mrpt::comms;

// ------- Subscriber --------------
Subscriber::Subscriber(
	std::function<void(const std::any&)>&& func,
	std::function<void()>&& cleanup)
	: m_func(std::move(func)), m_cleanup(std::move(cleanup))
{
}

Subscriber::~Subscriber() { m_cleanup(); }
Subscriber::Ptr Subscriber::create(
	std::function<void(const std::any&)>&& func,
	std::function<void()>&& cleanup)
{
	return Ptr(new Subscriber(std::move(func), std::move(cleanup)));
}

void Subscriber::pub(const std::any& a) { m_func(a); }
// ------- Topic --------------
Topic::Topic(std::function<void()>&& cleanup) : m_cleanup(std::move(cleanup)) {}
Topic::~Topic() { m_cleanup(); }
void Topic::publish(const std::any& any)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	for (auto& sub : m_subs)
	{
		sub.lock()->pub(any);
	}
}

void Topic::cleanupSubscriber(std::list<std::weak_ptr<Subscriber>>::iterator it)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_subs.erase(it);
}

// -------------- TopicDirectory ---------------------
TopicDirectory::TopicDirectory() = default;

void TopicDirectory::cleanupTopic(const std::string& key)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_mapService.erase(m_mapService.find(key));
}

TopicDirectory::Ptr TopicDirectory::create() { return Ptr(new TopicDirectory); }
