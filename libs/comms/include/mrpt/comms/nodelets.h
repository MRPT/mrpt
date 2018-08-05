/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <memory>  // shared_ptr
#include <unordered_map>
#include <mutex>
#include <list>
#include <any>
#include <iostream>
#include <functional>
#include <mrpt/typemeta/TTypeName.h>
#include <mrpt/typemeta/TTypeName_stl.h>

namespace mrpt
{
namespace comms
{
/** Nodelet-like Pub/Sub communications (in #include <mrpt/comms/nodelets.h>)
 * \ingroup mrpt_comms_grp
 * @{ */

class Subscriber
{
   private:
	Subscriber(
		std::function<void(const std::any&)>&& func,
		std::function<void()>&& cleanup);

   public:
	using Ptr = std::shared_ptr<Subscriber>;
	~Subscriber();

	static Ptr create(
		std::function<void(const std::any&)>&& func,
		std::function<void()>&& cleanup);

	void pub(const std::any& a);

   private:
	std::function<void(const std::any&)> m_func;
	std::function<void()> m_cleanup;
};

class Topic : public std::enable_shared_from_this<Topic>
{
   private:
	Topic(std::function<void()>&& cleanup);

   public:
	using Ptr = std::shared_ptr<Topic>;

	~Topic();

	template <typename ARG, typename Callable>
	Subscriber::Ptr createSubscriber(Callable&& func)
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_subs.emplace_back();
		auto it = m_subs.end();
		it--;
		auto capturedShared = shared_from_this();
		auto newNode = Subscriber::create(
			[func{std::forward<Callable>(func)}](const std::any& anyArg) {
				try
				{
					std::invoke(func, std::any_cast<const ARG&>(anyArg));
				}
				catch (std::bad_any_cast&)
				{
					std::cerr << "Subscriber has wrong type: "
							  << mrpt::typemeta::TTypeName<ARG>::get()
							  << std::endl;
				}
			},
			// cleanup function
			[=] {
				// list operations don't invalidate iterators
				// This iterator capture will be valid
				capturedShared->cleanupSubscriber(it);
			});
		*it = newNode;
		return newNode;
	}

	void publish(const std::any& any);
	void cleanupSubscriber(std::list<std::weak_ptr<Subscriber>>::iterator it);

	template <typename CLEANUP>
	static Ptr create(CLEANUP&& cleanup)
	{
		return Ptr(new Topic(std::forward<CLEANUP>(cleanup)));
	}

   private:
	std::mutex m_mutex;
	std::list<std::weak_ptr<Subscriber>> m_subs;
	std::function<void()> m_cleanup;
};

/** The central directory of existing topics for pub/sub */
class TopicDirectory : public std::enable_shared_from_this<TopicDirectory>
{
   private:
	TopicDirectory();

   public:
	using Ptr = std::shared_ptr<TopicDirectory>;

	template <typename PATH>
	Topic::Ptr getTopic(PATH&& path)
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		auto it = m_mapService.find(path);
		if (it != m_mapService.end())
		{
			auto ptr = it->second.lock();
			if (ptr) return ptr;
		}

		auto capturedShared = shared_from_this();
		auto newNode =
			Topic::create([=]() { capturedShared->cleanupTopic(path); });
		m_mapService.insert({path, newNode});
		return newNode;
	}

	void cleanupTopic(const std::string& key);
	static Ptr create();

   private:
	std::mutex m_mutex;
	std::unordered_map<std::string, std::weak_ptr<Topic>> m_mapService;
};

/** @} */  // end grouping

}  // namespace comms
}  // namespace mrpt
