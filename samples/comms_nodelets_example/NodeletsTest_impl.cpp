/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

/** \example comms_nodelets_example/NodeletsTest_impl.cpp */

bool nodelets_test_passed_ok = false;

//! [example-nodelets]
#include <mrpt/comms/nodelets.h>
#include <mrpt/poses/CPose3D.h>
#include <cstdio>  // printf()
#include <thread>
#include <chrono>
#include <iostream>

// Test payload:
const mrpt::poses::CPose3D p_tx(1.0, 2.0, 3.0, 0.2, 0.4, 0.6);

// Create the topic directory. Only once per process, and must be shared
// by all nodelets/threads.
auto dir = mrpt::comms::TopicDirectory::create();

void thread_publisher()
{
	using namespace mrpt::comms;
	using namespace std;

	try
	{
#ifdef NODELETS_TEST_VERBOSE
		printf("[publisher] Started\n");
#endif

		for (int i = 0; i < 5; i++)
		{
			std::this_thread::sleep_for(100ms);
			dir->getTopic("/robot/odom")->publish(p_tx);
		}

#ifdef NODELETS_TEST_VERBOSE
		printf("[publisher] Finish\n");
#endif
	}
	catch (const std::exception& e)
	{
		cerr << e.what() << endl;
	}
	catch (...)
	{
		printf("[thread_publisher] Runtime error!\n");
	}
}

void onNewMsg(const mrpt::poses::CPose3D& p)
{
#ifdef NODELETS_TEST_VERBOSE
	std::cout << "sub2: rx CPose3D" << p.asString() << std::endl;
#endif
}

void onNewMsg2(int idx, const mrpt::poses::CPose3D& p)
{
#ifdef NODELETS_TEST_VERBOSE
	std::cout << "onNewMsg2: idx=" << idx << " rx CPose3D" << p.asString()
			  << std::endl;
#endif
}

void thread_subscriber()
{
	using namespace mrpt::comms;
	using namespace std;

	try
	{
#ifdef NODELETS_TEST_VERBOSE
		printf("[subscriber] Connecting\n");
#endif

#ifdef NODELETS_TEST_VERBOSE
		printf("[subscriber] Connected. Waiting for a message...\n");
#endif

		// Create a subscriber with a lambda:
		Subscriber::Ptr sub1 =
			dir->getTopic("/robot/odom")
				->createSubscriber<mrpt::poses::CPose3D>(
					[](const mrpt::poses::CPose3D& p_rx) -> void {
#ifdef NODELETS_TEST_VERBOSE
						std::cout << "sub1: rx CPose3D" << p_rx.asString()
								  << std::endl;
#endif
						nodelets_test_passed_ok = (p_rx == p_tx);
					});

		// Create a subscriber with a regular function via std::function:
		auto sub2 = dir->getTopic("/robot/odom")
						->createSubscriber<mrpt::poses::CPose3D>(
							std::function<void(const mrpt::poses::CPose3D&)>(
								&onNewMsg));

		// Create a subscriber with a regular function:
		auto sub3 = dir->getTopic("/robot/odom")
						->createSubscriber<mrpt::poses::CPose3D>(&onNewMsg);

		// Create a subscriber with std::bind:
		using namespace std::placeholders;
		auto sub4 = dir->getTopic("/robot/odom")
						->createSubscriber<mrpt::poses::CPose3D>(
							[](auto&& arg1) { return onNewMsg2(123, arg1); });

		// wait for messages to arrive.
		// The nodelet is up and live until "sub" gets out of scope.
		std::this_thread::sleep_for(1000ms);

#ifdef NODELETS_TEST_VERBOSE
		printf("[subscriber] Finish\n");
#endif
	}
	catch (const std::exception& e)
	{
		cerr << e.what() << endl;
	}
	catch (...)
	{
		cerr << "[thread_subscriber] Runtime error!" << endl;
	}
}

void NodeletsTest()
{
	using namespace std::chrono_literals;

	std::thread(thread_publisher).detach();
	std::thread(thread_subscriber).detach();
	std::this_thread::sleep_for(1000ms);
}
//! [example-nodelets]
