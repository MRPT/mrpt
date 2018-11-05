/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/comms/CServerTCPSocket.h>
#include <mrpt/comms/CClientTCPSocket.h>
#include <mrpt/serialization/CMessage.h>
#include <mrpt/system/scheduler.h>  // changeCurrentThreadPriority()
#include <mrpt/poses/CPose3D.h>
#include <cstdio>  // printf()
#include <thread>
#include <chrono>
#include <iostream>

bool sockets_test_passed_ok = false;
// Test payload:
const mrpt::poses::CPose3D p_tx(1.0, 2.0, 3.0, 0.2, 0.4, 0.6);

using mrpt::serialization::CMessage;

void thread_server()
{
	using namespace mrpt::comms;
	using namespace std;

	try
	{
#ifdef SOCKET_TEST_VERBOSE
		printf("[Server] Started\n");
#endif

		CServerTCPSocket server(
			15000, "127.0.0.1", 10,
#ifdef SOCKET_TEST_VERBOSE
			mrpt::system::LVL_DEBUG
#else
			mrpt::system::LVL_ERROR
#endif
		);
		std::unique_ptr<CClientTCPSocket> client = server.accept(2000);

		if (client)
		{
#ifdef SOCKET_TEST_VERBOSE
			printf("[Server] Connection accepted\n");
#endif
			// Send a message with the payload:
			CMessage msg;
			msg.type = 0x10;
			msg.serializeObject(&p_tx);

			client->sendMessage(msg);

			std::this_thread::sleep_for(50ms);
		}

#ifdef SOCKET_TEST_VERBOSE
		printf("[Server] Finish\n");
#endif
	}
	catch (const std::exception& e)
	{
		cerr << e.what() << endl;
	}
	catch (...)
	{
		printf("[thread_server] Runtime error!\n");
	}
}

void thread_client()
{
	using namespace mrpt::comms;
	using namespace std;

	mrpt::system::changeCurrentThreadPriority(mrpt::system::tpLow);

	try
	{
		CClientTCPSocket sock;

#ifdef SOCKET_TEST_VERBOSE
		printf("[Client] Connecting\n");
#endif

		sock.connect("127.0.0.1", 15000);

#ifdef SOCKET_TEST_VERBOSE
		printf("[Client] Connected. Waiting for a message...\n");
#endif
		//		cout << "pending: " << sock.getReadPendingBytes() << endl;
		//		std::this_thread::sleep_for(4000ms);
		//		cout << "pending: " << sock.getReadPendingBytes() << endl;

		CMessage msg;
		bool ok = sock.receiveMessage(msg, 2000, 2000);

		if (!ok)
		{
			printf("[Client] Error receiving message!!\n");
		}
		else
		{
#ifdef SOCKET_TEST_VERBOSE
			printf("[Client] Message received OK!:\n");
			printf("  MSG Type: %i\n", msg.type);
			printf(
				"  MSG Length: %u bytes\n", (unsigned int)msg.content.size());
			printf("[Client] Parsing payload...\n");
#endif
			mrpt::poses::CPose3D p_rx;
			msg.deserializeIntoExistingObject(&p_rx);

#ifdef SOCKET_TEST_VERBOSE
			printf("[Client] Received payload: %s\n", p_rx.asString().c_str());
			printf("[Client] tx payload: %s\n", p_tx.asString().c_str());
			printf("[Client] Done!!\n");
#endif

			sockets_test_passed_ok = (p_rx == p_tx);
		}

#ifdef SOCKET_TEST_VERBOSE
		printf("[Client] Finish\n");
#endif
	}
	catch (const std::exception& e)
	{
		cerr << e.what() << endl;
	}
	catch (...)
	{
		cerr << "[thread_client] Runtime error!" << endl;
		;
	}
}

// ------------------------------------------------------
//				SocketsTest
// ------------------------------------------------------
void SocketsTest()
{
	using namespace std::chrono_literals;

	std::thread(thread_server).detach();
	std::this_thread::sleep_for(100ms);

	std::thread(thread_client).detach();
	std::this_thread::sleep_for(1000ms);
}
