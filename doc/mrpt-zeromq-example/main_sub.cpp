/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <zmq.h>
#include <mrpt/utils/serialization_zmq.h>
#include <assert.h>
#include <stdio.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/utils/CTicTac.h>

using mrpt::utils::DEG2RAD;

int main()
{
	try
	{
		printf ("Subscribing to server...\n");
		void *context = zmq_ctx_new ();
		void *sub_sock = zmq_socket (context, ZMQ_SUB);
		int rc = zmq_connect (sub_sock, "tcp://localhost:5555");
		assert (rc == 0);
		zmq_setsockopt( sub_sock, ZMQ_SUBSCRIBE, "", 0 ); // Subscribe to everything.
		printf ("Subscribed.\n");

		for (int i=0;i<10;i++)
		{
			mrpt::utils::CSerializablePtr obj;
			size_t rx_len;

			printf ("Waiting %d-th incomming pkt...",i);
			obj = mrpt::utils::mrpt_recv_from_zmq(sub_sock, false /*false:blocking call*/, &rx_len);
			if (obj) {
				printf("OK!. Type: `%s` (%u bytes)\n", obj->GetRuntimeClass()->className, static_cast<unsigned>(rx_len));
			} else {
				printf("failed!\n");
			}

#if 0
			// Example for mrpt_recv_from_zmq_into():
			mrpt::poses::CPose3D pose;
			mrpt_recv_from_zmq_into(sub_sock, pose);
			std::cout << "pose: "<< pose << std::endl;
#endif
		}

		zmq_close (sub_sock);
		zmq_ctx_destroy (context);
		return 0;
	} catch (std::exception &e)
	{
		std::cerr << "**Exception**: " << e.what() << std::endl;
		return -1;
	}
}
