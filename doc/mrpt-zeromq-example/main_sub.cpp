/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <zmq.h>
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
		printf ("Subscribed.\n");

		for (int i=0;i<10;i++)
		{
			// Create an empty ZMQ message
			zmq_msg_t msg;
			rc = zmq_msg_init (&msg);
			assert (rc == 0);

			// Block until a message is available to be received from socket
			printf ("Waiting incomming pkt...");
			rc = zmq_msg_recv(&msg,sub_sock, 0 /* block */);
			assert (rc == 0);
			printf ("Received!\n");

			printf("%d msg received: \n", i);

			zmq_msg_close (&msg); // Free msg
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
