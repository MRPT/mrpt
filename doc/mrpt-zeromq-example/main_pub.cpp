/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+
 */

#include <assert.h>
#include <mrpt/img/CImage.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/serialization_zmq.h>
#include <mrpt/system/CTicTac.h>
#include <zmq.h>

using mrpt::DEG2RAD;

int main()
{
	try
	{
		printf("Starting publisher...\n");

		void* context = zmq_ctx_new();
		void* pub_sock = zmq_socket(context, ZMQ_PUB);
		int rc = zmq_bind(pub_sock, "tcp://*:5555");
		assert(rc == 0);

		while (1)
		{
			mrpt::poses::CPose3D my_pose(
				0.5f, 0.5f, 1.5f, DEG2RAD(-90.0f), DEG2RAD(0), DEG2RAD(-90.0f));
			printf("Publishing pose...\n");
			mrpt::comms::mrpt_send_to_zmq(pub_sock, my_pose);
			std::this_thread::sleep_for(100ms);

			mrpt::img::CImage my_img(800, 600, CH_RGB);
			printf("Publishing img...\n");
			mrpt::comms::mrpt_send_to_zmq(
				pub_sock, my_img, 0 /* max_packet_len: 0=no max size */);
			std::this_thread::sleep_for(100ms);
		}

		zmq_close(pub_sock);
		zmq_ctx_destroy(context);
		return 0;
	}
	catch (std::exception& e)
	{
		std::cerr << "**Exception**: " << e.what() << std::endl;
		return -1;
	}
}
