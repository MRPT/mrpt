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
#include <mrpt/poses/CPose3D.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/system/threads.h> // sleep()

using mrpt::utils::DEG2RAD;


//  Convert C string to 0MQ string and send to socket
static int
s_send (void *socket, char *string) {
    zmq_msg_t message;
    zmq_msg_init_size (&message, strlen (string));
    memcpy (zmq_msg_data (&message), string, strlen (string));
    int size = zmq_msg_send (&message, socket, 0);
    zmq_msg_close (&message);
    return (size);
}

int main()
{
	try
	{
		printf ("Starting publisher...\n");

		void *context = zmq_ctx_new ();
		void *pub_sock = zmq_socket (context, ZMQ_PUB);
		int rc = zmq_bind (pub_sock, "tcp://*:5555");
		assert (rc == 0);

		while (1)
		{
			//mrpt::poses::CPose3D  C(0.5f,0.5f,1.5f ,DEG2RAD(-90.0f),DEG2RAD(0),DEG2RAD(-90.0f)  );
			printf("Publishing...\n");
			s_send(pub_sock,"Hey there!");
			mrpt::system::sleep(1000);
		}


		zmq_close (pub_sock);
		zmq_ctx_destroy (context);
		return 0;
	} catch (std::exception &e)
	{
		std::cerr << "**Exception**: " << e.what() << std::endl;
		return -1;
	}
}
