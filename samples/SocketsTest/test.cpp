/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/utils.h>
#include <mrpt/system.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace std;

#include <mrpt/examples_config.h>
string   myDataDir( MRPT_EXAMPLES_BASE_DIRECTORY + string("imageCorrelation/") ); // Reuse it's images


void thread_server()
{
	try
	{
		printf("[Server] Started\n");

		CServerTCPSocket		server( 15000, "127.0.0.1" , 10, true );
		CClientTCPSocket		*client;

		client = server.accept( 2000 );

		if (client)
		{
			printf("[Server] Conection accepted\n");

			// Load test image:
			CImage	img;
			img.loadFromFile(myDataDir+string("fft2_test_image.jpg"));

			// Send a message with the image:
			CMessage	msg;
			msg.type	= 0x10;
			msg.serializeObject( &img );

			printf("[Server] Sending message...\n");
			client->sendMessage( msg );
			printf("[Server] Message sent!!\n");

			mrpt::system::sleep(50);

			delete client;
		}

		printf("[Server] Finish\n");
	}
	catch(std::exception &e)
	{
		cerr << e.what() << endl;
	}
	catch(...)
	{
		printf("[thread_server] Runtime error!\n");
	}
}


void thread_client()
{
	try
	{
		printf("[Client] Started\n");
		CClientTCPSocket	sock;

		printf("[Client] Connecting\n");

		sock.connect( "127.0.0.1", 15000 );

		printf("[Client] Connected. Waiting for a message...\n");

//		cout << "pending: " << sock.getReadPendingBytes() << endl;
//		mrpt::system::sleep(4000);
//		cout << "pending: " << sock.getReadPendingBytes() << endl;

		CMessage	msg;
		bool ok = sock.receiveMessage( msg, 2000,2000);

		if (!ok)
		{
			printf("[Client] Error receiving message!!\n");
		}
		else
		{
			printf("[Client] Message received OK!:\n");
			printf("  MSG Type: %i\n", msg.type );

			printf("[Client] Parsing image...\n");
			CImage		img;
			msg.deserializeIntoExistingObject( &img );

			printf("[Client] Saving image...\n");
			img.saveToFile("received_frame.jpg");
			printf("[Client] Done!!\n");
		}

		printf("[Client] Finish\n");
	}
	catch(std::exception &e)
	{
		cerr << e.what() << endl;
	}
	catch(...)
	{
		cerr << "[thread_client] Runtime error!" << endl;;
	}
}



// ------------------------------------------------------
//				SocketsTest
// ------------------------------------------------------
void SocketsTest()
{
	mrpt::system::createThread(thread_server);
	mrpt::system::createThread(thread_client);

	mrpt::system::sleep(2000);
	mrpt::system::pause();
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		SocketsTest();
		return 0;
	} catch (std::exception &e)
	{
		cerr << "MRPT exception caught: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!!");
		return -1;
	}
}

