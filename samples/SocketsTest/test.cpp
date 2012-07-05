/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#include <mrpt/utils.h>
#include <mrpt/system.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace std;

#include <mrpt/examples_config.h>
string   myDataDir( MRPT_EXAMPLES_BASE_DIRECTORY + string("imageBasics/") ); // Reuse it's images


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
			img.loadFromFile(myDataDir+string("frame_color.jpg"));

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
			printf("  MSG Length: %u bytes\n", (unsigned int)msg.content.size() );

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
	// mrpt::utils::CImage::DISABLE_JPEG_COMPRESSION = true;
	//mrpt::utils::CImage::SERIALIZATION_JPEG_QUALITY = 90;

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

