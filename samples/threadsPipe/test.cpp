/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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
#include <mrpt/synch.h>
#include <mrpt/poses/CPose3D.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::synch;
using namespace mrpt::system;
using namespace std;


void thread_reader(std::auto_ptr<CPipeReadEndPoint> read_pipe)
{
	try
	{
		printf("[thread_reader ID:%lu] Started.\n", getCurrentThreadId());

		// Simple read commands:
		size_t len=0;
		char buf[100];
		read_pipe->ReadBuffer(&len,sizeof(len));
		read_pipe->ReadBuffer(buf,len);
		buf[len]=0;
		
		cout << "RX: " << buf << endl;

		// Read MRPT object from a pipe:
		// *Note*: If the object class is known in advance, one can avoid smart pointers with ReadObject(&existingObj)
		CSerializablePtr obj = read_pipe->ReadObject();
		if (IS_CLASS(obj,CPose3D))
		{
			CPose3DPtr ptrPose = CPose3DPtr(obj);
			cout << "RX pose: " << *ptrPose << endl;
		}


		printf("[thread_reader] Finished.\n");

	}
	catch(std::exception &e)
	{
		cerr << e.what() << endl;
	}
}

void thread_writer(std::auto_ptr<CPipeWriteEndPoint> write_pipe)
{
	try
	{
		printf("[thread_writer ID:%lu] Started.\n", getCurrentThreadId());

		// Simple write commands:
		const char *str = "Hello world!";
		size_t len = strlen(str);
		write_pipe->WriteBuffer(&len,sizeof(len));
		write_pipe->WriteBuffer(str,len);


		// Send MRPT objects:
		// *NOTE*: For efficiency, one should first write to an intermediary mrpt::utils::CMemoryChunk to write only once to the pipe.
		mrpt::poses::CPose3D pose(1,2,3,0.1,0.2,0.3);
		write_pipe->WriteObject(&pose);

		printf("[thread_writer] Finished.\n");

	}
	catch(std::exception &e)
	{
		cerr << e.what() << endl;
	}
}


// ------------------------------------------------------
//				ThreadsTest
// ------------------------------------------------------
void ThreadsTest()
{
	// Create a pipe:
	std::auto_ptr<CPipeReadEndPoint>  read_pipe; 
	std::auto_ptr<CPipeWriteEndPoint> write_pipe;
	
	CPipe::createPipe(read_pipe,write_pipe);

	// And send the two end-points to two threads:
	mrpt::system::TThreadHandle hT1 = createThread( thread_reader, read_pipe );
	mrpt::system::TThreadHandle hT2 = createThread( thread_writer, write_pipe );

	// Wait for the threads to end.
	mrpt::system::joinThread(hT1);
	mrpt::system::joinThread(hT2);
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		ThreadsTest();

		return 0;
	} catch (std::exception &e)
	{
		std::cout << "MRPT exception caught: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!!");
		return -1;
	}
}
