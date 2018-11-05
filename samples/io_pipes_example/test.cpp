/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/io/CPipe.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CArchive.h>
#include <iostream>
#include <thread>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::io;
using namespace std;

void thread_reader(CPipeReadEndPoint& read_pipe)
{
	try
	{
		std::cout << "[thread_reader ID:" << std::this_thread::get_id()
				  << "] Started." << std::endl;

		// Simple read commands:
		size_t len = 0;
		char buf[100];
		read_pipe.Read(&len, sizeof(len));
		read_pipe.Read(buf, len);
		buf[len] = 0;

		cout << "RX: " << buf << endl;

		// Read MRPT object from a pipe:
		// *Note*: If the object class is known in advance, one can avoid smart
		// pointers with ReadObject(&existingObj)
		auto arch = mrpt::serialization::archiveFrom(read_pipe);
		auto obj = arch.ReadObject();
		if (IS_CLASS(obj, CPose3D))
		{
			CPose3D::Ptr ptrPose = std::dynamic_pointer_cast<CPose3D>(obj);
			cout << "RX pose: " << *ptrPose << endl;
		}

		printf("[thread_reader] Finished.\n");
	}
	catch (const std::exception& e)
	{
		cerr << e.what() << endl;
	}
}

void thread_writer(CPipeWriteEndPoint& write_pipe)
{
	try
	{
		std::cout << "[thread_writer ID:" << std::this_thread::get_id()
				  << "] Started." << std::endl;

		// Simple write commands:
		const char* str = "Hello world!";
		size_t len = strlen(str);
		write_pipe.Write(&len, sizeof(len));
		write_pipe.Write(str, len);

		// Send MRPT objects:
		// *NOTE*: For efficiency, one should first write to an intermediary
		// mrpt::utils::CMemoryChunk to write only once to the pipe.
		mrpt::poses::CPose3D pose(1, 2, 3, 0.1, 0.2, 0.3);
		auto arch = mrpt::serialization::archiveFrom(write_pipe);
		arch.WriteObject(&pose);

		printf("[thread_writer] Finished.\n");
	}
	catch (const std::exception& e)
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
	std::unique_ptr<CPipeReadEndPoint> read_pipe;
	std::unique_ptr<CPipeWriteEndPoint> write_pipe;

	CPipe::createPipe(read_pipe, write_pipe);

	// And send the two end-points to two threads:
	std::thread hT1(thread_reader, std::ref(*read_pipe));
	std::thread hT2(thread_writer, std::ref(*write_pipe));

	// Wait for the threads to end.
	hT1.join();
	hT2.join();
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
	}
	catch (const std::exception& e)
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
