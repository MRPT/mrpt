/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/io/CPipe.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CArchive.h>
#include <thread>
#include <iostream>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::system;
using namespace mrpt::io;
using namespace mrpt::serialization;
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
		auto arch = archiveFrom(read_pipe);
#ifndef HAS_BROKEN_CLANG_STD_VISIT
		auto doprint = [](auto& pose) { cout << "RX pose: " << pose << endl; };
		auto var =
			arch.ReadVariant<mrpt::poses::CPose2D, mrpt::poses::CPose3D>();
		std::visit(doprint, var);
		var = arch.ReadVariant<mrpt::poses::CPose2D, mrpt::poses::CPose3D>();
		std::visit(doprint, var);
#endif

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
		mrpt::poses::CPose3D pose1(1, 2, 3, 1.57, 3.14, 0);
		mrpt::poses::CPose2D pose2(1.0, 2.0, 3.1415);
		std::variant<mrpt::poses::CPose3D, mrpt::poses::CPose2D> var1(
			std::move(pose1));
		std::variant<mrpt::poses::CPose3D, mrpt::poses::CPose2D> var2(
			std::move(pose2));
		auto arch = archiveFrom(write_pipe);
#ifndef HAS_BROKEN_CLANG_STD_VISIT
		arch.WriteVariant(var1);
		arch.WriteVariant(var2);
#endif
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

	using namespace std::chrono_literals;
	std::this_thread::sleep_for(10ms);
	// Wait for the threads to end.
	hT2.join();
	// We need to close this to ensure the pipe gets flushed
	// Remember Unix uses buffered io
	// write_pipe.reset();
	hT1.join();
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
