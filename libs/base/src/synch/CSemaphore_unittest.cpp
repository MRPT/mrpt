/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/synch/CSemaphore.h>
#include <mrpt/random.h>
#include <mrpt/system/threads.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::synch;
using namespace mrpt::random;
using namespace mrpt::system;
using namespace std;

// External in other .cpp
void launchTestWithTimeout(void (*func)(void), double timeout_secs, const std::string &fail_msg ); 


// TEST 1: Test named semaphores
// ----------------------------------------------------------------------------
void sem_thread_example(int id)
{
	try
	{
		//printf("[thread_example2 %i, ID:%lu] Started, trying to get into semaphore...\n", id, getCurrentThreadId());

		CSemaphore  sem(0,1,"mrpt-demo-sem1");

		sem.waitForSignal();

		double delay = randomGenerator.drawUniform(0.1,0.5);
		//printf("[thread_example2 %i, ID:%lu] I'm in. Delaying %f seconds...\n", id, getCurrentThreadId(), delay);
		mrpt::system::sleep( delay*1000 );

		//printf("[thread_example2 %i] Releasing..\n", id);
		sem.release();

	}
	catch(std::exception &e)
	{
		cerr << e.what() << endl;
	}
	catch(...)
	{
		printf("[thread_example2] Runtime error!\n");
	}
}

template <int num_threads,int initial_sem_count>
void my_CSemaphore_named()
{
	std::vector<TThreadHandle>  threads;

	// Create a named semaphore:
	CSemaphore  sem(initial_sem_count /*init val*/,5*num_threads /*max val*/,"mrpt-demo-sem1");

	for (int i=1;i<=num_threads;i++)
		threads.push_back( createThread( sem_thread_example, i ) );

	// Wait all threads exit:
	for (size_t i=0;i<threads.size();i++)
		joinThread( threads[i] );
}
TEST(Synch, CSemaphore_named_6t_1init )
{
	launchTestWithTimeout(my_CSemaphore_named<6,1>, 5.0, "CSemaphore named: #threads=6, init count=1");
}
TEST(Synch, CSemaphore_named_10t_5init )
{
	launchTestWithTimeout(my_CSemaphore_named<10,5>, 5.0, "CSemaphore named: #threads=10, init count=5");
}
