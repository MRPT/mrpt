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
#include <mrpt/random.h>
#include <mrpt/synch.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::synch;
using namespace mrpt::system;
using namespace mrpt::random;
using namespace std;


CCriticalSection  csCounter;
int counter = 0;

void thread_example(int id)
{
	try
	{
		{
			CCriticalSectionLocker lock(&csCounter);
			counter++;
		}

		double delay = randomGenerator.drawUniform(3.0,10.0);

		printf("[thread_example %i, ID:%lu] Started, will run for %f seconds\n", id, getCurrentThreadId(), delay);

		mrpt::system::sleep( delay*1000 );

		int remaining;
		{
			CCriticalSectionLocker lock(&csCounter);
			remaining = --counter;
		}

		time_t	timcr,timex;
		double	tim;
		getCurrentThreadTimes( timcr,timex, tim );

		printf("[thread_example %i] Finished... %i still alive (%fms CPU time)\n", id, remaining, tim*1e3);
	}
	catch(std::exception &e)
	{
		cerr << e.what() << endl;
	}
	catch(...)
	{
		printf("[thread_example] Runtime error!\n");
	}
}

// ------------------------------------------------------
//				ThreadsTest
// ------------------------------------------------------
void ThreadsTest()
{
	randomGenerator.randomize();

	cout << "# of processors detected by mrpt::system::getNumberOfProcessors(): " <<
        mrpt::system::getNumberOfProcessors() << endl << endl;

	for (int i=1;i<=10;i++)
		createThread( thread_example, i );

	// Wait all threads exit:
	int cnt;
	do
	{
		sleep(10);
		csCounter.enter();
		cnt = counter;
		csCounter.leave();
	} while (cnt);
}



void thread_example2(int id)
{
	try
	{
		printf("[thread_example2 %i, ID:%lu] Started, trying to get into semaphore...\n", id, getCurrentThreadId());

		CSemaphore  sem(0,0,"mrpt-demo-sem1");

		sem.waitForSignal();

		double delay = randomGenerator.drawUniform(3.0,10.0);
		printf("[thread_example2 %i, ID:%lu] I'm in. Delaying %f seconds...\n", id, getCurrentThreadId(), delay);
		mrpt::system::sleep( delay*1000 );

		sem.release();

		printf("[thread_example2 %i] Finished\n", id);
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

// ------------------------------------------------------
//				ThreadsTest2
// ------------------------------------------------------
void ThreadsTest2()
{
	std::vector<TThreadHandle>  threads;

	// Create a named semaphore:
	CSemaphore  sem(3 /*init val*/,50 /*max val*/,"mrpt-demo-sem1");

	for (int i=1;i<=10;i++)
		threads.push_back( createThread( thread_example2, i ) );

	// Wait all threads exit:
	for (size_t i=0;i<threads.size();i++)
		joinThread( threads[i] );
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		cout << " =============== TEST 1: CCriticalSection ================\n";
		ThreadsTest();

		cout << "\n =============== TEST 2: CSemaphore ================\n";
		ThreadsTest2();

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
