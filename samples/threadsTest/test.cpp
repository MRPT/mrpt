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
