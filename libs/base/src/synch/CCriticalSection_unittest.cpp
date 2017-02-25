/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/synch.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/random.h>
#include <mrpt/system.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::synch;
using namespace mrpt::random;
using namespace mrpt::system;
using namespace std;


struct TAux
{
	bool terminated;

	void (*m_func)(void);

	void run(int ) 
	{
		if (m_func) (*m_func)(); else throw std::runtime_error("functor is NULL!!");
		terminated = true;
	}

	TAux( void (*func)(void) ) : terminated(false),m_func(func) {}
};

void launchTestWithTimeout(void (*func)(void), double timeout_secs, const std::string &fail_msg )
{
	TAux obj(func);
	//mrpt::system::TThreadHandle th = 
	mrpt::system::createThreadFromObjectMethod<TAux>( &obj, &TAux::run, 0 );

	mrpt::utils::CTicTac tim;
	tim.Tic();
	
	while (!obj.terminated && tim.Tac()<timeout_secs) {
		mrpt::system::sleep(1);
	}

	if (!obj.terminated)
		EXPECT_TRUE(false) << "Thread didn't finished in timeout! While testing: " << fail_msg;
}


// TEST 1: Just test if creating a CS doesn't launch an exception or lock.
// ----------------------------------------------------------------------------
void my_CriticalSections_Simple()
{
	CCriticalSection  cs;
	cs.enter();
	cs.leave();
}
TEST(Synch, CriticalSections_Simple )
{
	launchTestWithTimeout(my_CriticalSections_Simple, 5.0, "CriticalSections_Simple");
}

// TEST 2: Assure that a double-enter in a CS will raise an exception
// ----------------------------------------------------------------------------
void my_CriticalSections_NoDoubleEnter()
{
	CCriticalSection  cs;
	cs.enter();
	try{
		cs.enter(); // Must fail!
		// Shouldn't reach here.
		EXPECT_TRUE(false) << "Fail to detect a double 'enter()' into a critical section!\n";
	}
	catch(std::exception&)
	{
		// OK
	}
}
TEST(Synch, CriticalSections_NoDoubleEnter)
{
	launchTestWithTimeout(my_CriticalSections_NoDoubleEnter, 2.0, "CriticalSections_NoDoubleEnter");
}


// TEST 3: Test with several threads competing for a CS:
// ----------------------------------------------------------------------------
CCriticalSection  csCounter;
int counter = 0;

void thread_example(int id)
{
	MRPT_UNUSED_PARAM(id);
	try
	{
		{
			CCriticalSectionLocker lock(&csCounter);
			counter++;
		}

		double delay = randomGenerator.drawUniform(0.1,1.6);

		//printf("[thread_example %i, ID:%lu] Started, will run for %f seconds\n", id, getCurrentThreadId(), delay);

		mrpt::system::sleep( delay*1000 );

		//int remaining;
		{
			CCriticalSectionLocker lock(&csCounter);
			//remaining = 
			--counter;
		}
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

void my_CriticalSections_Multi()
{
	randomGenerator.randomize();

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
TEST(Synch, CriticalSections_Multi)
{
	launchTestWithTimeout(my_CriticalSections_Multi, 6.0, "CriticalSections_Multi");
}

