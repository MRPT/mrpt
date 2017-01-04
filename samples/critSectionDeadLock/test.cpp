/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/utils.h>
#include <mrpt/synch/CCriticalSection.h>

using namespace mrpt;
using namespace mrpt::synch;
using namespace mrpt::utils;
using namespace std;

CCriticalSection  csTest("csTest");

CStdOutStream	myOutStream;

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		csTest.m_debugOut = &myOutStream;

		cout << "Part 1: Normal usage, we'll lock, then unlock the critical section" << endl << endl;
		{
			synch::CCriticalSectionLocker  locker(&csTest);
			cout << "I possess the crit. section..." << endl;
		}

		cout << endl << "Part 2: Bad usage, we'll lock, then lock again the critical section" << endl << endl;
		{
			synch::CCriticalSectionLocker  locker(&csTest);
			csTest.enter();

			cout << "This message shouldn't appear, an exception raised before instead!!" << endl;
		}

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
