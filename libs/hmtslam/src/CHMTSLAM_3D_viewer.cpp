/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hmtslam-precomp.h" // Precomp header

#include <mrpt/utils/CTicTac.h>
#include <mrpt/random.h>
#include <mrpt/utils/CFileStream.h>
#include <mrpt/system/os.h>

using namespace mrpt::slam;
using namespace mrpt::hmtslam;
using namespace mrpt::utils;

/*---------------------------------------------------------------

						CHMTSLAM_3D_viewer

	Optional 3D real-time viewer within HMT-SLAM

  ---------------------------------------------------------------*/
void CHMTSLAM::thread_3D_viewer(  )
{
	CHMTSLAM	*obj = this;
	CTicTac							tictac;

	try
	{
		// Start thread:
		// -------------------------
		obj->logFmt(mrpt::utils::LVL_DEBUG,"[thread_3D_viewer] Thread started (ID=0x%08lX)\n", mrpt::system::getCurrentThreadId() );

		// --------------------------------------------
		//    The main loop
		//  Executes until termination is signaled
		// --------------------------------------------
		while ( !obj->m_terminateThreads )
		{
			mrpt::system::sleep(100);
		};	// end while execute thread

		// Finish thread:
		// -------------------------
		time_t timCreat,timExit; double timCPU=0;
		try { mrpt::system::getCurrentThreadTimes( timCreat,timExit,timCPU); } catch(...) {};
		obj->logFmt(mrpt::utils::LVL_DEBUG,"[thread_3D_viewer] Thread finished. CPU time used:%.06f secs \n",timCPU);
		obj->m_terminationFlag_3D_viewer = true;

	}
	catch(std::exception &e)
	{
		obj->m_terminationFlag_3D_viewer = true;

		// Release semaphores:

		obj->logFmt(mrpt::utils::LVL_ERROR,"%s", e.what() );

		// DEBUG: Terminate application:
		obj->m_terminateThreads	= true;

	}
	catch(...)
	{
		obj->m_terminationFlag_3D_viewer = true;

		obj->logFmt(mrpt::utils::LVL_ERROR,
			"\n---------------------- EXCEPTION CAUGHT! ---------------------\n"
			" In CHierarchicalMappingFramework::thread_3D_viewer. Unexpected runtime error!!\n");

		// DEBUG: Terminate application:
		obj->m_terminateThreads	= true;
	}

}
