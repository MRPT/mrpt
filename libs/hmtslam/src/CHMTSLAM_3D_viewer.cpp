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

#include <mrpt/hmtslam.h> // Precomp header

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
		obj->printf_debug("[thread_3D_viewer] Thread started (ID=0x%08lX)\n", mrpt::system::getCurrentThreadId() );

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
		obj->printf_debug("[thread_3D_viewer] Thread finished. CPU time used:%.06f secs \n",timCPU);
		obj->m_terminationFlag_3D_viewer = true;

	}
	catch(std::exception &e)
	{
		obj->m_terminationFlag_3D_viewer = true;

		// Release semaphores:

		obj->printf_debug( (char*)e.what() );

		// DEBUG: Terminate application:
		obj->m_terminateThreads	= true;

	}
	catch(...)
	{
		obj->m_terminationFlag_3D_viewer = true;

		obj->printf_debug("\n---------------------- EXCEPTION CAUGHT! ---------------------\n");
		obj->printf_debug(" In CHierarchicalMappingFramework::thread_3D_viewer. Unexpected runtime error!!\n");

		// Release semaphores:

		// DEBUG: Terminate application:
		obj->m_terminateThreads	= true;
	}

}
