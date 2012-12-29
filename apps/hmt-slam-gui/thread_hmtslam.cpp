/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
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


#include "hmt_slam_guiMain.h"

#include <wx/msgdlg.h>

#include <mrpt/utils.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::hmtslam;
using namespace mrpt::slam;
using namespace mrpt::poses;
using namespace mrpt::utils;

void hmt_slam_guiFrame::thread_HMTSLAM()
{
	try
	{
		cout << "[HMTSLAMGUI_THREAD] Thread alive\n";

		bool is_running_slam = false;

		CFileGZInputStream  *fInRawlog = NULL;
		std::string 		OUT_DIR="./HMTSLAM_OUT";
		unsigned int		rawlogEntry = 0; // step = 0;

		while (true)
		{
			try
			{
				bool end=false;
				const bool old_is_running = is_running_slam;
				if (!m_thread_in_queue.empty())
				{
					TThreadMsg *msg = m_thread_in_queue.get();

					switch ( msg->opcode )
					{
						case OP_QUIT_THREAD:
							end=true;
							break;
						case OP_START_SLAM:
							is_running_slam = true;
							break;
						case OP_PAUSE_SLAM:
							is_running_slam = false;
							break;

						default:
							throw std::runtime_error("Unknown OPCODE!");
							break;
					}
					delete msg;
				}
				if (end) break; // end thread

				// If we are running SLAM, read actions/observations and feed them into the SLAM engine:
				// ----------------------------------------------------------------------------------------
				if (is_running_slam && !old_is_running)
				{
					// This is the FIRST iteration:
					if (fInRawlog) delete_safe(fInRawlog);

					// From the text block:
					//CConfigFileMemory  cfg( std::string(edRestParams->GetValue().mb_str()) );
					const string fil =  string(this->edInputRawlog->GetValue().mb_str()); // cfg.read_string("HMT-SLAM","rawlog_file","");
					if (!mrpt::system::fileExists( fil ))
					{
						is_running_slam = false;
						throw std::runtime_error(format("Rawlog file not found: %s",fil.c_str()));
					}
					else
					{
						fInRawlog = new CFileGZInputStream(fil);
						m_hmtslam->printf_debug("RAWLOG FILE: \n%s\n",fil.c_str());
						OUT_DIR = "HMT_SLAM_OUTPUT"; //cfg.read_string("HMT-SLAM","LOG_OUTPUT_DIR", "HMT_SLAM_OUTPUT");

						// Set relative path for externally-stored images in rawlogs:
						string	rawlog_images_path = extractFileDirectory( fil );
						rawlog_images_path+="/Images";
						CImage::IMAGES_PATH_BASE = rawlog_images_path;		// Set it.

						rawlogEntry = 0;
						//step = 0;
					}
				} // end first iteration

				if (!is_running_slam && fInRawlog)
					delete_safe(fInRawlog);

				if (is_running_slam)
				{
					ASSERT_(fInRawlog)

					// Wait for the mapping framework processed the data
					// ---------------------------------------------------
					if (!m_hmtslam->isInputQueueEmpty())
					{
						sleep(2);
						continue;
					}

					// Load next object from the rawlog:
					// ----------------------------------------
					CSerializablePtr objFromRawlog;
					try
					{
						(*fInRawlog) >> objFromRawlog;
						rawlogEntry++;
						cout << "[HMT-SLAM-GUI] Read rawlog entry " << rawlogEntry << endl;
					}
					catch(std::exception &)
					{
						is_running_slam = false;
						cout << endl << "=============== END OF RAWLOG FILE: ENDING HMT-SLAM ==============\n";
						continue;
					}
					catch(...) { printf("Untyped exception reading rawlog file!!\n");break;}

					// Process the action and observations:
					// --------------------------------------------
					if (IS_CLASS(objFromRawlog,CActionCollection))
					{
						m_hmtslam->pushAction( CActionCollectionPtr( objFromRawlog) ); // Memory will be freed in mapping class
					}
					else if (IS_CLASS(objFromRawlog,CSensoryFrame))
					{
						m_hmtslam->pushObservations( CSensoryFramePtr( objFromRawlog) ); // Memory will be freed in mapping class
					}
					else if (IS_CLASS(objFromRawlog,CObservation))
					{
						m_hmtslam->pushObservation( CObservationPtr( objFromRawlog) ); // Memory will be freed in mapping class
					}
					else THROW_EXCEPTION("Invalid object class from rawlog!!")

				} // end is_running_slam

				mrpt::system::sleep(5);
			}
			catch (std::exception &e)
			{
				cerr << "[HMTSLAMGUI_THREAD] Exception: \n" << e.what();
			}

		} // while running

		if (fInRawlog)
			delete_safe(fInRawlog);

	}
	catch (std::exception &e)
	{
		cerr << "[HMTSLAMGUI_THREAD] Exception: \n" << e.what();
	}
	cout << "[HMTSLAMGUI_THREAD] Thread closed\n";
}
