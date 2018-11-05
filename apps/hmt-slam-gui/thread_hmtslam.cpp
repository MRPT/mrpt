/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "hmt_slam_guiMain.h"

#include <wx/msgdlg.h>

#include <mrpt/system/filesystem.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/serialization/CArchive.h>
#include <memory>

using namespace std;
using namespace mrpt;
using namespace mrpt::serialization;
using namespace mrpt::hmtslam;
using namespace mrpt::img;
using namespace mrpt::slam;
using namespace mrpt::poses;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::maps;
using namespace mrpt::io;

void hmt_slam_guiFrame::thread_HMTSLAM()
{
	try
	{
		cout << "[HMTSLAMGUI_THREAD] Thread alive\n";

		bool is_running_slam = false;

		std::unique_ptr<CFileGZInputStream> fInRawlog;
		std::string OUT_DIR = "./HMTSLAM_OUT";
		unsigned int rawlogEntry = 0;  // step = 0;

		while (true)
		{
			try
			{
				bool end = false;
				const bool old_is_running = is_running_slam;
				if (!m_thread_in_queue.empty())
				{
					TThreadMsg* msg = m_thread_in_queue.get();

					switch (msg->opcode)
					{
						case OP_QUIT_THREAD:
							end = true;
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
				if (end) break;  // end thread

				// If we are running SLAM, read actions/observations and feed
				// them into the SLAM engine:
				// ----------------------------------------------------------------------------------------
				if (is_running_slam && !old_is_running)
				{
					// This is the FIRST iteration:
					fInRawlog.reset();

					// From the text block:
					// CConfigFileMemory  cfg(
					// std::string(edRestParams->GetValue().mb_str()) );
					const string fil = string(
						this->edInputRawlog->GetValue()
							.mb_str());  // cfg.read_string("HMT-SLAM","rawlog_file","");
					if (!mrpt::system::fileExists(fil))
					{
						is_running_slam = false;
						throw std::runtime_error(
							format("Rawlog file not found: %s", fil.c_str()));
					}
					else
					{
						fInRawlog = std::make_unique<CFileGZInputStream>(fil);
						m_hmtslam->logFmt(
							mrpt::system::LVL_INFO, "RAWLOG FILE: \n%s\n",
							fil.c_str());
						OUT_DIR =
							"HMT_SLAM_OUTPUT";  // cfg.read_string("HMT-SLAM","LOG_OUTPUT_DIR",
						// "HMT_SLAM_OUTPUT");

						// Set relative path for externally-stored images in
						// rawlogs:
						string rawlog_images_path = extractFileDirectory(fil);
						rawlog_images_path += "/Images";
						CImage::setImagesPathBase(
							rawlog_images_path);  // Set it.

						rawlogEntry = 0;
						// step = 0;
					}
				}  // end first iteration

				if (!is_running_slam && fInRawlog) fInRawlog.reset();

				if (is_running_slam)
				{
					ASSERT_(fInRawlog);

					// Wait for the mapping framework processed the data
					// ---------------------------------------------------
					if (!m_hmtslam->isInputQueueEmpty())
					{
						std::this_thread::sleep_for(2ms);
						continue;
					}

					// Load next object from the rawlog:
					// ----------------------------------------
					CSerializable::Ptr objFromRawlog;
					try
					{
						archiveFrom(*fInRawlog) >> objFromRawlog;
						rawlogEntry++;
						cout << "[HMT-SLAM-GUI] Read rawlog entry "
							 << rawlogEntry << endl;
					}
					catch (std::exception&)
					{
						is_running_slam = false;
						cout << endl
							 << "=============== END OF RAWLOG FILE: ENDING "
								"HMT-SLAM ==============\n";
						continue;
					}
					catch (...)
					{
						printf("Untyped exception reading rawlog file!!\n");
						break;
					}

					// Process the action and observations:
					// --------------------------------------------
					if (IS_CLASS(objFromRawlog, CActionCollection))
					{
						m_hmtslam->pushAction(
							std::dynamic_pointer_cast<CActionCollection>(
								objFromRawlog));  // Memory will be freed in
						// mapping class
					}
					else if (IS_CLASS(objFromRawlog, CSensoryFrame))
					{
						m_hmtslam->pushObservations(
							std::dynamic_pointer_cast<CSensoryFrame>(
								objFromRawlog));  // Memory will be freed in
						// mapping class
					}
					else if (IS_CLASS(objFromRawlog, CObservation))
					{
						m_hmtslam->pushObservation(
							std::dynamic_pointer_cast<CObservation>(
								objFromRawlog));  // Memory will be freed in
						// mapping class
					}
					else
						THROW_EXCEPTION("Invalid object class from rawlog!!");

				}  // end is_running_slam

				std::this_thread::sleep_for(5ms);
			}
			catch (const std::exception& e)
			{
				cerr << "[HMTSLAMGUI_THREAD] Exception: \n"
					 << mrpt::exception_to_str(e);
			}

		}  // while running
	}
	catch (const std::exception& e)
	{
		cerr << "[HMTSLAMGUI_THREAD] Exception: \n"
			 << mrpt::exception_to_str(e);
	}
	cout << "[HMTSLAMGUI_THREAD] Thread closed\n";
}
