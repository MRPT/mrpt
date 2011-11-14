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

#include "rawlog-edit-declarations.h"

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::system;
using namespace mrpt::rawlogtools;
using namespace std;


// ======================================================================
//		op_camera_params
// ======================================================================
DECLARE_OP_FUNCTION(op_camera_params)
{
	// A class to do this operation:
	class CRawlogProcessor_CamParams : public CRawlogProcessorOnEachObservation
	{
	protected:
		TOutputRawlogCreator	outrawlog;

		string   target_label;
		mrpt::utils::TCamera    new_cam_params;
		mrpt::utils::TCamera    new_cam_params_left, new_cam_params_right;
		bool   is_stereo;

	public:
		size_t  m_changedCams;

		CRawlogProcessor_CamParams(CFileGZInputStream &in_rawlog, TCLAP::CmdLine &cmdline, bool verbose) :
			CRawlogProcessorOnEachObservation(in_rawlog,cmdline,verbose)
		{
			m_changedCams = 0;

			// Load .ini file with poses:
			string   str;
			getArgValue<string>(cmdline,"camera-params",str);

			vector<string> lstTokens;
			tokenize(str,",",lstTokens);
			if (lstTokens.size()!=2)
				throw std::runtime_error("--camera-params op: argument must be in the format: --camera-params LABEL,file.ini");

			target_label = lstTokens[0];

			const string &fil = lstTokens[1];
			if (!fileExists(fil)) throw std::runtime_error(string("--camera-params op: config file can't be open:")+fil);

			// Load:
			CConfigFile  cfg( fil );
			is_stereo = true;
			string sErrorCam;
			try {
				new_cam_params.loadFromConfigFile("CAMERA_PARAMS",cfg);
				is_stereo = false;
			}
			catch(std::exception &e) {
				sErrorCam = e.what();
			}

			if (!sErrorCam.empty())
			{
				// Try with STEREO PARAMS:
				try {
					new_cam_params_left.loadFromConfigFile("CAMERA_PARAMS_LEFT",cfg);
					new_cam_params_right.loadFromConfigFile("CAMERA_PARAMS_RIGHT",cfg);
				}
				catch(std::exception &e) {
					throw std::runtime_error(string("--camera-params op: Error loading monocular camera params:\n")+sErrorCam+string("\nBut also an error found loading stereo config:\n")+string(e.what()) );
				}
			}
		}

		bool processOneObservation(CObservationPtr  &obs)
		{
			if ( strCmpI(obs->sensorLabel,target_label))
			{
				if (IS_CLASS(obs,CObservationImage))
				{
					CObservationImagePtr o = CObservationImagePtr(obs);
					o->cameraParams = new_cam_params;
					m_changedCams++;
				}
				else
				if (IS_CLASS(obs,CObservationStereoImages))
				{
					CObservationStereoImagesPtr o = CObservationStereoImagesPtr(obs);
					o->leftCamera = new_cam_params_left;
					o->rightCamera = new_cam_params_right;
					m_changedCams++;
				}
			}
			return true;
		}

		// This method can be reimplemented to save the modified object to an output stream.
		virtual void OnPostProcess(
			mrpt::slam::CActionCollectionPtr &actions,
			mrpt::slam::CSensoryFramePtr     &SF,
			mrpt::slam::CObservationPtr      &obs)
		{
			ASSERT_((actions && SF) || obs)
			if (actions)
					outrawlog.out_rawlog << actions << SF;
			else	outrawlog.out_rawlog << obs;
		}

	};

	// Process
	// ---------------------------------
	CRawlogProcessor_CamParams proc(in_rawlog,cmdline,verbose);
	proc.doProcessRawlog();

	// Dump statistics:
	// ---------------------------------
	VERBOSE_COUT << "Time to process file (sec)        : " << proc.m_timToParse << "\n";
	VERBOSE_COUT << "Number of modified entries        : " << proc.m_changedCams << "\n";

}

