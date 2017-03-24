/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "rawlog-edit-declarations.h"
#include <mrpt/utils/TCamera.h>
#include <mrpt/utils/TStereoCamera.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationStereoImages.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::obs;
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
		mrpt::utils::TCamera        new_cam_params;
		mrpt::utils::TStereoCamera  new_stereo_cam_params;
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
					new_stereo_cam_params.loadFromConfigFile("CAMERA_PARAMS",cfg);
					//cout << new_stereo_cam_params.dumpAsText() << endl;
				}
				catch(std::exception &e) {
					throw std::runtime_error(string("--camera-params op: Error loading monocular camera params:\n")+sErrorCam+string("\nBut also an error found loading stereo config:\n")+string(e.what()) );
				}
			}
			VERBOSE_COUT << "Type of camera configuration file found: " << (is_stereo ? "stereo":"monocular") << "\n";
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
					o->setStereoCameraParams(new_stereo_cam_params);
					m_changedCams++;
				}
			}
			return true;
		}

		// This method can be reimplemented to save the modified object to an output stream.
		virtual void OnPostProcess(
			mrpt::obs::CActionCollectionPtr &actions,
			mrpt::obs::CSensoryFramePtr     &SF,
			mrpt::obs::CObservationPtr      &obs)
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

