/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "apps-precomp.h"  // Precompiled headers
//

#include <mrpt/config/CConfigFile.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/img/TStereoCamera.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationStereoImages.h>

#include "rawlog-edit-declarations.h"

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::apps;
using namespace mrpt::io;
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
		TOutputRawlogCreator outrawlog;

		string target_label;

		// Monocular:
		std::optional<mrpt::img::TCamera> monoCam;
		// Stereo:
		std::optional<mrpt::img::TStereoCamera> stereoCam;
		// 3D depth:
		std::optional<mrpt::img::TCamera> depthCam, depthIntensity;

	   public:
		size_t m_changedCams;

		CRawlogProcessor_CamParams(
			CFileGZInputStream& in_rawlog, TCLAP::CmdLine& cmdline,
			bool Verbose)
			: CRawlogProcessorOnEachObservation(in_rawlog, cmdline, Verbose)
		{
			m_changedCams = 0;

			// Load .ini file with poses:
			string str;
			getArgValue<string>(cmdline, "camera-params", str);

			vector<string> lstTokens;
			tokenize(str, ",", lstTokens);
			if (lstTokens.size() != 2)
				throw std::runtime_error(
					"--camera-params op: argument must be in the format: "
					"--camera-params LABEL,file.ini");

			target_label = lstTokens[0];

			const string& fil = lstTokens[1];
			if (!fileExists(fil))
				THROW_EXCEPTION_FMT(
					"--camera-params op: config file can't be open: '%s'",
					fil.c_str());

			// Load:
			mrpt::config::CConfigFile cfg(fil);

			if (cfg.sectionExists("CAMERA_PARAMS"))
			{
				// Monocular:
				monoCam.emplace();
				monoCam->loadFromConfigFile("CAMERA_PARAMS", cfg);

				VERBOSE_COUT
					<< "Found camera configuration file type: Monocular.\n";
			}
			else if (
				cfg.sectionExists("CAMERA_PARAMS_LEFT") &&
				cfg.sectionExists("CAMERA_PARAMS_RIGHT"))
			{
				// Stereo:
				stereoCam.emplace();
				stereoCam->loadFromConfigFile("CAMERA_PARAMS", cfg);

				VERBOSE_COUT
					<< "Found camera configuration file type: Stereo.\n";
			}
			else if (
				cfg.sectionExists("DEPTH_CAM_PARAMS") &&
				cfg.sectionExists("INTENSITY_CAM_PARAMS"))
			{
				// Depth:
				depthCam.emplace();
				depthCam->loadFromConfigFile("DEPTH_CAM_PARAMS", cfg);

				depthIntensity.emplace();
				depthIntensity->loadFromConfigFile("INTENSITY_CAM_PARAMS", cfg);

				VERBOSE_COUT
					<< "Found camera configuration file type: Depth.\n";
			}
			else
			{
				THROW_EXCEPTION(
					"None of the expected INI file sections found: could not "
					"identify "
					"camera type.");
			}
		}

		bool processOneObservation(CObservation::Ptr& obs) override
		{
			ASSERT_(obs);

			if (!strCmpI(obs->sensorLabel, target_label)) return true;

			if (auto obsMono =
					std::dynamic_pointer_cast<CObservationImage>(obs);
				obsMono)
			{
				obsMono->cameraParams = *monoCam;
				m_changedCams++;
			}
			else if (auto obsStereo =
						 std::dynamic_pointer_cast<CObservationStereoImages>(
							 obs);
					 obsStereo)
			{
				obsStereo->setStereoCameraParams(*stereoCam);
				m_changedCams++;
			}
			else if (auto obsDepth =
						 std::dynamic_pointer_cast<CObservation3DRangeScan>(
							 obs);
					 obsDepth)
			{
				obsDepth->cameraParams = *depthCam;
				obsDepth->cameraParamsIntensity = *depthIntensity;
				m_changedCams++;
			}
			else
			{
				THROW_EXCEPTION_FMT(
					"Observation with matching sensorLabel '%s' found, but I "
					"do not know how to change its camera parameters (class: "
					"'%s')",
					target_label.c_str(), obs->GetRuntimeClass()->className);
			}

			return true;
		}

		// This method can be reimplemented to save the modified object to an
		// output stream.
		void OnPostProcess(
			mrpt::obs::CActionCollection::Ptr& actions,
			mrpt::obs::CSensoryFrame::Ptr& SF,
			mrpt::obs::CObservation::Ptr& obs) override
		{
			ASSERT_((actions && SF) || obs);
			if (actions) (*outrawlog.out_rawlog) << actions << SF;
			else
				(*outrawlog.out_rawlog) << obs;
		}
	};

	// Process
	// ---------------------------------
	CRawlogProcessor_CamParams proc(in_rawlog, cmdline, verbose);
	proc.doProcessRawlog();

	// Dump statistics:
	// ---------------------------------
	VERBOSE_COUT << "Time to process file [s] : " << proc.m_timToParse << "\n";
	VERBOSE_COUT << "Modified observations    : " << proc.m_changedCams << "\n";
}
