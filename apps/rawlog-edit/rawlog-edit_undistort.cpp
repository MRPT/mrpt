/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include "rawlog-edit-declarations.h"

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::rawlogtools;
using namespace std;
using namespace mrpt::io;

// ======================================================================
//		op_undistort
// ======================================================================
DECLARE_OP_FUNCTION(op_undistort)
{
	// A class to do this operation:
	class CRawlogProcessor_Undistort : public CRawlogProcessorFilterObservations
	{
	   protected:
		string m_out_file;

	   public:
		CRawlogProcessor_Undistort(
			CFileGZInputStream& in_rawlog, TCLAP::CmdLine& cmdline,
			bool Verbose, mrpt::io::CFileGZOutputStream& out_rawlog)
			: CRawlogProcessorFilterObservations(
				  in_rawlog, cmdline, Verbose, out_rawlog)
		{
		}

		bool tellIfThisObsPasses(mrpt::obs::CObservation::Ptr& obs) override
		{
			return true;
		}

		bool processOneObservation(CObservation::Ptr& obs) override
		{
			obs->load();

			mrpt::img::CImage tmp;

			if (auto obsSt =
					std::dynamic_pointer_cast<CObservationStereoImages>(obs);
				obsSt)
			{
				// save image to file & convert into external storage:
				obsSt->imageLeft.undistort(tmp, obsSt->leftCamera);
				obsSt->imageLeft = std::move(tmp);

				obsSt->imageRight.undistort(tmp, obsSt->rightCamera);
				obsSt->imageRight = std::move(tmp);
			}
			else if (auto obsIm =
						 std::dynamic_pointer_cast<CObservationImage>(obs);
					 obsIm)
			{
				obsIm->image.undistort(tmp, obsIm->cameraParams);
				obsIm->image = std::move(tmp);
			}
			else if (auto obs3D =
						 std::dynamic_pointer_cast<CObservation3DRangeScan>(
							 obs);
					 obs3D)
			{
				obs3D->undistort();
			}

			obs->unload();
			return true;
		}
	};

	// Process
	// ---------------------------------
	TOutputRawlogCreator outrawlog;
	CRawlogProcessor_Undistort proc(
		in_rawlog, cmdline, verbose, outrawlog.out_rawlog_io);
	proc.doProcessRawlog();

	// Dump statistics:
	// ---------------------------------
	VERBOSE_COUT << "Time to process file (sec)        : " << proc.m_timToParse
				 << "\n";
}
