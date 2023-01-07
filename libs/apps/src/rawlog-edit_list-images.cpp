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
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationStereoImages.h>

#include "rawlog-edit-declarations.h"

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::apps;
using namespace std;
using namespace mrpt::io;

// ======================================================================
//		op_list_images
// ======================================================================
DECLARE_OP_FUNCTION(op_list_images)
{
	// A class to do this operation:
	class CRawlogProcessor_ListImages : public CRawlogProcessorOnEachObservation
	{
	   protected:
		string m_out_file;
		std::ofstream m_out;

	   public:
		CRawlogProcessor_ListImages(
			CFileGZInputStream& in_rawlog, TCLAP::CmdLine& cmdline,
			bool Verbose)
			: CRawlogProcessorOnEachObservation(in_rawlog, cmdline, Verbose)
		{
			getArgValue<std::string>(cmdline, "text-file-output", m_out_file);
			VERBOSE_COUT << "Writing list to: " << m_out_file << endl;

			m_out.open(m_out_file.c_str());

			if (!m_out.is_open())
				throw std::runtime_error(
					"list-images: Cannot open output text file.");
		}

		bool processOneObservation(CObservation::Ptr& obs) override
		{
			const string label_time = format(
				"%s_%f", obs->sensorLabel.c_str(),
				timestampTotime_t(obs->timestamp));
			if (IS_CLASS(*obs, CObservationStereoImages))
			{
				CObservationStereoImages::Ptr obsSt =
					std::dynamic_pointer_cast<CObservationStereoImages>(obs);
				// save image to file & convert into external storage:
				if (obsSt->imageLeft.isExternallyStored())
					m_out << obsSt->imageLeft.getExternalStorageFile()
						  << std::endl;

				if (obsSt->imageRight.isExternallyStored())
					m_out << obsSt->imageRight.getExternalStorageFile()
						  << std::endl;
			}
			else if (IS_CLASS(*obs, CObservationImage))
			{
				CObservationImage::Ptr obsIm =
					std::dynamic_pointer_cast<CObservationImage>(obs);

				if (obsIm->image.isExternallyStored())
					m_out << obsIm->image.getExternalStorageFile() << std::endl;
			}
			else if (IS_CLASS(*obs, CObservation3DRangeScan))
			{
				CObservation3DRangeScan::Ptr obs3D =
					std::dynamic_pointer_cast<CObservation3DRangeScan>(obs);

				if (obs3D->intensityImage.isExternallyStored())
					m_out << obs3D->intensityImage.getExternalStorageFile()
						  << std::endl;
			}

			return true;
		}
	};

	// Process
	// ---------------------------------
	CRawlogProcessor_ListImages proc(in_rawlog, cmdline, verbose);
	proc.doProcessRawlog();

	// Dump statistics:
	// ---------------------------------
	VERBOSE_COUT << "Time to process file (sec)        : " << proc.m_timToParse
				 << "\n";
}
