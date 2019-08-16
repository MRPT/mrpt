/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include "rawlog-edit-declarations.h"

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::rawlogtools;
using namespace std;
using namespace mrpt::io;

// ======================================================================
//		op_deexternalize
// ======================================================================
DECLARE_OP_FUNCTION(op_deexternalize)
{
	// A class to do this operation:
	class CRawlogProcessor_DeExternalize
		: public CRawlogProcessorOnEachObservation
	{
	   protected:
		TOutputRawlogCreator outrawlog;

	   public:
		size_t entries_converted;
		size_t entries_skipped;  // Already external

		CRawlogProcessor_DeExternalize(
			CFileGZInputStream& in_rawlog, TCLAP::CmdLine& cmdline,
			bool _verbose)
			: CRawlogProcessorOnEachObservation(in_rawlog, cmdline, _verbose)
		{
			entries_converted = 0;
			entries_skipped = 0;
		}

		bool processOneObservation(CObservation::Ptr& obs) override
		{
			using namespace std::string_literals;

			const string label_time = format(
				"%s_%f", obs->sensorLabel.c_str(),
				timestampTotime_t(obs->timestamp));
			if (IS_CLASS(*obs, CObservationStereoImages))
			{
				auto obsSt =
					std::dynamic_pointer_cast<CObservationStereoImages>(obs);

				if (obsSt->imageLeft.isExternallyStored())
				{
					obsSt->imageLeft.loadFromFile(
						obsSt->imageLeft.getExternalStorageFileAbsolutePath());
					entries_converted++;
				}
				else
					entries_skipped++;

				if (obsSt->imageRight.isExternallyStored())
				{
					obsSt->imageRight.loadFromFile(
						obsSt->imageRight.getExternalStorageFileAbsolutePath());
					entries_converted++;
				}
				else
					entries_skipped++;
			}
			else if (IS_CLASS(*obs, CObservationImage))
			{
				auto obsIm = std::dynamic_pointer_cast<CObservationImage>(obs);

				if (obsIm->image.isExternallyStored())
				{
					obsIm->image.loadFromFile(
						obsIm->image.getExternalStorageFileAbsolutePath());
					entries_converted++;
				}
				else
					entries_skipped++;
			}
			else if (IS_CLASS(*obs, CObservationPointCloud))
			{
				auto obsPc =
					std::dynamic_pointer_cast<CObservationPointCloud>(obs);

				if (obsPc->isExternallyStored())
				{
					obsPc->load();
					obsPc->overrideExternalStorageFormatFlag(
						CObservationPointCloud::ExternalStorageFormat::None);
					entries_converted++;
				}
				else
					entries_skipped++;
			}
			else if (IS_CLASS(*obs, CObservation3DRangeScan))
			{
				auto obs3D =
					std::dynamic_pointer_cast<CObservation3DRangeScan>(obs);

				if (obs3D->rangeImage_isExternallyStored() ||
					obs3D->points3D_isExternallyStored() ||
					obs3D->confidenceImage.isExternallyStored() ||
					obs3D->intensityImage.isExternallyStored())
					entries_converted++;
				else
					entries_skipped++;

				// Intensity channel:
				if (obs3D->hasIntensityImage &&
					obs3D->intensityImage.isExternallyStored())
				{
					obs3D->intensityImage.loadFromFile(
						obs3D->intensityImage
							.getExternalStorageFileAbsolutePath());
				}

				// Confidence channel:
				if (obs3D->hasConfidenceImage &&
					obs3D->confidenceImage.isExternallyStored())
				{
					obs3D->confidenceImage.loadFromFile(
						obs3D->confidenceImage
							.getExternalStorageFileAbsolutePath());
				}

				// Range image & 3D points:
				obs3D->load();
				obs3D->rangeImage_forceResetExternalStorage();
				obs3D->points3D_overrideExternalStoredFlag(false);
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
			if (actions)
				(*outrawlog.out_rawlog) << actions << SF;
			else
				(*outrawlog.out_rawlog) << obs;
		}
	};

	// Process
	// ---------------------------------
	CRawlogProcessor_DeExternalize proc(in_rawlog, cmdline, verbose);
	proc.doProcessRawlog();

	// Dump statistics:
	// ---------------------------------
	VERBOSE_COUT << "Time to process file (sec)        : " << proc.m_timToParse
				 << "\n";
	VERBOSE_COUT << "Entries converted                 : "
				 << proc.entries_converted << "\n";
	VERBOSE_COUT << "Entries skipped (already external): "
				 << proc.entries_skipped << "\n";
}
