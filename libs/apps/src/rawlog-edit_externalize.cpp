/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "apps-precomp.h"  // Precompiled headers
//
#include <mrpt/io/lazy_load_path.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationStereoImages.h>

#include "rawlog-edit-declarations.h"

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::apps;
using namespace std;
using namespace mrpt::io;

// ======================================================================
//		op_externalize
// ======================================================================
DECLARE_OP_FUNCTION(op_externalize)
{
	// A class to do this operation:
	class CRawlogProcessor_Externalize
		: public CRawlogProcessorOnEachObservation
	{
	   protected:
		TOutputRawlogCreator outrawlog;

		string imgFileExtension;
		string outDir;
		bool m_external_txt{false};

	   public:
		size_t entries_converted;
		size_t entries_skipped;	 // Already external

		CRawlogProcessor_Externalize(
			CFileGZInputStream& in_rawlog, TCLAP::CmdLine& cmdline,
			bool _verbose)
			: CRawlogProcessorOnEachObservation(in_rawlog, cmdline, _verbose)
		{
			entries_converted = 0;
			entries_skipped = 0;
			getArgValue<string>(cmdline, "image-format", imgFileExtension);
			m_external_txt = isFlagSet(cmdline, "txt-externals");

			mrpt::obs::CObservation3DRangeScan::EXTERNALS_AS_TEXT(
				m_external_txt);

			// Create the default "/Images" directory.
			const string out_rawlog_basedir =
				extractFileDirectory(outrawlog.out_rawlog_filename);

			outDir = (out_rawlog_basedir.empty()
						  ? string()
						  : (out_rawlog_basedir + string("/"))) +
				extractFileName(outrawlog.out_rawlog_filename) +
				string("_Images");
			if (directoryExists(outDir))
				throw runtime_error(
					string("*ABORTING*: Output directory for images already "
						   "exists: ") +
					outDir +
					string("\n. Select a different output path or "
						   "remove the directory."));

			VERBOSE_COUT << "Creating directory: " << outDir << endl;

			mrpt::system::createDirectory(outDir);
			if (!fileExists(outDir))
				throw runtime_error(
					string("*ABORTING*: Couldn't create directory: ") + outDir);

			// Add the final /
			outDir += "/";

			// Establish as reference external path base:
			mrpt::io::setLazyLoadPathBase(outDir);
		}

		bool processOneObservation(CObservation::Ptr& obs) override
		{
			using namespace std::string_literals;

			const string label_time = format(
				"%s_%.09f", obs->sensorLabel.c_str(),
				timestampTotime_t(obs->timestamp));
			if (IS_CLASS(*obs, CObservationStereoImages))
			{
				CObservationStereoImages::Ptr obsSt =
					std::dynamic_pointer_cast<CObservationStereoImages>(obs);
				// save image to file & convert into external storage:
				if (!obsSt->imageLeft.isExternallyStored())
				{
					const string fileName =
						"img_"s + label_time + "_left."s + imgFileExtension;
					obsSt->imageLeft.saveToFile(outDir + fileName);
					obsSt->imageLeft.setExternalStorage(fileName);
					entries_converted++;
				}
				else
					entries_skipped++;

				if (!obsSt->imageRight.isExternallyStored())
				{
					const string fileName =
						"img_"s + label_time + "_right."s + imgFileExtension;
					obsSt->imageRight.saveToFile(outDir + fileName);
					obsSt->imageRight.setExternalStorage(fileName);
					entries_converted++;
				}
				else
					entries_skipped++;
			}
			else if (IS_CLASS(*obs, CObservationImage))
			{
				CObservationImage::Ptr obsIm =
					std::dynamic_pointer_cast<CObservationImage>(obs);

				if (!obsIm->image.isExternallyStored())
				{
					const string fileName =
						"img_"s + label_time + "."s + imgFileExtension;
					obsIm->image.saveToFile(outDir + fileName);
					obsIm->image.setExternalStorage(fileName);
					entries_converted++;
				}
				else
					entries_skipped++;
			}
			else if (IS_CLASS(*obs, CObservationPointCloud))
			{
				auto obsPc =
					std::dynamic_pointer_cast<CObservationPointCloud>(obs);

				if (obsPc->pointcloud && !obsPc->isExternallyStored())
				{
					const string fileName = "pc_"s + label_time +
						(m_external_txt ? ".txt"s : ".bin"s);
					obsPc->setAsExternalStorage(
						fileName,
						m_external_txt
							? CObservationPointCloud::ExternalStorageFormat::
								  PlainTextFile
							: CObservationPointCloud::ExternalStorageFormat::
								  MRPT_Serialization);

					obsPc->unload();  // this actually saves the data to disk
					entries_converted++;
				}
				else
					entries_skipped++;
			}
			else if (IS_CLASS(*obs, CObservation3DRangeScan))
			{
				CObservation3DRangeScan::Ptr obs3D =
					std::dynamic_pointer_cast<CObservation3DRangeScan>(obs);

				// save images to file & convert into external storage:
				// Intensity channel:
				if (obs3D->hasIntensityImage &&
					!obs3D->intensityImage.isExternallyStored())
				{
					const string fileName =
						"3DCAM_"s + label_time + "_INT."s + imgFileExtension;
					obs3D->intensityImage.saveToFile(outDir + fileName);
					obs3D->intensityImage.setExternalStorage(fileName);
					entries_converted++;
				}
				else
					entries_skipped++;

				// Confidence channel:
				if (obs3D->hasConfidenceImage &&
					!obs3D->confidenceImage.isExternallyStored())
				{
					const string fileName =
						"3DCAM_"s + label_time + "_CONF."s + imgFileExtension;
					obs3D->confidenceImage.saveToFile(outDir + fileName);
					obs3D->confidenceImage.setExternalStorage(fileName);
					entries_converted++;
				}
				else
					entries_skipped++;

				// 3D points:
				if (obs3D->hasPoints3D && !obs3D->points3D_isExternallyStored())
				{
					const string fileName = "3DCAM_"s + label_time + "_3D.bin"s;
					obs3D->points3D_convertToExternalStorage(fileName, outDir);
					entries_converted++;
				}
				else
					entries_skipped++;

				// Range image:
				if (obs3D->hasRangeImage &&
					!obs3D->rangeImage_isExternallyStored())
				{
					const string fileName =
						"3DCAM_"s + label_time + "_RANGES.bin"s;
					obs3D->rangeImage_convertToExternalStorage(
						fileName, outDir);
					entries_converted++;
				}
				else
					entries_skipped++;
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
	CRawlogProcessor_Externalize proc(in_rawlog, cmdline, verbose);
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
