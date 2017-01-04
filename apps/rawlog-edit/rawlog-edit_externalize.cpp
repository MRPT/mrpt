/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "rawlog-edit-declarations.h"
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationStereoImages.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::rawlogtools;
using namespace std;


// ======================================================================
//		op_externalize
// ======================================================================
DECLARE_OP_FUNCTION(op_externalize)
{
	// A class to do this operation:
	class CRawlogProcessor_Externalize : public CRawlogProcessorOnEachObservation
	{
	protected:
		TOutputRawlogCreator	outrawlog;

		string imgFileExtension;
		string outDir;

	public:
		size_t  entries_converted;
		size_t  entries_skipped; // Already external

		CRawlogProcessor_Externalize(CFileGZInputStream &in_rawlog, TCLAP::CmdLine &cmdline, bool verbose) :
			CRawlogProcessorOnEachObservation(in_rawlog,cmdline,verbose)
		{
			entries_converted = 0;
			entries_skipped  = 0;
			getArgValue<string>(cmdline,"image-format",imgFileExtension);

			mrpt::obs::CObservation3DRangeScan::EXTERNALS_AS_TEXT = isFlagSet(cmdline,"txt-externals");

			// Create the default "/Images" directory.
			const string out_rawlog_basedir = extractFileDirectory(outrawlog.out_rawlog_filename);

			outDir = (out_rawlog_basedir.empty() ? string() : (out_rawlog_basedir+string("/") )) + extractFileName(outrawlog.out_rawlog_filename) + string("_Images");
			if (directoryExists(outDir))
				throw runtime_error(string("*ABORTING*: Output directory for images already exists: ") + outDir + string("\n. Select a different output path or remove the directory.") );

			VERBOSE_COUT << "Creating directory: " << outDir << endl;

			mrpt::system::createDirectory( outDir );
			if (!fileExists(outDir))
				throw runtime_error(string("*ABORTING*: Couldn't create directory: ") + outDir );

			// Add the final /
			outDir+="/";
		}

		bool processOneObservation(CObservationPtr  &obs)
		{
			const string label_time = format("%s_%f", obs->sensorLabel.c_str(), timestampTotime_t(obs->timestamp) );
			if (IS_CLASS(obs, CObservationStereoImages ) )
			{
				CObservationStereoImagesPtr obsSt = CObservationStereoImagesPtr(obs);
				// save image to file & convert into external storage:
				if (!obsSt->imageLeft.isExternallyStored())
				{
					const string fileName = string("img_") + label_time + string("_left.") + imgFileExtension;
					obsSt->imageLeft.saveToFile( outDir + fileName );
					obsSt->imageLeft.setExternalStorage( fileName );
					entries_converted++;
				}
				else entries_skipped++;

				if (!obsSt->imageRight.isExternallyStored())
				{
					const string fileName = string("img_") + label_time + string("_right.") + imgFileExtension;
					obsSt->imageRight.saveToFile( outDir + fileName );
					obsSt->imageRight.setExternalStorage( fileName );
					entries_converted++;
				}
				else entries_skipped++;
			}
			else if (IS_CLASS(obs, CObservationImage ) )
			{
				CObservationImagePtr obsIm = CObservationImagePtr(obs);

				if (!obsIm->image.isExternallyStored())
				{
					const string fileName = string("img_") + label_time +string(".")+ imgFileExtension;
					obsIm->image.saveToFile( outDir + fileName );
					obsIm->image.setExternalStorage( fileName );
					entries_converted++;
				}
				else entries_skipped++;
			}
			else if (IS_CLASS(obs, CObservation3DRangeScan ) )
			{
				CObservation3DRangeScanPtr obs3D = CObservation3DRangeScanPtr(obs);

				// save images to file & convert into external storage:
				// Intensity channel:
				if (obs3D->hasIntensityImage && !obs3D->intensityImage.isExternallyStored())
				{
					const string fileName = string("3DCAM_") + label_time + string("_INT.") + imgFileExtension;
					obs3D->intensityImage.saveToFile( outDir + fileName );
					obs3D->intensityImage.setExternalStorage( fileName );
					entries_converted++;
				}
				else entries_skipped++;

				// Confidence channel:
				if (obs3D->hasConfidenceImage && !obs3D->confidenceImage.isExternallyStored())
				{
					const string fileName = string("3DCAM_") + label_time + string("_CONF.") + imgFileExtension;
					obs3D->confidenceImage.saveToFile( outDir + fileName );
					obs3D->confidenceImage.setExternalStorage( fileName );
					entries_converted++;
				}
				else entries_skipped++;

				// 3D points:
				if (obs3D->hasPoints3D && !obs3D->points3D_isExternallyStored())
				{
					const string fileName = string("3DCAM_") + label_time + string("_3D.bin");
					obs3D->points3D_convertToExternalStorage(fileName, outDir);
					entries_converted++;
				}
				else entries_skipped++;

				// Range image:
				if (obs3D->hasRangeImage  && !obs3D->rangeImage_isExternallyStored())
				{
					const string fileName = string("3DCAM_") + label_time + string("_RANGES.bin");
					obs3D->rangeImage_convertToExternalStorage(fileName, outDir);
					entries_converted++;
				}
				else entries_skipped++;
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
	CRawlogProcessor_Externalize proc(in_rawlog,cmdline,verbose);
	proc.doProcessRawlog();

	// Dump statistics:
	// ---------------------------------
	VERBOSE_COUT << "Time to process file (sec)        : " << proc.m_timToParse << "\n";
	VERBOSE_COUT << "Entries converted                 : " << proc.entries_converted << "\n";
	VERBOSE_COUT << "Entries skipped (already external): " << proc.entries_skipped << "\n";

}

