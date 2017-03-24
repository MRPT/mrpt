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
//		op_rename_externals
// ======================================================================
DECLARE_OP_FUNCTION(op_rename_externals)
{
	// A class to do this operation:
	class CRawlogProcessor_RenameExternals : public CRawlogProcessorOnEachObservation
	{
	protected:
		TOutputRawlogCreator	outrawlog;

		string	imgFileExtension;
		string 	outDir;

	public:
		size_t  entries_converted;
		size_t  entries_skipped; // Already external

		CRawlogProcessor_RenameExternals(CFileGZInputStream &in_rawlog, TCLAP::CmdLine &cmdline, bool verbose) :
			CRawlogProcessorOnEachObservation(in_rawlog,cmdline,verbose)
		{
			entries_converted = 0;
			entries_skipped  = 0;

			getArgValue<string>(cmdline,"image-format",imgFileExtension);

		}

		bool processOneObservation(CObservationPtr  &obs)
		{
			map<string,string> files2rename;

			const string label_time = format("%s_%f", obs->sensorLabel.c_str(), timestampTotime_t(obs->timestamp) );
			if (IS_CLASS(obs, CObservationStereoImages ) )
			{
				CObservationStereoImagesPtr obsSt = CObservationStereoImagesPtr(obs);
				// save image to file & convert into external storage:
				if (obsSt->imageLeft.isExternallyStored())
				{
					string prevName, newName;
					obsSt->imageLeft.getExternalStorageFileAbsolutePath( prevName );

					const string fileName = string("img_") + label_time + string("_left.") + imgFileExtension;
					obsSt->imageLeft.setExternalStorage( fileName );
					entries_converted++;

					obsSt->imageLeft.getExternalStorageFileAbsolutePath( newName );
					files2rename[prevName] = newName;
				}
				else entries_skipped++;

				if (obsSt->imageRight.isExternallyStored())
				{
					string prevName, newName;
					obsSt->imageRight.getExternalStorageFileAbsolutePath( prevName );

					const string fileName = string("img_") + label_time + string("_right.") + imgFileExtension;
					obsSt->imageRight.setExternalStorage( fileName );
					entries_converted++;

					obsSt->imageRight.getExternalStorageFileAbsolutePath( newName );
					files2rename[prevName] = newName;
				}
				else entries_skipped++;
			}
			else if (IS_CLASS(obs, CObservationImage ) )
			{
				CObservationImagePtr obsIm = CObservationImagePtr(obs);

				if (obsIm->image.isExternallyStored())
				{
					string prevName, newName;
					obsIm->image.getExternalStorageFileAbsolutePath( prevName );

					const string fileName = string("img_") + label_time +string(".")+ imgFileExtension;
					obsIm->image.setExternalStorage( fileName );
					entries_converted++;

					obsIm->image.getExternalStorageFileAbsolutePath( newName );
					files2rename[prevName] = newName;
				}
				else entries_skipped++;
			}
			else if (IS_CLASS(obs, CObservation3DRangeScan ) )
			{
				CObservation3DRangeScanPtr obs3D = CObservation3DRangeScanPtr(obs);

				// save images to file & convert into external storage:
				// Intensity channel:
				if (obs3D->hasIntensityImage && obs3D->intensityImage.isExternallyStored())
				{
					string prevName, newName;
					obs3D->intensityImage.getExternalStorageFileAbsolutePath( prevName );

					const string fileName = string("3DCAM_") + label_time + string("_INT.") + imgFileExtension;
					obs3D->intensityImage.setExternalStorage( fileName );
					entries_converted++;

					obs3D->intensityImage.getExternalStorageFileAbsolutePath( newName );
					files2rename[prevName] = newName;
				}
				else entries_skipped++;

				// Confidence channel:
				if (obs3D->hasConfidenceImage && obs3D->confidenceImage.isExternallyStored())
				{
					string prevName, newName;
					obs3D->confidenceImage.getExternalStorageFileAbsolutePath( prevName );

					const string fileName = string("3DCAM_") + label_time + string("_CONF.") + imgFileExtension;
					obs3D->confidenceImage.setExternalStorage( fileName );
					entries_converted++;

					obs3D->confidenceImage.getExternalStorageFileAbsolutePath( newName );
					files2rename[prevName] = newName;
				}
				else entries_skipped++;
			}

			// Do the actual file renaming:
			for (map<string,string>::const_iterator it=files2rename.begin();it!=files2rename.end();++it)
			{
				const string &prevFil = it->first;
				const string &newFil = it->second;

				if (mrpt::system::fileExists(prevFil))
				{
					string strErr;
					if (!mrpt::system::renameFile(prevFil,newFil,&strErr)) THROW_EXCEPTION(strErr)
				}
				else
				{
					std::cerr << "Warning: Missing external file: " << prevFil << std::endl;
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
	CRawlogProcessor_RenameExternals proc(in_rawlog,cmdline,verbose);
	proc.doProcessRawlog();

	// Dump statistics:
	// ---------------------------------
	VERBOSE_COUT << "Time to process file (sec)        : " << proc.m_timToParse << "\n";
	VERBOSE_COUT << "Entries converted                 : " << proc.entries_converted << "\n";
	VERBOSE_COUT << "Entries skipped (not external)    : " << proc.entries_skipped << "\n";

}

