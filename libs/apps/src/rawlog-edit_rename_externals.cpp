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
#include <mrpt/obs/format_externals_filename.h>

#include "rawlog-edit-declarations.h"

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::apps;
using namespace std;
using namespace mrpt::io;

// ======================================================================
//		op_rename_externals
// ======================================================================
DECLARE_OP_FUNCTION(op_rename_externals)
{
	// A class to do this operation:
	class CRawlogProcessor_RenameExternals
		: public CRawlogProcessorOnEachObservation
	{
	   protected:
		TOutputRawlogCreator outrawlog;

		string imgFileExtension;
		string outDir;

		std::string m_obsFmtString = "${type}_${label}_%.06%f";

	   public:
		size_t entries_converted;
		size_t entries_skipped;	 // Already external

		CRawlogProcessor_RenameExternals(
			CFileGZInputStream& in_rawlog, TCLAP::CmdLine& cmdline,
			bool Verbose)
			: CRawlogProcessorOnEachObservation(in_rawlog, cmdline, Verbose)
		{
			entries_converted = 0;
			entries_skipped = 0;

			getArgValue<string>(cmdline, "image-format", imgFileExtension);
			getArgValue<string>(
				cmdline, "externals-filename-format", m_obsFmtString);
		}

		bool processOneObservation(CObservation::Ptr& obs) override
		{
			using namespace std::string_literals;

			map<string, string> files2rename;

			string prefix =
				mrpt::obs::format_externals_filename(*obs, m_obsFmtString);

			if (IS_CLASS(*obs, CObservationStereoImages))
			{
				CObservationStereoImages::Ptr obsSt =
					std::dynamic_pointer_cast<CObservationStereoImages>(obs);
				// save image to file & convert into external storage:
				if (obsSt->imageLeft.isExternallyStored())
				{
					string prevName, newName;
					obsSt->imageLeft.getExternalStorageFileAbsolutePath(
						prevName);

					const string fileName = prefix =
						"_left."s + imgFileExtension;
					obsSt->imageLeft.setExternalStorage(fileName);
					entries_converted++;

					obsSt->imageLeft.getExternalStorageFileAbsolutePath(
						newName);
					files2rename[prevName] = newName;
				}
				else
					entries_skipped++;

				if (obsSt->imageRight.isExternallyStored())
				{
					string prevName, newName;
					obsSt->imageRight.getExternalStorageFileAbsolutePath(
						prevName);

					const string fileName =
						prefix + "_right."s + imgFileExtension;
					obsSt->imageRight.setExternalStorage(fileName);
					entries_converted++;

					obsSt->imageRight.getExternalStorageFileAbsolutePath(
						newName);
					files2rename[prevName] = newName;
				}
				else
					entries_skipped++;
			}
			else if (IS_CLASS(*obs, CObservationImage))
			{
				CObservationImage::Ptr obsIm =
					std::dynamic_pointer_cast<CObservationImage>(obs);

				if (obsIm->image.isExternallyStored())
				{
					string prevName, newName;
					obsIm->image.getExternalStorageFileAbsolutePath(prevName);

					const string fileName = prefix + "."s + imgFileExtension;
					obsIm->image.setExternalStorage(fileName);
					entries_converted++;

					obsIm->image.getExternalStorageFileAbsolutePath(newName);
					files2rename[prevName] = newName;
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
					obs3D->intensityImage.isExternallyStored())
				{
					string prevName, newName;
					obs3D->intensityImage.getExternalStorageFileAbsolutePath(
						prevName);

					const string fileName =
						prefix + "_INT."s + imgFileExtension;
					obs3D->intensityImage.setExternalStorage(fileName);
					entries_converted++;

					obs3D->intensityImage.getExternalStorageFileAbsolutePath(
						newName);
					files2rename[prevName] = newName;
				}
				else
					entries_skipped++;

				// Confidence channel:
				if (obs3D->hasConfidenceImage &&
					obs3D->confidenceImage.isExternallyStored())
				{
					string prevName, newName;
					obs3D->confidenceImage.getExternalStorageFileAbsolutePath(
						prevName);

					const string fileName =
						prefix + "_CONF."s + imgFileExtension;
					obs3D->confidenceImage.setExternalStorage(fileName);
					entries_converted++;

					obs3D->confidenceImage.getExternalStorageFileAbsolutePath(
						newName);
					files2rename[prevName] = newName;
				}
				else
					entries_skipped++;
			}

			// Do the actual file renaming:
			for (const auto& mapping : files2rename)
			{
				const string& prevFil = mapping.first;
				const string& newFil = mapping.second;

				if (mrpt::system::fileExists(prevFil))
				{
					string strErr;
					if (!mrpt::system::renameFile(prevFil, newFil, &strErr))
						THROW_EXCEPTION(strErr);
				}
				else
				{
					std::cerr << "Warning: Missing external file: " << prevFil
							  << "( => '" << newFil << "')\n";
				}
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
	CRawlogProcessor_RenameExternals proc(in_rawlog, cmdline, verbose);
	proc.doProcessRawlog();

	// Dump statistics:
	// ---------------------------------
	VERBOSE_COUT << "Time to process file (sec)        : " << proc.m_timToParse
				 << "\n";
	VERBOSE_COUT << "Entries converted                 : "
				 << proc.entries_converted << "\n";
	VERBOSE_COUT << "Entries skipped (not external)    : "
				 << proc.entries_skipped << "\n";
}
