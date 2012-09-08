/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#include "rawlog-edit-declarations.h"

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
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
		//CFileGZOutputStream out_rawlog;
		//string output_rawlog;

		string	imgFileExtension;
		string 	outDir;

	public:
		size_t  entries_converted;
		size_t  entries_skipped; // Already external

		CRawlogProcessor_Externalize(CFileGZInputStream &in_rawlog, TCLAP::CmdLine &cmdline, bool verbose) :
			CRawlogProcessorOnEachObservation(in_rawlog,cmdline,verbose)
		{
			entries_converted = 0;
			entries_skipped  = 0;
			getArgValue<string>(cmdline,"image-format",imgFileExtension); 

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
	CRawlogProcessor_Externalize proc(in_rawlog,cmdline,verbose);
	proc.doProcessRawlog();

	// Dump statistics:
	// ---------------------------------
	VERBOSE_COUT << "Time to process file (sec)        : " << proc.m_timToParse << "\n";
	VERBOSE_COUT << "Entries converted                 : " << proc.entries_converted << "\n";
	VERBOSE_COUT << "Entries skipped (already external): " << proc.entries_skipped << "\n";

}

