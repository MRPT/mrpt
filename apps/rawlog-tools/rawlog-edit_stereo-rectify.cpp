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
#include <mrpt/vision/CStereoRectifyMap.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::system;
using namespace mrpt::rawlogtools;
using namespace std;


// ======================================================================
//		op_stereo_rectify
// ======================================================================
DECLARE_OP_FUNCTION(op_stereo_rectify)
{
	// A class to do this operation:
	class CRawlogProcessor_StereoRectify : public CRawlogProcessorOnEachObservation
	{
	protected:
		TOutputRawlogCreator	outrawlog;

		string   target_label;
		string   outDir;
		string   imgFileExtension;
		double   rectify_alpha; // [0,1] see cvStereoRectify()

		bool     m_this_obs_is_ok;

		mrpt::vision::CStereoRectifyMap   rectify_map;

		size_t  m_num_external_files_failures;

	public:
		size_t  m_changedCams;

		CRawlogProcessor_StereoRectify(CFileGZInputStream &in_rawlog, TCLAP::CmdLine &cmdline, bool verbose) :
			CRawlogProcessorOnEachObservation(in_rawlog,cmdline,verbose)
		{
			m_changedCams = 0;
			m_num_external_files_failures = 0;

			// Load .ini file with poses:
			string   str;
			getArgValue<string>(cmdline,"stereo-rectify",str);

			vector<string> lstTokens;
			tokenize(str,",",lstTokens);
			if (lstTokens.size()!=2)
				throw std::runtime_error("--stereo-rectify op: argument must be in the format: --stereo-rectify LABEL,ALPHA_VALUE");

			target_label = lstTokens[0];

			const string &sAlpha = lstTokens[1];
			rectify_alpha = atof(sAlpha.c_str());
			if (rectify_alpha!=-1 && !(rectify_alpha>=0 && rectify_alpha<=1))
				throw std::runtime_error("--stereo-rectify op: Invalid ALPHA value. Use '-1' for auto guess.");

			getArgValue<string>(cmdline,"image-format",imgFileExtension);

			// Create a "/Images_Rectified" directory.
			const string out_rawlog_basedir = extractFileDirectory(outrawlog.out_rawlog_filename);

			outDir = (out_rawlog_basedir.empty() ? string() : (out_rawlog_basedir+string("/") )) + extractFileName(outrawlog.out_rawlog_filename) + string("_Images");
			if (directoryExists(outDir))
				throw runtime_error(string("*ABORTING*: Output directory for rectified images already exists: ") + outDir + string("\n. Select a different output path or remove the directory.") );

			VERBOSE_COUT << "Creating directory: " << outDir << endl;

			mrpt::system::createDirectory( outDir );
			if (!fileExists(outDir))
				throw runtime_error(string("*ABORTING*: Couldn't create directory: ") + outDir );

			// Add the final /
			outDir+="/";

			// Optional argument:  "--image-size=640x480"
			string   strResize;
			if (getArgValue<string>(cmdline,"image-size",str))
			{
                vector<string> lstTokens;
                tokenize(str,"x",lstTokens);
                if (lstTokens.size()!=2)
                    throw std::runtime_error("--stereo-rectify op: Expected format: --image-size NCOLSxNROWS");

                const int nCols = atoi(lstTokens[0].c_str());
                const int nRows = atoi(lstTokens[1].c_str());
                VERBOSE_COUT << "Will rectify and resize to " << nCols << "x" << nRows << " simultaneously.\n";

                rectify_map.enableResizeOutput(true,nCols,nRows);
			}
		}

		bool processOneObservation(CObservationPtr  &obs)
		{
		    m_this_obs_is_ok = true;

			if ( strCmpI(obs->sensorLabel,target_label))
			{
				if (IS_CLASS(obs,CObservationStereoImages))
				{
					CObservationStereoImagesPtr o = CObservationStereoImagesPtr(obs);

					try
					{
                        // Already initialized the rectification map?
                        if (!rectify_map.isSet())
                        {
                            // On the first ocassion, initialize map:
                            rectify_map.setAlpha( rectify_alpha );
                            rectify_map.setFromCamParams( *o );
                        }

			// This is needed to raise an exception of the correct type that reveal any missing external file:
			o->imageLeft.getWidth();
			o->imageRight.getWidth();

                        // This call rectifies the images in-place and also updates
                        // all the camera parameters as needed:
                        rectify_map.rectify(*o);

                        const string label_time = format("%s_%f", o->sensorLabel.c_str(), timestampTotime_t(o->timestamp) );
                        {
                            const string fileName = string("img_") + label_time + string("_left.") + imgFileExtension;
                            o->imageLeft.saveToFile( outDir + fileName );
                            o->imageLeft.setExternalStorage( fileName );
                        }
                        {
                            const string fileName = string("img_") + label_time + string("_right.") + imgFileExtension;
                            o->imageRight.saveToFile( outDir + fileName );
                            o->imageRight.setExternalStorage( fileName );
                        }
                        m_changedCams++;
					}
					catch (mrpt::utils::CExceptionExternalImageNotFound &)
					{
					    const size_t MAX_FAILURES = 1000;
					    m_num_external_files_failures++;

					    if (m_num_external_files_failures<MAX_FAILURES)
					    {
					        m_this_obs_is_ok = false;
                            cerr << "\n *WARNING*: Dropping one observation due to missing external image file at rawlog entry " << m_rawlogEntry << endl;
					    }
					    else
					    {
					        throw std::runtime_error("*ERROR* Too many external images missing, this doesn't seem spureous missings!");
					    }
					}
				}
			}
			return true;
		}

		// This method can be reimplemented to save the modified object to an output stream.
		virtual void OnPostProcess(
			mrpt::slam::CActionCollectionPtr &actions,
			mrpt::slam::CSensoryFramePtr     &SF,
			mrpt::slam::CObservationPtr      &obs)
		{
		    if (!m_this_obs_is_ok) return;

			ASSERT_((actions && SF) || obs)
			if (actions)
					outrawlog.out_rawlog << actions << SF;
			else	outrawlog.out_rawlog << obs;
		}

	};

	// Process
	// ---------------------------------
	CRawlogProcessor_StereoRectify proc(in_rawlog,cmdline,verbose);
	proc.doProcessRawlog();

	// Dump statistics:
	// ---------------------------------
	VERBOSE_COUT << "Time to process file (sec)        : " << proc.m_timToParse << "\n";
	VERBOSE_COUT << "Number of modified entries        : " << proc.m_changedCams << "\n";

}

