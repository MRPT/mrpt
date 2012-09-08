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
//		op_list_images
// ======================================================================
DECLARE_OP_FUNCTION(op_list_images)
{
	// A class to do this operation:
	class CRawlogProcessor_ListImages : public CRawlogProcessorOnEachObservation
	{
	protected:
		string 	       m_out_file;
		std::ofstream  m_out;

	public:
		CRawlogProcessor_ListImages(CFileGZInputStream &in_rawlog, TCLAP::CmdLine &cmdline, bool verbose) :
			CRawlogProcessorOnEachObservation(in_rawlog,cmdline,verbose)
		{
			getArgValue<std::string>(cmdline,"text-file-output",  m_out_file);
			VERBOSE_COUT << "Writing list to: " << m_out_file << endl;

			m_out.open(m_out_file.c_str());

			if (!m_out.is_open())
				throw std::runtime_error("list-images: Cannot open output text file.");
		}

		bool processOneObservation(CObservationPtr  &obs)
		{
			const string label_time = format("%s_%f", obs->sensorLabel.c_str(), timestampTotime_t(obs->timestamp) );
			if (IS_CLASS(obs, CObservationStereoImages ) )
			{
				CObservationStereoImagesPtr obsSt = CObservationStereoImagesPtr(obs);
				// save image to file & convert into external storage:
				if (obsSt->imageLeft.isExternallyStored())
					m_out << obsSt->imageLeft.getExternalStorageFileAbsolutePath() << std::endl;

				if (obsSt->imageRight.isExternallyStored())
					m_out << obsSt->imageRight.getExternalStorageFileAbsolutePath() << std::endl;
			}
			else if (IS_CLASS(obs, CObservationImage ) )
			{
				CObservationImagePtr obsIm = CObservationImagePtr(obs);

				if (obsIm->image.isExternallyStored())
					m_out << obsIm->image.getExternalStorageFileAbsolutePath() << std::endl;
			}

			return true;
		}

	};

	// Process
	// ---------------------------------
	CRawlogProcessor_ListImages proc(in_rawlog,cmdline,verbose);
	proc.doProcessRawlog();

	// Dump statistics:
	// ---------------------------------
	VERBOSE_COUT << "Time to process file (sec)        : " << proc.m_timToParse << "\n";

}

