/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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

#include <mrpt/topography.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::system;
using namespace mrpt::rawlogtools;
using namespace mrpt::topography;
using namespace std;

// ======================================================================
//		op_export_2d_scans_txt
// ======================================================================
DECLARE_OP_FUNCTION(op_export_2d_scans_txt)
{
	// A class to do this operation:
	class CRawlogProcessor_Export2DSCANS_TXT : public CRawlogProcessorOnEachObservation
	{
	protected:
		string	m_inFile;

		map<string, FILE*>	lstFiles,lstFilesTimes;
		string				m_filPrefix;

	public:
		size_t				m_entriesSaved;


		CRawlogProcessor_Export2DSCANS_TXT(CFileGZInputStream &in_rawlog, TCLAP::CmdLine &cmdline, bool verbose) :
			CRawlogProcessorOnEachObservation(in_rawlog,cmdline,verbose),
			m_entriesSaved(0)
		{
			getArgValue<string>(cmdline,"input",m_inFile);

			m_filPrefix =
				extractFileDirectory(m_inFile) +
				extractFileName(m_inFile);
		}

		// return false on any error.
		bool processOneObservation(CObservationPtr  &o)
		{
			if (!IS_CLASS(o, CObservation2DRangeScan ) )
				return true;

			const CObservation2DRangeScan* obs = CObservation2DRangeScanPtr(o).pointer();

			map<string, FILE*>::const_iterator  it = lstFiles.find( obs->sensorLabel );

			FILE *f_this, *f_this_times;

			if ( it==lstFiles.end() )	// A new file for this sensorlabel??
			{
				const std::string fileName =
					m_filPrefix+
					string("_") +
					fileNameStripInvalidChars( obs->sensorLabel ) +
					string(".txt");

				const std::string fileNameTimes =
					m_filPrefix+
					string("_") +
					fileNameStripInvalidChars( obs->sensorLabel ) +
					string("_times.txt");

				VERBOSE_COUT << "Writing LASER TXT file: " << fileName << endl;

				f_this        = lstFiles[ obs->sensorLabel ] = os::fopen( fileName.c_str(), "wt");
				f_this_times  = lstFilesTimes[ obs->sensorLabel ] = os::fopen( fileNameTimes.c_str(), "wt");
				if (!f_this || !f_this_times)
					THROW_EXCEPTION_CUSTOM_MSG1("Cannot open output file for write: %s", fileName.c_str() );
			}
			else
			{
				f_this = it->second;
				f_this_times = lstFilesTimes.find( obs->sensorLabel )->second;
			}

            // Time:
            const double 	sampleTime = timestampTotime_t(obs->timestamp);
            ::fprintf(f_this_times,"%14.4f\n",sampleTime);

            // Ranges:
            for (size_t j=0;j<obs->scan.size();j++)
                ::fprintf(f_this,"%.6f ", obs->validRange[j] ? obs->scan[j] : 0 );
            ::fprintf(f_this,"\n");

			m_entriesSaved++;

			return true; // All ok
		}


		// Destructor: close files and generate summary files:
		~CRawlogProcessor_Export2DSCANS_TXT()
		{
			for (map<string, FILE*>::const_iterator  it=lstFiles.begin();it!=lstFiles.end();++it)
			{
				os::fclose(it->second);
			}

			// Save the joint file:
			// -------------------------
			VERBOSE_COUT << "Number of different 2D-SCAN sensorLabels     : " << lstFiles.size() << endl;

			lstFiles.clear();
		} // end of destructor

	};

	// Process
	// ---------------------------------
	CRawlogProcessor_Export2DSCANS_TXT proc(in_rawlog,cmdline,verbose);
	proc.doProcessRawlog();

	// Dump statistics:
	// ---------------------------------
	VERBOSE_COUT << "Time to process file (sec)        : " << proc.m_timToParse << "\n";
	VERBOSE_COUT << "Number of records saved           : " << proc.m_entriesSaved << "\n";

}
