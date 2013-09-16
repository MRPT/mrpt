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
//		op_export_gps_txt
// ======================================================================
DECLARE_OP_FUNCTION(op_export_imu_txt)
{
	// A class to do this operation:
	class CRawlogProcessor_ExportIMU_TXT : public CRawlogProcessorOnEachObservation
	{
	protected:
		string	m_inFile;

		map<string, FILE*>	lstFiles;
		string				m_filPrefix;

	public:
		size_t				m_entriesSaved;


		CRawlogProcessor_ExportIMU_TXT(CFileGZInputStream &in_rawlog, TCLAP::CmdLine &cmdline, bool verbose) :
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
			if (!IS_CLASS(o, CObservationIMU ) )
				return true;

			const CObservationIMU* obs = CObservationIMUPtr(o).pointer();

			map<string, FILE*>::const_iterator  it = lstFiles.find( obs->sensorLabel );

			FILE *f_this;

			if ( it==lstFiles.end() )	// A new file for this sensorlabel??
			{
				const std::string fileName =
					m_filPrefix+
					string("_") +
					fileNameStripInvalidChars( obs->sensorLabel ) +
					string(".txt");

				VERBOSE_COUT << "Writing IMU TXT file: " << fileName << endl;

				f_this = lstFiles[ obs->sensorLabel ] = os::fopen( fileName.c_str(), "wt");
				if (!f_this)
					THROW_EXCEPTION_CUSTOM_MSG1("Cannot open output file for write: %s", fileName.c_str() );

				// The first line is a description of the columns:
				::fprintf(f_this,
					"%% "
					"%14s "				// TIMESTAMP
					"%22s %22s %22s "	// IMU_{X,Y,Z}_ACC
					"%22s %22s %22s "	// IMU_YAW_VEL...
					"%22s %22s %22s "	// IMU_X_VEL...
					"%22s %22s %22s "	// IMU_YAW...
					"%22s %22s %22s "	// IMU_X...
					"%22s %22s %22s "   // MAG_X MAG_Y MAG_Z 
					"%22s %22s %22s "   // PRESS ALTIT TEMP
					"\n"
					,
					"Time",
					"IMU_X_ACC","IMU_Y_ACC","IMU_Z_ACC",
					"IMU_YAW_VEL","IMU_PITCH_VEL","IMU_ROLL_VEL",
					"IMU_X_VEL","IMU_Y_VEL","IMU_Z_VEL",
					"IMU_YAW","IMU_PITCH","IMU_ROLL",
					"IMU_X","IMU_Y","IMU_Z",
					"MAG_X","MAG_Y","MAG_Z",
					"PRESS","ALTITUDE","TEMPERATURE"
					);
			}
			else
				f_this = it->second;


            ASSERT_( obs->dataIsPresent.size()==obs->rawMeasurements.size() );
            size_t nValuesPerRow = obs->dataIsPresent.size();

            // For each entry in this sequence: Compute the timestamp and save all 15 values:
            ASSERT_(obs->timestamp!=INVALID_TIMESTAMP);
            TTimeStamp	t  = obs->timestamp;

            double 	sampleTime = timestampTotime_t(t);

            // Time:
            ::fprintf(f_this,"%14.4f ",sampleTime);
            ASSERT_( obs->rawMeasurements.size()==obs->rawMeasurements.size() );
            for (size_t idx=0;idx<nValuesPerRow;idx++)
                ::fprintf(f_this,"%23.16f ",obs->dataIsPresent[idx] ? obs->rawMeasurements[idx] : 0);
            ::fprintf(f_this,"\n");

			m_entriesSaved++;

			return true; // All ok
		}


		// Destructor: close files and generate summary files:
		~CRawlogProcessor_ExportIMU_TXT()
		{
			for (map<string, FILE*>::const_iterator  it=lstFiles.begin();it!=lstFiles.end();++it)
			{
				os::fclose(it->second);
			}

			// Save the joint file:
			// -------------------------
			VERBOSE_COUT << "Number of different IMU sensorLabels     : " << lstFiles.size() << endl;

			lstFiles.clear();
		} // end of destructor

	};

	// Process
	// ---------------------------------
	CRawlogProcessor_ExportIMU_TXT proc(in_rawlog,cmdline,verbose);
	proc.doProcessRawlog();

	// Dump statistics:
	// ---------------------------------
	VERBOSE_COUT << "Time to process file (sec)        : " << proc.m_timToParse << "\n";
	VERBOSE_COUT << "Number of records saved           : " << proc.m_entriesSaved << "\n";

}
