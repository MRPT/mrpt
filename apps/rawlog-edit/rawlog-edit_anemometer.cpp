/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "rawlog-edit-declarations.h"

#include <mrpt/obs/CObservationWindSensor.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::rawlogtools;
using namespace std;

// ======================================================================
//		op_export_anemometer_txt
// ======================================================================
DECLARE_OP_FUNCTION(op_export_anemometer_txt)
{
	// A class to do this operation:
	class CRawlogProcessor_ExportANEMOMETER_TXT : public CRawlogProcessorOnEachObservation
	{
	protected:
		string	m_inFile;

		map<string, FILE*>	lstFiles;
		string				m_filPrefix;

	public:
		size_t				m_entriesSaved;


		CRawlogProcessor_ExportANEMOMETER_TXT(CFileGZInputStream &in_rawlog, TCLAP::CmdLine &cmdline, bool verbose) :
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
			if (!IS_CLASS(o, CObservationWindSensor ) )
				return true;

			const CObservationWindSensor* obs = CObservationWindSensorPtr(o).pointer();

			map<string, FILE*>::const_iterator  it = lstFiles.find( obs->sensorLabel );

			FILE *f_this;

			if ( it==lstFiles.end() )	// A new file for this sensorlabel??
			{
				const std::string fileName =
					m_filPrefix+
					string("_") +
					fileNameStripInvalidChars( obs->sensorLabel.empty() ? string("ANEMOMETER") : obs->sensorLabel ) +
					string(".txt");

				VERBOSE_COUT << "Writing anemometer TXT file: " << fileName << endl;

				f_this = lstFiles[ obs->sensorLabel ] = os::fopen( fileName.c_str(), "wt");
				if (!f_this)
					THROW_EXCEPTION_CUSTOM_MSG1("Cannot open output file for write: %s", fileName.c_str() );

				// The first line is a description of the columns:
				::fprintf(f_this,
					"%% "
					"%14s "			// TIMESTAMP
					"%18s %18s "	// WIND (mod, direction)					
					"\n"
					,
					"Time",
					"WIND_MODULE(m/s)",
					"WIND_DIRECTION (deg)"					
					);
			}
			else
				f_this = it->second;

            // For each entry in this sequence: Compute the timestamp and save values:
            ASSERT_(obs->timestamp!=INVALID_TIMESTAMP);
            TTimeStamp	t  = obs->timestamp;

            double 	sampleTime = timestampTotime_t(t);

            // Time:
			::fprintf(f_this,
				"%14.4f "			// TIMESTAMP
				"%3.4f %3.2f"		// WIND (mod, direction)
				"\n"
				,
				sampleTime, 
				obs->speed , obs->direction
				);
			m_entriesSaved++;
			return true; // All ok
		}

		// Destructor: close files and generate summary files:
		~CRawlogProcessor_ExportANEMOMETER_TXT()
		{
			for (map<string, FILE*>::const_iterator  it=lstFiles.begin();it!=lstFiles.end();++it)
			{
				os::fclose(it->second);
			}

			// Save the joint file:
			// -------------------------
			VERBOSE_COUT << "Number of different anemometer sensorLabels  : " << lstFiles.size() << endl;

			lstFiles.clear();
		} // end of destructor

	};

	// Process
	// ---------------------------------
	CRawlogProcessor_ExportANEMOMETER_TXT proc(in_rawlog,cmdline,verbose);
	proc.doProcessRawlog();

	// Dump statistics:
	// ---------------------------------
	VERBOSE_COUT << "Time to process file (sec)        : " << proc.m_timToParse << "\n";
	VERBOSE_COUT << "Number of records saved           : " << proc.m_entriesSaved << "\n";
}