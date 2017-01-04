/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "rawlog-edit-declarations.h"

#include <mrpt/obs/CObservationGasSensors.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::rawlogtools;
using namespace std;

// ======================================================================
//		op_export_enose_txt
// ======================================================================
DECLARE_OP_FUNCTION(op_export_enose_txt)
{
	// A class to do this operation:
	class CRawlogProcessor_ExportENOSE_TXT : public CRawlogProcessorOnEachObservation
	{
	protected:
		string	m_inFile;

		map<string, FILE*>	lstFiles;
		string				m_filPrefix;

	public:
		size_t				m_entriesSaved;

		//Default Constructor
		CRawlogProcessor_ExportENOSE_TXT(CFileGZInputStream &in_rawlog, TCLAP::CmdLine &cmdline, bool verbose) :
			CRawlogProcessorOnEachObservation(in_rawlog,cmdline,verbose),
			m_entriesSaved(0)
		{
			getArgValue<string>(cmdline,"input",m_inFile);

			m_filPrefix =
				extractFileDirectory(m_inFile) +
				extractFileName(m_inFile);
		}

		// Return false on any error.
		bool processOneObservation(CObservationPtr  &o)
		{
			if (!IS_CLASS(o, CObservationGasSensors ) )
				return true;

			const CObservationGasSensors* obs = CObservationGasSensorsPtr(o).pointer();

			map<string, FILE*>::const_iterator  it = lstFiles.find( obs->sensorLabel );

			FILE *f_this;

			if ( it==lstFiles.end() )	// A new file for this sensorlabel??
			{
				const std::string fileName =
					m_filPrefix+
					string("_") +
					fileNameStripInvalidChars( obs->sensorLabel.empty() ? string("ENOSE") : obs->sensorLabel ) +
					string(".txt");

				VERBOSE_COUT << "Writing e-nose TXT file: " << fileName << endl;

				f_this = lstFiles[ obs->sensorLabel ] = os::fopen( fileName.c_str(), "wt");
				if (!f_this)
					THROW_EXCEPTION_CUSTOM_MSG1("Cannot open output file for write: %s", fileName.c_str() );

				// The first line is a description of the columns:
				//------------------------------------------------
				// Time:
				::fprintf(f_this, "%% %13s ", "Time");

				//For each E-nose (if more than one)
				for (size_t j = 0; j<obs->m_readings.size(); j++)
				{
					//Temperature
					::fprintf(f_this, " Temp%u ", static_cast<unsigned int>(j));

					//For each sensor on the E-nose					
					for (size_t k = 0; k<obs->m_readings[j].readingsVoltage.size(); k++)
						::fprintf(f_this, " S%u_%u ", static_cast<unsigned int>(j),static_cast<unsigned int>(k));
				}
				::fprintf(f_this,"\n");
			}
			else
				f_this = it->second;

            // For each entry in this sequence: Compute the timestamp and save all values:
            ASSERT_(obs->timestamp!=INVALID_TIMESTAMP);
            TTimeStamp	t  = obs->timestamp;

            double 	sampleTime = timestampTotime_t(t);

            // Time:
			::fprintf(f_this, "%14.4f ", sampleTime);

			//For each E-nose (if more than one)
			for (size_t j = 0; j<obs->m_readings.size(); j++)
			{
				//Temperature
				float temp = 0.0;
				if (obs->m_readings[j].hasTemperature == true)
					temp = obs->m_readings[j].temperature;
				::fprintf(f_this, "%3.4f ", temp);
				
				//For each sensor on the E-nose
				for (std::vector<float>::const_iterator it = obs->m_readings[j].readingsVoltage.begin(); it != obs->m_readings[j].readingsVoltage.end(); ++it)
					::fprintf(f_this, "%5.5f ", *it);

				//Separation between different e-noses in the observation
				//::fprintf(f_this, "# ");
			}
			
			::fprintf(f_this, "\n");
			m_entriesSaved++;
			return true; // All ok
		}

		// Destructor: close files and generate summary files:
		~CRawlogProcessor_ExportENOSE_TXT()
		{
			for (map<string, FILE*>::const_iterator  it=lstFiles.begin();it!=lstFiles.end();++it)
			{
				os::fclose(it->second);
			}

			// Save the joint file:
			// -------------------------
			VERBOSE_COUT << "Number of different E-nose sensorLabels  : " << lstFiles.size() << endl;

			lstFiles.clear();
		} // end of destructor

	};

	// Process
	// ---------------------------------
	CRawlogProcessor_ExportENOSE_TXT proc(in_rawlog,cmdline,verbose);
	proc.doProcessRawlog();

	// Dump statistics:
	// ---------------------------------
	VERBOSE_COUT << "Time to process file (sec)        : " << proc.m_timToParse << "\n";
	VERBOSE_COUT << "Number of records saved           : " << proc.m_entriesSaved << "\n";
}
