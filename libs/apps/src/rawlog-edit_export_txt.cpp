/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "apps-precomp.h"  // Precompiled headers
//
#include <mrpt/obs/CObservationOdometry.h>

#include "rawlog-edit-declarations.h"

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::apps;
using namespace std;
using namespace mrpt::io;

// ======================================================================
//		op_export_txt
// ======================================================================
DECLARE_OP_FUNCTION(op_export_txt)
{
	// A class to do this operation:
	class CRawlogProcessor_Export_TXT : public CRawlogProcessorOnEachObservation
	{
	   protected:
		string m_inFile;

		map<string, FILE*> lstFiles;
		string m_filPrefix;

	   public:
		size_t m_entriesSaved;

		CRawlogProcessor_Export_TXT(
			CFileGZInputStream& in_rawlog, TCLAP::CmdLine& cmdline,
			bool Verbose)
			: CRawlogProcessorOnEachObservation(in_rawlog, cmdline, Verbose),
			  m_entriesSaved(0)
		{
			getArgValue<string>(cmdline, "input", m_inFile);

			m_filPrefix =
				extractFileDirectory(m_inFile) + extractFileName(m_inFile);
		}

		// return false on any error.
		bool processOneObservation(CObservation::Ptr& obs) override
		{
			if (!obs->exportTxtSupported()) return true;

			ASSERTMSG_(
				!obs->sensorLabel.empty(),
				mrpt::format(
					"Exporting as TXT requires all observations having "
					"non-empty sensorLabels, but empty label found for "
					"observation of type '%s'",
					obs->GetRuntimeClass()->className));

			auto it = lstFiles.find(obs->sensorLabel);

			FILE* f_this = nullptr;

			if (it == lstFiles.end())  // A new file for this sensorlabel??
			{
				const std::string fileName = m_filPrefix + string("_") +
					fileNameStripInvalidChars(obs->sensorLabel.empty()
												  ? string("ODOMETRY")
												  : obs->sensorLabel) +
					string(".txt");

				VERBOSE_COUT << "Writing TXT/CSV file: " << fileName << endl;

				f_this = lstFiles[obs->sensorLabel] =
					os::fopen(fileName.c_str(), "wt");
				if (!f_this)
					THROW_EXCEPTION_FMT(
						"Cannot open output file for write: %s",
						fileName.c_str());

				// The first line is a description of the columns:
				::fprintf(
					f_this,
					"%% "
					"%16s "	 // TIMESTAMP
					"%s\n",
					"Time", obs->exportTxtHeader().c_str());
			}
			else
				f_this = it->second;

			ASSERT_(obs->timestamp != INVALID_TIMESTAMP);
			double sampleTime = mrpt::Clock::toDouble(obs->timestamp);

			// Time:
			::fprintf(
				f_this,
				"%16.6f "  // TIMESTAMP
				"%s\n",
				sampleTime, obs->exportTxtDataRow().c_str());
			m_entriesSaved++;
			return true;  // All ok
		}

		// Destructor: close files and generate summary files:
		~CRawlogProcessor_Export_TXT()
		{
			for (auto it = lstFiles.begin(); it != lstFiles.end(); ++it)
				if (it->second) os::fclose(it->second);

			// Save the joint file:
			// -------------------------
			VERBOSE_COUT
				<< "Number of different sensorLabels exported to TXT/CSV: "
				<< lstFiles.size() << endl;

			lstFiles.clear();
		}  // end of destructor
	};

	// Process
	// ---------------------------------
	CRawlogProcessor_Export_TXT proc(in_rawlog, cmdline, verbose);
	proc.doProcessRawlog();

	// Dump statistics:
	// ---------------------------------
	VERBOSE_COUT << "Time to process file (sec)        : " << proc.m_timToParse
				 << "\n";
	VERBOSE_COUT << "Number of records saved           : "
				 << proc.m_entriesSaved << "\n";
}
