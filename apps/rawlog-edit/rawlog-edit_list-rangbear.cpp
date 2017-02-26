/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "rawlog-edit-declarations.h"
#include <mrpt/obs/CObservationBearingRange.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::rawlogtools;
using namespace std;


// ======================================================================
//		op_list_rangebearing
// ======================================================================
DECLARE_OP_FUNCTION(op_list_rangebearing)
{
	// A class to do this operation:
	class CRawlogProcessor_RangeBearing : public CRawlogProcessorOnEachObservation
	{
	protected:
		string 	       m_out_file;
		std::ofstream  m_out;

	public:
		CRawlogProcessor_RangeBearing(CFileGZInputStream &in_rawlog, TCLAP::CmdLine &cmdline, bool verbose) :
			CRawlogProcessorOnEachObservation(in_rawlog,cmdline,verbose)
		{
			getArgValue<std::string>(cmdline,"text-file-output",  m_out_file);
			VERBOSE_COUT << "Writing list to: " << m_out_file << endl;

			m_out.open(m_out_file.c_str());

			if (!m_out.is_open())
				throw std::runtime_error("list-range-bearing: Cannot open output text file.");

			// Header:
			m_out << "%           TIMESTAMP                INDEX_IN_OBS    ID    RANGE(m)    YAW(rad)   PITCH(rad) \n"
			      << "%--------------------------------------------------------------------------------------------\n";
		}

		bool processOneObservation(CObservationPtr  &obs)
		{
			if (IS_CLASS(obs, CObservationBearingRange ) )
			{
				const CObservationBearingRangePtr obsRB_ = CObservationBearingRangePtr(obs);
				const CObservationBearingRange * obsRB = obsRB_.pointer();

				const double tim = mrpt::system::timestampToDouble( obsRB->timestamp );

				for (size_t i=0;i<obsRB->sensedData.size();i++)
					m_out << format("%35.22f %8i %10i %10f %12f %12f\n",
						tim,
						(int)i,
						(int)obsRB->sensedData[i].landmarkID,
						(double)obsRB->sensedData[i].range,
						(double)obsRB->sensedData[i].yaw,
						(double)obsRB->sensedData[i].pitch );
			}

			return true;
		}

	};

	// Process
	// ---------------------------------
	CRawlogProcessor_RangeBearing proc(in_rawlog,cmdline,verbose);
	proc.doProcessRawlog();

	// Dump statistics:
	// ---------------------------------
	VERBOSE_COUT << "Time to process file (sec)        : " << proc.m_timToParse << "\n";

}

