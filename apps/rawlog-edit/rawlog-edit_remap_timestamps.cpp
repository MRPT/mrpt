/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "rawlog-edit-declarations.h"
#include <mrpt/system/string_utils.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::rawlogtools;
using namespace std;


// ======================================================================
//		op_remap_timestamps
// ======================================================================
DECLARE_OP_FUNCTION(op_remap_timestamps)
{
	// A class to do this operation:
	class CRawlogProcessor_RemapTimestamps : public CRawlogProcessorOnEachObservation
	{
	protected:
		TOutputRawlogCreator	outrawlog;
		const double   m_a,m_b;

	public:
		CRawlogProcessor_RemapTimestamps(CFileGZInputStream &in_rawlog, TCLAP::CmdLine &cmdline, bool verbose, double a,double b) :
			CRawlogProcessorOnEachObservation(in_rawlog,cmdline,verbose),
			m_a(a),m_b(b)
		{
			VERBOSE_COUT << "Applying timestamps remap a*t+b with: a=" << m_a << " b=" << m_b << endl;
		}

		bool processOneObservation(CObservationPtr  &obs)
		{
			// T_NEW = a * T_OLD + b
			const double t = mrpt::system::timestampToDouble( obs->timestamp );
			const double t_new = m_a * t + m_b;
			obs->timestamp = mrpt::system::time_tToTimestamp(t_new);
			return true;
		}

		// This method can be reimplemented to save the modified object to an output stream.
		virtual void OnPostProcess(
			mrpt::obs::CActionCollectionPtr &actions,
			mrpt::obs::CSensoryFramePtr     &SF,
			mrpt::obs::CObservationPtr      &obs)
		{
			ASSERT_((actions && SF) || obs)
			if (actions)
					outrawlog.out_rawlog << actions << SF;
			else	outrawlog.out_rawlog << obs;
		}


	};

	string sAB_params;
	if (!getArgValue<string>(cmdline,"remap-timestamps",sAB_params) || sAB_params.empty() )
		throw std::runtime_error("remap-timestamps: This operation needs two arguments in the format 'a;b'.");

	vector<string> sAB_tokens;
	mrpt::system::tokenize(sAB_params," ;",sAB_tokens);
	if (sAB_tokens.size()!=2)
		throw std::runtime_error("remap-timestamps: This operation needs two arguments in the format 'a;b'.");

	const double a = atof(sAB_tokens[0].c_str());
	const double b = atof(sAB_tokens[1].c_str());

	// Process
	// ---------------------------------
	CRawlogProcessor_RemapTimestamps proc(in_rawlog,cmdline,verbose,a,b);
	proc.doProcessRawlog();

	// Dump statistics:
	// ---------------------------------
	VERBOSE_COUT << "Time to process file (sec)        : " << proc.m_timToParse << "\n";

}

