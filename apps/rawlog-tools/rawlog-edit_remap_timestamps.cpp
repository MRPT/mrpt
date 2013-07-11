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

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
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

	string sAB_params;
	if (!getArgValue<string>(cmdline,"remap-timestamps",sAB_params) || sAB_params.empty() )
		throw std::runtime_error("remap-timestamps: This operation needs two arguments in the format 'a;b'.");

	vector<string> sAB_tokens;
	mrpt::utils::tokenize(sAB_params," ;",sAB_tokens);
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

