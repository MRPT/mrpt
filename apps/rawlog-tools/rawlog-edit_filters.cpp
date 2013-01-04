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
//		op_remove_label
// ======================================================================
DECLARE_OP_FUNCTION(op_remove_label)
{
	// A class to do this operation:
	class CRawlogProcessor_RemoveLabel : public CRawlogProcessorFilterObservations
	{
	protected:
		vector<string> m_filter_labels;

	public:
		CRawlogProcessor_RemoveLabel(
			mrpt::utils::CFileGZInputStream &in_rawlog, TCLAP::CmdLine &cmdline, bool verbose,
			CFileGZOutputStream &out_rawlog,
			const std::string &filter_label
			) :
				CRawlogProcessorFilterObservations(in_rawlog,cmdline,verbose, out_rawlog)
		{
			mrpt::system::tokenize(filter_label," ,",m_filter_labels);
			ASSERT_(!m_filter_labels.empty())
			if (verbose)
				for (size_t i=0;i<m_filter_labels.size();i++)
					cout << "Removing label: '" << m_filter_labels[i] << "'\n";
		}

		/** To be implemented by users: return false means the observation is  */
		virtual bool tellIfThisObsPasses(mrpt::slam::CObservationPtr  &obs)
		{
			for (size_t i=0;i<m_filter_labels.size();i++)
				if (obs->sensorLabel==m_filter_labels[i])
				{
					return false;
				}
			return true;
		}
	};

	// Process
	// ---------------------------------
	string filter_label;
	if (!getArgValue<string>(cmdline,"remove-label",filter_label) || filter_label.empty() )
		throw std::runtime_error("remove-label: This operation needs a non-empty argument.");

	TOutputRawlogCreator	outrawlog;
	CRawlogProcessor_RemoveLabel proc(in_rawlog,cmdline,verbose,outrawlog.out_rawlog,filter_label )	;
	proc.doProcessRawlog();

	// Dump statistics:
	// ---------------------------------
	VERBOSE_COUT << "Time to process file (sec)        : " << proc.m_timToParse << "\n";
	VERBOSE_COUT << "Analyzed entries                  : " << proc.m_entries_parsed << "\n";
	VERBOSE_COUT << "Removed entries                   : " << proc.m_entries_removed << "\n";

}

// ======================================================================
//		op_keep_label
// ======================================================================
DECLARE_OP_FUNCTION(op_keep_label)
{
	// A class to do this operation:
	class CRawlogProcessor_KeepLabel : public CRawlogProcessorFilterObservations
	{
	protected:
		vector<string> m_filter_labels;

	public:
		CRawlogProcessor_KeepLabel(
			mrpt::utils::CFileGZInputStream &in_rawlog, TCLAP::CmdLine &cmdline, bool verbose,
			CFileGZOutputStream &out_rawlog,
			const std::string &filter_label
			) :
				CRawlogProcessorFilterObservations(in_rawlog,cmdline,verbose, out_rawlog)
		{
			mrpt::system::tokenize(filter_label," ,",m_filter_labels);
			ASSERT_(!m_filter_labels.empty())
			if (verbose)
				for (size_t i=0;i<m_filter_labels.size();i++)
					cout << "Keeping label: '" << m_filter_labels[i] << "'\n";
		}

		/** To be implemented by users: return false means the observation is  */
		virtual bool tellIfThisObsPasses(mrpt::slam::CObservationPtr  &obs)
		{
			for (size_t i=0;i<m_filter_labels.size();i++)
				if (obs->sensorLabel==m_filter_labels[i])
				{
					return true;
				}
			return false;
		}
	};

	// Process
	// ---------------------------------
	string filter_label;
	if (!getArgValue<string>(cmdline,"keep-label",filter_label) || filter_label.empty() )
		throw std::runtime_error("keep-label: This operation needs a non-empty argument.");

	TOutputRawlogCreator	outrawlog;
	CRawlogProcessor_KeepLabel proc(in_rawlog,cmdline,verbose,outrawlog.out_rawlog,filter_label )	;
	proc.doProcessRawlog();

	// Dump statistics:
	// ---------------------------------
	VERBOSE_COUT << "Time to process file (sec)        : " << proc.m_timToParse << "\n";
	VERBOSE_COUT << "Analyzed entries                  : " << proc.m_entries_parsed << "\n";
	VERBOSE_COUT << "Removed entries                   : " << proc.m_entries_removed << "\n";

}

