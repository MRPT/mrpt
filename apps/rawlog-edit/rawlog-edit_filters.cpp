/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "rawlog-edit-declarations.h"

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::obs;
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
		virtual bool tellIfThisObsPasses(mrpt::obs::CObservationPtr  &obs)
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
		virtual bool tellIfThisObsPasses(mrpt::obs::CObservationPtr  &obs)
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

