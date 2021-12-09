/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "apps-precomp.h"  // Precompiled headers
//
#include "rawlog-edit-declarations.h"

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::poses;
using namespace mrpt::io;
using namespace mrpt::apps;
using namespace std;
using namespace mrpt::io;

// ======================================================================
//		op_list_poses
// ======================================================================
DECLARE_OP_FUNCTION(op_list_poses)
{
	// A class to do this operation:
	class CRawlogProcessor_ListPoses : public CRawlogProcessorOnEachObservation
	{
	   protected:
		string m_out_file;
		std::ofstream m_out;

	   public:
		CRawlogProcessor_ListPoses(
			CFileGZInputStream& in_rawlog, TCLAP::CmdLine& cmdline,
			bool Verbose)
			: CRawlogProcessorOnEachObservation(in_rawlog, cmdline, Verbose)
		{
			getArgValue<std::string>(cmdline, "text-file-output", m_out_file);
			VERBOSE_COUT << "Writing list to: " << m_out_file << endl;

			m_out.open(m_out_file.c_str());

			if (!m_out.is_open())
				throw std::runtime_error(
					"list-poses: Cannot open output text file.");
		}

		bool processOneObservation(CObservation::Ptr& obs) override
		{
			mrpt::poses::CPose3D pose;
			obs->getSensorPose(pose);
			m_out << pose.asString() << std::endl;

			return true;
		}
	};

	// Process
	// ---------------------------------
	CRawlogProcessor_ListPoses proc(in_rawlog, cmdline, verbose);
	proc.doProcessRawlog();

	// Dump statistics:
	// ---------------------------------
	VERBOSE_COUT << "Time to process file (sec)        : " << proc.m_timToParse
				 << "\n";
}
