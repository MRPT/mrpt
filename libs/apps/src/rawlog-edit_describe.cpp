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

using namespace mrpt::apps;

// ======================================================================
//		op_describe
// ======================================================================
DECLARE_OP_FUNCTION(op_describe)
{
	using namespace std;
	using namespace mrpt::io;
	using namespace mrpt::obs;

	// A class to do this operation:
	class CRawlogProcessor_Describe : public CRawlogProcessorOnEachObservation
	{
	   public:
		CRawlogProcessor_Describe(
			CFileGZInputStream& in_rawlog, TCLAP::CmdLine& cmdline,
			bool Verbose)
			: CRawlogProcessorOnEachObservation(in_rawlog, cmdline, Verbose)
		{
			std::cout << "total bytes: " << CRawlogProcessor::m_filSize
					  << std::endl;
		}
		~CRawlogProcessor_Describe() = default;

		// return false on any error.
		bool processOneObservation(CObservation::Ptr& obs) override
		{
			obs->getDescriptionAsText(std::cout);
			std::cout << "\n---\n";

			return true;  // All ok
		}

		bool processOneAction(mrpt::obs::CAction::Ptr& act) override
		{
			act->getDescriptionAsText(std::cout);
			std::cout << "\n---\n";
			return true;  // All ok
		}
	};

	// Process
	// ---------------------------------
	CRawlogProcessor_Describe proc(in_rawlog, cmdline, verbose);
	proc.doProcessRawlog();
}
