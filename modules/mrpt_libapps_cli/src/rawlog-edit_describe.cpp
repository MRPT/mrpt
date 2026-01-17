/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

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
        CFileGZInputStream& in_rawlog, TCLAP::CmdLine& cmdline, bool Verbose) :
        CRawlogProcessorOnEachObservation(in_rawlog, cmdline, Verbose)
    {
      std::cout << "total bytes: " << CRawlogProcessor::m_filSize << "\n";
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
