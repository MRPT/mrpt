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

#include <mrpt/obs/CObservation3DRangeScan.h>

#include "rawlog-edit-declarations.h"

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::apps;
using namespace std;
using namespace mrpt::io;

// ======================================================================
//		op_generate_3d_pointclouds
// ======================================================================
DECLARE_OP_FUNCTION(op_generate_3d_pointclouds)
{
  // A class to do this operation:
  class CRawlogProcessor_Generate3DPointClouds : public CRawlogProcessorOnEachObservation
  {
   protected:
    TOutputRawlogCreator outrawlog;

   public:
    size_t entries_modified;

    CRawlogProcessor_Generate3DPointClouds(
        CCompressedInputStream& in_rawlog, TCLAP::CmdLine& cmdline, bool Verbose) :
        CRawlogProcessorOnEachObservation(in_rawlog, cmdline, Verbose)
    {
      entries_modified = 0;
    }

    bool processOneObservation(CObservation::Ptr& obs) override
    {
      if (IS_CLASS(*obs, CObservation3DRangeScan))
      {
        CObservation3DRangeScan::Ptr obs3D =
            std::dynamic_pointer_cast<CObservation3DRangeScan>(obs);
        if (obs3D->hasRangeImage)
        {
          obs3D->load();  // We must be sure that depth has been
          // loaded, if stored separately.
          obs3D->unprojectInto(*obs3D);
          entries_modified++;
        }
      }

      return true;
    }

    // This method can be reimplemented to save the modified object to an
    // output stream.
    void OnPostProcess(
        mrpt::obs::CActionCollection::Ptr& actions,
        mrpt::obs::CSensoryFrame::Ptr& SF,
        mrpt::obs::CObservation::Ptr& obs) override
    {
      ASSERT_((actions && SF) || obs);
      if (actions)
        (*outrawlog.out_rawlog) << actions << SF;
      else
        (*outrawlog.out_rawlog) << obs;
    }
  };

  // Process
  // ---------------------------------
  CRawlogProcessor_Generate3DPointClouds proc(in_rawlog, cmdline, verbose);
  proc.doProcessRawlog();

  // Dump statistics:
  // ---------------------------------
  VERBOSE_COUT << "Time to process file (sec)        : " << proc.m_timToParse << "\n";
  VERBOSE_COUT << "Entries modified                  : " << proc.entries_modified << "\n";
}
