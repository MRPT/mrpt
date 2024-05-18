/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "apps-precomp.h"  // Precompiled headers
//
#include <mrpt/obs/CObservation2DRangeScan.h>

#include "rawlog-edit-declarations.h"

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::apps;
using namespace mrpt::io;
using namespace std;

DECLARE_OP_FUNCTION(op_export_txt);

// ======================================================================
//		op_export_2d_scans_txt
// ======================================================================
DECLARE_OP_FUNCTION(op_export_2d_scans_txt)
{
  // Forward:
  op_export_txt(in_rawlog, cmdline, verbose);
}
