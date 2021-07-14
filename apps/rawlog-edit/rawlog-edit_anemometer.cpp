/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/obs/CObservationWindSensor.h>

#include "rawlog-edit-declarations.h"

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::rawlogtools;
using namespace mrpt::io;
using namespace std;

DECLARE_OP_FUNCTION(op_export_txt);

// ======================================================================
//		op_export_anemometer_txt
// ======================================================================
DECLARE_OP_FUNCTION(op_export_anemometer_txt)
{
	// Forward:
	op_export_txt(in_rawlog, cmdline, verbose);
}
