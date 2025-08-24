/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include "apps-cli-precomp.h"  // Precompiled headers
//
#include <mrpt/obs/CObservationWindSensor.h>

#include "rawlog-edit-declarations.h"

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::apps;
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
