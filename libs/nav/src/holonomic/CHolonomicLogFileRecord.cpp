/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "nav-precomp.h"  // Precomp header
//
#include <mrpt/nav/holonomic/CHolonomicLogFileRecord.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt;
using namespace mrpt::nav;

IMPLEMENTS_VIRTUAL_SERIALIZABLE(
	CHolonomicLogFileRecord, CSerializable, mrpt::nav)
