/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "reactivenav-precomp.h" // Precomp header

#include <mrpt/reactivenav/CHolonomicLogFileRecord.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::reactivenav;

IMPLEMENTS_VIRTUAL_SERIALIZABLE( CHolonomicLogFileRecord, CSerializable, mrpt::reactivenav )
