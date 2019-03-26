/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/core/initializer.h>
#include <mrpt/pbmap.h>
#include "pbmap-precomp.h"  // precomp. hdr

using namespace mrpt::pbmap;

MRPT_INITIALIZER(registerAllClasses_mrpt_pbmap)
{
#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
	registerClass(CLASS_ID(Plane));
	registerClass(CLASS_ID(PbMap));
#endif
}
