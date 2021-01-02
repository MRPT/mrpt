/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/detectors.h>
#include "detectors-precomp.h"  // Precompiled headers

#include <mrpt/core/initializer.h>

using namespace mrpt::detectors;

MRPT_INITIALIZER(registerAllClasses_mrpt_detectors)
{
#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
	registerClass(CLASS_ID(CDetectableObject));
	registerClass(CLASS_ID(CDetectable2D));
	registerClass(CLASS_ID(CDetectable3D));
#endif
}
