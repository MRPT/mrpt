/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "hmtslam-precomp.h"  // Precomp header
//
#include <mrpt/core/initializer.h>
#include <mrpt/graphslam/registerAllClasses.h>
#include <mrpt/hmtslam.h>
#include <mrpt/hmtslam/registerAllClasses.h>

MRPT_INITIALIZER(registerAllClasses_mrpt_hmtslam)
{
	using namespace mrpt::hmtslam;

#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
	registerClass(CLASS_ID(CHMTSLAM));
	registerClass(CLASS_ID(CLSLAMParticleData));
	registerClass(CLASS_ID(CHierarchicalMHMap));
	registerClass(CLASS_ID(CHMHMapArc));
	registerClass(CLASS_ID(CHMHMapNode));
	registerClass(CLASS_ID(CRobotPosesGraph));
	registerClass(CLASS_ID(THypothesisIDSet));
	registerClass(CLASS_ID(CLocalMetricHypothesis));
#endif
}

void mrpt::hmtslam::registerAllClasses_mrpt_hmtslam()
{
	::registerAllClasses_mrpt_hmtslam();
	mrpt::graphslam::registerAllClasses_mrpt_graphslam();
}
