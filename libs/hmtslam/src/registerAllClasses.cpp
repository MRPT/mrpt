/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hmtslam-precomp.h" // Precomp header

#include <mrpt/hmtslam.h>
#include <mrpt/utils/initializer.h>

using namespace mrpt::utils;
using namespace mrpt::hmtslam;

MRPT_INITIALIZER(registerAllClasses_mrpt_hmtslam)
{
#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
	registerClass( CLASS_ID(CHMTSLAM) );
	registerClass( CLASS_ID(CLSLAMParticleData) );
	registerClass( CLASS_ID(CHierarchicalMHMap) );
	registerClass( CLASS_ID(CHMHMapArc) );
	registerClass( CLASS_ID(CHMHMapNode) );
	registerClass( CLASS_ID(CRobotPosesGraph ) );
	registerClass( CLASS_ID(THypothesisIDSet) );
	registerClass( CLASS_ID(CLocalMetricHypothesis) );
#endif
}


