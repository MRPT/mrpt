/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "detectors-precomp.h"  // Precompiled headers
#include <mrpt/detectors.h>

#include <mrpt/utils.h>


using namespace mrpt::detectors;
using namespace mrpt::utils;

void registerAllClasses_mrpt_detectors();

CStartUpClassesRegister  mrpt_detectors_class_reg(&registerAllClasses_mrpt_detectors);

/*---------------------------------------------------------------
					registerAllClasses_mrpt_detectors
  ---------------------------------------------------------------*/
void registerAllClasses_mrpt_detectors()
{
#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
	registerClass( CLASS_ID( CDetectableObject ) );
	registerClass( CLASS_ID( CDetectable2D ) );
	registerClass( CLASS_ID( CDetectable3D ) );
#endif
}

