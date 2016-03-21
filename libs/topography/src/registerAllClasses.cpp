/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "topography-precomp.h"
#include <mrpt/topography.h>

#include <mrpt/utils/CStartUpClassesRegister.h>


using namespace mrpt::topography;
using namespace mrpt::utils;

void registerAllClasses_mrpt_topography();

CStartUpClassesRegister  mrpt_topography_class_reg(&registerAllClasses_mrpt_topography);

/*---------------------------------------------------------------
					registerAllClasses_mrpt_topography
  ---------------------------------------------------------------*/
void registerAllClasses_mrpt_topography()
{
#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
//	registerClass( CLASS_ID( XXXX ) );
#endif
}

