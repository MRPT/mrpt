/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "tfest-precomp.h"
#include <mrpt/tfest.h>

#include <mrpt/utils/CStartUpClassesRegister.h>


using namespace mrpt::tfest;
using namespace mrpt::utils;

void registerAllClasses_mrpt_tfest();

CStartUpClassesRegister  mrpt_tfest_class_reg(&registerAllClasses_mrpt_tfest);

/*---------------------------------------------------------------
					registerAllClasses_mrpt_tfest
  ---------------------------------------------------------------*/
void registerAllClasses_mrpt_tfest()
{
#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
//	registerClass( CLASS_ID( XXXX ) );
#endif
}

