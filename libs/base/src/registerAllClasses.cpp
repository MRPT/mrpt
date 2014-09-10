/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"

#define MRPT_NO_WARN_BIG_HDR
//#include <mrpt/base.h>
#include <mrpt/utils/CStartUpClassesRegister.h>
#include <mrpt/poses/CPose2D.h>

#ifndef MRPT_ENABLE_PRECOMPILED_HDRS
#	define MRPT_ALWAYS_INCLUDE_ALL_HEADERS
#	undef mrpt_base_H
#	include "base-precomp.h"
#endif


using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::poses;

/*
Note: Why do we need "CStartUpClassesRegister" and "registerAllClasses_mrpt_base()" at all?
 One idea is to use static const members in every CObject, initialized via a call to registerClass(). 
 In this way, there is NO NEED to build the ugly list of classes to be registered below, and it works...
 but when building as STATIC LIBRARIES, the compiler optimizes out all non-directly used classes, 
 thus not registering them!!
 So, it seems we must live with this ugly method for the sake of static libs...
(Jose Luis Blanco, April 2013)
*/

void registerAllClasses_mrpt_base();

CStartUpClassesRegister  mrpt_base_class_reg(&registerAllClasses_mrpt_base);


/*---------------------------------------------------------------
					registerAllClasses_mrpt_base
  ---------------------------------------------------------------*/
void registerAllClasses_mrpt_base()
{

	registerClass( CLASS_ID( CPose2D ) );
}


