/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header

#include <mrpt/nav.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CStartUpClassesRegister.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::nav;
using namespace std;

void registerAllNavigationClasses();

CStartUpClassesRegister  mrpt_reactivenav_class_reg(&registerAllNavigationClasses);


void registerAllNavigationClasses()
{
	registerClass(CLASS_ID( CLogFileRecord ));
	registerClass(CLASS_ID( CLogFileRecord_ND ));
	registerClass(CLASS_ID( CLogFileRecord_VFF ));
}
