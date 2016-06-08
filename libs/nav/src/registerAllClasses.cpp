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
	// PTGs:
	registerClass(CLASS_ID( CPTG1 ));
	registerClass(CLASS_ID( CPTG2 ));
	registerClass(CLASS_ID( CPTG3 ));
	registerClass(CLASS_ID( CPTG4 ));
	registerClass(CLASS_ID( CPTG5 ));
	registerClass(CLASS_ID( CPTG6 ));
	registerClass(CLASS_ID( CPTG7 ));


	// Logs:
	registerClass(CLASS_ID( CLogFileRecord ));
	registerClass(CLASS_ID( CLogFileRecord_ND ));
	registerClass(CLASS_ID( CLogFileRecord_VFF ));
}
