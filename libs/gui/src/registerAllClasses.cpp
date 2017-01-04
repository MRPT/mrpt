/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "gui-precomp.h"

#include <mrpt/gui.h>

#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/initializer.h>

using namespace mrpt::gui;
using namespace mrpt::utils;

MRPT_INITIALIZER(registerAllClasses_mrpt_gui)
{
#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
	registerClass( CLASS_ID( CDisplayWindow ) );
	registerClass( CLASS_ID( CDisplayWindow3D ) );
	registerClass( CLASS_ID( CDisplayWindowPlots ) );
#endif
}
