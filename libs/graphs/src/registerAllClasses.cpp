/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "graphs-precomp.h"	 // Precompiled headers
//
#include <mrpt/core/initializer.h>
#include <mrpt/graphs/registerAllClasses.h>
// Deps:
#include <mrpt/opengl/registerAllClasses.h>

MRPT_INITIALIZER(registerAllClasses_mrpt_graphs)
{
#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
	//	registerClass( CLASS_ID( ... ) );

#endif
}

void mrpt::graphs::registerAllClasses_mrpt_graphs()
{
	::registerAllClasses_mrpt_graphs();
	// deps:
	mrpt::opengl::registerAllClasses_mrpt_opengl();
}
