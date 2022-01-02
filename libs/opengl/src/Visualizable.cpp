/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"	 // Precompiled header
//
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/Visualizable.h>

// Note: This method cannot be defined into the .h to avoid including the
// full CSetOfObjects.h header.
mrpt::opengl::CSetOfObjects::Ptr mrpt::opengl::Visualizable::getVisualization()
	const
{
	auto o = mrpt::opengl::CSetOfObjects::Create();
	getVisualizationInto(*o);
	return o;
}
