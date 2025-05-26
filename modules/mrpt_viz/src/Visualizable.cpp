/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2025, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "viz-precomp.h"  // Precompiled header
//
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/Visualizable.h>

// Note: This method cannot be defined into the .h to avoid including the
// full CSetOfObjects.h header.
mrpt::viz::CSetOfObjects::Ptr mrpt::viz::Visualizable::getVisualization() const
{
  auto o = mrpt::viz::CSetOfObjects::Create();
  getVisualizationInto(*o);
  return o;
}
