/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/Visualizable.h>

// Note: This method cannot be defined into the .h to avoid including the
// full CSetOfObjects.h header.
mrpt::opengl::CSetOfObjects::Ptr mrpt::opengl::Visualizable::getVisualization() const
{
  auto o = mrpt::opengl::CSetOfObjects::Create();
  getVisualizationInto(*o);
  return o;
}
