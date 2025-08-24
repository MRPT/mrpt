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

#include "containers-precomp.h"  // Precompiled headers
//
#include <mrpt/containers/CDynamicGrid.h>
#include <mrpt/core/format.h>

#include <fstream>

bool mrpt::containers::internal::dynamic_grid_txt_saver::saveToTextFile(
    const std::string& fileName) const
{
  std::ofstream f;
  f.open(fileName.c_str(), std::ofstream::out);
  if (!f.is_open())
  {
    return false;
  }
  const unsigned int sy = getSizeY(), sx = getSizeX();
  for (unsigned int cy = 0; cy < sy; cy++)
  {
    for (unsigned int cx = 0; cx < sx; cx++)
    {
      f << mrpt::format("%lf ", static_cast<double>(getCellAsFloat(cx, cy)));
    }
    f << "\n";
  }
  return true;
}
