/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "containers-precomp.h"  // Precompiled headers

#include <mrpt/containers/CDynamicGrid.h>
#include <mrpt/core/format.h>
#include <fstream>

bool mrpt::containers::internal::dynamic_grid_txt_saver::saveToTextFile(
	const std::string& fileName) const
{
	std::ofstream f;
	f.open(fileName.c_str(), std::ofstream::out);
	if (!f.is_open()) return false;
	const unsigned int sy = getSizeY(), sx = getSizeX();
	for (unsigned int cy = 0; cy < sy; cy++)
	{
		for (unsigned int cx = 0; cx < sx; cx++)
			f << mrpt::format("%f ", getCellAsFloat(cx, cy));
		f << "\n";
	}
	return true;
}
