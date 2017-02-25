/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/utils/CDynamicGrid.h>
#include <mrpt/system/os.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace std;

bool internal::dynamic_grid_txt_saver::saveToTextFile(const std::string &fileName) const
{
	FILE	*f=os::fopen(fileName.c_str(),"wt");
	if(!f) return false;
	const unsigned int sy = getSizeY(), sx = getSizeX();
	for (unsigned int cy=0;cy<sy;cy++)
	{
		for (unsigned int cx=0;cx<sx;cx++)
			os::fprintf(f,"%f ", getCellAsFloat(cx,cy) );
		os::fprintf(f,"\n");
	}
	os::fclose(f);
	return true;
}
