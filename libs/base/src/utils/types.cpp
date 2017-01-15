/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers


#include <mrpt/utils/types.h>
#include <mrpt/utils/TPixelCoord.h>

using namespace mrpt::utils;
using namespace std;


/*---------------------------------------------------------------
					<< for TPixelCoordf
 ---------------------------------------------------------------*/
std::ostream& mrpt::utils::operator <<(std::ostream& o, const TPixelCoordf& p)
{
	o << "(" << p.x << "," << p.y << ")";
	return o;
}

/*---------------------------------------------------------------
					<< for TPixelCoord
 ---------------------------------------------------------------*/
std::ostream& mrpt::utils::operator <<(std::ostream& o, const TPixelCoord& p)
{
	o << "(" << p.x << "," << p.y << ")";
	return o;
}
