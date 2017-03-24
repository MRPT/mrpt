/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef T2DScanProperties_H
#define T2DScanProperties_H

#include <mrpt/utils/core_defs.h>
#include <mrpt/obs/link_pragmas.h>

namespace mrpt
{
namespace obs
{
	/** Auxiliary struct that holds all the relevant *geometry* information about a 2D scan.
	  * This class is used in CSinCosLookUpTableFor2DScans
	  * \ingroup mrpt_obs_grp
	  * \sa CObservation2DRangeScan, CObservation2DRangeScan::getScanProperties, CSinCosLookUpTableFor2DScans
	  */
	struct OBS_IMPEXP T2DScanProperties {
		size_t  nRays;
		double  aperture;
		bool    rightToLeft; //!< Angles storage order: true=counterclockwise; false=clockwise
	};
	bool OBS_IMPEXP operator<(const T2DScanProperties&a, const T2DScanProperties&b);	//!< Order operator, so T2DScanProperties can appear in associative STL containers.

} // End of namespace
} // End of namespace

#endif
