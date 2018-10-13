/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

namespace mrpt::obs
{
/** Auxiliary struct that holds all the relevant *geometry* information about a
 * 2D scan.
 * This class is used in CSinCosLookUpTableFor2DScans
 * \ingroup mrpt_obs_grp
 * \sa CObservation2DRangeScan, CObservation2DRangeScan::getScanProperties,
 * CSinCosLookUpTableFor2DScans
 */
struct T2DScanProperties
{
	size_t nRays;
	double aperture;
	/** Angles storage order: true=counterclockwise; false=clockwise */
	bool rightToLeft;
};
/** Order operator, so T2DScanProperties can appear in associative STL
 * containers. */
bool operator<(const T2DScanProperties& a, const T2DScanProperties& b);

}  // namespace mrpt::obs
