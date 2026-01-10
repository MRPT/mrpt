/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
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
