/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */
#ifndef CSinCosLookUpTableFor2DScans_H
#define CSinCosLookUpTableFor2DScans_H

#include <mrpt/slam/CObservation2DRangeScan.h>

namespace mrpt
{
namespace slam
{
	/** A smart look-up-table (LUT) of sin/cos values for 2D laser scans.
	  *  Refer to the main method CSinCosLookUpTableFor2DScans::getSinCosForScan()
	  *
	  *  This class is used in mrpt::slam::CPointsMap
	 * \ingroup mrpt_obs_grp
	  */
	class OBS_IMPEXP CSinCosLookUpTableFor2DScans
	{
	public:
		/** A pair of vectors with the cos and sin values. */
		struct TSinCosValues {
			mrpt::vector_float ccos, csin;
		};

		/** Return two vectors with the cos and the sin of the angles for each of the
		  * rays in a scan, computing them only the first time and returning a cached copy the rest.
		  *  Usage:
		  * \code
		  *   CSinCosLookUpTableFor2DScans cache;
		  *   ...
		  *   const CSinCosLookUpTableFor2DScans::TSinCosValues & sincos_vals = cache.getSinCosForScan( scan );
		  * \endcode
		  */
		const TSinCosValues & getSinCosForScan(const CObservation2DRangeScan &scan);

	private:
		std::map<T2DScanProperties,TSinCosValues>  m_cache; //!< The cache of known scans and their sin/cos tables.
	};


} // end NS slam
} // end NS mrpt

#endif
