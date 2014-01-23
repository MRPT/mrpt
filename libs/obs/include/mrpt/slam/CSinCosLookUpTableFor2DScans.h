/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
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
