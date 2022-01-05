/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/CVectorDynamic.h>  //CVectorFloat
#include <mrpt/obs/T2DScanProperties.h>

#include <map>
#include <mutex>

namespace mrpt::obs
{
// Frwd decl:
class CObservation2DRangeScan;

/** A smart look-up-table (LUT) of sin/cos values for 2D laser scans.
 *  Refer to the main method CSinCosLookUpTableFor2DScans::getSinCosForScan()
 *
 *  This class is used in mrpt::maps::CPointsMap
 * \ingroup mrpt_obs_grp
 */
class CSinCosLookUpTableFor2DScans
{
   public:
	CSinCosLookUpTableFor2DScans() = default;
	/** Do NOT copy neither the cache nor the mutex */
	CSinCosLookUpTableFor2DScans(const CSinCosLookUpTableFor2DScans&) {}
	CSinCosLookUpTableFor2DScans& operator=(const CSinCosLookUpTableFor2DScans&)
	{
		return *this;
	}

	/** A pair of vectors with the cos and sin values. */
	struct TSinCosValues
	{
		mrpt::math::CVectorFloat ccos, csin;
	};

	/** Return two vectors with the cos and the sin of the angles for each of
	 * the
	 * rays in a scan, computing them only the first time and returning a
	 * cached copy the rest.
	 *  Usage:
	 * \code
	 *   CSinCosLookUpTableFor2DScans cache;
	 *   ...
	 *   const CSinCosLookUpTableFor2DScans::TSinCosValues & sincos_vals =
	 * cache.getSinCosForScan( scan );
	 * \endcode
	 */
	const TSinCosValues& getSinCosForScan(
		const CObservation2DRangeScan& scan) const;
	/** \overload */
	const TSinCosValues& getSinCosForScan(
		const T2DScanProperties& scan_prop) const;

   private:
	/** The cache of known scans and their sin/cos tables. */
	mutable std::map<T2DScanProperties, TSinCosValues> m_cache;
	mutable std::recursive_mutex m_cache_mtx;
};

}  // namespace mrpt::obs
