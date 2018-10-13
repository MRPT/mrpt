/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/core/aligned_std_map.h>

namespace mrpt::topography
{
/** \addtogroup mrpt_topography_grp
 *  @{ */

/** Used to return optional information from mrpt::topography::path_from_rtk_gps
 */
struct TPathFromRTKInfo
{
	/** the path of the "best" GPS. */
	std::map<mrpt::Clock::time_point, mrpt::math::TPoint3D> best_gps_path;
	/** A measure of the quality at each point (may be empty if not there is no
	 * enough information). */
	std::map<mrpt::Clock::time_point, double> mahalabis_quality_measure;
	/** The 6x6 covariance matrix for the uncertainty of each vehicle pose (may
	 * be empty if there is no W_star info). */
	mrpt::aligned_std_map<mrpt::Clock::time_point, mrpt::math::CMatrixDouble66>
		vehicle_uncertainty;
	/** The reference covariance matrix used to compute vehicle_uncertainty. */
	mrpt::math::CMatrixDouble W_star;
};

/** Reconstruct the path of a vehicle equipped with 3 RTK GPSs.
 *  \param robot_path [OUT] The reconstructed vehicle path
 *  \param rawlog [IN] The dataset. It must contain mrpt::obs::CObservationGPS
 * observations with GGA datums.
 *  \param rawlog_first [IN] The index of the first entry to process (first=0)
 *  \param rawlog_last [IN] The index of the last entry to process
 *  \param isGUI [IN] If set to true, some progress dialogs will be shown
 * during the computation (requires MRPT built with support for wxWidgets).
 *  \param disableGPSInterp [IN] Whether to interpolate missing GPS readings
 * between very close datums.
 *  \param path_smooth_filter_size [IN] Size of the window in the pitch & roll
 * noise filtering.
 *  \param outInfo [OUT] Optional output: additional information from the
 * optimization
 *
 *  For more details on the method, refer to the paper: (...)
 * \sa mrpt::topography
 */
void path_from_rtk_gps(
	mrpt::poses::CPose3DInterpolator& robot_path,
	const mrpt::obs::CRawlog& rawlog, size_t rawlog_first, size_t rawlog_last,
	bool isGUI = false, bool disableGPSInterp = false,
	int path_smooth_filter_size = 2, TPathFromRTKInfo* outInfo = nullptr);

/** @} */  // end of grouping

}  // namespace mrpt::topography
