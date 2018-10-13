/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/obs/obs_frwds.h>
#include <mrpt/obs/CSensoryFrame.h>

namespace mrpt::slam
{
/**  \addtogroup mrpt_slam_grp
 *   @{ */

/** @name Observations overlap functions
	@{  */

/** Estimates the "overlap" or "matching ratio" of two observations (range
 * [0,1]), possibly taking into account their relative positions.
 *  \note This is used in mrpt::slam::CIncrementalMapPartitioner
 */
double observationsOverlap(
	const mrpt::obs::CObservation* o1, const mrpt::obs::CObservation* o2,
	const mrpt::poses::CPose3D* pose_o2_wrt_o1 = nullptr);

/** Estimates the "overlap" or "matching ratio" of two observations (range
 * [0,1]), possibly taking into account their relative positions.
 *  \note This is used in mrpt::slam::CIncrementalMapPartitioner
 */
inline double observationsOverlap(
	const mrpt::obs::CObservation::Ptr& o1,
	const mrpt::obs::CObservation::Ptr& o2,
	const mrpt::poses::CPose3D* pose_o2_wrt_o1 = nullptr)
{
	return observationsOverlap(o1.get(), o2.get(), pose_o2_wrt_o1);
}

/** Estimates the "overlap" or "matching ratio" of two set of observations
 * (range [0,1]), possibly taking into account their relative positions.
 *   This method computes the average between each of the observations in each
 * SF.
 *  \note This is used in mrpt::slam::CIncrementalMapPartitioner
 */
double observationsOverlap(
	const mrpt::obs::CSensoryFrame& sf1, const mrpt::obs::CSensoryFrame& sf2,
	const mrpt::poses::CPose3D* pose_sf2_wrt_sf1 = nullptr);

/** Estimates the "overlap" or "matching ratio" of two set of observations
 * (range [0,1]), possibly taking into account their relative positions.
 *   This method computes the average between each of the observations in each
 * SF.
 *  \note This is used in mrpt::slam::CIncrementalMapPartitioner
 */
inline double observationsOverlap(
	const mrpt::obs::CSensoryFrame::Ptr& sf1,
	const mrpt::obs::CSensoryFrame::Ptr& sf2,
	const mrpt::poses::CPose3D* pose_sf2_wrt_sf1 = nullptr)
{
	return observationsOverlap(*sf1.get(), *sf2.get(), pose_sf2_wrt_sf1);
}

/** @} */

/** @} */  // end grouping

}  // namespace mrpt::slam
