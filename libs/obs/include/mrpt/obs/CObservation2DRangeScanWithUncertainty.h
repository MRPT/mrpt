/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CObservation2DRangeScanWithUncertainty_H
#define CObservation2DRangeScanWithUncertainty_H

#include <mrpt/obs/CObservation2DRangeScan.h>
#include <vector>

namespace mrpt
{
namespace obs
{
	/** A 2D range scan plus an uncertainty model for each range.
	  * \sa mrpt::maps::COccupancyGridMap2D::laserScanSimulatorWithUncertainty()
	  */
	class OBS_IMPEXP CObservation2DRangeScanWithUncertainty
	{
	public:
		CObservation2DRangeScan  rangeScan;     //!< The observation with the mean ranges in the scan field
		Eigen::VectorXd          rangesMean;    //!< The same ranges than in rangeScan.scan[], for convenience as an Eigen container, and with `double` precision
		Eigen::MatrixXd          rangesCovar;   //!< The covariance matrix for all the ranges in rangeScan.scan[]

		/** Returns a measure of the likelihood of a given scan, compared to this scan variances */
		double evaluateScanLikelihood(const CObservation2DRangeScan& otherScan, double outliers_prob = 0.5) const;
	};

} // End of namespace
} // End of namespace
#endif
