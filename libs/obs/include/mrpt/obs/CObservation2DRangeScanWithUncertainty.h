/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/obs/CObservation2DRangeScan.h>

namespace mrpt::obs
{
/** A 2D range scan plus an uncertainty model for each range.
 * \sa mrpt::maps::COccupancyGridMap2D::laserScanSimulatorWithUncertainty()
 */
class CObservation2DRangeScanWithUncertainty
{
 public:
  /** The observation with the mean ranges in the scan field */
  CObservation2DRangeScan rangeScan;
  /** The same ranges than in rangeScan.getScanRange(), for convenience as a
   * math vector container, and with `double` precision */
  mrpt::math::CVectorDouble rangesMean;
  /** The covariance matrix for all the ranges in rangeScan.getScanRange() */
  mrpt::math::CMatrixDouble rangesCovar;

  struct TEvalParams
  {
    /** (Default: 0.5) Probability of having an outlier (dynamic obstacles,
     * not mapped) in each scan ray. */
    double prob_outliers{0.5};
    /** (Default: 0.3) Conditional probability: how many of the "no return"
     * ranges come from a failure to detect a real obstacle. */
    double prob_lost_ray{0.3};
    /** (Default: 1.0m) Maximum std deviation of overall uncertainty for a
     * range prediction to be considered as reliable for evaluation */
    double max_prediction_std_dev{1.0};
    /** (Default: -20) Minimum log-likelihood of a single ray */
    double min_ray_log_lik{-20.0};

    TEvalParams();
  };

  /** Returns a measure of the likelihood of a given scan, compared to this
   * scan variances */
  double evaluateScanLikelihood(
      const CObservation2DRangeScan& otherScan, const TEvalParams& params) const;
};

}  // namespace mrpt::obs
