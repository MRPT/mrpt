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

#include <mrpt/core/round.h>  // round()
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/math/CVectorFixed.h>
#include <mrpt/math/transform_gaussian.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationRange.h>
#include <mrpt/random.h>

#include <Eigen/Dense>
#include <cmath>
#include <limits>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::random;
using namespace mrpt::poses;
using namespace std;

// See docs in header
void COccupancyGridMap2D::laserScanSimulator(
    mrpt::obs::CObservation2DRangeScan& inout_Scan,
    const CPose2D& robotPose,
    float threshold,
    size_t N,
    float noiseStd,
    unsigned int decimation,
    float angleNoiseStd) const
{
  MRPT_START

  ASSERT_(decimation >= 1);
  ASSERT_(N >= 2);

  // Sensor pose in global coordinates
  CPose3D sensorPose3D = CPose3D(robotPose) + inout_Scan.sensorPose;
  // Aproximation: grid is 2D !!!
  CPose2D sensorPose(sensorPose3D);

  // Scan size:
  inout_Scan.resizeScan(N);

  double A = sensorPose.phi() + (inout_Scan.rightToLeft ? -0.5 : +0.5) * inout_Scan.aperture;
  const double AA =
      (inout_Scan.rightToLeft ? 1.0 : -1.0) * (inout_Scan.aperture / static_cast<double>(N - 1));

  const float free_thres = 1.0f - threshold;

  for (size_t i = 0; i < N; i += decimation, A += AA * decimation)
  {
    bool valid;
    float out_range;
    simulateScanRay(
        sensorPose.x(), sensorPose.y(), A, out_range, valid, inout_Scan.maxRange, free_thres,
        noiseStd, angleNoiseStd);
    inout_Scan.setScanRange(i, out_range);
    inout_Scan.setScanRangeValidity(i, valid);
  }

  MRPT_END
}

void COccupancyGridMap2D::sonarSimulator(
    CObservationRange& inout_observation,
    const CPose2D& robotPose,
    float threshold,
    float rangeNoiseStd,
    float angleNoiseStd) const
{
  const float free_thres = 1.0f - threshold;

  for (auto itR = inout_observation.begin(); itR != inout_observation.end(); ++itR)
  {
    const CPose2D sensorAbsolutePose = CPose2D(CPose3D(robotPose) + CPose3D(itR->sensorPose));

    // For each sonar cone, simulate several rays and keep the shortest
    // distance:
    ASSERT_(inout_observation.sensorConeAperture > 0);
    size_t nRays = round(1 + inout_observation.sensorConeAperture / 1.0_deg);

    double direction = sensorAbsolutePose.phi() - 0.5 * inout_observation.sensorConeAperture;
    const double Adir = inout_observation.sensorConeAperture / static_cast<double>(nRays);

    float min_detected_obs = 0;
    for (size_t i = 0; i < nRays; i++, direction += Adir)
    {
      bool valid;
      float sim_rang;
      simulateScanRay(
          sensorAbsolutePose.x(), sensorAbsolutePose.y(), direction, sim_rang, valid,
          inout_observation.maxSensorDistance, free_thres, rangeNoiseStd, angleNoiseStd);

      if (valid && (sim_rang < min_detected_obs || !i)) min_detected_obs = sim_rang;
    }
    // Save:
    itR->sensedDistance = min_detected_obs;
  }
}

void COccupancyGridMap2D::simulateScanRay(
    const double start_x,
    const double start_y,
    const double angle_direction,
    float& out_range,
    bool& out_valid,
    const double max_range_meters,
    const float threshold_free,
    const double noiseStd,
    const double angleNoiseStd) const
{
  const double A_ =
      angle_direction +
      (angleNoiseStd > .0 ? getRandomGenerator().drawGaussian1D_normalized() * angleNoiseStd : .0);

  // Unit vector in the direction of the ray:
  const double Arx = std::cos(A_);
  const double Ary = std::sin(A_);

  // Exact traversal of all the cells crossed by the ray (Amanatides & Woo),
  // so no cell is skipped, and the range is the distance at which the ray
  // enters the first non-free cell, i.e. its closest point to the sensor.
  const double fx = (start_x - m_xMin) / m_resolution;
  const double fy = (start_y - m_yMin) / m_resolution;
  int x = static_cast<int>(std::floor(fx));
  int y = static_cast<int>(std::floor(fy));

  const int stepX = Arx >= 0 ? 1 : -1;
  const int stepY = Ary >= 0 ? 1 : -1;
  constexpr double kInf = std::numeric_limits<double>::infinity();
  // Ray length [m] to cross one cell along x / y:
  const double tDeltaX = std::abs(Arx) > 1e-12 ? m_resolution / std::abs(Arx) : kInf;
  const double tDeltaY = std::abs(Ary) > 1e-12 ? m_resolution / std::abs(Ary) : kInf;
  // Ray length [m] to the first cell boundary along x / y:
  double tMaxX = std::abs(Arx) > 1e-12
                     ? (stepX > 0 ? (x + 1 - fx) : (fx - x)) * m_resolution / std::abs(Arx)
                     : kInf;
  double tMaxY = std::abs(Ary) > 1e-12
                     ? (stepY > 0 ? (y + 1 - fy) : (fy - y)) * m_resolution / std::abs(Ary)
                     : kInf;

  const cellType threshold_free_int = p2l(threshold_free);
  cellType hitCellOcc_int = 0;  // p2l(0.5f)
  double t = 0;                 // ray length at which it enters the current cell
  bool insideGrid = false;
  bool hit = false;

  for (;;)
  {
    insideGrid =
        x >= 0 && y >= 0 && x < static_cast<int>(m_size_x) && y < static_cast<int>(m_size_y);
    if (!insideGrid || t > max_range_meters) break;

    hitCellOcc_int = m_map[x + y * m_size_x];
    if (hitCellOcc_int <= threshold_free_int)
    {
      hit = true;
      break;
    }
    // Move on to the next cell crossed by the ray:
    if (tMaxX < tMaxY)
    {
      t = tMaxX;
      tMaxX += tDeltaX;
      x += stepX;
    }
    else
    {
      t = tMaxY;
      tMaxY += tDeltaY;
      y += stepY;
    }
  }

  // Store:
  // Out of the grid, out of range, or an unknown cell?
  if (!hit || std::abs(hitCellOcc_int) <= 1)
  {
    out_valid = false;
    out_range = static_cast<float>(max_range_meters);
  }
  else
  {  // No: The normal case:
    out_range = static_cast<float>(t);
    out_valid = true;
    // Add additive Gaussian noise:
    if (noiseStd > 0)
      out_range += static_cast<float>(noiseStd * getRandomGenerator().drawGaussian1D_normalized());
  }
}

COccupancyGridMap2D::TLaserSimulUncertaintyParams::TLaserSimulUncertaintyParams()

    = default;

COccupancyGridMap2D::TLaserSimulUncertaintyResult::TLaserSimulUncertaintyResult() = default;

struct TFunctorLaserSimulData
{
  const COccupancyGridMap2D::TLaserSimulUncertaintyParams* params;
  const COccupancyGridMap2D* grid;
};

static void func_laserSimul_callback(
    const mrpt::math::CVectorFixedDouble<3>& x_pose,
    const TFunctorLaserSimulData& fixed_param,
    mrpt::math::CVectorDouble& y_scanRanges)
{
  ASSERT_(fixed_param.params && fixed_param.grid);
  ASSERT_(fixed_param.params->decimation >= 1);
  ASSERT_(fixed_param.params->nRays >= 2);

  const size_t N = fixed_param.params->nRays;

  // Sensor pose in global coordinates
  const CPose3D sensorPose3D =
      CPose3D(x_pose[0], x_pose[1], .0, x_pose[2], .0, .0) + fixed_param.params->sensorPose;
  // Aproximation: grid is 2D !!!
  const CPose2D sensorPose(sensorPose3D);

  // Scan size:
  y_scanRanges.resize(N);

  double A = sensorPose.phi() +
             (fixed_param.params->rightToLeft ? -0.5 : +0.5) * fixed_param.params->aperture;
  const double AA = (fixed_param.params->rightToLeft ? 1.0 : -1.0) *
                    (fixed_param.params->aperture / static_cast<double>(N - 1));

  const float free_thres = 1.0f - fixed_param.params->threshold;

  for (size_t i = 0; i < N;
       i += fixed_param.params->decimation, A += AA * fixed_param.params->decimation)
  {
    bool valid;
    float range;

    fixed_param.grid->simulateScanRay(
        sensorPose.x(), sensorPose.y(), A, range, valid, fixed_param.params->maxRange, free_thres,
        .0 /*noiseStd*/, .0 /*angleNoiseStd*/);
    y_scanRanges[i] = valid ? range : fixed_param.params->maxRange;
  }
}

void COccupancyGridMap2D::laserScanSimulatorWithUncertainty(
    const COccupancyGridMap2D::TLaserSimulUncertaintyParams& in_params,
    COccupancyGridMap2D::TLaserSimulUncertaintyResult& out_results) const
{
  const mrpt::math::CVectorFixedDouble<3> robPoseMean = in_params.robotPose.mean.asVectorVal();

  TFunctorLaserSimulData simulData;
  simulData.grid = this;
  simulData.params = &in_params;

  switch (in_params.method)
  {
    case sumUnscented:
      mrpt::math::transform_gaussian_unscented(
          robPoseMean,                // x_mean
          in_params.robotPose.cov,    // x_cov
          &func_laserSimul_callback,  // void  (*functor)(const
          // VECTORLIKE1 &x,const USERPARAM
          // &fixed_param, VECTORLIKE3 &y)
          simulData,  // const USERPARAM &fixed_param,
          out_results.scanWithUncert.rangesMean, out_results.scanWithUncert.rangesCovar,
          nullptr,  // elem_do_wrap2pi,
          in_params.UT_alpha, in_params.UT_kappa,
          in_params.UT_beta  // alpha, K, beta
      );
      break;
    case sumMonteCarlo:
      //
      mrpt::math::transform_gaussian_montecarlo(
          robPoseMean,                // x_mean
          in_params.robotPose.cov,    // x_cov
          &func_laserSimul_callback,  // void (*functor)(const
          // VECTORLIKE1 &x,const USERPARAM
          // &fixed_param, VECTORLIKE3 &y)
          simulData,  // const USERPARAM &fixed_param,
          out_results.scanWithUncert.rangesMean, out_results.scanWithUncert.rangesCovar,
          in_params.MC_samples);
      break;
    default:
      throw std::runtime_error("[laserScanSimulatorWithUncertainty] Unknown `method` value");
      break;
  };

  // Outputs:
  out_results.scanWithUncert.rangeScan.aperture = in_params.aperture;
  out_results.scanWithUncert.rangeScan.maxRange = in_params.maxRange;
  out_results.scanWithUncert.rangeScan.rightToLeft = in_params.rightToLeft;
  out_results.scanWithUncert.rangeScan.sensorPose = in_params.sensorPose;

  out_results.scanWithUncert.rangeScan.resizeScan(in_params.nRays);
  for (unsigned i = 0; i < in_params.nRays; i++)
  {
    out_results.scanWithUncert.rangeScan.setScanRange(
        i, static_cast<float>(out_results.scanWithUncert.rangesMean[i]));
    out_results.scanWithUncert.rangeScan.setScanRangeValidity(i, true);
  }

  // Add minimum uncertainty: grid cell resolution:
  out_results.scanWithUncert.rangesCovar.asEigen().diagonal().array() +=
      0.5 * m_resolution * m_resolution;
}
