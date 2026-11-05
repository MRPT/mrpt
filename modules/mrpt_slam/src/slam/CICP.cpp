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

#include <mrpt/config/CConfigFileBase.h>  // MRPT_LOAD_*()
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/ops_containers.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFSOG.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/slam/CICP.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/tfest.h>

#include <Eigen/Dense>

using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::math;
using namespace mrpt::tfest;
using namespace mrpt::system;
using namespace mrpt::poses;
using namespace std;

namespace
{
/** Weight of a correspondence with squared distance d2 in the Gauss-Newton
 * Hessian of the robust cost, normalized to 1 for small residuals. */
double robustWeight(double d2, const CICP::TConfigParams& o)
{
  if (!o.use_kernel)
  {
    return 1.0;
  }
  const double rho2 = mrpt::square(o.kernel_rho);
  return mrpt::square(rho2 / (d2 + rho2));
}

/** Covariance of a SE(2) ICP solution: the inverse of the Gauss-Newton
 * Hessian of the robust negative log-likelihood of all correspondences, with
 * an isotropic variance options.covariance_varPoints per point coordinate. */
CMatrixDouble33 icpCovarianceSE2(
    const TMatchingPairList& corrs, const TPose2D& q, const CICP::TConfigParams& o)
{
  const double c = std::cos(q.phi);
  const double s = std::sin(q.phi);
  Eigen::Matrix3d info = Eigen::Matrix3d::Zero();
  for (const auto& p : corrs)
  {
    const double lx = p.local.x;
    const double ly = p.local.y;
    const double ex = q.x + c * lx - s * ly - p.global.x;
    const double ey = q.y + s * lx + c * ly - p.global.y;
    Eigen::Matrix<double, 2, 3> J;
    J << 1, 0, -s * lx - c * ly, 0, 1, c * lx - s * ly;
    info += robustWeight(ex * ex + ey * ey, o) * (J.transpose() * J);
  }
  // Avoid singular matrices (e.g. degenerate geometry), with a negligible
  // effect otherwise:
  info += 1e-9 * Eigen::Matrix3d::Identity();
  CMatrixDouble33 cov;
  cov.asEigen() = o.covariance_varPoints * info.inverse();
  return cov;
}

/** Like icpCovarianceSE2(), for SE(3) in (x,y,z,yaw,pitch,roll) */
CMatrixDouble66 icpCovarianceSE3(
    const TMatchingPairList& corrs, const CPose3D& q, const CICP::TConfigParams& o)
{
  Eigen::Matrix<double, 6, 6> info = Eigen::Matrix<double, 6, 6>::Zero();
  for (const auto& p : corrs)
  {
    double gx;
    double gy;
    double gz;
    CMatrixDouble36 J;
    q.composePoint(p.local.x, p.local.y, p.local.z, gx, gy, gz, std::nullopt, J);
    const double d2 = mrpt::square(gx - p.global.x) + mrpt::square(gy - p.global.y) +
                      mrpt::square(gz - p.global.z);
    info += robustWeight(d2, o) * (J.asEigen().transpose() * J.asEigen());
  }
  info += 1e-9 * Eigen::Matrix<double, 6, 6>::Identity();
  CMatrixDouble66 cov;
  cov.asEigen() = o.covariance_varPoints * info.inverse();
  return cov;
}
}  // namespace

CPosePDF::Ptr CICP::AlignPDF(
    const mrpt::maps::CMetricMap* m1,
    const mrpt::maps::CMetricMap* mm2,
    const CPosePDFGaussian& initialEstimationPDF,
    mrpt::optional_ref<TMetricMapAlignmentResult> outInfo)
{
  MRPT_START

  CTicTac tictac;
  TReturnInfo outInfoVal;
  CPosePDF::Ptr resultPDF;

  if (outInfo) tictac.Tic();

  switch (options.ICP_algorithm)
  {
    case icpClassic:
      resultPDF = ICP_Method_Classic(m1, mm2, initialEstimationPDF, outInfoVal);
      break;
    case icpLevenbergMarquardt:
      resultPDF = ICP_Method_LM(m1, mm2, initialEstimationPDF, outInfoVal);
      break;
    default:
      THROW_EXCEPTION_FMT(
          "Invalid value for ICP_algorithm: %i", static_cast<int>(options.ICP_algorithm));
  }  // end switch

  if (outInfo) outInfoVal.executionTime = tictac.Tac();

  // Copy the output info if requested:
  if (outInfo)
    if (auto* o = dynamic_cast<TReturnInfo*>(&outInfo.value().get()); o) *o = outInfoVal;

  return resultPDF;

  MRPT_END
}

/*---------------------------------------------------------------
          loadFromConfigFile
  ---------------------------------------------------------------*/
void CICP::TConfigParams::loadFromConfigFile(
    const mrpt::config::CConfigFileBase& iniFile, const std::string& section)
{
  MRPT_LOAD_CONFIG_VAR(maxIterations, int, iniFile, section);
  MRPT_LOAD_CONFIG_VAR(minAbsStep_trans, double, iniFile, section);
  MRPT_LOAD_CONFIG_VAR(minAbsStep_rot, double, iniFile, section);

  ICP_algorithm = iniFile.read_enum<TICPAlgorithm>(section, "ICP_algorithm", ICP_algorithm);
  ICP_covariance_method = iniFile.read_enum<TICPCovarianceMethod>(
      section, "ICP_covariance_method", ICP_covariance_method);

  MRPT_LOAD_CONFIG_VAR(thresholdDist, double, iniFile, section);
  thresholdAng =
      DEG2RAD(iniFile.read_double(section.c_str(), "thresholdAng_DEG", RAD2DEG(thresholdAng)));

  MRPT_LOAD_CONFIG_VAR(ALFA, double, iniFile, section);
  MRPT_LOAD_CONFIG_VAR(smallestThresholdDist, double, iniFile, section);
  MRPT_LOAD_CONFIG_VAR(onlyUniqueRobust, bool, iniFile, section);
  MRPT_LOAD_CONFIG_VAR(doRANSAC, bool, iniFile, section);
  MRPT_LOAD_CONFIG_VAR(covariance_varPoints, double, iniFile, section);

  MRPT_LOAD_CONFIG_VAR(ransac_minSetSize, int, iniFile, section);
  MRPT_LOAD_CONFIG_VAR(ransac_maxSetSize, int, iniFile, section);
  MRPT_LOAD_CONFIG_VAR(ransac_mahalanobisDistanceThreshold, double, iniFile, section);
  MRPT_LOAD_CONFIG_VAR(ransac_nSimulations, int, iniFile, section);
  MRPT_LOAD_CONFIG_VAR(normalizationStd, double, iniFile, section);
  MRPT_LOAD_CONFIG_VAR(ransac_fuseByCorrsMatch, bool, iniFile, section);
  MRPT_LOAD_CONFIG_VAR(ransac_fuseMaxDiffXY, double, iniFile, section);
  ransac_fuseMaxDiffPhi = DEG2RAD(iniFile.read_double(
      section.c_str(), "ransac_fuseMaxDiffPhi_DEG", RAD2DEG(ransac_fuseMaxDiffPhi)));

  MRPT_LOAD_CONFIG_VAR(kernel_rho, double, iniFile, section);
  MRPT_LOAD_CONFIG_VAR(use_kernel, bool, iniFile, section);
  MRPT_LOAD_CONFIG_VAR(Axy_aprox_derivatives, double, iniFile, section);
  MRPT_LOAD_CONFIG_VAR(LM_initial_lambda, double, iniFile, section);

  MRPT_LOAD_CONFIG_VAR(skip_cov_calculation, bool, iniFile, section);
  MRPT_LOAD_CONFIG_VAR(skip_quality_calculation, bool, iniFile, section);

  MRPT_LOAD_CONFIG_VAR(corresponding_points_decimation, int, iniFile, section);
}

void CICP::TConfigParams::saveToConfigFile(
    mrpt::config::CConfigFileBase& c, const std::string& s) const
{
  MRPT_SAVE_CONFIG_VAR_COMMENT(ICP_algorithm, "The ICP algorithm to use (see enum TICPAlgorithm)");
  MRPT_SAVE_CONFIG_VAR_COMMENT(
      ICP_covariance_method,
      "Method to use for covariance estimation (see enum "
      "TICPCovarianceMethod)");
  MRPT_SAVE_CONFIG_VAR_COMMENT(
      onlyUniqueRobust,
      "Only the closest correspondence for each reference point will be "
      "kept");
  MRPT_SAVE_CONFIG_VAR_COMMENT(maxIterations, "Iterations");
  MRPT_SAVE_CONFIG_VAR_COMMENT(minAbsStep_trans, "Termination criteria");
  MRPT_SAVE_CONFIG_VAR_COMMENT(minAbsStep_rot, "Termination criteria");
  MRPT_SAVE_CONFIG_VAR_COMMENT(
      thresholdDist, "Initial threshold distance for two points to be a match");
  MRPT_SAVE_CONFIG_VAR_COMMENT(
      thresholdAng, "Initial threshold distance for two points to be a match");
  MRPT_SAVE_CONFIG_VAR_COMMENT(
      ALFA, "The scale factor for thresholds every time convergence is achieved");
  MRPT_SAVE_CONFIG_VAR_COMMENT(
      smallestThresholdDist,
      "The size for threshold such that iterations will stop, "
      "since it is considered precise enough.");
  MRPT_SAVE_CONFIG_VAR_COMMENT(covariance_varPoints, "");
  MRPT_SAVE_CONFIG_VAR_COMMENT(doRANSAC, "Perform a RANSAC step");
  MRPT_SAVE_CONFIG_VAR_COMMENT(ransac_minSetSize, "");
  MRPT_SAVE_CONFIG_VAR_COMMENT(ransac_maxSetSize, "");
  MRPT_SAVE_CONFIG_VAR_COMMENT(ransac_nSimulations, "");
  MRPT_SAVE_CONFIG_VAR_COMMENT(ransac_mahalanobisDistanceThreshold, "");
  MRPT_SAVE_CONFIG_VAR_COMMENT(normalizationStd, "");
  MRPT_SAVE_CONFIG_VAR_COMMENT(ransac_fuseByCorrsMatch, "");
  MRPT_SAVE_CONFIG_VAR_COMMENT(ransac_fuseMaxDiffXY, "");
  MRPT_SAVE_CONFIG_VAR_COMMENT(ransac_fuseMaxDiffPhi, "");
  MRPT_SAVE_CONFIG_VAR_COMMENT(kernel_rho, "");
  MRPT_SAVE_CONFIG_VAR_COMMENT(use_kernel, "");
  MRPT_SAVE_CONFIG_VAR_COMMENT(Axy_aprox_derivatives, "");
  MRPT_SAVE_CONFIG_VAR_COMMENT(LM_initial_lambda, "");
  MRPT_SAVE_CONFIG_VAR_COMMENT(skip_cov_calculation, "");
  MRPT_SAVE_CONFIG_VAR_COMMENT(skip_quality_calculation, "");
  MRPT_SAVE_CONFIG_VAR_COMMENT(corresponding_points_decimation, "");
}

float CICP::kernel(float x2, float rho2) { return options.use_kernel ? (x2 / (x2 + rho2)) : x2; }

/*----------------------------------------------------------------------------

          ICP_Method_Classic

  ----------------------------------------------------------------------------*/
CPosePDF::Ptr CICP::ICP_Method_Classic(
    const mrpt::maps::CMetricMap* m1,
    const mrpt::maps::CMetricMap* mm2,
    const CPosePDFGaussian& initialEstimationPDF,
    TReturnInfo& outInfo)
{
  MRPT_START

  // The result can be either a Gaussian or a SOG:
  CPosePDF::Ptr resultPDF;
  CPosePDFGaussian::Ptr gaussPdf;
  CPosePDFSOG::Ptr SOG;

  size_t nCorrespondences = 0;
  bool keepApproaching;
  CPose2D grossEst = initialEstimationPDF.mean;
  mrpt::tfest::TMatchingPairList correspondences, old_correspondences;
  CPose2D lastMeanPose;

  // Assure the class of the maps:
  const mrpt::maps::CMetricMap* m2 = mm2;

  // Asserts:
  // -----------------
  ASSERT_(options.ALFA >= 0 && options.ALFA < 1);

  // The algorithm output auxiliar info:
  // -------------------------------------------------
  outInfo.nIterations = 0;
  outInfo.goodness = 1;
  outInfo.quality = 0;

  // The gaussian PDF to estimate:
  // ------------------------------------------------------
  gaussPdf = std::make_shared<CPosePDFGaussian>();

  // First gross approximation:
  gaussPdf->mean = grossEst;

  // Initial thresholds:
  mrpt::maps::TMatchingParams matchParams;
  mrpt::maps::TMatchingExtraResults matchExtraResults;

  matchParams.maxDistForCorrespondence =
      static_cast<float>(options.thresholdDist);  // Distance threshold.
  matchParams.maxAngularDistForCorrespondence =
      static_cast<float>(options.thresholdAng);  // Angular threshold.
  // Option onlyClosestCorrespondences removed in MRPT 2.0.
  matchParams.onlyKeepTheClosest = true;
  matchParams.onlyUniqueRobust = options.onlyUniqueRobust;
  matchParams.decimation_other_map_points = options.corresponding_points_decimation;

  // Ensure maps are not empty!
  // ------------------------------------------------------
  if (!m2->isEmpty())
  {
    matchParams.offset_other_map_points = 0;

    // ------------------------------------------------------
    //					The ICP loop
    // ------------------------------------------------------
    do
    {
      // ------------------------------------------------------
      //		Find the matching (for a points map)
      // ------------------------------------------------------
      // Pivot point for angular measurements
      matchParams.angularDistPivotPoint = mrpt::math::TPoint3D(gaussPdf->mean.translation());

      m1->determineMatching2D(
          m2,              // The other map
          gaussPdf->mean,  // The other map pose
          correspondences, matchParams, matchExtraResults);

      nCorrespondences = correspondences.size();

      // ***DEBUG***
      //				correspondences.dumpToFile("debug_correspondences.txt");

      if (!nCorrespondences)
      {
        // Nothing we can do !!
        keepApproaching = false;
      }
      else
      {
        // Compute the estimated pose.
        //  (Method from paper of J.Gonzalez, Martinez y Morales)
        // ----------------------------------------------------------------------
        mrpt::math::TPose2D est_mean;
        mrpt::tfest::se2_l2(correspondences, est_mean);

        gaussPdf->mean = mrpt::poses::CPose2D(est_mean);

        // If matching has not changed, decrease the thresholds:
        // --------------------------------------------------------
        keepApproaching = true;
        if (!(fabs(lastMeanPose.x() - gaussPdf->mean.x()) > options.minAbsStep_trans ||
              fabs(lastMeanPose.y() - gaussPdf->mean.y()) > options.minAbsStep_trans ||
              fabs(math::wrapToPi(lastMeanPose.phi() - gaussPdf->mean.phi())) >
                  options.minAbsStep_rot))
        {
          matchParams.maxDistForCorrespondence *= static_cast<float>(options.ALFA);
          matchParams.maxAngularDistForCorrespondence *= static_cast<float>(options.ALFA);
          if (matchParams.maxDistForCorrespondence <
              static_cast<float>(options.smallestThresholdDist))
            keepApproaching = false;

          if (++matchParams.offset_other_map_points >= options.corresponding_points_decimation)
            matchParams.offset_other_map_points = 0;
        }

        lastMeanPose = gaussPdf->mean;

      }  // end of "else, there are correspondences"

      // Next iteration:
      outInfo.nIterations++;

      if (outInfo.nIterations >= options.maxIterations &&
          matchParams.maxDistForCorrespondence > static_cast<float>(options.smallestThresholdDist))
      {
        matchParams.maxDistForCorrespondence *= static_cast<float>(options.ALFA);
      }

    } while (
        (keepApproaching && outInfo.nIterations < options.maxIterations) ||
        (outInfo.nIterations >= options.maxIterations &&
         matchParams.maxDistForCorrespondence > static_cast<float>(options.smallestThresholdDist)));

    // -------------------------------------------------
    //   Obtain the covariance matrix of the estimation
    // -------------------------------------------------
    if (!options.skip_cov_calculation && nCorrespondences)
    {
      switch (options.ICP_covariance_method)
      {
        // ----------------------------------------------
        // METHOD: MSE linear estimation
        // ----------------------------------------------
        case icpCovLinealMSE:
          mrpt::tfest::se2_l2(correspondences, *gaussPdf);
          // Scale covariance:
          gaussPdf->cov *= options.covariance_varPoints;
          break;

        // ----------------------------------------------
        // METHOD: Gauss-Newton covariance of the robust
        //  (kernel_rho) cost at the solution.
        // ----------------------------------------------
        case icpCovFiniteDifferences:
          gaussPdf->cov = icpCovarianceSE2(correspondences, gaussPdf->mean.asTPose(), options);
          break;
        default:
          THROW_EXCEPTION_FMT(
              "Invalid value for ICP_covariance_method: %i",
              static_cast<int>(options.ICP_covariance_method));
      }
    }

    outInfo.goodness = matchExtraResults.correspondencesRatio;

    if (!nCorrespondences || options.skip_quality_calculation)
    {
      outInfo.quality = matchExtraResults.correspondencesRatio;
    }
    else
    {
      // Compute a crude estimate of the quality of scan matching at this
      // local minimum:
      // -----------------------------------------------------------------------------------
      float Axy = matchParams.maxDistForCorrespondence * 0.9f;

      CPose2D P0(gaussPdf->mean);
      CPose2D PX1(P0);
      PX1.x_incr(-Axy);
      CPose2D PX2(P0);
      PX2.x_incr(+Axy);
      CPose2D PY1(P0);
      PY1.y_incr(-Axy);
      CPose2D PY2(P0);
      PY2.y_incr(+Axy);

      // Pivot point for angular measurements
      matchParams.angularDistPivotPoint = mrpt::math::TPoint3D(gaussPdf->mean.translation());

      // Do not overwrite "correspondences", used below by RANSAC:
      mrpt::tfest::TMatchingPairList qualityCorrs;

      m1->determineMatching2D(
          m2,  // The other map
          P0,  // The other map pose
          qualityCorrs, matchParams, matchExtraResults);
      const float E0 = matchExtraResults.correspondencesRatio;

      m1->determineMatching2D(
          m2,   // The other map
          PX1,  // The other map pose
          qualityCorrs, matchParams, matchExtraResults);
      const float EX1 = matchExtraResults.correspondencesRatio;

      m1->determineMatching2D(
          m2,   // The other map
          PX2,  // The other map pose
          qualityCorrs, matchParams, matchExtraResults);
      const float EX2 = matchExtraResults.correspondencesRatio;

      m1->determineMatching2D(
          m2,   // The other map
          PY1,  // The other map pose
          qualityCorrs, matchParams, matchExtraResults);
      const float EY1 = matchExtraResults.correspondencesRatio;
      m1->determineMatching2D(
          m2,   // The other map
          PY2,  // The other map pose
          qualityCorrs, matchParams, matchExtraResults);
      const float EY2 = matchExtraResults.correspondencesRatio;

      outInfo.quality = -max(EX1 - E0, max(EX2 - E0, max(EY1 - E0, EY2 - E0))) / (E0 + 1e-1);
    }

  }  // end of "if m2 is not empty"

  // We'll return a CPosePDFGaussian or a CPosePDFSOG if RANSAC is enabled:
  // -----------------------------------------------------------------------

  // RANSAC?
  if (options.doRANSAC)
  {
    mrpt::tfest::TSE2RobustParams params;
    params.ransac_minSetSize = options.ransac_minSetSize;
    params.ransac_maxSetSize = options.ransac_maxSetSize;
    params.ransac_mahalanobisDistanceThreshold = options.ransac_mahalanobisDistanceThreshold;
    params.ransac_nSimulations = options.ransac_nSimulations;
    params.ransac_fuseByCorrsMatch = options.ransac_fuseByCorrsMatch;
    params.ransac_fuseMaxDiffXY = options.ransac_fuseMaxDiffXY;
    params.ransac_fuseMaxDiffPhi = options.ransac_fuseMaxDiffPhi;
    params.ransac_algorithmForLandmarks = false;

    mrpt::tfest::TSE2RobustResult results;
    mrpt::tfest::se2_l2_robust(correspondences, options.normalizationStd, params, results);

    SOG = std::make_shared<CPosePDFSOG>();
    *SOG = results.transformation;

    // And return the SOG:
    resultPDF = SOG;
  }
  else
  {
    // Return the gaussian distribution:
    resultPDF = gaussPdf;
  }

  return resultPDF;

  MRPT_END
}

/*----------------------------------------------------------------------------

            ICP_Method_LM

  ----------------------------------------------------------------------------*/
CPosePDF::Ptr CICP::ICP_Method_LM(
    const mrpt::maps::CMetricMap* mm1,
    const mrpt::maps::CMetricMap* m2,
    const CPosePDFGaussian& initialEstimationPDF,
    TReturnInfo& outInfo)
{
  MRPT_START

  // The result can be either a Gaussian or a SOG:
  size_t nCorrespondences = 0;
  bool keepIteratingICP;
  CPose2D grossEst = initialEstimationPDF.mean;
  TMatchingPairList correspondences, old_correspondences;
  CPose2D lastMeanPose;
  std::vector<float> other_xs_trans, other_ys_trans;  // temporary container
  // of "other" map (map2)
  // transformed by "q"
  CMatrixFloat dJ_dq;  // The jacobian
  CPose2D q;           // The updated 2D transformation estimate
  CPose2D q_new;

  const bool onlyUniqueRobust = options.onlyUniqueRobust;

  // Assure the class of the maps:
  ASSERT_(mm1->GetRuntimeClass()->derivedFrom(CLASS_ID(CPointsMap)));
  const auto* m1 = dynamic_cast<const CPointsMap*>(mm1);

  // Asserts:
  // -----------------
  ASSERT_(options.ALFA > 0 && options.ALFA < 1);

  // The algorithm output auxiliar info:
  // -------------------------------------------------
  outInfo.nIterations = 0;
  outInfo.goodness = 1.0f;

  TMatchingParams matchParams;
  TMatchingExtraResults matchExtraResults;

  matchParams.maxDistForCorrespondence =
      static_cast<float>(options.thresholdDist);  // Distance threshold
  matchParams.maxAngularDistForCorrespondence =
      static_cast<float>(options.thresholdAng);  // Angular threshold
  matchParams.onlyKeepTheClosest = true;
  matchParams.onlyUniqueRobust = onlyUniqueRobust;
  matchParams.decimation_other_map_points = options.corresponding_points_decimation;

  // The gaussian PDF to estimate:
  // ------------------------------------------------------
  // First gross approximation:
  q = grossEst;

  // For LM inverse
  CMatrixFixed<float, 3, 3> C;
  CMatrixFixed<float, 3, 3> C_inv;  // This will keep the cov. matrix at the end

  // Ensure maps are not empty!
  // ------------------------------------------------------
  if (!m2->isEmpty())
  {
    matchParams.offset_other_map_points = 0;
    // ------------------------------------------------------
    //					The ICP loop
    // ------------------------------------------------------
    do
    {
      // ------------------------------------------------------
      //		Find the matching (for a points map)
      // ------------------------------------------------------
      // Pivot point for angular measurements:
      matchParams.angularDistPivotPoint = mrpt::math::TPoint3D(q.translation());

      m1->determineMatching2D(
          m2,  // The other map
          q,   // The other map pose
          correspondences, matchParams, matchExtraResults);

      nCorrespondences = correspondences.size();

      if (!nCorrespondences)
      {
        // Nothing we can do !!
        keepIteratingICP = false;
      }
      else
      {
        // Compute the estimated pose through iterative least-squares:
        // Levenberg-Marquardt
        // ----------------------------------------------------------------------
        dJ_dq.setSize(3, nCorrespondences);  // The jacobian of the
        // error function wrt the
        // transformation q

        double lambda = options.LM_initial_lambda;  // The LM parameter

        double ccos = cos(q.phi());
        double csin = sin(q.phi());

        float w1, w2, w3;
        double q1, q2, q3;
        double A, B;
        const float Axy =
            static_cast<float>(options.Axy_aprox_derivatives);  // For approximating the
        // derivatives

        // Compute at once the square errors for each point with the
        // current "q" and the transformed points:
        std::vector<float> sq_errors;
        correspondences.squareErrorVector(q, sq_errors, other_xs_trans, other_ys_trans);
        double OSE_initial = math::sum(sq_errors);

        // Compute "dJ_dq"
        // ------------------------------------
        const float rho2 = static_cast<float>(square(options.kernel_rho));
        mrpt::tfest::TMatchingPairList::iterator it;
        std::vector<float>::const_iterator other_x_trans, other_y_trans;
        size_t i;

        for (i = 0, it = correspondences.begin(), other_x_trans = other_xs_trans.begin(),
            other_y_trans = other_ys_trans.begin();
             i < nCorrespondences; ++i, ++it, ++other_x_trans, ++other_y_trans)
        {
          // Jacobian: dJ_dx
          // --------------------------------------
          //#define ICP_DISTANCES_TO_LINE

#ifndef ICP_DISTANCES_TO_LINE
          w1 = *other_x_trans - Axy;
          q1 = m1->squareDistanceToClosestCorrespondence(static_cast<float>(w1), *other_y_trans);
          q1 = kernel(static_cast<float>(q1), rho2);

          w2 = *other_x_trans;
          q2 = m1->squareDistanceToClosestCorrespondence(static_cast<float>(w2), *other_y_trans);
          q2 = kernel(static_cast<float>(q2), rho2);

          w3 = *other_x_trans + Axy;
          q3 = m1->squareDistanceToClosestCorrespondence(static_cast<float>(w3), *other_y_trans);
          q3 = kernel(static_cast<float>(q3), rho2);
#else
          // The distance to the line that interpolates the TWO
          // closest points:
          float x1, y1, x2, y2, d1, d2;
          m1->kdTreeTwoClosestPoint2D(
              *other_x_trans,
              *other_y_trans,  // The query
              x1, y1,          // Closest point #1
              x2, y2,          // Closest point #2
              d1, d2);

          w1 = *other_x_trans - Axy;
          q1 = math::squaredDistancePointToLine(w1, *other_y_trans, x1, y1, x2, y2);
          q1 = kernel(q1, rho2);

          w2 = *other_x_trans;
          q2 = math::squaredDistancePointToLine(w2, *other_y_trans, x1, y1, x2, y2);
          q2 = kernel(q2, rho2);

          w3 = *other_x_trans + Axy;
          q3 = math::squaredDistancePointToLine(w3, *other_y_trans, x1, y1, x2, y2);
          q3 = kernel(q3, rho2);
#endif
          // interpolate
          A = ((q3 - q2) / ((w3 - w2) * (w3 - w1))) - ((q1 - q2) / ((w1 - w2) * (w3 - w1)));
          B = ((q1 - q2) + (A * ((w2 * w2) - (w1 * w1)))) / (w1 - w2);

          dJ_dq(0, i) = static_cast<float>((2 * A * *other_x_trans) + B);

          // Jacobian: dJ_dy
          // --------------------------------------
          w1 = *other_y_trans - Axy;
#ifdef ICP_DISTANCES_TO_LINE
          q1 = math::squaredDistancePointToLine(*other_x_trans, w1, x1, y1, x2, y2);
          q1 = kernel(q1, rho2);
#else
          q1 = kernel(square(it->global.x - *other_x_trans) + square(it->global.y - w1), rho2);
#endif

          w2 = *other_y_trans;
          // q2 is alreay computed from above!
          // q2 = m1->squareDistanceToClosestCorrespondence(
          // *other_x_trans, w2 );
          // q2= kernel( square(it->global.x - *other_x_trans)+
          // square( it->global.y - w2 ),  rho2 );

          w3 = *other_y_trans + Axy;
#ifdef ICP_DISTANCES_TO_LINE
          q3 = math::squaredDistancePointToLine(*other_x_trans, w3, x1, y1, x2, y2);
          q3 = kernel(q3, rho2);
#else
          q3 = kernel(square(it->global.x - *other_x_trans) + square(it->global.y - w3), rho2);
#endif

          // interpolate
          A = ((q3 - q2) / ((w3 - w2) * (w3 - w1))) - ((q1 - q2) / ((w1 - w2) * (w3 - w1)));
          B = ((q1 - q2) + (A * ((w2 * w2) - (w1 * w1)))) / (w1 - w2);

          dJ_dq(1, i) = static_cast<float>((2 * A * *other_y_trans) + B);

          // Jacobian: dR_dphi
          // --------------------------------------
          dJ_dq(2, i) = static_cast<float>(
              dJ_dq(0, i) * (-csin * it->local.x - ccos * it->local.y) +
              dJ_dq(1, i) * (ccos * it->local.x - csin * it->local.y));

        }  // end for each corresp.

        // Now we have the Jacobian in dJ_dq.

        // Compute the Hessian matrix H = dJ_dq * dJ_dq^T
        CMatrixFloat H_(3, 3);
        H_.matProductOf_AAt(dJ_dq);

        CMatrixFixed<float, 3, 3> H;
        H = H_;

        bool keepIteratingLM = true;

        // ---------------------------------------------------
        // Iterate the inner LM loop until convergence:
        // ---------------------------------------------------
        q_new = q;

        std::vector<float> new_sq_errors, new_other_xs_trans, new_other_ys_trans;
        size_t nLMiters = 0;
        const size_t maxLMiters = 100;

        while (keepIteratingLM && ++nLMiters < maxLMiters)
        {
          // The LM heuristic is:
          //  x_{k+1} = x_k  - ( H + \lambda diag(H) )^-1 * grad(J)
          //  grad(J) = dJ_dq * e (vector of errors)
          C = H;
          for (i = 0; i < 3; i++)
            C(i, i) *= static_cast<float>(1 + lambda);  // Levenberg-Maquardt heuristic

          C_inv = C.inverse_LLt();

          // LM_delta = C_inv * dJ_dq * sq_errors
          const Eigen::Vector3f LM_delta =
              (C_inv.asEigen() * dJ_dq.asEigen() *
               Eigen::Map<Eigen::VectorXf>(&sq_errors[0], sq_errors.size()))
                  .eval();

          q_new.x(q.x() - LM_delta[0]);
          q_new.y(q.y() - LM_delta[1]);
          q_new.phi(q.phi() - LM_delta[2]);

          // Compute the new square errors:
          // ---------------------------------------
          correspondences.squareErrorVector(
              q_new, new_sq_errors, new_other_xs_trans, new_other_ys_trans);

          float OSE_new = math::sum(new_sq_errors);

          bool improved = OSE_new < OSE_initial;

          keepIteratingLM = fabs(LM_delta[0]) > options.minAbsStep_trans ||
                            fabs(LM_delta[1]) > options.minAbsStep_trans ||
                            fabs(LM_delta[2]) > options.minAbsStep_rot;

          if (improved)
          {
            // If resids have gone down, keep change and make lambda
            // smaller by factor of 10
            lambda /= 10;
            q = q_new;
            OSE_initial = OSE_new;
          }
          else
          {
            // Discard movement and try with larger lambda:
            lambda *= 10;
          }

        }  // end iterative LM

#if 0  // Debuggin'
				cout << "ICP loop: " << lastMeanPose  << " -> " << q << " LM iters: " << nLMiters  << " threshold: " << matchParams.maxDistForCorrespondence << "\n";
#endif
        // --------------------------------------------------------
        // now the conditions for the outer ICP loop
        // --------------------------------------------------------
        keepIteratingICP = true;
        if (fabs(lastMeanPose.x() - q.x()) < options.minAbsStep_trans &&
            fabs(lastMeanPose.y() - q.y()) < options.minAbsStep_trans &&
            fabs(math::wrapToPi(lastMeanPose.phi() - q.phi())) < options.minAbsStep_rot)
        {
          matchParams.maxDistForCorrespondence *= static_cast<float>(options.ALFA);
          matchParams.maxAngularDistForCorrespondence *= static_cast<float>(options.ALFA);
          if (matchParams.maxDistForCorrespondence <
              static_cast<float>(options.smallestThresholdDist))
            keepIteratingICP = false;

          if (++matchParams.offset_other_map_points >= options.corresponding_points_decimation)
            matchParams.offset_other_map_points = 0;
        }
        lastMeanPose = q;
      }  // end of "else, there are correspondences"

      // Next iteration:
      outInfo.nIterations++;

      if (outInfo.nIterations >= options.maxIterations &&
          matchParams.maxDistForCorrespondence > static_cast<float>(options.smallestThresholdDist))
      {
        matchParams.maxDistForCorrespondence *= static_cast<float>(options.ALFA);
      }

    } while (
        (keepIteratingICP && outInfo.nIterations < options.maxIterations) ||
        (outInfo.nIterations >= options.maxIterations &&
         matchParams.maxDistForCorrespondence > static_cast<float>(options.smallestThresholdDist)));

    outInfo.goodness = matchExtraResults.correspondencesRatio;

    // if (!options.skip_quality_calculation)
    {
      outInfo.quality = matchExtraResults.correspondencesRatio;
    }

  }  // end of "if m2 is not empty"

  if (!options.skip_cov_calculation && nCorrespondences)
  {
    return CPosePDFGaussian::Create(q, icpCovarianceSE2(correspondences, q.asTPose(), options));
  }
  return CPosePDFGaussian::Create(q, C_inv.cast_double());
  MRPT_END
}

CPose3DPDF::Ptr CICP::Align3DPDF(
    const mrpt::maps::CMetricMap* m1,
    const mrpt::maps::CMetricMap* mm2,
    const CPose3DPDFGaussian& initialEstimationPDF,
    mrpt::optional_ref<TMetricMapAlignmentResult> outInfo)
{
  MRPT_START

  static CTicTac tictac;
  TReturnInfo outInfoVal;
  CPose3DPDF::Ptr resultPDF;

  if (outInfo) tictac.Tic();

  switch (options.ICP_algorithm)
  {
    case icpClassic:
      resultPDF = ICP3D_Method_Classic(m1, mm2, initialEstimationPDF, outInfoVal);
      break;
    case icpLevenbergMarquardt:
      THROW_EXCEPTION("Only icpClassic is implemented for ICP-3D");
      break;
    default:
      THROW_EXCEPTION_FMT(
          "Invalid value for ICP_algorithm: %i", static_cast<int>(options.ICP_algorithm));
  }  // end switch

  if (outInfo) outInfoVal.executionTime = tictac.Tac();

  // Copy the output info if requested:
  if (outInfo)
    if (auto* o = dynamic_cast<TReturnInfo*>(&outInfo.value().get()); o) *o = outInfoVal;

  return resultPDF;

  MRPT_END
}

CPose3DPDF::Ptr CICP::ICP3D_Method_Classic(
    const mrpt::maps::CMetricMap* m1,
    const mrpt::maps::CMetricMap* mm2,
    const CPose3DPDFGaussian& initialEstimationPDF,
    TReturnInfo& outInfo)
{
  MRPT_START

  // The result can be either a Gaussian or a SOG:
  CPose3DPDF::Ptr resultPDF;
  CPose3DPDFGaussian::Ptr gaussPdf;

  size_t nCorrespondences = 0;
  bool keepApproaching;
  CPose3D grossEst = initialEstimationPDF.mean;
  mrpt::tfest::TMatchingPairList correspondences, old_correspondences;
  CPose3D lastMeanPose;

  // Assure the class of the maps:
  ASSERT_(mm2->GetRuntimeClass()->derivedFrom(CLASS_ID(CPointsMap)));
  const CPointsMap* m2 = static_cast<const CPointsMap*>(mm2);

  // Asserts:
  // -----------------
  ASSERT_(options.ALFA > 0 && options.ALFA < 1);

  // The algorithm output auxiliar info:
  // -------------------------------------------------
  outInfo.nIterations = 0;
  outInfo.goodness = 1;
  outInfo.quality = 0;

  // The gaussian PDF to estimate:
  // ------------------------------------------------------
  gaussPdf = std::make_shared<CPose3DPDFGaussian>();

  // First gross approximation:
  gaussPdf->mean = grossEst;

  // Initial thresholds:
  TMatchingParams matchParams;
  TMatchingExtraResults matchExtraResults;

  matchParams.maxDistForCorrespondence =
      static_cast<float>(options.thresholdDist);  // Distance threshold
  matchParams.maxAngularDistForCorrespondence =
      static_cast<float>(options.thresholdAng);  // Angular threshold
  matchParams.onlyKeepTheClosest = true;
  matchParams.onlyUniqueRobust = options.onlyUniqueRobust;
  matchParams.decimation_other_map_points = options.corresponding_points_decimation;

  // Ensure maps are not empty!
  // ------------------------------------------------------
  if (!m2->isEmpty())
  {
    matchParams.offset_other_map_points = 0;

    // ------------------------------------------------------
    //					The ICP loop
    // ------------------------------------------------------
    do
    {
      // Pivot point for angular measurements:
      matchParams.angularDistPivotPoint = gaussPdf->mean.translation();

      // ------------------------------------------------------
      //		Find the matching (for a points map)
      // ------------------------------------------------------
      m1->determineMatching3D(
          m2,              // The other map
          gaussPdf->mean,  // The other map pose
          correspondences, matchParams, matchExtraResults);

      nCorrespondences = correspondences.size();

      if (!nCorrespondences)
      {
        // Nothing we can do !!
        keepApproaching = false;
      }
      else
      {
        // Compute the estimated pose, using Horn's method.
        // ----------------------------------------------------------------------
        mrpt::poses::CPose3DQuat estPoseQuat;
        double transf_scale;
        mrpt::tfest::se3_l2(
            correspondences, estPoseQuat, transf_scale, true /* rigid: force unit scale */);
        gaussPdf->mean = mrpt::poses::CPose3D(estPoseQuat);

        // If matching has not changed, decrease the thresholds:
        // --------------------------------------------------------
        keepApproaching = true;
        if (!(fabs(lastMeanPose.x() - gaussPdf->mean.x()) > options.minAbsStep_trans ||
              fabs(lastMeanPose.y() - gaussPdf->mean.y()) > options.minAbsStep_trans ||
              fabs(lastMeanPose.z() - gaussPdf->mean.z()) > options.minAbsStep_trans ||
              fabs(math::wrapToPi(lastMeanPose.yaw() - gaussPdf->mean.yaw())) >
                  options.minAbsStep_rot ||
              fabs(math::wrapToPi(lastMeanPose.pitch() - gaussPdf->mean.pitch())) >
                  options.minAbsStep_rot ||
              fabs(math::wrapToPi(lastMeanPose.roll() - gaussPdf->mean.roll())) >
                  options.minAbsStep_rot))
        {
          matchParams.maxDistForCorrespondence *= static_cast<float>(options.ALFA);
          matchParams.maxAngularDistForCorrespondence *= static_cast<float>(options.ALFA);
          if (matchParams.maxDistForCorrespondence <
              static_cast<float>(options.smallestThresholdDist))
            keepApproaching = false;

          if (++matchParams.offset_other_map_points >= options.corresponding_points_decimation)
            matchParams.offset_other_map_points = 0;
        }

        lastMeanPose = gaussPdf->mean;

      }  // end of "else, there are correspondences"

      // Next iteration:
      outInfo.nIterations++;

      if (outInfo.nIterations >= options.maxIterations &&
          matchParams.maxDistForCorrespondence > static_cast<float>(options.smallestThresholdDist))
      {
        matchParams.maxDistForCorrespondence *= static_cast<float>(options.ALFA);
      }

    } while (
        (keepApproaching && outInfo.nIterations < options.maxIterations) ||
        (outInfo.nIterations >= options.maxIterations &&
         matchParams.maxDistForCorrespondence > static_cast<float>(options.smallestThresholdDist)));

    // -------------------------------------------------
    //   Obtain the covariance matrix of the estimation
    // -------------------------------------------------
    if (!options.skip_cov_calculation && nCorrespondences)
    {
      gaussPdf->cov = icpCovarianceSE3(correspondences, gaussPdf->mean, options);
    }

    //
    outInfo.goodness = matchExtraResults.correspondencesRatio;

  }  // end of "if m2 is not empty"

  // Return the gaussian distribution:
  resultPDF = gaussPdf;

  return resultPDF;

  MRPT_END
}
