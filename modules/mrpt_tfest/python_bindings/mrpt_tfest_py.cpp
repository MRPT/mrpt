/* _
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

// pybind11
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// mrpt
#include <mrpt/math/TPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPosePDFSOG.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/tfest/se2.h>
#include <mrpt/tfest/se3.h>

#include <sstream>

namespace py = pybind11;
using namespace pybind11::literals;
using namespace mrpt::tfest;

namespace
{
template <typename T>
std::string pairToString(const TMatchingPairTempl<T>& p)
{
  std::ostringstream ss;
  p.print(ss);
  return ss.str();
}
}  // namespace

PYBIND11_MODULE(_bindings, m)
{
  m.doc() = "Python bindings for mrpt_tfest (Transformation Estimation)";

  // -------------------------------------------------------------------------
  // TMatchingPair
  // -------------------------------------------------------------------------
  // Instantiate the double version as the default
  py::class_<TMatchingPair>(
      m, "TMatchingPair", "A pair of corresponding points (float coordinates) in two point sets.")
      .def(py::init<>(), "Default constructor.")
      .def(
          py::init<
              uint32_t, uint32_t, const mrpt::math::TPoint3Df&, const mrpt::math::TPoint3Df&>(),
          "globalIdx"_a, "localIdx"_a, "global_pt"_a, "local_pt"_a,
          "Builds the pair from the indices and coordinates of the global and local points.")
      .def(
          py::init<uint32_t, uint32_t, float, float, float, float, float, float>(), "globalIdx"_a,
          "localIdx"_a, "global_x"_a, "global_y"_a, "global_z"_a, "local_x"_a, "local_y"_a,
          "local_z"_a,
          "Builds the pair from the indices and coordinates of the global and local points.")
      .def_readwrite("globalIdx", &TMatchingPair::globalIdx)
      .def_readwrite("localIdx", &TMatchingPair::localIdx)
      .def_readwrite("local_pt", &TMatchingPair::local)
      .def_readwrite("global_pt", &TMatchingPair::global)
      .def_readwrite(
          "errorSquareAfterTransformation", &TMatchingPair::errorSquareAfterTransformation)
      .def("__repr__", &pairToString<float>);

  py::class_<TMatchingPair_d>(
      m, "TMatchingPair_d",
      "A pair of corresponding points (double coordinates) in two point sets.")
      .def(py::init<>(), "Default constructor.")
      .def(
          py::init<uint32_t, uint32_t, const mrpt::math::TPoint3D&, const mrpt::math::TPoint3D&>(),
          "globalIdx"_a, "localIdx"_a, "global_pt"_a, "local_pt"_a,
          "Builds the pair from the indices and coordinates of the global and local points.")
      .def(
          py::init<uint32_t, uint32_t, double, double, double, double, double, double>(),
          "globalIdx"_a, "localIdx"_a, "global_x"_a, "global_y"_a, "global_z"_a, "local_x"_a,
          "local_y"_a, "local_z"_a,
          "Builds the pair from the indices and coordinates of the global and local points.")
      .def_readwrite("globalIdx", &TMatchingPair_d::globalIdx)
      .def_readwrite("localIdx", &TMatchingPair_d::localIdx)
      .def_readwrite("local_pt", &TMatchingPair_d::local)
      .def_readwrite("global_pt", &TMatchingPair_d::global)
      .def_readwrite(
          "errorSquareAfterTransformation", &TMatchingPair_d::errorSquareAfterTransformation)
      .def("__repr__", &pairToString<double>);

  // TMatchingPairList (std::vector wrapper)
  py::class_<mrpt::tfest::TMatchingPairList>(m, "TMatchingPairList", "A list of TMatchingPair.")
      .def(py::init<>(), "Default constructor.")
      .def(
          "size", [](const TMatchingPairList& l) { return l.size(); },
          "Returns the number of pairs.")
      .def(
          "push_back",
          [](TMatchingPairList& list, const TMatchingPair& element) { list.push_back(element); },
          "Appends a pair.")
      .def("__len__", [](const TMatchingPairList& l) { return l.size(); })
      .def(
          "__iter__", [](TMatchingPairList& v) { return py::make_iterator(v.begin(), v.end()); },
          py::keep_alive<0, 1>());

  py::class_<mrpt::tfest::TMatchingPairList_d>(
      m, "TMatchingPairList_d", "A list of TMatchingPair_d.")
      .def(py::init<>(), "Default constructor.")
      .def(
          "push_back",
          [](TMatchingPairList_d& list, const TMatchingPair_d& element)
          { list.push_back(element); },
          "Appends a pair.")
      .def(
          "size", [](const TMatchingPairList_d& l) { return l.size(); },
          "Returns the number of pairs.")
      .def("__len__", [](const TMatchingPairList_d& l) { return l.size(); })
      .def(
          "__iter__", [](TMatchingPairList_d& v) { return py::make_iterator(v.begin(), v.end()); },
          py::keep_alive<0, 1>());

  // -------------------------------------------------------------------------
  // SE(2) Estimation
  // -------------------------------------------------------------------------
  py::class_<TPotentialMatch>(
      m, "TPotentialMatch",
      "A potential pairing of two points, passed to user_individual_compat_callback.")
      .def(py::init<>(), "Default constructor.")
      .def_readwrite("idx_this", &TPotentialMatch::idx_this, "Index of the point in this set")
      .def_readwrite(
          "idx_other", &TPotentialMatch::idx_other, "Index of the point in the other set");

  m.def(
      "se2_l2",
      [](const TMatchingPairList& correspondences)
      {
        mrpt::math::TPose2D out_estimate;
        bool ok = mrpt::tfest::se2_l2(correspondences, out_estimate);
        return py::make_tuple(ok, out_estimate);
      },
      "correspondences", "Least-squares SE(2) estimation. Returns (ok, mrpt::math::TPose2D)");

  m.def(
      "se2_l2_with_covariance",
      [](const TMatchingPairList& correspondences)
      {
        mrpt::math::TPose2D out_estimate;
        mrpt::math::CMatrixDouble33 cov;
        const bool ok = mrpt::tfest::se2_l2(correspondences, out_estimate, &cov);
        return py::make_tuple(ok, out_estimate, cov);
      },
      "correspondences"_a,
      "Least-squares SE(2) estimation, also returning its normalized 3x3 covariance, which must "
      "be multiplied by the variance of the point coordinates. Returns (ok, mrpt.math.TPose2D, "
      "mrpt.math.CMatrixDouble33). ok is false for less than two pairs, or if the rotation is "
      "undefined.");

  py::class_<TSE2RobustParams>(m, "TSE2RobustParams", "Parameters of se2_l2_robust().")
      .def(py::init<>(), "Default constructor.")
      .def_readwrite(
          "ransac_minSetSize", &TSE2RobustParams::ransac_minSetSize,
          "Minimum size of a consensus set (default: 3).")
      .def_readwrite(
          "ransac_maxSetSize", &TSE2RobustParams::ransac_maxSetSize,
          "Number of pairs tested for consensus in each iteration (default: 20). Set it to the "
          "number of correspondences to test all of them.")
      .def_readwrite(
          "ransac_mahalanobisDistanceThreshold",
          &TSE2RobustParams::ransac_mahalanobisDistanceThreshold,
          "Threshold to accept a pair into a consensus set (default: 3.0).")
      .def_readwrite(
          "ransac_nSimulations", &TSE2RobustParams::ransac_nSimulations,
          "Number of iterations, or 0 (default) for an adaptive number, from "
          "probability_find_good_model.")
      .def_readwrite(
          "ransac_fuseByCorrsMatch", &TSE2RobustParams::ransac_fuseByCorrsMatch,
          "If true (default), modes are fused when their subsets of pairs match; otherwise, when "
          "their means are closer than ransac_fuseMaxDiffXY and ransac_fuseMaxDiffPhi.")
      .def_readwrite(
          "ransac_fuseMaxDiffXY", &TSE2RobustParams::ransac_fuseMaxDiffXY,
          "Maximum difference in x and y to fuse two modes (default: 0.01).")
      .def_readwrite(
          "ransac_fuseMaxDiffPhi", &TSE2RobustParams::ransac_fuseMaxDiffPhi,
          "Maximum difference in phi to fuse two modes, in radians (default: 0.1 deg).")
      .def_readwrite(
          "ransac_algorithmForLandmarks", &TSE2RobustParams::ransac_algorithmForLandmarks,
          "Use the Mahalanobis distance (true, default) or the Euclidean distance (false).")
      .def_readwrite(
          "probability_find_good_model", &TSE2RobustParams::probability_find_good_model,
          "Probability of finding a good model, for the adaptive number of iterations (default: "
          "0.999).")
      .def_readwrite(
          "ransac_min_nSimulations", &TSE2RobustParams::ransac_min_nSimulations,
          "Minimum number of iterations with the adaptive method (default: 1500).")
      .def_readwrite(
          "max_rmse_to_end", &TSE2RobustParams::max_rmse_to_end,
          "Stops when a solution has an RMSE below this value. 0 (default) means "
          "2*normalizationStd.")
      .def_readwrite("verbose", &TSE2RobustParams::verbose, "Prints debug information.")
      .def_readwrite(
          "user_individual_compat_callback", &TSE2RobustParams::user_individual_compat_callback,
          "Optional function(TPotentialMatch) -> bool: return True to accept a candidate pairing, "
          "False to reject it (None: accept all)");

  py::class_<TSE2RobustResult>(m, "TSE2RobustResult", "Result of se2_l2_robust().")
      .def(py::init<>(), "Default constructor.")
      .def_readwrite(
          "transformation", &TSE2RobustResult::transformation,
          "The estimated transformations, as a sum of Gaussians (mrpt.poses.CPosePDFSOG).")
      .def_readwrite(
          "largestSubSet", &TSE2RobustResult::largestSubSet,
          "The consensus subset of pairs with the lowest RMSE.")
      .def_readwrite(
          "ransac_iters", &TSE2RobustResult::ransac_iters, "Number of iterations executed.");

  m.def(
      "se2_l2_robust",
      [](const TMatchingPairList& in_correspondences, double in_normalizationStd,
         const TSE2RobustParams& in_ransac_params)
      {
        TSE2RobustResult results;
        const bool ok = mrpt::tfest::se2_l2_robust(
            in_correspondences, in_normalizationStd, in_ransac_params, results);
        return py::make_tuple(ok, results);
      },
      "in_correspondences"_a, "in_normalizationStd"_a, "in_ransac_params"_a,
      "Robust SE(2) estimation using RANSAC, returning a sum of Gaussians. in_normalizationStd is "
      "the standard deviation of the point coordinates. Returns (ok, TSE2RobustResult).");

  // -------------------------------------------------------------------------
  // SE(3) Estimation
  // -------------------------------------------------------------------------
  m.def(
      "se3_l2",
      [](const TMatchingPairList& in_correspondences, bool forceScaleToUnity)
      {
        mrpt::poses::CPose3DQuat out_transform;
        double out_scale = 0;
        const bool ok =
            mrpt::tfest::se3_l2(in_correspondences, out_transform, out_scale, forceScaleToUnity);
        return py::make_tuple(ok, mrpt::poses::CPose3D(out_transform), out_scale);
      },
      "in_correspondences"_a, "forceScaleToUnity"_a = false,
      "Least-squares SE(3) estimation (Horn's method). Returns (ok, mrpt.poses.CPose3D, scale).");

  m.def(
      "se3_l2",
      [](const TMatchingPairList_d& in_correspondences, bool forceScaleToUnity)
      {
        mrpt::poses::CPose3DQuat out_transform;
        double out_scale = 0;
        const bool ok =
            mrpt::tfest::se3_l2(in_correspondences, out_transform, out_scale, forceScaleToUnity);
        return py::make_tuple(ok, mrpt::poses::CPose3D(out_transform), out_scale);
      },
      "in_correspondences"_a, "forceScaleToUnity"_a = false,
      "Least-squares SE(3) estimation (Horn's method), for double precision pairs. Returns (ok, "
      "mrpt.poses.CPose3D, scale).");

  m.def(
      "se3_l2",
      [](const std::vector<mrpt::math::TPoint3D>& in_points_this,
         const std::vector<mrpt::math::TPoint3D>& in_points_other, bool forceScaleToUnity)
      {
        mrpt::poses::CPose3DQuat out_transform;
        double out_scale = 0;
        const bool ok = mrpt::tfest::se3_l2(
            in_points_this, in_points_other, out_transform, out_scale, forceScaleToUnity);
        return py::make_tuple(ok, mrpt::poses::CPose3D(out_transform), out_scale);
      },
      "in_points_this"_a, "in_points_other"_a, "forceScaleToUnity"_a = false,
      "Least-squares SE(3) estimation (Horn's method) from two lists of corresponding points of "
      "the same length. Returns (ok, mrpt.poses.CPose3D, scale).");

  py::class_<TSE3RobustParams>(m, "TSE3RobustParams", "Parameters of se3_l2_robust().")
      .def(py::init<>(), "Default constructor.")
      .def_readwrite("ransac_minSetSize", &TSE3RobustParams::ransac_minSetSize)
      .def_readwrite("ransac_nmaxSimulations", &TSE3RobustParams::ransac_nmaxSimulations)
      .def_readwrite("ransac_maxSetSizePct", &TSE3RobustParams::ransac_maxSetSizePct)
      .def_readwrite(
          "user_individual_compat_callback", &TSE3RobustParams::user_individual_compat_callback,
          "Optional function(TPotentialMatch) -> bool: return True to accept a candidate pairing, "
          "False to reject it (None: accept all)")
      .def_readwrite("ransac_threshold_lin", &TSE3RobustParams::ransac_threshold_lin)
      .def_readwrite(
          "ransac_threshold_ang", &TSE3RobustParams::ransac_threshold_ang,
          "Maximum rotation difference for a solution to match a candidate, in radians (default: "
          "1 deg).")
      .def_readwrite(
          "ransac_threshold_scale", &TSE3RobustParams::ransac_threshold_scale,
          "Maximum scale difference for a solution to match a candidate (default: 0.03).")
      .def_readwrite(
          "forceScaleToUnity", &TSE3RobustParams::forceScaleToUnity,
          "Estimates a rigid transformation, with scale 1 (default: true).")
      .def_readwrite("verbose", &TSE3RobustParams::verbose, "Prints debug information.");

  py::class_<TSE3RobustResult>(m, "TSE3RobustResult", "Result of se3_l2_robust().")
      .def(py::init<>(), "Default constructor.")
      .def_property(
          "transformation",
          [](const TSE3RobustResult& r) -> mrpt::poses::CPose3D
          { return mrpt::poses::CPose3D(r.transformation); },
          [](TSE3RobustResult& r, const mrpt::poses::CPose3D& p)
          { r.transformation = mrpt::poses::CPose3DQuat(p); },
          "Estimated SE(3) transform as CPose3D (converted from internal CPose3DQuat).")
      .def_readwrite("scale", &TSE3RobustResult::scale)
      .def_readwrite("inliers_idx", &TSE3RobustResult::inliers_idx);

  m.def(
      "se3_l2_robust",
      [](const TMatchingPairList& in_correspondences, const TSE3RobustParams& in_params)
      {
        TSE3RobustResult results;
        bool ok = mrpt::tfest::se3_l2_robust(in_correspondences, in_params, results);
        return py::make_tuple(ok, results);
      },
      "in_correspondences", "in_params",
      "Robust SE(3) estimation using RANSAC. Returns (ok, TSE3RobustResult)");
}