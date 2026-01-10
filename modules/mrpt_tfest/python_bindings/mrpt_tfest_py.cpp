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
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// mrpt
#include <mrpt/math/TPose2D.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/tfest/se2.h>
#include <mrpt/tfest/se3.h>

namespace py = pybind11;
using namespace mrpt::tfest;

PYBIND11_MODULE(_bindings, m)
{
  m.doc() = "Python bindings for mrpt_tfest (Transformation Estimation)";

  // -------------------------------------------------------------------------
  // TMatchingPair
  // -------------------------------------------------------------------------
  // Instantiate the double version as the default
  py::class_<TMatchingPair>(m, "TMatchingPair")
      .def(py::init<>())
      .def_readwrite("globalIdx", &TMatchingPair::globalIdx)
      .def_readwrite("localIdx", &TMatchingPair::localIdx)
      .def_readwrite("local_pt", &TMatchingPair::local)
      .def_readwrite("global_pt", &TMatchingPair::global)
      .def_readwrite(
          "errorSquareAfterTransformation", &TMatchingPair::errorSquareAfterTransformation);

  py::class_<TMatchingPair_d>(m, "TMatchingPair_d")
      .def(py::init<>())
      .def_readwrite("globalIdx", &TMatchingPair_d::globalIdx)
      .def_readwrite("localIdx", &TMatchingPair_d::localIdx)
      .def_readwrite("local_pt", &TMatchingPair_d::local)
      .def_readwrite("global_pt", &TMatchingPair_d::global)
      .def_readwrite(
          "errorSquareAfterTransformation", &TMatchingPair_d::errorSquareAfterTransformation);

  // TMatchingPairList (std::vector wrapper)
  py::class_<mrpt::tfest::TMatchingPairList>(m, "TMatchingPairList")
      .def(py::init<>())
      .def("size", &TMatchingPairList::size)
      .def(
          "push_back",
          [](TMatchingPairList& list, const TMatchingPair& element) { list.push_back(element); })
      .def("__len__", &TMatchingPairList::size)
      .def(
          "__iter__", [](TMatchingPairList& v) { return py::make_iterator(v.begin(), v.end()); },
          py::keep_alive<0, 1>());

  py::class_<mrpt::tfest::TMatchingPairList_d>(m, "TMatchingPairList_d")
      .def(py::init<>())
      .def(
          "push_back", [](TMatchingPairList_d& list, const TMatchingPair_d& element)
          { list.push_back(element); })
      .def("size", &TMatchingPairList_d::size)
      .def("__len__", &TMatchingPairList_d::size)
      .def(
          "__iter__", [](TMatchingPairList_d& v) { return py::make_iterator(v.begin(), v.end()); },
          py::keep_alive<0, 1>());

  // -------------------------------------------------------------------------
  // SE(2) Estimation
  // -------------------------------------------------------------------------
  m.def(
      "se2_l2",
      [](const TMatchingPairList& correspondences)
      {
        mrpt::math::TPose2D out_estimate;
        bool ok = mrpt::tfest::se2_l2(correspondences, out_estimate);
        return py::make_tuple(ok, out_estimate);
      },
      "correspondences", "Least-squares SE(2) estimation. Returns (ok, mrpt::math::TPose2D)");

  // -------------------------------------------------------------------------
  // SE(3) Estimation
  // -------------------------------------------------------------------------
  py::class_<TSE3RobustParams>(m, "TSE3RobustParams")
      .def(py::init<>())
      .def_readwrite("ransac_minSetSize", &TSE3RobustParams::ransac_minSetSize)
      .def_readwrite("ransac_nmaxSimulations", &TSE3RobustParams::ransac_nmaxSimulations)
      .def_readwrite("ransac_maxSetSizePct", &TSE3RobustParams::ransac_maxSetSizePct)
      .def_readwrite(
          "user_individual_compat_callback", &TSE3RobustParams::user_individual_compat_callback)
      .def_readwrite("ransac_threshold_lin", &TSE3RobustParams::ransac_threshold_lin);

  py::class_<TSE3RobustResult>(m, "TSE3RobustResult")
      .def(py::init<>())
      .def_readwrite("transformation", &TSE3RobustResult::transformation)
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