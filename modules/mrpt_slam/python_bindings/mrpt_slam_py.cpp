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

#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/slam/CICP.h>
#include <mrpt/slam/CMetricMapBuilder.h>
#include <mrpt/slam/CMetricMapBuilderICP.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_MODULE(_bindings, m)
{
  m.doc() = "Python bindings for mrpt::slam — SLAM and localization algorithms";

  // -------------------------------------------------------------------------
  // TICPAlgorithm enum
  // -------------------------------------------------------------------------
  py::enum_<mrpt::slam::TICPAlgorithm>(m, "TICPAlgorithm")
      .value("icpClassic", mrpt::slam::icpClassic)
      .value("icpLevenbergMarquardt", mrpt::slam::icpLevenbergMarquardt)
      .export_values();

  // -------------------------------------------------------------------------
  // TICPCovarianceMethod enum
  // -------------------------------------------------------------------------
  py::enum_<mrpt::slam::TICPCovarianceMethod>(m, "TICPCovarianceMethod")
      .value("icpCovLinealMSE", mrpt::slam::icpCovLinealMSE)
      .value("icpCovFiniteDifferences", mrpt::slam::icpCovFiniteDifferences)
      .export_values();

  // -------------------------------------------------------------------------
  // CICP::TReturnInfo — ICP result information struct
  // -------------------------------------------------------------------------
  py::class_<mrpt::slam::CICP::TReturnInfo>(m, "TICPReturnInfo")
      .def(py::init<>())
      .def_readwrite("nIterations", &mrpt::slam::CICP::TReturnInfo::nIterations)
      .def_readwrite("goodness", &mrpt::slam::CICP::TReturnInfo::goodness)
      .def_readwrite("quality", &mrpt::slam::CICP::TReturnInfo::quality)
      .def(
          "__repr__",
          [](const mrpt::slam::CICP::TReturnInfo& r)
          {
            return "TICPReturnInfo(nIter=" + std::to_string(r.nIterations) +
                   ", goodness=" + std::to_string(r.goodness) +
                   ", quality=" + std::to_string(r.quality) + ")";
          });

  // -------------------------------------------------------------------------
  // CICP::TConfigParams — ICP algorithm parameters
  // -------------------------------------------------------------------------
  py::class_<mrpt::slam::CICP::TConfigParams>(m, "CICPOptions")
      .def(py::init<>())
      .def_readwrite("ICP_algorithm", &mrpt::slam::CICP::TConfigParams::ICP_algorithm)
      .def_readwrite(
          "ICP_covariance_method", &mrpt::slam::CICP::TConfigParams::ICP_covariance_method)
      .def_readwrite("maxIterations", &mrpt::slam::CICP::TConfigParams::maxIterations)
      .def_readwrite("thresholdDist", &mrpt::slam::CICP::TConfigParams::thresholdDist)
      .def_readwrite("thresholdAng", &mrpt::slam::CICP::TConfigParams::thresholdAng)
      .def_readwrite("ALFA", &mrpt::slam::CICP::TConfigParams::ALFA)
      .def_readwrite(
          "smallestThresholdDist", &mrpt::slam::CICP::TConfigParams::smallestThresholdDist)
      .def_readwrite("doRANSAC", &mrpt::slam::CICP::TConfigParams::doRANSAC)
      .def_readwrite("skip_cov_calculation", &mrpt::slam::CICP::TConfigParams::skip_cov_calculation)
      .def_readwrite(
          "skip_quality_calculation", &mrpt::slam::CICP::TConfigParams::skip_quality_calculation)
      .def_readwrite(
          "corresponding_points_decimation",
          &mrpt::slam::CICP::TConfigParams::corresponding_points_decimation);

  // -------------------------------------------------------------------------
  // CICP — ICP alignment algorithm
  // -------------------------------------------------------------------------
  py::class_<mrpt::slam::CICP>(m, "CICP")
      .def(py::init<>())
      .def(py::init<const mrpt::slam::CICP::TConfigParams&>(), "options"_a)
      .def_readwrite("options", &mrpt::slam::CICP::options)
      .def(
          "AlignPDF",
          [](mrpt::slam::CICP& icp, const mrpt::maps::CMetricMap* m1,
             const mrpt::maps::CMetricMap* m2, const mrpt::poses::CPosePDFGaussian& initEst)
          {
            mrpt::slam::CICP::TReturnInfo info;
            auto pdf = icp.AlignPDF(m1, m2, initEst, info);
            return py::make_tuple(pdf, info);
          },
          "m1"_a, "m2"_a, "initialEstimationPDF"_a,
          "Align two maps. Returns (CPosePDF, TICPReturnInfo) tuple.")
      .def("__repr__", [](const mrpt::slam::CICP&) { return "CICP()"; });

  // -------------------------------------------------------------------------
  // CMetricMapBuilder — abstract base for SLAM map builders
  // -------------------------------------------------------------------------
  py::class_<mrpt::slam::CMetricMapBuilder>(m, "CMetricMapBuilder")
      .def(
          "initialize",
          [](mrpt::slam::CMetricMapBuilder& b, const mrpt::maps::CSimpleMap& map)
          { b.initialize(map); },
          "initialMap"_a = mrpt::maps::CSimpleMap(),
          "Initialize the builder with an optional initial map.")
      .def("getCurrentPoseEstimation", &mrpt::slam::CMetricMapBuilder::getCurrentPoseEstimation)
      .def(
          "processActionObservation", &mrpt::slam::CMetricMapBuilder::processActionObservation,
          "action"_a, "sf"_a)
      .def("getCurrentlyBuiltMapSize", &mrpt::slam::CMetricMapBuilder::getCurrentlyBuiltMapSize)
      .def(
          "saveCurrentMapToFile", &mrpt::slam::CMetricMapBuilder::saveCurrentMapToFile,
          "fileName"_a, "compressGZ"_a = true);

  // -------------------------------------------------------------------------
  // CMetricMapBuilderICP — simple ICP-based SLAM builder
  // -------------------------------------------------------------------------
  py::class_<mrpt::slam::CMetricMapBuilderICP, mrpt::slam::CMetricMapBuilder>(
      m, "CMetricMapBuilderICP")
      .def(py::init<>())
      .def_readwrite("ICP_options", &mrpt::slam::CMetricMapBuilderICP::ICP_options)
      .def_readwrite("ICP_params", &mrpt::slam::CMetricMapBuilderICP::ICP_params)
      .def(
          "initialize",
          [](mrpt::slam::CMetricMapBuilderICP& b, const mrpt::maps::CSimpleMap& map)
          { b.initialize(map); },
          "initialMap"_a = mrpt::maps::CSimpleMap())
      .def(
          "processObservation", &mrpt::slam::CMetricMapBuilderICP::processObservation, "obs"_a,
          "Process a single observation (new-style API).")
      .def(
          "processActionObservation", &mrpt::slam::CMetricMapBuilderICP::processActionObservation,
          "action"_a, "sf"_a, "Process action+sensoryframe pair (classic API).")
      .def("getCurrentPoseEstimation", &mrpt::slam::CMetricMapBuilderICP::getCurrentPoseEstimation)
      .def("getCurrentlyBuiltMapSize", &mrpt::slam::CMetricMapBuilderICP::getCurrentlyBuiltMapSize)
      .def(
          "getCurrentMapPoints",
          [](mrpt::slam::CMetricMapBuilderICP& b)
          {
            std::vector<float> x, y;
            b.getCurrentMapPoints(x, y);
            return py::make_tuple(x, y);
          },
          "Returns (xs, ys) float lists of current point-map coordinates.")
      .def(
          "saveCurrentMapToFile",
          [](mrpt::slam::CMetricMapBuilderICP& b, const std::string& fn, bool gz)
          { b.saveCurrentMapToFile(fn, gz); },
          "fileName"_a, "compressGZ"_a = true)
      .def(
          "__repr__",
          [](const mrpt::slam::CMetricMapBuilderICP& b)
          {
            return "CMetricMapBuilderICP(mapSize=" +
                   std::to_string(const_cast<mrpt::slam::CMetricMapBuilderICP&>(b)
                                      .getCurrentlyBuiltMapSize()) +
                   ")";
          });

  // -------------------------------------------------------------------------
  // CMetricMapBuilderICP::TConfigParams — ICP-SLAM builder options
  // -------------------------------------------------------------------------
  py::class_<mrpt::slam::CMetricMapBuilderICP::TConfigParams>(m, "CMetricMapBuilderICPOptions")
      .def_readwrite(
          "matchAgainstTheGrid",
          &mrpt::slam::CMetricMapBuilderICP::TConfigParams::matchAgainstTheGrid)
      .def_readwrite(
          "insertionLinDistance",
          &mrpt::slam::CMetricMapBuilderICP::TConfigParams::insertionLinDistance)
      .def_readwrite(
          "insertionAngDistance",
          &mrpt::slam::CMetricMapBuilderICP::TConfigParams::insertionAngDistance)
      .def_readwrite(
          "localizationLinDistance",
          &mrpt::slam::CMetricMapBuilderICP::TConfigParams::localizationLinDistance)
      .def_readwrite(
          "localizationAngDistance",
          &mrpt::slam::CMetricMapBuilderICP::TConfigParams::localizationAngDistance)
      .def_readwrite(
          "minICPgoodnessToAccept",
          &mrpt::slam::CMetricMapBuilderICP::TConfigParams::minICPgoodnessToAccept);
}
