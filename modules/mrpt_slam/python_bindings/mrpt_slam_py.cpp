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

#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/bayes/CParticleFilterCapable.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/CMultiMetricMapPDF.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/slam/CICP.h>
#include <mrpt/slam/CMetricMapBuilder.h>
#include <mrpt/slam/CMetricMapBuilderICP.h>
#include <mrpt/slam/CMetricMapBuilderRBPF.h>
#include <mrpt/slam/CMonteCarloLocalization2D.h>
#include <mrpt/slam/CMonteCarloLocalization3D.h>
#include <mrpt/slam/TKLDParams.h>
#include <mrpt/slam/TMonteCarloLocalizationParams.h>
#include <mrpt/viz/CSetOfObjects.h>
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
  py::class_<
      mrpt::slam::CICP::TConfigParams, mrpt::config::CLoadableOptions,
      std::shared_ptr<mrpt::slam::CICP::TConfigParams>>(m, "CICPOptions")
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
          [](mrpt::slam::CMetricMapBuilder& b) { b.initialize(mrpt::maps::CSimpleMap()); },
          "Initialize the builder with an empty map.")
      .def(
          "initialize",
          [](mrpt::slam::CMetricMapBuilder& b, const mrpt::maps::CSimpleMap& map)
          { b.initialize(map); },
          "initialMap"_a, "Initialize the builder with a given initial map.")
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
          "useSimplePointsMap",
          [](mrpt::slam::CMetricMapBuilderICP& b)
          {
            const std::string ini = "[Map]\nCSimplePointsMap_count=1\n";
            mrpt::config::CConfigFileMemory cfg(ini);
            b.ICP_options.mapInitializers.loadFromConfigFile(cfg, "Map");
          },
          "Configure the builder to use a single CSimplePointsMap. Call before initialize().")
      .def(
          "initialize",
          [](mrpt::slam::CMetricMapBuilderICP& b) { b.initialize(mrpt::maps::CSimpleMap()); })
      .def(
          "initialize",
          [](mrpt::slam::CMetricMapBuilderICP& b, const mrpt::maps::CSimpleMap& map)
          { b.initialize(map); },
          "initialMap"_a)
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
  py::class_<
      mrpt::slam::CMetricMapBuilderICP::TConfigParams, mrpt::config::CLoadableOptions,
      std::shared_ptr<mrpt::slam::CMetricMapBuilderICP::TConfigParams>>(
      m, "CMetricMapBuilderICPOptions")
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
          &mrpt::slam::CMetricMapBuilderICP::TConfigParams::minICPgoodnessToAccept)
      .def(
          "loadFromConfigFile",
          [](mrpt::slam::CMetricMapBuilderICP::TConfigParams& self, const std::string& iniContent,
             const std::string& section)
          {
            mrpt::config::CConfigFileMemory cfg(iniContent);
            self.loadFromConfigFile(cfg, section);
          },
          "iniContent"_a, "section"_a,
          "Load options (including mapInitializers) from an INI-format string.")
      .def(
          "loadFromConfigFile",
          [](mrpt::slam::CMetricMapBuilderICP::TConfigParams& self,
             const mrpt::config::CConfigFileBase& cfg, const std::string& section)
          { self.loadFromConfigFile(cfg, section); },
          "source"_a, "section"_a, "Load options (including mapInitializers) from a config file.");

  // -------------------------------------------------------------------------
  // CParticleFilter.executeOn(): attached here to the mrpt.bayes class, since
  // it takes mrpt_obs types that mrpt_bayes does not link against.
  // -------------------------------------------------------------------------
  {
    py::object pfClass = py::module_::import("mrpt.bayes").attr("CParticleFilter");
    pfClass.attr("executeOn") = py::cpp_function(
        [](const mrpt::bayes::CParticleFilter& pf, mrpt::bayes::CParticleFilterCapable& obj,
           const mrpt::obs::CActionCollection* action, const mrpt::obs::CSensoryFrame* observation)
        {
          mrpt::bayes::CParticleFilter::TParticleFilterStats stats;
          pf.executeOn(obj, action, observation, &stats);
          return stats;
        },
        py::name("executeOn"), py::is_method(pfClass), "obj"_a, "action"_a, "observation"_a,
        "Runs one prediction + update (+ resampling) step of the particle filter on obj, e.g. a "
        "CMonteCarloLocalization2D. action or observation may be None. Returns a "
        "TParticleFilterStats.");
  }

  // -------------------------------------------------------------------------
  // Monte Carlo localization (particle filter localization on a known map)
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::slam::TKLDParams, mrpt::config::CLoadableOptions,
      std::shared_ptr<mrpt::slam::TKLDParams>>(m, "TKLDParams")
      .def(py::init<>())
      .def_readwrite("KLD_binSize_XY", &mrpt::slam::TKLDParams::KLD_binSize_XY)
      .def_readwrite("KLD_binSize_PHI", &mrpt::slam::TKLDParams::KLD_binSize_PHI)
      .def_readwrite("KLD_delta", &mrpt::slam::TKLDParams::KLD_delta)
      .def_readwrite("KLD_epsilon", &mrpt::slam::TKLDParams::KLD_epsilon)
      .def_readwrite("KLD_minSampleSize", &mrpt::slam::TKLDParams::KLD_minSampleSize)
      .def_readwrite("KLD_maxSampleSize", &mrpt::slam::TKLDParams::KLD_maxSampleSize)
      .def_readwrite("KLD_minSamplesPerBin", &mrpt::slam::TKLDParams::KLD_minSamplesPerBin);

  using MCLParams = mrpt::slam::TMonteCarloLocalizationParams;
  py::class_<MCLParams>(m, "TMonteCarloLocalizationParams")
      .def(py::init<>())
      .def_property(
          "metricMap",
          [](const MCLParams& p)
          { return std::const_pointer_cast<mrpt::maps::CMetricMap>(p.metricMap); },
          [](MCLParams& p, const mrpt::maps::CMetricMap::Ptr& map) { p.metricMap = map; },
          "The map used to evaluate observation likelihoods (e.g. a CMultiMetricMap)")
      .def_property(
          "metricMaps",
          [](const MCLParams& p)
          {
            std::vector<mrpt::maps::CMetricMap::Ptr> v;
            for (const auto& mp : p.metricMaps)
            {
              v.push_back(std::const_pointer_cast<mrpt::maps::CMetricMap>(mp));
            }
            return v;
          },
          [](MCLParams& p, const std::vector<mrpt::maps::CMetricMap::Ptr>& maps)
          { p.metricMaps.assign(maps.begin(), maps.end()); },
          "Alternative to metricMap: one map per particle (rarely used)")
      .def_readwrite("KLD_params", &MCLParams::KLD_params);

  using MCL2D = mrpt::slam::CMonteCarloLocalization2D;
  py::class_<MCL2D, mrpt::poses::CPosePDFParticles, std::shared_ptr<MCL2D>>(
      m, "CMonteCarloLocalization2D")
      .def(py::init<size_t>(), "M"_a = 1, "Creates a filter with M particles")
      .def_readwrite("options", &MCL2D::options)
      .def(
          "resetUniformFreeSpace", &MCL2D::resetUniformFreeSpace, "theMap"_a,
          "freeCellsThreshold"_a = 0.7, "particlesCount"_a = -1, "x_min"_a = -1e10,
          "x_max"_a = 1e10, "y_min"_a = -1e10, "y_max"_a = 1e10, "phi_min"_a = -M_PI,
          "phi_max"_a = M_PI,
          "Spreads particles uniformly over the free space of an occupancy grid (global "
          "localization)")
      .def(
          "getVisualization",
          [](const MCL2D& pdf) { return mrpt::viz::CSetOfObjects::posePDF2opengl(pdf); },
          "Returns a 3D representation of the particles")
      .def(
          "__repr__", [](const MCL2D& pdf)
          { return "CMonteCarloLocalization2D(" + std::to_string(pdf.size()) + " particles)"; });

  using MCL3D = mrpt::slam::CMonteCarloLocalization3D;
  py::class_<MCL3D, mrpt::poses::CPose3DPDFParticles, std::shared_ptr<MCL3D>>(
      m, "CMonteCarloLocalization3D")
      .def(py::init<size_t>(), "M"_a = 1, "Creates a filter with M particles")
      .def_readwrite("options", &MCL3D::options)
      .def(
          "getVisualization", [](const MCL3D& pdf) { return pdf.getVisualization(); },
          "Returns a 3D representation of the particles")
      .def(
          "__repr__", [](const MCL3D& pdf)
          { return "CMonteCarloLocalization3D(" + std::to_string(pdf.size()) + " particles)"; });

  // -------------------------------------------------------------------------
  // CMetricMapBuilderRBPF: Rao-Blackwellized particle filter SLAM
  // -------------------------------------------------------------------------
  using PredParams = mrpt::maps::CMultiMetricMapPDF::TPredictionParams;
  py::class_<PredParams, mrpt::config::CLoadableOptions, std::shared_ptr<PredParams>>(
      m, "TPredictionParams")
      .def(py::init<>())
      .def_readwrite("pfOptimalProposal_mapSelection", &PredParams::pfOptimalProposal_mapSelection)
      .def_readwrite("ICPGlobalAlign_MinQuality", &PredParams::ICPGlobalAlign_MinQuality)
      .def_readwrite("KLD_params", &PredParams::KLD_params)
      .def_readwrite("icp_params", &PredParams::icp_params);

  using RBPF = mrpt::slam::CMetricMapBuilderRBPF;
  py::class_<RBPF, mrpt::slam::CMetricMapBuilder> rbpf(m, "CMetricMapBuilderRBPF");

  py::class_<
      RBPF::TConstructionOptions, mrpt::config::CLoadableOptions,
      std::shared_ptr<RBPF::TConstructionOptions>>(rbpf, "TConstructionOptions")
      .def(py::init<>())
      .def_readwrite("insertionLinDistance", &RBPF::TConstructionOptions::insertionLinDistance)
      .def_readwrite("insertionAngDistance", &RBPF::TConstructionOptions::insertionAngDistance)
      .def_readwrite("localizeLinDistance", &RBPF::TConstructionOptions::localizeLinDistance)
      .def_readwrite("localizeAngDistance", &RBPF::TConstructionOptions::localizeAngDistance)
      .def_readwrite("PF_options", &RBPF::TConstructionOptions::PF_options)
      .def_readwrite("mapsInitializers", &RBPF::TConstructionOptions::mapsInitializers)
      .def_readwrite("predictionOptions", &RBPF::TConstructionOptions::predictionOptions);

  rbpf.def(py::init<>())
      .def(py::init<const RBPF::TConstructionOptions&>(), "options"_a)
      .def(
          "initialize",
          [](RBPF& b, const mrpt::maps::CSimpleMap& initialMap) { b.initialize(initialMap); },
          "initialMap"_a = mrpt::maps::CSimpleMap(),
          "Resets the filter, optionally starting from a given map")
      .def("clear", &RBPF::clear, "Clears all maps and resets the filter")
      .def(
          "processActionObservation", &RBPF::processActionObservation, "action"_a, "sf"_a,
          "Processes one (action, sensory frame) pair")
      .def(
          "getCurrentPoseEstimation", &RBPF::getCurrentPoseEstimation,
          "Returns the current robot pose estimation (a CPose3DPDF)")
      .def(
          "getCurrentlyBuiltMetricMap",
          [](const RBPF& b) -> const mrpt::maps::CMultiMetricMap&
          { return b.getCurrentlyBuiltMetricMap(); },
          py::return_value_policy::reference_internal,
          "Returns the map of the most likely particle")
      .def(
          "getCurrentlyBuiltMap",
          [](const RBPF& b)
          {
            mrpt::maps::CSimpleMap sm;
            b.getCurrentlyBuiltMap(sm);
            return sm;
          },
          "Returns the keyframes of the most likely particle as a CSimpleMap")
      .def("getCurrentlyBuiltMapSize", &RBPF::getCurrentlyBuiltMapSize)
      .def(
          "getCurrentMostLikelyPath",
          [](const RBPF& b)
          {
            std::deque<mrpt::math::TPose3D> path;
            b.getCurrentMostLikelyPath(path);
            return std::vector<mrpt::math::TPose3D>(path.begin(), path.end());
          },
          "Returns the robot path of the most likely particle, as a list of TPose3D")
      .def("getCurrentJointEntropy", &RBPF::getCurrentJointEntropy)
      .def(
          "saveCurrentPathEstimationToTextFile", &RBPF::saveCurrentPathEstimationToTextFile,
          "fileName"_a);
}
