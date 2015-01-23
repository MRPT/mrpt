/* MRPT */
#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservationRange.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CActionRobotMovement2D.h>

#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>

#include <mrpt/slam/CICP.h>
#include <mrpt/slam/CMetricMapBuilderICP.h>
#include <mrpt/slam/CMetricMapBuilderRBPF.h>

#include <mrpt/poses/CPosePDFGaussian.h>

/* bindings */
#include "bindings.h"
#include "maps_bindings.h"
#include "poses_bindings.h"

/* STD */
#include <stdint.h>

using namespace boost::python;

using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::obs;

// TMetricMapInitializer
void TMetricMapInitializer_set_COccupancyGridMap2D(TMetricMapInitializer &self, float min_x, float max_x, float min_y, float max_y, float resolution)
{
// FIXME old_stable: 1.2
//     self.metricMapClassType = CLASS_ID(COccupancyGridMap2D);
//     self.m_disableSaveAs3DObject = true;
//     self.occupancyGridMap2D_options.min_x = min_x;
//     self.occupancyGridMap2D_options.max_x = max_x;
//     self.occupancyGridMap2D_options.min_y = min_y;
//     self.occupancyGridMap2D_options.max_y = max_y;
//     self.occupancyGridMap2D_options.resolution = resolution;
}

// CICP
tuple CICP_AlignPDF1(CICP &self, COccupancyGridMap2D &m1, CSimplePointsMap &m2, CPosePDFGaussian &initialEstimationPDF)
{
    CPosePDFGaussian posePDF;
    float runningTime;
    CICP::TReturnInfo info;

    CPosePDFPtr posePDFPtr = self.AlignPDF(&m1, &m2, initialEstimationPDF, &runningTime, &info);
    posePDF.copyFrom(*posePDFPtr);

    boost::python::list ret_val;
    ret_val.append(posePDF);
    ret_val.append(runningTime);
    ret_val.append(info);
    return tuple(ret_val);
}

tuple CICP_AlignPDF2(CICP &self, CSimplePointsMap &m1, CSimplePointsMap &m2, CPosePDFGaussian &initialEstimationPDF)
{
    CPosePDFGaussian posePDF;
    float runningTime;
    CICP::TReturnInfo info;

    CPosePDFPtr posePDFPtr = self.AlignPDF(&m1, &m2, initialEstimationPDF, &runningTime, &info);
    posePDF.copyFrom(*posePDFPtr);

    boost::python::list ret_val;
    ret_val.append(posePDF);
    ret_val.append(runningTime);
    ret_val.append(info);
    return tuple(ret_val);
}
// end of CICP

// CMetricMap
void CMetricMapWrap::internal_clear()
{
    this->get_override("internal_clear")();
}

bool CMetricMapWrap::isEmpty()
{
    return this->get_override("isEmpty")();
}

double CMetricMapWrap::computeObservationLikelihood(const CObservation *obs, const CPose3D &takenFrom)
{
    return this->get_override("computeObservationLikelihood")(obs, takenFrom);
}

void CMetricMapWrap::saveMetricMapRepresentationToFile(const std::string &filNamePrefix)
{
    this->get_override("saveMetricMapRepresentationToFile")(filNamePrefix);
}

void CMetricMapWrap::getAs3DObject(mrpt::opengl::CSetOfObjectsPtr &outObj)
{
    this->get_override("getAs3DObject")(outObj);
}

// CMetricMapBuilder
struct CMetricMapBuilderWrap : CMetricMapBuilder, wrapper<CMetricMapBuilder>
{
    void initialize(const CSimpleMap &initialMap = CSimpleMap(), CPosePDF *x0 = NULL)
    {
        this->get_override("initialize")(initialMap, x0);
    }

    CPose3DPDFPtr getCurrentPoseEstimation() const
    {
        return this->get_override("getCurrentPoseEstimation")();
    }

    void processActionObservation(CActionCollection &action, CSensoryFrame &observations)
    {
        this->get_override("processActionObservation")(action, observations);
    }

    void getCurrentlyBuiltMap(CSimpleMap &out_map) const
    {
        this->get_override("getCurrentlyBuiltMap")(out_map);
    }

    void saveCurrentEstimationToImage(const std::string &file, bool formatEMF_BMP = true)
    {
        this->get_override("saveCurrentEstimationToImage")(file, formatEMF_BMP);
    }
};

CSimpleMap CMetricMapBuilder_getCurrentlyBuiltMap(CMetricMapBuilder &self)
{
  CSimpleMap out_map;
  self.getCurrentlyBuiltMap(out_map);
  return out_map;
}

CPose2D CMetricMapBuilder_getCurrentPose2D(CMetricMapBuilder &self)
{
  return CPose2D(self.getCurrentPoseEstimation()->getMeanVal());
}

void CMetricMapBuilderICP_initialize(CMetricMapBuilderICP& self, CSimpleMap& initialMap, CPosePDFGaussian& x0)
{
    self.initialize(initialMap, &x0);
}

void CMetricMapBuilderRBPF_initialize(CMetricMapBuilderRBPF& self, CSimpleMap& initialMap, CPosePDFParticles& x0)
{
    self.initialize(initialMap, &x0);
}

CPose3DPDFParticlesPtr CMetricMapBuilderRBPF_getCurrentPoseEstimation(CMetricMapBuilderRBPF& self)
{
    return (CPose3DPDFParticlesPtr) self.getCurrentPoseEstimation();
}
// end of CMetricMapBuilder


void export_slam()
{
    // map namespace to be submodule of mrpt package
    object slam_module(handle<>(borrowed(PyImport_AddModule("mrpt.slam"))));
    scope().attr("slam") = slam_module;
    scope slam_scope = slam_module;

    // CMetricMap
    {
        scope s = class_<CMetricMapWrap, boost::noncopyable>("CMetricMap", no_init)
        ;
    }

    // CICP
    {
        scope s = class_<CICP>("CICP", init<CICP::TConfigParams>())
            .def_readwrite("options", &CICP::options)
            .def("AlignPDF", &CICP_AlignPDF1, "This method computes the PDF of the displacement (relative pose) between two maps (COccupancyGridMap2D/CSimplePointsMap): the relative pose of m2 with respect to m1. This pose is returned as a PDF rather than a single value.")
            .def("AlignPDF", &CICP_AlignPDF2, "This method computes the PDF of the displacement (relative pose) between two maps (CSimplePointsMap/CSimplePointsMap): the relative pose of m2 with respect to m1. This pose is returned as a PDF rather than a single value.")
        ;

        class_<CICP::TConfigParams, bases<CLoadableOptions> >("TConfigParams", init<>())
            .def_readwrite("ICP_algorithm", &CICP::TConfigParams::ICP_algorithm)
            .def_readwrite("onlyClosestCorrespondences", &CICP::TConfigParams::onlyClosestCorrespondences)
            .def_readwrite("onlyUniqueRobust", &CICP::TConfigParams::onlyUniqueRobust)
            .def_readwrite("maxIterations", &CICP::TConfigParams::maxIterations)
            .def_readwrite("minAbsStep_trans", &CICP::TConfigParams::minAbsStep_trans)
            .def_readwrite("minAbsStep_rot", &CICP::TConfigParams::minAbsStep_rot)
            .def_readwrite("thresholdDist", &CICP::TConfigParams::thresholdDist)
            .def_readwrite("thresholdAng", &CICP::TConfigParams::thresholdAng)
            .def_readwrite("ALFA", &CICP::TConfigParams::ALFA)
            .def_readwrite("smallestThresholdDist", &CICP::TConfigParams::smallestThresholdDist)
            .def_readwrite("covariance_varPoints", &CICP::TConfigParams::covariance_varPoints)
            .def_readwrite("doRANSAC", &CICP::TConfigParams::doRANSAC)
            .def_readwrite("ransac_minSetSize", &CICP::TConfigParams::ransac_minSetSize)
            .def_readwrite("ransac_maxSetSize", &CICP::TConfigParams::ransac_maxSetSize)
            .def_readwrite("ransac_nSimulations", &CICP::TConfigParams::ransac_nSimulations)
            .def_readwrite("ransac_mahalanobisDistanceThreshold", &CICP::TConfigParams::ransac_mahalanobisDistanceThreshold)
            .def_readwrite("normalizationStd", &CICP::TConfigParams::normalizationStd)
            .def_readwrite("ransac_fuseByCorrsMatch", &CICP::TConfigParams::ransac_fuseByCorrsMatch)
            .def_readwrite("ransac_fuseMaxDiffXY", &CICP::TConfigParams::ransac_fuseMaxDiffXY)
            .def_readwrite("ransac_fuseMaxDiffPhi", &CICP::TConfigParams::ransac_fuseMaxDiffPhi)
            .def_readwrite("kernel_rho", &CICP::TConfigParams::kernel_rho)
            .def_readwrite("use_kernel", &CICP::TConfigParams::use_kernel)
            .def_readwrite("Axy_aprox_derivatives", &CICP::TConfigParams::Axy_aprox_derivatives)
            .def_readwrite("LM_initial_lambda", &CICP::TConfigParams::LM_initial_lambda)
            .def_readwrite("skip_cov_calculation", &CICP::TConfigParams::skip_cov_calculation)
//          .def_readwrite("skip_quality_calculation", &CICP::TConfigParams::skip_quality_calculation)
            .def_readwrite("corresponding_points_decimation", &CICP::TConfigParams::corresponding_points_decimation)
        ;

        class_<CICP::TReturnInfo>("TReturnInfo", init<>())
            .def_readwrite("cbSize", &CICP::TReturnInfo::cbSize)
            .def_readwrite("nIterations", &CICP::TReturnInfo::nIterations)
            .def_readwrite("goodness", &CICP::TReturnInfo::goodness)
            .def_readwrite("quality", &CICP::TReturnInfo::quality)
        ;
    }
    // CMetricMapBuilder
    {
        scope s = class_<CMetricMapBuilderWrap, boost::noncopyable>("CMetricMapBuilder", no_init)
            .def("getCurrentPose2D", &CMetricMapBuilder_getCurrentPose2D, "Returns a copy of the current best pose estimation as a 2D pose.")
            .def("processActionObservation", &CMetricMapBuilder::processActionObservation, "Process a new action and observations pair to update this map.")
            .def("getCurrentlyBuiltMap", &CMetricMapBuilder_getCurrentlyBuiltMap, "Returns \"out_map\" with the set of \"poses\"-\"sensory-frames\", thus the so far built map.")
            .def("getCurrentlyBuiltMapSize", &CMetricMapBuilder::getCurrentlyBuiltMapSize, "Returns just how many sensory-frames are stored in the currently build map.")
            .def("saveCurrentEstimationToImage", &CMetricMapBuilder::saveCurrentEstimationToImage, "A useful method for debugging: the current map (and/or poses) estimation is dumped to an image file.")
            .def("clear", &CMetricMapBuilder::clear, "Clear all elements of the maps, and reset localization to (0,0,0deg).")
            .def("enableMapUpdating", &CMetricMapBuilder::enableMapUpdating, "Enables or disables the map updating (default state is enabled).")
            .def("loadCurrentMapFromFile", &CMetricMapBuilder::loadCurrentMapFromFile, "Load map (mrpt::slam::CSimpleMap) from a \".simplemap\" file")
            .def("saveCurrentMapToFile", &CMetricMapBuilder::saveCurrentMapToFile, "Save map (mrpt::slam::CSimpleMap) to a \".simplemap\" file.")
        ;
    }

    // CMetricMapBuilderICP
    {
        scope s = class_<CMetricMapBuilderICP, bases<CMetricMapBuilder> >("CMetricMapBuilderICP", init<>())
            .def("initialize", &CMetricMapBuilderICP_initialize, "Initialize the method, starting with a known location PDF \"x0\"(if supplied, set to NULL to left unmodified) and a given fixed, past map.")
            .def_readwrite("ICP_options", &CMetricMapBuilderICP::ICP_options)
            .def_readwrite("ICP_params", &CMetricMapBuilderICP::ICP_params)
        ;

        // TConfigParams
        class_<CMetricMapBuilderICP::TConfigParams, bases<CLoadableOptions> >("TConfigParams", init<>())
            .def_readwrite("matchAgainstTheGrid", &CMetricMapBuilderICP::TConfigParams::matchAgainstTheGrid)
            .def_readwrite("insertionLinDistance", &CMetricMapBuilderICP::TConfigParams::insertionLinDistance)
            .def_readwrite("insertionAngDistance", &CMetricMapBuilderICP::TConfigParams::insertionAngDistance)
            .def_readwrite("localizationLinDistance", &CMetricMapBuilderICP::TConfigParams::localizationLinDistance)
            .def_readwrite("localizationAngDistance", &CMetricMapBuilderICP::TConfigParams::localizationAngDistance)
            .def_readwrite("minICPgoodnessToAccept", &CMetricMapBuilderICP::TConfigParams::minICPgoodnessToAccept)
            .def_readwrite("mapInitializers", &CMetricMapBuilderICP::TConfigParams::mapInitializers)
        ;
    }

    // CMetricMapBuilderRBPF
    {
        scope s = class_<CMetricMapBuilderRBPF, bases<CMetricMapBuilder> >("CMetricMapBuilderRBPF", init<CMetricMapBuilderRBPF::TConstructionOptions>())
            .def("getCurrentPoseEstimation", &CMetricMapBuilderRBPF_getCurrentPoseEstimation, "Returns a copy of the current best pose estimation as a pose PDF.")
//          .def("drawCurrentEstimationToImage", &CMetricMapBuilderRBPF::drawCurrentEstimationToImage) // this function seems not to be useful within python
            .def("initialize", &CMetricMapBuilderRBPF_initialize, "Initialize the method, starting with a known location PDF \"x0\"(if supplied, set to NULL to left unmodified) and a given fixed, past map.")
            .def("saveCurrentPathEstimationToTextFile", &CMetricMapBuilderRBPF::saveCurrentPathEstimationToTextFile)
            .def("getCurrentJointEntropy", &CMetricMapBuilderRBPF::getCurrentJointEntropy)
            .def_readwrite("mapPDF", &CMetricMapBuilderRBPF::mapPDF)
        ;

        // TConstructionOptions
        class_<CMetricMapBuilderRBPF::TConstructionOptions, bases<CLoadableOptions> >("TConstructionOptions", init<>())
            .def_readwrite("insertionLinDistance", &CMetricMapBuilderRBPF::TConstructionOptions::insertionLinDistance)
            .def_readwrite("insertionAngDistance", &CMetricMapBuilderRBPF::TConstructionOptions::insertionAngDistance)
            .def_readwrite("localizeLinDistance", &CMetricMapBuilderRBPF::TConstructionOptions::localizeLinDistance)
            .def_readwrite("localizeAngDistance", &CMetricMapBuilderRBPF::TConstructionOptions::localizeAngDistance)
            .def_readwrite("PF_options", &CMetricMapBuilderRBPF::TConstructionOptions::PF_options)
            .def_readwrite("mapsInitializers", &CMetricMapBuilderRBPF::TConstructionOptions::mapsInitializers)
            .def_readwrite("predictionOptions", &CMetricMapBuilderRBPF::TConstructionOptions::predictionOptions)
        ;
    }
}
