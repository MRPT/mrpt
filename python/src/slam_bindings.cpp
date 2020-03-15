/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#include "bindings.h"

/* MRPT */
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationRange.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/serialization/CArchive.h>

#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>

#include <mrpt/slam/CICP.h>
#include <mrpt/slam/CMetricMapBuilderICP.h>
#include <mrpt/slam/CMetricMapBuilderRBPF.h>
#include <mrpt/slam/CMonteCarloLocalization2D.h>
#include <mrpt/slam/CMonteCarloLocalization3D.h>
#include <mrpt/slam/CRangeBearingKFSLAM.h>
#include <mrpt/slam/CRangeBearingKFSLAM2D.h>

#include <mrpt/bayes/CParticleFilter.h>

#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFParticles.h>

#include <mrpt/io/CStream.h>

/* STD */
#include <cstdint>

using namespace boost::python;
using namespace mrpt::config;
using namespace mrpt::poses;
using namespace mrpt::bayes;
using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::io;

// CICP
tuple CICP_AlignPDF1(
	CICP& self, COccupancyGridMap2D& m1, CSimplePointsMap& m2,
	CPosePDFGaussian& initialEstimationPDF)
{
	CPosePDFGaussian posePDF;
	float runningTime;
	CICP::TReturnInfo info;

	CPosePDF::Ptr posePDFPtr =
		self.AlignPDF(&m1, &m2, initialEstimationPDF, &runningTime, &info);
	posePDF.copyFrom(*posePDFPtr);

	boost::python::list ret_val;
	ret_val.append(posePDF);
	ret_val.append(runningTime);
	ret_val.append(info);
	return tuple(ret_val);
}

tuple CICP_AlignPDF2(
	CICP& self, CSimplePointsMap& m1, CSimplePointsMap& m2,
	CPosePDFGaussian& initialEstimationPDF)
{
	CPosePDFGaussian posePDF;
	float runningTime;
	CICP::TReturnInfo info;

	CPosePDF::Ptr posePDFPtr =
		self.AlignPDF(&m1, &m2, initialEstimationPDF, &runningTime, &info);
	posePDF.copyFrom(*posePDFPtr);

	boost::python::list ret_val;
	ret_val.append(posePDF);
	ret_val.append(runningTime);
	ret_val.append(info);
	return tuple(ret_val);
}
// end of CICP

// CMetricMapBuilder
struct CMetricMapBuilderWrap : CMetricMapBuilder, wrapper<CMetricMapBuilder>
{
	void initialize(
		const CSimpleMap& initialMap = CSimpleMap(), CPosePDF* x0 = nullptr)
	{
		this->get_override("initialize")(initialMap, x0);
	}

	CPose3DPDF::Ptr getCurrentPoseEstimation() const override
	{
		return this->get_override("getCurrentPoseEstimation")();
	}

	void processActionObservation(
		CActionCollection& action, CSensoryFrame& observations) override
	{
		this->get_override("processActionObservation")(action, observations);
	}

	void getCurrentlyBuiltMap(CSimpleMap& out_map) const override
	{
		this->get_override("getCurrentlyBuiltMap")(out_map);
	}

	void saveCurrentEstimationToImage(
		const std::string& file, bool formatEMF_BMP = true) override
	{
		this->get_override("saveCurrentEstimationToImage")(file, formatEMF_BMP);
	}
};

CSimpleMap CMetricMapBuilder_getCurrentlyBuiltMap(CMetricMapBuilder& self)
{
	CSimpleMap out_map;
	self.getCurrentlyBuiltMap(out_map);
	return out_map;
}

CPose2D CMetricMapBuilder_getCurrentPose2D(CMetricMapBuilder& self)
{
	return CPose2D(self.getCurrentPoseEstimation()->getMeanVal());
}

void CMetricMapBuilderICP_initialize(
	CMetricMapBuilderICP& self, CSimpleMap& initialMap, CPosePDFGaussian& x0)
{
	self.initialize(initialMap, &x0);
}

void CMetricMapBuilderRBPF_initialize(
	CMetricMapBuilderRBPF& self, CSimpleMap& initialMap, CPosePDFParticles& x0)
{
	self.initialize(initialMap, &x0);
}

CPose3DPDFParticles::Ptr CMetricMapBuilderRBPF_getCurrentPoseEstimation(
	CMetricMapBuilderRBPF& self)
{
	return std::dynamic_pointer_cast<CPose3DPDFParticles>(
		self.getCurrentPoseEstimation());
}
// end of CMetricMapBuilder

// CRangeBearingKFSLAM2D
tuple CRangeBearingKFSLAM2D_getCurrentState(CRangeBearingKFSLAM2D& self)
{
	list ret_val;

	CPosePDFGaussian out_robotPose;
	std::vector<mrpt::math::TPoint2D> out_landmarksPositions;
	std::map<unsigned int, mrpt::maps::CLandmark::TLandmarkID> out_landmarkIDs;
	mrpt::math::CVectorDouble out_fullState;
	mrpt::math::CMatrixDouble out_fullCovariance;

	self.getCurrentState(
		out_robotPose, out_landmarksPositions, out_landmarkIDs, out_fullState,
		out_fullCovariance);

	ret_val.append(out_robotPose);
	ret_val.append(out_landmarksPositions);
	ret_val.append(out_landmarkIDs);
	ret_val.append(out_fullState);
	ret_val.append(out_fullCovariance);

	return tuple(ret_val);
}

CPosePDFGaussian CRangeBearingKFSLAM2D_getCurrentRobotPose(
	CRangeBearingKFSLAM2D& self)
{
	CPosePDFGaussian out_robotPose;
	self.getCurrentRobotPose(out_robotPose);
	return out_robotPose;
}

mrpt::opengl::CSetOfObjects::Ptr CRangeBearingKFSLAM2D_getAs3DObject(
	CRangeBearingKFSLAM2D& self)
{
	mrpt::opengl::CSetOfObjects::Ptr outObj =
		mrpt::opengl::CSetOfObjects::Create();
	self.getAs3DObject(outObj);
	return outObj;
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(
	CRangeBearingKFSLAM2D_saveMapAndPath2DRepresentationAsMATLABFile_overloads,
	saveMapAndPath2DRepresentationAsMATLABFile, 1, 5)
// end of CRangeBearingKFSLAM2D

// CMonteCarloLocalization2D
void CMonteCarloLocalization2D_prediction_and_update_pfStandardProposal(
	CMonteCarloLocalization2D& self,
	const mrpt::obs::CActionCollection::Ptr action,
	const mrpt::obs::CSensoryFrame::Ptr observation,
	const CParticleFilter::TParticleFilterOptions& PF_options)
{
	self.prediction_and_update_pfStandardProposal(
		action.get(), observation.get(), PF_options);
}

void CMonteCarloLocalization2D_prediction_and_update_pfAuxiliaryPFStandard(
	CMonteCarloLocalization2D& self,
	const mrpt::obs::CActionCollection::Ptr action,
	const mrpt::obs::CSensoryFrame::Ptr observation,
	const CParticleFilter::TParticleFilterOptions& PF_options)
{
	self.prediction_and_update_pfAuxiliaryPFStandard(
		action.get(), observation.get(), PF_options);
}

void CMonteCarloLocalization2D_prediction_and_update_pfAuxiliaryPFOptimal(
	CMonteCarloLocalization2D& self,
	const mrpt::obs::CActionCollection::Ptr action,
	const mrpt::obs::CSensoryFrame::Ptr observation,
	const CParticleFilter::TParticleFilterOptions& PF_options)
{
	self.prediction_and_update_pfAuxiliaryPFOptimal(
		action.get(), observation.get(), PF_options);
}

void CMonteCarloLocalization2D_prediction_and_update(
	CMonteCarloLocalization2D& self,
	const mrpt::obs::CActionCollection::Ptr action,
	const mrpt::obs::CSensoryFrame::Ptr observation,
	const CParticleFilter::TParticleFilterOptions& PF_options)
{
	self.prediction_and_update(action.get(), observation.get(), PF_options);
}

CPose2D CMonteCarloLocalization2D_getMean(CMonteCarloLocalization2D& self)
{
	CPose2D mean;
	self.getMean(mean);
	return mean;
}

tuple CMonteCarloLocalization2D_getCovarianceAndMean(
	CMonteCarloLocalization2D& self)
{
	list ret_val;
	const auto [cov, mean_point] = self.getCovarianceAndMean();
	ret_val.append(cov);
	ret_val.append(mean_point);
	return tuple(ret_val);
}

CPose2D CMonteCarloLocalization2D_drawSingleSample(
	CMonteCarloLocalization2D& self)
{
	CPose2D outPart;
	self.drawSingleSample(outPart);
	return outPart;
}

CPosePDFParticles CMonteCarloLocalization2D_inverse(
	CMonteCarloLocalization2D& self)
{
	CPosePDFParticles o;
	self.inverse(o);
	return o;
}

void CMonteCarloLocalization2D_writeParticlesToStream(
	CMonteCarloLocalization2D& self, CStream& out)
{
	auto arch = mrpt::serialization::archiveFrom(out);
	self.writeParticlesToStream(arch);
}

void CMonteCarloLocalization2D_readParticlesFromStream(
	CMonteCarloLocalization2D& self, CStream& in)
{
	auto arch = mrpt::serialization::archiveFrom(in);
	self.readParticlesFromStream(arch);
}

mrpt::opengl::CSetOfObjects::Ptr CMonteCarloLocalization2D_getAs3DObject(
	CMonteCarloLocalization2D& self)
{
	mrpt::opengl::CSetOfObjects::Ptr outObj =
		mrpt::opengl::CSetOfObjects::Create();
	self.getAs3DObject(outObj);
	return outObj;
}

tuple CMonteCarloLocalization2D_normalizeWeights(
	CMonteCarloLocalization2D& self)
{
	list ret_val;
	double out_max_log_w;
	double norm = self.normalizeWeights(&out_max_log_w);
	ret_val.append(norm);
	ret_val.append(out_max_log_w);
	return tuple(ret_val);
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(
	CMonteCarloLocalization2D_resetUniformFreeSpace_overloads,
	resetUniformFreeSpace, 1, 8)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(
	CMonteCarloLocalization2D_resetDeterministic_overloads, resetDeterministic,
	1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(
	CMonteCarloLocalization2D_resetUniform_overloads, resetUniform, 4, 7)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(
	CMonteCarloLocalization2D_performResampling_overloads, performResampling, 1,
	2)
// end of CMonteCarloLocalization2D

// CMonteCarloLocalization3D
void CMonteCarloLocalization3D_prediction_and_update_pfStandardProposal(
	CMonteCarloLocalization3D& self,
	const mrpt::obs::CActionCollection::Ptr action,
	const mrpt::obs::CSensoryFrame::Ptr observation,
	const CParticleFilter::TParticleFilterOptions& PF_options)
{
	self.prediction_and_update_pfStandardProposal(
		action.get(), observation.get(), PF_options);
}

void CMonteCarloLocalization3D_prediction_and_update_pfAuxiliaryPFStandard(
	CMonteCarloLocalization3D& self,
	const mrpt::obs::CActionCollection::Ptr action,
	const mrpt::obs::CSensoryFrame::Ptr observation,
	const CParticleFilter::TParticleFilterOptions& PF_options)
{
	self.prediction_and_update_pfAuxiliaryPFStandard(
		action.get(), observation.get(), PF_options);
}

void CMonteCarloLocalization3D_prediction_and_update_pfAuxiliaryPFOptimal(
	CMonteCarloLocalization3D& self,
	const mrpt::obs::CActionCollection::Ptr action,
	const mrpt::obs::CSensoryFrame::Ptr observation,
	const CParticleFilter::TParticleFilterOptions& PF_options)
{
	self.prediction_and_update_pfAuxiliaryPFOptimal(
		action.get(), observation.get(), PF_options);
}

void CMonteCarloLocalization3D_prediction_and_update(
	CMonteCarloLocalization3D& self,
	const mrpt::obs::CActionCollection::Ptr action,
	const mrpt::obs::CSensoryFrame::Ptr observation,
	const CParticleFilter::TParticleFilterOptions& PF_options)
{
	self.prediction_and_update(action.get(), observation.get(), PF_options);
}

CPose3D CMonteCarloLocalization3D_getMean(CMonteCarloLocalization3D& self)
{
	CPose3D mean;
	self.getMean(mean);
	return mean;
}

tuple CMonteCarloLocalization3D_getCovarianceAndMean(
	CMonteCarloLocalization3D& self)
{
	list ret_val;
	const auto [cov, mean_point] = self.getCovarianceAndMean();
	ret_val.append(cov);
	ret_val.append(mean_point);
	return tuple(ret_val);
}

CPose3D CMonteCarloLocalization3D_drawSingleSample(
	CMonteCarloLocalization3D& self)
{
	CPose3D outPart;
	self.drawSingleSample(outPart);
	return outPart;
}

CPose3DPDFParticles CMonteCarloLocalization3D_inverse(
	CMonteCarloLocalization3D& self)
{
	CPose3DPDFParticles o;
	self.inverse(o);
	return o;
}

void CMonteCarloLocalization3D_writeParticlesToStream(
	CMonteCarloLocalization3D& self, CStream& out)
{
	auto arch = mrpt::serialization::archiveFrom(out);
	self.writeParticlesToStream(arch);
}

void CMonteCarloLocalization3D_readParticlesFromStream(
	CMonteCarloLocalization3D& self, CStream& in)
{
	auto arch = mrpt::serialization::archiveFrom(in);
	self.readParticlesFromStream(arch);
}

mrpt::opengl::CSetOfObjects::Ptr CMonteCarloLocalization3D_getAs3DObject(
	CMonteCarloLocalization3D& self)
{
	mrpt::opengl::CSetOfObjects::Ptr outObj =
		mrpt::opengl::CSetOfObjects::Create();
	self.getAs3DObject(outObj);
	return outObj;
}

tuple CMonteCarloLocalization3D_normalizeWeights(
	CMonteCarloLocalization3D& self)
{
	list ret_val;
	double out_max_log_w;
	double norm = self.normalizeWeights(&out_max_log_w);
	ret_val.append(norm);
	ret_val.append(out_max_log_w);
	return tuple(ret_val);
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(
	CMonteCarloLocalization3D_resetDeterministic_overloads, resetDeterministic,
	1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(
	CMonteCarloLocalization3D_performResampling_overloads, performResampling, 1,
	2)
// end of CMonteCarloLocalization3D

void export_slam()
{
	// map namespace to be submodule of mrpt package
	MAKE_SUBMODULE(slam)

	// CICP
	{
		scope s = class_<CICP>("CICP", init<CICP::TConfigParams>())
					  .def_readwrite("options", &CICP::options)
					  .def(
						  "AlignPDF", &CICP_AlignPDF1,
						  "This method computes the PDF of the displacement "
						  "(relative pose) between two maps "
						  "(COccupancyGridMap2D/CSimplePointsMap): the "
						  "relative pose of m2 with respect to m1. This pose "
						  "is returned as a PDF rather than a single value.")
					  .def(
						  "AlignPDF", &CICP_AlignPDF2,
						  "This method computes the PDF of the displacement "
						  "(relative pose) between two maps "
						  "(CSimplePointsMap/CSimplePointsMap): the relative "
						  "pose of m2 with respect to m1. This pose is "
						  "returned as a PDF rather than a single value.");

		class_<CICP::TConfigParams, bases<CLoadableOptions>>(
			"TConfigParams", init<>())
			.def_readwrite("ICP_algorithm", &CICP::TConfigParams::ICP_algorithm)
			.def_readwrite(
				"onlyUniqueRobust", &CICP::TConfigParams::onlyUniqueRobust)
			.def_readwrite("maxIterations", &CICP::TConfigParams::maxIterations)
			.def_readwrite(
				"minAbsStep_trans", &CICP::TConfigParams::minAbsStep_trans)
			.def_readwrite(
				"minAbsStep_rot", &CICP::TConfigParams::minAbsStep_rot)
			.def_readwrite("thresholdDist", &CICP::TConfigParams::thresholdDist)
			.def_readwrite("thresholdAng", &CICP::TConfigParams::thresholdAng)
			.def_readwrite("ALFA", &CICP::TConfigParams::ALFA)
			.def_readwrite(
				"smallestThresholdDist",
				&CICP::TConfigParams::smallestThresholdDist)
			.def_readwrite(
				"covariance_varPoints",
				&CICP::TConfigParams::covariance_varPoints)
			.def_readwrite("doRANSAC", &CICP::TConfigParams::doRANSAC)
			.def_readwrite(
				"ransac_minSetSize", &CICP::TConfigParams::ransac_minSetSize)
			.def_readwrite(
				"ransac_maxSetSize", &CICP::TConfigParams::ransac_maxSetSize)
			.def_readwrite(
				"ransac_nSimulations",
				&CICP::TConfigParams::ransac_nSimulations)
			.def_readwrite(
				"ransac_mahalanobisDistanceThreshold",
				&CICP::TConfigParams::ransac_mahalanobisDistanceThreshold)
			.def_readwrite(
				"normalizationStd", &CICP::TConfigParams::normalizationStd)
			.def_readwrite(
				"ransac_fuseByCorrsMatch",
				&CICP::TConfigParams::ransac_fuseByCorrsMatch)
			.def_readwrite(
				"ransac_fuseMaxDiffXY",
				&CICP::TConfigParams::ransac_fuseMaxDiffXY)
			.def_readwrite(
				"ransac_fuseMaxDiffPhi",
				&CICP::TConfigParams::ransac_fuseMaxDiffPhi)
			.def_readwrite("kernel_rho", &CICP::TConfigParams::kernel_rho)
			.def_readwrite("use_kernel", &CICP::TConfigParams::use_kernel)
			.def_readwrite(
				"Axy_aprox_derivatives",
				&CICP::TConfigParams::Axy_aprox_derivatives)
			.def_readwrite(
				"LM_initial_lambda", &CICP::TConfigParams::LM_initial_lambda)
			.def_readwrite(
				"skip_cov_calculation",
				&CICP::TConfigParams::skip_cov_calculation)
			//          .def_readwrite("skip_quality_calculation",
			//          &CICP::TConfigParams::skip_quality_calculation)
			.def_readwrite(
				"corresponding_points_decimation",
				&CICP::TConfigParams::corresponding_points_decimation);

		class_<CICP::TReturnInfo>("TReturnInfo", init<>())
			.def_readwrite("nIterations", &CICP::TReturnInfo::nIterations)
			.def_readwrite("goodness", &CICP::TReturnInfo::goodness)
			.def_readwrite("quality", &CICP::TReturnInfo::quality);
	}
	// CMetricMapBuilder
	{
		scope s =
			class_<CMetricMapBuilderWrap, boost::noncopyable>(
				"CMetricMapBuilder", no_init)
				.def(
					"getCurrentPose2D", &CMetricMapBuilder_getCurrentPose2D,
					"Returns a copy of the current best pose estimation as a "
					"2D pose.")
				.def(
					"processActionObservation",
					&CMetricMapBuilder::processActionObservation,
					"Process a new action and observations pair to update this "
					"map.")
				.def(
					"getCurrentlyBuiltMap",
					&CMetricMapBuilder_getCurrentlyBuiltMap,
					"Returns \"out_map\" with the set of "
					"\"poses\"-\"sensory-frames\", thus the so far built map.")
				.def(
					"getCurrentlyBuiltMapSize",
					&CMetricMapBuilder::getCurrentlyBuiltMapSize,
					"Returns just how many sensory-frames are stored in the "
					"currently build map.")
				.def(
					"saveCurrentEstimationToImage",
					&CMetricMapBuilder::saveCurrentEstimationToImage,
					"A useful method for debugging: the current map (and/or "
					"poses) estimation is dumped to an image file.")
				.def(
					"clear", &CMetricMapBuilder::clear,
					"Clear all elements of the maps, and reset localization to "
					"(0,0,0deg).")
				.def(
					"enableMapUpdating", &CMetricMapBuilder::enableMapUpdating,
					"Enables or disables the map updating (default state is "
					"enabled).")
				.def(
					"loadCurrentMapFromFile",
					&CMetricMapBuilder::loadCurrentMapFromFile,
					"Load map (mrpt::slam::CSimpleMap) from a \".simplemap\" "
					"file")
				.def(
					"saveCurrentMapToFile",
					&CMetricMapBuilder::saveCurrentMapToFile,
					"Save map (mrpt::slam::CSimpleMap) to a \".simplemap\" "
					"file.");
	}

	// CMetricMapBuilderICP
	{
		scope s =
			class_<
				CMetricMapBuilderICP, bases<CMetricMapBuilder>,
				boost::noncopyable>("CMetricMapBuilderICP", init<>())
				.def(
					"initialize", &CMetricMapBuilderICP_initialize,
					"Initialize the method, starting with a known location PDF "
					"\"x0\"(if supplied, set to nullptr to left unmodified) "
					"and a given fixed, past map.")
				.def_readwrite(
					"ICP_options", &CMetricMapBuilderICP::ICP_options)
				.def_readwrite("ICP_params", &CMetricMapBuilderICP::ICP_params);

		// TConfigParams
		class_<CMetricMapBuilderICP::TConfigParams, bases<CLoadableOptions>>(
			"TConfigParams", init<mrpt::system::VerbosityLevel&>())
			.def_readwrite(
				"matchAgainstTheGrid",
				&CMetricMapBuilderICP::TConfigParams::matchAgainstTheGrid)
			.def_readwrite(
				"insertionLinDistance",
				&CMetricMapBuilderICP::TConfigParams::insertionLinDistance)
			.def_readwrite(
				"insertionAngDistance",
				&CMetricMapBuilderICP::TConfigParams::insertionAngDistance)
			.def_readwrite(
				"localizationLinDistance",
				&CMetricMapBuilderICP::TConfigParams::localizationLinDistance)
			.def_readwrite(
				"localizationAngDistance",
				&CMetricMapBuilderICP::TConfigParams::localizationAngDistance)
			.def_readwrite(
				"minICPgoodnessToAccept",
				&CMetricMapBuilderICP::TConfigParams::minICPgoodnessToAccept)
			.def_readwrite(
				"mapInitializers",
				&CMetricMapBuilderICP::TConfigParams::mapInitializers);
	}

	// CMetricMapBuilderRBPF
	{
		scope s =
			class_<
				CMetricMapBuilderRBPF, bases<CMetricMapBuilder>,
				boost::noncopyable>(
				"CMetricMapBuilderRBPF",
				init<CMetricMapBuilderRBPF::TConstructionOptions>())
				.def(
					"getCurrentPoseEstimation",
					&CMetricMapBuilderRBPF_getCurrentPoseEstimation,
					"Returns a copy of the current best pose estimation as a "
					"pose PDF.")
				.def(
					"drawCurrentEstimationToImage",
					&CMetricMapBuilderRBPF::drawCurrentEstimationToImage)
				.def(
					"initialize", &CMetricMapBuilderRBPF_initialize,
					"Initialize the method, starting with a known location PDF "
					"\"x0\"(if supplied, set to nullptr to left unmodified) "
					"and a given fixed, past map.")
				.def(
					"saveCurrentPathEstimationToTextFile",
					&CMetricMapBuilderRBPF::saveCurrentPathEstimationToTextFile)
				.def(
					"getCurrentJointEntropy",
					&CMetricMapBuilderRBPF::getCurrentJointEntropy)
				.def_readwrite("mapPDF", &CMetricMapBuilderRBPF::mapPDF);

		// TConstructionOptions
		class_<
			CMetricMapBuilderRBPF::TConstructionOptions,
			bases<CLoadableOptions>>("TConstructionOptions", init<>())
			.def_readwrite(
				"insertionLinDistance",
				&CMetricMapBuilderRBPF::TConstructionOptions::
					insertionLinDistance)
			.def_readwrite(
				"insertionAngDistance",
				&CMetricMapBuilderRBPF::TConstructionOptions::
					insertionAngDistance)
			.def_readwrite(
				"localizeLinDistance",
				&CMetricMapBuilderRBPF::TConstructionOptions::
					localizeLinDistance)
			.def_readwrite(
				"localizeAngDistance",
				&CMetricMapBuilderRBPF::TConstructionOptions::
					localizeAngDistance)
			.def_readwrite(
				"PF_options",
				&CMetricMapBuilderRBPF::TConstructionOptions::PF_options)
			.def_readwrite(
				"mapsInitializers",
				&CMetricMapBuilderRBPF::TConstructionOptions::mapsInitializers)
			.def_readwrite(
				"predictionOptions",
				&CMetricMapBuilderRBPF::TConstructionOptions::
					predictionOptions);
	}

	// CRangeBearingKFSLAM2D
	{
		scope s =
			class_<CRangeBearingKFSLAM2D, boost::noncopyable>(
				"CRangeBearingKFSLAM2D", init<>())
				.def(
					"reset", &CRangeBearingKFSLAM2D::reset,
					"Reset the state of the SLAM filter: The map is emptied "
					"and the robot put back to (0,0,0).")
				.def(
					"processActionObservation",
					&CRangeBearingKFSLAM2D::processActionObservation,
					"Process one new action and observations to update the map "
					"and robot pose estimate.")
				.def(
					"getCurrentState", &CRangeBearingKFSLAM2D_getCurrentState,
					"Returns the complete mean and cov.")
				.def(
					"getNumberOfLandmarksInTheMap",
					&CRangeBearingKFSLAM2D::getNumberOfLandmarksInTheMap,
					"Get number of landmarks in the map.")
				.def(
					"getCurrentRobotPose",
					&CRangeBearingKFSLAM2D_getCurrentRobotPose,
					"Returns the mean & 3x3 covariance matrix of the robot 2D "
					"pose.")
				.def(
					"getAs3DObject", &CRangeBearingKFSLAM2D_getAs3DObject,
					"Returns a 3D representation of the landmarks in the map "
					"and the robot 3D position according to the current filter "
					"state.")
				.def(
					"loadOptions", &CRangeBearingKFSLAM2D::loadOptions,
					"Load options from a ini-like file/text.")
				.def(
					"saveMapAndPath2DRepresentationAsMATLABFile",
					&CRangeBearingKFSLAM2D::
						saveMapAndPath2DRepresentationAsMATLABFile,
					CRangeBearingKFSLAM2D_saveMapAndPath2DRepresentationAsMATLABFile_overloads())  //"Save the current state of the filter (robot pose & map) to a MATLAB script which displays all the elements in 2D.")
				.def_readwrite("options", &CRangeBearingKFSLAM2D::options);

		// TConstructionOptions
		class_<CRangeBearingKFSLAM2D::TOptions, bases<CLoadableOptions>>(
			"TOptions", init<>())
			.def_readwrite(
				"stds_Q_no_odo",
				&CRangeBearingKFSLAM2D::TOptions::stds_Q_no_odo)
			.def_readwrite(
				"std_sensor_range",
				&CRangeBearingKFSLAM2D::TOptions::std_sensor_range)
			.def_readwrite(
				"std_sensor_yaw",
				&CRangeBearingKFSLAM2D::TOptions::std_sensor_yaw)
			.def_readwrite(
				"quantiles_3D_representation",
				&CRangeBearingKFSLAM2D::TOptions::quantiles_3D_representation)
			.def_readwrite(
				"create_simplemap",
				&CRangeBearingKFSLAM2D::TOptions::create_simplemap)
			// TODO add data association options
			;
	}

	// TKLDParams
	{
		class_<TKLDParams, bases<CLoadableOptions>>("TKLDParams", init<>())
			.def_readwrite("KLD_binSize_XY", &TKLDParams::KLD_binSize_XY)
			.def_readwrite("KLD_binSize_PHI", &TKLDParams::KLD_binSize_PHI)
			.def_readwrite("KLD_delta", &TKLDParams::KLD_delta)
			.def_readwrite("KLD_epsilon", &TKLDParams::KLD_epsilon)
			.def_readwrite("KLD_minSampleSize", &TKLDParams::KLD_minSampleSize)
			.def_readwrite("KLD_maxSampleSize", &TKLDParams::KLD_maxSampleSize)
			.def_readwrite(
				"KLD_minSamplesPerBin", &TKLDParams::KLD_minSamplesPerBin);
	}

	// TMonteCarloLocalizationParams
	{
		class_<TMonteCarloLocalizationParams>(
			"TMonteCarloLocalizationParams", init<>())
			.def_readwrite(
				"metricMaps", &TMonteCarloLocalizationParams::metricMaps)
			.def_readwrite(
				"KLD_params", &TMonteCarloLocalizationParams::KLD_params)
			.def_readwrite(
				"metricMap", &TMonteCarloLocalizationParams::metricMap);
	}

	// CMonteCarloLocalization2D
	{
		scope s =
			class_<
				CMonteCarloLocalization2D, boost::noncopyable,
				bases<CParticleFilterCapable>>(
				"CMonteCarloLocalization2D", init<optional<size_t>>())
				.def(
					"resetUniformFreeSpace",
					&CMonteCarloLocalization2D::resetUniformFreeSpace,
					CMonteCarloLocalization2D_resetUniformFreeSpace_overloads())
				.def(
					"prediction_and_update_pfStandardProposal",
					&CMonteCarloLocalization2D_prediction_and_update_pfStandardProposal,
					"Update the m_particles, predicting the posterior of robot "
					"pose and map after a movement command.")
				.def(
					"prediction_and_update_pfAuxiliaryPFStandard",
					&CMonteCarloLocalization2D_prediction_and_update_pfAuxiliaryPFStandard,
					"Update the m_particles, predicting the posterior of robot "
					"pose and map after a movement command.")
				.def(
					"prediction_and_update_pfAuxiliaryPFOptimal",
					&CMonteCarloLocalization2D_prediction_and_update_pfAuxiliaryPFOptimal,
					"Update the m_particles, predicting the posterior of robot "
					"pose and map after a movement command.")
				.def(
					"clear", &CMonteCarloLocalization2D::clear,
					"Free all the memory associated to m_particles, and set "
					"the number of parts = 0.")
				.def(
					"copyFrom", &CMonteCarloLocalization2D::copyFrom,
					"Copy operator, translating if necesary (for example, "
					"between m_particles and gaussian representations).")
				.def(
					"resetDeterministic",
					&CMonteCarloLocalization2D::resetDeterministic,
					CMonteCarloLocalization2D_resetDeterministic_overloads())
				.def(
					"resetUniform", &CMonteCarloLocalization2D::resetUniform,
					CMonteCarloLocalization2D_resetUniform_overloads())
				.def(
					"getMean", &CMonteCarloLocalization2D_getMean,
					"Returns an estimate of the pose, (the mean, or "
					"mathematical expectation of the PDF).")
				.def(
					"getCovarianceAndMean",
					&CMonteCarloLocalization2D_getCovarianceAndMean,
					"Returns an estimate of the pose covariance matrix (3x3 "
					"cov matrix) and the mean, both at once.")
				.def(
					"getParticlePose",
					&CMonteCarloLocalization2D::getParticlePose,
					"Returns the pose of the i'th particle.")
				.def(
					"saveToTextFile",
					&CMonteCarloLocalization2D::saveToTextFile,
					"Save PDF's m_particles to a text file.")
				.def(
					"size", &CMonteCarloLocalization2D::size,
					"Get the m_particles count (equivalent to "
					"\"particlesCount\").")
				.def(
					"changeCoordinatesReference",
					&CMonteCarloLocalization2D::changeCoordinatesReference,
					"this = p (+) this.")
				.def(
					"drawSingleSample",
					&CMonteCarloLocalization2D_drawSingleSample,
					"Draws a single sample from the distribution (WARNING: "
					"weights are assumed to be normalized!).")
				.def(self += mrpt::math::TPose2D())
				.def(
					"append", &CMonteCarloLocalization2D::append,
					"Appends (add to the list) a set of m_particles to the "
					"existing ones, and then normalize weights.")
				.def(
					"inverse", &CMonteCarloLocalization2D_inverse,
					"Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF.")
				.def(
					"getMostLikelyParticle",
					&CMonteCarloLocalization2D::getMostLikelyParticle,
					"Returns the particle with the highest weight.")
				.def(
					"getAs3DObject", &CMonteCarloLocalization2D_getAs3DObject,
					"Returns a 3D representation of this PDF (it doesn't clear "
					"the current contents of out_obj, but append new OpenGL "
					"objects to that list).")
				.def(
					"getCovarianceEntropy",
					&CMonteCarloLocalization2D::getCovarianceEntropy,
					"Compute the entropy of the estimated covariance matrix.")
				.def(
					"clearParticles",
					&CMonteCarloLocalization2D::clearParticles,
					"Free the memory of all the particles and reset the array "
					"\"m_particles\" to length zero.")
				.def(
					"writeParticlesToStream",
					&CMonteCarloLocalization2D_writeParticlesToStream,
					"Dumps the sequence of particles and their weights to a "
					"stream (requires T implementing CSerializable).")
				.def(
					"readParticlesFromStream",
					&CMonteCarloLocalization2D_readParticlesFromStream,
					"Reads the sequence of particles and their weights from a "
					"stream (requires T implementing CSerializable).")
				.def(
					"getWeights", &CMonteCarloLocalization2D::getWeights,
					"Returns a vector with the sequence of the logaritmic "
					"weights of all the samples.")
				.def(
					"getW", &CMonteCarloLocalization2D::getW,
					"Access to i'th particle (logarithm) weight, where first "
					"one is index 0.")
				.def(
					"setW", &CMonteCarloLocalization2D::setW,
					"Modifies i'th particle (logarithm) weight, where first "
					"one is index 0.")
				.def(
					"particlesCount",
					&CMonteCarloLocalization2D::particlesCount,
					"Get the m_particles count.")
				.def(
					"normalizeWeights",
					&CMonteCarloLocalization2D_normalizeWeights,
					"Normalize the (logarithmic) weights, such as the maximum "
					"weight is zero.")
				.def(
					"ESS", &CMonteCarloLocalization2D::ESS,
					"Returns the normalized ESS (Estimated Sample Size), in "
					"the range [0,1].")
				.def(
					"performSubstitution",
					&CMonteCarloLocalization2D::performSubstitution,
					"Replaces the old particles by copies determined by the "
					"indexes in \"indx\", performing an efficient copy of the "
					"necesary particles only and allowing the number of "
					"particles to change.")
				.def(
					"prediction_and_update",
					&CMonteCarloLocalization2D_prediction_and_update,
					"Returns the normalized ESS (Estimated Sample Size), in "
					"the range [0,1].")
				.def(
					"performResampling",
					&CMonteCarloLocalization2D::performResampling,
					CMonteCarloLocalization2D_performResampling_overloads())
				.def_readwrite("options", &CMonteCarloLocalization2D::options)
				.def_readwrite(
					"m_particles", &CMonteCarloLocalization2D::m_particles);
	}

	// CMonteCarloLocalization3D
	{
		scope s =
			class_<
				CMonteCarloLocalization3D, boost::noncopyable,
				bases<CParticleFilterCapable>>(
				"CMonteCarloLocalization3D", init<optional<size_t>>())
				.def(
					"prediction_and_update_pfStandardProposal",
					&CMonteCarloLocalization3D_prediction_and_update_pfStandardProposal,
					"Update the m_particles, predicting the posterior of robot "
					"pose and map after a movement command.")
				.def(
					"prediction_and_update_pfAuxiliaryPFStandard",
					&CMonteCarloLocalization3D_prediction_and_update_pfAuxiliaryPFStandard,
					"Update the m_particles, predicting the posterior of robot "
					"pose and map after a movement command.")
				.def(
					"prediction_and_update_pfAuxiliaryPFOptimal",
					&CMonteCarloLocalization3D_prediction_and_update_pfAuxiliaryPFOptimal,
					"Update the m_particles, predicting the posterior of robot "
					"pose and map after a movement command.")
				.def(
					"copyFrom", &CMonteCarloLocalization3D::copyFrom,
					"Copy operator, translating if necesary (for example, "
					"between m_particles and gaussian representations).")
				.def(
					"resetDeterministic",
					&CMonteCarloLocalization3D::resetDeterministic,
					CMonteCarloLocalization2D_resetDeterministic_overloads())
				.def(
					"getMean", &CMonteCarloLocalization3D_getMean,
					"Returns an estimate of the pose, (the mean, or "
					"mathematical expectation of the PDF).")
				.def(
					"getCovarianceAndMean",
					&CMonteCarloLocalization3D_getCovarianceAndMean,
					"Returns an estimate of the pose covariance matrix (3x3 "
					"cov matrix) and the mean, both at once.")
				.def(
					"getParticlePose",
					&CMonteCarloLocalization3D::getParticlePose,
					"Returns the pose of the i'th particle.")
				.def(
					"saveToTextFile",
					&CMonteCarloLocalization3D::saveToTextFile,
					"Save PDF's m_particles to a text file.")
				.def(
					"size", &CMonteCarloLocalization3D::size,
					"Get the m_particles count (equivalent to "
					"\"particlesCount\").")
				.def(
					"changeCoordinatesReference",
					&CMonteCarloLocalization3D::changeCoordinatesReference,
					"this = p (+) this.")
				.def(
					"drawSingleSample",
					&CMonteCarloLocalization3D_drawSingleSample,
					"Draws a single sample from the distribution (WARNING: "
					"weights are assumed to be normalized!).")
				.def(self += CPose3D())
				.def(
					"append", &CMonteCarloLocalization3D::append,
					"Appends (add to the list) a set of m_particles to the "
					"existing ones, and then normalize weights.")
				.def(
					"inverse", &CMonteCarloLocalization3D_inverse,
					"Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF.")
				.def(
					"getMostLikelyParticle",
					&CMonteCarloLocalization3D::getMostLikelyParticle,
					"Returns the particle with the highest weight.")
				.def(
					"getAs3DObject", &CMonteCarloLocalization3D_getAs3DObject,
					"Returns a 3D representation of this PDF (it doesn't clear "
					"the current contents of out_obj, but append new OpenGL "
					"objects to that list).")
				.def(
					"getCovarianceEntropy",
					&CMonteCarloLocalization3D::getCovarianceEntropy,
					"Compute the entropy of the estimated covariance matrix.")
				.def(
					"clearParticles",
					&CMonteCarloLocalization3D::clearParticles,
					"Free the memory of all the particles and reset the array "
					"\"m_particles\" to length zero.")
				.def(
					"writeParticlesToStream",
					&CMonteCarloLocalization3D_writeParticlesToStream,
					"Dumps the sequence of particles and their weights to a "
					"stream (requires T implementing CSerializable).")
				.def(
					"readParticlesFromStream",
					&CMonteCarloLocalization3D_readParticlesFromStream,
					"Reads the sequence of particles and their weights from a "
					"stream (requires T implementing CSerializable).")
				.def(
					"getWeights", &CMonteCarloLocalization3D::getWeights,
					"Returns a vector with the sequence of the logaritmic "
					"weights of all the samples.")
				.def(
					"getW", &CMonteCarloLocalization3D::getW,
					"Access to i'th particle (logarithm) weight, where first "
					"one is index 0.")
				.def(
					"setW", &CMonteCarloLocalization3D::setW,
					"Modifies i'th particle (logarithm) weight, where first "
					"one is index 0.")
				.def(
					"particlesCount",
					&CMonteCarloLocalization3D::particlesCount,
					"Get the m_particles count.")
				.def(
					"normalizeWeights",
					&CMonteCarloLocalization3D_normalizeWeights,
					"Normalize the (logarithmic) weights, such as the maximum "
					"weight is zero.")
				.def(
					"ESS", &CMonteCarloLocalization3D::ESS,
					"Returns the normalized ESS (Estimated Sample Size), in "
					"the range [0,1].")
				.def(
					"performSubstitution",
					&CMonteCarloLocalization3D::performSubstitution,
					"Replaces the old particles by copies determined by the "
					"indexes in \"indx\", performing an efficient copy of the "
					"necesary particles only and allowing the number of "
					"particles to change.")
				.def(
					"prediction_and_update",
					&CMonteCarloLocalization3D_prediction_and_update,
					"Returns the normalized ESS (Estimated Sample Size), in "
					"the range [0,1].")
				.def(
					"performResampling",
					&CMonteCarloLocalization3D::performResampling,
					CMonteCarloLocalization2D_performResampling_overloads())
				.def_readwrite("options", &CMonteCarloLocalization3D::options)
				.def_readwrite(
					"m_particles", &CMonteCarloLocalization3D::m_particles);
	}
}
