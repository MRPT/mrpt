#include <iterator>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/metric_map_types.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint2DPDFGaussian.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPose3DQuatPDFGaussian.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/slam/CICP.h>
#include <mrpt/slam/CMetricMapsAlignmentAlgorithm.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <sstream> // __str__
#include <string>
#include <tuple>
#include <vector>

#include <functional>
#include <pybind11/pybind11.h>
#include <string>
#include <pybind11/stl.h>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

// mrpt::slam::CICP file:mrpt/slam/CICP.h line:64
struct PyCallBack_mrpt_slam_CICP : public mrpt::slam::CICP {
	using mrpt::slam::CICP::CICP;

};

// mrpt::slam::CICP::TConfigParams file:mrpt/slam/CICP.h line:69
struct PyCallBack_mrpt_slam_CICP_TConfigParams : public mrpt::slam::CICP::TConfigParams {
	using mrpt::slam::CICP::TConfigParams::TConfigParams;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CICP::TConfigParams *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TConfigParams::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CICP::TConfigParams *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TConfigParams::saveToConfigFile(a0, a1);
	}
};

void bind_mrpt_slam_CMetricMapsAlignmentAlgorithm(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::slam::TMetricMapAlignmentResult file:mrpt/slam/CMetricMapsAlignmentAlgorithm.h line:25
		pybind11::class_<mrpt::slam::TMetricMapAlignmentResult, std::shared_ptr<mrpt::slam::TMetricMapAlignmentResult>> cl(M("mrpt::slam"), "TMetricMapAlignmentResult", "Used as base class for other result structures of each particular algorithm\n in CMetricMapsAlignmentAlgorithm derived classes.\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::slam::TMetricMapAlignmentResult(); } ) );
		cl.def( pybind11::init( [](mrpt::slam::TMetricMapAlignmentResult const &o){ return new mrpt::slam::TMetricMapAlignmentResult(o); } ) );
		cl.def_readwrite("executionTime", &mrpt::slam::TMetricMapAlignmentResult::executionTime);
		cl.def("assign", (struct mrpt::slam::TMetricMapAlignmentResult & (mrpt::slam::TMetricMapAlignmentResult::*)(const struct mrpt::slam::TMetricMapAlignmentResult &)) &mrpt::slam::TMetricMapAlignmentResult::operator=, "C++: mrpt::slam::TMetricMapAlignmentResult::operator=(const struct mrpt::slam::TMetricMapAlignmentResult &) --> struct mrpt::slam::TMetricMapAlignmentResult &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::slam::CMetricMapsAlignmentAlgorithm file:mrpt/slam/CMetricMapsAlignmentAlgorithm.h line:40
		pybind11::class_<mrpt::slam::CMetricMapsAlignmentAlgorithm, std::shared_ptr<mrpt::slam::CMetricMapsAlignmentAlgorithm>> cl(M("mrpt::slam"), "CMetricMapsAlignmentAlgorithm", "A base class for any algorithm able of maps alignment. There are two methods\n   depending on an PDF or a single 2D Pose value is available as initial guess\n for the methods.\n\n \n CPointsMap\n \n\n\n ");
		cl.def("assign", (class mrpt::slam::CMetricMapsAlignmentAlgorithm & (mrpt::slam::CMetricMapsAlignmentAlgorithm::*)(const class mrpt::slam::CMetricMapsAlignmentAlgorithm &)) &mrpt::slam::CMetricMapsAlignmentAlgorithm::operator=, "C++: mrpt::slam::CMetricMapsAlignmentAlgorithm::operator=(const class mrpt::slam::CMetricMapsAlignmentAlgorithm &) --> class mrpt::slam::CMetricMapsAlignmentAlgorithm &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	// mrpt::slam::TICPAlgorithm file:mrpt/slam/CICP.h line:19
	pybind11::enum_<mrpt::slam::TICPAlgorithm>(M("mrpt::slam"), "TICPAlgorithm", pybind11::arithmetic(), "The ICP algorithm selection, used in mrpt::slam::CICP::options  \n\n mrpt_slam_grp  ")
		.value("icpClassic", mrpt::slam::icpClassic)
		.value("icpLevenbergMarquardt", mrpt::slam::icpLevenbergMarquardt)
		.export_values();

;

	// mrpt::slam::TICPCovarianceMethod file:mrpt/slam/CICP.h line:27
	pybind11::enum_<mrpt::slam::TICPCovarianceMethod>(M("mrpt::slam"), "TICPCovarianceMethod", pybind11::arithmetic(), "ICP covariance estimation methods, used in mrpt::slam::CICP::options\n \n")
		.value("icpCovLinealMSE", mrpt::slam::icpCovLinealMSE)
		.value("icpCovFiniteDifferences", mrpt::slam::icpCovFiniteDifferences)
		.export_values();

;

	{ // mrpt::slam::CICP file:mrpt/slam/CICP.h line:64
		pybind11::class_<mrpt::slam::CICP, std::shared_ptr<mrpt::slam::CICP>, PyCallBack_mrpt_slam_CICP, mrpt::slam::CMetricMapsAlignmentAlgorithm> cl(M("mrpt::slam"), "CICP", "Several implementations of ICP (Iterative closest point) algorithms for\n aligning two point maps or a point map wrt a grid map.\n\n  CICP::AlignPDF() or CICP::Align() are the two main entry points of the\n algorithm.\n\n  To choose among existing ICP algorithms or customizing their parameters, see\n CICP::TConfigParams and the member \n\n  There exists an extension of the original ICP algorithm that provides\n multihypotheses-support for the correspondences, and which generates a\n Sum-of-Gaussians (SOG)\n    PDF as output. See mrpt::tfest::se2_l2_robust()\n\n For further details on the implemented methods, check the web:\n   https://www.mrpt.org/Iterative_Closest_Point_(ICP)_and_other_matching_algorithms\n\n  For a paper explaining some of the basic equations, see for example:\n   J. Martinez, J. Gonzalez, J. Morales, A. Mandow, A. Garcia-Cerezo,\n   \"Mobile robot motion estimation by 2D scan matching with genetic and\n iterative closest point algorithms\",\n    Journal of Field Robotics, vol. 23, no. 1, pp. 21-34, 2006. (\n http://babel.isa.uma.es/~jlblanco/papers/martinez2006gil.pdf )\n\n \n CMetricMapsAlignmentAlgorithm\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::slam::CICP(); }, [](){ return new PyCallBack_mrpt_slam_CICP(); } ) );
		cl.def( pybind11::init<const class mrpt::slam::CICP::TConfigParams &>(), pybind11::arg("icpParams") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_slam_CICP const &o){ return new PyCallBack_mrpt_slam_CICP(o); } ) );
		cl.def( pybind11::init( [](mrpt::slam::CICP const &o){ return new mrpt::slam::CICP(o); } ) );
		cl.def_readwrite("options", &mrpt::slam::CICP::options);
		cl.def("assign", (class mrpt::slam::CICP & (mrpt::slam::CICP::*)(const class mrpt::slam::CICP &)) &mrpt::slam::CICP::operator=, "C++: mrpt::slam::CICP::operator=(const class mrpt::slam::CICP &) --> class mrpt::slam::CICP &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::slam::CICP::TConfigParams file:mrpt/slam/CICP.h line:69
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::slam::CICP::TConfigParams, std::shared_ptr<mrpt::slam::CICP::TConfigParams>, PyCallBack_mrpt_slam_CICP_TConfigParams, mrpt::config::CLoadableOptions> cl(enclosing_class, "TConfigParams", "The ICP algorithm configuration data");
			cl.def( pybind11::init( [](PyCallBack_mrpt_slam_CICP_TConfigParams const &o){ return new PyCallBack_mrpt_slam_CICP_TConfigParams(o); } ) );
			cl.def( pybind11::init( [](mrpt::slam::CICP::TConfigParams const &o){ return new mrpt::slam::CICP::TConfigParams(o); } ) );
			cl.def( pybind11::init( [](){ return new mrpt::slam::CICP::TConfigParams(); }, [](){ return new PyCallBack_mrpt_slam_CICP_TConfigParams(); } ) );
			cl.def_readwrite("ICP_algorithm", &mrpt::slam::CICP::TConfigParams::ICP_algorithm);
			cl.def_readwrite("ICP_covariance_method", &mrpt::slam::CICP::TConfigParams::ICP_covariance_method);
			cl.def_readwrite("onlyUniqueRobust", &mrpt::slam::CICP::TConfigParams::onlyUniqueRobust);
			cl.def_readwrite("maxIterations", &mrpt::slam::CICP::TConfigParams::maxIterations);
			cl.def_readwrite("minAbsStep_trans", &mrpt::slam::CICP::TConfigParams::minAbsStep_trans);
			cl.def_readwrite("minAbsStep_rot", &mrpt::slam::CICP::TConfigParams::minAbsStep_rot);
			cl.def_readwrite("thresholdDist", &mrpt::slam::CICP::TConfigParams::thresholdDist);
			cl.def_readwrite("thresholdAng", &mrpt::slam::CICP::TConfigParams::thresholdAng);
			cl.def_readwrite("ALFA", &mrpt::slam::CICP::TConfigParams::ALFA);
			cl.def_readwrite("smallestThresholdDist", &mrpt::slam::CICP::TConfigParams::smallestThresholdDist);
			cl.def_readwrite("covariance_varPoints", &mrpt::slam::CICP::TConfigParams::covariance_varPoints);
			cl.def_readwrite("doRANSAC", &mrpt::slam::CICP::TConfigParams::doRANSAC);
			cl.def_readwrite("ransac_minSetSize", &mrpt::slam::CICP::TConfigParams::ransac_minSetSize);
			cl.def_readwrite("ransac_maxSetSize", &mrpt::slam::CICP::TConfigParams::ransac_maxSetSize);
			cl.def_readwrite("ransac_nSimulations", &mrpt::slam::CICP::TConfigParams::ransac_nSimulations);
			cl.def_readwrite("ransac_mahalanobisDistanceThreshold", &mrpt::slam::CICP::TConfigParams::ransac_mahalanobisDistanceThreshold);
			cl.def_readwrite("normalizationStd", &mrpt::slam::CICP::TConfigParams::normalizationStd);
			cl.def_readwrite("ransac_fuseByCorrsMatch", &mrpt::slam::CICP::TConfigParams::ransac_fuseByCorrsMatch);
			cl.def_readwrite("ransac_fuseMaxDiffXY", &mrpt::slam::CICP::TConfigParams::ransac_fuseMaxDiffXY);
			cl.def_readwrite("ransac_fuseMaxDiffPhi", &mrpt::slam::CICP::TConfigParams::ransac_fuseMaxDiffPhi);
			cl.def_readwrite("kernel_rho", &mrpt::slam::CICP::TConfigParams::kernel_rho);
			cl.def_readwrite("use_kernel", &mrpt::slam::CICP::TConfigParams::use_kernel);
			cl.def_readwrite("Axy_aprox_derivatives", &mrpt::slam::CICP::TConfigParams::Axy_aprox_derivatives);
			cl.def_readwrite("LM_initial_lambda", &mrpt::slam::CICP::TConfigParams::LM_initial_lambda);
			cl.def_readwrite("skip_cov_calculation", &mrpt::slam::CICP::TConfigParams::skip_cov_calculation);
			cl.def_readwrite("skip_quality_calculation", &mrpt::slam::CICP::TConfigParams::skip_quality_calculation);
			cl.def_readwrite("corresponding_points_decimation", &mrpt::slam::CICP::TConfigParams::corresponding_points_decimation);
			cl.def("loadFromConfigFile", (void (mrpt::slam::CICP::TConfigParams::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::slam::CICP::TConfigParams::loadFromConfigFile, "C++: mrpt::slam::CICP::TConfigParams::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
			cl.def("saveToConfigFile", (void (mrpt::slam::CICP::TConfigParams::*)(class mrpt::config::CConfigFileBase &, const std::string &) const) &mrpt::slam::CICP::TConfigParams::saveToConfigFile, "C++: mrpt::slam::CICP::TConfigParams::saveToConfigFile(class mrpt::config::CConfigFileBase &, const std::string &) const --> void", pybind11::arg("c"), pybind11::arg("s"));
			cl.def("assign", (class mrpt::slam::CICP::TConfigParams & (mrpt::slam::CICP::TConfigParams::*)(const class mrpt::slam::CICP::TConfigParams &)) &mrpt::slam::CICP::TConfigParams::operator=, "C++: mrpt::slam::CICP::TConfigParams::operator=(const class mrpt::slam::CICP::TConfigParams &) --> class mrpt::slam::CICP::TConfigParams &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::slam::CICP::TReturnInfo file:mrpt/slam/CICP.h line:186
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::slam::CICP::TReturnInfo, std::shared_ptr<mrpt::slam::CICP::TReturnInfo>, mrpt::slam::TMetricMapAlignmentResult> cl(enclosing_class, "TReturnInfo", "The ICP algorithm return information");
			cl.def( pybind11::init( [](){ return new mrpt::slam::CICP::TReturnInfo(); } ) );
			cl.def( pybind11::init( [](mrpt::slam::CICP::TReturnInfo const &o){ return new mrpt::slam::CICP::TReturnInfo(o); } ) );
			cl.def_readwrite("nIterations", &mrpt::slam::CICP::TReturnInfo::nIterations);
			cl.def_readwrite("goodness", &mrpt::slam::CICP::TReturnInfo::goodness);
			cl.def_readwrite("quality", &mrpt::slam::CICP::TReturnInfo::quality);
			cl.def("assign", (struct mrpt::slam::CICP::TReturnInfo & (mrpt::slam::CICP::TReturnInfo::*)(const struct mrpt::slam::CICP::TReturnInfo &)) &mrpt::slam::CICP::TReturnInfo::operator=, "C++: mrpt::slam::CICP::TReturnInfo::operator=(const struct mrpt::slam::CICP::TReturnInfo &) --> struct mrpt::slam::CICP::TReturnInfo &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
}
