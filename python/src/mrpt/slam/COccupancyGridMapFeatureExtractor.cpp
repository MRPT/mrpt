#include <ios>
#include <iterator>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/containers/CDynamicGrid.h>
#include <mrpt/img/CImage.h>
#include <mrpt/maps/CLandmarksMap.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/TMetricMapInitializer.h>
#include <mrpt/maps/metric_map_types.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationRange.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/poses/CPoint2DPDFGaussian.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DQuatPDFGaussian.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/slam/CGridMapAligner.h>
#include <mrpt/slam/CMetricMapsAlignmentAlgorithm.h>
#include <mrpt/slam/COccupancyGridMapFeatureExtractor.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/mrptEvent.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/typemeta/static_string.h>
#include <mrpt/vision/CFeatureExtraction.h>
#include <mrpt/vision/types.h>
#include <optional>
#include <ostream>
#include <sstream> // __str__
#include <streambuf>
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

// mrpt::slam::COccupancyGridMapFeatureExtractor file:mrpt/slam/COccupancyGridMapFeatureExtractor.h line:31
struct PyCallBack_mrpt_slam_COccupancyGridMapFeatureExtractor : public mrpt::slam::COccupancyGridMapFeatureExtractor {
	using mrpt::slam::COccupancyGridMapFeatureExtractor::COccupancyGridMapFeatureExtractor;

	void OnEvent(const class mrpt::system::mrptEvent & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::COccupancyGridMapFeatureExtractor *>(this), "OnEvent");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COccupancyGridMapFeatureExtractor::OnEvent(a0);
	}
};

// mrpt::slam::CGridMapAligner file:mrpt/slam/CGridMapAligner.h line:39
struct PyCallBack_mrpt_slam_CGridMapAligner : public mrpt::slam::CGridMapAligner {
	using mrpt::slam::CGridMapAligner::CGridMapAligner;

};

// mrpt::slam::CGridMapAligner::TConfigParams file:mrpt/slam/CGridMapAligner.h line:54
struct PyCallBack_mrpt_slam_CGridMapAligner_TConfigParams : public mrpt::slam::CGridMapAligner::TConfigParams {
	using mrpt::slam::CGridMapAligner::TConfigParams::TConfigParams;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CGridMapAligner::TConfigParams *>(this), "loadFromConfigFile");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CGridMapAligner::TConfigParams *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CLoadableOptions::saveToConfigFile(a0, a1);
	}
};

void bind_mrpt_slam_COccupancyGridMapFeatureExtractor(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::slam::COccupancyGridMapFeatureExtractor file:mrpt/slam/COccupancyGridMapFeatureExtractor.h line:31
		pybind11::class_<mrpt::slam::COccupancyGridMapFeatureExtractor, std::shared_ptr<mrpt::slam::COccupancyGridMapFeatureExtractor>, PyCallBack_mrpt_slam_COccupancyGridMapFeatureExtractor, mrpt::system::CObserver> cl(M("mrpt::slam"), "COccupancyGridMapFeatureExtractor", "A class for detecting features from occupancy grid maps.\n   The main method is \"COccupancyGridMapFeatureExtractor::extractFeatures()\",\n which makes use\n    of an advanced cache mechanism to avoid redoing work when applied several\n times on the same\n    occupancy grid maps (unless they changed in the meanwhile).\n\n  For an uncached version (which is a static method that can be called\n without instantiating COccupancyGridMapFeatureExtractor)\n  see COccupancyGridMapFeatureExtractor::uncached_extractFeatures()\n\n \n\n ");
		cl.def( pybind11::init( [](PyCallBack_mrpt_slam_COccupancyGridMapFeatureExtractor const &o){ return new PyCallBack_mrpt_slam_COccupancyGridMapFeatureExtractor(o); } ) );
		cl.def( pybind11::init( [](mrpt::slam::COccupancyGridMapFeatureExtractor const &o){ return new mrpt::slam::COccupancyGridMapFeatureExtractor(o); } ) );
		cl.def( pybind11::init( [](){ return new mrpt::slam::COccupancyGridMapFeatureExtractor(); }, [](){ return new PyCallBack_mrpt_slam_COccupancyGridMapFeatureExtractor(); } ) );
		cl.def("extractFeatures", (void (mrpt::slam::COccupancyGridMapFeatureExtractor::*)(const class mrpt::maps::COccupancyGridMap2D &, class mrpt::maps::CLandmarksMap &, size_t, const enum mrpt::vision::TDescriptorType, const struct mrpt::vision::CFeatureExtraction::TOptions &)) &mrpt::slam::COccupancyGridMapFeatureExtractor::extractFeatures, "Computes a set of distinctive landmarks from an occupancy grid, and\n store them (previous content is not erased!) into the given landmarks\n map.\n   Landmarks type can be any declared in\n mrpt::vision::CFeatureExtraction::TOptions\n\n \n See the paper \"...\"\n \n\n uncached_extractFeatures\n\nC++: mrpt::slam::COccupancyGridMapFeatureExtractor::extractFeatures(const class mrpt::maps::COccupancyGridMap2D &, class mrpt::maps::CLandmarksMap &, size_t, const enum mrpt::vision::TDescriptorType, const struct mrpt::vision::CFeatureExtraction::TOptions &) --> void", pybind11::arg("grid"), pybind11::arg("outMap"), pybind11::arg("number_of_features"), pybind11::arg("descriptors"), pybind11::arg("feat_options"));
		cl.def_static("uncached_extractFeatures", (void (*)(const class mrpt::maps::COccupancyGridMap2D &, class mrpt::maps::CLandmarksMap &, size_t, const enum mrpt::vision::TDescriptorType, const struct mrpt::vision::CFeatureExtraction::TOptions &)) &mrpt::slam::COccupancyGridMapFeatureExtractor::uncached_extractFeatures, "Computes a set of distinctive landmarks from an occupancy grid, and\n store them (previous content is not erased!) into the given landmarks\n map.\n   Landmarks type can be any declared in\n mrpt::vision::CFeatureExtraction::TOptions\n\n \n See the paper \"...\"\n \n\n uncached_extractFeatures\n\nC++: mrpt::slam::COccupancyGridMapFeatureExtractor::uncached_extractFeatures(const class mrpt::maps::COccupancyGridMap2D &, class mrpt::maps::CLandmarksMap &, size_t, const enum mrpt::vision::TDescriptorType, const struct mrpt::vision::CFeatureExtraction::TOptions &) --> void", pybind11::arg("grid"), pybind11::arg("outMap"), pybind11::arg("number_of_features"), pybind11::arg("descriptors"), pybind11::arg("feat_options"));
		cl.def("assign", (class mrpt::slam::COccupancyGridMapFeatureExtractor & (mrpt::slam::COccupancyGridMapFeatureExtractor::*)(const class mrpt::slam::COccupancyGridMapFeatureExtractor &)) &mrpt::slam::COccupancyGridMapFeatureExtractor::operator=, "C++: mrpt::slam::COccupancyGridMapFeatureExtractor::operator=(const class mrpt::slam::COccupancyGridMapFeatureExtractor &) --> class mrpt::slam::COccupancyGridMapFeatureExtractor &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::slam::CGridMapAligner file:mrpt/slam/CGridMapAligner.h line:39
		pybind11::class_<mrpt::slam::CGridMapAligner, std::shared_ptr<mrpt::slam::CGridMapAligner>, PyCallBack_mrpt_slam_CGridMapAligner, mrpt::slam::CMetricMapsAlignmentAlgorithm> cl(M("mrpt::slam"), "CGridMapAligner", "A class for aligning two multi-metric maps (with an occupancy grid maps and\n a points map, at least) based on features extraction and matching.\n The matching pose is returned as a Sum of Gaussians (poses::CPosePDFSOG).\n\n  This class can use three methods (see options.methodSelection):\n   - amCorrelation: \"Brute-force\" correlation of the two maps over a\n 2D+orientation grid of possible 2D poses.\n   - amRobustMatch: Detection of features + RANSAC matching\n   - amModifiedRANSAC: Detection of features + modified multi-hypothesis\n RANSAC matching as described in \n\n See CGridMapAligner::Align for more instructions.\n\n \n CMetricMapsAlignmentAlgorithm\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::slam::CGridMapAligner(); }, [](){ return new PyCallBack_mrpt_slam_CGridMapAligner(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_slam_CGridMapAligner const &o){ return new PyCallBack_mrpt_slam_CGridMapAligner(o); } ) );
		cl.def( pybind11::init( [](mrpt::slam::CGridMapAligner const &o){ return new mrpt::slam::CGridMapAligner(o); } ) );

		pybind11::enum_<mrpt::slam::CGridMapAligner::TAlignerMethod>(cl, "TAlignerMethod", pybind11::arithmetic(), "The type for selecting the grid-map alignment algorithm.")
			.value("amRobustMatch", mrpt::slam::CGridMapAligner::amRobustMatch)
			.value("amCorrelation", mrpt::slam::CGridMapAligner::amCorrelation)
			.value("amModifiedRANSAC", mrpt::slam::CGridMapAligner::amModifiedRANSAC)
			.export_values();

		cl.def_readwrite("options", &mrpt::slam::CGridMapAligner::options);
		cl.def("assign", (class mrpt::slam::CGridMapAligner & (mrpt::slam::CGridMapAligner::*)(const class mrpt::slam::CGridMapAligner &)) &mrpt::slam::CGridMapAligner::operator=, "C++: mrpt::slam::CGridMapAligner::operator=(const class mrpt::slam::CGridMapAligner &) --> class mrpt::slam::CGridMapAligner &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::slam::CGridMapAligner::TConfigParams file:mrpt/slam/CGridMapAligner.h line:54
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::slam::CGridMapAligner::TConfigParams, std::shared_ptr<mrpt::slam::CGridMapAligner::TConfigParams>, PyCallBack_mrpt_slam_CGridMapAligner_TConfigParams, mrpt::config::CLoadableOptions> cl(enclosing_class, "TConfigParams", "The ICP algorithm configuration data");
			cl.def( pybind11::init( [](){ return new mrpt::slam::CGridMapAligner::TConfigParams(); }, [](){ return new PyCallBack_mrpt_slam_CGridMapAligner_TConfigParams(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_slam_CGridMapAligner_TConfigParams const &o){ return new PyCallBack_mrpt_slam_CGridMapAligner_TConfigParams(o); } ) );
			cl.def( pybind11::init( [](mrpt::slam::CGridMapAligner::TConfigParams const &o){ return new mrpt::slam::CGridMapAligner::TConfigParams(o); } ) );
			cl.def_readwrite("methodSelection", &mrpt::slam::CGridMapAligner::TConfigParams::methodSelection);
			cl.def_readwrite("feature_descriptor", &mrpt::slam::CGridMapAligner::TConfigParams::feature_descriptor);
			cl.def_readwrite("feature_detector_options", &mrpt::slam::CGridMapAligner::TConfigParams::feature_detector_options);
			cl.def_readwrite("ransac_minSetSizeRatio", &mrpt::slam::CGridMapAligner::TConfigParams::ransac_minSetSizeRatio);
			cl.def_readwrite("ransac_SOG_sigma_m", &mrpt::slam::CGridMapAligner::TConfigParams::ransac_SOG_sigma_m);
			cl.def_readwrite("ransac_mahalanobisDistanceThreshold", &mrpt::slam::CGridMapAligner::TConfigParams::ransac_mahalanobisDistanceThreshold);
			cl.def_readwrite("ransac_chi2_quantile", &mrpt::slam::CGridMapAligner::TConfigParams::ransac_chi2_quantile);
			cl.def_readwrite("ransac_prob_good_inliers", &mrpt::slam::CGridMapAligner::TConfigParams::ransac_prob_good_inliers);
			cl.def_readwrite("featsPerSquareMeter", &mrpt::slam::CGridMapAligner::TConfigParams::featsPerSquareMeter);
			cl.def_readwrite("threshold_max", &mrpt::slam::CGridMapAligner::TConfigParams::threshold_max);
			cl.def_readwrite("threshold_delta", &mrpt::slam::CGridMapAligner::TConfigParams::threshold_delta);
			cl.def_readwrite("min_ICP_goodness", &mrpt::slam::CGridMapAligner::TConfigParams::min_ICP_goodness);
			cl.def_readwrite("max_ICP_mahadist", &mrpt::slam::CGridMapAligner::TConfigParams::max_ICP_mahadist);
			cl.def_readwrite("maxKLd_for_merge", &mrpt::slam::CGridMapAligner::TConfigParams::maxKLd_for_merge);
			cl.def_readwrite("save_feat_coors", &mrpt::slam::CGridMapAligner::TConfigParams::save_feat_coors);
			cl.def_readwrite("debug_save_map_pairs", &mrpt::slam::CGridMapAligner::TConfigParams::debug_save_map_pairs);
			cl.def("loadFromConfigFile", (void (mrpt::slam::CGridMapAligner::TConfigParams::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::slam::CGridMapAligner::TConfigParams::loadFromConfigFile, "C++: mrpt::slam::CGridMapAligner::TConfigParams::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
			cl.def("assign", (class mrpt::slam::CGridMapAligner::TConfigParams & (mrpt::slam::CGridMapAligner::TConfigParams::*)(const class mrpt::slam::CGridMapAligner::TConfigParams &)) &mrpt::slam::CGridMapAligner::TConfigParams::operator=, "C++: mrpt::slam::CGridMapAligner::TConfigParams::operator=(const class mrpt::slam::CGridMapAligner::TConfigParams &) --> class mrpt::slam::CGridMapAligner::TConfigParams &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::slam::CGridMapAligner::TReturnInfo file:mrpt/slam/CGridMapAligner.h line:128
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::slam::CGridMapAligner::TReturnInfo, std::shared_ptr<mrpt::slam::CGridMapAligner::TReturnInfo>, mrpt::slam::TMetricMapAlignmentResult> cl(enclosing_class, "TReturnInfo", "The ICP algorithm return information.");
			cl.def( pybind11::init( [](){ return new mrpt::slam::CGridMapAligner::TReturnInfo(); } ) );
			cl.def( pybind11::init( [](mrpt::slam::CGridMapAligner::TReturnInfo const &o){ return new mrpt::slam::CGridMapAligner::TReturnInfo(o); } ) );
			cl.def_readwrite("goodness", &mrpt::slam::CGridMapAligner::TReturnInfo::goodness);
			cl.def_readwrite("noRobustEstimation", &mrpt::slam::CGridMapAligner::TReturnInfo::noRobustEstimation);
			cl.def_readwrite("sog1", &mrpt::slam::CGridMapAligner::TReturnInfo::sog1);
			cl.def_readwrite("sog2", &mrpt::slam::CGridMapAligner::TReturnInfo::sog2);
			cl.def_readwrite("sog3", &mrpt::slam::CGridMapAligner::TReturnInfo::sog3);
			cl.def_readwrite("landmarks_map1", &mrpt::slam::CGridMapAligner::TReturnInfo::landmarks_map1);
			cl.def_readwrite("landmarks_map2", &mrpt::slam::CGridMapAligner::TReturnInfo::landmarks_map2);
			cl.def_readwrite("correspondences", &mrpt::slam::CGridMapAligner::TReturnInfo::correspondences);
			cl.def_readwrite("correspondences_dists_maha", &mrpt::slam::CGridMapAligner::TReturnInfo::correspondences_dists_maha);
			cl.def_readwrite("icp_goodness_all_sog_modes", &mrpt::slam::CGridMapAligner::TReturnInfo::icp_goodness_all_sog_modes);
			cl.def("assign", (struct mrpt::slam::CGridMapAligner::TReturnInfo & (mrpt::slam::CGridMapAligner::TReturnInfo::*)(const struct mrpt::slam::CGridMapAligner::TReturnInfo &)) &mrpt::slam::CGridMapAligner::TReturnInfo::operator=, "C++: mrpt::slam::CGridMapAligner::TReturnInfo::operator=(const struct mrpt::slam::CGridMapAligner::TReturnInfo &) --> struct mrpt::slam::CGridMapAligner::TReturnInfo &", pybind11::return_value_policy::automatic, pybind11::arg(""));

			{ // mrpt::slam::CGridMapAligner::TReturnInfo::TPairPlusDistance file:mrpt/slam/CGridMapAligner.h line:161
				auto & enclosing_class = cl;
				pybind11::class_<mrpt::slam::CGridMapAligner::TReturnInfo::TPairPlusDistance, std::shared_ptr<mrpt::slam::CGridMapAligner::TReturnInfo::TPairPlusDistance>> cl(enclosing_class, "TPairPlusDistance", "");
				cl.def( pybind11::init<size_t, size_t, float>(), pybind11::arg("i1"), pybind11::arg("i2"), pybind11::arg("d") );

				cl.def_readwrite("idx_this", &mrpt::slam::CGridMapAligner::TReturnInfo::TPairPlusDistance::idx_this);
				cl.def_readwrite("idx_other", &mrpt::slam::CGridMapAligner::TReturnInfo::TPairPlusDistance::idx_other);
				cl.def_readwrite("dist", &mrpt::slam::CGridMapAligner::TReturnInfo::TPairPlusDistance::dist);
			}

		}

	}
}
