#include <functional>
#include <ios>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/img/TPixelCoord.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/vision/types.h>
#include <ostream>
#include <sstream> // __str__
#include <streambuf>
#include <string>
#include <utility>
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

// mrpt::vision::TStereoSystemParams file:mrpt/vision/types.h line:234
struct PyCallBack_mrpt_vision_TStereoSystemParams : public mrpt::vision::TStereoSystemParams {
	using mrpt::vision::TStereoSystemParams::TStereoSystemParams;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::vision::TStereoSystemParams *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TStereoSystemParams::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::vision::TStereoSystemParams *>(this), "saveToConfigFile");
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

// mrpt::vision::TMatchingOptions file:mrpt/vision/types.h line:340
struct PyCallBack_mrpt_vision_TMatchingOptions : public mrpt::vision::TMatchingOptions {
	using mrpt::vision::TMatchingOptions::TMatchingOptions;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::vision::TMatchingOptions *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TMatchingOptions::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::vision::TMatchingOptions *>(this), "saveToConfigFile");
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

void bind_mrpt_vision_types(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::vision::TKeyPointMethod file:mrpt/vision/types.h line:45
	pybind11::enum_<mrpt::vision::TKeyPointMethod>(M("mrpt::vision"), "TKeyPointMethod", pybind11::arithmetic(), "Types of key point detectors ")
		.value("featNotDefined", mrpt::vision::featNotDefined)
		.value("featKLT", mrpt::vision::featKLT)
		.value("featHarris", mrpt::vision::featHarris)
		.value("featSIFT", mrpt::vision::featSIFT)
		.value("featSURF", mrpt::vision::featSURF)
		.value("featBeacon", mrpt::vision::featBeacon)
		.value("featFAST", mrpt::vision::featFAST)
		.value("featORB", mrpt::vision::featORB)
		.value("featAKAZE", mrpt::vision::featAKAZE)
		.value("featLSD", mrpt::vision::featLSD)
		.export_values();

;

	// mrpt::vision::TDescriptorType file:mrpt/vision/types.h line:77
	pybind11::enum_<mrpt::vision::TDescriptorType>(M("mrpt::vision"), "TDescriptorType", pybind11::arithmetic(), "The bitwise OR combination of values of TDescriptorType are used in\n CFeatureExtraction::computeDescriptors to indicate which descriptors are to\n be computed for features.")
		.value("descAny", mrpt::vision::descAny)
		.value("descSIFT", mrpt::vision::descSIFT)
		.value("descSURF", mrpt::vision::descSURF)
		.value("descSpinImages", mrpt::vision::descSpinImages)
		.value("descPolarImages", mrpt::vision::descPolarImages)
		.value("descLogPolarImages", mrpt::vision::descLogPolarImages)
		.value("descORB", mrpt::vision::descORB)
		.value("descBLD", mrpt::vision::descBLD)
		.value("descLATCH", mrpt::vision::descLATCH)
		.export_values();

;

	// mrpt::vision::TFeatureTrackStatus file:mrpt/vision/types.h line:98
	pybind11::enum_<mrpt::vision::TFeatureTrackStatus>(M("mrpt::vision"), "TFeatureTrackStatus", pybind11::arithmetic(), "")
		.value("status_IDLE", mrpt::vision::status_IDLE)
		.value("status_TRACKED", mrpt::vision::status_TRACKED)
		.value("status_OOB", mrpt::vision::status_OOB)
		.value("status_LOST", mrpt::vision::status_LOST)
		.export_values();

;

	{ // mrpt::vision::TFeatureObservation file:mrpt/vision/types.h line:119
		pybind11::class_<mrpt::vision::TFeatureObservation, std::shared_ptr<mrpt::vision::TFeatureObservation>> cl(M("mrpt::vision"), "TFeatureObservation", "One feature observation entry, used within sequences with\n TSequenceFeatureObservations ");
		cl.def( pybind11::init( [](){ return new mrpt::vision::TFeatureObservation(); } ) );
		cl.def( pybind11::init<const unsigned long, const unsigned long, const struct mrpt::img::TPixelCoordf &>(), pybind11::arg("_id_feature"), pybind11::arg("_id_frame"), pybind11::arg("_px") );

		cl.def( pybind11::init( [](mrpt::vision::TFeatureObservation const &o){ return new mrpt::vision::TFeatureObservation(o); } ) );
		cl.def_readwrite("id_feature", &mrpt::vision::TFeatureObservation::id_feature);
		cl.def_readwrite("id_frame", &mrpt::vision::TFeatureObservation::id_frame);
		cl.def_readwrite("px", &mrpt::vision::TFeatureObservation::px);
		cl.def("assign", (struct mrpt::vision::TFeatureObservation & (mrpt::vision::TFeatureObservation::*)(const struct mrpt::vision::TFeatureObservation &)) &mrpt::vision::TFeatureObservation::operator=, "C++: mrpt::vision::TFeatureObservation::operator=(const struct mrpt::vision::TFeatureObservation &) --> struct mrpt::vision::TFeatureObservation &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::vision::TRelativeFeaturePos file:mrpt/vision/types.h line:142
		pybind11::class_<mrpt::vision::TRelativeFeaturePos, std::shared_ptr<mrpt::vision::TRelativeFeaturePos>> cl(M("mrpt::vision"), "TRelativeFeaturePos", "One relative feature observation entry, used with some relative\n bundle-adjustment functions.");
		cl.def( pybind11::init( [](){ return new mrpt::vision::TRelativeFeaturePos(); } ) );
		cl.def( pybind11::init<const unsigned long, const struct mrpt::math::TPoint3D_<double> &>(), pybind11::arg("_id_frame_base"), pybind11::arg("_pos") );

		cl.def_readwrite("id_frame_base", &mrpt::vision::TRelativeFeaturePos::id_frame_base);
		cl.def_readwrite("pos", &mrpt::vision::TRelativeFeaturePos::pos);
	}
	{ // mrpt::vision::TSequenceFeatureObservations file:mrpt/vision/types.h line:170
		pybind11::class_<mrpt::vision::TSequenceFeatureObservations, std::shared_ptr<mrpt::vision::TSequenceFeatureObservations>> cl(M("mrpt::vision"), "TSequenceFeatureObservations", "A complete sequence of observations of features from different camera frames\n (poses).\n  This structure is the input to some (Bundle-adjustment) methods in\n mrpt::vision\n  \n\n Pixel coordinates can be either \"raw\" or \"undistorted\". Read the doc\n of functions handling this structure to see what they expect.\n  \n\n mrpt::vision::bundle_adj_full");
		cl.def( pybind11::init( [](){ return new mrpt::vision::TSequenceFeatureObservations(); } ) );
		cl.def( pybind11::init<size_t>(), pybind11::arg("size") );

		cl.def( pybind11::init( [](mrpt::vision::TSequenceFeatureObservations const &o){ return new mrpt::vision::TSequenceFeatureObservations(o); } ) );
		cl.def("saveToTextFile", [](mrpt::vision::TSequenceFeatureObservations const &o, const std::string & a0) -> void { return o.saveToTextFile(a0); }, "", pybind11::arg("filName"));
		cl.def("saveToTextFile", (void (mrpt::vision::TSequenceFeatureObservations::*)(const std::string &, bool) const) &mrpt::vision::TSequenceFeatureObservations::saveToTextFile, "Saves all entries to a text file, with each line having this format:\n #FRAME_ID  #FEAT_ID  #PIXEL_X  #PIXEL_Y\n The file is self-descripting, since the first line contains a comment\n line (starting with '%') explaining the format.\n Generated files can be loaded from MATLAB.\n \n\n loadFromTextFile \n std::exception On I/O error  \n\nC++: mrpt::vision::TSequenceFeatureObservations::saveToTextFile(const std::string &, bool) const --> void", pybind11::arg("filName"), pybind11::arg("skipFirstCommentLine"));
		cl.def("loadFromTextFile", (void (mrpt::vision::TSequenceFeatureObservations::*)(const std::string &)) &mrpt::vision::TSequenceFeatureObservations::loadFromTextFile, "Load from a text file, in the format described in \n \n\n std::exception On I/O or format error \n\nC++: mrpt::vision::TSequenceFeatureObservations::loadFromTextFile(const std::string &) --> void", pybind11::arg("filName"));
		cl.def("removeFewObservedFeatures", [](mrpt::vision::TSequenceFeatureObservations &o) -> size_t { return o.removeFewObservedFeatures(); }, "");
		cl.def("removeFewObservedFeatures", (size_t (mrpt::vision::TSequenceFeatureObservations::*)(size_t)) &mrpt::vision::TSequenceFeatureObservations::removeFewObservedFeatures, "Remove all those features that don't have a minimum number of\n observations from different camera frame IDs.\n \n\n the number of erased entries.\n \n\n After calling this you may want to call  \n\nC++: mrpt::vision::TSequenceFeatureObservations::removeFewObservedFeatures(size_t) --> size_t", pybind11::arg("minNumObservations"));
		cl.def("decimateCameraFrames", (void (mrpt::vision::TSequenceFeatureObservations::*)(size_t)) &mrpt::vision::TSequenceFeatureObservations::decimateCameraFrames, "Remove all but one out of  camera frame IDs from the\n list (eg: from N camera pose IDs at return there will be just\n N/decimate_ratio)\n The algorithm first builds a sorted list of frame IDs, then keep the\n lowest ID, remove the next \"decimate_ratio-1\", and so on.\n \n\n After calling this you may want to call  \n\nC++: mrpt::vision::TSequenceFeatureObservations::decimateCameraFrames(size_t) --> void", pybind11::arg("decimate_ratio"));
		cl.def("assign", (struct mrpt::vision::TSequenceFeatureObservations & (mrpt::vision::TSequenceFeatureObservations::*)(const struct mrpt::vision::TSequenceFeatureObservations &)) &mrpt::vision::TSequenceFeatureObservations::operator=, "C++: mrpt::vision::TSequenceFeatureObservations::operator=(const struct mrpt::vision::TSequenceFeatureObservations &) --> struct mrpt::vision::TSequenceFeatureObservations &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::vision::TStereoSystemParams file:mrpt/vision/types.h line:234
		pybind11::class_<mrpt::vision::TStereoSystemParams, std::shared_ptr<mrpt::vision::TStereoSystemParams>, PyCallBack_mrpt_vision_TStereoSystemParams, mrpt::config::CLoadableOptions> cl(M("mrpt::vision"), "TStereoSystemParams", "Parameters associated to a stereo system");
		cl.def( pybind11::init( [](){ return new mrpt::vision::TStereoSystemParams(); }, [](){ return new PyCallBack_mrpt_vision_TStereoSystemParams(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_vision_TStereoSystemParams const &o){ return new PyCallBack_mrpt_vision_TStereoSystemParams(o); } ) );
		cl.def( pybind11::init( [](mrpt::vision::TStereoSystemParams const &o){ return new mrpt::vision::TStereoSystemParams(o); } ) );

		pybind11::enum_<mrpt::vision::TStereoSystemParams::TUnc_Prop_Method>(cl, "TUnc_Prop_Method", pybind11::arithmetic(), "Method for propagating the feature's image coordinate uncertainty into\n 3D space. Default value: Prop_Linear")
			.value("Prop_Linear", mrpt::vision::TStereoSystemParams::Prop_Linear)
			.value("Prop_UT", mrpt::vision::TStereoSystemParams::Prop_UT)
			.value("Prop_SUT", mrpt::vision::TStereoSystemParams::Prop_SUT)
			.export_values();

		cl.def_readwrite("uncPropagation", &mrpt::vision::TStereoSystemParams::uncPropagation);
		cl.def_readwrite("F", &mrpt::vision::TStereoSystemParams::F);
		cl.def_readwrite("K", &mrpt::vision::TStereoSystemParams::K);
		cl.def_readwrite("baseline", &mrpt::vision::TStereoSystemParams::baseline);
		cl.def_readwrite("stdPixel", &mrpt::vision::TStereoSystemParams::stdPixel);
		cl.def_readwrite("stdDisp", &mrpt::vision::TStereoSystemParams::stdDisp);
		cl.def_readwrite("maxZ", &mrpt::vision::TStereoSystemParams::maxZ);
		cl.def_readwrite("minZ", &mrpt::vision::TStereoSystemParams::minZ);
		cl.def_readwrite("maxY", &mrpt::vision::TStereoSystemParams::maxY);
		cl.def_readwrite("factor_k", &mrpt::vision::TStereoSystemParams::factor_k);
		cl.def_readwrite("factor_a", &mrpt::vision::TStereoSystemParams::factor_a);
		cl.def_readwrite("factor_b", &mrpt::vision::TStereoSystemParams::factor_b);
		cl.def("loadFromConfigFile", (void (mrpt::vision::TStereoSystemParams::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::vision::TStereoSystemParams::loadFromConfigFile, "C++: mrpt::vision::TStereoSystemParams::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
		cl.def("assign", (struct mrpt::vision::TStereoSystemParams & (mrpt::vision::TStereoSystemParams::*)(const struct mrpt::vision::TStereoSystemParams &)) &mrpt::vision::TStereoSystemParams::operator=, "C++: mrpt::vision::TStereoSystemParams::operator=(const struct mrpt::vision::TStereoSystemParams &) --> struct mrpt::vision::TStereoSystemParams &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::vision::TROI file:mrpt/vision/types.h line:306
		pybind11::class_<mrpt::vision::TROI, std::shared_ptr<mrpt::vision::TROI>> cl(M("mrpt::vision"), "TROI", "A structure for storing a 3D ROI");
		cl.def( pybind11::init( [](){ return new mrpt::vision::TROI(); } ) );
		cl.def( pybind11::init<float, float, float, float, float, float>(), pybind11::arg("x1"), pybind11::arg("x2"), pybind11::arg("y1"), pybind11::arg("y2"), pybind11::arg("z1"), pybind11::arg("z2") );

		cl.def_readwrite("xMin", &mrpt::vision::TROI::xMin);
		cl.def_readwrite("xMax", &mrpt::vision::TROI::xMax);
		cl.def_readwrite("yMin", &mrpt::vision::TROI::yMin);
		cl.def_readwrite("yMax", &mrpt::vision::TROI::yMax);
		cl.def_readwrite("zMin", &mrpt::vision::TROI::zMin);
		cl.def_readwrite("zMax", &mrpt::vision::TROI::zMax);
	}
	{ // mrpt::vision::TImageROI file:mrpt/vision/types.h line:326
		pybind11::class_<mrpt::vision::TImageROI, std::shared_ptr<mrpt::vision::TImageROI>> cl(M("mrpt::vision"), "TImageROI", "A structure for defining a ROI within an image");
		cl.def( pybind11::init( [](){ return new mrpt::vision::TImageROI(); } ) );
		cl.def( pybind11::init<size_t, size_t, size_t, size_t>(), pybind11::arg("x1"), pybind11::arg("x2"), pybind11::arg("y1"), pybind11::arg("y2") );

		cl.def( pybind11::init( [](mrpt::vision::TImageROI const &o){ return new mrpt::vision::TImageROI(o); } ) );
		cl.def_readwrite("xMin", &mrpt::vision::TImageROI::xMin);
		cl.def_readwrite("xMax", &mrpt::vision::TImageROI::xMax);
		cl.def_readwrite("yMin", &mrpt::vision::TImageROI::yMin);
		cl.def_readwrite("yMax", &mrpt::vision::TImageROI::yMax);
	}
	{ // mrpt::vision::TMatchingOptions file:mrpt/vision/types.h line:340
		pybind11::class_<mrpt::vision::TMatchingOptions, std::shared_ptr<mrpt::vision::TMatchingOptions>, PyCallBack_mrpt_vision_TMatchingOptions, mrpt::config::CLoadableOptions> cl(M("mrpt::vision"), "TMatchingOptions", "A structure containing options for the matching");
		cl.def( pybind11::init( [](){ return new mrpt::vision::TMatchingOptions(); }, [](){ return new PyCallBack_mrpt_vision_TMatchingOptions(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_vision_TMatchingOptions const &o){ return new PyCallBack_mrpt_vision_TMatchingOptions(o); } ) );
		cl.def( pybind11::init( [](mrpt::vision::TMatchingOptions const &o){ return new mrpt::vision::TMatchingOptions(o); } ) );

		pybind11::enum_<mrpt::vision::TMatchingOptions::TMatchingMethod>(cl, "TMatchingMethod", pybind11::arithmetic(), "Method for propagating the feature's image coordinate uncertainty into\n 3D space. Default value: Prop_Linear")
			.value("mmCorrelation", mrpt::vision::TMatchingOptions::mmCorrelation)
			.value("mmDescriptorSIFT", mrpt::vision::TMatchingOptions::mmDescriptorSIFT)
			.value("mmDescriptorSURF", mrpt::vision::TMatchingOptions::mmDescriptorSURF)
			.value("mmSAD", mrpt::vision::TMatchingOptions::mmSAD)
			.value("mmDescriptorORB", mrpt::vision::TMatchingOptions::mmDescriptorORB)
			.export_values();

		cl.def_readwrite("useEpipolarRestriction", &mrpt::vision::TMatchingOptions::useEpipolarRestriction);
		cl.def_readwrite("hasFundamentalMatrix", &mrpt::vision::TMatchingOptions::hasFundamentalMatrix);
		cl.def_readwrite("parallelOpticalAxis", &mrpt::vision::TMatchingOptions::parallelOpticalAxis);
		cl.def_readwrite("useXRestriction", &mrpt::vision::TMatchingOptions::useXRestriction);
		cl.def_readwrite("addMatches", &mrpt::vision::TMatchingOptions::addMatches);
		cl.def_readwrite("useDisparityLimits", &mrpt::vision::TMatchingOptions::useDisparityLimits);
		cl.def_readwrite("enable_robust_1to1_match", &mrpt::vision::TMatchingOptions::enable_robust_1to1_match);
		cl.def_readwrite("min_disp", &mrpt::vision::TMatchingOptions::min_disp);
		cl.def_readwrite("max_disp", &mrpt::vision::TMatchingOptions::max_disp);
		cl.def_readwrite("F", &mrpt::vision::TMatchingOptions::F);
		cl.def_readwrite("matching_method", &mrpt::vision::TMatchingOptions::matching_method);
		cl.def_readwrite("epipolar_TH", &mrpt::vision::TMatchingOptions::epipolar_TH);
		cl.def_readwrite("maxEDD_TH", &mrpt::vision::TMatchingOptions::maxEDD_TH);
		cl.def_readwrite("EDD_RATIO", &mrpt::vision::TMatchingOptions::EDD_RATIO);
		cl.def_readwrite("minCC_TH", &mrpt::vision::TMatchingOptions::minCC_TH);
		cl.def_readwrite("minDCC_TH", &mrpt::vision::TMatchingOptions::minDCC_TH);
		cl.def_readwrite("rCC_TH", &mrpt::vision::TMatchingOptions::rCC_TH);
		cl.def_readwrite("maxEDSD_TH", &mrpt::vision::TMatchingOptions::maxEDSD_TH);
		cl.def_readwrite("EDSD_RATIO", &mrpt::vision::TMatchingOptions::EDSD_RATIO);
		cl.def_readwrite("maxSAD_TH", &mrpt::vision::TMatchingOptions::maxSAD_TH);
		cl.def_readwrite("SAD_RATIO", &mrpt::vision::TMatchingOptions::SAD_RATIO);
		cl.def_readwrite("maxORB_dist", &mrpt::vision::TMatchingOptions::maxORB_dist);
		cl.def_readwrite("estimateDepth", &mrpt::vision::TMatchingOptions::estimateDepth);
		cl.def_readwrite("maxDepthThreshold", &mrpt::vision::TMatchingOptions::maxDepthThreshold);
		cl.def("loadFromConfigFile", (void (mrpt::vision::TMatchingOptions::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::vision::TMatchingOptions::loadFromConfigFile, "C++: mrpt::vision::TMatchingOptions::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
		cl.def("__eq__", (bool (mrpt::vision::TMatchingOptions::*)(const struct mrpt::vision::TMatchingOptions &) const) &mrpt::vision::TMatchingOptions::operator==, "C++: mrpt::vision::TMatchingOptions::operator==(const struct mrpt::vision::TMatchingOptions &) const --> bool", pybind11::arg("o"));
		cl.def("assign", (void (mrpt::vision::TMatchingOptions::*)(const struct mrpt::vision::TMatchingOptions &)) &mrpt::vision::TMatchingOptions::operator=, "C++: mrpt::vision::TMatchingOptions::operator=(const struct mrpt::vision::TMatchingOptions &) --> void", pybind11::arg("o"));
	}
}
