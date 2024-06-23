#include <ios>
#include <iterator>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/vision/types.h>
#include <ostream>
#include <sstream> // __str__
#include <streambuf>
#include <string>
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

// mrpt::vision::TMultiResDescMatchOptions file:mrpt/vision/types.h line:518
struct PyCallBack_mrpt_vision_TMultiResDescMatchOptions : public mrpt::vision::TMultiResDescMatchOptions {
	using mrpt::vision::TMultiResDescMatchOptions::TMultiResDescMatchOptions;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::vision::TMultiResDescMatchOptions *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TMultiResDescMatchOptions::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::vision::TMultiResDescMatchOptions *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TMultiResDescMatchOptions::saveToConfigFile(a0, a1);
	}
};

// mrpt::vision::TMultiResDescOptions file:mrpt/vision/types.h line:603
struct PyCallBack_mrpt_vision_TMultiResDescOptions : public mrpt::vision::TMultiResDescOptions {
	using mrpt::vision::TMultiResDescOptions::TMultiResDescOptions;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::vision::TMultiResDescOptions *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TMultiResDescOptions::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::vision::TMultiResDescOptions *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TMultiResDescOptions::saveToConfigFile(a0, a1);
	}
};

void bind_mrpt_vision_types_1(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::vision::TMultiResMatchingOutput file:mrpt/vision/types.h line:495
		pybind11::class_<mrpt::vision::TMultiResMatchingOutput, std::shared_ptr<mrpt::vision::TMultiResMatchingOutput>> cl(M("mrpt::vision"), "TMultiResMatchingOutput", "Struct containing the output after matching multi-resolution SIFT-like\n descriptors");
		cl.def( pybind11::init( [](){ return new mrpt::vision::TMultiResMatchingOutput(); } ) );
		cl.def( pybind11::init( [](mrpt::vision::TMultiResMatchingOutput const &o){ return new mrpt::vision::TMultiResMatchingOutput(o); } ) );
		cl.def_readwrite("nMatches", &mrpt::vision::TMultiResMatchingOutput::nMatches);
		cl.def_readwrite("firstListCorrespondences", &mrpt::vision::TMultiResMatchingOutput::firstListCorrespondences);
		cl.def_readwrite("secondListCorrespondences", &mrpt::vision::TMultiResMatchingOutput::secondListCorrespondences);
		cl.def_readwrite("firstListFoundScales", &mrpt::vision::TMultiResMatchingOutput::firstListFoundScales);
		cl.def_readwrite("firstListDistance", &mrpt::vision::TMultiResMatchingOutput::firstListDistance);
		cl.def("assign", (struct mrpt::vision::TMultiResMatchingOutput & (mrpt::vision::TMultiResMatchingOutput::*)(const struct mrpt::vision::TMultiResMatchingOutput &)) &mrpt::vision::TMultiResMatchingOutput::operator=, "C++: mrpt::vision::TMultiResMatchingOutput::operator=(const struct mrpt::vision::TMultiResMatchingOutput &) --> struct mrpt::vision::TMultiResMatchingOutput &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::vision::TMultiResDescMatchOptions file:mrpt/vision/types.h line:518
		pybind11::class_<mrpt::vision::TMultiResDescMatchOptions, std::shared_ptr<mrpt::vision::TMultiResDescMatchOptions>, PyCallBack_mrpt_vision_TMultiResDescMatchOptions, mrpt::config::CLoadableOptions> cl(M("mrpt::vision"), "TMultiResDescMatchOptions", "Struct containing the options when matching multi-resolution SIFT-like\n descriptors");
		cl.def( pybind11::init( [](){ return new mrpt::vision::TMultiResDescMatchOptions(); }, [](){ return new PyCallBack_mrpt_vision_TMultiResDescMatchOptions(); } ) );
		cl.def( pybind11::init<bool, double, bool, double, double, const unsigned int &, const unsigned int &, const unsigned int &, const unsigned int &, int, int, int, int, int>(), pybind11::arg("_useOriFilter"), pybind11::arg("_oriThreshold"), pybind11::arg("_useDepthFilter"), pybind11::arg("_th"), pybind11::arg("_th2"), pybind11::arg("_lwscl1"), pybind11::arg("_lwscl2"), pybind11::arg("_hwscl1"), pybind11::arg("_hwscl2"), pybind11::arg("_searchAreaSize"), pybind11::arg("_lsth"), pybind11::arg("_tsth"), pybind11::arg("_minFeaturesToFind"), pybind11::arg("_minFeaturesToBeLost") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_vision_TMultiResDescMatchOptions const &o){ return new PyCallBack_mrpt_vision_TMultiResDescMatchOptions(o); } ) );
		cl.def( pybind11::init( [](mrpt::vision::TMultiResDescMatchOptions const &o){ return new mrpt::vision::TMultiResDescMatchOptions(o); } ) );
		cl.def_readwrite("useOriFilter", &mrpt::vision::TMultiResDescMatchOptions::useOriFilter);
		cl.def_readwrite("oriThreshold", &mrpt::vision::TMultiResDescMatchOptions::oriThreshold);
		cl.def_readwrite("useDepthFilter", &mrpt::vision::TMultiResDescMatchOptions::useDepthFilter);
		cl.def_readwrite("matchingThreshold", &mrpt::vision::TMultiResDescMatchOptions::matchingThreshold);
		cl.def_readwrite("matchingRatioThreshold", &mrpt::vision::TMultiResDescMatchOptions::matchingRatioThreshold);
		cl.def_readwrite("lowScl1", &mrpt::vision::TMultiResDescMatchOptions::lowScl1);
		cl.def_readwrite("lowScl2", &mrpt::vision::TMultiResDescMatchOptions::lowScl2);
		cl.def_readwrite("highScl1", &mrpt::vision::TMultiResDescMatchOptions::highScl1);
		cl.def_readwrite("highScl2", &mrpt::vision::TMultiResDescMatchOptions::highScl2);
		cl.def_readwrite("searchAreaSize", &mrpt::vision::TMultiResDescMatchOptions::searchAreaSize);
		cl.def_readwrite("lastSeenThreshold", &mrpt::vision::TMultiResDescMatchOptions::lastSeenThreshold);
		cl.def_readwrite("timesSeenThreshold", &mrpt::vision::TMultiResDescMatchOptions::timesSeenThreshold);
		cl.def_readwrite("minFeaturesToFind", &mrpt::vision::TMultiResDescMatchOptions::minFeaturesToFind);
		cl.def_readwrite("minFeaturesToBeLost", &mrpt::vision::TMultiResDescMatchOptions::minFeaturesToBeLost);
		cl.def("loadFromConfigFile", (void (mrpt::vision::TMultiResDescMatchOptions::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::vision::TMultiResDescMatchOptions::loadFromConfigFile, "C++: mrpt::vision::TMultiResDescMatchOptions::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("cfg"), pybind11::arg("section"));
		cl.def("saveToConfigFile", (void (mrpt::vision::TMultiResDescMatchOptions::*)(class mrpt::config::CConfigFileBase &, const std::string &) const) &mrpt::vision::TMultiResDescMatchOptions::saveToConfigFile, "C++: mrpt::vision::TMultiResDescMatchOptions::saveToConfigFile(class mrpt::config::CConfigFileBase &, const std::string &) const --> void", pybind11::arg("cfg"), pybind11::arg("section"));
		cl.def("assign", (struct mrpt::vision::TMultiResDescMatchOptions & (mrpt::vision::TMultiResDescMatchOptions::*)(const struct mrpt::vision::TMultiResDescMatchOptions &)) &mrpt::vision::TMultiResDescMatchOptions::operator=, "C++: mrpt::vision::TMultiResDescMatchOptions::operator=(const struct mrpt::vision::TMultiResDescMatchOptions &) --> struct mrpt::vision::TMultiResDescMatchOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::vision::TMultiResDescOptions file:mrpt/vision/types.h line:603
		pybind11::class_<mrpt::vision::TMultiResDescOptions, std::shared_ptr<mrpt::vision::TMultiResDescOptions>, PyCallBack_mrpt_vision_TMultiResDescOptions, mrpt::config::CLoadableOptions> cl(M("mrpt::vision"), "TMultiResDescOptions", "Struct containing the options when computing the multi-resolution SIFT-like\n descriptors");
		cl.def( pybind11::init( [](){ return new mrpt::vision::TMultiResDescOptions(); }, [](){ return new PyCallBack_mrpt_vision_TMultiResDescOptions(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_vision_TMultiResDescOptions const &o){ return new PyCallBack_mrpt_vision_TMultiResDescOptions(o); } ) );
		cl.def( pybind11::init( [](mrpt::vision::TMultiResDescOptions const &o){ return new mrpt::vision::TMultiResDescOptions(o); } ) );
		cl.def_readwrite("basePSize", &mrpt::vision::TMultiResDescOptions::basePSize);
		cl.def_readwrite("scales", &mrpt::vision::TMultiResDescOptions::scales);
		cl.def_readwrite("comLScl", &mrpt::vision::TMultiResDescOptions::comLScl);
		cl.def_readwrite("comHScl", &mrpt::vision::TMultiResDescOptions::comHScl);
		cl.def_readwrite("sg1", &mrpt::vision::TMultiResDescOptions::sg1);
		cl.def_readwrite("sg2", &mrpt::vision::TMultiResDescOptions::sg2);
		cl.def_readwrite("sg3", &mrpt::vision::TMultiResDescOptions::sg3);
		cl.def_readwrite("computeDepth", &mrpt::vision::TMultiResDescOptions::computeDepth);
		cl.def_readwrite("blurImage", &mrpt::vision::TMultiResDescOptions::blurImage);
		cl.def_readwrite("fx", &mrpt::vision::TMultiResDescOptions::fx);
		cl.def_readwrite("cx", &mrpt::vision::TMultiResDescOptions::cx);
		cl.def_readwrite("cy", &mrpt::vision::TMultiResDescOptions::cy);
		cl.def_readwrite("baseline", &mrpt::vision::TMultiResDescOptions::baseline);
		cl.def_readwrite("computeHashCoeffs", &mrpt::vision::TMultiResDescOptions::computeHashCoeffs);
		cl.def_readwrite("cropValue", &mrpt::vision::TMultiResDescOptions::cropValue);
		cl.def("loadFromConfigFile", (void (mrpt::vision::TMultiResDescOptions::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::vision::TMultiResDescOptions::loadFromConfigFile, "C++: mrpt::vision::TMultiResDescOptions::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
		cl.def("saveToConfigFile", (void (mrpt::vision::TMultiResDescOptions::*)(class mrpt::config::CConfigFileBase &, const std::string &) const) &mrpt::vision::TMultiResDescOptions::saveToConfigFile, "C++: mrpt::vision::TMultiResDescOptions::saveToConfigFile(class mrpt::config::CConfigFileBase &, const std::string &) const --> void", pybind11::arg("cfg"), pybind11::arg("section"));
		cl.def("assign", (struct mrpt::vision::TMultiResDescOptions & (mrpt::vision::TMultiResDescOptions::*)(const struct mrpt::vision::TMultiResDescOptions &)) &mrpt::vision::TMultiResDescOptions::operator=, "C++: mrpt::vision::TMultiResDescOptions::operator=(const struct mrpt::vision::TMultiResDescOptions &) --> struct mrpt::vision::TMultiResDescOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
