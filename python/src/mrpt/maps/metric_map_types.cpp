#include <ios>
#include <iterator>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/maps/TMetricMapInitializer.h>
#include <mrpt/maps/metric_map_types.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/typemeta/static_string.h>
#include <ostream>
#include <sstream> // __str__
#include <streambuf>
#include <string>
#include <variant>
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

// mrpt::maps::TMapGenericParams file:mrpt/maps/metric_map_types.h line:77
struct PyCallBack_mrpt_maps_TMapGenericParams : public mrpt::maps::TMapGenericParams {
	using mrpt::maps::TMapGenericParams::TMapGenericParams;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::TMapGenericParams *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return TMapGenericParams::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::TMapGenericParams *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return TMapGenericParams::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::TMapGenericParams *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return TMapGenericParams::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::TMapGenericParams *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TMapGenericParams::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::TMapGenericParams *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TMapGenericParams::serializeFrom(a0, a1);
	}
	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::TMapGenericParams *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TMapGenericParams::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::TMapGenericParams *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TMapGenericParams::saveToConfigFile(a0, a1);
	}
};

// mrpt::maps::TSetOfMetricMapInitializers file:mrpt/maps/TMetricMapInitializer.h line:88
struct PyCallBack_mrpt_maps_TSetOfMetricMapInitializers : public mrpt::maps::TSetOfMetricMapInitializers {
	using mrpt::maps::TSetOfMetricMapInitializers::TSetOfMetricMapInitializers;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::TSetOfMetricMapInitializers *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TSetOfMetricMapInitializers::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::TSetOfMetricMapInitializers *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TSetOfMetricMapInitializers::saveToConfigFile(a0, a1);
	}
};

void bind_mrpt_maps_metric_map_types(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::maps::TMatchingParams file:mrpt/maps/metric_map_types.h line:20
		pybind11::class_<mrpt::maps::TMatchingParams, std::shared_ptr<mrpt::maps::TMatchingParams>> cl(M("mrpt::maps"), "TMatchingParams", "Parameters for the determination of matchings between point clouds, etc. \n\n CMetricMap::determineMatching2D, CMetricMap::determineMatching3D ");
		cl.def( pybind11::init( [](){ return new mrpt::maps::TMatchingParams(); } ) );
		cl.def_readwrite("maxDistForCorrespondence", &mrpt::maps::TMatchingParams::maxDistForCorrespondence);
		cl.def_readwrite("maxAngularDistForCorrespondence", &mrpt::maps::TMatchingParams::maxAngularDistForCorrespondence);
		cl.def_readwrite("onlyKeepTheClosest", &mrpt::maps::TMatchingParams::onlyKeepTheClosest);
		cl.def_readwrite("onlyUniqueRobust", &mrpt::maps::TMatchingParams::onlyUniqueRobust);
		cl.def_readwrite("decimation_other_map_points", &mrpt::maps::TMatchingParams::decimation_other_map_points);
		cl.def_readwrite("offset_other_map_points", &mrpt::maps::TMatchingParams::offset_other_map_points);
		cl.def_readwrite("angularDistPivotPoint", &mrpt::maps::TMatchingParams::angularDistPivotPoint);
	}
	{ // mrpt::maps::TMatchingExtraResults file:mrpt/maps/metric_map_types.h line:51
		pybind11::class_<mrpt::maps::TMatchingExtraResults, std::shared_ptr<mrpt::maps::TMatchingExtraResults>> cl(M("mrpt::maps"), "TMatchingExtraResults", "Additional results from the determination of matchings between point clouds,\n etc., apart from the pairings themselves \n\n CMetricMap::determineMatching2D,\n CMetricMap::determineMatching3D ");
		cl.def( pybind11::init( [](){ return new mrpt::maps::TMatchingExtraResults(); } ) );
		cl.def_readwrite("correspondencesRatio", &mrpt::maps::TMatchingExtraResults::correspondencesRatio);
		cl.def_readwrite("sumSqrDist", &mrpt::maps::TMatchingExtraResults::sumSqrDist);
	}
	{ // mrpt::maps::TMatchingRatioParams file:mrpt/maps/metric_map_types.h line:64
		pybind11::class_<mrpt::maps::TMatchingRatioParams, std::shared_ptr<mrpt::maps::TMatchingRatioParams>> cl(M("mrpt::maps"), "TMatchingRatioParams", "Parameters for CMetricMap::compute3DMatchingRatio() ");
		cl.def( pybind11::init( [](){ return new mrpt::maps::TMatchingRatioParams(); } ) );
		cl.def( pybind11::init( [](mrpt::maps::TMatchingRatioParams const &o){ return new mrpt::maps::TMatchingRatioParams(o); } ) );
		cl.def_readwrite("maxDistForCorr", &mrpt::maps::TMatchingRatioParams::maxDistForCorr);
		cl.def_readwrite("maxMahaDistForCorr", &mrpt::maps::TMatchingRatioParams::maxMahaDistForCorr);
		cl.def("assign", (struct mrpt::maps::TMatchingRatioParams & (mrpt::maps::TMatchingRatioParams::*)(const struct mrpt::maps::TMatchingRatioParams &)) &mrpt::maps::TMatchingRatioParams::operator=, "C++: mrpt::maps::TMatchingRatioParams::operator=(const struct mrpt::maps::TMatchingRatioParams &) --> struct mrpt::maps::TMatchingRatioParams &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::maps::TMapGenericParams file:mrpt/maps/metric_map_types.h line:77
		pybind11::class_<mrpt::maps::TMapGenericParams, std::shared_ptr<mrpt::maps::TMapGenericParams>, PyCallBack_mrpt_maps_TMapGenericParams, mrpt::config::CLoadableOptions, mrpt::serialization::CSerializable> cl(M("mrpt::maps"), "TMapGenericParams", "Common params to all maps derived from mrpt::maps::CMetricMap  ");
		cl.def( pybind11::init( [](PyCallBack_mrpt_maps_TMapGenericParams const &o){ return new PyCallBack_mrpt_maps_TMapGenericParams(o); } ) );
		cl.def( pybind11::init( [](mrpt::maps::TMapGenericParams const &o){ return new mrpt::maps::TMapGenericParams(o); } ) );
		cl.def( pybind11::init( [](){ return new mrpt::maps::TMapGenericParams(); }, [](){ return new PyCallBack_mrpt_maps_TMapGenericParams(); } ) );
		cl.def_readwrite("enableSaveAs3DObject", &mrpt::maps::TMapGenericParams::enableSaveAs3DObject);
		cl.def_readwrite("enableObservationLikelihood", &mrpt::maps::TMapGenericParams::enableObservationLikelihood);
		cl.def_readwrite("enableObservationInsertion", &mrpt::maps::TMapGenericParams::enableObservationInsertion);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::maps::TMapGenericParams::GetRuntimeClassIdStatic, "C++: mrpt::maps::TMapGenericParams::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::maps::TMapGenericParams::*)() const) &mrpt::maps::TMapGenericParams::GetRuntimeClass, "C++: mrpt::maps::TMapGenericParams::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::maps::TMapGenericParams::*)() const) &mrpt::maps::TMapGenericParams::clone, "C++: mrpt::maps::TMapGenericParams::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::maps::TMapGenericParams::CreateObject, "C++: mrpt::maps::TMapGenericParams::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("loadFromConfigFile", (void (mrpt::maps::TMapGenericParams::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::maps::TMapGenericParams::loadFromConfigFile, "C++: mrpt::maps::TMapGenericParams::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("sectionNamePrefix"));
		cl.def("saveToConfigFile", (void (mrpt::maps::TMapGenericParams::*)(class mrpt::config::CConfigFileBase &, const std::string &) const) &mrpt::maps::TMapGenericParams::saveToConfigFile, "C++: mrpt::maps::TMapGenericParams::saveToConfigFile(class mrpt::config::CConfigFileBase &, const std::string &) const --> void", pybind11::arg("target"), pybind11::arg("section"));
		cl.def("assign", (class mrpt::maps::TMapGenericParams & (mrpt::maps::TMapGenericParams::*)(const class mrpt::maps::TMapGenericParams &)) &mrpt::maps::TMapGenericParams::operator=, "C++: mrpt::maps::TMapGenericParams::operator=(const class mrpt::maps::TMapGenericParams &) --> class mrpt::maps::TMapGenericParams &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::maps::TSetOfMetricMapInitializers file:mrpt/maps/TMetricMapInitializer.h line:88
		pybind11::class_<mrpt::maps::TSetOfMetricMapInitializers, std::shared_ptr<mrpt::maps::TSetOfMetricMapInitializers>, PyCallBack_mrpt_maps_TSetOfMetricMapInitializers, mrpt::config::CLoadableOptions> cl(M("mrpt::maps"), "TSetOfMetricMapInitializers", "A set of TMetricMapInitializer structures, passed to the constructor\n CMultiMetricMap::CMultiMetricMap\n  See the comments for TSetOfMetricMapInitializers::loadFromConfigFile, and\n \"CMultiMetricMap::setListOfMaps\" for\n   effectively creating the list of desired maps.\n \n\n CMultiMetricMap::CMultiMetricMap, CLoadableOptions\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::maps::TSetOfMetricMapInitializers(); }, [](){ return new PyCallBack_mrpt_maps_TSetOfMetricMapInitializers(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_maps_TSetOfMetricMapInitializers const &o){ return new PyCallBack_mrpt_maps_TSetOfMetricMapInitializers(o); } ) );
		cl.def( pybind11::init( [](mrpt::maps::TSetOfMetricMapInitializers const &o){ return new mrpt::maps::TSetOfMetricMapInitializers(o); } ) );
		cl.def("size", (size_t (mrpt::maps::TSetOfMetricMapInitializers::*)() const) &mrpt::maps::TSetOfMetricMapInitializers::size, "C++: mrpt::maps::TSetOfMetricMapInitializers::size() const --> size_t");
		cl.def("clear", (void (mrpt::maps::TSetOfMetricMapInitializers::*)()) &mrpt::maps::TSetOfMetricMapInitializers::clear, "C++: mrpt::maps::TSetOfMetricMapInitializers::clear() --> void");
		cl.def("loadFromConfigFile", (void (mrpt::maps::TSetOfMetricMapInitializers::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::maps::TSetOfMetricMapInitializers::loadFromConfigFile, "Loads the configuration for the set of internal maps from a textual\ndefinition in an INI-like file.\n  The format of the ini file is defined in CConfigFile. The list\nof maps and their options\n   will be loaded from a handle of sections:\n\n  \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n  Where:\n		- ##: Represents the index of the map (e.g. \"00\",\"01\",...)\n		- By default, the variables into each \"TOptions\" structure of the\nmaps\nare defined in textual form by the same name of the corresponding C++\nvariable (e.g. \"float resolution;\" -> \"resolution=0.10\")\n\n \n Examples of map definitions can be found in the '.ini' files\nprovided in the demo directories: \"share/mrpt/config-files/\"\n\nC++: mrpt::maps::TSetOfMetricMapInitializers::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("sectionName"));
		cl.def("saveToConfigFile", (void (mrpt::maps::TSetOfMetricMapInitializers::*)(class mrpt::config::CConfigFileBase &, const std::string &) const) &mrpt::maps::TSetOfMetricMapInitializers::saveToConfigFile, "C++: mrpt::maps::TSetOfMetricMapInitializers::saveToConfigFile(class mrpt::config::CConfigFileBase &, const std::string &) const --> void", pybind11::arg("target"), pybind11::arg("section"));
		cl.def("assign", (class mrpt::maps::TSetOfMetricMapInitializers & (mrpt::maps::TSetOfMetricMapInitializers::*)(const class mrpt::maps::TSetOfMetricMapInitializers &)) &mrpt::maps::TSetOfMetricMapInitializers::operator=, "C++: mrpt::maps::TSetOfMetricMapInitializers::operator=(const class mrpt::maps::TSetOfMetricMapInitializers &) --> class mrpt::maps::TSetOfMetricMapInitializers &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
