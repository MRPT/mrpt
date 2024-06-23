#include <iterator>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/containers/map_as_vector.h>
#include <mrpt/containers/traits_map.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/nav/holonomic/CAbstractHolonomicReactiveMethod.h>
#include <mrpt/nav/holonomic/CHolonomicVFF.h>
#include <mrpt/nav/planners/PlannerRRT_common.h>
#include <mrpt/nav/planners/TMoveTree.h>
#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/typemeta/static_string.h>
#include <sstream> // __str__
#include <string>
#include <string_view>
#include <utility>
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

// mrpt::nav::CLogFileRecord_VFF file:mrpt/nav/holonomic/CHolonomicVFF.h line:26
struct PyCallBack_mrpt_nav_CLogFileRecord_VFF : public mrpt::nav::CLogFileRecord_VFF {
	using mrpt::nav::CLogFileRecord_VFF::CLogFileRecord_VFF;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CLogFileRecord_VFF *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CLogFileRecord_VFF::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CLogFileRecord_VFF *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CLogFileRecord_VFF::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CLogFileRecord_VFF *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CLogFileRecord_VFF::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CLogFileRecord_VFF *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CLogFileRecord_VFF::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CLogFileRecord_VFF *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CLogFileRecord_VFF::serializeFrom(a0, a1);
	}
	const class mrpt::math::CMatrixD * getDirectionScores() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CLogFileRecord_VFF *>(this), "getDirectionScores");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const class mrpt::math::CMatrixD *>::value) {
				static pybind11::detail::override_caster_t<const class mrpt::math::CMatrixD *> caster;
				return pybind11::detail::cast_ref<const class mrpt::math::CMatrixD *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const class mrpt::math::CMatrixD *>(std::move(o));
		}
		return CHolonomicLogFileRecord::getDirectionScores();
	}
};

// mrpt::nav::CHolonomicVFF file:mrpt/nav/holonomic/CHolonomicVFF.h line:48
struct PyCallBack_mrpt_nav_CHolonomicVFF : public mrpt::nav::CHolonomicVFF {
	using mrpt::nav::CHolonomicVFF::CHolonomicVFF;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicVFF *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CHolonomicVFF::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicVFF *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CHolonomicVFF::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicVFF *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CHolonomicVFF::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicVFF *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CHolonomicVFF::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicVFF *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CHolonomicVFF::serializeFrom(a0, a1);
	}
	void navigate(const struct mrpt::nav::CAbstractHolonomicReactiveMethod::NavInput & a0, struct mrpt::nav::CAbstractHolonomicReactiveMethod::NavOutput & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicVFF *>(this), "navigate");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CHolonomicVFF::navigate(a0, a1);
	}
	void initialize(const class mrpt::config::CConfigFileBase & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicVFF *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CHolonomicVFF::initialize(a0);
	}
	void saveConfigFile(class mrpt::config::CConfigFileBase & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicVFF *>(this), "saveConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CHolonomicVFF::saveConfigFile(a0);
	}
	double getTargetApproachSlowDownDistance() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicVFF *>(this), "getTargetApproachSlowDownDistance");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CHolonomicVFF::getTargetApproachSlowDownDistance();
	}
	void setTargetApproachSlowDownDistance(const double a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicVFF *>(this), "setTargetApproachSlowDownDistance");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CHolonomicVFF::setTargetApproachSlowDownDistance(a0);
	}
};

// mrpt::nav::CHolonomicVFF::TOptions file:mrpt/nav/holonomic/CHolonomicVFF.h line:63
struct PyCallBack_mrpt_nav_CHolonomicVFF_TOptions : public mrpt::nav::CHolonomicVFF::TOptions {
	using mrpt::nav::CHolonomicVFF::TOptions::TOptions;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicVFF::TOptions *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TOptions::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicVFF::TOptions *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TOptions::saveToConfigFile(a0, a1);
	}
};

void bind_mrpt_nav_holonomic_CHolonomicVFF(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::nav::CLogFileRecord_VFF file:mrpt/nav/holonomic/CHolonomicVFF.h line:26
		pybind11::class_<mrpt::nav::CLogFileRecord_VFF, std::shared_ptr<mrpt::nav::CLogFileRecord_VFF>, PyCallBack_mrpt_nav_CLogFileRecord_VFF, mrpt::nav::CHolonomicLogFileRecord> cl(M("mrpt::nav"), "CLogFileRecord_VFF", "A class for storing extra information about the execution of\n    CHolonomicVFF navigation.\n \n\n CHolonomicVFF, CHolonomicLogFileRecord");
		cl.def( pybind11::init( [](){ return new mrpt::nav::CLogFileRecord_VFF(); }, [](){ return new PyCallBack_mrpt_nav_CLogFileRecord_VFF(); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::nav::CLogFileRecord_VFF::GetRuntimeClassIdStatic, "C++: mrpt::nav::CLogFileRecord_VFF::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::nav::CLogFileRecord_VFF::*)() const) &mrpt::nav::CLogFileRecord_VFF::GetRuntimeClass, "C++: mrpt::nav::CLogFileRecord_VFF::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::nav::CLogFileRecord_VFF::*)() const) &mrpt::nav::CLogFileRecord_VFF::clone, "C++: mrpt::nav::CLogFileRecord_VFF::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::nav::CLogFileRecord_VFF::CreateObject, "C++: mrpt::nav::CLogFileRecord_VFF::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("assign", (class mrpt::nav::CLogFileRecord_VFF & (mrpt::nav::CLogFileRecord_VFF::*)(const class mrpt::nav::CLogFileRecord_VFF &)) &mrpt::nav::CLogFileRecord_VFF::operator=, "C++: mrpt::nav::CLogFileRecord_VFF::operator=(const class mrpt::nav::CLogFileRecord_VFF &) --> class mrpt::nav::CLogFileRecord_VFF &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::nav::CHolonomicVFF file:mrpt/nav/holonomic/CHolonomicVFF.h line:48
		pybind11::class_<mrpt::nav::CHolonomicVFF, std::shared_ptr<mrpt::nav::CHolonomicVFF>, PyCallBack_mrpt_nav_CHolonomicVFF, mrpt::nav::CAbstractHolonomicReactiveMethod> cl(M("mrpt::nav"), "CHolonomicVFF", "A holonomic reactive navigation method, based on Virtual Force Fields (VFF).\n\n These are the optional parameters of the method which can be set by means of\n a configuration file passed to the constructor or to CHolonomicND::initialize\n (see also the field CHolonomicVFF::options).\n\n \n\n\n\n\n\n\n  \n CAbstractHolonomicReactiveMethod,CReactiveNavigationSystem");
		cl.def( pybind11::init( [](){ return new mrpt::nav::CHolonomicVFF(); }, [](){ return new PyCallBack_mrpt_nav_CHolonomicVFF(); } ), "doc");
		cl.def( pybind11::init<const class mrpt::config::CConfigFileBase *>(), pybind11::arg("INI_FILE") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_nav_CHolonomicVFF const &o){ return new PyCallBack_mrpt_nav_CHolonomicVFF(o); } ) );
		cl.def( pybind11::init( [](mrpt::nav::CHolonomicVFF const &o){ return new mrpt::nav::CHolonomicVFF(o); } ) );
		cl.def_readwrite("options", &mrpt::nav::CHolonomicVFF::options);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::nav::CHolonomicVFF::GetRuntimeClassIdStatic, "C++: mrpt::nav::CHolonomicVFF::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::nav::CHolonomicVFF::*)() const) &mrpt::nav::CHolonomicVFF::GetRuntimeClass, "C++: mrpt::nav::CHolonomicVFF::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::nav::CHolonomicVFF::*)() const) &mrpt::nav::CHolonomicVFF::clone, "C++: mrpt::nav::CHolonomicVFF::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::nav::CHolonomicVFF::CreateObject, "C++: mrpt::nav::CHolonomicVFF::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("navigate", (void (mrpt::nav::CHolonomicVFF::*)(const struct mrpt::nav::CAbstractHolonomicReactiveMethod::NavInput &, struct mrpt::nav::CAbstractHolonomicReactiveMethod::NavOutput &)) &mrpt::nav::CHolonomicVFF::navigate, "C++: mrpt::nav::CHolonomicVFF::navigate(const struct mrpt::nav::CAbstractHolonomicReactiveMethod::NavInput &, struct mrpt::nav::CAbstractHolonomicReactiveMethod::NavOutput &) --> void", pybind11::arg("ni"), pybind11::arg("no"));
		cl.def("initialize", (void (mrpt::nav::CHolonomicVFF::*)(const class mrpt::config::CConfigFileBase &)) &mrpt::nav::CHolonomicVFF::initialize, "C++: mrpt::nav::CHolonomicVFF::initialize(const class mrpt::config::CConfigFileBase &) --> void", pybind11::arg("INI_FILE"));
		cl.def("saveConfigFile", (void (mrpt::nav::CHolonomicVFF::*)(class mrpt::config::CConfigFileBase &) const) &mrpt::nav::CHolonomicVFF::saveConfigFile, "C++: mrpt::nav::CHolonomicVFF::saveConfigFile(class mrpt::config::CConfigFileBase &) const --> void", pybind11::arg("c"));
		cl.def("getTargetApproachSlowDownDistance", (double (mrpt::nav::CHolonomicVFF::*)() const) &mrpt::nav::CHolonomicVFF::getTargetApproachSlowDownDistance, "C++: mrpt::nav::CHolonomicVFF::getTargetApproachSlowDownDistance() const --> double");
		cl.def("setTargetApproachSlowDownDistance", (void (mrpt::nav::CHolonomicVFF::*)(const double)) &mrpt::nav::CHolonomicVFF::setTargetApproachSlowDownDistance, "C++: mrpt::nav::CHolonomicVFF::setTargetApproachSlowDownDistance(const double) --> void", pybind11::arg("dist"));
		cl.def("assign", (class mrpt::nav::CHolonomicVFF & (mrpt::nav::CHolonomicVFF::*)(const class mrpt::nav::CHolonomicVFF &)) &mrpt::nav::CHolonomicVFF::operator=, "C++: mrpt::nav::CHolonomicVFF::operator=(const class mrpt::nav::CHolonomicVFF &) --> class mrpt::nav::CHolonomicVFF &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::nav::CHolonomicVFF::TOptions file:mrpt/nav/holonomic/CHolonomicVFF.h line:63
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::nav::CHolonomicVFF::TOptions, std::shared_ptr<mrpt::nav::CHolonomicVFF::TOptions>, PyCallBack_mrpt_nav_CHolonomicVFF_TOptions, mrpt::config::CLoadableOptions> cl(enclosing_class, "TOptions", "Algorithm options ");
			cl.def( pybind11::init( [](){ return new mrpt::nav::CHolonomicVFF::TOptions(); }, [](){ return new PyCallBack_mrpt_nav_CHolonomicVFF_TOptions(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_nav_CHolonomicVFF_TOptions const &o){ return new PyCallBack_mrpt_nav_CHolonomicVFF_TOptions(o); } ) );
			cl.def( pybind11::init( [](mrpt::nav::CHolonomicVFF::TOptions const &o){ return new mrpt::nav::CHolonomicVFF::TOptions(o); } ) );
			cl.def_readwrite("TARGET_SLOW_APPROACHING_DISTANCE", &mrpt::nav::CHolonomicVFF::TOptions::TARGET_SLOW_APPROACHING_DISTANCE);
			cl.def_readwrite("TARGET_ATTRACTIVE_FORCE", &mrpt::nav::CHolonomicVFF::TOptions::TARGET_ATTRACTIVE_FORCE);
			cl.def("loadFromConfigFile", (void (mrpt::nav::CHolonomicVFF::TOptions::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::nav::CHolonomicVFF::TOptions::loadFromConfigFile, "C++: mrpt::nav::CHolonomicVFF::TOptions::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
			cl.def("saveToConfigFile", (void (mrpt::nav::CHolonomicVFF::TOptions::*)(class mrpt::config::CConfigFileBase &, const std::string &) const) &mrpt::nav::CHolonomicVFF::TOptions::saveToConfigFile, "C++: mrpt::nav::CHolonomicVFF::TOptions::saveToConfigFile(class mrpt::config::CConfigFileBase &, const std::string &) const --> void", pybind11::arg("cfg"), pybind11::arg("section"));
			cl.def("assign", (struct mrpt::nav::CHolonomicVFF::TOptions & (mrpt::nav::CHolonomicVFF::TOptions::*)(const struct mrpt::nav::CHolonomicVFF::TOptions &)) &mrpt::nav::CHolonomicVFF::TOptions::operator=, "C++: mrpt::nav::CHolonomicVFF::TOptions::operator=(const struct mrpt::nav::CHolonomicVFF::TOptions &) --> struct mrpt::nav::CHolonomicVFF::TOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
	{ // mrpt::nav::TPlannerInputTempl file:mrpt/nav/planners/PlannerRRT_common.h line:29
		pybind11::class_<mrpt::nav::TPlannerInputTempl<mrpt::math::TPose2D,mrpt::math::TPose2D>, std::shared_ptr<mrpt::nav::TPlannerInputTempl<mrpt::math::TPose2D,mrpt::math::TPose2D>>> cl(M("mrpt::nav"), "TPlannerInputTempl_mrpt_math_TPose2D_mrpt_math_TPose2D_t", "");
		cl.def( pybind11::init( [](mrpt::nav::TPlannerInputTempl<mrpt::math::TPose2D,mrpt::math::TPose2D> const &o){ return new mrpt::nav::TPlannerInputTempl<mrpt::math::TPose2D,mrpt::math::TPose2D>(o); } ) );
		cl.def( pybind11::init( [](){ return new mrpt::nav::TPlannerInputTempl<mrpt::math::TPose2D,mrpt::math::TPose2D>(); } ) );
		cl.def_readwrite("start_pose", &mrpt::nav::TPlannerInputTempl<mrpt::math::TPose2D,mrpt::math::TPose2D>::start_pose);
		cl.def_readwrite("goal_pose", &mrpt::nav::TPlannerInputTempl<mrpt::math::TPose2D,mrpt::math::TPose2D>::goal_pose);
		cl.def_readwrite("world_bbox_min", &mrpt::nav::TPlannerInputTempl<mrpt::math::TPose2D,mrpt::math::TPose2D>::world_bbox_min);
		cl.def_readwrite("world_bbox_max", &mrpt::nav::TPlannerInputTempl<mrpt::math::TPose2D,mrpt::math::TPose2D>::world_bbox_max);
		cl.def_readwrite("obstacles_points", &mrpt::nav::TPlannerInputTempl<mrpt::math::TPose2D,mrpt::math::TPose2D>::obstacles_points);
		cl.def("assign", (struct mrpt::nav::TPlannerInputTempl<struct mrpt::math::TPose2D, struct mrpt::math::TPose2D> & (mrpt::nav::TPlannerInputTempl<mrpt::math::TPose2D,mrpt::math::TPose2D>::*)(const struct mrpt::nav::TPlannerInputTempl<struct mrpt::math::TPose2D, struct mrpt::math::TPose2D> &)) &mrpt::nav::TPlannerInputTempl<mrpt::math::TPose2D, mrpt::math::TPose2D>::operator=, "C++: mrpt::nav::TPlannerInputTempl<mrpt::math::TPose2D, mrpt::math::TPose2D>::operator=(const struct mrpt::nav::TPlannerInputTempl<struct mrpt::math::TPose2D, struct mrpt::math::TPose2D> &) --> struct mrpt::nav::TPlannerInputTempl<struct mrpt::math::TPose2D, struct mrpt::math::TPose2D> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::nav::TPlannerResultTempl file:mrpt/nav/planners/PlannerRRT_common.h line:39
		pybind11::class_<mrpt::nav::TPlannerResultTempl<mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP, mrpt::nav::TMoveEdgeSE2_TP>>, std::shared_ptr<mrpt::nav::TPlannerResultTempl<mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP, mrpt::nav::TMoveEdgeSE2_TP>>>> cl(M("mrpt::nav"), "TPlannerResultTempl_mrpt_nav_TMoveTree_mrpt_nav_TNodeSE2_TP_mrpt_nav_TMoveEdgeSE2_TP_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::nav::TPlannerResultTempl<mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP, mrpt::nav::TMoveEdgeSE2_TP>>(); } ) );
		cl.def( pybind11::init( [](mrpt::nav::TPlannerResultTempl<mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP, mrpt::nav::TMoveEdgeSE2_TP>> const &o){ return new mrpt::nav::TPlannerResultTempl<mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP, mrpt::nav::TMoveEdgeSE2_TP>>(o); } ) );
		cl.def_readwrite("success", &mrpt::nav::TPlannerResultTempl<mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP, mrpt::nav::TMoveEdgeSE2_TP>>::success);
		cl.def_readwrite("computation_time", &mrpt::nav::TPlannerResultTempl<mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP, mrpt::nav::TMoveEdgeSE2_TP>>::computation_time);
		cl.def_readwrite("goal_distance", &mrpt::nav::TPlannerResultTempl<mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP, mrpt::nav::TMoveEdgeSE2_TP>>::goal_distance);
		cl.def_readwrite("path_cost", &mrpt::nav::TPlannerResultTempl<mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP, mrpt::nav::TMoveEdgeSE2_TP>>::path_cost);
		cl.def_readwrite("best_goal_node_id", &mrpt::nav::TPlannerResultTempl<mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP, mrpt::nav::TMoveEdgeSE2_TP>>::best_goal_node_id);
		cl.def_readwrite("acceptable_goal_node_ids", &mrpt::nav::TPlannerResultTempl<mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP, mrpt::nav::TMoveEdgeSE2_TP>>::acceptable_goal_node_ids);
		cl.def_readwrite("move_tree", &mrpt::nav::TPlannerResultTempl<mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP, mrpt::nav::TMoveEdgeSE2_TP>>::move_tree);
		cl.def("assign", (struct mrpt::nav::TPlannerResultTempl<class mrpt::nav::TMoveTree<struct mrpt::nav::TNodeSE2_TP, struct mrpt::nav::TMoveEdgeSE2_TP> > & (mrpt::nav::TPlannerResultTempl<mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP, mrpt::nav::TMoveEdgeSE2_TP>>::*)(const struct mrpt::nav::TPlannerResultTempl<class mrpt::nav::TMoveTree<struct mrpt::nav::TNodeSE2_TP, struct mrpt::nav::TMoveEdgeSE2_TP> > &)) &mrpt::nav::TPlannerResultTempl<mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP, mrpt::nav::TMoveEdgeSE2_TP>>::operator=, "C++: mrpt::nav::TPlannerResultTempl<mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP, mrpt::nav::TMoveEdgeSE2_TP>>::operator=(const struct mrpt::nav::TPlannerResultTempl<class mrpt::nav::TMoveTree<struct mrpt::nav::TNodeSE2_TP, struct mrpt::nav::TMoveEdgeSE2_TP> > &) --> struct mrpt::nav::TPlannerResultTempl<class mrpt::nav::TMoveTree<struct mrpt::nav::TNodeSE2_TP, struct mrpt::nav::TMoveEdgeSE2_TP> > &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::nav::RRTEndCriteria file:mrpt/nav/planners/PlannerRRT_common.h line:66
		pybind11::class_<mrpt::nav::RRTEndCriteria, std::shared_ptr<mrpt::nav::RRTEndCriteria>> cl(M("mrpt::nav"), "RRTEndCriteria", "");
		cl.def( pybind11::init( [](){ return new mrpt::nav::RRTEndCriteria(); } ) );
		cl.def( pybind11::init( [](mrpt::nav::RRTEndCriteria const &o){ return new mrpt::nav::RRTEndCriteria(o); } ) );
		cl.def_readwrite("acceptedDistToTarget", &mrpt::nav::RRTEndCriteria::acceptedDistToTarget);
		cl.def_readwrite("acceptedAngToTarget", &mrpt::nav::RRTEndCriteria::acceptedAngToTarget);
		cl.def_readwrite("maxComputationTime", &mrpt::nav::RRTEndCriteria::maxComputationTime);
		cl.def_readwrite("minComputationTime", &mrpt::nav::RRTEndCriteria::minComputationTime);
		cl.def("assign", (struct mrpt::nav::RRTEndCriteria & (mrpt::nav::RRTEndCriteria::*)(const struct mrpt::nav::RRTEndCriteria &)) &mrpt::nav::RRTEndCriteria::operator=, "C++: mrpt::nav::RRTEndCriteria::operator=(const struct mrpt::nav::RRTEndCriteria &) --> struct mrpt::nav::RRTEndCriteria &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::nav::RRTAlgorithmParams file:mrpt/nav/planners/PlannerRRT_common.h line:86
		pybind11::class_<mrpt::nav::RRTAlgorithmParams, std::shared_ptr<mrpt::nav::RRTAlgorithmParams>> cl(M("mrpt::nav"), "RRTAlgorithmParams", "");
		cl.def( pybind11::init( [](){ return new mrpt::nav::RRTAlgorithmParams(); } ) );
		cl.def( pybind11::init( [](mrpt::nav::RRTAlgorithmParams const &o){ return new mrpt::nav::RRTAlgorithmParams(o); } ) );
		cl.def_readwrite("robot_shape", &mrpt::nav::RRTAlgorithmParams::robot_shape);
		cl.def_readwrite("robot_shape_circular_radius", &mrpt::nav::RRTAlgorithmParams::robot_shape_circular_radius);
		cl.def_readwrite("ptg_cache_files_directory", &mrpt::nav::RRTAlgorithmParams::ptg_cache_files_directory);
		cl.def_readwrite("goalBias", &mrpt::nav::RRTAlgorithmParams::goalBias);
		cl.def_readwrite("maxLength", &mrpt::nav::RRTAlgorithmParams::maxLength);
		cl.def_readwrite("minDistanceBetweenNewNodes", &mrpt::nav::RRTAlgorithmParams::minDistanceBetweenNewNodes);
		cl.def_readwrite("minAngBetweenNewNodes", &mrpt::nav::RRTAlgorithmParams::minAngBetweenNewNodes);
		cl.def_readwrite("ptg_verbose", &mrpt::nav::RRTAlgorithmParams::ptg_verbose);
		cl.def_readwrite("save_3d_log_freq", &mrpt::nav::RRTAlgorithmParams::save_3d_log_freq);
		cl.def("assign", (struct mrpt::nav::RRTAlgorithmParams & (mrpt::nav::RRTAlgorithmParams::*)(const struct mrpt::nav::RRTAlgorithmParams &)) &mrpt::nav::RRTAlgorithmParams::operator=, "C++: mrpt::nav::RRTAlgorithmParams::operator=(const struct mrpt::nav::RRTAlgorithmParams &) --> struct mrpt::nav::RRTAlgorithmParams &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::nav::PlannerTPS_VirtualBase file:mrpt/nav/planners/PlannerRRT_common.h line:132
		pybind11::class_<mrpt::nav::PlannerTPS_VirtualBase, std::shared_ptr<mrpt::nav::PlannerTPS_VirtualBase>> cl(M("mrpt::nav"), "PlannerTPS_VirtualBase", "Virtual base class for TP-Space-based path planners ");
		cl.def( pybind11::init( [](){ return new mrpt::nav::PlannerTPS_VirtualBase(); } ) );
		cl.def( pybind11::init( [](mrpt::nav::PlannerTPS_VirtualBase const &o){ return new mrpt::nav::PlannerTPS_VirtualBase(o); } ) );
		cl.def_readwrite("end_criteria", &mrpt::nav::PlannerTPS_VirtualBase::end_criteria);
		cl.def_readwrite("params", &mrpt::nav::PlannerTPS_VirtualBase::params);
		cl.def("getProfiler", (class mrpt::system::CTimeLogger & (mrpt::nav::PlannerTPS_VirtualBase::*)()) &mrpt::nav::PlannerTPS_VirtualBase::getProfiler, "C++: mrpt::nav::PlannerTPS_VirtualBase::getProfiler() --> class mrpt::system::CTimeLogger &", pybind11::return_value_policy::automatic);
		cl.def("assign", (class mrpt::nav::PlannerTPS_VirtualBase & (mrpt::nav::PlannerTPS_VirtualBase::*)(const class mrpt::nav::PlannerTPS_VirtualBase &)) &mrpt::nav::PlannerTPS_VirtualBase::operator=, "C++: mrpt::nav::PlannerTPS_VirtualBase::operator=(const class mrpt::nav::PlannerTPS_VirtualBase &) --> class mrpt::nav::PlannerTPS_VirtualBase &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions file:mrpt/nav/planners/PlannerRRT_common.h line:145
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions, std::shared_ptr<mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions>> cl(enclosing_class, "TRenderPlannedPathOptions", "Options for renderMoveTree()  ");
			cl.def( pybind11::init( [](){ return new mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions(); } ) );
			cl.def( pybind11::init( [](mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions const &o){ return new mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions(o); } ) );
			cl.def_readwrite("highlight_path_to_node_id", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::highlight_path_to_node_id);
			cl.def_readwrite("draw_shape_decimation", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::draw_shape_decimation);
			cl.def_readwrite("xyzcorners_scale", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::xyzcorners_scale);
			cl.def_readwrite("highlight_last_added_edge", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::highlight_last_added_edge);
			cl.def_readwrite("ground_xy_grid_frequency", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::ground_xy_grid_frequency);
			cl.def_readwrite("color_vehicle", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::color_vehicle);
			cl.def_readwrite("color_obstacles", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::color_obstacles);
			cl.def_readwrite("color_local_obstacles", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::color_local_obstacles);
			cl.def_readwrite("color_start", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::color_start);
			cl.def_readwrite("color_goal", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::color_goal);
			cl.def_readwrite("color_ground_xy_grid", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::color_ground_xy_grid);
			cl.def_readwrite("color_normal_edge", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::color_normal_edge);
			cl.def_readwrite("color_last_edge", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::color_last_edge);
			cl.def_readwrite("color_optimal_edge", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::color_optimal_edge);
			cl.def_readwrite("width_last_edge", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::width_last_edge);
			cl.def_readwrite("width_normal_edge", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::width_normal_edge);
			cl.def_readwrite("width_optimal_edge", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::width_optimal_edge);
			cl.def_readwrite("point_size_obstacles", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::point_size_obstacles);
			cl.def_readwrite("point_size_local_obstacles", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::point_size_local_obstacles);
			cl.def_readwrite("vehicle_shape_z", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::vehicle_shape_z);
			cl.def_readwrite("vehicle_line_width", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::vehicle_line_width);
			cl.def_readwrite("draw_obstacles", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::draw_obstacles);
			cl.def_readwrite("log_msg", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::log_msg);
			cl.def_readwrite("log_msg_position", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::log_msg_position);
			cl.def_readwrite("log_msg_scale", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::log_msg_scale);
			cl.def("assign", (struct mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions & (mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::*)(const struct mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions &)) &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::operator=, "C++: mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::operator=(const struct mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions &) --> struct mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
}
