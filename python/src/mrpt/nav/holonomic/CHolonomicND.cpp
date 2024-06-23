#include <iterator>
#include <memory>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/nav/holonomic/CHolonomicND.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/typemeta/static_string.h>
#include <sstream> // __str__
#include <string>
#include <variant>

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

// mrpt::nav::CLogFileRecord_ND file:mrpt/nav/holonomic/CHolonomicND.h line:167
struct PyCallBack_mrpt_nav_CLogFileRecord_ND : public mrpt::nav::CLogFileRecord_ND {
	using mrpt::nav::CLogFileRecord_ND::CLogFileRecord_ND;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CLogFileRecord_ND *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CLogFileRecord_ND::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CLogFileRecord_ND *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CLogFileRecord_ND::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CLogFileRecord_ND *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CLogFileRecord_ND::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CLogFileRecord_ND *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CLogFileRecord_ND::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CLogFileRecord_ND *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CLogFileRecord_ND::serializeFrom(a0, a1);
	}
	const class mrpt::math::CMatrixD * getDirectionScores() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CLogFileRecord_ND *>(this), "getDirectionScores");
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

void bind_mrpt_nav_holonomic_CHolonomicND(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::nav::CLogFileRecord_ND file:mrpt/nav/holonomic/CHolonomicND.h line:167
		pybind11::class_<mrpt::nav::CLogFileRecord_ND, std::shared_ptr<mrpt::nav::CLogFileRecord_ND>, PyCallBack_mrpt_nav_CLogFileRecord_ND, mrpt::nav::CHolonomicLogFileRecord> cl(M("mrpt::nav"), "CLogFileRecord_ND", "A class for storing extra information about the execution of\n    CHolonomicND navigation.\n \n\n CHolonomicND, CHolonomicLogFileRecord");
		cl.def( pybind11::init( [](){ return new mrpt::nav::CLogFileRecord_ND(); }, [](){ return new PyCallBack_mrpt_nav_CLogFileRecord_ND(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_nav_CLogFileRecord_ND const &o){ return new PyCallBack_mrpt_nav_CLogFileRecord_ND(o); } ) );
		cl.def( pybind11::init( [](mrpt::nav::CLogFileRecord_ND const &o){ return new mrpt::nav::CLogFileRecord_ND(o); } ) );
		cl.def_readwrite("gaps_ini", &mrpt::nav::CLogFileRecord_ND::gaps_ini);
		cl.def_readwrite("gaps_end", &mrpt::nav::CLogFileRecord_ND::gaps_end);
		cl.def_readwrite("gaps_eval", &mrpt::nav::CLogFileRecord_ND::gaps_eval);
		cl.def_readwrite("selectedSector", &mrpt::nav::CLogFileRecord_ND::selectedSector);
		cl.def_readwrite("evaluation", &mrpt::nav::CLogFileRecord_ND::evaluation);
		cl.def_readwrite("riskEvaluation", &mrpt::nav::CLogFileRecord_ND::riskEvaluation);
		cl.def_readwrite("situation", &mrpt::nav::CLogFileRecord_ND::situation);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::nav::CLogFileRecord_ND::GetRuntimeClassIdStatic, "C++: mrpt::nav::CLogFileRecord_ND::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::nav::CLogFileRecord_ND::*)() const) &mrpt::nav::CLogFileRecord_ND::GetRuntimeClass, "C++: mrpt::nav::CLogFileRecord_ND::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::nav::CLogFileRecord_ND::*)() const) &mrpt::nav::CLogFileRecord_ND::clone, "C++: mrpt::nav::CLogFileRecord_ND::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::nav::CLogFileRecord_ND::CreateObject, "C++: mrpt::nav::CLogFileRecord_ND::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("assign", (class mrpt::nav::CLogFileRecord_ND & (mrpt::nav::CLogFileRecord_ND::*)(const class mrpt::nav::CLogFileRecord_ND &)) &mrpt::nav::CLogFileRecord_ND::operator=, "C++: mrpt::nav::CLogFileRecord_ND::operator=(const class mrpt::nav::CLogFileRecord_ND &) --> class mrpt::nav::CLogFileRecord_ND &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
