#include <ios>
#include <iterator>
#include <memory>
#include <mrpt/obs/CAction.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <ostream>
#include <sstream> // __str__
#include <streambuf>
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

// mrpt::obs::CAction file:mrpt/obs/CAction.h line:24
struct PyCallBack_mrpt_obs_CAction : public mrpt::obs::CAction {
	using mrpt::obs::CAction::CAction;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CAction *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CAction::GetRuntimeClass();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CAction *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CSerializable::serializeGetVersion\"");
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CAction *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CSerializable::serializeTo\"");
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CAction *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CSerializable::serializeFrom\"");
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CAction *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CObject::clone\"");
	}
};

void bind_mrpt_obs_CAction(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::obs::CAction file:mrpt/obs/CAction.h line:24
		pybind11::class_<mrpt::obs::CAction, std::shared_ptr<mrpt::obs::CAction>, PyCallBack_mrpt_obs_CAction, mrpt::serialization::CSerializable> cl(M("mrpt::obs"), "CAction", "Declares a class for storing a robot action. It is used in\n mrpt::obs::CRawlog,\n    for logs storage and particle filter based simulations.\n  See derived classes for implementations.\n\n \n CActionCollection, CRawlog\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_obs_CAction(); } ) );
		cl.def(pybind11::init<PyCallBack_mrpt_obs_CAction const &>());
		cl.def_readwrite("timestamp", &mrpt::obs::CAction::timestamp);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::obs::CAction::*)() const) &mrpt::obs::CAction::GetRuntimeClass, "C++: mrpt::obs::CAction::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::obs::CAction::GetRuntimeClassIdStatic, "C++: mrpt::obs::CAction::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("getDescriptionAsTextValue", (std::string (mrpt::obs::CAction::*)() const) &mrpt::obs::CAction::getDescriptionAsTextValue, "Return by value version of getDescriptionAsText(std::ostream&) \n\nC++: mrpt::obs::CAction::getDescriptionAsTextValue() const --> std::string");
		cl.def("assign", (class mrpt::obs::CAction & (mrpt::obs::CAction::*)(const class mrpt::obs::CAction &)) &mrpt::obs::CAction::operator=, "C++: mrpt::obs::CAction::operator=(const class mrpt::obs::CAction &) --> class mrpt::obs::CAction &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
