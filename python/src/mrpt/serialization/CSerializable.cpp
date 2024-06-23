#include <iterator>
#include <memory>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CMessage.h>
#include <mrpt/serialization/CSerializable.h>
#include <sstream> // __str__
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

// mrpt::serialization::CSerializable file:mrpt/serialization/CSerializable.h line:31
struct PyCallBack_mrpt_serialization_CSerializable : public mrpt::serialization::CSerializable {
	using mrpt::serialization::CSerializable::CSerializable;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::serialization::CSerializable *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CSerializable::GetRuntimeClass();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::serialization::CSerializable *>(this), "serializeGetVersion");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::serialization::CSerializable *>(this), "serializeTo");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::serialization::CSerializable *>(this), "serializeFrom");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::serialization::CSerializable *>(this), "clone");
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

void bind_mrpt_serialization_CSerializable(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::serialization::CSerializable file:mrpt/serialization/CSerializable.h line:31
		pybind11::class_<mrpt::serialization::CSerializable, std::shared_ptr<mrpt::serialization::CSerializable>, PyCallBack_mrpt_serialization_CSerializable, mrpt::rtti::CObject> cl(M("mrpt::serialization"), "CSerializable", "The virtual base class which provides a unified interface for all persistent\nobjects in MRPT.\n  Many important properties of this class are inherited from\nmrpt::rtti::CObject.\n Refer to the library tutorial: \n \n\n CArchive\n \n\n\n ");
		cl.def(pybind11::init<PyCallBack_mrpt_serialization_CSerializable const &>());
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_serialization_CSerializable(); } ) );
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::serialization::CSerializable::*)() const) &mrpt::serialization::CSerializable::GetRuntimeClass, "C++: mrpt::serialization::CSerializable::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::serialization::CSerializable::GetRuntimeClassIdStatic, "C++: mrpt::serialization::CSerializable::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("assign", (class mrpt::serialization::CSerializable & (mrpt::serialization::CSerializable::*)(const class mrpt::serialization::CSerializable &)) &mrpt::serialization::CSerializable::operator=, "C++: mrpt::serialization::CSerializable::operator=(const class mrpt::serialization::CSerializable &) --> class mrpt::serialization::CSerializable &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	// mrpt::serialization::ObjectToOctetVector(const class mrpt::serialization::CSerializable *, class std::vector<unsigned char> &) file:mrpt/serialization/CSerializable.h line:103
	M("mrpt::serialization").def("ObjectToOctetVector", (void (*)(const class mrpt::serialization::CSerializable *, class std::vector<unsigned char> &)) &mrpt::serialization::ObjectToOctetVector, "Converts (serializes) an MRPT object into an array of bytes.\n \n\n The object to be serialized.\n \n\n The vector which at return will contain the data. Size will\n be set automatically.\n \n\n OctetVectorToObject, ObjectToString\n\nC++: mrpt::serialization::ObjectToOctetVector(const class mrpt::serialization::CSerializable *, class std::vector<unsigned char> &) --> void", pybind11::arg("o"), pybind11::arg("out_vector"));

	// mrpt::serialization::OctetVectorToObject(const class std::vector<unsigned char> &, class std::shared_ptr<class mrpt::serialization::CSerializable> &) file:mrpt/serialization/CSerializable.h line:113
	M("mrpt::serialization").def("OctetVectorToObject", (void (*)(const class std::vector<unsigned char> &, class std::shared_ptr<class mrpt::serialization::CSerializable> &)) &mrpt::serialization::OctetVectorToObject, "Converts back (de-serializes) a sequence of binary data into a MRPT object,\n without prior information about the object's class.\n \n\n The serialized input data representing the object.\n \n\n The newly created object will be stored in this smart pointer.\n \n\n None On any internal exception, this function returns a nullptr\n pointer.\n \n\n ObjectToOctetVector, StringToObject\n\nC++: mrpt::serialization::OctetVectorToObject(const class std::vector<unsigned char> &, class std::shared_ptr<class mrpt::serialization::CSerializable> &) --> void", pybind11::arg("in_data"), pybind11::arg("obj"));

}
