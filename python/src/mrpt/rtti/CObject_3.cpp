#include <iterator>
#include <memory>
#include <mrpt/rtti/CObject.h>
#include <sstream> // __str__
#include <string>

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

// mrpt::rtti::CObject file:mrpt/rtti/CObject.h line:175
struct PyCallBack_mrpt_rtti_CObject : public mrpt::rtti::CObject {
	using mrpt::rtti::CObject::CObject;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::rtti::CObject *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CObject::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::rtti::CObject *>(this), "clone");
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

void bind_mrpt_rtti_CObject_3(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::rtti::CObject file:mrpt/rtti/CObject.h line:175
		pybind11::class_<mrpt::rtti::CObject, std::shared_ptr<mrpt::rtti::CObject>, PyCallBack_mrpt_rtti_CObject> cl(M("mrpt::rtti"), "CObject", "Virtual base to provide a compiler-independent RTTI system.\n\n Each class named `Foo` will have associated smart pointer types:\n - `Foo::Ptr` => `std::shared_ptr<Foo>` (the most commonly-used one)\n - `Foo::ConstPtr` => `std::shared_ptr<const Foo>`\n - `Foo::UniquePtr` => `std::unique_ptr<Foo>`\n - `Foo::ConstUniquePtr` => `std::unique_ptr<const Foo>`\n\n It is recommended to use MRPT-defined `std::make_shared<>` instead\n of `std::make_shared<>` to create objects, to avoid memory alignment\n problems caused by classes containing Eigen vectors or matrices. Example:\n \n\n\n\n Or using the shorter auxiliary static method `::Create()` for conciseness or\n to keep compatibility with MRPT 1.5.* code bases:\n \n\n\n\n If a special memory allocator is needed, use `Foo::CreateAlloc(alloc,...);`.\n \n\n  mrpt::rtti::CObject\n \n\n\n ");
		cl.def(pybind11::init<PyCallBack_mrpt_rtti_CObject const &>());
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_rtti_CObject(); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::rtti::CObject::GetRuntimeClassIdStatic, "C++: mrpt::rtti::CObject::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::rtti::CObject::*)() const) &mrpt::rtti::CObject::GetRuntimeClass, "Returns information about the class of an object in runtime. \n\nC++: mrpt::rtti::CObject::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("duplicateGetSmartPtr", (class std::shared_ptr<class mrpt::rtti::CObject> (mrpt::rtti::CObject::*)() const) &mrpt::rtti::CObject::duplicateGetSmartPtr, "Makes a deep copy of the object and returns a smart pointer to it \n\nC++: mrpt::rtti::CObject::duplicateGetSmartPtr() const --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::rtti::CObject::*)() const) &mrpt::rtti::CObject::clone, "Returns a deep copy (clone) of the object, indepently of its class. \n\nC++: mrpt::rtti::CObject::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def("assign", (class mrpt::rtti::CObject & (mrpt::rtti::CObject::*)(const class mrpt::rtti::CObject &)) &mrpt::rtti::CObject::operator=, "C++: mrpt::rtti::CObject::operator=(const class mrpt::rtti::CObject &) --> class mrpt::rtti::CObject &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	// mrpt::rtti::registerAllPendingClasses() file:mrpt/rtti/CObject.h line:321
	M("mrpt::rtti").def("registerAllPendingClasses", (void (*)()) &mrpt::rtti::registerAllPendingClasses, "Register all pending classes - to be called just before\n de-serializing an object, for example. After calling this method,\n pending_class_registers_modified is set to false until\n pending_class_registers() is invoked.\n\nC++: mrpt::rtti::registerAllPendingClasses() --> void");

	// mrpt::rtti::classFactory(const std::string &) file:mrpt/rtti/CObject.h line:325
	M("mrpt::rtti").def("classFactory", (class std::shared_ptr<class mrpt::rtti::CObject> (*)(const std::string &)) &mrpt::rtti::classFactory, "Creates an object given by its registered name.\n \n\n findRegisteredClass(), registerClass() \n\nC++: mrpt::rtti::classFactory(const std::string &) --> class std::shared_ptr<class mrpt::rtti::CObject>", pybind11::arg("className"));

}
