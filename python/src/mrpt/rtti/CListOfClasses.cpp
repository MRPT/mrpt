#include <iterator>
#include <memory>
#include <mrpt/rtti/CListOfClasses.h>
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

// mrpt::rtti::CListOfClasses file:mrpt/rtti/CListOfClasses.h line:23
struct PyCallBack_mrpt_rtti_CListOfClasses : public mrpt::rtti::CListOfClasses {
	using mrpt::rtti::CListOfClasses::CListOfClasses;

	std::string asString() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::rtti::CListOfClasses *>(this), "asString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CListOfClasses::asString();
	}
};

void bind_mrpt_rtti_CListOfClasses(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::rtti::CListOfClasses file:mrpt/rtti/CListOfClasses.h line:23
		pybind11::class_<mrpt::rtti::CListOfClasses, std::shared_ptr<mrpt::rtti::CListOfClasses>, PyCallBack_mrpt_rtti_CListOfClasses, mrpt::Stringifyable> cl(M("mrpt::rtti"), "CListOfClasses", "A list (actually based on a std::set) of MRPT classes, capable of keeping\n any class registered by the mechanism of CObject classes. Access to \"data\"\n for the actual content, or use any of the helper methods in this class.\n \n\n\n ");
		cl.def( pybind11::init( [](PyCallBack_mrpt_rtti_CListOfClasses const &o){ return new PyCallBack_mrpt_rtti_CListOfClasses(o); } ) );
		cl.def( pybind11::init( [](mrpt::rtti::CListOfClasses const &o){ return new mrpt::rtti::CListOfClasses(o); } ) );
		cl.def( pybind11::init( [](){ return new mrpt::rtti::CListOfClasses(); }, [](){ return new PyCallBack_mrpt_rtti_CListOfClasses(); } ) );
		cl.def_readwrite("data", &mrpt::rtti::CListOfClasses::data);
		cl.def("insert", (void (mrpt::rtti::CListOfClasses::*)(const struct mrpt::rtti::TRuntimeClassId *)) &mrpt::rtti::CListOfClasses::insert, "Insert a class in the list. Example of usage:\n   \n\n\n\n   \n\nC++: mrpt::rtti::CListOfClasses::insert(const struct mrpt::rtti::TRuntimeClassId *) --> void", pybind11::arg("id"));
		cl.def("contains", (bool (mrpt::rtti::CListOfClasses::*)(const struct mrpt::rtti::TRuntimeClassId *) const) &mrpt::rtti::CListOfClasses::contains, "Does the list contains this class? \n\nC++: mrpt::rtti::CListOfClasses::contains(const struct mrpt::rtti::TRuntimeClassId *) const --> bool", pybind11::arg("id"));
		cl.def("containsDerivedFrom", (bool (mrpt::rtti::CListOfClasses::*)(const struct mrpt::rtti::TRuntimeClassId *) const) &mrpt::rtti::CListOfClasses::containsDerivedFrom, "Does the list contains a class derived from...? \n\nC++: mrpt::rtti::CListOfClasses::containsDerivedFrom(const struct mrpt::rtti::TRuntimeClassId *) const --> bool", pybind11::arg("id"));
		cl.def("asString", (std::string (mrpt::rtti::CListOfClasses::*)() const) &mrpt::rtti::CListOfClasses::asString, "Return a string representation of the list, for example: \"CPose2D,\n CObservation, CPose3D\".\n\nC++: mrpt::rtti::CListOfClasses::asString() const --> std::string");
		cl.def("fromString", (void (mrpt::rtti::CListOfClasses::*)(const std::string &)) &mrpt::rtti::CListOfClasses::fromString, "Builds from a string representation of the list, for example: \"CPose2D,\n CObservation, CPose3D\".\n \n\n std::exception On unregistered class name found.\n\nC++: mrpt::rtti::CListOfClasses::fromString(const std::string &) --> void", pybind11::arg("s"));
		cl.def("assign", (class mrpt::rtti::CListOfClasses & (mrpt::rtti::CListOfClasses::*)(const class mrpt::rtti::CListOfClasses &)) &mrpt::rtti::CListOfClasses::operator=, "C++: mrpt::rtti::CListOfClasses::operator=(const class mrpt::rtti::CListOfClasses &) --> class mrpt::rtti::CListOfClasses &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
