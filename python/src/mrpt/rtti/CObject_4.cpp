#include <memory>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CSerializable.h>
#include <sstream> // __str__

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

void bind_mrpt_rtti_CObject_4(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::ptr_cast file:mrpt/rtti/CObject.h line:336
		pybind11::class_<mrpt::ptr_cast<mrpt::serialization::CSerializable>, std::shared_ptr<mrpt::ptr_cast<mrpt::serialization::CSerializable>>> cl(M("mrpt"), "ptr_cast_mrpt_serialization_CSerializable_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::ptr_cast<mrpt::serialization::CSerializable>(); } ) );
		cl.def_static("from", (class std::shared_ptr<class mrpt::serialization::CSerializable> (*)(const class std::shared_ptr<class mrpt::rtti::CObject> &)) &mrpt::ptr_cast<mrpt::serialization::CSerializable>::from<std::shared_ptr<mrpt::rtti::CObject>>, "C++: mrpt::ptr_cast<mrpt::serialization::CSerializable>::from(const class std::shared_ptr<class mrpt::rtti::CObject> &) --> class std::shared_ptr<class mrpt::serialization::CSerializable>", pybind11::arg("ptr"));
	}
}
