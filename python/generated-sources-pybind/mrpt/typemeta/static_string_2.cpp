#include <mrpt/typemeta/static_string.h>
#include <sstream> // __str__

#include <functional>
#include <pybind11/pybind11.h>
#include <string>
#include <stl_binders.hpp>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

void bind_mrpt_typemeta_static_string_2(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::typemeta::string_literal file:mrpt/typemeta/static_string.h line:25
		pybind11::class_<mrpt::typemeta::string_literal<43>, std::shared_ptr<mrpt::typemeta::string_literal<43>>> cl(M("mrpt::typemeta"), "string_literal_43_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::string_literal<43> const &o){ return new mrpt::typemeta::string_literal<43>(o); } ) );
		cl.def("size", (std::size_t (mrpt::typemeta::string_literal<43>::*)() const) &mrpt::typemeta::string_literal<43>::size, "C++: mrpt::typemeta::string_literal<43>::size() const --> std::size_t");
		cl.def("__getitem__", (char (mrpt::typemeta::string_literal<43>::*)(int) const) &mrpt::typemeta::string_literal<43>::operator[], "C++: mrpt::typemeta::string_literal<43>::operator[](int) const --> char", pybind11::arg("i"));
		cl.def("c_str", (const char * (mrpt::typemeta::string_literal<43>::*)() const) &mrpt::typemeta::string_literal<43>::c_str, "C++: mrpt::typemeta::string_literal<43>::c_str() const --> const char *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::typemeta::string_literal file:mrpt/typemeta/static_string.h line:25
		pybind11::class_<mrpt::typemeta::string_literal<37>, std::shared_ptr<mrpt::typemeta::string_literal<37>>> cl(M("mrpt::typemeta"), "string_literal_37_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::string_literal<37> const &o){ return new mrpt::typemeta::string_literal<37>(o); } ) );
		cl.def("size", (std::size_t (mrpt::typemeta::string_literal<37>::*)() const) &mrpt::typemeta::string_literal<37>::size, "C++: mrpt::typemeta::string_literal<37>::size() const --> std::size_t");
		cl.def("__getitem__", (char (mrpt::typemeta::string_literal<37>::*)(int) const) &mrpt::typemeta::string_literal<37>::operator[], "C++: mrpt::typemeta::string_literal<37>::operator[](int) const --> char", pybind11::arg("i"));
		cl.def("c_str", (const char * (mrpt::typemeta::string_literal<37>::*)() const) &mrpt::typemeta::string_literal<37>::c_str, "C++: mrpt::typemeta::string_literal<37>::c_str() const --> const char *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::typemeta::string_literal file:mrpt/typemeta/static_string.h line:25
		pybind11::class_<mrpt::typemeta::string_literal<38>, std::shared_ptr<mrpt::typemeta::string_literal<38>>> cl(M("mrpt::typemeta"), "string_literal_38_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::string_literal<38> const &o){ return new mrpt::typemeta::string_literal<38>(o); } ) );
		cl.def("size", (std::size_t (mrpt::typemeta::string_literal<38>::*)() const) &mrpt::typemeta::string_literal<38>::size, "C++: mrpt::typemeta::string_literal<38>::size() const --> std::size_t");
		cl.def("__getitem__", (char (mrpt::typemeta::string_literal<38>::*)(int) const) &mrpt::typemeta::string_literal<38>::operator[], "C++: mrpt::typemeta::string_literal<38>::operator[](int) const --> char", pybind11::arg("i"));
		cl.def("c_str", (const char * (mrpt::typemeta::string_literal<38>::*)() const) &mrpt::typemeta::string_literal<38>::c_str, "C++: mrpt::typemeta::string_literal<38>::c_str() const --> const char *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::typemeta::string_literal file:mrpt/typemeta/static_string.h line:25
		pybind11::class_<mrpt::typemeta::string_literal<23>, std::shared_ptr<mrpt::typemeta::string_literal<23>>> cl(M("mrpt::typemeta"), "string_literal_23_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::string_literal<23> const &o){ return new mrpt::typemeta::string_literal<23>(o); } ) );
		cl.def("size", (std::size_t (mrpt::typemeta::string_literal<23>::*)() const) &mrpt::typemeta::string_literal<23>::size, "C++: mrpt::typemeta::string_literal<23>::size() const --> std::size_t");
		cl.def("__getitem__", (char (mrpt::typemeta::string_literal<23>::*)(int) const) &mrpt::typemeta::string_literal<23>::operator[], "C++: mrpt::typemeta::string_literal<23>::operator[](int) const --> char", pybind11::arg("i"));
		cl.def("c_str", (const char * (mrpt::typemeta::string_literal<23>::*)() const) &mrpt::typemeta::string_literal<23>::c_str, "C++: mrpt::typemeta::string_literal<23>::c_str() const --> const char *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::typemeta::string_literal file:mrpt/typemeta/static_string.h line:25
		pybind11::class_<mrpt::typemeta::string_literal<42>, std::shared_ptr<mrpt::typemeta::string_literal<42>>> cl(M("mrpt::typemeta"), "string_literal_42_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::string_literal<42> const &o){ return new mrpt::typemeta::string_literal<42>(o); } ) );
		cl.def("size", (std::size_t (mrpt::typemeta::string_literal<42>::*)() const) &mrpt::typemeta::string_literal<42>::size, "C++: mrpt::typemeta::string_literal<42>::size() const --> std::size_t");
		cl.def("__getitem__", (char (mrpt::typemeta::string_literal<42>::*)(int) const) &mrpt::typemeta::string_literal<42>::operator[], "C++: mrpt::typemeta::string_literal<42>::operator[](int) const --> char", pybind11::arg("i"));
		cl.def("c_str", (const char * (mrpt::typemeta::string_literal<42>::*)() const) &mrpt::typemeta::string_literal<42>::c_str, "C++: mrpt::typemeta::string_literal<42>::c_str() const --> const char *", pybind11::return_value_policy::automatic);
	}
}
