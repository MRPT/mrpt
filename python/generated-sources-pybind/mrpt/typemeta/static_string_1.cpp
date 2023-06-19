#include <mrpt/typemeta/static_string.h>
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

void bind_mrpt_typemeta_static_string_1(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::typemeta::string_literal file:mrpt/typemeta/static_string.h line:25
		pybind11::class_<mrpt::typemeta::string_literal<29>, std::shared_ptr<mrpt::typemeta::string_literal<29>>> cl(M("mrpt::typemeta"), "string_literal_29_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::string_literal<29> const &o){ return new mrpt::typemeta::string_literal<29>(o); } ) );
		cl.def("size", (std::size_t (mrpt::typemeta::string_literal<29>::*)() const) &mrpt::typemeta::string_literal<29>::size, "C++: mrpt::typemeta::string_literal<29>::size() const --> std::size_t");
		cl.def("__getitem__", (char (mrpt::typemeta::string_literal<29>::*)(int) const) &mrpt::typemeta::string_literal<29>::operator[], "C++: mrpt::typemeta::string_literal<29>::operator[](int) const --> char", pybind11::arg("i"));
		cl.def("c_str", (const char * (mrpt::typemeta::string_literal<29>::*)() const) &mrpt::typemeta::string_literal<29>::c_str, "C++: mrpt::typemeta::string_literal<29>::c_str() const --> const char *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::typemeta::string_literal file:mrpt/typemeta/static_string.h line:25
		pybind11::class_<mrpt::typemeta::string_literal<24>, std::shared_ptr<mrpt::typemeta::string_literal<24>>> cl(M("mrpt::typemeta"), "string_literal_24_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::string_literal<24> const &o){ return new mrpt::typemeta::string_literal<24>(o); } ) );
		cl.def("size", (std::size_t (mrpt::typemeta::string_literal<24>::*)() const) &mrpt::typemeta::string_literal<24>::size, "C++: mrpt::typemeta::string_literal<24>::size() const --> std::size_t");
		cl.def("__getitem__", (char (mrpt::typemeta::string_literal<24>::*)(int) const) &mrpt::typemeta::string_literal<24>::operator[], "C++: mrpt::typemeta::string_literal<24>::operator[](int) const --> char", pybind11::arg("i"));
		cl.def("c_str", (const char * (mrpt::typemeta::string_literal<24>::*)() const) &mrpt::typemeta::string_literal<24>::c_str, "C++: mrpt::typemeta::string_literal<24>::c_str() const --> const char *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::typemeta::string_literal file:mrpt/typemeta/static_string.h line:25
		pybind11::class_<mrpt::typemeta::string_literal<32>, std::shared_ptr<mrpt::typemeta::string_literal<32>>> cl(M("mrpt::typemeta"), "string_literal_32_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::string_literal<32> const &o){ return new mrpt::typemeta::string_literal<32>(o); } ) );
		cl.def("size", (std::size_t (mrpt::typemeta::string_literal<32>::*)() const) &mrpt::typemeta::string_literal<32>::size, "C++: mrpt::typemeta::string_literal<32>::size() const --> std::size_t");
		cl.def("__getitem__", (char (mrpt::typemeta::string_literal<32>::*)(int) const) &mrpt::typemeta::string_literal<32>::operator[], "C++: mrpt::typemeta::string_literal<32>::operator[](int) const --> char", pybind11::arg("i"));
		cl.def("c_str", (const char * (mrpt::typemeta::string_literal<32>::*)() const) &mrpt::typemeta::string_literal<32>::c_str, "C++: mrpt::typemeta::string_literal<32>::c_str() const --> const char *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::typemeta::string_literal file:mrpt/typemeta/static_string.h line:25
		pybind11::class_<mrpt::typemeta::string_literal<27>, std::shared_ptr<mrpt::typemeta::string_literal<27>>> cl(M("mrpt::typemeta"), "string_literal_27_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::string_literal<27> const &o){ return new mrpt::typemeta::string_literal<27>(o); } ) );
		cl.def("size", (std::size_t (mrpt::typemeta::string_literal<27>::*)() const) &mrpt::typemeta::string_literal<27>::size, "C++: mrpt::typemeta::string_literal<27>::size() const --> std::size_t");
		cl.def("__getitem__", (char (mrpt::typemeta::string_literal<27>::*)(int) const) &mrpt::typemeta::string_literal<27>::operator[], "C++: mrpt::typemeta::string_literal<27>::operator[](int) const --> char", pybind11::arg("i"));
		cl.def("c_str", (const char * (mrpt::typemeta::string_literal<27>::*)() const) &mrpt::typemeta::string_literal<27>::c_str, "C++: mrpt::typemeta::string_literal<27>::c_str() const --> const char *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::typemeta::string_literal file:mrpt/typemeta/static_string.h line:25
		pybind11::class_<mrpt::typemeta::string_literal<22>, std::shared_ptr<mrpt::typemeta::string_literal<22>>> cl(M("mrpt::typemeta"), "string_literal_22_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::string_literal<22> const &o){ return new mrpt::typemeta::string_literal<22>(o); } ) );
		cl.def("size", (std::size_t (mrpt::typemeta::string_literal<22>::*)() const) &mrpt::typemeta::string_literal<22>::size, "C++: mrpt::typemeta::string_literal<22>::size() const --> std::size_t");
		cl.def("__getitem__", (char (mrpt::typemeta::string_literal<22>::*)(int) const) &mrpt::typemeta::string_literal<22>::operator[], "C++: mrpt::typemeta::string_literal<22>::operator[](int) const --> char", pybind11::arg("i"));
		cl.def("c_str", (const char * (mrpt::typemeta::string_literal<22>::*)() const) &mrpt::typemeta::string_literal<22>::c_str, "C++: mrpt::typemeta::string_literal<22>::c_str() const --> const char *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::typemeta::string_literal file:mrpt/typemeta/static_string.h line:25
		pybind11::class_<mrpt::typemeta::string_literal<30>, std::shared_ptr<mrpt::typemeta::string_literal<30>>> cl(M("mrpt::typemeta"), "string_literal_30_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::string_literal<30> const &o){ return new mrpt::typemeta::string_literal<30>(o); } ) );
		cl.def("size", (std::size_t (mrpt::typemeta::string_literal<30>::*)() const) &mrpt::typemeta::string_literal<30>::size, "C++: mrpt::typemeta::string_literal<30>::size() const --> std::size_t");
		cl.def("__getitem__", (char (mrpt::typemeta::string_literal<30>::*)(int) const) &mrpt::typemeta::string_literal<30>::operator[], "C++: mrpt::typemeta::string_literal<30>::operator[](int) const --> char", pybind11::arg("i"));
		cl.def("c_str", (const char * (mrpt::typemeta::string_literal<30>::*)() const) &mrpt::typemeta::string_literal<30>::c_str, "C++: mrpt::typemeta::string_literal<30>::c_str() const --> const char *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::typemeta::string_literal file:mrpt/typemeta/static_string.h line:25
		pybind11::class_<mrpt::typemeta::string_literal<12>, std::shared_ptr<mrpt::typemeta::string_literal<12>>> cl(M("mrpt::typemeta"), "string_literal_12_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::string_literal<12> const &o){ return new mrpt::typemeta::string_literal<12>(o); } ) );
		cl.def("size", (std::size_t (mrpt::typemeta::string_literal<12>::*)() const) &mrpt::typemeta::string_literal<12>::size, "C++: mrpt::typemeta::string_literal<12>::size() const --> std::size_t");
		cl.def("__getitem__", (char (mrpt::typemeta::string_literal<12>::*)(int) const) &mrpt::typemeta::string_literal<12>::operator[], "C++: mrpt::typemeta::string_literal<12>::operator[](int) const --> char", pybind11::arg("i"));
		cl.def("c_str", (const char * (mrpt::typemeta::string_literal<12>::*)() const) &mrpt::typemeta::string_literal<12>::c_str, "C++: mrpt::typemeta::string_literal<12>::c_str() const --> const char *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::typemeta::string_literal file:mrpt/typemeta/static_string.h line:25
		pybind11::class_<mrpt::typemeta::string_literal<17>, std::shared_ptr<mrpt::typemeta::string_literal<17>>> cl(M("mrpt::typemeta"), "string_literal_17_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::string_literal<17> const &o){ return new mrpt::typemeta::string_literal<17>(o); } ) );
		cl.def("size", (std::size_t (mrpt::typemeta::string_literal<17>::*)() const) &mrpt::typemeta::string_literal<17>::size, "C++: mrpt::typemeta::string_literal<17>::size() const --> std::size_t");
		cl.def("__getitem__", (char (mrpt::typemeta::string_literal<17>::*)(int) const) &mrpt::typemeta::string_literal<17>::operator[], "C++: mrpt::typemeta::string_literal<17>::operator[](int) const --> char", pybind11::arg("i"));
		cl.def("c_str", (const char * (mrpt::typemeta::string_literal<17>::*)() const) &mrpt::typemeta::string_literal<17>::c_str, "C++: mrpt::typemeta::string_literal<17>::c_str() const --> const char *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::typemeta::string_literal file:mrpt/typemeta/static_string.h line:25
		pybind11::class_<mrpt::typemeta::string_literal<21>, std::shared_ptr<mrpt::typemeta::string_literal<21>>> cl(M("mrpt::typemeta"), "string_literal_21_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::string_literal<21> const &o){ return new mrpt::typemeta::string_literal<21>(o); } ) );
		cl.def("size", (std::size_t (mrpt::typemeta::string_literal<21>::*)() const) &mrpt::typemeta::string_literal<21>::size, "C++: mrpt::typemeta::string_literal<21>::size() const --> std::size_t");
		cl.def("__getitem__", (char (mrpt::typemeta::string_literal<21>::*)(int) const) &mrpt::typemeta::string_literal<21>::operator[], "C++: mrpt::typemeta::string_literal<21>::operator[](int) const --> char", pybind11::arg("i"));
		cl.def("c_str", (const char * (mrpt::typemeta::string_literal<21>::*)() const) &mrpt::typemeta::string_literal<21>::c_str, "C++: mrpt::typemeta::string_literal<21>::c_str() const --> const char *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::typemeta::string_literal file:mrpt/typemeta/static_string.h line:25
		pybind11::class_<mrpt::typemeta::string_literal<10>, std::shared_ptr<mrpt::typemeta::string_literal<10>>> cl(M("mrpt::typemeta"), "string_literal_10_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::string_literal<10> const &o){ return new mrpt::typemeta::string_literal<10>(o); } ) );
		cl.def("size", (std::size_t (mrpt::typemeta::string_literal<10>::*)() const) &mrpt::typemeta::string_literal<10>::size, "C++: mrpt::typemeta::string_literal<10>::size() const --> std::size_t");
		cl.def("__getitem__", (char (mrpt::typemeta::string_literal<10>::*)(int) const) &mrpt::typemeta::string_literal<10>::operator[], "C++: mrpt::typemeta::string_literal<10>::operator[](int) const --> char", pybind11::arg("i"));
		cl.def("c_str", (const char * (mrpt::typemeta::string_literal<10>::*)() const) &mrpt::typemeta::string_literal<10>::c_str, "C++: mrpt::typemeta::string_literal<10>::c_str() const --> const char *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::typemeta::string_literal file:mrpt/typemeta/static_string.h line:25
		pybind11::class_<mrpt::typemeta::string_literal<25>, std::shared_ptr<mrpt::typemeta::string_literal<25>>> cl(M("mrpt::typemeta"), "string_literal_25_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::string_literal<25> const &o){ return new mrpt::typemeta::string_literal<25>(o); } ) );
		cl.def("size", (std::size_t (mrpt::typemeta::string_literal<25>::*)() const) &mrpt::typemeta::string_literal<25>::size, "C++: mrpt::typemeta::string_literal<25>::size() const --> std::size_t");
		cl.def("__getitem__", (char (mrpt::typemeta::string_literal<25>::*)(int) const) &mrpt::typemeta::string_literal<25>::operator[], "C++: mrpt::typemeta::string_literal<25>::operator[](int) const --> char", pybind11::arg("i"));
		cl.def("c_str", (const char * (mrpt::typemeta::string_literal<25>::*)() const) &mrpt::typemeta::string_literal<25>::c_str, "C++: mrpt::typemeta::string_literal<25>::c_str() const --> const char *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::typemeta::string_literal file:mrpt/typemeta/static_string.h line:25
		pybind11::class_<mrpt::typemeta::string_literal<34>, std::shared_ptr<mrpt::typemeta::string_literal<34>>> cl(M("mrpt::typemeta"), "string_literal_34_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::string_literal<34> const &o){ return new mrpt::typemeta::string_literal<34>(o); } ) );
		cl.def("size", (std::size_t (mrpt::typemeta::string_literal<34>::*)() const) &mrpt::typemeta::string_literal<34>::size, "C++: mrpt::typemeta::string_literal<34>::size() const --> std::size_t");
		cl.def("__getitem__", (char (mrpt::typemeta::string_literal<34>::*)(int) const) &mrpt::typemeta::string_literal<34>::operator[], "C++: mrpt::typemeta::string_literal<34>::operator[](int) const --> char", pybind11::arg("i"));
		cl.def("c_str", (const char * (mrpt::typemeta::string_literal<34>::*)() const) &mrpt::typemeta::string_literal<34>::c_str, "C++: mrpt::typemeta::string_literal<34>::c_str() const --> const char *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::typemeta::string_literal file:mrpt/typemeta/static_string.h line:25
		pybind11::class_<mrpt::typemeta::string_literal<26>, std::shared_ptr<mrpt::typemeta::string_literal<26>>> cl(M("mrpt::typemeta"), "string_literal_26_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::string_literal<26> const &o){ return new mrpt::typemeta::string_literal<26>(o); } ) );
		cl.def("size", (std::size_t (mrpt::typemeta::string_literal<26>::*)() const) &mrpt::typemeta::string_literal<26>::size, "C++: mrpt::typemeta::string_literal<26>::size() const --> std::size_t");
		cl.def("__getitem__", (char (mrpt::typemeta::string_literal<26>::*)(int) const) &mrpt::typemeta::string_literal<26>::operator[], "C++: mrpt::typemeta::string_literal<26>::operator[](int) const --> char", pybind11::arg("i"));
		cl.def("c_str", (const char * (mrpt::typemeta::string_literal<26>::*)() const) &mrpt::typemeta::string_literal<26>::c_str, "C++: mrpt::typemeta::string_literal<26>::c_str() const --> const char *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::typemeta::string_literal file:mrpt/typemeta/static_string.h line:25
		pybind11::class_<mrpt::typemeta::string_literal<11>, std::shared_ptr<mrpt::typemeta::string_literal<11>>> cl(M("mrpt::typemeta"), "string_literal_11_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::string_literal<11> const &o){ return new mrpt::typemeta::string_literal<11>(o); } ) );
		cl.def("size", (std::size_t (mrpt::typemeta::string_literal<11>::*)() const) &mrpt::typemeta::string_literal<11>::size, "C++: mrpt::typemeta::string_literal<11>::size() const --> std::size_t");
		cl.def("__getitem__", (char (mrpt::typemeta::string_literal<11>::*)(int) const) &mrpt::typemeta::string_literal<11>::operator[], "C++: mrpt::typemeta::string_literal<11>::operator[](int) const --> char", pybind11::arg("i"));
		cl.def("c_str", (const char * (mrpt::typemeta::string_literal<11>::*)() const) &mrpt::typemeta::string_literal<11>::c_str, "C++: mrpt::typemeta::string_literal<11>::c_str() const --> const char *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::typemeta::string_literal file:mrpt/typemeta/static_string.h line:25
		pybind11::class_<mrpt::typemeta::string_literal<35>, std::shared_ptr<mrpt::typemeta::string_literal<35>>> cl(M("mrpt::typemeta"), "string_literal_35_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::string_literal<35> const &o){ return new mrpt::typemeta::string_literal<35>(o); } ) );
		cl.def("size", (std::size_t (mrpt::typemeta::string_literal<35>::*)() const) &mrpt::typemeta::string_literal<35>::size, "C++: mrpt::typemeta::string_literal<35>::size() const --> std::size_t");
		cl.def("__getitem__", (char (mrpt::typemeta::string_literal<35>::*)(int) const) &mrpt::typemeta::string_literal<35>::operator[], "C++: mrpt::typemeta::string_literal<35>::operator[](int) const --> char", pybind11::arg("i"));
		cl.def("c_str", (const char * (mrpt::typemeta::string_literal<35>::*)() const) &mrpt::typemeta::string_literal<35>::c_str, "C++: mrpt::typemeta::string_literal<35>::c_str() const --> const char *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::typemeta::string_literal file:mrpt/typemeta/static_string.h line:25
		pybind11::class_<mrpt::typemeta::string_literal<36>, std::shared_ptr<mrpt::typemeta::string_literal<36>>> cl(M("mrpt::typemeta"), "string_literal_36_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::string_literal<36> const &o){ return new mrpt::typemeta::string_literal<36>(o); } ) );
		cl.def("size", (std::size_t (mrpt::typemeta::string_literal<36>::*)() const) &mrpt::typemeta::string_literal<36>::size, "C++: mrpt::typemeta::string_literal<36>::size() const --> std::size_t");
		cl.def("__getitem__", (char (mrpt::typemeta::string_literal<36>::*)(int) const) &mrpt::typemeta::string_literal<36>::operator[], "C++: mrpt::typemeta::string_literal<36>::operator[](int) const --> char", pybind11::arg("i"));
		cl.def("c_str", (const char * (mrpt::typemeta::string_literal<36>::*)() const) &mrpt::typemeta::string_literal<36>::c_str, "C++: mrpt::typemeta::string_literal<36>::c_str() const --> const char *", pybind11::return_value_policy::automatic);
	}
}
