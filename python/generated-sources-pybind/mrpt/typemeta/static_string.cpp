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

void bind_mrpt_typemeta_static_string(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::typemeta::string_literal file:mrpt/typemeta/static_string.h line:25
		pybind11::class_<mrpt::typemeta::string_literal<16>, std::shared_ptr<mrpt::typemeta::string_literal<16>>> cl(M("mrpt::typemeta"), "string_literal_16_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::string_literal<16> const &o){ return new mrpt::typemeta::string_literal<16>(o); } ) );
		cl.def("size", (std::size_t (mrpt::typemeta::string_literal<16>::*)() const) &mrpt::typemeta::string_literal<16>::size, "C++: mrpt::typemeta::string_literal<16>::size() const --> std::size_t");
		cl.def("__getitem__", (char (mrpt::typemeta::string_literal<16>::*)(int) const) &mrpt::typemeta::string_literal<16>::operator[], "C++: mrpt::typemeta::string_literal<16>::operator[](int) const --> char", pybind11::arg("i"));
		cl.def("c_str", (const char * (mrpt::typemeta::string_literal<16>::*)() const) &mrpt::typemeta::string_literal<16>::c_str, "C++: mrpt::typemeta::string_literal<16>::c_str() const --> const char *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::typemeta::string_literal file:mrpt/typemeta/static_string.h line:25
		pybind11::class_<mrpt::typemeta::string_literal<1>, std::shared_ptr<mrpt::typemeta::string_literal<1>>> cl(M("mrpt::typemeta"), "string_literal_1_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::string_literal<1> const &o){ return new mrpt::typemeta::string_literal<1>(o); } ) );
		cl.def("size", (std::size_t (mrpt::typemeta::string_literal<1>::*)() const) &mrpt::typemeta::string_literal<1>::size, "C++: mrpt::typemeta::string_literal<1>::size() const --> std::size_t");
		cl.def("__getitem__", (char (mrpt::typemeta::string_literal<1>::*)(int) const) &mrpt::typemeta::string_literal<1>::operator[], "C++: mrpt::typemeta::string_literal<1>::operator[](int) const --> char", pybind11::arg("i"));
		cl.def("c_str", (const char * (mrpt::typemeta::string_literal<1>::*)() const) &mrpt::typemeta::string_literal<1>::c_str, "C++: mrpt::typemeta::string_literal<1>::c_str() const --> const char *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::typemeta::string_literal file:mrpt/typemeta/static_string.h line:25
		pybind11::class_<mrpt::typemeta::string_literal<4>, std::shared_ptr<mrpt::typemeta::string_literal<4>>> cl(M("mrpt::typemeta"), "string_literal_4_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::string_literal<4> const &o){ return new mrpt::typemeta::string_literal<4>(o); } ) );
		cl.def("size", (std::size_t (mrpt::typemeta::string_literal<4>::*)() const) &mrpt::typemeta::string_literal<4>::size, "C++: mrpt::typemeta::string_literal<4>::size() const --> std::size_t");
		cl.def("__getitem__", (char (mrpt::typemeta::string_literal<4>::*)(int) const) &mrpt::typemeta::string_literal<4>::operator[], "C++: mrpt::typemeta::string_literal<4>::operator[](int) const --> char", pybind11::arg("i"));
		cl.def("c_str", (const char * (mrpt::typemeta::string_literal<4>::*)() const) &mrpt::typemeta::string_literal<4>::c_str, "C++: mrpt::typemeta::string_literal<4>::c_str() const --> const char *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::typemeta::string_literal file:mrpt/typemeta/static_string.h line:25
		pybind11::class_<mrpt::typemeta::string_literal<6>, std::shared_ptr<mrpt::typemeta::string_literal<6>>> cl(M("mrpt::typemeta"), "string_literal_6_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::string_literal<6> const &o){ return new mrpt::typemeta::string_literal<6>(o); } ) );
		cl.def("size", (std::size_t (mrpt::typemeta::string_literal<6>::*)() const) &mrpt::typemeta::string_literal<6>::size, "C++: mrpt::typemeta::string_literal<6>::size() const --> std::size_t");
		cl.def("__getitem__", (char (mrpt::typemeta::string_literal<6>::*)(int) const) &mrpt::typemeta::string_literal<6>::operator[], "C++: mrpt::typemeta::string_literal<6>::operator[](int) const --> char", pybind11::arg("i"));
		cl.def("c_str", (const char * (mrpt::typemeta::string_literal<6>::*)() const) &mrpt::typemeta::string_literal<6>::c_str, "C++: mrpt::typemeta::string_literal<6>::c_str() const --> const char *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::typemeta::string_literal file:mrpt/typemeta/static_string.h line:25
		pybind11::class_<mrpt::typemeta::string_literal<5>, std::shared_ptr<mrpt::typemeta::string_literal<5>>> cl(M("mrpt::typemeta"), "string_literal_5_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::string_literal<5> const &o){ return new mrpt::typemeta::string_literal<5>(o); } ) );
		cl.def("size", (std::size_t (mrpt::typemeta::string_literal<5>::*)() const) &mrpt::typemeta::string_literal<5>::size, "C++: mrpt::typemeta::string_literal<5>::size() const --> std::size_t");
		cl.def("__getitem__", (char (mrpt::typemeta::string_literal<5>::*)(int) const) &mrpt::typemeta::string_literal<5>::operator[], "C++: mrpt::typemeta::string_literal<5>::operator[](int) const --> char", pybind11::arg("i"));
		cl.def("c_str", (const char * (mrpt::typemeta::string_literal<5>::*)() const) &mrpt::typemeta::string_literal<5>::c_str, "C++: mrpt::typemeta::string_literal<5>::c_str() const --> const char *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::typemeta::string_literal file:mrpt/typemeta/static_string.h line:25
		pybind11::class_<mrpt::typemeta::string_literal<8>, std::shared_ptr<mrpt::typemeta::string_literal<8>>> cl(M("mrpt::typemeta"), "string_literal_8_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::string_literal<8> const &o){ return new mrpt::typemeta::string_literal<8>(o); } ) );
		cl.def("size", (std::size_t (mrpt::typemeta::string_literal<8>::*)() const) &mrpt::typemeta::string_literal<8>::size, "C++: mrpt::typemeta::string_literal<8>::size() const --> std::size_t");
		cl.def("__getitem__", (char (mrpt::typemeta::string_literal<8>::*)(int) const) &mrpt::typemeta::string_literal<8>::operator[], "C++: mrpt::typemeta::string_literal<8>::operator[](int) const --> char", pybind11::arg("i"));
		cl.def("c_str", (const char * (mrpt::typemeta::string_literal<8>::*)() const) &mrpt::typemeta::string_literal<8>::c_str, "C++: mrpt::typemeta::string_literal<8>::c_str() const --> const char *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::typemeta::string_literal file:mrpt/typemeta/static_string.h line:25
		pybind11::class_<mrpt::typemeta::string_literal<7>, std::shared_ptr<mrpt::typemeta::string_literal<7>>> cl(M("mrpt::typemeta"), "string_literal_7_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::string_literal<7> const &o){ return new mrpt::typemeta::string_literal<7>(o); } ) );
		cl.def("size", (std::size_t (mrpt::typemeta::string_literal<7>::*)() const) &mrpt::typemeta::string_literal<7>::size, "C++: mrpt::typemeta::string_literal<7>::size() const --> std::size_t");
		cl.def("__getitem__", (char (mrpt::typemeta::string_literal<7>::*)(int) const) &mrpt::typemeta::string_literal<7>::operator[], "C++: mrpt::typemeta::string_literal<7>::operator[](int) const --> char", pybind11::arg("i"));
		cl.def("c_str", (const char * (mrpt::typemeta::string_literal<7>::*)() const) &mrpt::typemeta::string_literal<7>::c_str, "C++: mrpt::typemeta::string_literal<7>::c_str() const --> const char *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::typemeta::string_literal file:mrpt/typemeta/static_string.h line:25
		pybind11::class_<mrpt::typemeta::string_literal<13>, std::shared_ptr<mrpt::typemeta::string_literal<13>>> cl(M("mrpt::typemeta"), "string_literal_13_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::string_literal<13> const &o){ return new mrpt::typemeta::string_literal<13>(o); } ) );
		cl.def("size", (std::size_t (mrpt::typemeta::string_literal<13>::*)() const) &mrpt::typemeta::string_literal<13>::size, "C++: mrpt::typemeta::string_literal<13>::size() const --> std::size_t");
		cl.def("__getitem__", (char (mrpt::typemeta::string_literal<13>::*)(int) const) &mrpt::typemeta::string_literal<13>::operator[], "C++: mrpt::typemeta::string_literal<13>::operator[](int) const --> char", pybind11::arg("i"));
		cl.def("c_str", (const char * (mrpt::typemeta::string_literal<13>::*)() const) &mrpt::typemeta::string_literal<13>::c_str, "C++: mrpt::typemeta::string_literal<13>::c_str() const --> const char *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::typemeta::string_literal file:mrpt/typemeta/static_string.h line:25
		pybind11::class_<mrpt::typemeta::string_literal<19>, std::shared_ptr<mrpt::typemeta::string_literal<19>>> cl(M("mrpt::typemeta"), "string_literal_19_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::string_literal<19> const &o){ return new mrpt::typemeta::string_literal<19>(o); } ) );
		cl.def("size", (std::size_t (mrpt::typemeta::string_literal<19>::*)() const) &mrpt::typemeta::string_literal<19>::size, "C++: mrpt::typemeta::string_literal<19>::size() const --> std::size_t");
		cl.def("__getitem__", (char (mrpt::typemeta::string_literal<19>::*)(int) const) &mrpt::typemeta::string_literal<19>::operator[], "C++: mrpt::typemeta::string_literal<19>::operator[](int) const --> char", pybind11::arg("i"));
		cl.def("c_str", (const char * (mrpt::typemeta::string_literal<19>::*)() const) &mrpt::typemeta::string_literal<19>::c_str, "C++: mrpt::typemeta::string_literal<19>::c_str() const --> const char *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::typemeta::string_literal file:mrpt/typemeta/static_string.h line:25
		pybind11::class_<mrpt::typemeta::string_literal<18>, std::shared_ptr<mrpt::typemeta::string_literal<18>>> cl(M("mrpt::typemeta"), "string_literal_18_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::string_literal<18> const &o){ return new mrpt::typemeta::string_literal<18>(o); } ) );
		cl.def("size", (std::size_t (mrpt::typemeta::string_literal<18>::*)() const) &mrpt::typemeta::string_literal<18>::size, "C++: mrpt::typemeta::string_literal<18>::size() const --> std::size_t");
		cl.def("__getitem__", (char (mrpt::typemeta::string_literal<18>::*)(int) const) &mrpt::typemeta::string_literal<18>::operator[], "C++: mrpt::typemeta::string_literal<18>::operator[](int) const --> char", pybind11::arg("i"));
		cl.def("c_str", (const char * (mrpt::typemeta::string_literal<18>::*)() const) &mrpt::typemeta::string_literal<18>::c_str, "C++: mrpt::typemeta::string_literal<18>::c_str() const --> const char *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::typemeta::string_literal file:mrpt/typemeta/static_string.h line:25
		pybind11::class_<mrpt::typemeta::string_literal<9>, std::shared_ptr<mrpt::typemeta::string_literal<9>>> cl(M("mrpt::typemeta"), "string_literal_9_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::string_literal<9> const &o){ return new mrpt::typemeta::string_literal<9>(o); } ) );
		cl.def("size", (std::size_t (mrpt::typemeta::string_literal<9>::*)() const) &mrpt::typemeta::string_literal<9>::size, "C++: mrpt::typemeta::string_literal<9>::size() const --> std::size_t");
		cl.def("__getitem__", (char (mrpt::typemeta::string_literal<9>::*)(int) const) &mrpt::typemeta::string_literal<9>::operator[], "C++: mrpt::typemeta::string_literal<9>::operator[](int) const --> char", pybind11::arg("i"));
		cl.def("c_str", (const char * (mrpt::typemeta::string_literal<9>::*)() const) &mrpt::typemeta::string_literal<9>::c_str, "C++: mrpt::typemeta::string_literal<9>::c_str() const --> const char *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::typemeta::string_literal file:mrpt/typemeta/static_string.h line:25
		pybind11::class_<mrpt::typemeta::string_literal<20>, std::shared_ptr<mrpt::typemeta::string_literal<20>>> cl(M("mrpt::typemeta"), "string_literal_20_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::string_literal<20> const &o){ return new mrpt::typemeta::string_literal<20>(o); } ) );
		cl.def("size", (std::size_t (mrpt::typemeta::string_literal<20>::*)() const) &mrpt::typemeta::string_literal<20>::size, "C++: mrpt::typemeta::string_literal<20>::size() const --> std::size_t");
		cl.def("__getitem__", (char (mrpt::typemeta::string_literal<20>::*)(int) const) &mrpt::typemeta::string_literal<20>::operator[], "C++: mrpt::typemeta::string_literal<20>::operator[](int) const --> char", pybind11::arg("i"));
		cl.def("c_str", (const char * (mrpt::typemeta::string_literal<20>::*)() const) &mrpt::typemeta::string_literal<20>::c_str, "C++: mrpt::typemeta::string_literal<20>::c_str() const --> const char *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::typemeta::string_literal file:mrpt/typemeta/static_string.h line:25
		pybind11::class_<mrpt::typemeta::string_literal<15>, std::shared_ptr<mrpt::typemeta::string_literal<15>>> cl(M("mrpt::typemeta"), "string_literal_15_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::string_literal<15> const &o){ return new mrpt::typemeta::string_literal<15>(o); } ) );
		cl.def("size", (std::size_t (mrpt::typemeta::string_literal<15>::*)() const) &mrpt::typemeta::string_literal<15>::size, "C++: mrpt::typemeta::string_literal<15>::size() const --> std::size_t");
		cl.def("__getitem__", (char (mrpt::typemeta::string_literal<15>::*)(int) const) &mrpt::typemeta::string_literal<15>::operator[], "C++: mrpt::typemeta::string_literal<15>::operator[](int) const --> char", pybind11::arg("i"));
		cl.def("c_str", (const char * (mrpt::typemeta::string_literal<15>::*)() const) &mrpt::typemeta::string_literal<15>::c_str, "C++: mrpt::typemeta::string_literal<15>::c_str() const --> const char *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::typemeta::string_literal file:mrpt/typemeta/static_string.h line:25
		pybind11::class_<mrpt::typemeta::string_literal<33>, std::shared_ptr<mrpt::typemeta::string_literal<33>>> cl(M("mrpt::typemeta"), "string_literal_33_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::string_literal<33> const &o){ return new mrpt::typemeta::string_literal<33>(o); } ) );
		cl.def("size", (std::size_t (mrpt::typemeta::string_literal<33>::*)() const) &mrpt::typemeta::string_literal<33>::size, "C++: mrpt::typemeta::string_literal<33>::size() const --> std::size_t");
		cl.def("__getitem__", (char (mrpt::typemeta::string_literal<33>::*)(int) const) &mrpt::typemeta::string_literal<33>::operator[], "C++: mrpt::typemeta::string_literal<33>::operator[](int) const --> char", pybind11::arg("i"));
		cl.def("c_str", (const char * (mrpt::typemeta::string_literal<33>::*)() const) &mrpt::typemeta::string_literal<33>::c_str, "C++: mrpt::typemeta::string_literal<33>::c_str() const --> const char *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::typemeta::string_literal file:mrpt/typemeta/static_string.h line:25
		pybind11::class_<mrpt::typemeta::string_literal<31>, std::shared_ptr<mrpt::typemeta::string_literal<31>>> cl(M("mrpt::typemeta"), "string_literal_31_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::string_literal<31> const &o){ return new mrpt::typemeta::string_literal<31>(o); } ) );
		cl.def("size", (std::size_t (mrpt::typemeta::string_literal<31>::*)() const) &mrpt::typemeta::string_literal<31>::size, "C++: mrpt::typemeta::string_literal<31>::size() const --> std::size_t");
		cl.def("__getitem__", (char (mrpt::typemeta::string_literal<31>::*)(int) const) &mrpt::typemeta::string_literal<31>::operator[], "C++: mrpt::typemeta::string_literal<31>::operator[](int) const --> char", pybind11::arg("i"));
		cl.def("c_str", (const char * (mrpt::typemeta::string_literal<31>::*)() const) &mrpt::typemeta::string_literal<31>::c_str, "C++: mrpt::typemeta::string_literal<31>::c_str() const --> const char *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::typemeta::string_literal file:mrpt/typemeta/static_string.h line:25
		pybind11::class_<mrpt::typemeta::string_literal<28>, std::shared_ptr<mrpt::typemeta::string_literal<28>>> cl(M("mrpt::typemeta"), "string_literal_28_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::string_literal<28> const &o){ return new mrpt::typemeta::string_literal<28>(o); } ) );
		cl.def("size", (std::size_t (mrpt::typemeta::string_literal<28>::*)() const) &mrpt::typemeta::string_literal<28>::size, "C++: mrpt::typemeta::string_literal<28>::size() const --> std::size_t");
		cl.def("__getitem__", (char (mrpt::typemeta::string_literal<28>::*)(int) const) &mrpt::typemeta::string_literal<28>::operator[], "C++: mrpt::typemeta::string_literal<28>::operator[](int) const --> char", pybind11::arg("i"));
		cl.def("c_str", (const char * (mrpt::typemeta::string_literal<28>::*)() const) &mrpt::typemeta::string_literal<28>::c_str, "C++: mrpt::typemeta::string_literal<28>::c_str() const --> const char *", pybind11::return_value_policy::automatic);
	}
}
