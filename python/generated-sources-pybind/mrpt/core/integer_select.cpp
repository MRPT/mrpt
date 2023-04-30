#include <mrpt/core/integer_select.h>
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

void bind_mrpt_core_integer_select(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::int_select_by_bytecount file:mrpt/core/integer_select.h line:26
		pybind11::class_<mrpt::int_select_by_bytecount<1>, std::shared_ptr<mrpt::int_select_by_bytecount<1>>> cl(M("mrpt"), "int_select_by_bytecount_1_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::int_select_by_bytecount<1>(); } ) );
	}
	{ // mrpt::int_select_by_bytecount file:mrpt/core/integer_select.h line:31
		pybind11::class_<mrpt::int_select_by_bytecount<2>, std::shared_ptr<mrpt::int_select_by_bytecount<2>>> cl(M("mrpt"), "int_select_by_bytecount_2_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::int_select_by_bytecount<2>(); } ) );
	}
	{ // mrpt::int_select_by_bytecount file:mrpt/core/integer_select.h line:36
		pybind11::class_<mrpt::int_select_by_bytecount<3>, std::shared_ptr<mrpt::int_select_by_bytecount<3>>> cl(M("mrpt"), "int_select_by_bytecount_3_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::int_select_by_bytecount<3>(); } ) );
	}
	{ // mrpt::int_select_by_bytecount file:mrpt/core/integer_select.h line:41
		pybind11::class_<mrpt::int_select_by_bytecount<4>, std::shared_ptr<mrpt::int_select_by_bytecount<4>>> cl(M("mrpt"), "int_select_by_bytecount_4_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::int_select_by_bytecount<4>(); } ) );
	}
	{ // mrpt::int_select_by_bytecount file:mrpt/core/integer_select.h line:46
		pybind11::class_<mrpt::int_select_by_bytecount<8>, std::shared_ptr<mrpt::int_select_by_bytecount<8>>> cl(M("mrpt"), "int_select_by_bytecount_8_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::int_select_by_bytecount<8>(); } ) );
	}
	{ // mrpt::uint_select_by_bytecount file:mrpt/core/integer_select.h line:62
		pybind11::class_<mrpt::uint_select_by_bytecount<1>, std::shared_ptr<mrpt::uint_select_by_bytecount<1>>> cl(M("mrpt"), "uint_select_by_bytecount_1_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::uint_select_by_bytecount<1>(); } ) );
	}
	{ // mrpt::uint_select_by_bytecount file:mrpt/core/integer_select.h line:67
		pybind11::class_<mrpt::uint_select_by_bytecount<2>, std::shared_ptr<mrpt::uint_select_by_bytecount<2>>> cl(M("mrpt"), "uint_select_by_bytecount_2_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::uint_select_by_bytecount<2>(); } ) );
	}
	{ // mrpt::uint_select_by_bytecount file:mrpt/core/integer_select.h line:72
		pybind11::class_<mrpt::uint_select_by_bytecount<3>, std::shared_ptr<mrpt::uint_select_by_bytecount<3>>> cl(M("mrpt"), "uint_select_by_bytecount_3_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::uint_select_by_bytecount<3>(); } ) );
	}
	{ // mrpt::uint_select_by_bytecount file:mrpt/core/integer_select.h line:77
		pybind11::class_<mrpt::uint_select_by_bytecount<4>, std::shared_ptr<mrpt::uint_select_by_bytecount<4>>> cl(M("mrpt"), "uint_select_by_bytecount_4_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::uint_select_by_bytecount<4>(); } ) );
	}
	{ // mrpt::uint_select_by_bytecount file:mrpt/core/integer_select.h line:82
		pybind11::class_<mrpt::uint_select_by_bytecount<8>, std::shared_ptr<mrpt::uint_select_by_bytecount<8>>> cl(M("mrpt"), "uint_select_by_bytecount_8_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::uint_select_by_bytecount<8>(); } ) );
	}
}
