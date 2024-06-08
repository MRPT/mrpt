#include <mrpt/core/integer_select.h>
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

void bind_mrpt_core_integer_select(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::int_select_by_bytecount file:mrpt/core/integer_select.h line:26
		pybind11::class_<mrpt::int_select_by_bytecount<1U>, std::shared_ptr<mrpt::int_select_by_bytecount<1U>>> cl(M("mrpt"), "int_select_by_bytecount_1U_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::int_select_by_bytecount<1U>(); } ) );
	}
	{ // mrpt::int_select_by_bytecount file:mrpt/core/integer_select.h line:31
		pybind11::class_<mrpt::int_select_by_bytecount<2U>, std::shared_ptr<mrpt::int_select_by_bytecount<2U>>> cl(M("mrpt"), "int_select_by_bytecount_2U_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::int_select_by_bytecount<2U>(); } ) );
	}
	{ // mrpt::int_select_by_bytecount file:mrpt/core/integer_select.h line:36
		pybind11::class_<mrpt::int_select_by_bytecount<3U>, std::shared_ptr<mrpt::int_select_by_bytecount<3U>>> cl(M("mrpt"), "int_select_by_bytecount_3U_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::int_select_by_bytecount<3U>(); } ) );
	}
	{ // mrpt::int_select_by_bytecount file:mrpt/core/integer_select.h line:41
		pybind11::class_<mrpt::int_select_by_bytecount<4U>, std::shared_ptr<mrpt::int_select_by_bytecount<4U>>> cl(M("mrpt"), "int_select_by_bytecount_4U_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::int_select_by_bytecount<4U>(); } ) );
	}
	{ // mrpt::int_select_by_bytecount file:mrpt/core/integer_select.h line:46
		pybind11::class_<mrpt::int_select_by_bytecount<8U>, std::shared_ptr<mrpt::int_select_by_bytecount<8U>>> cl(M("mrpt"), "int_select_by_bytecount_8U_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::int_select_by_bytecount<8U>(); } ) );
	}
	{ // mrpt::uint_select_by_bytecount file:mrpt/core/integer_select.h line:61
		pybind11::class_<mrpt::uint_select_by_bytecount<1U>, std::shared_ptr<mrpt::uint_select_by_bytecount<1U>>> cl(M("mrpt"), "uint_select_by_bytecount_1U_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::uint_select_by_bytecount<1U>(); } ) );
	}
	{ // mrpt::uint_select_by_bytecount file:mrpt/core/integer_select.h line:66
		pybind11::class_<mrpt::uint_select_by_bytecount<2U>, std::shared_ptr<mrpt::uint_select_by_bytecount<2U>>> cl(M("mrpt"), "uint_select_by_bytecount_2U_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::uint_select_by_bytecount<2U>(); } ) );
	}
	{ // mrpt::uint_select_by_bytecount file:mrpt/core/integer_select.h line:71
		pybind11::class_<mrpt::uint_select_by_bytecount<3U>, std::shared_ptr<mrpt::uint_select_by_bytecount<3U>>> cl(M("mrpt"), "uint_select_by_bytecount_3U_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::uint_select_by_bytecount<3U>(); } ) );
	}
	{ // mrpt::uint_select_by_bytecount file:mrpt/core/integer_select.h line:76
		pybind11::class_<mrpt::uint_select_by_bytecount<4U>, std::shared_ptr<mrpt::uint_select_by_bytecount<4U>>> cl(M("mrpt"), "uint_select_by_bytecount_4U_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::uint_select_by_bytecount<4U>(); } ) );
	}
	{ // mrpt::uint_select_by_bytecount file:mrpt/core/integer_select.h line:81
		pybind11::class_<mrpt::uint_select_by_bytecount<8U>, std::shared_ptr<mrpt::uint_select_by_bytecount<8U>>> cl(M("mrpt"), "uint_select_by_bytecount_8U_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::uint_select_by_bytecount<8U>(); } ) );
	}
}
