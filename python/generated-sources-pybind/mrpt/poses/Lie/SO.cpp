#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/poses/Lie/SO.h>
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

void bind_mrpt_poses_Lie_SO(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::poses::Lie::SO file:mrpt/poses/Lie/SO.h line:27
		pybind11::class_<mrpt::poses::Lie::SO<3>, std::shared_ptr<mrpt::poses::Lie::SO<3>>> cl(M("mrpt::poses::Lie"), "SO_3_t", "Traits for SO(3), rotations in R^3 space.\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::poses::Lie::SO<3>(); } ) );
	}
	{ // mrpt::poses::Lie::SO file:mrpt/poses/Lie/SO.h line:88
		pybind11::class_<mrpt::poses::Lie::SO<2>, std::shared_ptr<mrpt::poses::Lie::SO<2>>> cl(M("mrpt::poses::Lie"), "SO_2_t", "Traits for SO(2), rotations in R^2 space.\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::poses::Lie::SO<2>(); } ) );
	}
}
