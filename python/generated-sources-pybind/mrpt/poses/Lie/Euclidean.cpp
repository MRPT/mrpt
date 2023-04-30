#include <mrpt/poses/Lie/Euclidean.h>
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

void bind_mrpt_poses_Lie_Euclidean(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::poses::Lie::EuclideanBase file:mrpt/poses/Lie/Euclidean.h line:25
		pybind11::class_<mrpt::poses::Lie::EuclideanBase<2>, std::shared_ptr<mrpt::poses::Lie::EuclideanBase<2>>> cl(M("mrpt::poses::Lie"), "EuclideanBase_2_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::poses::Lie::EuclideanBase<2>(); } ) );
	}
	{ // mrpt::poses::Lie::EuclideanBase file:mrpt/poses/Lie/Euclidean.h line:25
		pybind11::class_<mrpt::poses::Lie::EuclideanBase<3>, std::shared_ptr<mrpt::poses::Lie::EuclideanBase<3>>> cl(M("mrpt::poses::Lie"), "EuclideanBase_3_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::poses::Lie::EuclideanBase<3>(); } ) );
	}
	{ // mrpt::poses::Lie::Euclidean file:mrpt/poses/Lie/Euclidean.h line:32
		pybind11::class_<mrpt::poses::Lie::Euclidean<2>, std::shared_ptr<mrpt::poses::Lie::Euclidean<2>>, mrpt::poses::Lie::EuclideanBase<2>> cl(M("mrpt::poses::Lie"), "Euclidean_2_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::poses::Lie::Euclidean<2>(); } ) );
	}
	{ // mrpt::poses::Lie::Euclidean file:mrpt/poses/Lie/Euclidean.h line:39
		pybind11::class_<mrpt::poses::Lie::Euclidean<3>, std::shared_ptr<mrpt::poses::Lie::Euclidean<3>>, mrpt::poses::Lie::EuclideanBase<3>> cl(M("mrpt::poses::Lie"), "Euclidean_3_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::poses::Lie::Euclidean<3>(); } ) );
	}
}
