#include <mrpt/poses/Lie/Euclidean.h>
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

void bind_mrpt_poses_Lie_Euclidean(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::poses::Lie::EuclideanBase file:mrpt/poses/Lie/Euclidean.h line:25
		pybind11::class_<mrpt::poses::Lie::EuclideanBase<2U>, std::shared_ptr<mrpt::poses::Lie::EuclideanBase<2U>>> cl(M("mrpt::poses::Lie"), "EuclideanBase_2U_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::poses::Lie::EuclideanBase<2U>(); } ) );
	}
	{ // mrpt::poses::Lie::EuclideanBase file:mrpt/poses/Lie/Euclidean.h line:25
		pybind11::class_<mrpt::poses::Lie::EuclideanBase<3U>, std::shared_ptr<mrpt::poses::Lie::EuclideanBase<3U>>> cl(M("mrpt::poses::Lie"), "EuclideanBase_3U_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::poses::Lie::EuclideanBase<3U>(); } ) );
	}
	{ // mrpt::poses::Lie::Euclidean file:mrpt/poses/Lie/Euclidean.h line:32
		pybind11::class_<mrpt::poses::Lie::Euclidean<2U>, std::shared_ptr<mrpt::poses::Lie::Euclidean<2U>>, mrpt::poses::Lie::EuclideanBase<2U>> cl(M("mrpt::poses::Lie"), "Euclidean_2U_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::poses::Lie::Euclidean<2U>(); } ) );
	}
	{ // mrpt::poses::Lie::Euclidean file:mrpt/poses/Lie/Euclidean.h line:39
		pybind11::class_<mrpt::poses::Lie::Euclidean<3U>, std::shared_ptr<mrpt::poses::Lie::Euclidean<3U>>, mrpt::poses::Lie::EuclideanBase<3U>> cl(M("mrpt::poses::Lie"), "Euclidean_3U_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::poses::Lie::Euclidean<3U>(); } ) );
	}
}
