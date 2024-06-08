#include <iterator>
#include <memory>
#include <mrpt/bayes/CProbabilityParticle.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <optional>
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

void bind_mrpt_bayes_CProbabilityParticle_1(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::bayes::CProbabilityParticle file:mrpt/bayes/CProbabilityParticle.h line:54
		pybind11::class_<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D,mrpt::bayes::particle_storage_mode::VALUE>, std::shared_ptr<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D,mrpt::bayes::particle_storage_mode::VALUE>>, mrpt::bayes::CProbabilityParticleBase> cl(M("mrpt::bayes"), "CProbabilityParticle_mrpt_math_TPose3D_mrpt_bayes_particle_storage_mode_VALUE_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D,mrpt::bayes::particle_storage_mode::VALUE>(); } ) );
		cl.def( pybind11::init<const struct mrpt::math::TPose3D &, const double>(), pybind11::arg("data"), pybind11::arg("logw") );

		cl.def_readwrite("d", &mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D,mrpt::bayes::particle_storage_mode::VALUE>::d);
		cl.def_readwrite("log_w", &mrpt::bayes::CProbabilityParticleBase::log_w);
		cl.def("assign", (struct mrpt::bayes::CProbabilityParticleBase & (mrpt::bayes::CProbabilityParticleBase::*)(const struct mrpt::bayes::CProbabilityParticleBase &)) &mrpt::bayes::CProbabilityParticleBase::operator=, "C++: mrpt::bayes::CProbabilityParticleBase::operator=(const struct mrpt::bayes::CProbabilityParticleBase &) --> struct mrpt::bayes::CProbabilityParticleBase &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::bayes::CProbabilityParticle file:mrpt/bayes/CProbabilityParticle.h line:54
		pybind11::class_<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D,mrpt::bayes::particle_storage_mode::VALUE>, std::shared_ptr<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D,mrpt::bayes::particle_storage_mode::VALUE>>, mrpt::bayes::CProbabilityParticleBase> cl(M("mrpt::bayes"), "CProbabilityParticle_mrpt_math_TPose2D_mrpt_bayes_particle_storage_mode_VALUE_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D,mrpt::bayes::particle_storage_mode::VALUE>(); } ) );
		cl.def( pybind11::init<const struct mrpt::math::TPose2D &, const double>(), pybind11::arg("data"), pybind11::arg("logw") );

		cl.def_readwrite("d", &mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D,mrpt::bayes::particle_storage_mode::VALUE>::d);
		cl.def_readwrite("log_w", &mrpt::bayes::CProbabilityParticleBase::log_w);
		cl.def("assign", (struct mrpt::bayes::CProbabilityParticleBase & (mrpt::bayes::CProbabilityParticleBase::*)(const struct mrpt::bayes::CProbabilityParticleBase &)) &mrpt::bayes::CProbabilityParticleBase::operator=, "C++: mrpt::bayes::CProbabilityParticleBase::operator=(const struct mrpt::bayes::CProbabilityParticleBase &) --> struct mrpt::bayes::CProbabilityParticleBase &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::bayes::CProbabilityParticle file:mrpt/bayes/CProbabilityParticle.h line:46
		pybind11::class_<mrpt::bayes::CProbabilityParticle<mrpt::math::TPoint3D_<float>,mrpt::bayes::particle_storage_mode::POINTER>, std::shared_ptr<mrpt::bayes::CProbabilityParticle<mrpt::math::TPoint3D_<float>,mrpt::bayes::particle_storage_mode::POINTER>>, mrpt::bayes::CProbabilityParticleBase> cl(M("mrpt::bayes"), "CProbabilityParticle_mrpt_math_TPoint3D_float_mrpt_bayes_particle_storage_mode_POINTER_t", "");
		cl.def( pybind11::init( [](mrpt::bayes::CProbabilityParticle<mrpt::math::TPoint3D_<float>,mrpt::bayes::particle_storage_mode::POINTER> const &o){ return new mrpt::bayes::CProbabilityParticle<mrpt::math::TPoint3D_<float>,mrpt::bayes::particle_storage_mode::POINTER>(o); } ) );
		cl.def( pybind11::init( [](){ return new mrpt::bayes::CProbabilityParticle<mrpt::math::TPoint3D_<float>,mrpt::bayes::particle_storage_mode::POINTER>(); } ) );
		cl.def_readwrite("d", &mrpt::bayes::CProbabilityParticle<mrpt::math::TPoint3D_<float>,mrpt::bayes::particle_storage_mode::POINTER>::d);
		cl.def("assign", (struct mrpt::bayes::CProbabilityParticle<struct mrpt::math::TPoint3D_<float>, mrpt::bayes::particle_storage_mode::POINTER> & (mrpt::bayes::CProbabilityParticle<mrpt::math::TPoint3D_<float>,mrpt::bayes::particle_storage_mode::POINTER>::*)(const struct mrpt::bayes::CProbabilityParticle<struct mrpt::math::TPoint3D_<float>, mrpt::bayes::particle_storage_mode::POINTER> &)) &mrpt::bayes::CProbabilityParticle<mrpt::math::TPoint3D_<float>, mrpt::bayes::particle_storage_mode::POINTER>::operator=, "C++: mrpt::bayes::CProbabilityParticle<mrpt::math::TPoint3D_<float>, mrpt::bayes::particle_storage_mode::POINTER>::operator=(const struct mrpt::bayes::CProbabilityParticle<struct mrpt::math::TPoint3D_<float>, mrpt::bayes::particle_storage_mode::POINTER> &) --> struct mrpt::bayes::CProbabilityParticle<struct mrpt::math::TPoint3D_<float>, mrpt::bayes::particle_storage_mode::POINTER> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		cl.def_readwrite("log_w", &mrpt::bayes::CProbabilityParticleBase::log_w);
		cl.def("assign", (struct mrpt::bayes::CProbabilityParticleBase & (mrpt::bayes::CProbabilityParticleBase::*)(const struct mrpt::bayes::CProbabilityParticleBase &)) &mrpt::bayes::CProbabilityParticleBase::operator=, "C++: mrpt::bayes::CProbabilityParticleBase::operator=(const struct mrpt::bayes::CProbabilityParticleBase &) --> struct mrpt::bayes::CProbabilityParticleBase &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
