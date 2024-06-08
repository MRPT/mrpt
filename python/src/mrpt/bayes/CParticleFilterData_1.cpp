#include <iterator>
#include <memory>
#include <mrpt/bayes/CParticleFilterData.h>
#include <mrpt/bayes/CProbabilityParticle.h>
#include <mrpt/maps/CMultiMetricMapPDF.h>
#include <mrpt/maps/TMetricMapInitializer.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <sstream> // __str__
#include <string>
#include <vector>

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

void bind_mrpt_bayes_CParticleFilterData_1(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::bayes::CParticleFilterData file:mrpt/bayes/CParticleFilterData.h line:169
		pybind11::class_<mrpt::bayes::CParticleFilterData<mrpt::math::TPose3D,mrpt::bayes::particle_storage_mode::VALUE>, std::shared_ptr<mrpt::bayes::CParticleFilterData<mrpt::math::TPose3D,mrpt::bayes::particle_storage_mode::VALUE>>> cl(M("mrpt::bayes"), "CParticleFilterData_mrpt_math_TPose3D_mrpt_bayes_particle_storage_mode_VALUE_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::bayes::CParticleFilterData<mrpt::math::TPose3D,mrpt::bayes::particle_storage_mode::VALUE>(); } ) );
		cl.def( pybind11::init( [](mrpt::bayes::CParticleFilterData<mrpt::math::TPose3D,mrpt::bayes::particle_storage_mode::VALUE> const &o){ return new mrpt::bayes::CParticleFilterData<mrpt::math::TPose3D,mrpt::bayes::particle_storage_mode::VALUE>(o); } ) );
		cl.def_readwrite("m_particles", &mrpt::bayes::CParticleFilterData<mrpt::math::TPose3D,mrpt::bayes::particle_storage_mode::VALUE>::m_particles);
		cl.def("clearParticles", (void (mrpt::bayes::CParticleFilterData<mrpt::math::TPose3D,mrpt::bayes::particle_storage_mode::VALUE>::*)()) &mrpt::bayes::CParticleFilterData<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE>::clearParticles, "C++: mrpt::bayes::CParticleFilterData<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE>::clearParticles() --> void");
		cl.def("getMostLikelyParticle", (const struct mrpt::bayes::CProbabilityParticle<struct mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> * (mrpt::bayes::CParticleFilterData<mrpt::math::TPose3D,mrpt::bayes::particle_storage_mode::VALUE>::*)() const) &mrpt::bayes::CParticleFilterData<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE>::getMostLikelyParticle, "C++: mrpt::bayes::CParticleFilterData<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE>::getMostLikelyParticle() const --> const struct mrpt::bayes::CProbabilityParticle<struct mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> *", pybind11::return_value_policy::automatic);
		cl.def("assign", (class mrpt::bayes::CParticleFilterData<struct mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> & (mrpt::bayes::CParticleFilterData<mrpt::math::TPose3D,mrpt::bayes::particle_storage_mode::VALUE>::*)(const class mrpt::bayes::CParticleFilterData<struct mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> &)) &mrpt::bayes::CParticleFilterData<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE>::operator=, "C++: mrpt::bayes::CParticleFilterData<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE>::operator=(const class mrpt::bayes::CParticleFilterData<struct mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> &) --> class mrpt::bayes::CParticleFilterData<struct mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::bayes::CParticleFilterData file:mrpt/bayes/CParticleFilterData.h line:169
		pybind11::class_<mrpt::bayes::CParticleFilterData<mrpt::math::TPose2D,mrpt::bayes::particle_storage_mode::VALUE>, std::shared_ptr<mrpt::bayes::CParticleFilterData<mrpt::math::TPose2D,mrpt::bayes::particle_storage_mode::VALUE>>> cl(M("mrpt::bayes"), "CParticleFilterData_mrpt_math_TPose2D_mrpt_bayes_particle_storage_mode_VALUE_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::bayes::CParticleFilterData<mrpt::math::TPose2D,mrpt::bayes::particle_storage_mode::VALUE>(); } ) );
		cl.def( pybind11::init( [](mrpt::bayes::CParticleFilterData<mrpt::math::TPose2D,mrpt::bayes::particle_storage_mode::VALUE> const &o){ return new mrpt::bayes::CParticleFilterData<mrpt::math::TPose2D,mrpt::bayes::particle_storage_mode::VALUE>(o); } ) );
		cl.def_readwrite("m_particles", &mrpt::bayes::CParticleFilterData<mrpt::math::TPose2D,mrpt::bayes::particle_storage_mode::VALUE>::m_particles);
		cl.def("clearParticles", (void (mrpt::bayes::CParticleFilterData<mrpt::math::TPose2D,mrpt::bayes::particle_storage_mode::VALUE>::*)()) &mrpt::bayes::CParticleFilterData<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE>::clearParticles, "C++: mrpt::bayes::CParticleFilterData<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE>::clearParticles() --> void");
		cl.def("getMostLikelyParticle", (const struct mrpt::bayes::CProbabilityParticle<struct mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> * (mrpt::bayes::CParticleFilterData<mrpt::math::TPose2D,mrpt::bayes::particle_storage_mode::VALUE>::*)() const) &mrpt::bayes::CParticleFilterData<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE>::getMostLikelyParticle, "C++: mrpt::bayes::CParticleFilterData<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE>::getMostLikelyParticle() const --> const struct mrpt::bayes::CProbabilityParticle<struct mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> *", pybind11::return_value_policy::automatic);
		cl.def("assign", (class mrpt::bayes::CParticleFilterData<struct mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> & (mrpt::bayes::CParticleFilterData<mrpt::math::TPose2D,mrpt::bayes::particle_storage_mode::VALUE>::*)(const class mrpt::bayes::CParticleFilterData<struct mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> &)) &mrpt::bayes::CParticleFilterData<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE>::operator=, "C++: mrpt::bayes::CParticleFilterData<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE>::operator=(const class mrpt::bayes::CParticleFilterData<struct mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> &) --> class mrpt::bayes::CParticleFilterData<struct mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::bayes::CParticleFilterData file:mrpt/bayes/CParticleFilterData.h line:169
		pybind11::class_<mrpt::bayes::CParticleFilterData<mrpt::maps::CRBPFParticleData,mrpt::bayes::particle_storage_mode::POINTER>, std::shared_ptr<mrpt::bayes::CParticleFilterData<mrpt::maps::CRBPFParticleData,mrpt::bayes::particle_storage_mode::POINTER>>> cl(M("mrpt::bayes"), "CParticleFilterData_mrpt_maps_CRBPFParticleData_mrpt_bayes_particle_storage_mode_POINTER_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::bayes::CParticleFilterData<mrpt::maps::CRBPFParticleData,mrpt::bayes::particle_storage_mode::POINTER>(); } ) );
		cl.def( pybind11::init( [](mrpt::bayes::CParticleFilterData<mrpt::maps::CRBPFParticleData,mrpt::bayes::particle_storage_mode::POINTER> const &o){ return new mrpt::bayes::CParticleFilterData<mrpt::maps::CRBPFParticleData,mrpt::bayes::particle_storage_mode::POINTER>(o); } ) );
		cl.def_readwrite("m_particles", &mrpt::bayes::CParticleFilterData<mrpt::maps::CRBPFParticleData,mrpt::bayes::particle_storage_mode::POINTER>::m_particles);
		cl.def("clearParticles", (void (mrpt::bayes::CParticleFilterData<mrpt::maps::CRBPFParticleData,mrpt::bayes::particle_storage_mode::POINTER>::*)()) &mrpt::bayes::CParticleFilterData<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>::clearParticles, "C++: mrpt::bayes::CParticleFilterData<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>::clearParticles() --> void");
		cl.def("assign", (class mrpt::bayes::CParticleFilterData<class mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER> & (mrpt::bayes::CParticleFilterData<mrpt::maps::CRBPFParticleData,mrpt::bayes::particle_storage_mode::POINTER>::*)(const class mrpt::bayes::CParticleFilterData<class mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER> &)) &mrpt::bayes::CParticleFilterData<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>::operator=, "C++: mrpt::bayes::CParticleFilterData<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>::operator=(const class mrpt::bayes::CParticleFilterData<class mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER> &) --> class mrpt::bayes::CParticleFilterData<class mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::bayes::CParticleFilterData file:mrpt/bayes/CParticleFilterData.h line:169
		pybind11::class_<mrpt::bayes::CParticleFilterData<mrpt::math::TPoint3D_<float>,mrpt::bayes::particle_storage_mode::POINTER>, std::shared_ptr<mrpt::bayes::CParticleFilterData<mrpt::math::TPoint3D_<float>,mrpt::bayes::particle_storage_mode::POINTER>>> cl(M("mrpt::bayes"), "CParticleFilterData_mrpt_math_TPoint3D_float_mrpt_bayes_particle_storage_mode_POINTER_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::bayes::CParticleFilterData<mrpt::math::TPoint3D_<float>,mrpt::bayes::particle_storage_mode::POINTER>(); } ) );
		cl.def( pybind11::init( [](mrpt::bayes::CParticleFilterData<mrpt::math::TPoint3D_<float>,mrpt::bayes::particle_storage_mode::POINTER> const &o){ return new mrpt::bayes::CParticleFilterData<mrpt::math::TPoint3D_<float>,mrpt::bayes::particle_storage_mode::POINTER>(o); } ) );
		cl.def_readwrite("m_particles", &mrpt::bayes::CParticleFilterData<mrpt::math::TPoint3D_<float>,mrpt::bayes::particle_storage_mode::POINTER>::m_particles);
		cl.def("clearParticles", (void (mrpt::bayes::CParticleFilterData<mrpt::math::TPoint3D_<float>,mrpt::bayes::particle_storage_mode::POINTER>::*)()) &mrpt::bayes::CParticleFilterData<mrpt::math::TPoint3D_<float>, mrpt::bayes::particle_storage_mode::POINTER>::clearParticles, "C++: mrpt::bayes::CParticleFilterData<mrpt::math::TPoint3D_<float>, mrpt::bayes::particle_storage_mode::POINTER>::clearParticles() --> void");
		cl.def("getMostLikelyParticle", (const struct mrpt::bayes::CProbabilityParticle<struct mrpt::math::TPoint3D_<float>, mrpt::bayes::particle_storage_mode::POINTER> * (mrpt::bayes::CParticleFilterData<mrpt::math::TPoint3D_<float>,mrpt::bayes::particle_storage_mode::POINTER>::*)() const) &mrpt::bayes::CParticleFilterData<mrpt::math::TPoint3D_<float>, mrpt::bayes::particle_storage_mode::POINTER>::getMostLikelyParticle, "C++: mrpt::bayes::CParticleFilterData<mrpt::math::TPoint3D_<float>, mrpt::bayes::particle_storage_mode::POINTER>::getMostLikelyParticle() const --> const struct mrpt::bayes::CProbabilityParticle<struct mrpt::math::TPoint3D_<float>, mrpt::bayes::particle_storage_mode::POINTER> *", pybind11::return_value_policy::automatic);
		cl.def("assign", (class mrpt::bayes::CParticleFilterData<struct mrpt::math::TPoint3D_<float>, mrpt::bayes::particle_storage_mode::POINTER> & (mrpt::bayes::CParticleFilterData<mrpt::math::TPoint3D_<float>,mrpt::bayes::particle_storage_mode::POINTER>::*)(const class mrpt::bayes::CParticleFilterData<struct mrpt::math::TPoint3D_<float>, mrpt::bayes::particle_storage_mode::POINTER> &)) &mrpt::bayes::CParticleFilterData<mrpt::math::TPoint3D_<float>, mrpt::bayes::particle_storage_mode::POINTER>::operator=, "C++: mrpt::bayes::CParticleFilterData<mrpt::math::TPoint3D_<float>, mrpt::bayes::particle_storage_mode::POINTER>::operator=(const class mrpt::bayes::CParticleFilterData<struct mrpt::math::TPoint3D_<float>, mrpt::bayes::particle_storage_mode::POINTER> &) --> class mrpt::bayes::CParticleFilterData<struct mrpt::math::TPoint3D_<float>, mrpt::bayes::particle_storage_mode::POINTER> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
