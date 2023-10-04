#include <deque>
#include <iterator>
#include <memory>
#include <mrpt/bayes/CProbabilityParticle.h>
#include <mrpt/maps/CMultiMetricMapPDF.h>
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

PYBIND11_MAKE_OPAQUE(std::deque<mrpt::math::TPose3D>)
PYBIND11_MAKE_OPAQUE(std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>)

void bind_std_stl_deque_1(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // std::deque file:bits/stl_deque.h line:767
		pybind11::class_<std::deque<mrpt::math::TPose3D>, std::shared_ptr<std::deque<mrpt::math::TPose3D>>> cl(M("std"), "deque_mrpt_math_TPose3D_t", "");
		cl.def( pybind11::init( [](){ return new std::deque<mrpt::math::TPose3D>(); } ) );
		cl.def( pybind11::init<const class std::allocator<struct mrpt::math::TPose3D> &>(), pybind11::arg("__a") );

		cl.def( pybind11::init( [](size_t const & a0){ return new std::deque<mrpt::math::TPose3D>(a0); } ), "doc" , pybind11::arg("__n"));
		cl.def( pybind11::init<size_t, const class std::allocator<struct mrpt::math::TPose3D> &>(), pybind11::arg("__n"), pybind11::arg("__a") );

		cl.def( pybind11::init( [](size_t const & a0, const struct mrpt::math::TPose3D & a1){ return new std::deque<mrpt::math::TPose3D>(a0, a1); } ), "doc" , pybind11::arg("__n"), pybind11::arg("__value"));
		cl.def( pybind11::init<size_t, const struct mrpt::math::TPose3D &, const class std::allocator<struct mrpt::math::TPose3D> &>(), pybind11::arg("__n"), pybind11::arg("__value"), pybind11::arg("__a") );

		cl.def( pybind11::init( [](std::deque<mrpt::math::TPose3D> const &o){ return new std::deque<mrpt::math::TPose3D>(o); } ) );
		cl.def( pybind11::init<const class std::deque<struct mrpt::math::TPose3D> &, const class std::allocator<struct mrpt::math::TPose3D> &>(), pybind11::arg("__x"), pybind11::arg("__a") );

		cl.def("assign", (class std::deque<struct mrpt::math::TPose3D> & (std::deque<mrpt::math::TPose3D>::*)(const class std::deque<struct mrpt::math::TPose3D> &)) &std::deque<mrpt::math::TPose3D>::operator=, "C++: std::deque<mrpt::math::TPose3D>::operator=(const class std::deque<struct mrpt::math::TPose3D> &) --> class std::deque<struct mrpt::math::TPose3D> &", pybind11::return_value_policy::automatic, pybind11::arg("__x"));
		cl.def("assign", (void (std::deque<mrpt::math::TPose3D>::*)(size_t, const struct mrpt::math::TPose3D &)) &std::deque<mrpt::math::TPose3D>::assign, "C++: std::deque<mrpt::math::TPose3D>::assign(size_t, const struct mrpt::math::TPose3D &) --> void", pybind11::arg("__n"), pybind11::arg("__val"));
		cl.def("get_allocator", (class std::allocator<struct mrpt::math::TPose3D> (std::deque<mrpt::math::TPose3D>::*)() const) &std::deque<mrpt::math::TPose3D>::get_allocator, "C++: std::deque<mrpt::math::TPose3D>::get_allocator() const --> class std::allocator<struct mrpt::math::TPose3D>");
		cl.def("size", (size_t (std::deque<mrpt::math::TPose3D>::*)() const) &std::deque<mrpt::math::TPose3D>::size, "C++: std::deque<mrpt::math::TPose3D>::size() const --> size_t");
		cl.def("max_size", (size_t (std::deque<mrpt::math::TPose3D>::*)() const) &std::deque<mrpt::math::TPose3D>::max_size, "C++: std::deque<mrpt::math::TPose3D>::max_size() const --> size_t");
		cl.def("resize", (void (std::deque<mrpt::math::TPose3D>::*)(size_t)) &std::deque<mrpt::math::TPose3D>::resize, "C++: std::deque<mrpt::math::TPose3D>::resize(size_t) --> void", pybind11::arg("__new_size"));
		cl.def("resize", (void (std::deque<mrpt::math::TPose3D>::*)(size_t, const struct mrpt::math::TPose3D &)) &std::deque<mrpt::math::TPose3D>::resize, "C++: std::deque<mrpt::math::TPose3D>::resize(size_t, const struct mrpt::math::TPose3D &) --> void", pybind11::arg("__new_size"), pybind11::arg("__x"));
		cl.def("shrink_to_fit", (void (std::deque<mrpt::math::TPose3D>::*)()) &std::deque<mrpt::math::TPose3D>::shrink_to_fit, "C++: std::deque<mrpt::math::TPose3D>::shrink_to_fit() --> void");
		cl.def("empty", (bool (std::deque<mrpt::math::TPose3D>::*)() const) &std::deque<mrpt::math::TPose3D>::empty, "C++: std::deque<mrpt::math::TPose3D>::empty() const --> bool");
		cl.def("__getitem__", (struct mrpt::math::TPose3D & (std::deque<mrpt::math::TPose3D>::*)(size_t)) &std::deque<mrpt::math::TPose3D>::operator[], "C++: std::deque<mrpt::math::TPose3D>::operator[](size_t) --> struct mrpt::math::TPose3D &", pybind11::return_value_policy::automatic, pybind11::arg("__n"));
		cl.def("at", (struct mrpt::math::TPose3D & (std::deque<mrpt::math::TPose3D>::*)(size_t)) &std::deque<mrpt::math::TPose3D>::at, "C++: std::deque<mrpt::math::TPose3D>::at(size_t) --> struct mrpt::math::TPose3D &", pybind11::return_value_policy::automatic, pybind11::arg("__n"));
		cl.def("front", (struct mrpt::math::TPose3D & (std::deque<mrpt::math::TPose3D>::*)()) &std::deque<mrpt::math::TPose3D>::front, "C++: std::deque<mrpt::math::TPose3D>::front() --> struct mrpt::math::TPose3D &", pybind11::return_value_policy::automatic);
		cl.def("back", (struct mrpt::math::TPose3D & (std::deque<mrpt::math::TPose3D>::*)()) &std::deque<mrpt::math::TPose3D>::back, "C++: std::deque<mrpt::math::TPose3D>::back() --> struct mrpt::math::TPose3D &", pybind11::return_value_policy::automatic);
		cl.def("push_front", (void (std::deque<mrpt::math::TPose3D>::*)(const struct mrpt::math::TPose3D &)) &std::deque<mrpt::math::TPose3D>::push_front, "C++: std::deque<mrpt::math::TPose3D>::push_front(const struct mrpt::math::TPose3D &) --> void", pybind11::arg("__x"));
		cl.def("push_back", (void (std::deque<mrpt::math::TPose3D>::*)(const struct mrpt::math::TPose3D &)) &std::deque<mrpt::math::TPose3D>::push_back, "C++: std::deque<mrpt::math::TPose3D>::push_back(const struct mrpt::math::TPose3D &) --> void", pybind11::arg("__x"));
		cl.def("pop_front", (void (std::deque<mrpt::math::TPose3D>::*)()) &std::deque<mrpt::math::TPose3D>::pop_front, "C++: std::deque<mrpt::math::TPose3D>::pop_front() --> void");
		cl.def("pop_back", (void (std::deque<mrpt::math::TPose3D>::*)()) &std::deque<mrpt::math::TPose3D>::pop_back, "C++: std::deque<mrpt::math::TPose3D>::pop_back() --> void");
		cl.def("swap", (void (std::deque<mrpt::math::TPose3D>::*)(class std::deque<struct mrpt::math::TPose3D> &)) &std::deque<mrpt::math::TPose3D>::swap, "C++: std::deque<mrpt::math::TPose3D>::swap(class std::deque<struct mrpt::math::TPose3D> &) --> void", pybind11::arg("__x"));
		cl.def("clear", (void (std::deque<mrpt::math::TPose3D>::*)()) &std::deque<mrpt::math::TPose3D>::clear, "C++: std::deque<mrpt::math::TPose3D>::clear() --> void");

		cl.def("__iter__", [](const std::deque<mrpt::math::TPose3D> &o) {
			return pybind11::make_iterator(o.begin(), o.end());
			}, pybind11::keep_alive<0, 1>());

	}
	{ // std::deque file:bits/stl_deque.h line:767
		pybind11::class_<std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>, std::shared_ptr<std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>>> cl(M("std"), "deque_mrpt_bayes_CProbabilityParticle_mrpt_maps_CRBPFParticleData_mrpt_bayes_particle_storage_mode_POINTER_t", "");
		cl.def( pybind11::init( [](){ return new std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>(); } ) );
		cl.def( pybind11::init<const class std::allocator<struct mrpt::bayes::CProbabilityParticle<class mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER> > &>(), pybind11::arg("__a") );

		cl.def( pybind11::init( [](size_t const & a0){ return new std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>(a0); } ), "doc" , pybind11::arg("__n"));
		cl.def( pybind11::init<size_t, const class std::allocator<struct mrpt::bayes::CProbabilityParticle<class mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER> > &>(), pybind11::arg("__n"), pybind11::arg("__a") );

		cl.def( pybind11::init( [](std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>> const &o){ return new std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>(o); } ) );
		cl.def( pybind11::init<const class std::deque<struct mrpt::bayes::CProbabilityParticle<class mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER> > &, const class std::allocator<struct mrpt::bayes::CProbabilityParticle<class mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER> > &>(), pybind11::arg("__x"), pybind11::arg("__a") );

		cl.def("assign", (class std::deque<struct mrpt::bayes::CProbabilityParticle<class mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER> > & (std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::*)(const class std::deque<struct mrpt::bayes::CProbabilityParticle<class mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER> > &)) &std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::operator=, "C++: std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::operator=(const class std::deque<struct mrpt::bayes::CProbabilityParticle<class mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER> > &) --> class std::deque<struct mrpt::bayes::CProbabilityParticle<class mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER> > &", pybind11::return_value_policy::automatic, pybind11::arg("__x"));
		cl.def("get_allocator", (class std::allocator<struct mrpt::bayes::CProbabilityParticle<class mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER> > (std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::*)() const) &std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::get_allocator, "C++: std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::get_allocator() const --> class std::allocator<struct mrpt::bayes::CProbabilityParticle<class mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER> >");
		cl.def("size", (size_t (std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::*)() const) &std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::size, "C++: std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::size() const --> size_t");
		cl.def("max_size", (size_t (std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::*)() const) &std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::max_size, "C++: std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::max_size() const --> size_t");
		cl.def("resize", (void (std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::*)(size_t)) &std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::resize, "C++: std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::resize(size_t) --> void", pybind11::arg("__new_size"));
		cl.def("shrink_to_fit", (void (std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::*)()) &std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::shrink_to_fit, "C++: std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::shrink_to_fit() --> void");
		cl.def("empty", (bool (std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::*)() const) &std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::empty, "C++: std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::empty() const --> bool");
		cl.def("__getitem__", (struct mrpt::bayes::CProbabilityParticle<class mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER> & (std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::*)(size_t)) &std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::operator[], "C++: std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::operator[](size_t) --> struct mrpt::bayes::CProbabilityParticle<class mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER> &", pybind11::return_value_policy::automatic, pybind11::arg("__n"));
		cl.def("at", (struct mrpt::bayes::CProbabilityParticle<class mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER> & (std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::*)(size_t)) &std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::at, "C++: std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::at(size_t) --> struct mrpt::bayes::CProbabilityParticle<class mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER> &", pybind11::return_value_policy::automatic, pybind11::arg("__n"));
		cl.def("front", (struct mrpt::bayes::CProbabilityParticle<class mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER> & (std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::*)()) &std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::front, "C++: std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::front() --> struct mrpt::bayes::CProbabilityParticle<class mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER> &", pybind11::return_value_policy::automatic);
		cl.def("back", (struct mrpt::bayes::CProbabilityParticle<class mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER> & (std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::*)()) &std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::back, "C++: std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::back() --> struct mrpt::bayes::CProbabilityParticle<class mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER> &", pybind11::return_value_policy::automatic);
		cl.def("pop_front", (void (std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::*)()) &std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::pop_front, "C++: std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::pop_front() --> void");
		cl.def("pop_back", (void (std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::*)()) &std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::pop_back, "C++: std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::pop_back() --> void");
		cl.def("swap", (void (std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::*)(class std::deque<struct mrpt::bayes::CProbabilityParticle<class mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER> > &)) &std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::swap, "C++: std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::swap(class std::deque<struct mrpt::bayes::CProbabilityParticle<class mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER> > &) --> void", pybind11::arg("__x"));
		cl.def("clear", (void (std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::*)()) &std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::clear, "C++: std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>>::clear() --> void");

		cl.def("__iter__", [](const std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER>> &o) {
			return pybind11::make_iterator(o.begin(), o.end());
			}, pybind11::keep_alive<0, 1>());

	}
}
