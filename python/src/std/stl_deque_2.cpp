#include <deque>
#include <memory>
#include <mrpt/system/CDirectoryExplorer.h>
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

PYBIND11_MAKE_OPAQUE(std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>)

void bind_std_stl_deque_2(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // std::deque file:bits/stl_deque.h line:767
		pybind11::class_<std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>, std::shared_ptr<std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>>> cl(M("std"), "deque_mrpt_system_CDirectoryExplorer_TFileInfo_t", "");
		cl.def( pybind11::init( [](){ return new std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>(); } ) );
		cl.def( pybind11::init<const class std::allocator<struct mrpt::system::CDirectoryExplorer::TFileInfo> &>(), pybind11::arg("__a") );

		cl.def( pybind11::init( [](size_t const & a0){ return new std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>(a0); } ), "doc" , pybind11::arg("__n"));
		cl.def( pybind11::init<size_t, const class std::allocator<struct mrpt::system::CDirectoryExplorer::TFileInfo> &>(), pybind11::arg("__n"), pybind11::arg("__a") );

		cl.def( pybind11::init( [](size_t const & a0, const struct mrpt::system::CDirectoryExplorer::TFileInfo & a1){ return new std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>(a0, a1); } ), "doc" , pybind11::arg("__n"), pybind11::arg("__value"));
		cl.def( pybind11::init<size_t, const struct mrpt::system::CDirectoryExplorer::TFileInfo &, const class std::allocator<struct mrpt::system::CDirectoryExplorer::TFileInfo> &>(), pybind11::arg("__n"), pybind11::arg("__value"), pybind11::arg("__a") );

		cl.def( pybind11::init( [](std::deque<mrpt::system::CDirectoryExplorer::TFileInfo> const &o){ return new std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>(o); } ) );
		cl.def( pybind11::init<const class std::deque<struct mrpt::system::CDirectoryExplorer::TFileInfo> &, const class std::allocator<struct mrpt::system::CDirectoryExplorer::TFileInfo> &>(), pybind11::arg("__x"), pybind11::arg("__a") );

		cl.def("assign", (class std::deque<struct mrpt::system::CDirectoryExplorer::TFileInfo> & (std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::*)(const class std::deque<struct mrpt::system::CDirectoryExplorer::TFileInfo> &)) &std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::operator=, "C++: std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::operator=(const class std::deque<struct mrpt::system::CDirectoryExplorer::TFileInfo> &) --> class std::deque<struct mrpt::system::CDirectoryExplorer::TFileInfo> &", pybind11::return_value_policy::automatic, pybind11::arg("__x"));
		cl.def("assign", (void (std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::*)(size_t, const struct mrpt::system::CDirectoryExplorer::TFileInfo &)) &std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::assign, "C++: std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::assign(size_t, const struct mrpt::system::CDirectoryExplorer::TFileInfo &) --> void", pybind11::arg("__n"), pybind11::arg("__val"));
		cl.def("get_allocator", (class std::allocator<struct mrpt::system::CDirectoryExplorer::TFileInfo> (std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::*)() const) &std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::get_allocator, "C++: std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::get_allocator() const --> class std::allocator<struct mrpt::system::CDirectoryExplorer::TFileInfo>");
		cl.def("size", (size_t (std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::*)() const) &std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::size, "C++: std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::size() const --> size_t");
		cl.def("max_size", (size_t (std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::*)() const) &std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::max_size, "C++: std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::max_size() const --> size_t");
		cl.def("resize", (void (std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::*)(size_t)) &std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::resize, "C++: std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::resize(size_t) --> void", pybind11::arg("__new_size"));
		cl.def("resize", (void (std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::*)(size_t, const struct mrpt::system::CDirectoryExplorer::TFileInfo &)) &std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::resize, "C++: std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::resize(size_t, const struct mrpt::system::CDirectoryExplorer::TFileInfo &) --> void", pybind11::arg("__new_size"), pybind11::arg("__x"));
		cl.def("shrink_to_fit", (void (std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::*)()) &std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::shrink_to_fit, "C++: std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::shrink_to_fit() --> void");
		cl.def("empty", (bool (std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::*)() const) &std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::empty, "C++: std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::empty() const --> bool");
		cl.def("__getitem__", (struct mrpt::system::CDirectoryExplorer::TFileInfo & (std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::*)(size_t)) &std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::operator[], "C++: std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::operator[](size_t) --> struct mrpt::system::CDirectoryExplorer::TFileInfo &", pybind11::return_value_policy::automatic, pybind11::arg("__n"));
		cl.def("at", (struct mrpt::system::CDirectoryExplorer::TFileInfo & (std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::*)(size_t)) &std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::at, "C++: std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::at(size_t) --> struct mrpt::system::CDirectoryExplorer::TFileInfo &", pybind11::return_value_policy::automatic, pybind11::arg("__n"));
		cl.def("front", (struct mrpt::system::CDirectoryExplorer::TFileInfo & (std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::*)()) &std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::front, "C++: std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::front() --> struct mrpt::system::CDirectoryExplorer::TFileInfo &", pybind11::return_value_policy::automatic);
		cl.def("back", (struct mrpt::system::CDirectoryExplorer::TFileInfo & (std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::*)()) &std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::back, "C++: std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::back() --> struct mrpt::system::CDirectoryExplorer::TFileInfo &", pybind11::return_value_policy::automatic);
		cl.def("push_front", (void (std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::*)(const struct mrpt::system::CDirectoryExplorer::TFileInfo &)) &std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::push_front, "C++: std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::push_front(const struct mrpt::system::CDirectoryExplorer::TFileInfo &) --> void", pybind11::arg("__x"));
		cl.def("push_back", (void (std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::*)(const struct mrpt::system::CDirectoryExplorer::TFileInfo &)) &std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::push_back, "C++: std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::push_back(const struct mrpt::system::CDirectoryExplorer::TFileInfo &) --> void", pybind11::arg("__x"));
		cl.def("pop_front", (void (std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::*)()) &std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::pop_front, "C++: std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::pop_front() --> void");
		cl.def("pop_back", (void (std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::*)()) &std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::pop_back, "C++: std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::pop_back() --> void");
		cl.def("swap", (void (std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::*)(class std::deque<struct mrpt::system::CDirectoryExplorer::TFileInfo> &)) &std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::swap, "C++: std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::swap(class std::deque<struct mrpt::system::CDirectoryExplorer::TFileInfo> &) --> void", pybind11::arg("__x"));
		cl.def("clear", (void (std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::*)()) &std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::clear, "C++: std::deque<mrpt::system::CDirectoryExplorer::TFileInfo>::clear() --> void");
	}
}
