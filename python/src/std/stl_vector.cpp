#include <iterator>
#include <memory>
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

PYBIND11_MAKE_OPAQUE(std::vector<std::string>)

void bind_std_stl_vector(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // std::vector file:bits/stl_vector.h line:389
		pybind11::class_<std::vector<std::string>, std::shared_ptr<std::vector<std::string>>> cl(M("std"), "vector_std_string_t", "");
		cl.def( pybind11::init( [](){ return new std::vector<std::string>(); } ) );
		cl.def( pybind11::init<const class std::allocator<std::string > &>(), pybind11::arg("__a") );

		cl.def( pybind11::init( [](size_t const & a0){ return new std::vector<std::string>(a0); } ), "doc" , pybind11::arg("__n"));
		cl.def( pybind11::init<size_t, const class std::allocator<std::string > &>(), pybind11::arg("__n"), pybind11::arg("__a") );

		cl.def( pybind11::init( [](size_t const & a0, const std::string & a1){ return new std::vector<std::string>(a0, a1); } ), "doc" , pybind11::arg("__n"), pybind11::arg("__value"));
		cl.def( pybind11::init<size_t, const std::string &, const class std::allocator<std::string > &>(), pybind11::arg("__n"), pybind11::arg("__value"), pybind11::arg("__a") );

		cl.def( pybind11::init( [](std::vector<std::string> const &o){ return new std::vector<std::string>(o); } ) );
		cl.def( pybind11::init<const class std::vector<std::string > &, const class std::allocator<std::string > &>(), pybind11::arg("__x"), pybind11::arg("__a") );

		cl.def("assign", (class std::vector<std::string > & (std::vector<std::string>::*)(const class std::vector<std::string > &)) &std::vector<std::string>::operator=, "C++: std::vector<std::string>::operator=(const class std::vector<std::string > &) --> class std::vector<std::string > &", pybind11::return_value_policy::automatic, pybind11::arg("__x"));
		cl.def("assign", (void (std::vector<std::string>::*)(size_t, const std::string &)) &std::vector<std::string>::assign, "C++: std::vector<std::string>::assign(size_t, const std::string &) --> void", pybind11::arg("__n"), pybind11::arg("__val"));
		cl.def("begin", (class __gnu_cxx::__normal_iterator<std::string *, class std::vector<std::string > > (std::vector<std::string>::*)()) &std::vector<std::string>::begin, "C++: std::vector<std::string>::begin() --> class __gnu_cxx::__normal_iterator<std::string *, class std::vector<std::string > >");
		cl.def("end", (class __gnu_cxx::__normal_iterator<std::string *, class std::vector<std::string > > (std::vector<std::string>::*)()) &std::vector<std::string>::end, "C++: std::vector<std::string>::end() --> class __gnu_cxx::__normal_iterator<std::string *, class std::vector<std::string > >");
		cl.def("cbegin", (class __gnu_cxx::__normal_iterator<const std::string *, class std::vector<std::string > > (std::vector<std::string>::*)() const) &std::vector<std::string>::cbegin, "C++: std::vector<std::string>::cbegin() const --> class __gnu_cxx::__normal_iterator<const std::string *, class std::vector<std::string > >");
		cl.def("cend", (class __gnu_cxx::__normal_iterator<const std::string *, class std::vector<std::string > > (std::vector<std::string>::*)() const) &std::vector<std::string>::cend, "C++: std::vector<std::string>::cend() const --> class __gnu_cxx::__normal_iterator<const std::string *, class std::vector<std::string > >");
		cl.def("size", (size_t (std::vector<std::string>::*)() const) &std::vector<std::string>::size, "C++: std::vector<std::string>::size() const --> size_t");
		cl.def("max_size", (size_t (std::vector<std::string>::*)() const) &std::vector<std::string>::max_size, "C++: std::vector<std::string>::max_size() const --> size_t");
		cl.def("resize", (void (std::vector<std::string>::*)(size_t)) &std::vector<std::string>::resize, "C++: std::vector<std::string>::resize(size_t) --> void", pybind11::arg("__new_size"));
		cl.def("resize", (void (std::vector<std::string>::*)(size_t, const std::string &)) &std::vector<std::string>::resize, "C++: std::vector<std::string>::resize(size_t, const std::string &) --> void", pybind11::arg("__new_size"), pybind11::arg("__x"));
		cl.def("shrink_to_fit", (void (std::vector<std::string>::*)()) &std::vector<std::string>::shrink_to_fit, "C++: std::vector<std::string>::shrink_to_fit() --> void");
		cl.def("capacity", (size_t (std::vector<std::string>::*)() const) &std::vector<std::string>::capacity, "C++: std::vector<std::string>::capacity() const --> size_t");
		cl.def("empty", (bool (std::vector<std::string>::*)() const) &std::vector<std::string>::empty, "C++: std::vector<std::string>::empty() const --> bool");
		cl.def("reserve", (void (std::vector<std::string>::*)(size_t)) &std::vector<std::string>::reserve, "C++: std::vector<std::string>::reserve(size_t) --> void", pybind11::arg("__n"));
		cl.def("__getitem__", (std::string & (std::vector<std::string>::*)(size_t)) &std::vector<std::string>::operator[], "C++: std::vector<std::string>::operator[](size_t) --> std::string &", pybind11::return_value_policy::automatic, pybind11::arg("__n"));
		cl.def("at", (std::string & (std::vector<std::string>::*)(size_t)) &std::vector<std::string>::at, "C++: std::vector<std::string>::at(size_t) --> std::string &", pybind11::return_value_policy::automatic, pybind11::arg("__n"));
		cl.def("front", (std::string & (std::vector<std::string>::*)()) &std::vector<std::string>::front, "C++: std::vector<std::string>::front() --> std::string &", pybind11::return_value_policy::automatic);
		cl.def("back", (std::string & (std::vector<std::string>::*)()) &std::vector<std::string>::back, "C++: std::vector<std::string>::back() --> std::string &", pybind11::return_value_policy::automatic);
		cl.def("data", (std::string * (std::vector<std::string>::*)()) &std::vector<std::string>::data, "C++: std::vector<std::string>::data() --> std::string *", pybind11::return_value_policy::automatic);
		cl.def("push_back", (void (std::vector<std::string>::*)(const std::string &)) &std::vector<std::string>::push_back, "C++: std::vector<std::string>::push_back(const std::string &) --> void", pybind11::arg("__x"));
		cl.def("pop_back", (void (std::vector<std::string>::*)()) &std::vector<std::string>::pop_back, "C++: std::vector<std::string>::pop_back() --> void");
		cl.def("insert", (class __gnu_cxx::__normal_iterator<std::string *, class std::vector<std::string > > (std::vector<std::string>::*)(class __gnu_cxx::__normal_iterator<const std::string *, class std::vector<std::string > >, const std::string &)) &std::vector<std::string>::insert, "C++: std::vector<std::string>::insert(class __gnu_cxx::__normal_iterator<const std::string *, class std::vector<std::string > >, const std::string &) --> class __gnu_cxx::__normal_iterator<std::string *, class std::vector<std::string > >", pybind11::arg("__position"), pybind11::arg("__x"));
		cl.def("insert", (class __gnu_cxx::__normal_iterator<std::string *, class std::vector<std::string > > (std::vector<std::string>::*)(class __gnu_cxx::__normal_iterator<const std::string *, class std::vector<std::string > >, size_t, const std::string &)) &std::vector<std::string>::insert, "C++: std::vector<std::string>::insert(class __gnu_cxx::__normal_iterator<const std::string *, class std::vector<std::string > >, size_t, const std::string &) --> class __gnu_cxx::__normal_iterator<std::string *, class std::vector<std::string > >", pybind11::arg("__position"), pybind11::arg("__n"), pybind11::arg("__x"));
		cl.def("erase", (class __gnu_cxx::__normal_iterator<std::string *, class std::vector<std::string > > (std::vector<std::string>::*)(class __gnu_cxx::__normal_iterator<const std::string *, class std::vector<std::string > >)) &std::vector<std::string>::erase, "C++: std::vector<std::string>::erase(class __gnu_cxx::__normal_iterator<const std::string *, class std::vector<std::string > >) --> class __gnu_cxx::__normal_iterator<std::string *, class std::vector<std::string > >", pybind11::arg("__position"));
		cl.def("erase", (class __gnu_cxx::__normal_iterator<std::string *, class std::vector<std::string > > (std::vector<std::string>::*)(class __gnu_cxx::__normal_iterator<const std::string *, class std::vector<std::string > >, class __gnu_cxx::__normal_iterator<const std::string *, class std::vector<std::string > >)) &std::vector<std::string>::erase, "C++: std::vector<std::string>::erase(class __gnu_cxx::__normal_iterator<const std::string *, class std::vector<std::string > >, class __gnu_cxx::__normal_iterator<const std::string *, class std::vector<std::string > >) --> class __gnu_cxx::__normal_iterator<std::string *, class std::vector<std::string > >", pybind11::arg("__first"), pybind11::arg("__last"));
		cl.def("swap", (void (std::vector<std::string>::*)(class std::vector<std::string > &)) &std::vector<std::string>::swap, "C++: std::vector<std::string>::swap(class std::vector<std::string > &) --> void", pybind11::arg("__x"));
		cl.def("clear", (void (std::vector<std::string>::*)()) &std::vector<std::string>::clear, "C++: std::vector<std::string>::clear() --> void");

		cl.def("__iter__", [](const std::vector<std::string> &o) {
			return pybind11::make_iterator(o.begin(), o.end());
			}, pybind11::keep_alive<0, 1>());
	}
}
