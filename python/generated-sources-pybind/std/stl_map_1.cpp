#include <functional>
#include <map>
#include <memory>
#include <sstream> // __str__
#include <utility>

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

PYBIND11_MAKE_OPAQUE(std::map<unsigned int,long>)
PYBIND11_MAKE_OPAQUE(std::map<long,unsigned int>)


void bind_std_stl_map_1(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // std::map file:bits/stl_map.h line:100
		pybind11::class_<std::map<unsigned int,long>, std::shared_ptr<std::map<unsigned int,long>>> cl(M("std"), "map_unsigned_int_long_t", "");
		cl.def( pybind11::init( [](){ return new std::map<unsigned int,long>(); } ) );
		cl.def( pybind11::init( [](const struct std::less<unsigned int> & a0){ return new std::map<unsigned int,long>(a0); } ), "doc" , pybind11::arg("__comp"));
		cl.def( pybind11::init<const struct std::less<unsigned int> &, const class std::allocator<struct std::pair<const unsigned int, long> > &>(), pybind11::arg("__comp"), pybind11::arg("__a") );

		cl.def( pybind11::init( [](std::map<unsigned int,long> const &o){ return new std::map<unsigned int,long>(o); } ) );
		cl.def( pybind11::init<const class std::allocator<struct std::pair<const unsigned int, long> > &>(), pybind11::arg("__a") );

		cl.def( pybind11::init<const class std::map<unsigned int, long> &, const class std::allocator<struct std::pair<const unsigned int, long> > &>(), pybind11::arg("__m"), pybind11::arg("__a") );

		cl.def("assign", (class std::map<unsigned int, long> & (std::map<unsigned int,long>::*)(const class std::map<unsigned int, long> &)) &std::map<unsigned int, long>::operator=, "C++: std::map<unsigned int, long>::operator=(const class std::map<unsigned int, long> &) --> class std::map<unsigned int, long> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		cl.def("get_allocator", (class std::allocator<struct std::pair<const unsigned int, long> > (std::map<unsigned int,long>::*)() const) &std::map<unsigned int, long>::get_allocator, "C++: std::map<unsigned int, long>::get_allocator() const --> class std::allocator<struct std::pair<const unsigned int, long> >");
		cl.def("empty", (bool (std::map<unsigned int,long>::*)() const) &std::map<unsigned int, long>::empty, "C++: std::map<unsigned int, long>::empty() const --> bool");
		cl.def("size", (unsigned long (std::map<unsigned int,long>::*)() const) &std::map<unsigned int, long>::size, "C++: std::map<unsigned int, long>::size() const --> unsigned long");
		cl.def("max_size", (unsigned long (std::map<unsigned int,long>::*)() const) &std::map<unsigned int, long>::max_size, "C++: std::map<unsigned int, long>::max_size() const --> unsigned long");
		cl.def("__getitem__", (long & (std::map<unsigned int,long>::*)(const unsigned int &)) &std::map<unsigned int, long>::operator[], "C++: std::map<unsigned int, long>::operator[](const unsigned int &) --> long &", pybind11::return_value_policy::automatic, pybind11::arg("__k"));
		cl.def("at", (long & (std::map<unsigned int,long>::*)(const unsigned int &)) &std::map<unsigned int, long>::at, "C++: std::map<unsigned int, long>::at(const unsigned int &) --> long &", pybind11::return_value_policy::automatic, pybind11::arg("__k"));
		cl.def("insert", (struct std::pair<struct std::_Rb_tree_iterator<struct std::pair<const unsigned int, long> >, bool> (std::map<unsigned int,long>::*)(const struct std::pair<const unsigned int, long> &)) &std::map<unsigned int, long>::insert, "C++: std::map<unsigned int, long>::insert(const struct std::pair<const unsigned int, long> &) --> struct std::pair<struct std::_Rb_tree_iterator<struct std::pair<const unsigned int, long> >, bool>", pybind11::arg("__x"));
		cl.def("erase", (unsigned long (std::map<unsigned int,long>::*)(const unsigned int &)) &std::map<unsigned int, long>::erase, "C++: std::map<unsigned int, long>::erase(const unsigned int &) --> unsigned long", pybind11::arg("__x"));
		cl.def("swap", (void (std::map<unsigned int,long>::*)(class std::map<unsigned int, long> &)) &std::map<unsigned int, long>::swap, "C++: std::map<unsigned int, long>::swap(class std::map<unsigned int, long> &) --> void", pybind11::arg("__x"));
		cl.def("clear", (void (std::map<unsigned int,long>::*)()) &std::map<unsigned int, long>::clear, "C++: std::map<unsigned int, long>::clear() --> void");
		cl.def("key_comp", (struct std::less<unsigned int> (std::map<unsigned int,long>::*)() const) &std::map<unsigned int, long>::key_comp, "C++: std::map<unsigned int, long>::key_comp() const --> struct std::less<unsigned int>");
		cl.def("count", (unsigned long (std::map<unsigned int,long>::*)(const unsigned int &) const) &std::map<unsigned int, long>::count, "C++: std::map<unsigned int, long>::count(const unsigned int &) const --> unsigned long", pybind11::arg("__x"));
		cl.def("equal_range", (struct std::pair<struct std::_Rb_tree_iterator<struct std::pair<const unsigned int, long> >, struct std::_Rb_tree_iterator<struct std::pair<const unsigned int, long> > > (std::map<unsigned int,long>::*)(const unsigned int &)) &std::map<unsigned int, long>::equal_range, "C++: std::map<unsigned int, long>::equal_range(const unsigned int &) --> struct std::pair<struct std::_Rb_tree_iterator<struct std::pair<const unsigned int, long> >, struct std::_Rb_tree_iterator<struct std::pair<const unsigned int, long> > >", pybind11::arg("__x"));
	}
	{ // std::map file:bits/stl_map.h line:100
		pybind11::class_<std::map<long,unsigned int>, std::shared_ptr<std::map<long,unsigned int>>> cl(M("std"), "map_long_unsigned_int_t", "");
		cl.def( pybind11::init( [](){ return new std::map<long,unsigned int>(); } ) );
		cl.def( pybind11::init( [](const struct std::less<long> & a0){ return new std::map<long,unsigned int>(a0); } ), "doc" , pybind11::arg("__comp"));
		cl.def( pybind11::init<const struct std::less<long> &, const class std::allocator<struct std::pair<const long, unsigned int> > &>(), pybind11::arg("__comp"), pybind11::arg("__a") );

		cl.def( pybind11::init( [](std::map<long,unsigned int> const &o){ return new std::map<long,unsigned int>(o); } ) );
		cl.def( pybind11::init<const class std::allocator<struct std::pair<const long, unsigned int> > &>(), pybind11::arg("__a") );

		cl.def( pybind11::init<const class std::map<long, unsigned int> &, const class std::allocator<struct std::pair<const long, unsigned int> > &>(), pybind11::arg("__m"), pybind11::arg("__a") );

		cl.def("assign", (class std::map<long, unsigned int> & (std::map<long,unsigned int>::*)(const class std::map<long, unsigned int> &)) &std::map<long, unsigned int>::operator=, "C++: std::map<long, unsigned int>::operator=(const class std::map<long, unsigned int> &) --> class std::map<long, unsigned int> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		cl.def("get_allocator", (class std::allocator<struct std::pair<const long, unsigned int> > (std::map<long,unsigned int>::*)() const) &std::map<long, unsigned int>::get_allocator, "C++: std::map<long, unsigned int>::get_allocator() const --> class std::allocator<struct std::pair<const long, unsigned int> >");
		cl.def("empty", (bool (std::map<long,unsigned int>::*)() const) &std::map<long, unsigned int>::empty, "C++: std::map<long, unsigned int>::empty() const --> bool");
		cl.def("size", (unsigned long (std::map<long,unsigned int>::*)() const) &std::map<long, unsigned int>::size, "C++: std::map<long, unsigned int>::size() const --> unsigned long");
		cl.def("max_size", (unsigned long (std::map<long,unsigned int>::*)() const) &std::map<long, unsigned int>::max_size, "C++: std::map<long, unsigned int>::max_size() const --> unsigned long");
		cl.def("__getitem__", (unsigned int & (std::map<long,unsigned int>::*)(const long &)) &std::map<long, unsigned int>::operator[], "C++: std::map<long, unsigned int>::operator[](const long &) --> unsigned int &", pybind11::return_value_policy::automatic, pybind11::arg("__k"));
		cl.def("at", (unsigned int & (std::map<long,unsigned int>::*)(const long &)) &std::map<long, unsigned int>::at, "C++: std::map<long, unsigned int>::at(const long &) --> unsigned int &", pybind11::return_value_policy::automatic, pybind11::arg("__k"));
		cl.def("insert", (struct std::pair<struct std::_Rb_tree_iterator<struct std::pair<const long, unsigned int> >, bool> (std::map<long,unsigned int>::*)(const struct std::pair<const long, unsigned int> &)) &std::map<long, unsigned int>::insert, "C++: std::map<long, unsigned int>::insert(const struct std::pair<const long, unsigned int> &) --> struct std::pair<struct std::_Rb_tree_iterator<struct std::pair<const long, unsigned int> >, bool>", pybind11::arg("__x"));
		cl.def("erase", (unsigned long (std::map<long,unsigned int>::*)(const long &)) &std::map<long, unsigned int>::erase, "C++: std::map<long, unsigned int>::erase(const long &) --> unsigned long", pybind11::arg("__x"));
		cl.def("swap", (void (std::map<long,unsigned int>::*)(class std::map<long, unsigned int> &)) &std::map<long, unsigned int>::swap, "C++: std::map<long, unsigned int>::swap(class std::map<long, unsigned int> &) --> void", pybind11::arg("__x"));
		cl.def("clear", (void (std::map<long,unsigned int>::*)()) &std::map<long, unsigned int>::clear, "C++: std::map<long, unsigned int>::clear() --> void");
		cl.def("key_comp", (struct std::less<long> (std::map<long,unsigned int>::*)() const) &std::map<long, unsigned int>::key_comp, "C++: std::map<long, unsigned int>::key_comp() const --> struct std::less<long>");
		cl.def("count", (unsigned long (std::map<long,unsigned int>::*)(const long &) const) &std::map<long, unsigned int>::count, "C++: std::map<long, unsigned int>::count(const long &) const --> unsigned long", pybind11::arg("__x"));
		cl.def("equal_range", (struct std::pair<struct std::_Rb_tree_iterator<struct std::pair<const long, unsigned int> >, struct std::_Rb_tree_iterator<struct std::pair<const long, unsigned int> > > (std::map<long,unsigned int>::*)(const long &)) &std::map<long, unsigned int>::equal_range, "C++: std::map<long, unsigned int>::equal_range(const long &) --> struct std::pair<struct std::_Rb_tree_iterator<struct std::pair<const long, unsigned int> >, struct std::_Rb_tree_iterator<struct std::pair<const long, unsigned int> > >", pybind11::arg("__x"));
	}
}
