#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <sstream> // __str__
#include <string>
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

PYBIND11_MAKE_OPAQUE(std::map<std::string,std::string>)
PYBIND11_MAKE_OPAQUE(std::map<std::string,double>)
PYBIND11_MAKE_OPAQUE(std::map<double,double>)
PYBIND11_MAKE_OPAQUE(std::map<std::string,mrpt::poses::CPose3D>)

void bind_std_stl_map(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // std::map file:bits/stl_map.h line:100
		pybind11::class_<std::map<std::string,std::string>, std::shared_ptr<std::map<std::string,std::string>>> cl(M("std"), "map_std_string_std_string_t", "");
		cl.def( pybind11::init( [](){ return new std::map<std::string,std::string>(); } ) );
		cl.def( pybind11::init( [](const struct std::less<std::string > & a0){ return new std::map<std::string,std::string>(a0); } ), "doc" , pybind11::arg("__comp"));
		cl.def( pybind11::init<const struct std::less<std::string > &, const class std::allocator<struct std::pair<const std::string, std::string > > &>(), pybind11::arg("__comp"), pybind11::arg("__a") );

		cl.def( pybind11::init( [](std::map<std::string,std::string> const &o){ return new std::map<std::string,std::string>(o); } ) );
		cl.def( pybind11::init<const class std::allocator<struct std::pair<const std::string, std::string > > &>(), pybind11::arg("__a") );

		cl.def( pybind11::init<const class std::map<std::string, std::string > &, const class std::allocator<struct std::pair<const std::string, std::string > > &>(), pybind11::arg("__m"), pybind11::arg("__a") );

		cl.def("assign", (class std::map<std::string, std::string > & (std::map<std::string,std::string>::*)(const class std::map<std::string, std::string > &)) &std::map<std::string, std::string>::operator=, "C++: std::map<std::string, std::string>::operator=(const class std::map<std::string, std::string > &) --> class std::map<std::string, std::string > &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		cl.def("get_allocator", (class std::allocator<struct std::pair<const std::string, std::string > > (std::map<std::string,std::string>::*)() const) &std::map<std::string, std::string>::get_allocator, "C++: std::map<std::string, std::string>::get_allocator() const --> class std::allocator<struct std::pair<const std::string, std::string > >");
		cl.def("empty", (bool (std::map<std::string,std::string>::*)() const) &std::map<std::string, std::string>::empty, "C++: std::map<std::string, std::string>::empty() const --> bool");
		cl.def("size", (size_t (std::map<std::string,std::string>::*)() const) &std::map<std::string, std::string>::size, "C++: std::map<std::string, std::string>::size() const --> size_t");
		cl.def("max_size", (size_t (std::map<std::string,std::string>::*)() const) &std::map<std::string, std::string>::max_size, "C++: std::map<std::string, std::string>::max_size() const --> size_t");
		cl.def("__getitem__", (std::string & (std::map<std::string,std::string>::*)(const std::string &)) &std::map<std::string, std::string>::operator[], "C++: std::map<std::string, std::string>::operator[](const std::string &) --> std::string &", pybind11::return_value_policy::automatic, pybind11::arg("__k"));
		cl.def("at", (std::string & (std::map<std::string,std::string>::*)(const std::string &)) &std::map<std::string, std::string>::at, "C++: std::map<std::string, std::string>::at(const std::string &) --> std::string &", pybind11::return_value_policy::automatic, pybind11::arg("__k"));
		cl.def("insert", (struct std::pair<struct std::_Rb_tree_iterator<struct std::pair<const std::string, std::string > >, bool> (std::map<std::string,std::string>::*)(const struct std::pair<const std::string, std::string > &)) &std::map<std::string, std::string>::insert, "C++: std::map<std::string, std::string>::insert(const struct std::pair<const std::string, std::string > &) --> struct std::pair<struct std::_Rb_tree_iterator<struct std::pair<const std::string, std::string > >, bool>", pybind11::arg("__x"));
		cl.def("erase", (size_t (std::map<std::string,std::string>::*)(const std::string &)) &std::map<std::string, std::string>::erase, "C++: std::map<std::string, std::string>::erase(const std::string &) --> size_t", pybind11::arg("__x"));
		cl.def("swap", (void (std::map<std::string,std::string>::*)(class std::map<std::string, std::string > &)) &std::map<std::string, std::string>::swap, "C++: std::map<std::string, std::string>::swap(class std::map<std::string, std::string > &) --> void", pybind11::arg("__x"));
		cl.def("clear", (void (std::map<std::string,std::string>::*)()) &std::map<std::string, std::string>::clear, "C++: std::map<std::string, std::string>::clear() --> void");
		cl.def("key_comp", (struct std::less<std::string > (std::map<std::string,std::string>::*)() const) &std::map<std::string, std::string>::key_comp, "C++: std::map<std::string, std::string>::key_comp() const --> struct std::less<std::string >");
		cl.def("count", (size_t (std::map<std::string,std::string>::*)(const std::string &) const) &std::map<std::string, std::string>::count, "C++: std::map<std::string, std::string>::count(const std::string &) const --> size_t", pybind11::arg("__x"));
		cl.def("equal_range", (struct std::pair<struct std::_Rb_tree_iterator<struct std::pair<const std::string, std::string > >, struct std::_Rb_tree_iterator<struct std::pair<const std::string, std::string > > > (std::map<std::string,std::string>::*)(const std::string &)) &std::map<std::string, std::string>::equal_range, "C++: std::map<std::string, std::string>::equal_range(const std::string &) --> struct std::pair<struct std::_Rb_tree_iterator<struct std::pair<const std::string, std::string > >, struct std::_Rb_tree_iterator<struct std::pair<const std::string, std::string > > >", pybind11::arg("__x"));
	}
	{ // std::map file:bits/stl_map.h line:100
		pybind11::class_<std::map<std::string,double>, std::shared_ptr<std::map<std::string,double>>> cl(M("std"), "map_std_string_double_t", "");
		cl.def( pybind11::init( [](){ return new std::map<std::string,double>(); } ) );
		cl.def( pybind11::init( [](const struct std::less<std::string > & a0){ return new std::map<std::string,double>(a0); } ), "doc" , pybind11::arg("__comp"));
		cl.def( pybind11::init<const struct std::less<std::string > &, const class std::allocator<struct std::pair<const std::string, double> > &>(), pybind11::arg("__comp"), pybind11::arg("__a") );

		cl.def( pybind11::init( [](std::map<std::string,double> const &o){ return new std::map<std::string,double>(o); } ) );
		cl.def( pybind11::init<const class std::allocator<struct std::pair<const std::string, double> > &>(), pybind11::arg("__a") );

		cl.def( pybind11::init<const class std::map<std::string, double> &, const class std::allocator<struct std::pair<const std::string, double> > &>(), pybind11::arg("__m"), pybind11::arg("__a") );

		cl.def("assign", (class std::map<std::string, double> & (std::map<std::string,double>::*)(const class std::map<std::string, double> &)) &std::map<std::string, double>::operator=, "C++: std::map<std::string, double>::operator=(const class std::map<std::string, double> &) --> class std::map<std::string, double> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		cl.def("get_allocator", (class std::allocator<struct std::pair<const std::string, double> > (std::map<std::string,double>::*)() const) &std::map<std::string, double>::get_allocator, "C++: std::map<std::string, double>::get_allocator() const --> class std::allocator<struct std::pair<const std::string, double> >");
		cl.def("empty", (bool (std::map<std::string,double>::*)() const) &std::map<std::string, double>::empty, "C++: std::map<std::string, double>::empty() const --> bool");
		cl.def("size", (size_t (std::map<std::string,double>::*)() const) &std::map<std::string, double>::size, "C++: std::map<std::string, double>::size() const --> size_t");
		cl.def("max_size", (size_t (std::map<std::string,double>::*)() const) &std::map<std::string, double>::max_size, "C++: std::map<std::string, double>::max_size() const --> size_t");
		cl.def("__getitem__", (double & (std::map<std::string,double>::*)(const std::string &)) &std::map<std::string, double>::operator[], "C++: std::map<std::string, double>::operator[](const std::string &) --> double &", pybind11::return_value_policy::automatic, pybind11::arg("__k"));
		cl.def("at", (double & (std::map<std::string,double>::*)(const std::string &)) &std::map<std::string, double>::at, "C++: std::map<std::string, double>::at(const std::string &) --> double &", pybind11::return_value_policy::automatic, pybind11::arg("__k"));
		cl.def("insert", (struct std::pair<struct std::_Rb_tree_iterator<struct std::pair<const std::string, double> >, bool> (std::map<std::string,double>::*)(const struct std::pair<const std::string, double> &)) &std::map<std::string, double>::insert, "C++: std::map<std::string, double>::insert(const struct std::pair<const std::string, double> &) --> struct std::pair<struct std::_Rb_tree_iterator<struct std::pair<const std::string, double> >, bool>", pybind11::arg("__x"));
		cl.def("erase", (size_t (std::map<std::string,double>::*)(const std::string &)) &std::map<std::string, double>::erase, "C++: std::map<std::string, double>::erase(const std::string &) --> size_t", pybind11::arg("__x"));
		cl.def("swap", (void (std::map<std::string,double>::*)(class std::map<std::string, double> &)) &std::map<std::string, double>::swap, "C++: std::map<std::string, double>::swap(class std::map<std::string, double> &) --> void", pybind11::arg("__x"));
		cl.def("clear", (void (std::map<std::string,double>::*)()) &std::map<std::string, double>::clear, "C++: std::map<std::string, double>::clear() --> void");
		cl.def("key_comp", (struct std::less<std::string > (std::map<std::string,double>::*)() const) &std::map<std::string, double>::key_comp, "C++: std::map<std::string, double>::key_comp() const --> struct std::less<std::string >");
		cl.def("count", (size_t (std::map<std::string,double>::*)(const std::string &) const) &std::map<std::string, double>::count, "C++: std::map<std::string, double>::count(const std::string &) const --> size_t", pybind11::arg("__x"));
		cl.def("equal_range", (struct std::pair<struct std::_Rb_tree_iterator<struct std::pair<const std::string, double> >, struct std::_Rb_tree_iterator<struct std::pair<const std::string, double> > > (std::map<std::string,double>::*)(const std::string &)) &std::map<std::string, double>::equal_range, "C++: std::map<std::string, double>::equal_range(const std::string &) --> struct std::pair<struct std::_Rb_tree_iterator<struct std::pair<const std::string, double> >, struct std::_Rb_tree_iterator<struct std::pair<const std::string, double> > >", pybind11::arg("__x"));
	}
	{ // std::map file:bits/stl_map.h line:100
		pybind11::class_<std::map<double,double>, std::shared_ptr<std::map<double,double>>> cl(M("std"), "map_double_double_t", "");
		cl.def( pybind11::init( [](){ return new std::map<double,double>(); } ) );
		cl.def( pybind11::init( [](const struct std::less<double> & a0){ return new std::map<double,double>(a0); } ), "doc" , pybind11::arg("__comp"));
		cl.def( pybind11::init<const struct std::less<double> &, const class std::allocator<struct std::pair<const double, double> > &>(), pybind11::arg("__comp"), pybind11::arg("__a") );

		cl.def( pybind11::init( [](std::map<double,double> const &o){ return new std::map<double,double>(o); } ) );
		cl.def( pybind11::init<const class std::allocator<struct std::pair<const double, double> > &>(), pybind11::arg("__a") );

		cl.def( pybind11::init<const class std::map<double, double> &, const class std::allocator<struct std::pair<const double, double> > &>(), pybind11::arg("__m"), pybind11::arg("__a") );

		cl.def("assign", (class std::map<double, double> & (std::map<double,double>::*)(const class std::map<double, double> &)) &std::map<double, double>::operator=, "C++: std::map<double, double>::operator=(const class std::map<double, double> &) --> class std::map<double, double> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		cl.def("get_allocator", (class std::allocator<struct std::pair<const double, double> > (std::map<double,double>::*)() const) &std::map<double, double>::get_allocator, "C++: std::map<double, double>::get_allocator() const --> class std::allocator<struct std::pair<const double, double> >");
		cl.def("empty", (bool (std::map<double,double>::*)() const) &std::map<double, double>::empty, "C++: std::map<double, double>::empty() const --> bool");
		cl.def("size", (size_t (std::map<double,double>::*)() const) &std::map<double, double>::size, "C++: std::map<double, double>::size() const --> size_t");
		cl.def("max_size", (size_t (std::map<double,double>::*)() const) &std::map<double, double>::max_size, "C++: std::map<double, double>::max_size() const --> size_t");
		cl.def("__getitem__", (double & (std::map<double,double>::*)(const double &)) &std::map<double, double>::operator[], "C++: std::map<double, double>::operator[](const double &) --> double &", pybind11::return_value_policy::automatic, pybind11::arg("__k"));
		cl.def("at", (double & (std::map<double,double>::*)(const double &)) &std::map<double, double>::at, "C++: std::map<double, double>::at(const double &) --> double &", pybind11::return_value_policy::automatic, pybind11::arg("__k"));
		cl.def("insert", (struct std::pair<struct std::_Rb_tree_iterator<struct std::pair<const double, double> >, bool> (std::map<double,double>::*)(const struct std::pair<const double, double> &)) &std::map<double, double>::insert, "C++: std::map<double, double>::insert(const struct std::pair<const double, double> &) --> struct std::pair<struct std::_Rb_tree_iterator<struct std::pair<const double, double> >, bool>", pybind11::arg("__x"));
		cl.def("erase", (size_t (std::map<double,double>::*)(const double &)) &std::map<double, double>::erase, "C++: std::map<double, double>::erase(const double &) --> size_t", pybind11::arg("__x"));
		cl.def("swap", (void (std::map<double,double>::*)(class std::map<double, double> &)) &std::map<double, double>::swap, "C++: std::map<double, double>::swap(class std::map<double, double> &) --> void", pybind11::arg("__x"));
		cl.def("clear", (void (std::map<double,double>::*)()) &std::map<double, double>::clear, "C++: std::map<double, double>::clear() --> void");
		cl.def("key_comp", (struct std::less<double> (std::map<double,double>::*)() const) &std::map<double, double>::key_comp, "C++: std::map<double, double>::key_comp() const --> struct std::less<double>");
		cl.def("count", (size_t (std::map<double,double>::*)(const double &) const) &std::map<double, double>::count, "C++: std::map<double, double>::count(const double &) const --> size_t", pybind11::arg("__x"));
		cl.def("equal_range", (struct std::pair<struct std::_Rb_tree_iterator<struct std::pair<const double, double> >, struct std::_Rb_tree_iterator<struct std::pair<const double, double> > > (std::map<double,double>::*)(const double &)) &std::map<double, double>::equal_range, "C++: std::map<double, double>::equal_range(const double &) --> struct std::pair<struct std::_Rb_tree_iterator<struct std::pair<const double, double> >, struct std::_Rb_tree_iterator<struct std::pair<const double, double> > >", pybind11::arg("__x"));
	}
	{ // std::map file:bits/stl_map.h line:100
		pybind11::class_<std::map<std::string,mrpt::poses::CPose3D>, std::shared_ptr<std::map<std::string,mrpt::poses::CPose3D>>> cl(M("std"), "map_std_string_mrpt_poses_CPose3D_t", "");
		cl.def( pybind11::init( [](){ return new std::map<std::string,mrpt::poses::CPose3D>(); } ) );
		cl.def( pybind11::init( [](const struct std::less<std::string > & a0){ return new std::map<std::string,mrpt::poses::CPose3D>(a0); } ), "doc" , pybind11::arg("__comp"));
		cl.def( pybind11::init<const struct std::less<std::string > &, const class std::allocator<struct std::pair<const std::string, class mrpt::poses::CPose3D> > &>(), pybind11::arg("__comp"), pybind11::arg("__a") );

		cl.def( pybind11::init( [](std::map<std::string,mrpt::poses::CPose3D> const &o){ return new std::map<std::string,mrpt::poses::CPose3D>(o); } ) );
		cl.def( pybind11::init<const class std::allocator<struct std::pair<const std::string, class mrpt::poses::CPose3D> > &>(), pybind11::arg("__a") );

		cl.def( pybind11::init<const class std::map<std::string, class mrpt::poses::CPose3D> &, const class std::allocator<struct std::pair<const std::string, class mrpt::poses::CPose3D> > &>(), pybind11::arg("__m"), pybind11::arg("__a") );

		cl.def("assign", (class std::map<std::string, class mrpt::poses::CPose3D> & (std::map<std::string,mrpt::poses::CPose3D>::*)(const class std::map<std::string, class mrpt::poses::CPose3D> &)) &std::map<std::string, mrpt::poses::CPose3D>::operator=, "C++: std::map<std::string, mrpt::poses::CPose3D>::operator=(const class std::map<std::string, class mrpt::poses::CPose3D> &) --> class std::map<std::string, class mrpt::poses::CPose3D> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		cl.def("get_allocator", (class std::allocator<struct std::pair<const std::string, class mrpt::poses::CPose3D> > (std::map<std::string,mrpt::poses::CPose3D>::*)() const) &std::map<std::string, mrpt::poses::CPose3D>::get_allocator, "C++: std::map<std::string, mrpt::poses::CPose3D>::get_allocator() const --> class std::allocator<struct std::pair<const std::string, class mrpt::poses::CPose3D> >");
		cl.def("empty", (bool (std::map<std::string,mrpt::poses::CPose3D>::*)() const) &std::map<std::string, mrpt::poses::CPose3D>::empty, "C++: std::map<std::string, mrpt::poses::CPose3D>::empty() const --> bool");
		cl.def("size", (size_t (std::map<std::string,mrpt::poses::CPose3D>::*)() const) &std::map<std::string, mrpt::poses::CPose3D>::size, "C++: std::map<std::string, mrpt::poses::CPose3D>::size() const --> size_t");
		cl.def("max_size", (size_t (std::map<std::string,mrpt::poses::CPose3D>::*)() const) &std::map<std::string, mrpt::poses::CPose3D>::max_size, "C++: std::map<std::string, mrpt::poses::CPose3D>::max_size() const --> size_t");
		cl.def("__getitem__", (class mrpt::poses::CPose3D & (std::map<std::string,mrpt::poses::CPose3D>::*)(const std::string &)) &std::map<std::string, mrpt::poses::CPose3D>::operator[], "C++: std::map<std::string, mrpt::poses::CPose3D>::operator[](const std::string &) --> class mrpt::poses::CPose3D &", pybind11::return_value_policy::automatic, pybind11::arg("__k"));
		cl.def("at", (class mrpt::poses::CPose3D & (std::map<std::string,mrpt::poses::CPose3D>::*)(const std::string &)) &std::map<std::string, mrpt::poses::CPose3D>::at, "C++: std::map<std::string, mrpt::poses::CPose3D>::at(const std::string &) --> class mrpt::poses::CPose3D &", pybind11::return_value_policy::automatic, pybind11::arg("__k"));
		cl.def("insert", (struct std::pair<struct std::_Rb_tree_iterator<struct std::pair<const std::string, class mrpt::poses::CPose3D> >, bool> (std::map<std::string,mrpt::poses::CPose3D>::*)(const struct std::pair<const std::string, class mrpt::poses::CPose3D> &)) &std::map<std::string, mrpt::poses::CPose3D>::insert, "C++: std::map<std::string, mrpt::poses::CPose3D>::insert(const struct std::pair<const std::string, class mrpt::poses::CPose3D> &) --> struct std::pair<struct std::_Rb_tree_iterator<struct std::pair<const std::string, class mrpt::poses::CPose3D> >, bool>", pybind11::arg("__x"));
		cl.def("erase", (size_t (std::map<std::string,mrpt::poses::CPose3D>::*)(const std::string &)) &std::map<std::string, mrpt::poses::CPose3D>::erase, "C++: std::map<std::string, mrpt::poses::CPose3D>::erase(const std::string &) --> size_t", pybind11::arg("__x"));
		cl.def("swap", (void (std::map<std::string,mrpt::poses::CPose3D>::*)(class std::map<std::string, class mrpt::poses::CPose3D> &)) &std::map<std::string, mrpt::poses::CPose3D>::swap, "C++: std::map<std::string, mrpt::poses::CPose3D>::swap(class std::map<std::string, class mrpt::poses::CPose3D> &) --> void", pybind11::arg("__x"));
		cl.def("clear", (void (std::map<std::string,mrpt::poses::CPose3D>::*)()) &std::map<std::string, mrpt::poses::CPose3D>::clear, "C++: std::map<std::string, mrpt::poses::CPose3D>::clear() --> void");
		cl.def("key_comp", (struct std::less<std::string > (std::map<std::string,mrpt::poses::CPose3D>::*)() const) &std::map<std::string, mrpt::poses::CPose3D>::key_comp, "C++: std::map<std::string, mrpt::poses::CPose3D>::key_comp() const --> struct std::less<std::string >");
		cl.def("count", (size_t (std::map<std::string,mrpt::poses::CPose3D>::*)(const std::string &) const) &std::map<std::string, mrpt::poses::CPose3D>::count, "C++: std::map<std::string, mrpt::poses::CPose3D>::count(const std::string &) const --> size_t", pybind11::arg("__x"));
		cl.def("equal_range", (struct std::pair<struct std::_Rb_tree_iterator<struct std::pair<const std::string, class mrpt::poses::CPose3D> >, struct std::_Rb_tree_iterator<struct std::pair<const std::string, class mrpt::poses::CPose3D> > > (std::map<std::string,mrpt::poses::CPose3D>::*)(const std::string &)) &std::map<std::string, mrpt::poses::CPose3D>::equal_range, "C++: std::map<std::string, mrpt::poses::CPose3D>::equal_range(const std::string &) --> struct std::pair<struct std::_Rb_tree_iterator<struct std::pair<const std::string, class mrpt::poses::CPose3D> >, struct std::_Rb_tree_iterator<struct std::pair<const std::string, class mrpt::poses::CPose3D> > >", pybind11::arg("__x"));
	}
}
