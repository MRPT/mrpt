#include <any>
#include <chrono>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/containers/CommentPosition.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/Clock.h>
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
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ratio>
#include <sstream> // __str__
#include <string>
#include <string_view>
#include <utility>
#include <vector>

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

void bind_std_stl_multimap(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // std::multimap file:bits/stl_multimap.h line:99
		pybind11::class_<std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000> > >,std::shared_ptr<mrpt::serialization::CSerializable>>, std::shared_ptr<std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000> > >,std::shared_ptr<mrpt::serialization::CSerializable>>>> cl(M("std"), "multimap_std_chrono_time_point_mrpt_Clock_std_chrono_duration_long_std_ratio_1_10000000_std_shared_ptr_mrpt_serialization_CSerializable_t", "");
		cl.def( pybind11::init( [](){ return new std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000> > >,std::shared_ptr<mrpt::serialization::CSerializable>>(); } ) );
		cl.def( pybind11::init( [](const struct std::less<mrpt::Clock::time_point > & a0){ return new std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000> > >,std::shared_ptr<mrpt::serialization::CSerializable>>(a0); } ), "doc" , pybind11::arg("__comp"));
		cl.def( pybind11::init<const struct std::less<mrpt::Clock::time_point > &, const class std::allocator<struct std::pair<const mrpt::Clock::time_point, class std::shared_ptr<class mrpt::serialization::CSerializable> > > &>(), pybind11::arg("__comp"), pybind11::arg("__a") );

		cl.def( pybind11::init( [](std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000> > >,std::shared_ptr<mrpt::serialization::CSerializable>> const &o){ return new std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000> > >,std::shared_ptr<mrpt::serialization::CSerializable>>(o); } ) );
		cl.def( pybind11::init<const class std::allocator<struct std::pair<const mrpt::Clock::time_point, class std::shared_ptr<class mrpt::serialization::CSerializable> > > &>(), pybind11::arg("__a") );

		cl.def( pybind11::init<const class std::multimap<mrpt::Clock::time_point, class std::shared_ptr<class mrpt::serialization::CSerializable> > &, const class std::allocator<struct std::pair<const mrpt::Clock::time_point, class std::shared_ptr<class mrpt::serialization::CSerializable> > > &>(), pybind11::arg("__m"), pybind11::arg("__a") );

		cl.def("assign", (class std::multimap<mrpt::Clock::time_point, class std::shared_ptr<class mrpt::serialization::CSerializable> > & (std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000> > >,std::shared_ptr<mrpt::serialization::CSerializable>>::*)(const class std::multimap<mrpt::Clock::time_point, class std::shared_ptr<class mrpt::serialization::CSerializable> > &)) &std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000>>>, std::shared_ptr<mrpt::serialization::CSerializable>>::operator=, "C++: std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000>>>, std::shared_ptr<mrpt::serialization::CSerializable>>::operator=(const class std::multimap<mrpt::Clock::time_point, class std::shared_ptr<class mrpt::serialization::CSerializable> > &) --> class std::multimap<mrpt::Clock::time_point, class std::shared_ptr<class mrpt::serialization::CSerializable> > &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		cl.def("get_allocator", (class std::allocator<struct std::pair<const mrpt::Clock::time_point, class std::shared_ptr<class mrpt::serialization::CSerializable> > > (std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000> > >,std::shared_ptr<mrpt::serialization::CSerializable>>::*)() const) &std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000>>>, std::shared_ptr<mrpt::serialization::CSerializable>>::get_allocator, "C++: std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000>>>, std::shared_ptr<mrpt::serialization::CSerializable>>::get_allocator() const --> class std::allocator<struct std::pair<const mrpt::Clock::time_point, class std::shared_ptr<class mrpt::serialization::CSerializable> > >");
		cl.def("empty", (bool (std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000> > >,std::shared_ptr<mrpt::serialization::CSerializable>>::*)() const) &std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000>>>, std::shared_ptr<mrpt::serialization::CSerializable>>::empty, "C++: std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000>>>, std::shared_ptr<mrpt::serialization::CSerializable>>::empty() const --> bool");
		cl.def("size", (size_t (std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000> > >,std::shared_ptr<mrpt::serialization::CSerializable>>::*)() const) &std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000>>>, std::shared_ptr<mrpt::serialization::CSerializable>>::size, "C++: std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000>>>, std::shared_ptr<mrpt::serialization::CSerializable>>::size() const --> size_t");
		cl.def("max_size", (size_t (std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000> > >,std::shared_ptr<mrpt::serialization::CSerializable>>::*)() const) &std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000>>>, std::shared_ptr<mrpt::serialization::CSerializable>>::max_size, "C++: std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000>>>, std::shared_ptr<mrpt::serialization::CSerializable>>::max_size() const --> size_t");
		cl.def("erase", (size_t (std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000> > >,std::shared_ptr<mrpt::serialization::CSerializable>>::*)(const mrpt::Clock::time_point &)) &std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000>>>, std::shared_ptr<mrpt::serialization::CSerializable>>::erase, "C++: std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000>>>, std::shared_ptr<mrpt::serialization::CSerializable>>::erase(const mrpt::Clock::time_point &) --> size_t", pybind11::arg("__x"));
		cl.def("swap", (void (std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000> > >,std::shared_ptr<mrpt::serialization::CSerializable>>::*)(class std::multimap<mrpt::Clock::time_point, class std::shared_ptr<class mrpt::serialization::CSerializable> > &)) &std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000>>>, std::shared_ptr<mrpt::serialization::CSerializable>>::swap, "C++: std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000>>>, std::shared_ptr<mrpt::serialization::CSerializable>>::swap(class std::multimap<mrpt::Clock::time_point, class std::shared_ptr<class mrpt::serialization::CSerializable> > &) --> void", pybind11::arg("__x"));
		cl.def("clear", (void (std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000> > >,std::shared_ptr<mrpt::serialization::CSerializable>>::*)()) &std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000>>>, std::shared_ptr<mrpt::serialization::CSerializable>>::clear, "C++: std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000>>>, std::shared_ptr<mrpt::serialization::CSerializable>>::clear() --> void");
		cl.def("key_comp", (struct std::less<mrpt::Clock::time_point > (std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000> > >,std::shared_ptr<mrpt::serialization::CSerializable>>::*)() const) &std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000>>>, std::shared_ptr<mrpt::serialization::CSerializable>>::key_comp, "C++: std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000>>>, std::shared_ptr<mrpt::serialization::CSerializable>>::key_comp() const --> struct std::less<mrpt::Clock::time_point >");
		cl.def("count", (size_t (std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000> > >,std::shared_ptr<mrpt::serialization::CSerializable>>::*)(const mrpt::Clock::time_point &) const) &std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000>>>, std::shared_ptr<mrpt::serialization::CSerializable>>::count, "C++: std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000>>>, std::shared_ptr<mrpt::serialization::CSerializable>>::count(const mrpt::Clock::time_point &) const --> size_t", pybind11::arg("__x"));
		cl.def("equal_range", (struct std::pair<struct std::_Rb_tree_iterator<struct std::pair<const mrpt::Clock::time_point, class std::shared_ptr<class mrpt::serialization::CSerializable> > >, struct std::_Rb_tree_iterator<struct std::pair<const mrpt::Clock::time_point, class std::shared_ptr<class mrpt::serialization::CSerializable> > > > (std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000> > >,std::shared_ptr<mrpt::serialization::CSerializable>>::*)(const mrpt::Clock::time_point &)) &std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000>>>, std::shared_ptr<mrpt::serialization::CSerializable>>::equal_range, "C++: std::multimap<std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000>>>, std::shared_ptr<mrpt::serialization::CSerializable>>::equal_range(const mrpt::Clock::time_point &) --> struct std::pair<struct std::_Rb_tree_iterator<struct std::pair<const mrpt::Clock::time_point, class std::shared_ptr<class mrpt::serialization::CSerializable> > >, struct std::_Rb_tree_iterator<struct std::pair<const mrpt::Clock::time_point, class std::shared_ptr<class mrpt::serialization::CSerializable> > > >", pybind11::arg("__x"));
	}
	// std::map file:bits/stl_map.h line:100
	binder::map_binder<mrpt::containers::yaml::node_t,mrpt::containers::yaml::node_t,std::less<mrpt::containers::yaml::node_t>,std::allocator<std::pair<const mrpt::containers::yaml::node_t, mrpt::containers::yaml::node_t> >>(M("std"), "mrpt_containers_yaml_node_t", "mrpt_containers_yaml_node_t", "std_less_mrpt_containers_yaml_node_t_t", "std_allocator_std_pair_const_mrpt_containers_yaml_node_t_mrpt_containers_yaml_node_t_t");

	// std::map file:bits/stl_map.h line:100
	binder::map_binder<std::string,std::string,std::less<std::string >,std::allocator<std::pair<const std::string, std::string > >>(M("std"), "std_string", "std_string", "std_less_std_string_t", "std_allocator_std_pair_const_std_string_std_string_t");

	// std::map file:bits/stl_map.h line:100
	binder::map_binder<std::string,double,std::less<std::string >,std::allocator<std::pair<const std::string, double> >>(M("std"), "std_string", "double", "std_less_std_string_t", "std_allocator_std_pair_const_std_string_double_t");

	// std::map file:bits/stl_map.h line:100
	binder::map_binder<double,double,std::less<double>,std::allocator<std::pair<const double, double> >>(M("std"), "double", "double", "std_less_double_t", "std_allocator_std_pair_const_double_double_t");

	// std::map file:bits/stl_map.h line:100
	binder::map_binder<std::string,mrpt::poses::CPose3D,std::less<std::string >,std::allocator<std::pair<const std::string, mrpt::poses::CPose3D> >>(M("std"), "std_string", "mrpt_poses_CPose3D", "std_less_std_string_t", "std_allocator_std_pair_const_std_string_mrpt_poses_CPose3D_t");

	// std::map file:bits/stl_map.h line:100
	binder::map_binder<unsigned int,long,std::less<unsigned int>,std::allocator<std::pair<const unsigned int, long> >>(M("std"), "unsigned_int", "long", "std_less_unsigned_int_t", "std_allocator_std_pair_const_unsigned_int_long_t");

	// std::map file:bits/stl_map.h line:100
	binder::map_binder<long,unsigned int,std::less<long>,std::allocator<std::pair<const long, unsigned int> >>(M("std"), "long", "unsigned_int", "std_less_long_t", "std_allocator_std_pair_const_long_unsigned_int_t");

}
