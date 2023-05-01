#include <any>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/containers/CommentPosition.h>
#include <mrpt/containers/yaml.h>
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

void bind_std_stl_map(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
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
	binder::map_binder<unsigned long,unsigned long,std::less<unsigned long>,std::allocator<std::pair<const unsigned long, unsigned long> >>(M("std"), "unsigned_long", "unsigned_long", "std_less_unsigned_long_t", "std_allocator_std_pair_const_unsigned_long_unsigned_long_t");

	// std::map file:bits/stl_map.h line:100
	binder::map_binder<unsigned int,long,std::less<unsigned int>,std::allocator<std::pair<const unsigned int, long> >>(M("std"), "unsigned_int", "long", "std_less_unsigned_int_t", "std_allocator_std_pair_const_unsigned_int_long_t");

	// std::map file:bits/stl_map.h line:100
	binder::map_binder<long,unsigned int,std::less<long>,std::allocator<std::pair<const long, unsigned int> >>(M("std"), "long", "unsigned_int", "std_less_long_t", "std_allocator_std_pair_const_long_unsigned_int_t");

}
