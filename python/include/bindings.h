/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef __BINDINGS_H__
#define __BINDINGS_H__

/* BOOST */
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python/suite/indexing/indexing_suite.hpp>

/* std */
#include <vector>
#include <deque>

/* macros */
#define STRINGIFY(str) #str

#define MAKE_PTR(class_name)                                                  \
	class_<class_name::Ptr>(                                                  \
		STRINGIFY(class_name::Ptr), "class_name smart pointer type", no_init) \
		.def("ctx", &class_name##Ptr_get_ctx, return_internal_reference<>())  \
		.def("ctx", &class_name##Ptr_set_ctx)                                 \
		.def(                                                                 \
			"pointer", &class_name##Ptr_pointer,                              \
			return_internal_reference<>());

#define MAKE_PTR_NAMED(class_name, ptr_name)                                 \
	class_<class_name::Ptr>(                                                 \
		STRINGIFY(ptr_name::Ptr), "class_name smart pointer type", no_init)  \
		.def("ctx", &class_name##Ptr_get_ctx, return_internal_reference<>()) \
		.def("ctx", &class_name##Ptr_set_ctx)                                \
		.def(                                                                \
			"pointer", &class_name##Ptr_pointer,                             \
			return_internal_reference<>());

#define MAKE_CREATE(class_name)                          \
	.def(                                                \
		"Create", mrpt::make_aligned_shared<class_name>, \
		"Create smart pointer from class.")              \
		.staticmethod("Create")

#define MAKE_VEC(class_name)                                       \
	class_<std::vector<class_name>>(STRINGIFY(class_name##Vector)) \
		.def(vector_indexing_suite<std::vector<class_name>>());

#define MAKE_VEC_NAMED(class_name, vec_name)        \
	class_<std::vector<class_name>>(#vec_name).def( \
		vector_indexing_suite<std::vector<class_name>>());

#define MAKE_PTR_CTX(class_name)                                         \
	class_name& class_name##Ptr_get_ctx(class_name::Ptr& self)           \
	{                                                                    \
		return *self;                                                    \
	}                                                                    \
	void class_name##Ptr_set_ctx(class_name::Ptr& self, class_name& ctx) \
	{                                                                    \
		*self = ctx;                                                     \
	}                                                                    \
	class_name* class_name##Ptr_pointer(class_name::Ptr& self)           \
	{                                                                    \
		return self.get();                                               \
	}

#define MAKE_AS_STR(class_name)                         \
	std::string class_name##_asString(class_name& self) \
	{                                                   \
		return self.asString();                         \
	}

#define MAKE_GETITEM(class_name, value_type)                    \
	value_type class_name##_getitem(class_name& self, size_t i) \
	{                                                           \
		return self[i];                                         \
	}

#define MAKE_SETITEM(class_name, value_type)                                \
	void class_name##_setitem(class_name& self, size_t i, value_type value) \
	{                                                                       \
		self[i] = value;                                                    \
	}

#define MAKE_SUBMODULE(mod)                                             \
	object mod_module(                                                  \
		handle<>(borrowed(PyImport_AddModule(STRINGIFY(pymrpt.mod))))); \
	scope().attr(STRINGIFY(mod)) = mod_module;                          \
	scope mod_scope = mod_module;

// Helpers
void IndexError();
void TypeError(std::string message);
// end of Helpers

// STL list-like containers (vector, list, deque)
template <class T>
struct StlListLike
{
	typedef typename T::value_type V;
	static V& get(T& x, uint i)
	{
		if (i < x.size()) return x[i];
		IndexError();
		// only for removing the return-type warning; code is never reached:
		return x[0];
	}
	static void set(T& x, uint i, V const& v)
	{
		if (i < x.size())
			x[i] = v;
		else
			IndexError();
	}
	static void del(T& x, uint i)
	{
		if (i < x.size())
			x.erase(x.begin() + i);
		else
			IndexError();
	}
	static void add(T& x, V const& v) { x.push_back(v); }
};
// end of STL list-like containers

/* exporters */
void export_gui();
void export_opengl();
void export_math();
void export_math_stl();
void export_obs();
void export_maps();
void export_slam();
void export_nav();
void export_poses();
void export_poses_stl();
void export_system();
void export_utils();
void export_utils_stl();
void export_kinematics();
void export_bayes();
void export_pnp();

#ifdef ROS_EXTENSIONS
// time conversion
boost::python::object TTimeStamp_to_ROS_Time(boost::python::long_ timestamp);
boost::python::long_ TTimeStamp_from_ROS_Time(boost::python::object ros_time);
#endif

#endif
