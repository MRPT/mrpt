/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#include "bindings.h"

/* namespaces */
using namespace boost::python;

//
// Helpers
//
void IndexError()
{
	PyErr_SetString(PyExc_IndexError, "Index out of range");
	throw_error_already_set();
}
void TypeError(std::string message)
{
	PyErr_SetString(PyExc_TypeError, message.c_str());
	throw_error_already_set();
}
//
//  The Greeter
//
const std::string greeter =
	"\nWelcome to the mrpt-python-bindings!"
	"\n===================================="
	"\n"
	"\nThis is still a work in progress, but it is already usable now!"
	"\nAn example should be deployed with mrpt sources."
	"\n"
	"\nThere are already some ROS extensions which make those bindings"
	"\nreally powerful and easy to use with famous ROS."
	"\n"
	"\nIf you want to add new bindings feel free to to so! Have a look"
	"\ninto the sources, it's not that hard! ;-)";

//
// The Module
//
BOOST_PYTHON_MODULE(pymrpt)
{
	// define this module to be a package
	object package = scope();
	package.attr("__path__") = "pymrpt";
	package.attr("greeter") = greeter;

	// STL
	{
		object stl_module(handle<>(borrowed(PyImport_AddModule("pymrpt.stl"))));
		scope().attr("stl") = stl_module;
		scope stl_scope = stl_module;

		export_math_stl();
		export_poses_stl();
		export_utils_stl();
	}

	////////////
	// system //
	////////////
	export_system();

	///////////
	// utils //
	///////////
	export_utils();

	///////////
	// poses //
	///////////
	export_poses();

	///////////
	//  obs  //
	///////////
	export_obs();

	////////////
	//  maps  //
	////////////
	export_maps();

	///////////
	//  gui  //
	///////////
	export_gui();

	////////////
	// opengl //
	////////////
	export_opengl();

	///////////
	//  nav  //
	///////////
	export_nav();

	////////////
	//  math  //
	////////////
	export_math();

	///////////
	// bayes //
	///////////
	export_bayes();

	////////////
	//  slam  //
	////////////
	export_slam();

	///////////
	// kinematics //
	///////////
	export_kinematics();

	////////////
	//  pnp  //
	////////////
	export_pnp();
}
