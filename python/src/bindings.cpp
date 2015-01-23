/* BOOST */
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python/suite/indexing/indexing_suite.hpp>

#include "bindings.h"
#include "utils_bindings.h"
#include "poses_bindings.h"
#include "system_bindings.h"

/* namespaces */
using namespace boost::python;


//
// Helpers
//
void IndexError() { PyErr_SetString(PyExc_IndexError, "Index out of range"); }

//
//  The Greeter
//
const std::string greeter = \
"\nWelcome to the mrpt-python-bindings!"\
"\n===================================="\
"\n"\
"\nThis is still a work in progress, but it is already usable now!"\
"\nAn example should be deployed with mrpt sources."\
"\n"\
"\nThere are already some ROS extensions which make those bindings"\
"\nreally powerful and easy to use with famous ROS."\
"\n"\
"\nIf you want to add new bindings feel free to to so! Have a look"\
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
    class_<std::vector<float> >("FloatVector")
            .def(vector_indexing_suite<std::vector<float> >());
    class_<std::vector<double> >("DoubleVector")
            .def(vector_indexing_suite<std::vector<double> >());
    class_<std::vector<char> >("CharVector")
            .def(vector_indexing_suite<std::vector<char> >());

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

////////////
//  slam  //
////////////
    export_slam();

///////////
//  gui  //
///////////
    export_gui();

////////////
// opengl //
////////////
    export_opengl();

////////////
//  math  //
////////////
    export_math();

///////////
// bayes //
///////////
    export_bayes();

}
