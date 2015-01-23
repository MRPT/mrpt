/* bindings */
#include "bindings.h"

/* MRPT */
// #include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/math/lightweight_geom_data.h>

using namespace mrpt::math;
using namespace mrpt::poses;

using namespace boost::python;


// exporter
void export_math()
{
    // map namespace to be submodule of mrpt package
    object math_module(handle<>(borrowed(PyImport_AddModule("mrpt.math"))));
    scope().attr("math") = math_module;
    scope math_scope = math_module;

    {
        class_<TPose3D>("TPose3D", init<>())
            .def(init<TPoint2D>())
            .def(init<TPose2D>())
            .def(init<TPoint3D>())
            .def(init<CPose3D>())
            .def(init<double, double, double, double, double, double>())
            .def_readwrite("x", &TPose3D::x)
            .def_readwrite("y", &TPose3D::y)
            .def_readwrite("z", &TPose3D::z)
            .def_readwrite("yaw", &TPose3D::yaw)
            .def_readwrite("pitch", &TPose3D::pitch)
            .def_readwrite("roll", &TPose3D::roll)
        ;
    }
}
