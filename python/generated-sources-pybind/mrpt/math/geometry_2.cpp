#include <any>
#include <functional>
#include <istream>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/containers/CommentPosition.h>
#include <mrpt/containers/YamlEmitOptions.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPlane.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPolygon2D.h>
#include <mrpt/math/TPolygon3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TSegment3D.h>
#include <mrpt/math/geometry.h>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <typeinfo>
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

void bind_mrpt_math_geometry_2(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::math::squaredDistancePointToLine(double, double, double, double, double, double) file:mrpt/math/geometry.h line:977
	M("mrpt::math").def("squaredDistancePointToLine", (double (*)(double, double, double, double, double, double)) &mrpt::math::squaredDistancePointToLine, "Returns the square distance from a point to a line.\n\nC++: mrpt::math::squaredDistancePointToLine(double, double, double, double, double, double) --> double", pybind11::arg("Px"), pybind11::arg("Py"), pybind11::arg("x1"), pybind11::arg("y1"), pybind11::arg("x2"), pybind11::arg("y2"));

	// mrpt::math::squaredDistancePointToLine(const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPoint2D_<double> &) file:mrpt/math/geometry.h line:981
	M("mrpt::math").def("squaredDistancePointToLine", (double (*)(const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPoint2D_<double> &)) &mrpt::math::squaredDistancePointToLine, "C++: mrpt::math::squaredDistancePointToLine(const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPoint2D_<double> &) --> double", pybind11::arg("query"), pybind11::arg("linePt1"), pybind11::arg("linePt2"));

	// mrpt::math::distanceBetweenPoints(const double, const double, const double, const double) file:mrpt/math/geometry.h line:991
	M("mrpt::math").def("distanceBetweenPoints", (double (*)(const double, const double, const double, const double)) &mrpt::math::distanceBetweenPoints<double>, "C++: mrpt::math::distanceBetweenPoints(const double, const double, const double, const double) --> double", pybind11::arg("x1"), pybind11::arg("y1"), pybind11::arg("x2"), pybind11::arg("y2"));

	// mrpt::math::RectanglesIntersection(double, double, double, double, double, double, double, double, double, double, double) file:mrpt/math/geometry.h line:1070
	M("mrpt::math").def("RectanglesIntersection", (bool (*)(double, double, double, double, double, double, double, double, double, double, double)) &mrpt::math::RectanglesIntersection, "Returns whether two rotated rectangles intersect.\n  The first rectangle is not rotated and given by\n (R1_x_min,R1_x_max)-(R1_y_min,R1_y_max).\n  The second rectangle is given is a similar way, but it is internally rotated\n according\n   to the given coordinates translation\n (R2_pose_x,R2_pose_y,R2_pose_phi(radians)), relative\n   to the coordinates system of rectangle 1.\n\nC++: mrpt::math::RectanglesIntersection(double, double, double, double, double, double, double, double, double, double, double) --> bool", pybind11::arg("R1_x_min"), pybind11::arg("R1_x_max"), pybind11::arg("R1_y_min"), pybind11::arg("R1_y_max"), pybind11::arg("R2_x_min"), pybind11::arg("R2_x_max"), pybind11::arg("R2_y_min"), pybind11::arg("R2_y_max"), pybind11::arg("R2_pose_x"), pybind11::arg("R2_pose_y"), pybind11::arg("R2_pose_phi"));

	// mrpt::math::signedArea(const class mrpt::math::TPolygon2D &) file:mrpt/math/geometry.h line:1119
	M("mrpt::math").def("signedArea", (double (*)(const class mrpt::math::TPolygon2D &)) &mrpt::math::signedArea, "Returns the area of a polygon, positive if vertices listed in CCW ordering,\n negative if CW.\n\n  \n (New in MRPT 2.4.1)\n\nC++: mrpt::math::signedArea(const class mrpt::math::TPolygon2D &) --> double", pybind11::arg("p"));

}
