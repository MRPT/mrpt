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
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
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
#include <pybind11/stl.h>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

void bind_mrpt_math_geometry_2(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::math::closestFromPointToLine(const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPoint2D_<double> &) file:mrpt/math/geometry.h line:911
	M("mrpt::math").def("closestFromPointToLine", (struct mrpt::math::TPoint2D_<double> (*)(const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPoint2D_<double> &)) &mrpt::math::closestFromPointToLine, "C++: mrpt::math::closestFromPointToLine(const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPoint2D_<double> &) --> struct mrpt::math::TPoint2D_<double>", pybind11::arg("query"), pybind11::arg("linePt1"), pybind11::arg("linePt2"));

	// mrpt::math::squaredDistancePointToLine(double, double, double, double, double, double) file:mrpt/math/geometry.h line:923
	M("mrpt::math").def("squaredDistancePointToLine", (double (*)(double, double, double, double, double, double)) &mrpt::math::squaredDistancePointToLine, "Returns the square distance from a point to a line.\n\nC++: mrpt::math::squaredDistancePointToLine(double, double, double, double, double, double) --> double", pybind11::arg("Px"), pybind11::arg("Py"), pybind11::arg("x1"), pybind11::arg("y1"), pybind11::arg("x2"), pybind11::arg("y2"));

	// mrpt::math::squaredDistancePointToLine(const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPoint2D_<double> &) file:mrpt/math/geometry.h line:926
	M("mrpt::math").def("squaredDistancePointToLine", (double (*)(const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPoint2D_<double> &)) &mrpt::math::squaredDistancePointToLine, "C++: mrpt::math::squaredDistancePointToLine(const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPoint2D_<double> &) --> double", pybind11::arg("query"), pybind11::arg("linePt1"), pybind11::arg("linePt2"));

	// mrpt::math::distanceBetweenPoints(const double, const double, const double, const double) file:mrpt/math/geometry.h line:936
	M("mrpt::math").def("distanceBetweenPoints", (double (*)(const double, const double, const double, const double)) &mrpt::math::distanceBetweenPoints<double>, "C++: mrpt::math::distanceBetweenPoints(const double, const double, const double, const double) --> double", pybind11::arg("x1"), pybind11::arg("y1"), pybind11::arg("x2"), pybind11::arg("y2"));

	// mrpt::math::RectanglesIntersection(double, double, double, double, double, double, double, double, double, double, double) file:mrpt/math/geometry.h line:1018
	M("mrpt::math").def("RectanglesIntersection", (bool (*)(double, double, double, double, double, double, double, double, double, double, double)) &mrpt::math::RectanglesIntersection, "Returns whether two rotated rectangles intersect.\n  The first rectangle is not rotated and given by\n (R1_x_min,R1_x_max)-(R1_y_min,R1_y_max).\n  The second rectangle is given is a similar way, but it is internally rotated\n according\n   to the given coordinates translation\n (R2_pose_x,R2_pose_y,R2_pose_phi(radians)), relative\n   to the coordinates system of rectangle 1.\n\nC++: mrpt::math::RectanglesIntersection(double, double, double, double, double, double, double, double, double, double, double) --> bool", pybind11::arg("R1_x_min"), pybind11::arg("R1_x_max"), pybind11::arg("R1_y_min"), pybind11::arg("R1_y_max"), pybind11::arg("R2_x_min"), pybind11::arg("R2_x_max"), pybind11::arg("R2_y_min"), pybind11::arg("R2_y_max"), pybind11::arg("R2_pose_x"), pybind11::arg("R2_pose_y"), pybind11::arg("R2_pose_phi"));

	// mrpt::math::generateAxisBaseFromDirection(double, double, double) file:mrpt/math/geometry.h line:1068
	M("mrpt::math").def("generateAxisBaseFromDirection", (class mrpt::math::CMatrixFixed<double, 3, 3> (*)(double, double, double)) &mrpt::math::generateAxisBaseFromDirection, "Computes an axis base (a set of three 3D normal vectors) with the given\n  vector being the first of them (\"X\")\n NOTE: Make sure of passing all floats or doubles and that the template of\n  the receiving matrix is of the same type!\n\n  If   \n is the input vector, then this function\n  returns a matrix \n\n such as:\n\n  \n\n\n\n\n\n\n   And the three normal vectors are computed as:\n\n  \n\n If (dx!=0 or dy!=0):\n \n\n\n otherwise (the direction vector is vertical):\n \n\n\n And finally, the third vector is the cross product of the others:\n\n    \n\n \n The 3x3 matrix (CMatrixDynamic<T>), containing one vector\n  per column.\n  Throws an std::exception on invalid input (i.e. null direction\n  vector)\n \n\n generateAxisBaseFromDirectionAndAxis()\n\n (JLB @ 18-SEP-2007)\n\nC++: mrpt::math::generateAxisBaseFromDirection(double, double, double) --> class mrpt::math::CMatrixFixed<double, 3, 3>", pybind11::arg("dx"), pybind11::arg("dy"), pybind11::arg("dz"));

	// mrpt::math::signedArea(const class mrpt::math::TPolygon2D &) file:mrpt/math/geometry.h line:1075
	M("mrpt::math").def("signedArea", (double (*)(const class mrpt::math::TPolygon2D &)) &mrpt::math::signedArea, "Returns the area of a polygon, positive if vertices listed in CCW ordering,\n negative if CW.\n\n  \n (New in MRPT 2.4.1)\n\nC++: mrpt::math::signedArea(const class mrpt::math::TPolygon2D &) --> double", pybind11::arg("p"));

}
