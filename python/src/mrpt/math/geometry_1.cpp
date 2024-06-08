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
#include <mrpt/math/TLine2D.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TObject2D.h>
#include <mrpt/math/TObject3D.h>
#include <mrpt/math/TPlane.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPolygon2D.h>
#include <mrpt/math/TPolygon3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TSegment2D.h>
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

void bind_mrpt_math_geometry_1(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::math::intersect(const struct mrpt::math::TObject2D &, const struct mrpt::math::TObject2D &, struct mrpt::math::TObject2D &) file:mrpt/math/geometry.h line:556
	M("mrpt::math").def("intersect", (bool (*)(const struct mrpt::math::TObject2D &, const struct mrpt::math::TObject2D &, struct mrpt::math::TObject2D &)) &mrpt::math::intersect, "Gets the intersection between any pair of 2D objects.\n\nC++: mrpt::math::intersect(const struct mrpt::math::TObject2D &, const struct mrpt::math::TObject2D &, struct mrpt::math::TObject2D &) --> bool", pybind11::arg("o1"), pybind11::arg("o2"), pybind11::arg("obj"));

	// mrpt::math::intersect(const struct mrpt::math::TObject3D &, const struct mrpt::math::TObject3D &, struct mrpt::math::TObject3D &) file:mrpt/math/geometry.h line:558
	M("mrpt::math").def("intersect", (bool (*)(const struct mrpt::math::TObject3D &, const struct mrpt::math::TObject3D &, struct mrpt::math::TObject3D &)) &mrpt::math::intersect, "Gets the intersection between any pair of 3D objects.\n\nC++: mrpt::math::intersect(const struct mrpt::math::TObject3D &, const struct mrpt::math::TObject3D &, struct mrpt::math::TObject3D &) --> bool", pybind11::arg("o1"), pybind11::arg("o2"), pybind11::arg("obj"));

	// mrpt::math::distance(const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPoint2D_<double> &) file:mrpt/math/geometry.h line:566
	M("mrpt::math").def("distance", (double (*)(const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPoint2D_<double> &)) &mrpt::math::distance, "@{\n\n Gets the distance between two points in a 2D space. \n\nC++: mrpt::math::distance(const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPoint2D_<double> &) --> double", pybind11::arg("p1"), pybind11::arg("p2"));

	// mrpt::math::distance(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &) file:mrpt/math/geometry.h line:568
	M("mrpt::math").def("distance", (double (*)(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &)) &mrpt::math::distance, "Gets the distance between two points in a 3D space. \n\nC++: mrpt::math::distance(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &) --> double", pybind11::arg("p1"), pybind11::arg("p2"));

	// mrpt::math::distance(const struct mrpt::math::TLine2D &, const struct mrpt::math::TLine2D &) file:mrpt/math/geometry.h line:570
	M("mrpt::math").def("distance", (double (*)(const struct mrpt::math::TLine2D &, const struct mrpt::math::TLine2D &)) &mrpt::math::distance, "Gets the distance between two lines in a 2D space. \n\nC++: mrpt::math::distance(const struct mrpt::math::TLine2D &, const struct mrpt::math::TLine2D &) --> double", pybind11::arg("r1"), pybind11::arg("r2"));

	// mrpt::math::distance(const struct mrpt::math::TLine3D &, const struct mrpt::math::TLine3D &) file:mrpt/math/geometry.h line:572
	M("mrpt::math").def("distance", (double (*)(const struct mrpt::math::TLine3D &, const struct mrpt::math::TLine3D &)) &mrpt::math::distance, "Gets the distance between two lines in a 3D space. \n\nC++: mrpt::math::distance(const struct mrpt::math::TLine3D &, const struct mrpt::math::TLine3D &) --> double", pybind11::arg("r1"), pybind11::arg("r2"));

	// mrpt::math::distance(const struct mrpt::math::TPlane &, const struct mrpt::math::TPlane &) file:mrpt/math/geometry.h line:575
	M("mrpt::math").def("distance", (double (*)(const struct mrpt::math::TPlane &, const struct mrpt::math::TPlane &)) &mrpt::math::distance, "Gets the distance between two planes. It will be zero if the planes are not\n parallel. \n\nC++: mrpt::math::distance(const struct mrpt::math::TPlane &, const struct mrpt::math::TPlane &) --> double", pybind11::arg("p1"), pybind11::arg("p2"));

	// mrpt::math::distance(const class mrpt::math::TPolygon2D &, const class mrpt::math::TPolygon2D &) file:mrpt/math/geometry.h line:577
	M("mrpt::math").def("distance", (double (*)(const class mrpt::math::TPolygon2D &, const class mrpt::math::TPolygon2D &)) &mrpt::math::distance, "Gets the distance between two polygons in a 2D space. \n\nC++: mrpt::math::distance(const class mrpt::math::TPolygon2D &, const class mrpt::math::TPolygon2D &) --> double", pybind11::arg("p1"), pybind11::arg("p2"));

	// mrpt::math::distance(const class mrpt::math::TPolygon2D &, const struct mrpt::math::TSegment2D &) file:mrpt/math/geometry.h line:579
	M("mrpt::math").def("distance", (double (*)(const class mrpt::math::TPolygon2D &, const struct mrpt::math::TSegment2D &)) &mrpt::math::distance, "Gets the distance between a polygon and a segment in a 2D space. \n\nC++: mrpt::math::distance(const class mrpt::math::TPolygon2D &, const struct mrpt::math::TSegment2D &) --> double", pybind11::arg("p1"), pybind11::arg("s2"));

	// mrpt::math::distance(const struct mrpt::math::TSegment2D &, const class mrpt::math::TPolygon2D &) file:mrpt/math/geometry.h line:581
	M("mrpt::math").def("distance", (double (*)(const struct mrpt::math::TSegment2D &, const class mrpt::math::TPolygon2D &)) &mrpt::math::distance, "Gets the distance between a segment and a polygon in a 2D space. \n\nC++: mrpt::math::distance(const struct mrpt::math::TSegment2D &, const class mrpt::math::TPolygon2D &) --> double", pybind11::arg("s1"), pybind11::arg("p2"));

	// mrpt::math::distance(const class mrpt::math::TPolygon2D &, const struct mrpt::math::TLine2D &) file:mrpt/math/geometry.h line:583
	M("mrpt::math").def("distance", (double (*)(const class mrpt::math::TPolygon2D &, const struct mrpt::math::TLine2D &)) &mrpt::math::distance, "Gets the distance between a polygon and a line in a 2D space. \n\nC++: mrpt::math::distance(const class mrpt::math::TPolygon2D &, const struct mrpt::math::TLine2D &) --> double", pybind11::arg("p1"), pybind11::arg("l2"));

	// mrpt::math::distance(const struct mrpt::math::TLine2D &, const class mrpt::math::TPolygon2D &) file:mrpt/math/geometry.h line:584
	M("mrpt::math").def("distance", (double (*)(const struct mrpt::math::TLine2D &, const class mrpt::math::TPolygon2D &)) &mrpt::math::distance, "C++: mrpt::math::distance(const struct mrpt::math::TLine2D &, const class mrpt::math::TPolygon2D &) --> double", pybind11::arg("l1"), pybind11::arg("p2"));

	// mrpt::math::distance(const class mrpt::math::TPolygon3D &, const class mrpt::math::TPolygon3D &) file:mrpt/math/geometry.h line:586
	M("mrpt::math").def("distance", (double (*)(const class mrpt::math::TPolygon3D &, const class mrpt::math::TPolygon3D &)) &mrpt::math::distance, "Gets the distance between two polygons in a 3D space. \n\nC++: mrpt::math::distance(const class mrpt::math::TPolygon3D &, const class mrpt::math::TPolygon3D &) --> double", pybind11::arg("p1"), pybind11::arg("p2"));

	// mrpt::math::distance(const class mrpt::math::TPolygon3D &, const struct mrpt::math::TSegment3D &) file:mrpt/math/geometry.h line:588
	M("mrpt::math").def("distance", (double (*)(const class mrpt::math::TPolygon3D &, const struct mrpt::math::TSegment3D &)) &mrpt::math::distance, "Gets the distance between a polygon and a segment in a 3D space. \n\nC++: mrpt::math::distance(const class mrpt::math::TPolygon3D &, const struct mrpt::math::TSegment3D &) --> double", pybind11::arg("p1"), pybind11::arg("s2"));

	// mrpt::math::distance(const struct mrpt::math::TSegment3D &, const class mrpt::math::TPolygon3D &) file:mrpt/math/geometry.h line:590
	M("mrpt::math").def("distance", (double (*)(const struct mrpt::math::TSegment3D &, const class mrpt::math::TPolygon3D &)) &mrpt::math::distance, "Gets the distance between a segment and a polygon in a 3D space.\n\nC++: mrpt::math::distance(const struct mrpt::math::TSegment3D &, const class mrpt::math::TPolygon3D &) --> double", pybind11::arg("s1"), pybind11::arg("p2"));

	// mrpt::math::distance(const class mrpt::math::TPolygon3D &, const struct mrpt::math::TLine3D &) file:mrpt/math/geometry.h line:592
	M("mrpt::math").def("distance", (double (*)(const class mrpt::math::TPolygon3D &, const struct mrpt::math::TLine3D &)) &mrpt::math::distance, "Gets the distance between a polygon and a line in a 3D space. \n\nC++: mrpt::math::distance(const class mrpt::math::TPolygon3D &, const struct mrpt::math::TLine3D &) --> double", pybind11::arg("p1"), pybind11::arg("l2"));

	// mrpt::math::distance(const struct mrpt::math::TLine3D &, const class mrpt::math::TPolygon3D &) file:mrpt/math/geometry.h line:594
	M("mrpt::math").def("distance", (double (*)(const struct mrpt::math::TLine3D &, const class mrpt::math::TPolygon3D &)) &mrpt::math::distance, "Gets the distance between a line and a polygon in a 3D space \n\nC++: mrpt::math::distance(const struct mrpt::math::TLine3D &, const class mrpt::math::TPolygon3D &) --> double", pybind11::arg("l1"), pybind11::arg("p2"));

	// mrpt::math::distance(const class mrpt::math::TPolygon3D &, const struct mrpt::math::TPlane &) file:mrpt/math/geometry.h line:596
	M("mrpt::math").def("distance", (double (*)(const class mrpt::math::TPolygon3D &, const struct mrpt::math::TPlane &)) &mrpt::math::distance, "Gets the distance between a polygon and a plane. \n\nC++: mrpt::math::distance(const class mrpt::math::TPolygon3D &, const struct mrpt::math::TPlane &) --> double", pybind11::arg("po"), pybind11::arg("pl"));

	// mrpt::math::distance(const struct mrpt::math::TPlane &, const class mrpt::math::TPolygon3D &) file:mrpt/math/geometry.h line:598
	M("mrpt::math").def("distance", (double (*)(const struct mrpt::math::TPlane &, const class mrpt::math::TPolygon3D &)) &mrpt::math::distance, "Gets the distance between a plane and a polygon.\n\nC++: mrpt::math::distance(const struct mrpt::math::TPlane &, const class mrpt::math::TPolygon3D &) --> double", pybind11::arg("pl"), pybind11::arg("po"));

	// mrpt::math::createPlaneFromPoseXY(const struct mrpt::math::TPose3D &, struct mrpt::math::TPlane &) file:mrpt/math/geometry.h line:619
	M("mrpt::math").def("createPlaneFromPoseXY", (void (*)(const struct mrpt::math::TPose3D &, struct mrpt::math::TPlane &)) &mrpt::math::createPlaneFromPoseXY, "@{\n\n Given a pose, creates a plane orthogonal to its Z vector.\n \n\n createPlaneFromPoseXZ,createPlaneFromPoseYZ,createPlaneFromPoseAndNormal\n\nC++: mrpt::math::createPlaneFromPoseXY(const struct mrpt::math::TPose3D &, struct mrpt::math::TPlane &) --> void", pybind11::arg("pose"), pybind11::arg("plane"));

	// mrpt::math::createPlaneFromPoseXZ(const struct mrpt::math::TPose3D &, struct mrpt::math::TPlane &) file:mrpt/math/geometry.h line:624
	M("mrpt::math").def("createPlaneFromPoseXZ", (void (*)(const struct mrpt::math::TPose3D &, struct mrpt::math::TPlane &)) &mrpt::math::createPlaneFromPoseXZ, "Given a pose, creates a plane orthogonal to its Y vector.\n \n\n createPlaneFromPoseXY,createPlaneFromPoseYZ,createPlaneFromPoseAndNormal\n\nC++: mrpt::math::createPlaneFromPoseXZ(const struct mrpt::math::TPose3D &, struct mrpt::math::TPlane &) --> void", pybind11::arg("pose"), pybind11::arg("plane"));

	// mrpt::math::createPlaneFromPoseYZ(const struct mrpt::math::TPose3D &, struct mrpt::math::TPlane &) file:mrpt/math/geometry.h line:629
	M("mrpt::math").def("createPlaneFromPoseYZ", (void (*)(const struct mrpt::math::TPose3D &, struct mrpt::math::TPlane &)) &mrpt::math::createPlaneFromPoseYZ, "Given a pose, creates a plane orthogonal to its X vector.\n \n\n createPlaneFromPoseXY,createPlaneFromPoseXZ,createPlaneFromPoseAndNormal\n\nC++: mrpt::math::createPlaneFromPoseYZ(const struct mrpt::math::TPose3D &, struct mrpt::math::TPlane &) --> void", pybind11::arg("pose"), pybind11::arg("plane"));

	// mrpt::math::generateAxisBaseFromDirectionAndAxis(const struct mrpt::math::TPoint3D_<double> &, uint8_t) file:mrpt/math/geometry.h line:643
	M("mrpt::math").def("generateAxisBaseFromDirectionAndAxis", (class mrpt::math::CMatrixFixed<double, 4, 4> (*)(const struct mrpt::math::TPoint3D_<double> &, uint8_t)) &mrpt::math::generateAxisBaseFromDirectionAndAxis, "Creates a homogeneus matrix (4x4) such that the coordinate given (0 for x, 1\n for y, 2 for z) corresponds to the provided vector.\n \n\n must be a *unitary* vector\n \n\n generateAxisBaseFromDirectionAndAxis()\n\nC++: mrpt::math::generateAxisBaseFromDirectionAndAxis(const struct mrpt::math::TPoint3D_<double> &, uint8_t) --> class mrpt::math::CMatrixFixed<double, 4, 4>", pybind11::arg("vec"), pybind11::arg("coord"));

	// mrpt::math::getSegmentBisector(const struct mrpt::math::TSegment2D &, struct mrpt::math::TLine2D &) file:mrpt/math/geometry.h line:724
	M("mrpt::math").def("getSegmentBisector", (void (*)(const struct mrpt::math::TSegment2D &, struct mrpt::math::TLine2D &)) &mrpt::math::getSegmentBisector, "Gets the bisector of a 2D segment.\n\nC++: mrpt::math::getSegmentBisector(const struct mrpt::math::TSegment2D &, struct mrpt::math::TLine2D &) --> void", pybind11::arg("sgm"), pybind11::arg("bis"));

	// mrpt::math::getSegmentBisector(const struct mrpt::math::TSegment3D &, struct mrpt::math::TPlane &) file:mrpt/math/geometry.h line:728
	M("mrpt::math").def("getSegmentBisector", (void (*)(const struct mrpt::math::TSegment3D &, struct mrpt::math::TPlane &)) &mrpt::math::getSegmentBisector, "Gets the bisector of a 3D segment.\n\nC++: mrpt::math::getSegmentBisector(const struct mrpt::math::TSegment3D &, struct mrpt::math::TPlane &) --> void", pybind11::arg("sgm"), pybind11::arg("bis"));

	// mrpt::math::getAngleBisector(const struct mrpt::math::TLine2D &, const struct mrpt::math::TLine2D &, struct mrpt::math::TLine2D &) file:mrpt/math/geometry.h line:733
	M("mrpt::math").def("getAngleBisector", (void (*)(const struct mrpt::math::TLine2D &, const struct mrpt::math::TLine2D &, struct mrpt::math::TLine2D &)) &mrpt::math::getAngleBisector, "Gets the bisector of two lines or segments (implicit constructor will be\n used if necessary)\n\nC++: mrpt::math::getAngleBisector(const struct mrpt::math::TLine2D &, const struct mrpt::math::TLine2D &, struct mrpt::math::TLine2D &) --> void", pybind11::arg("l1"), pybind11::arg("l2"), pybind11::arg("bis"));

	// mrpt::math::getAngleBisector(const struct mrpt::math::TLine3D &, const struct mrpt::math::TLine3D &, struct mrpt::math::TLine3D &) file:mrpt/math/geometry.h line:739
	M("mrpt::math").def("getAngleBisector", (void (*)(const struct mrpt::math::TLine3D &, const struct mrpt::math::TLine3D &, struct mrpt::math::TLine3D &)) &mrpt::math::getAngleBisector, "Gets the bisector of two lines or segments (implicit constructor will be\n used if necessary)\n \n\n std::logic_error if the lines do not fit in a single plane.\n\nC++: mrpt::math::getAngleBisector(const struct mrpt::math::TLine3D &, const struct mrpt::math::TLine3D &, struct mrpt::math::TLine3D &) --> void", pybind11::arg("l1"), pybind11::arg("l2"), pybind11::arg("bis"));

	// mrpt::math::closestFromPointToSegment(double, double, double, double, double, double, double &, double &) file:mrpt/math/geometry.h line:890
	M("mrpt::math").def("closestFromPointToSegment", (void (*)(double, double, double, double, double, double, double &, double &)) &mrpt::math::closestFromPointToSegment, "Computes the closest point from a given point to a segment.\n \n\n closestFromPointToLine\n\nC++: mrpt::math::closestFromPointToSegment(double, double, double, double, double, double, double &, double &) --> void", pybind11::arg("Px"), pybind11::arg("Py"), pybind11::arg("x1"), pybind11::arg("y1"), pybind11::arg("x2"), pybind11::arg("y2"), pybind11::arg("out_x"), pybind11::arg("out_y"));

	// mrpt::math::closestFromPointToSegment(const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPoint2D_<double> &) file:mrpt/math/geometry.h line:894
	M("mrpt::math").def("closestFromPointToSegment", (struct mrpt::math::TPoint2D_<double> (*)(const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPoint2D_<double> &)) &mrpt::math::closestFromPointToSegment, "C++: mrpt::math::closestFromPointToSegment(const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPoint2D_<double> &) --> struct mrpt::math::TPoint2D_<double>", pybind11::arg("query"), pybind11::arg("segPt1"), pybind11::arg("segPt2"));

	// mrpt::math::closestFromPointToLine(double, double, double, double, double, double, double &, double &) file:mrpt/math/geometry.h line:907
	M("mrpt::math").def("closestFromPointToLine", (void (*)(double, double, double, double, double, double, double &, double &)) &mrpt::math::closestFromPointToLine, "Computes the closest point from a given point to a (infinite) line.\n \n\n closestFromPointToSegment\n\nC++: mrpt::math::closestFromPointToLine(double, double, double, double, double, double, double &, double &) --> void", pybind11::arg("Px"), pybind11::arg("Py"), pybind11::arg("x1"), pybind11::arg("y1"), pybind11::arg("x2"), pybind11::arg("y2"), pybind11::arg("out_x"), pybind11::arg("out_y"));

}
