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

void bind_mrpt_math_geometry(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::math::createFromPoseX(const struct mrpt::math::TPose2D &, struct mrpt::math::TLine2D &) file:mrpt/math/geometry.h line:240
	M("mrpt::math").def("createFromPoseX", (void (*)(const struct mrpt::math::TPose2D &, struct mrpt::math::TLine2D &)) &mrpt::math::createFromPoseX, "Gets a 2D line corresponding to the X axis in a given pose. An implicit\n constructor is used if a CPose2D is given.\n \n\n createFromPoseY,createFromPoseAndVector\n\nC++: mrpt::math::createFromPoseX(const struct mrpt::math::TPose2D &, struct mrpt::math::TLine2D &) --> void", pybind11::arg("p"), pybind11::arg("r"));

	// mrpt::math::createFromPoseY(const struct mrpt::math::TPose2D &, struct mrpt::math::TLine2D &) file:mrpt/math/geometry.h line:246
	M("mrpt::math").def("createFromPoseY", (void (*)(const struct mrpt::math::TPose2D &, struct mrpt::math::TLine2D &)) &mrpt::math::createFromPoseY, "Gets a 2D line corresponding to the Y axis in a given pose. An implicit\n constructor is used if a CPose2D is given.\n \n\n createFromPoseX,createFromPoseAndVector\n\nC++: mrpt::math::createFromPoseY(const struct mrpt::math::TPose2D &, struct mrpt::math::TLine2D &) --> void", pybind11::arg("p"), pybind11::arg("r"));

	// mrpt::math::project3D(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPose3D &, struct mrpt::math::TPoint3D_<double> &) file:mrpt/math/geometry.h line:298
	M("mrpt::math").def("project3D", (void (*)(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPose3D &, struct mrpt::math::TPoint3D_<double> &)) &mrpt::math::project3D, "@{\n\n Uses the given pose 3D to project a point into a new base \n\nC++: mrpt::math::project3D(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPose3D &, struct mrpt::math::TPoint3D_<double> &) --> void", pybind11::arg("point"), pybind11::arg("newXYpose"), pybind11::arg("newPoint"));

	// mrpt::math::project3D(const struct mrpt::math::TSegment3D &, const struct mrpt::math::TPose3D &, struct mrpt::math::TSegment3D &) file:mrpt/math/geometry.h line:304
	M("mrpt::math").def("project3D", (void (*)(const struct mrpt::math::TSegment3D &, const struct mrpt::math::TPose3D &, struct mrpt::math::TSegment3D &)) &mrpt::math::project3D, "Uses the given pose 3D to project a segment into a new base  \n\nC++: mrpt::math::project3D(const struct mrpt::math::TSegment3D &, const struct mrpt::math::TPose3D &, struct mrpt::math::TSegment3D &) --> void", pybind11::arg("segment"), pybind11::arg("newXYpose"), pybind11::arg("newSegment"));

	// mrpt::math::project3D(const struct mrpt::math::TLine3D &, const struct mrpt::math::TPose3D &, struct mrpt::math::TLine3D &) file:mrpt/math/geometry.h line:312
	M("mrpt::math").def("project3D", (void (*)(const struct mrpt::math::TLine3D &, const struct mrpt::math::TPose3D &, struct mrpt::math::TLine3D &)) &mrpt::math::project3D, "Uses the given pose 3D to project a line into a new base \n\nC++: mrpt::math::project3D(const struct mrpt::math::TLine3D &, const struct mrpt::math::TPose3D &, struct mrpt::math::TLine3D &) --> void", pybind11::arg("line"), pybind11::arg("newXYpose"), pybind11::arg("newLine"));

	// mrpt::math::project3D(const struct mrpt::math::TPlane &, const struct mrpt::math::TPose3D &, struct mrpt::math::TPlane &) file:mrpt/math/geometry.h line:314
	M("mrpt::math").def("project3D", (void (*)(const struct mrpt::math::TPlane &, const struct mrpt::math::TPose3D &, struct mrpt::math::TPlane &)) &mrpt::math::project3D, "Uses the given pose 3D to project a plane into a new base \n\nC++: mrpt::math::project3D(const struct mrpt::math::TPlane &, const struct mrpt::math::TPose3D &, struct mrpt::math::TPlane &) --> void", pybind11::arg("plane"), pybind11::arg("newXYpose"), pybind11::arg("newPlane"));

	// mrpt::math::project3D(const class mrpt::math::TPolygon3D &, const struct mrpt::math::TPose3D &, class mrpt::math::TPolygon3D &) file:mrpt/math/geometry.h line:316
	M("mrpt::math").def("project3D", (void (*)(const class mrpt::math::TPolygon3D &, const struct mrpt::math::TPose3D &, class mrpt::math::TPolygon3D &)) &mrpt::math::project3D, "Uses the given pose 3D to project a polygon into a new base \n\nC++: mrpt::math::project3D(const class mrpt::math::TPolygon3D &, const struct mrpt::math::TPose3D &, class mrpt::math::TPolygon3D &) --> void", pybind11::arg("polygon"), pybind11::arg("newXYpose"), pybind11::arg("newPolygon"));

	// mrpt::math::project3D(const struct mrpt::math::TObject3D &, const struct mrpt::math::TPose3D &, struct mrpt::math::TObject3D &) file:mrpt/math/geometry.h line:319
	M("mrpt::math").def("project3D", (void (*)(const struct mrpt::math::TObject3D &, const struct mrpt::math::TPose3D &, struct mrpt::math::TObject3D &)) &mrpt::math::project3D, "Uses the given pose 3D to project any 3D object into a new base. \n\nC++: mrpt::math::project3D(const struct mrpt::math::TObject3D &, const struct mrpt::math::TPose3D &, struct mrpt::math::TObject3D &) --> void", pybind11::arg("object"), pybind11::arg("newXYPose"), pybind11::arg("newObject"));

	// mrpt::math::project2D(const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPose2D &, struct mrpt::math::TPoint2D_<double> &) file:mrpt/math/geometry.h line:367
	M("mrpt::math").def("project2D", (void (*)(const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPose2D &, struct mrpt::math::TPoint2D_<double> &)) &mrpt::math::project2D, "Uses the given pose 2D to project a point into a new base. \n\nC++: mrpt::math::project2D(const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPose2D &, struct mrpt::math::TPoint2D_<double> &) --> void", pybind11::arg("point"), pybind11::arg("newXpose"), pybind11::arg("newPoint"));

	// mrpt::math::project2D(const struct mrpt::math::TSegment2D &, const struct mrpt::math::TPose2D &, struct mrpt::math::TSegment2D &) file:mrpt/math/geometry.h line:370
	M("mrpt::math").def("project2D", (void (*)(const struct mrpt::math::TSegment2D &, const struct mrpt::math::TPose2D &, struct mrpt::math::TSegment2D &)) &mrpt::math::project2D, "Uses the given pose 2D to project a segment into a new base  \n\nC++: mrpt::math::project2D(const struct mrpt::math::TSegment2D &, const struct mrpt::math::TPose2D &, struct mrpt::math::TSegment2D &) --> void", pybind11::arg("segment"), pybind11::arg("newXpose"), pybind11::arg("newSegment"));

	// mrpt::math::project2D(const struct mrpt::math::TLine2D &, const struct mrpt::math::TPose2D &, struct mrpt::math::TLine2D &) file:mrpt/math/geometry.h line:377
	M("mrpt::math").def("project2D", (void (*)(const struct mrpt::math::TLine2D &, const struct mrpt::math::TPose2D &, struct mrpt::math::TLine2D &)) &mrpt::math::project2D, "Uses the given pose 2D to project a line into a new base \n\nC++: mrpt::math::project2D(const struct mrpt::math::TLine2D &, const struct mrpt::math::TPose2D &, struct mrpt::math::TLine2D &) --> void", pybind11::arg("line"), pybind11::arg("newXpose"), pybind11::arg("newLine"));

	// mrpt::math::project2D(const class mrpt::math::TPolygon2D &, const struct mrpt::math::TPose2D &, class mrpt::math::TPolygon2D &) file:mrpt/math/geometry.h line:379
	M("mrpt::math").def("project2D", (void (*)(const class mrpt::math::TPolygon2D &, const struct mrpt::math::TPose2D &, class mrpt::math::TPolygon2D &)) &mrpt::math::project2D, "Uses the given pose 2D to project a polygon into a new base. \n\nC++: mrpt::math::project2D(const class mrpt::math::TPolygon2D &, const struct mrpt::math::TPose2D &, class mrpt::math::TPolygon2D &) --> void", pybind11::arg("polygon"), pybind11::arg("newXpose"), pybind11::arg("newPolygon"));

	// mrpt::math::project2D(const struct mrpt::math::TObject2D &, const struct mrpt::math::TPose2D &, struct mrpt::math::TObject2D &) file:mrpt/math/geometry.h line:381
	M("mrpt::math").def("project2D", (void (*)(const struct mrpt::math::TObject2D &, const struct mrpt::math::TPose2D &, struct mrpt::math::TObject2D &)) &mrpt::math::project2D, "Uses the given pose 2D to project any 2D object into a new base \n\nC++: mrpt::math::project2D(const struct mrpt::math::TObject2D &, const struct mrpt::math::TPose2D &, struct mrpt::math::TObject2D &) --> void", pybind11::arg("object"), pybind11::arg("newXpose"), pybind11::arg("newObject"));

	// mrpt::math::intersect(const class mrpt::math::TPolygon2D &, const struct mrpt::math::TSegment2D &, struct mrpt::math::TObject2D &) file:mrpt/math/geometry.h line:438
	M("mrpt::math").def("intersect", (bool (*)(const class mrpt::math::TPolygon2D &, const struct mrpt::math::TSegment2D &, struct mrpt::math::TObject2D &)) &mrpt::math::intersect, "than in raw numerical operations.\n  @{\n\n Gets the intersection between a 2D polygon and a 2D segment. \n TObject2D\n\nC++: mrpt::math::intersect(const class mrpt::math::TPolygon2D &, const struct mrpt::math::TSegment2D &, struct mrpt::math::TObject2D &) --> bool", pybind11::arg("p1"), pybind11::arg("s2"), pybind11::arg("obj"));

	// mrpt::math::intersect(const class mrpt::math::TPolygon2D &, const struct mrpt::math::TLine2D &, struct mrpt::math::TObject2D &) file:mrpt/math/geometry.h line:440
	M("mrpt::math").def("intersect", (bool (*)(const class mrpt::math::TPolygon2D &, const struct mrpt::math::TLine2D &, struct mrpt::math::TObject2D &)) &mrpt::math::intersect, "Gets the intersection between a 2D polygon and a 2D line. \n TObject2D  \n\nC++: mrpt::math::intersect(const class mrpt::math::TPolygon2D &, const struct mrpt::math::TLine2D &, struct mrpt::math::TObject2D &) --> bool", pybind11::arg("p1"), pybind11::arg("r2"), pybind11::arg("obj"));

	// mrpt::math::intersect(const class mrpt::math::TPolygon2D &, const class mrpt::math::TPolygon2D &) file:mrpt/math/geometry.h line:456
	M("mrpt::math").def("intersect", (class mrpt::math::TPolygon2D (*)(const class mrpt::math::TPolygon2D &, const class mrpt::math::TPolygon2D &)) &mrpt::math::intersect, "The [Sutherland-Hodgman\n algorithm](https://en.wikipedia.org/wiki/Sutherland%E2%80%93Hodgman_algorithm)\n  for finding the intersection between two 2D polygons.\n\n  Note that at least one of the polygons (`clipping`) must be **convex**.\n  The other polygon (`subject`) may be convex or concave.\n\n  See code example: \n\n  \n The intersection, or an empty (no points) polygon if there is no\n intersection at all.\n\n \n (New in MRPT 2.4.1)\n\nC++: mrpt::math::intersect(const class mrpt::math::TPolygon2D &, const class mrpt::math::TPolygon2D &) --> class mrpt::math::TPolygon2D", pybind11::arg("subject"), pybind11::arg("clipping"));

	// mrpt::math::intersect(const class mrpt::math::TPolygon2D &, const class mrpt::math::TPolygon2D &, struct mrpt::math::TObject2D &) file:mrpt/math/geometry.h line:461
	M("mrpt::math").def("intersect", (bool (*)(const class mrpt::math::TPolygon2D &, const class mrpt::math::TPolygon2D &, struct mrpt::math::TObject2D &)) &mrpt::math::intersect, "false if there is no intersection at all, true otherwise.\n\nC++: mrpt::math::intersect(const class mrpt::math::TPolygon2D &, const class mrpt::math::TPolygon2D &, struct mrpt::math::TObject2D &) --> bool", pybind11::arg("subject"), pybind11::arg("clipping"), pybind11::arg("result"));

	// mrpt::math::intersect(const struct mrpt::math::TSegment2D &, const class mrpt::math::TPolygon2D &, struct mrpt::math::TObject2D &) file:mrpt/math/geometry.h line:465
	M("mrpt::math").def("intersect", (bool (*)(const struct mrpt::math::TSegment2D &, const class mrpt::math::TPolygon2D &, struct mrpt::math::TObject2D &)) &mrpt::math::intersect, "Gets the intersection between a 2D segment and a 2D polygon.  \n TObject2D\n\nC++: mrpt::math::intersect(const struct mrpt::math::TSegment2D &, const class mrpt::math::TPolygon2D &, struct mrpt::math::TObject2D &) --> bool", pybind11::arg("s1"), pybind11::arg("p2"), pybind11::arg("obj"));

	// mrpt::math::intersect(const struct mrpt::math::TLine2D &, const class mrpt::math::TPolygon2D &, struct mrpt::math::TObject2D &) file:mrpt/math/geometry.h line:470
	M("mrpt::math").def("intersect", (bool (*)(const struct mrpt::math::TLine2D &, const class mrpt::math::TPolygon2D &, struct mrpt::math::TObject2D &)) &mrpt::math::intersect, "Gets the intersection between a 2D line and a 2D polygon.\n TObject2D \n\nC++: mrpt::math::intersect(const struct mrpt::math::TLine2D &, const class mrpt::math::TPolygon2D &, struct mrpt::math::TObject2D &) --> bool", pybind11::arg("r1"), pybind11::arg("p2"), pybind11::arg("obj"));

	// mrpt::math::intersect(const class mrpt::math::TPolygon3D &, const struct mrpt::math::TSegment3D &, struct mrpt::math::TObject3D &) file:mrpt/math/geometry.h line:476
	M("mrpt::math").def("intersect", (bool (*)(const class mrpt::math::TPolygon3D &, const struct mrpt::math::TSegment3D &, struct mrpt::math::TObject3D &)) &mrpt::math::intersect, "Gets the intersection between a 3D polygon and a 3D segment. \n TObject3D\n\nC++: mrpt::math::intersect(const class mrpt::math::TPolygon3D &, const struct mrpt::math::TSegment3D &, struct mrpt::math::TObject3D &) --> bool", pybind11::arg("p1"), pybind11::arg("s2"), pybind11::arg("obj"));

	// mrpt::math::intersect(const class mrpt::math::TPolygon3D &, const struct mrpt::math::TLine3D &, struct mrpt::math::TObject3D &) file:mrpt/math/geometry.h line:478
	M("mrpt::math").def("intersect", (bool (*)(const class mrpt::math::TPolygon3D &, const struct mrpt::math::TLine3D &, struct mrpt::math::TObject3D &)) &mrpt::math::intersect, "Gets the intersection between a 3D polygon and a 3D line. \n TObject3D  \n\nC++: mrpt::math::intersect(const class mrpt::math::TPolygon3D &, const struct mrpt::math::TLine3D &, struct mrpt::math::TObject3D &) --> bool", pybind11::arg("p1"), pybind11::arg("r2"), pybind11::arg("obj"));

	// mrpt::math::intersect(const class mrpt::math::TPolygon3D &, const struct mrpt::math::TPlane &, struct mrpt::math::TObject3D &) file:mrpt/math/geometry.h line:480
	M("mrpt::math").def("intersect", (bool (*)(const class mrpt::math::TPolygon3D &, const struct mrpt::math::TPlane &, struct mrpt::math::TObject3D &)) &mrpt::math::intersect, "Gets the intersection between a 3D polygon and a plane. \n TObject3D \n\nC++: mrpt::math::intersect(const class mrpt::math::TPolygon3D &, const struct mrpt::math::TPlane &, struct mrpt::math::TObject3D &) --> bool", pybind11::arg("p1"), pybind11::arg("p2"), pybind11::arg("obj"));

	// mrpt::math::intersect(const class mrpt::math::TPolygon3D &, const class mrpt::math::TPolygon3D &, struct mrpt::math::TObject3D &) file:mrpt/math/geometry.h line:482
	M("mrpt::math").def("intersect", (bool (*)(const class mrpt::math::TPolygon3D &, const class mrpt::math::TPolygon3D &, struct mrpt::math::TObject3D &)) &mrpt::math::intersect, "Gets the intersection between two 3D polygons. \n TObject3D \n\nC++: mrpt::math::intersect(const class mrpt::math::TPolygon3D &, const class mrpt::math::TPolygon3D &, struct mrpt::math::TObject3D &) --> bool", pybind11::arg("p1"), pybind11::arg("p2"), pybind11::arg("obj"));

	// mrpt::math::intersect(const struct mrpt::math::TSegment3D &, const class mrpt::math::TPolygon3D &, struct mrpt::math::TObject3D &) file:mrpt/math/geometry.h line:485
	M("mrpt::math").def("intersect", (bool (*)(const struct mrpt::math::TSegment3D &, const class mrpt::math::TPolygon3D &, struct mrpt::math::TObject3D &)) &mrpt::math::intersect, "Gets the intersection between a 3D segment and a 3D polygon. \n TObject3D\n\nC++: mrpt::math::intersect(const struct mrpt::math::TSegment3D &, const class mrpt::math::TPolygon3D &, struct mrpt::math::TObject3D &) --> bool", pybind11::arg("s1"), pybind11::arg("p2"), pybind11::arg("obj"));

	// mrpt::math::intersect(const struct mrpt::math::TLine3D &, const class mrpt::math::TPolygon3D &, struct mrpt::math::TObject3D &) file:mrpt/math/geometry.h line:490
	M("mrpt::math").def("intersect", (bool (*)(const struct mrpt::math::TLine3D &, const class mrpt::math::TPolygon3D &, struct mrpt::math::TObject3D &)) &mrpt::math::intersect, "Gets the intersection between a 3D line and a 3D polygon.\n TObject3D  \n\nC++: mrpt::math::intersect(const struct mrpt::math::TLine3D &, const class mrpt::math::TPolygon3D &, struct mrpt::math::TObject3D &) --> bool", pybind11::arg("r1"), pybind11::arg("p2"), pybind11::arg("obj"));

	// mrpt::math::intersect(const struct mrpt::math::TPlane &, const class mrpt::math::TPolygon3D &, struct mrpt::math::TObject3D &) file:mrpt/math/geometry.h line:495
	M("mrpt::math").def("intersect", (bool (*)(const struct mrpt::math::TPlane &, const class mrpt::math::TPolygon3D &, struct mrpt::math::TObject3D &)) &mrpt::math::intersect, "Gets the intersection between a plane and a 3D polygon. \n TObject3D \n\nC++: mrpt::math::intersect(const struct mrpt::math::TPlane &, const class mrpt::math::TPolygon3D &, struct mrpt::math::TObject3D &) --> bool", pybind11::arg("p1"), pybind11::arg("p2"), pybind11::arg("obj"));

}
