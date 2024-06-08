#include <iterator>
#include <memory>
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
#include <mrpt/math/epsilon.h>
#include <mrpt/math/geometry.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <optional>
#include <string>
#include <tuple>
#include <type_traits>
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

void bind_mrpt_math_epsilon(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::math::setEpsilon(double) file:mrpt/math/epsilon.h line:18
	M("mrpt::math").def("setEpsilon", (void (*)(double)) &mrpt::math::setEpsilon, "Changes the value of the geometric epsilon (default = 1e-5)\n \n\n getEpsilon\n \n\n\n \n\nC++: mrpt::math::setEpsilon(double) --> void", pybind11::arg("nE"));

	// mrpt::math::getEpsilon() file:mrpt/math/epsilon.h line:24
	M("mrpt::math").def("getEpsilon", (double (*)()) &mrpt::math::getEpsilon, "Gets the value of the geometric epsilon  (default = 1e-5)\n \n\n setEpsilon\n \n\n\n \n\nC++: mrpt::math::getEpsilon() --> double");

	// mrpt::math::intersect(const struct mrpt::math::TSegment3D &, const struct mrpt::math::TSegment3D &, struct mrpt::math::TObject3D &) file:mrpt/math/geometry.h line:40
	M("mrpt::math").def("intersect", (bool (*)(const struct mrpt::math::TSegment3D &, const struct mrpt::math::TSegment3D &, struct mrpt::math::TObject3D &)) &mrpt::math::intersect, "operations.\n  @{\n\n Gets the intersection between two 3D segments. Possible outcomes:\n		- Segments intersect: Return=true, obj.getType()=GEOMETRIC_TYPE_POINT\n		- Segments don't intersect & are parallel: Return=true,\nobj.getType()=GEOMETRIC_TYPE_SEGMENT, obj is the segment \"in between\" both\nsegments.\n		- Segments don't intersect & aren't parallel: Return=false.\n \n\n TObject3D\n\nC++: mrpt::math::intersect(const struct mrpt::math::TSegment3D &, const struct mrpt::math::TSegment3D &, struct mrpt::math::TObject3D &) --> bool", pybind11::arg("s1"), pybind11::arg("s2"), pybind11::arg("obj"));

	// mrpt::math::intersect(const struct mrpt::math::TSegment3D &, const struct mrpt::math::TPlane &, struct mrpt::math::TObject3D &) file:mrpt/math/geometry.h line:50
	M("mrpt::math").def("intersect", (bool (*)(const struct mrpt::math::TSegment3D &, const struct mrpt::math::TPlane &, struct mrpt::math::TObject3D &)) &mrpt::math::intersect, "Gets the intersection between a 3D segment and a plane. Possible outcomes:\n		- Don't intersect: Return=false\n		- s1 is within the plane: Return=true,\nobj.getType()=GEOMETRIC_TYPE_SEGMENT\n		- s1 intersects the plane at one point: Return=true,\nobj.getType()=GEOMETRIC_TYPE_POINT\n \n\n TObject3D\n\nC++: mrpt::math::intersect(const struct mrpt::math::TSegment3D &, const struct mrpt::math::TPlane &, struct mrpt::math::TObject3D &) --> bool", pybind11::arg("s1"), pybind11::arg("p2"), pybind11::arg("obj"));

	// mrpt::math::intersect(const struct mrpt::math::TSegment3D &, const struct mrpt::math::TLine3D &, struct mrpt::math::TObject3D &) file:mrpt/math/geometry.h line:60
	M("mrpt::math").def("intersect", (bool (*)(const struct mrpt::math::TSegment3D &, const struct mrpt::math::TLine3D &, struct mrpt::math::TObject3D &)) &mrpt::math::intersect, "Gets the intersection between a 3D segment and a 3D line. Possible outcomes:\n		- They don't intersect : Return=false\n		- s1 lies within the line: Return=true,\nobj.getType()=GEOMETRIC_TYPE_SEGMENT\n		- s1 intersects the line at a point: Return=true,\nobj.getType()=GEOMETRIC_TYPE_POINT\n \n\n TObject3D\n\nC++: mrpt::math::intersect(const struct mrpt::math::TSegment3D &, const struct mrpt::math::TLine3D &, struct mrpt::math::TObject3D &) --> bool", pybind11::arg("s1"), pybind11::arg("r2"), pybind11::arg("obj"));

	// mrpt::math::intersect(const struct mrpt::math::TPlane &, const struct mrpt::math::TSegment3D &, struct mrpt::math::TObject3D &) file:mrpt/math/geometry.h line:70
	M("mrpt::math").def("intersect", (bool (*)(const struct mrpt::math::TPlane &, const struct mrpt::math::TSegment3D &, struct mrpt::math::TObject3D &)) &mrpt::math::intersect, "Gets the intersection between a plane and a 3D segment. Possible outcomes:\n		- Don't intersect: Return=false\n		- s2 is within the plane: Return=true,\nobj.getType()=GEOMETRIC_TYPE_SEGMENT\n		- s2 intersects the plane at one point: Return=true,\nobj.getType()=GEOMETRIC_TYPE_POINT\n \n\n TObject3D\n\nC++: mrpt::math::intersect(const struct mrpt::math::TPlane &, const struct mrpt::math::TSegment3D &, struct mrpt::math::TObject3D &) --> bool", pybind11::arg("p1"), pybind11::arg("s2"), pybind11::arg("obj"));

	// mrpt::math::intersect(const struct mrpt::math::TPlane &, const struct mrpt::math::TPlane &, struct mrpt::math::TObject3D &) file:mrpt/math/geometry.h line:81
	M("mrpt::math").def("intersect", (bool (*)(const struct mrpt::math::TPlane &, const struct mrpt::math::TPlane &, struct mrpt::math::TObject3D &)) &mrpt::math::intersect, "Gets the intersection between two planes. Possible outcomes:\n		- Planes are parallel: Return=false\n		- Planes intersect into a line: Return=true,\nobj.getType()=GEOMETRIC_TYPE_LINE\n \n\n TObject3D\n\nC++: mrpt::math::intersect(const struct mrpt::math::TPlane &, const struct mrpt::math::TPlane &, struct mrpt::math::TObject3D &) --> bool", pybind11::arg("p1"), pybind11::arg("p2"), pybind11::arg("obj"));

	// mrpt::math::intersect(const struct mrpt::math::TPlane &, const struct mrpt::math::TLine3D &, struct mrpt::math::TObject3D &) file:mrpt/math/geometry.h line:91
	M("mrpt::math").def("intersect", (bool (*)(const struct mrpt::math::TPlane &, const struct mrpt::math::TLine3D &, struct mrpt::math::TObject3D &)) &mrpt::math::intersect, "Gets the intersection between a plane and a 3D line. Possible outcomes:\n		- Line is parallel to plane but not within it: Return=false\n		- Line is contained in the plane: Return=true,\nobj.getType()=GEOMETRIC_TYPE_LINE\n		- Line intersects the plane at one point: Return=true,\nobj.getType()=GEOMETRIC_TYPE_POINT\n \n\n TObject3D\n\nC++: mrpt::math::intersect(const struct mrpt::math::TPlane &, const struct mrpt::math::TLine3D &, struct mrpt::math::TObject3D &) --> bool", pybind11::arg("p1"), pybind11::arg("p2"), pybind11::arg("obj"));

	// mrpt::math::intersect(const struct mrpt::math::TLine3D &, const struct mrpt::math::TSegment3D &, struct mrpt::math::TObject3D &) file:mrpt/math/geometry.h line:101
	M("mrpt::math").def("intersect", (bool (*)(const struct mrpt::math::TLine3D &, const struct mrpt::math::TSegment3D &, struct mrpt::math::TObject3D &)) &mrpt::math::intersect, "Gets the intersection between a 3D line and a 3D segment. Possible outcomes:\n		- They don't intersect : Return=false\n		- s2 lies within the line: Return=true,\nobj.getType()=GEOMETRIC_TYPE_SEGMENT\n		- s2 intersects the line at a point: Return=true,\nobj.getType()=GEOMETRIC_TYPE_POINT\n \n\n TObject3D\n\nC++: mrpt::math::intersect(const struct mrpt::math::TLine3D &, const struct mrpt::math::TSegment3D &, struct mrpt::math::TObject3D &) --> bool", pybind11::arg("r1"), pybind11::arg("s2"), pybind11::arg("obj"));

	// mrpt::math::intersect(const struct mrpt::math::TLine3D &, const struct mrpt::math::TPlane &, struct mrpt::math::TObject3D &) file:mrpt/math/geometry.h line:114
	M("mrpt::math").def("intersect", (bool (*)(const struct mrpt::math::TLine3D &, const struct mrpt::math::TPlane &, struct mrpt::math::TObject3D &)) &mrpt::math::intersect, "Gets the intersection between a 3D line and a plane. Possible outcomes:\n		- Line is parallel to plane but not within it: Return=false\n		- Line is contained in the plane: Return=true,\nobj.getType()=GEOMETRIC_TYPE_LINE\n		- Line intersects the plane at one point: Return=true,\nobj.getType()=GEOMETRIC_TYPE_POINT\n \n\n TObject3D\n\nC++: mrpt::math::intersect(const struct mrpt::math::TLine3D &, const struct mrpt::math::TPlane &, struct mrpt::math::TObject3D &) --> bool", pybind11::arg("r1"), pybind11::arg("p2"), pybind11::arg("obj"));

	// mrpt::math::intersect(const struct mrpt::math::TLine3D &, const struct mrpt::math::TLine3D &, struct mrpt::math::TObject3D &) file:mrpt/math/geometry.h line:128
	M("mrpt::math").def("intersect", (bool (*)(const struct mrpt::math::TLine3D &, const struct mrpt::math::TLine3D &, struct mrpt::math::TObject3D &)) &mrpt::math::intersect, "Gets the intersection between two 3D lines. Possible outcomes:\n		- Lines do not intersect: Return=false\n		- Lines are parallel and do not coincide: Return=false\n		- Lines coincide (are the same): Return=true,\nobj.getType()=GEOMETRIC_TYPE_LINE\n		- Lines intesect in a point: Return=true,\nobj.getType()=GEOMETRIC_TYPE_POINT\n \n\n TObject3D\n\nC++: mrpt::math::intersect(const struct mrpt::math::TLine3D &, const struct mrpt::math::TLine3D &, struct mrpt::math::TObject3D &) --> bool", pybind11::arg("r1"), pybind11::arg("r2"), pybind11::arg("obj"));

	// mrpt::math::intersect(const struct mrpt::math::TLine2D &, const struct mrpt::math::TLine2D &, struct mrpt::math::TObject2D &) file:mrpt/math/geometry.h line:139
	M("mrpt::math").def("intersect", (bool (*)(const struct mrpt::math::TLine2D &, const struct mrpt::math::TLine2D &, struct mrpt::math::TObject2D &)) &mrpt::math::intersect, "Gets the intersection between two 2D lines. Possible outcomes:\n		- Lines do not intersect: Return=false\n		- Lines are parallel and do not coincide: Return=false\n		- Lines coincide (are the same): Return=true,\nobj.getType()=GEOMETRIC_TYPE_LINE\n		- Lines intesect in a point: Return=true,\nobj.getType()=GEOMETRIC_TYPE_POINT\n \n\n TObject2D\n\nC++: mrpt::math::intersect(const struct mrpt::math::TLine2D &, const struct mrpt::math::TLine2D &, struct mrpt::math::TObject2D &) --> bool", pybind11::arg("r1"), pybind11::arg("r2"), pybind11::arg("obj"));

	// mrpt::math::intersect(const struct mrpt::math::TLine2D &, const struct mrpt::math::TSegment2D &, struct mrpt::math::TObject2D &) file:mrpt/math/geometry.h line:149
	M("mrpt::math").def("intersect", (bool (*)(const struct mrpt::math::TLine2D &, const struct mrpt::math::TSegment2D &, struct mrpt::math::TObject2D &)) &mrpt::math::intersect, "Gets the intersection between a 2D line and a 2D segment. Possible outcomes:\n		- They don't intersect: Return=false\n		- s2 lies within the line: Return=true,\nobj.getType()=GEOMETRIC_TYPE_SEGMENT\n		- Both intersects in one point: Return=true,\nobj.getType()=GEOMETRIC_TYPE_POINT\n \n\n TObject2D\n\nC++: mrpt::math::intersect(const struct mrpt::math::TLine2D &, const struct mrpt::math::TSegment2D &, struct mrpt::math::TObject2D &) --> bool", pybind11::arg("r1"), pybind11::arg("s2"), pybind11::arg("obj"));

	// mrpt::math::intersect(const struct mrpt::math::TSegment2D &, const struct mrpt::math::TLine2D &, struct mrpt::math::TObject2D &) file:mrpt/math/geometry.h line:159
	M("mrpt::math").def("intersect", (bool (*)(const struct mrpt::math::TSegment2D &, const struct mrpt::math::TLine2D &, struct mrpt::math::TObject2D &)) &mrpt::math::intersect, "Gets the intersection between a 2D line and a 2D segment. Possible outcomes:\n		- They don't intersect: Return=false\n		- s1 lies within the line: Return=true,\nobj.getType()=GEOMETRIC_TYPE_SEGMENT\n		- Both intersects in one point: Return=true,\nobj.getType()=GEOMETRIC_TYPE_POINT\n \n\n TObject2D\n\nC++: mrpt::math::intersect(const struct mrpt::math::TSegment2D &, const struct mrpt::math::TLine2D &, struct mrpt::math::TObject2D &) --> bool", pybind11::arg("s1"), pybind11::arg("r2"), pybind11::arg("obj"));

	// mrpt::math::intersect(const struct mrpt::math::TSegment2D &, const struct mrpt::math::TSegment2D &, struct mrpt::math::TObject2D &) file:mrpt/math/geometry.h line:172
	M("mrpt::math").def("intersect", (bool (*)(const struct mrpt::math::TSegment2D &, const struct mrpt::math::TSegment2D &, struct mrpt::math::TObject2D &)) &mrpt::math::intersect, "Gets the intersection between two 2D segments. Possible outcomes:\n		- Segments intersect: Return=true, obj.getType()=GEOMETRIC_TYPE_POINT\n		- Segments don't intersect & are parallel: Return=true,\nobj.getType()=GEOMETRIC_TYPE_SEGMENT, obj is the segment \"in between\" both\nsegments.\n		- Segments don't intersect & aren't parallel: Return=false.\n \n\n TObject2D\n\nC++: mrpt::math::intersect(const struct mrpt::math::TSegment2D &, const struct mrpt::math::TSegment2D &, struct mrpt::math::TObject2D &) --> bool", pybind11::arg("s1"), pybind11::arg("s2"), pybind11::arg("obj"));

	// mrpt::math::getAngle(const struct mrpt::math::TPlane &, const struct mrpt::math::TPlane &) file:mrpt/math/geometry.h line:184
	M("mrpt::math").def("getAngle", (double (*)(const struct mrpt::math::TPlane &, const struct mrpt::math::TPlane &)) &mrpt::math::getAngle, "automatically use TLines' implicit constructors.\n  @{\n\n Computes the angle between two planes.\n\nC++: mrpt::math::getAngle(const struct mrpt::math::TPlane &, const struct mrpt::math::TPlane &) --> double", pybind11::arg("p1"), pybind11::arg("p2"));

	// mrpt::math::getAngle(const struct mrpt::math::TPlane &, const struct mrpt::math::TLine3D &) file:mrpt/math/geometry.h line:189
	M("mrpt::math").def("getAngle", (double (*)(const struct mrpt::math::TPlane &, const struct mrpt::math::TLine3D &)) &mrpt::math::getAngle, "Computes the angle between a plane and a 3D line or segment (implicit\n constructor will be used if passing a segment instead of a line).\n\nC++: mrpt::math::getAngle(const struct mrpt::math::TPlane &, const struct mrpt::math::TLine3D &) --> double", pybind11::arg("p1"), pybind11::arg("r2"));

	// mrpt::math::getAngle(const struct mrpt::math::TLine3D &, const struct mrpt::math::TPlane &) file:mrpt/math/geometry.h line:194
	M("mrpt::math").def("getAngle", (double (*)(const struct mrpt::math::TLine3D &, const struct mrpt::math::TPlane &)) &mrpt::math::getAngle, "Computes the angle between a 3D line or segment and a plane (implicit\n constructor will be used if passing a segment instead of a line).\n\nC++: mrpt::math::getAngle(const struct mrpt::math::TLine3D &, const struct mrpt::math::TPlane &) --> double", pybind11::arg("r1"), pybind11::arg("p2"));

	// mrpt::math::getAngle(const struct mrpt::math::TLine3D &, const struct mrpt::math::TLine3D &) file:mrpt/math/geometry.h line:199
	M("mrpt::math").def("getAngle", (double (*)(const struct mrpt::math::TLine3D &, const struct mrpt::math::TLine3D &)) &mrpt::math::getAngle, "Computes the accute relative angle (range: [-PI/2,PI/2]) between two lines.\n \n\n Implicit constructor allows passing a segment as argument too.\n\nC++: mrpt::math::getAngle(const struct mrpt::math::TLine3D &, const struct mrpt::math::TLine3D &) --> double", pybind11::arg("r1"), pybind11::arg("r2"));

	// mrpt::math::getAngle(const struct mrpt::math::TLine2D &, const struct mrpt::math::TLine2D &) file:mrpt/math/geometry.h line:204
	M("mrpt::math").def("getAngle", (double (*)(const struct mrpt::math::TLine2D &, const struct mrpt::math::TLine2D &)) &mrpt::math::getAngle, "Computes the relative angle (range: [-PI,PI]) of line 2 wrt line 1.\n \n\n Implicit constructor allows passing a segment as argument too.\n\nC++: mrpt::math::getAngle(const struct mrpt::math::TLine2D &, const struct mrpt::math::TLine2D &) --> double", pybind11::arg("r1"), pybind11::arg("r2"));

	// mrpt::math::createFromPoseX(const struct mrpt::math::TPose3D &, struct mrpt::math::TLine3D &) file:mrpt/math/geometry.h line:216
	M("mrpt::math").def("createFromPoseX", (void (*)(const struct mrpt::math::TPose3D &, struct mrpt::math::TLine3D &)) &mrpt::math::createFromPoseX, "@{\n\n Gets a 3D line corresponding to the X axis in a given pose. An implicit\n constructor is used if a TPose3D is given.\n \n\n createFromPoseY,createFromPoseZ,createFromPoseAndVector\n\nC++: mrpt::math::createFromPoseX(const struct mrpt::math::TPose3D &, struct mrpt::math::TLine3D &) --> void", pybind11::arg("p"), pybind11::arg("r"));

	// mrpt::math::createFromPoseY(const struct mrpt::math::TPose3D &, struct mrpt::math::TLine3D &) file:mrpt/math/geometry.h line:222
	M("mrpt::math").def("createFromPoseY", (void (*)(const struct mrpt::math::TPose3D &, struct mrpt::math::TLine3D &)) &mrpt::math::createFromPoseY, "Gets a 3D line corresponding to the Y axis in a given pose. An implicit\n constructor is used if a TPose3D is given.\n \n\n createFromPoseX,createFromPoseZ,createFromPoseAndVector\n\nC++: mrpt::math::createFromPoseY(const struct mrpt::math::TPose3D &, struct mrpt::math::TLine3D &) --> void", pybind11::arg("p"), pybind11::arg("r"));

	// mrpt::math::createFromPoseZ(const struct mrpt::math::TPose3D &, struct mrpt::math::TLine3D &) file:mrpt/math/geometry.h line:228
	M("mrpt::math").def("createFromPoseZ", (void (*)(const struct mrpt::math::TPose3D &, struct mrpt::math::TLine3D &)) &mrpt::math::createFromPoseZ, "Gets a 3D line corresponding to the Z axis in a given pose. An implicit\n constructor is used if a TPose3D is given.\n \n\n createFromPoseX,createFromPoseY,createFromPoseAndVector\n\nC++: mrpt::math::createFromPoseZ(const struct mrpt::math::TPose3D &, struct mrpt::math::TLine3D &) --> void", pybind11::arg("p"), pybind11::arg("r"));

}
