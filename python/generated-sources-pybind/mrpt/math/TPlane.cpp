#include <ios>
#include <iterator>
#include <locale>
#include <memory>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TLine2D.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TPlane.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TSegment2D.h>
#include <mrpt/math/TSegment3D.h>
#include <optional>
#include <ostream>
#include <sstream> // __str__
#include <streambuf>
#include <string>

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

void bind_mrpt_math_TPlane(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::math::TPlane file:mrpt/math/TPlane.h line:28
		pybind11::class_<mrpt::math::TPlane, std::shared_ptr<mrpt::math::TPlane>> cl(M("mrpt::math"), "TPlane", "3D Plane, represented by its equation \n\n \n TSegment3D,TLine3D,TPolygon3D,TPoint3D");
		cl.def( pybind11::init( [](){ return new mrpt::math::TPlane(); } ) );
		cl.def( pybind11::init<double, double, double, double>(), pybind11::arg("A"), pybind11::arg("B"), pybind11::arg("C"), pybind11::arg("D") );

		cl.def( pybind11::init<const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &>(), pybind11::arg("p1"), pybind11::arg("p2"), pybind11::arg("p3") );

		cl.def( pybind11::init<const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &>(), pybind11::arg("p1"), pybind11::arg("normal") );

		cl.def( pybind11::init<const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TLine3D &>(), pybind11::arg("p1"), pybind11::arg("r2") );

		cl.def( pybind11::init<const struct mrpt::math::TLine3D &, const struct mrpt::math::TLine3D &>(), pybind11::arg("r1"), pybind11::arg("r2") );

		cl.def( pybind11::init( [](mrpt::math::TPlane const &o){ return new mrpt::math::TPlane(o); } ) );
		cl.def_readwrite("coefs", &mrpt::math::TPlane::coefs);
		cl.def_static("From3Points", (struct mrpt::math::TPlane (*)(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &)) &mrpt::math::TPlane::From3Points, "[New in MRPT 2.1.0]\n \n\n mrpt::math::getRegressionPlane()\n\nC++: mrpt::math::TPlane::From3Points(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &) --> struct mrpt::math::TPlane", pybind11::arg("p1"), pybind11::arg("p2"), pybind11::arg("p3"));
		cl.def_static("FromPointAndNormal", (struct mrpt::math::TPlane (*)(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &)) &mrpt::math::TPlane::FromPointAndNormal, "[New in MRPT 2.1.0] \n\nC++: mrpt::math::TPlane::FromPointAndNormal(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &) --> struct mrpt::math::TPlane", pybind11::arg("p1"), pybind11::arg("normal"));
		cl.def_static("FromPointAndLine", (struct mrpt::math::TPlane (*)(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TLine3D &)) &mrpt::math::TPlane::FromPointAndLine, "Defines a plane which contains this point and this line.\n \n\n std::logic_error if the point is inside the line.\n \n\n [New in MRPT 2.1.0]\n\nC++: mrpt::math::TPlane::FromPointAndLine(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TLine3D &) --> struct mrpt::math::TPlane", pybind11::arg("p1"), pybind11::arg("r"));
		cl.def_static("FromTwoLines", (struct mrpt::math::TPlane (*)(const struct mrpt::math::TLine3D &, const struct mrpt::math::TLine3D &)) &mrpt::math::TPlane::FromTwoLines, "Defines a plane which contains the two lines.\n \n\n std::logic_error if the lines do not cross.\n\nC++: mrpt::math::TPlane::FromTwoLines(const struct mrpt::math::TLine3D &, const struct mrpt::math::TLine3D &) --> struct mrpt::math::TPlane", pybind11::arg("r1"), pybind11::arg("r2"));
		cl.def("evaluatePoint", (double (mrpt::math::TPlane::*)(const struct mrpt::math::TPoint3D_<double> &) const) &mrpt::math::TPlane::evaluatePoint, "Evaluate a point in the plane's equation \n\nC++: mrpt::math::TPlane::evaluatePoint(const struct mrpt::math::TPoint3D_<double> &) const --> double", pybind11::arg("point"));
		cl.def("contains", (bool (mrpt::math::TPlane::*)(const struct mrpt::math::TPoint3D_<double> &) const) &mrpt::math::TPlane::contains, "Check whether a point is contained into the plane.\n\nC++: mrpt::math::TPlane::contains(const struct mrpt::math::TPoint3D_<double> &) const --> bool", pybind11::arg("point"));
		cl.def("contains", (bool (mrpt::math::TPlane::*)(const struct mrpt::math::TSegment3D &) const) &mrpt::math::TPlane::contains, "Check whether a segment is fully contained into the plane.\n\nC++: mrpt::math::TPlane::contains(const struct mrpt::math::TSegment3D &) const --> bool", pybind11::arg("segment"));
		cl.def("contains", (bool (mrpt::math::TPlane::*)(const struct mrpt::math::TLine3D &) const) &mrpt::math::TPlane::contains, "Check whether a line is fully contained into the plane.\n\nC++: mrpt::math::TPlane::contains(const struct mrpt::math::TLine3D &) const --> bool", pybind11::arg("line"));
		cl.def("distance", (double (mrpt::math::TPlane::*)(const struct mrpt::math::TPoint3D_<double> &) const) &mrpt::math::TPlane::distance, "Absolute distance to 3D point \n\nC++: mrpt::math::TPlane::distance(const struct mrpt::math::TPoint3D_<double> &) const --> double", pybind11::arg("point"));
		cl.def("signedDistance", (double (mrpt::math::TPlane::*)(const struct mrpt::math::TPoint3D_<double> &) const) &mrpt::math::TPlane::signedDistance, "Signed distance (positive on the normal vector side) to 3D point.\n  \n\n (New in MRPT 2.4.9)\n\nC++: mrpt::math::TPlane::signedDistance(const struct mrpt::math::TPoint3D_<double> &) const --> double", pybind11::arg("point"));
		cl.def("distance", (double (mrpt::math::TPlane::*)(const struct mrpt::math::TLine3D &) const) &mrpt::math::TPlane::distance, "Distance to 3D line. Will be zero if the line is not parallel to the\n plane.\n\nC++: mrpt::math::TPlane::distance(const struct mrpt::math::TLine3D &) const --> double", pybind11::arg("line"));
		cl.def("getNormalVector", (struct mrpt::math::TPoint3D_<double> (mrpt::math::TPlane::*)() const) &mrpt::math::TPlane::getNormalVector, "Get plane's normal vector \n\nC++: mrpt::math::TPlane::getNormalVector() const --> struct mrpt::math::TPoint3D_<double>");
		cl.def("unitarize", (void (mrpt::math::TPlane::*)()) &mrpt::math::TPlane::unitarize, "Unitarize normal vector.\n\nC++: mrpt::math::TPlane::unitarize() --> void");
		cl.def("getAsPose3D", (void (mrpt::math::TPlane::*)(struct mrpt::math::TPose3D &) const) &mrpt::math::TPlane::getAsPose3D, "C++: mrpt::math::TPlane::getAsPose3D(struct mrpt::math::TPose3D &) const --> void", pybind11::arg("outPose"));
		cl.def("getAsPose3DForcingOrigin", (void (mrpt::math::TPlane::*)(const struct mrpt::math::TPoint3D_<double> &, struct mrpt::math::TPose3D &) const) &mrpt::math::TPlane::getAsPose3DForcingOrigin, "C++: mrpt::math::TPlane::getAsPose3DForcingOrigin(const struct mrpt::math::TPoint3D_<double> &, struct mrpt::math::TPose3D &) const --> void", pybind11::arg("center"), pybind11::arg("pose"));
		cl.def("getAsPose3DForcingOrigin", (struct mrpt::math::TPose3D (mrpt::math::TPlane::*)(const struct mrpt::math::TPoint3D_<double> &) const) &mrpt::math::TPlane::getAsPose3DForcingOrigin, "C++: mrpt::math::TPlane::getAsPose3DForcingOrigin(const struct mrpt::math::TPoint3D_<double> &) const --> struct mrpt::math::TPose3D", pybind11::arg("center"));
		cl.def("getUnitaryNormalVector", (struct mrpt::math::TPoint3D_<double> (mrpt::math::TPlane::*)() const) &mrpt::math::TPlane::getUnitaryNormalVector, "Get normal vector \n\nC++: mrpt::math::TPlane::getUnitaryNormalVector() const --> struct mrpt::math::TPoint3D_<double>");
		cl.def("asString", (std::string (mrpt::math::TPlane::*)() const) &mrpt::math::TPlane::asString, "Returns \"[A, B, C, D]\"\n \n\n [New in MRPT 2.1.0]\n\nC++: mrpt::math::TPlane::asString() const --> std::string");
		cl.def("assign", (struct mrpt::math::TPlane & (mrpt::math::TPlane::*)(const struct mrpt::math::TPlane &)) &mrpt::math::TPlane::operator=, "C++: mrpt::math::TPlane::operator=(const struct mrpt::math::TPlane &) --> struct mrpt::math::TPlane &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		cl.def("__str__", [](mrpt::math::TPlane const &o) -> std::string { std::ostringstream s; using namespace mrpt::math; s << o; return s.str(); } );
	}
}
