#include <ios>
#include <iterator>
#include <locale>
#include <memory>
#include <mrpt/math/TLine2D.h>
#include <mrpt/math/TLine3D.h>
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

void bind_mrpt_math_TLine3D(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::math::TLine3D file:mrpt/math/TLine3D.h line:25
		pybind11::class_<mrpt::math::TLine3D, std::shared_ptr<mrpt::math::TLine3D>> cl(M("mrpt::math"), "TLine3D", "3D line, represented by a base point and a director vector.\n \n\n TLine2D,TSegment3D,TPlane,TPolygon3D,TPoint3D");
		cl.def( pybind11::init( [](){ return new mrpt::math::TLine3D(); } ) );
		cl.def( pybind11::init<const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &>(), pybind11::arg("p1"), pybind11::arg("p2") );

		cl.def( pybind11::init<const struct mrpt::math::TSegment3D &>(), pybind11::arg("s") );

		cl.def( pybind11::init<const struct mrpt::math::TLine2D &>(), pybind11::arg("l") );

		cl.def( pybind11::init( [](mrpt::math::TLine3D const &o){ return new mrpt::math::TLine3D(o); } ) );
		cl.def_readwrite("pBase", &mrpt::math::TLine3D::pBase);
		cl.def_readwrite("director", &mrpt::math::TLine3D::director);
		cl.def_static("FromPointAndDirector", (struct mrpt::math::TLine3D (*)(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &)) &mrpt::math::TLine3D::FromPointAndDirector, "Static constructor from a point and a director vector.\n \n\n [New in MRPT 2.0.4]\n\nC++: mrpt::math::TLine3D::FromPointAndDirector(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &) --> struct mrpt::math::TLine3D", pybind11::arg("basePoint"), pybind11::arg("directorVector"));
		cl.def_static("FromTwoPoints", (struct mrpt::math::TLine3D (*)(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &)) &mrpt::math::TLine3D::FromTwoPoints, "Static constructor from two points.\n \n\n [New in MRPT 2.0.4]\n \n\n mrpt::math::getRegressionLine()\n\nC++: mrpt::math::TLine3D::FromTwoPoints(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &) --> struct mrpt::math::TLine3D", pybind11::arg("p1"), pybind11::arg("p2"));
		cl.def("contains", (bool (mrpt::math::TLine3D::*)(const struct mrpt::math::TPoint3D_<double> &) const) &mrpt::math::TLine3D::contains, "Check whether a point is inside the line \n\nC++: mrpt::math::TLine3D::contains(const struct mrpt::math::TPoint3D_<double> &) const --> bool", pybind11::arg("point"));
		cl.def("distance", (double (mrpt::math::TLine3D::*)(const struct mrpt::math::TPoint3D_<double> &) const) &mrpt::math::TLine3D::distance, "Absolute distance between the line and a point. \n\nC++: mrpt::math::TLine3D::distance(const struct mrpt::math::TPoint3D_<double> &) const --> double", pybind11::arg("point"));
		cl.def("closestPointTo", (struct mrpt::math::TPoint3D_<double> (mrpt::math::TLine3D::*)(const struct mrpt::math::TPoint3D_<double> &) const) &mrpt::math::TLine3D::closestPointTo, "Closest point to `p` along the line. It is computed as the intersection\n of `this` with the plane perpendicular to `this` that passes through `p`\n \n\n [New in MRPT 2.3.1]\n\nC++: mrpt::math::TLine3D::closestPointTo(const struct mrpt::math::TPoint3D_<double> &) const --> struct mrpt::math::TPoint3D_<double>", pybind11::arg("p"));
		cl.def("unitarize", (void (mrpt::math::TLine3D::*)()) &mrpt::math::TLine3D::unitarize, "Unitarize director vector.\n\nC++: mrpt::math::TLine3D::unitarize() --> void");
		cl.def("getDirectorVector", (const struct mrpt::math::TPoint3D_<double> & (mrpt::math::TLine3D::*)() const) &mrpt::math::TLine3D::getDirectorVector, "Get director vector (may be NOT unitary if not set so by the user) \n\n getUnitaryDirectorVector(), unitarize() \n\nC++: mrpt::math::TLine3D::getDirectorVector() const --> const struct mrpt::math::TPoint3D_<double> &", pybind11::return_value_policy::automatic);
		cl.def("generate2DObject", (void (mrpt::math::TLine3D::*)(struct mrpt::math::TLine2D &) const) &mrpt::math::TLine3D::generate2DObject, "Project into 2D space, discarding the Z coordinate.\n \n\n std::logic_error if the line's director vector is orthogonal to\n the XY plane.\n\nC++: mrpt::math::TLine3D::generate2DObject(struct mrpt::math::TLine2D &) const --> void", pybind11::arg("l"));
		cl.def("asString", (std::string (mrpt::math::TLine3D::*)() const) &mrpt::math::TLine3D::asString, "Returns \"P=[x,y,z] u=[ux,uy,uz]\"\n \n\n [New in MRPT 2.1.0]\n\nC++: mrpt::math::TLine3D::asString() const --> std::string");
		cl.def("assign", (struct mrpt::math::TLine3D & (mrpt::math::TLine3D::*)(const struct mrpt::math::TLine3D &)) &mrpt::math::TLine3D::operator=, "C++: mrpt::math::TLine3D::operator=(const struct mrpt::math::TLine3D &) --> struct mrpt::math::TLine3D &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		cl.def("__str__", [](mrpt::math::TLine3D const &o) -> std::string { std::ostringstream s; using namespace mrpt::math; s << o; return s.str(); } );
	}
}
