#include <ios>
#include <iterator>
#include <locale>
#include <memory>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TSegment2D.h>
#include <mrpt/math/TSegment3D.h>
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

void bind_mrpt_math_TSegment2D(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::math::TSegment2D file:mrpt/math/TSegment2D.h line:21
		pybind11::class_<mrpt::math::TSegment2D, std::shared_ptr<mrpt::math::TSegment2D>> cl(M("mrpt::math"), "TSegment2D", "2D segment, consisting of two points.\n \n\n TSegment3D,TLine2D,TPolygon2D,TPoint2D\n \n\n\n ");
		cl.def( pybind11::init<const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPoint2D_<double> &>(), pybind11::arg("p1"), pybind11::arg("p2") );

		cl.def( pybind11::init( [](){ return new mrpt::math::TSegment2D(); } ) );
		cl.def( pybind11::init<const struct mrpt::math::TSegment3D &>(), pybind11::arg("s") );

		cl.def( pybind11::init( [](mrpt::math::TSegment2D const &o){ return new mrpt::math::TSegment2D(o); } ) );
		cl.def_readwrite("point1", &mrpt::math::TSegment2D::point1);
		cl.def_readwrite("point2", &mrpt::math::TSegment2D::point2);
		cl.def_static("FromPoints", (struct mrpt::math::TSegment2D (*)(const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPoint2D_<double> &)) &mrpt::math::TSegment2D::FromPoints, "Static method, returns segment from two points \n New in MRPT 2.3.0\n\nC++: mrpt::math::TSegment2D::FromPoints(const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPoint2D_<double> &) --> struct mrpt::math::TSegment2D", pybind11::arg("p1"), pybind11::arg("p2"));
		cl.def("length", (double (mrpt::math::TSegment2D::*)() const) &mrpt::math::TSegment2D::length, "Segment length.\n\nC++: mrpt::math::TSegment2D::length() const --> double");
		cl.def("distance", (double (mrpt::math::TSegment2D::*)(const struct mrpt::math::TPoint2D_<double> &) const) &mrpt::math::TSegment2D::distance, "Absolute distance to point. \n\nC++: mrpt::math::TSegment2D::distance(const struct mrpt::math::TPoint2D_<double> &) const --> double", pybind11::arg("point"));
		cl.def("signedDistance", (double (mrpt::math::TSegment2D::*)(const struct mrpt::math::TPoint2D_<double> &) const) &mrpt::math::TSegment2D::signedDistance, "Distance with sign to point (sign indicates which side the point is) \n\nC++: mrpt::math::TSegment2D::signedDistance(const struct mrpt::math::TPoint2D_<double> &) const --> double", pybind11::arg("point"));
		cl.def("contains", (bool (mrpt::math::TSegment2D::*)(const struct mrpt::math::TPoint2D_<double> &) const) &mrpt::math::TSegment2D::contains, "Check whether a point is inside a segment.\n\nC++: mrpt::math::TSegment2D::contains(const struct mrpt::math::TPoint2D_<double> &) const --> bool", pybind11::arg("point"));
		cl.def("__getitem__", (struct mrpt::math::TPoint2D_<double> & (mrpt::math::TSegment2D::*)(size_t)) &mrpt::math::TSegment2D::operator[], "Access to points using operator[0-1] \n\nC++: mrpt::math::TSegment2D::operator[](size_t) --> struct mrpt::math::TPoint2D_<double> &", pybind11::return_value_policy::automatic, pybind11::arg("i"));
		cl.def("generate3DObject", (void (mrpt::math::TSegment2D::*)(struct mrpt::math::TSegment3D &) const) &mrpt::math::TSegment2D::generate3DObject, "Project into 3D space, setting the z to 0.\n\nC++: mrpt::math::TSegment2D::generate3DObject(struct mrpt::math::TSegment3D &) const --> void", pybind11::arg("s"));
		cl.def("getCenter", (void (mrpt::math::TSegment2D::*)(struct mrpt::math::TPoint2D_<double> &) const) &mrpt::math::TSegment2D::getCenter, "Segment's central point.\n\nC++: mrpt::math::TSegment2D::getCenter(struct mrpt::math::TPoint2D_<double> &) const --> void", pybind11::arg("p"));
		cl.def("assign", (struct mrpt::math::TSegment2D & (mrpt::math::TSegment2D::*)(const struct mrpt::math::TSegment2D &)) &mrpt::math::TSegment2D::operator=, "C++: mrpt::math::TSegment2D::operator=(const struct mrpt::math::TSegment2D &) --> struct mrpt::math::TSegment2D &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		cl.def("__str__", [](mrpt::math::TSegment2D const &o) -> std::string { std::ostringstream s; using namespace mrpt::math; s << o; return s.str(); } );
	}
}
