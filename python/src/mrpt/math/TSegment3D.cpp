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

void bind_mrpt_math_TSegment3D(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::math::TSegment3D file:mrpt/math/TSegment3D.h line:21
		pybind11::class_<mrpt::math::TSegment3D, std::shared_ptr<mrpt::math::TSegment3D>> cl(M("mrpt::math"), "TSegment3D", "3D segment, consisting of two points.\n \n\n TSegment2D,TLine3D,TPlane,TPolygon3D,TPoint3D\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::math::TSegment3D(); } ) );
		cl.def( pybind11::init<const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &>(), pybind11::arg("p1"), pybind11::arg("p2") );

		cl.def( pybind11::init<const struct mrpt::math::TSegment2D &>(), pybind11::arg("s") );

		cl.def( pybind11::init( [](mrpt::math::TSegment3D const &o){ return new mrpt::math::TSegment3D(o); } ) );
		cl.def_readwrite("point1", &mrpt::math::TSegment3D::point1);
		cl.def_readwrite("point2", &mrpt::math::TSegment3D::point2);
		cl.def_static("FromPoints", (struct mrpt::math::TSegment3D (*)(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &)) &mrpt::math::TSegment3D::FromPoints, "Static method, returns segment from two points \n New in MRPT 2.3.0\n\nC++: mrpt::math::TSegment3D::FromPoints(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &) --> struct mrpt::math::TSegment3D", pybind11::arg("p1"), pybind11::arg("p2"));
		cl.def("length", (double (mrpt::math::TSegment3D::*)() const) &mrpt::math::TSegment3D::length, "Segment length \n\nC++: mrpt::math::TSegment3D::length() const --> double");
		cl.def("distance", (double (mrpt::math::TSegment3D::*)(const struct mrpt::math::TPoint3D_<double> &) const) &mrpt::math::TSegment3D::distance, "Distance to point \n\nC++: mrpt::math::TSegment3D::distance(const struct mrpt::math::TPoint3D_<double> &) const --> double", pybind11::arg("point"));
		cl.def("distance", (double (mrpt::math::TSegment3D::*)(const struct mrpt::math::TSegment3D &) const) &mrpt::math::TSegment3D::distance, "Distance to another segment \n\nC++: mrpt::math::TSegment3D::distance(const struct mrpt::math::TSegment3D &) const --> double", pybind11::arg("segment"));
		cl.def("contains", (bool (mrpt::math::TSegment3D::*)(const struct mrpt::math::TPoint3D_<double> &) const) &mrpt::math::TSegment3D::contains, "Check whether a point is inside the segment.\n\nC++: mrpt::math::TSegment3D::contains(const struct mrpt::math::TPoint3D_<double> &) const --> bool", pybind11::arg("point"));
		cl.def("__getitem__", (struct mrpt::math::TPoint3D_<double> & (mrpt::math::TSegment3D::*)(size_t)) &mrpt::math::TSegment3D::operator[], "Access to points using operator[0-1] \n\nC++: mrpt::math::TSegment3D::operator[](size_t) --> struct mrpt::math::TPoint3D_<double> &", pybind11::return_value_policy::automatic, pybind11::arg("i"));
		cl.def("generate2DObject", (void (mrpt::math::TSegment3D::*)(struct mrpt::math::TSegment2D &) const) &mrpt::math::TSegment3D::generate2DObject, "Projection into 2D space, discarding the z.\n\nC++: mrpt::math::TSegment3D::generate2DObject(struct mrpt::math::TSegment2D &) const --> void", pybind11::arg("s"));
		cl.def("getCenter", (void (mrpt::math::TSegment3D::*)(struct mrpt::math::TPoint3D_<double> &) const) &mrpt::math::TSegment3D::getCenter, "Segment's central point.\n\nC++: mrpt::math::TSegment3D::getCenter(struct mrpt::math::TPoint3D_<double> &) const --> void", pybind11::arg("p"));
		cl.def("assign", (struct mrpt::math::TSegment3D & (mrpt::math::TSegment3D::*)(const struct mrpt::math::TSegment3D &)) &mrpt::math::TSegment3D::operator=, "C++: mrpt::math::TSegment3D::operator=(const struct mrpt::math::TSegment3D &) --> struct mrpt::math::TSegment3D &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		cl.def("__str__", [](mrpt::math::TSegment3D const &o) -> std::string { std::ostringstream s; using namespace mrpt::math; s << o; return s.str(); } );
	}
}
