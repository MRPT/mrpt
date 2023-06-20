#include <ios>
#include <iterator>
#include <locale>
#include <memory>
#include <mrpt/containers/yaml.h>
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
#include <optional>
#include <ostream>
#include <sstream> // __str__
#include <streambuf>
#include <string>
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

void bind_mrpt_math_TObject3D(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::math::TObject3D file:mrpt/math/TObject3D.h line:30
		pybind11::class_<mrpt::math::TObject3D, std::shared_ptr<mrpt::math::TObject3D>> cl(M("mrpt::math"), "TObject3D", "A variant type for any lightweight 3D type: point, segment, line, plane,\n polygon. Use provided helper method, or directly access the variant `data`.\n\n \n TPoint3D,TSegment3D,TLine3D,TPlane,TPolygon3D");
		cl.def( pybind11::init( [](){ return new mrpt::math::TObject3D(); } ) );
		cl.def( pybind11::init( [](mrpt::math::TObject3D const &o){ return new mrpt::math::TObject3D(o); } ) );
		cl.def_readwrite("data", &mrpt::math::TObject3D::data);
		cl.def("getAs", (const struct mrpt::math::TPoint3D_<double> & (mrpt::math::TObject3D::*)() const) &mrpt::math::TObject3D::getAs<mrpt::math::TPoint3D_<double>>, "C++: mrpt::math::TObject3D::getAs() const --> const struct mrpt::math::TPoint3D_<double> &", pybind11::return_value_policy::automatic);
		cl.def("getAs", (const struct mrpt::math::TSegment3D & (mrpt::math::TObject3D::*)() const) &mrpt::math::TObject3D::getAs<mrpt::math::TSegment3D>, "C++: mrpt::math::TObject3D::getAs() const --> const struct mrpt::math::TSegment3D &", pybind11::return_value_policy::automatic);
		cl.def("getAs", (const struct mrpt::math::TLine3D & (mrpt::math::TObject3D::*)() const) &mrpt::math::TObject3D::getAs<mrpt::math::TLine3D>, "C++: mrpt::math::TObject3D::getAs() const --> const struct mrpt::math::TLine3D &", pybind11::return_value_policy::automatic);
		cl.def("getAs", (const class mrpt::math::TPolygon3D & (mrpt::math::TObject3D::*)() const) &mrpt::math::TObject3D::getAs<mrpt::math::TPolygon3D>, "C++: mrpt::math::TObject3D::getAs() const --> const class mrpt::math::TPolygon3D &", pybind11::return_value_policy::automatic);
		cl.def("getAs", (const struct mrpt::math::TPlane & (mrpt::math::TObject3D::*)() const) &mrpt::math::TObject3D::getAs<mrpt::math::TPlane>, "C++: mrpt::math::TObject3D::getAs() const --> const struct mrpt::math::TPlane &", pybind11::return_value_policy::automatic);
		cl.def("isPoint", (bool (mrpt::math::TObject3D::*)() const) &mrpt::math::TObject3D::isPoint, "Checks whether content is a point. \n\nC++: mrpt::math::TObject3D::isPoint() const --> bool");
		cl.def("isSegment", (bool (mrpt::math::TObject3D::*)() const) &mrpt::math::TObject3D::isSegment, "Checks whether content is a segment. \n\nC++: mrpt::math::TObject3D::isSegment() const --> bool");
		cl.def("isLine", (bool (mrpt::math::TObject3D::*)() const) &mrpt::math::TObject3D::isLine, "Checks whether content is a line.\n\nC++: mrpt::math::TObject3D::isLine() const --> bool");
		cl.def("isPolygon", (bool (mrpt::math::TObject3D::*)() const) &mrpt::math::TObject3D::isPolygon, "Checks whether content is a polygon.\n\nC++: mrpt::math::TObject3D::isPolygon() const --> bool");
		cl.def("isPlane", (bool (mrpt::math::TObject3D::*)() const) &mrpt::math::TObject3D::isPlane, "Checks whether content is a plane.\n\nC++: mrpt::math::TObject3D::isPlane() const --> bool");
		cl.def("empty", (bool (mrpt::math::TObject3D::*)() const) &mrpt::math::TObject3D::empty, "C++: mrpt::math::TObject3D::empty() const --> bool");
		cl.def("getPoint", (bool (mrpt::math::TObject3D::*)(struct mrpt::math::TPoint3D_<double> &) const) &mrpt::math::TObject3D::getPoint, "returns true if the objects is a point, and retrieves its value in\n `out`. Prefer getAs(). This method was left in mrpt 2.3.0 for backwards\n compatibility.\n\nC++: mrpt::math::TObject3D::getPoint(struct mrpt::math::TPoint3D_<double> &) const --> bool", pybind11::arg("out"));
		cl.def("getSegment", (bool (mrpt::math::TObject3D::*)(struct mrpt::math::TSegment3D &) const) &mrpt::math::TObject3D::getSegment, "returns true if the objects is a segment, and retrieves its value in\n `out`. Prefer getAs(). This method was left in mrpt 2.3.0 for backwards\n compatibility.\n\nC++: mrpt::math::TObject3D::getSegment(struct mrpt::math::TSegment3D &) const --> bool", pybind11::arg("out"));
		cl.def("getLine", (bool (mrpt::math::TObject3D::*)(struct mrpt::math::TLine3D &) const) &mrpt::math::TObject3D::getLine, "returns true if the objects is a line, and retrieves its value in\n `out`. Prefer getAs(). This method was left in mrpt 2.3.0 for backwards\n compatibility.\n\nC++: mrpt::math::TObject3D::getLine(struct mrpt::math::TLine3D &) const --> bool", pybind11::arg("out"));
		cl.def("getPolygon", (bool (mrpt::math::TObject3D::*)(class mrpt::math::TPolygon3D &) const) &mrpt::math::TObject3D::getPolygon, "returns true if the objects is a TPolygon3D, and retrieves its value in\n `out`. Prefer getAs(). This method was left in mrpt 2.3.0 for backwards\n compatibility.\n\nC++: mrpt::math::TObject3D::getPolygon(class mrpt::math::TPolygon3D &) const --> bool", pybind11::arg("out"));
		cl.def("getPlane", (bool (mrpt::math::TObject3D::*)(struct mrpt::math::TPlane &) const) &mrpt::math::TObject3D::getPlane, "returns true if the objects is a TPlane, and retrieves its value in\n `out`. Prefer getAs(). This method was left in mrpt 2.3.0 for backwards\n compatibility.\n\nC++: mrpt::math::TObject3D::getPlane(struct mrpt::math::TPlane &) const --> bool", pybind11::arg("out"));
		cl.def("asString", (std::string (mrpt::math::TObject3D::*)() const) &mrpt::math::TObject3D::asString, "Gets a string with the type and the parameters of the object. `empty` if\n not defined. \n\n New in MRPT 2.3.0 \n\nC++: mrpt::math::TObject3D::asString() const --> std::string");
		cl.def("generate2DObject", (struct mrpt::math::TObject2D (mrpt::math::TObject3D::*)() const) &mrpt::math::TObject3D::generate2DObject, "Cast into 2D space.\n \n\n std::logic_error if the 3D object loses its properties when\n projecting into 2D space (for example, it's a plane or a vertical line).\n\nC++: mrpt::math::TObject3D::generate2DObject() const --> struct mrpt::math::TObject2D");
		cl.def("assign", (struct mrpt::math::TObject3D & (mrpt::math::TObject3D::*)(const struct mrpt::math::TObject3D &)) &mrpt::math::TObject3D::operator=, "C++: mrpt::math::TObject3D::operator=(const struct mrpt::math::TObject3D &) --> struct mrpt::math::TObject3D &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		cl.def("__str__", [](mrpt::math::TObject3D const &o) -> std::string { std::ostringstream s; using namespace mrpt::math; s << o; return s.str(); } );
	}
}
