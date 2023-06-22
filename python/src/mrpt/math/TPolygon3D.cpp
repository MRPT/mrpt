#include <any>
#include <functional>
#include <ios>
#include <istream>
#include <iterator>
#include <locale>
#include <map>
#include <memory>
#include <mrpt/containers/CommentPosition.h>
#include <mrpt/containers/YamlEmitOptions.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TPlane.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPolygon2D.h>
#include <mrpt/math/TPolygon3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TSegment3D.h>
#include <optional>
#include <ostream>
#include <sstream> // __str__
#include <streambuf>
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

void bind_mrpt_math_TPolygon3D(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::math::TPolygon3D file:mrpt/math/TPolygon3D.h line:23
		pybind11::class_<mrpt::math::TPolygon3D, std::shared_ptr<mrpt::math::TPolygon3D>> cl(M("mrpt::math"), "TPolygon3D", "3D polygon, inheriting from std::vector<TPoint3D>\n \n\n TPolygon2D,TSegment3D,TLine3D,TPlane,TPoint3D\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::math::TPolygon3D(); } ) );
		cl.def( pybind11::init<size_t>(), pybind11::arg("N") );

		cl.def( pybind11::init<const class mrpt::math::TPolygon2D &>(), pybind11::arg("p") );

		cl.def( pybind11::init( [](mrpt::math::TPolygon3D const &o){ return new mrpt::math::TPolygon3D(o); } ) );
		cl.def_static("FromYAML", (class mrpt::math::TPolygon3D (*)(const class mrpt::containers::yaml &)) &mrpt::math::TPolygon3D::FromYAML, "Builds a polygon from a YAML sequence (vertices) of sequences (`[x y z]`\n coordinates).\n \n\n asYAML\n \n\n User must include `#include <mrpt/containers/yaml.h>` if using this\n       method, only a forward declaration is defined here to speed up\n       compilation \n\n (New in MRPT 2.4.1)\n\nC++: mrpt::math::TPolygon3D::FromYAML(const class mrpt::containers::yaml &) --> class mrpt::math::TPolygon3D", pybind11::arg("c"));
		cl.def("asYAML", (class mrpt::containers::yaml (mrpt::math::TPolygon3D::*)() const) &mrpt::math::TPolygon3D::asYAML, "Returns a YAML representation of the polygon as a sequence (vertices) of\n sequences (`[x y z]` coordinates).\n\n \n FromYAML\n\n \n User must include `#include <mrpt/containers/yaml.h>` if using this\n       method, only a forward declaration is defined here to speed up\n       compilation \n\n (New in MRPT 2.4.1)\n\nC++: mrpt::math::TPolygon3D::asYAML() const --> class mrpt::containers::yaml");
		cl.def("distance", (double (mrpt::math::TPolygon3D::*)(const struct mrpt::math::TPoint3D_<double> &) const) &mrpt::math::TPolygon3D::distance, "Absolute distance to point \n\nC++: mrpt::math::TPolygon3D::distance(const struct mrpt::math::TPoint3D_<double> &) const --> double", pybind11::arg("point"));
		cl.def("contains", (bool (mrpt::math::TPolygon3D::*)(const struct mrpt::math::TPoint3D_<double> &) const) &mrpt::math::TPolygon3D::contains, "Check whether a point is inside (or within geometryEpsilon of a polygon\n edge). This works for concave or convex polygons. \n\nC++: mrpt::math::TPolygon3D::contains(const struct mrpt::math::TPoint3D_<double> &) const --> bool", pybind11::arg("point"));
		cl.def("getPlane", (bool (mrpt::math::TPolygon3D::*)(struct mrpt::math::TPlane &) const) &mrpt::math::TPolygon3D::getPlane, "Gets a plane which contains the polygon. Returns false if the polygon is\n skew and cannot be fit inside a plane. \n\nC++: mrpt::math::TPolygon3D::getPlane(struct mrpt::math::TPlane &) const --> bool", pybind11::arg("p"));
		cl.def("getBestFittingPlane", (void (mrpt::math::TPolygon3D::*)(struct mrpt::math::TPlane &) const) &mrpt::math::TPolygon3D::getBestFittingPlane, "Gets the best fitting plane, disregarding whether the polygon actually\n fits inside or not. \n\n getBestFittingPlane \n\nC++: mrpt::math::TPolygon3D::getBestFittingPlane(struct mrpt::math::TPlane &) const --> void", pybind11::arg("p"));
		cl.def("generate2DObject", (void (mrpt::math::TPolygon3D::*)(class mrpt::math::TPolygon2D &) const) &mrpt::math::TPolygon3D::generate2DObject, "Projects into a 2D space, discarding the z. \n getPlane,isSkew \n\nC++: mrpt::math::TPolygon3D::generate2DObject(class mrpt::math::TPolygon2D &) const --> void", pybind11::arg("p"));
		cl.def("getCenter", (void (mrpt::math::TPolygon3D::*)(struct mrpt::math::TPoint3D_<double> &) const) &mrpt::math::TPolygon3D::getCenter, "Get polygon's central point. \n\nC++: mrpt::math::TPolygon3D::getCenter(struct mrpt::math::TPoint3D_<double> &) const --> void", pybind11::arg("p"));
		cl.def("isSkew", (bool (mrpt::math::TPolygon3D::*)() const) &mrpt::math::TPolygon3D::isSkew, "Check whether the polygon is skew. Returns true if there doesn't exist a\n plane in which the polygon can fit. \n\n getBestFittingPlane \n\nC++: mrpt::math::TPolygon3D::isSkew() const --> bool");
		cl.def("removeRepeatedVertices", (void (mrpt::math::TPolygon3D::*)()) &mrpt::math::TPolygon3D::removeRepeatedVertices, "Remove polygon's repeated vertices. \n\nC++: mrpt::math::TPolygon3D::removeRepeatedVertices() --> void");
		cl.def("removeRedundantVertices", (void (mrpt::math::TPolygon3D::*)()) &mrpt::math::TPolygon3D::removeRedundantVertices, "Erase every redundant vertex, thus saving space. \n\nC++: mrpt::math::TPolygon3D::removeRedundantVertices() --> void");
		cl.def_static("createRegularPolygon", (void (*)(size_t, double, class mrpt::math::TPolygon3D &)) &mrpt::math::TPolygon3D::createRegularPolygon, "C++: mrpt::math::TPolygon3D::createRegularPolygon(size_t, double, class mrpt::math::TPolygon3D &) --> void", pybind11::arg("numEdges"), pybind11::arg("radius"), pybind11::arg("poly"));
		cl.def_static("createRegularPolygon", (void (*)(size_t, double, class mrpt::math::TPolygon3D &, const struct mrpt::math::TPose3D &)) &mrpt::math::TPolygon3D::createRegularPolygon, "Static method to create a regular polygon, given its size and radius.\n The center will be located on the given pose.\n \n\n std::logic_error if number of edges is less than three, or radius\n is near zero.\n\nC++: mrpt::math::TPolygon3D::createRegularPolygon(size_t, double, class mrpt::math::TPolygon3D &, const struct mrpt::math::TPose3D &) --> void", pybind11::arg("numEdges"), pybind11::arg("radius"), pybind11::arg("poly"), pybind11::arg("pose"));
		cl.def("assign", (class mrpt::math::TPolygon3D & (mrpt::math::TPolygon3D::*)(const class mrpt::math::TPolygon3D &)) &mrpt::math::TPolygon3D::operator=, "C++: mrpt::math::TPolygon3D::operator=(const class mrpt::math::TPolygon3D &) --> class mrpt::math::TPolygon3D &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		cl.def("__str__", [](mrpt::math::TPolygon3D const &o) -> std::string { std::ostringstream s; using namespace mrpt::math; s << o; return s.str(); } );
	}
}
