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
#include <mrpt/math/TPlane.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPolygon2D.h>
#include <mrpt/math/TPolygon3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TSegment3D.h>
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

void bind_mrpt_math_TPolygon2D(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::math::TPolygon2D file:mrpt/math/TPolygon2D.h line:25
		pybind11::class_<mrpt::math::TPolygon2D, std::shared_ptr<mrpt::math::TPolygon2D>> cl(M("mrpt::math"), "TPolygon2D", "2D polygon, inheriting from std::vector<TPoint2D>.\n \n\n TPolygon3D,TSegment2D,TLine2D,TPoint2D, CPolygon\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::math::TPolygon2D(); } ) );
		cl.def( pybind11::init<size_t>(), pybind11::arg("N") );

		cl.def( pybind11::init<const class mrpt::math::TPolygon3D &>(), pybind11::arg("p") );

		cl.def( pybind11::init( [](mrpt::math::TPolygon2D const &o){ return new mrpt::math::TPolygon2D(o); } ) );
		cl.def_static("FromYAML", (class mrpt::math::TPolygon2D (*)(const class mrpt::containers::yaml &)) &mrpt::math::TPolygon2D::FromYAML, "Builds a polygon from a YAML sequence (vertices) of sequences (`[x y]`\n coordinates).\n \n\n asYAML\n \n\n User must include `#include <mrpt/containers/yaml.h>` if using this\n       method, only a forward declaration is defined here to speed up\n       compilation \n\n (New in MRPT 2.4.1)\n\nC++: mrpt::math::TPolygon2D::FromYAML(const class mrpt::containers::yaml &) --> class mrpt::math::TPolygon2D", pybind11::arg("c"));
		cl.def("distance", (double (mrpt::math::TPolygon2D::*)(const struct mrpt::math::TPoint2D_<double> &) const) &mrpt::math::TPolygon2D::distance, "Distance to a point (always >=0) \n\nC++: mrpt::math::TPolygon2D::distance(const struct mrpt::math::TPoint2D_<double> &) const --> double", pybind11::arg("point"));
		cl.def("contains", (bool (mrpt::math::TPolygon2D::*)(const struct mrpt::math::TPoint2D_<double> &) const) &mrpt::math::TPolygon2D::contains, "Check whether a point is inside (or within geometryEpsilon of a polygon\n edge). This works for concave or convex polygons. \n\nC++: mrpt::math::TPolygon2D::contains(const struct mrpt::math::TPoint2D_<double> &) const --> bool", pybind11::arg("point"));
		cl.def("generate3DObject", (void (mrpt::math::TPolygon2D::*)(class mrpt::math::TPolygon3D &) const) &mrpt::math::TPolygon2D::generate3DObject, "Projects into 3D space, zeroing the z. \n\nC++: mrpt::math::TPolygon2D::generate3DObject(class mrpt::math::TPolygon3D &) const --> void", pybind11::arg("p"));
		cl.def("getCenter", (void (mrpt::math::TPolygon2D::*)(struct mrpt::math::TPoint2D_<double> &) const) &mrpt::math::TPolygon2D::getCenter, "Polygon's central point. \n\nC++: mrpt::math::TPolygon2D::getCenter(struct mrpt::math::TPoint2D_<double> &) const --> void", pybind11::arg("p"));
		cl.def("isConvex", (bool (mrpt::math::TPolygon2D::*)() const) &mrpt::math::TPolygon2D::isConvex, "Checks whether is convex. \n\nC++: mrpt::math::TPolygon2D::isConvex() const --> bool");
		cl.def("removeRepeatedVertices", (void (mrpt::math::TPolygon2D::*)()) &mrpt::math::TPolygon2D::removeRepeatedVertices, "Erase repeated vertices.  \n removeRedundantVertices \n\nC++: mrpt::math::TPolygon2D::removeRepeatedVertices() --> void");
		cl.def("removeRedundantVertices", (void (mrpt::math::TPolygon2D::*)()) &mrpt::math::TPolygon2D::removeRedundantVertices, "Erase every redundant vertex from the polygon, saving space. \n\n removeRepeatedVertices \n\nC++: mrpt::math::TPolygon2D::removeRedundantVertices() --> void");
		cl.def("getPlotData", (class std::tuple<class std::vector<double>, class std::vector<double> > (mrpt::math::TPolygon2D::*)() const) &mrpt::math::TPolygon2D::getPlotData, "C++: mrpt::math::TPolygon2D::getPlotData() const --> class std::tuple<class std::vector<double>, class std::vector<double> >");
		cl.def("asYAML", (class mrpt::containers::yaml (mrpt::math::TPolygon2D::*)() const) &mrpt::math::TPolygon2D::asYAML, "Returns a YAML representation of the polygon as a sequence (vertices) of\n sequences (`[x y]` coordinates).\n\n \n FromYAML\n\n \n User must include `#include <mrpt/containers/yaml.h>` if using this\n       method, only a forward declaration is defined here to speed up\n       compilation \n\n (New in MRPT 2.4.1)\n\nC++: mrpt::math::TPolygon2D::asYAML() const --> class mrpt::containers::yaml");
		cl.def("getBoundingBox", (void (mrpt::math::TPolygon2D::*)(struct mrpt::math::TPoint2D_<double> &, struct mrpt::math::TPoint2D_<double> &) const) &mrpt::math::TPolygon2D::getBoundingBox, "Get polygon bounding box. \n On empty polygon \n\nC++: mrpt::math::TPolygon2D::getBoundingBox(struct mrpt::math::TPoint2D_<double> &, struct mrpt::math::TPoint2D_<double> &) const --> void", pybind11::arg("min_coords"), pybind11::arg("max_coords"));
		cl.def_static("createRegularPolygon", (void (*)(size_t, double, class mrpt::math::TPolygon2D &)) &mrpt::math::TPolygon2D::createRegularPolygon, "Static method to create a regular polygon, given its size and radius.\n \n\n std::logic_error if radius is near zero or the number of edges is\n less than three.\n\nC++: mrpt::math::TPolygon2D::createRegularPolygon(size_t, double, class mrpt::math::TPolygon2D &) --> void", pybind11::arg("numEdges"), pybind11::arg("radius"), pybind11::arg("poly"));
		cl.def_static("createRegularPolygon", (void (*)(size_t, double, class mrpt::math::TPolygon2D &, const struct mrpt::math::TPose2D &)) &mrpt::math::TPolygon2D::createRegularPolygon, "Static method to create a regular polygon from its size and radius. The\n center will correspond to the given pose.\n \n\n std::logic_error if radius is near zero or the number of edges is\n less than three.\n\nC++: mrpt::math::TPolygon2D::createRegularPolygon(size_t, double, class mrpt::math::TPolygon2D &, const struct mrpt::math::TPose2D &) --> void", pybind11::arg("numEdges"), pybind11::arg("radius"), pybind11::arg("poly"), pybind11::arg("pose"));
		cl.def("assign", (class mrpt::math::TPolygon2D & (mrpt::math::TPolygon2D::*)(const class mrpt::math::TPolygon2D &)) &mrpt::math::TPolygon2D::operator=, "C++: mrpt::math::TPolygon2D::operator=(const class mrpt::math::TPolygon2D &) --> class mrpt::math::TPolygon2D &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		cl.def("__str__", [](mrpt::math::TPolygon2D const &o) -> std::string { std::ostringstream s; using namespace mrpt::math; s << o; return s.str(); } );
	}
}
