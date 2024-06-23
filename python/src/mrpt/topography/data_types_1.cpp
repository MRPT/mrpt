#include <iterator>
#include <memory>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/topography/conversions.h>
#include <mrpt/topography/data_types.h>
#include <optional>
#include <sstream> // __str__
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

void bind_mrpt_topography_data_types_1(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::topography::TDatumHelmert3D_TOPCON file:mrpt/topography/data_types.h line:382
		pybind11::class_<mrpt::topography::TDatumHelmert3D_TOPCON, std::shared_ptr<mrpt::topography::TDatumHelmert3D_TOPCON>> cl(M("mrpt::topography"), "TDatumHelmert3D_TOPCON", "Parameters for a topographic transfomation\n \n\n TDatumHelmert2D, transformHelmert3D");
		cl.def( pybind11::init<const double, const double, const double, const double, const double, const double, const double>(), pybind11::arg("_a"), pybind11::arg("_b"), pybind11::arg("_c"), pybind11::arg("_d"), pybind11::arg("_e"), pybind11::arg("_f"), pybind11::arg("_g") );

		cl.def_readwrite("a", &mrpt::topography::TDatumHelmert3D_TOPCON::a);
		cl.def_readwrite("b", &mrpt::topography::TDatumHelmert3D_TOPCON::b);
		cl.def_readwrite("c", &mrpt::topography::TDatumHelmert3D_TOPCON::c);
		cl.def_readwrite("d", &mrpt::topography::TDatumHelmert3D_TOPCON::d);
		cl.def_readwrite("e", &mrpt::topography::TDatumHelmert3D_TOPCON::e);
		cl.def_readwrite("f", &mrpt::topography::TDatumHelmert3D_TOPCON::f);
		cl.def_readwrite("g", &mrpt::topography::TDatumHelmert3D_TOPCON::g);
	}
	{ // mrpt::topography::TDatum1DTransf file:mrpt/topography/data_types.h line:402
		pybind11::class_<mrpt::topography::TDatum1DTransf, std::shared_ptr<mrpt::topography::TDatum1DTransf>> cl(M("mrpt::topography"), "TDatum1DTransf", "Parameters for a topographic transfomation\n \n\n transform1D");
		cl.def( pybind11::init<const double, const double, const double, const double>(), pybind11::arg("_dX"), pybind11::arg("_dY"), pybind11::arg("_DZ"), pybind11::arg("_dS") );

		cl.def_readwrite("dX", &mrpt::topography::TDatum1DTransf::dX);
		cl.def_readwrite("dY", &mrpt::topography::TDatum1DTransf::dY);
		cl.def_readwrite("DZ", &mrpt::topography::TDatum1DTransf::DZ);
		cl.def_readwrite("dS", &mrpt::topography::TDatum1DTransf::dS);
	}
	{ // mrpt::topography::TDatumTransfInterpolation file:mrpt/topography/data_types.h line:419
		pybind11::class_<mrpt::topography::TDatumTransfInterpolation, std::shared_ptr<mrpt::topography::TDatumTransfInterpolation>> cl(M("mrpt::topography"), "TDatumTransfInterpolation", "Parameters for a topographic transfomation\n \n\n transform1D");
		cl.def( pybind11::init<const double, const double, const double, const double, const double>(), pybind11::arg("_dX"), pybind11::arg("_dY"), pybind11::arg("_dSx"), pybind11::arg("_dSy"), pybind11::arg("_beta") );

		cl.def_readwrite("dX", &mrpt::topography::TDatumTransfInterpolation::dX);
		cl.def_readwrite("dY", &mrpt::topography::TDatumTransfInterpolation::dY);
		cl.def_readwrite("dSx", &mrpt::topography::TDatumTransfInterpolation::dSx);
		cl.def_readwrite("dSy", &mrpt::topography::TDatumTransfInterpolation::dSy);
		cl.def_readwrite("beta", &mrpt::topography::TDatumTransfInterpolation::beta);
	}
	// mrpt::topography::geodeticToENU_WGS84(const struct mrpt::topography::TGeodeticCoords &, struct mrpt::math::TPoint3D_<double> &, const struct mrpt::topography::TGeodeticCoords &) file:mrpt/topography/conversions.h line:43
	M("mrpt::topography").def("geodeticToENU_WGS84", (void (*)(const struct mrpt::topography::TGeodeticCoords &, struct mrpt::math::TPoint3D_<double> &, const struct mrpt::topography::TGeodeticCoords &)) &mrpt::topography::geodeticToENU_WGS84, "Coordinates transformation from longitude/latitude/height to ENU\n (East-North-Up)  X/Y/Z coordinates\n  The WGS84 ellipsoid is used for the transformation. The coordinates are in\n 3D\n   relative to some user-provided point, with local X axis being east-ward, Y\n north-ward, Z up-ward.\n  For an explanation, refer to\n http://en.wikipedia.org/wiki/Reference_ellipsoid\n \n\n coordinatesTransformation_WGS84_geocentric, ENU_axes_from_WGS84,\n ENUToGeocentric\n \n\n The \"Up\" (Z) direction in ENU is the normal to the ellipsoid, which\n coincides with the direction of an increasing geodetic height.\n\nC++: mrpt::topography::geodeticToENU_WGS84(const struct mrpt::topography::TGeodeticCoords &, struct mrpt::math::TPoint3D_<double> &, const struct mrpt::topography::TGeodeticCoords &) --> void", pybind11::arg("in_coords"), pybind11::arg("out_ENU_point"), pybind11::arg("in_coords_origin"));

	// mrpt::topography::ENUToGeocentric(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::topography::TGeodeticCoords &, struct mrpt::math::TPoint3D_<double> &, const struct mrpt::topography::TEllipsoid &) file:mrpt/topography/conversions.h line:49
	M("mrpt::topography").def("ENUToGeocentric", (void (*)(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::topography::TGeodeticCoords &, struct mrpt::math::TPoint3D_<double> &, const struct mrpt::topography::TEllipsoid &)) &mrpt::topography::ENUToGeocentric, "ENU to geocentric coordinates. \n geodeticToENU_WGS84 \n\nC++: mrpt::topography::ENUToGeocentric(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::topography::TGeodeticCoords &, struct mrpt::math::TPoint3D_<double> &, const struct mrpt::topography::TEllipsoid &) --> void", pybind11::arg("in_ENU_point"), pybind11::arg("in_coords_origin"), pybind11::arg("out_coords"), pybind11::arg("ellip"));

	// mrpt::topography::geocentricToENU_WGS84(const struct mrpt::math::TPoint3D_<double> &, struct mrpt::math::TPoint3D_<double> &, const struct mrpt::topography::TGeodeticCoords &) file:mrpt/topography/conversions.h line:57
	M("mrpt::topography").def("geocentricToENU_WGS84", (void (*)(const struct mrpt::math::TPoint3D_<double> &, struct mrpt::math::TPoint3D_<double> &, const struct mrpt::topography::TGeodeticCoords &)) &mrpt::topography::geocentricToENU_WGS84, "ENU to EFEC (Geocentric) coordinates \n ENUToGeocentric,\n geodeticToENU_WGS84 \n\nC++: mrpt::topography::geocentricToENU_WGS84(const struct mrpt::math::TPoint3D_<double> &, struct mrpt::math::TPoint3D_<double> &, const struct mrpt::topography::TGeodeticCoords &) --> void", pybind11::arg("in_geocentric_point"), pybind11::arg("out_ENU_point"), pybind11::arg("in_coords_origin"));

	// mrpt::topography::geodeticToGeocentric_WGS84(const struct mrpt::topography::TGeodeticCoords &, struct mrpt::math::TPoint3D_<double> &) file:mrpt/topography/conversions.h line:77
	M("mrpt::topography").def("geodeticToGeocentric_WGS84", (void (*)(const struct mrpt::topography::TGeodeticCoords &, struct mrpt::math::TPoint3D_<double> &)) &mrpt::topography::geodeticToGeocentric_WGS84, "Coordinates transformation from longitude/latitude/height to geocentric\n X/Y/Z coordinates (with a WGS84 geoid).\n  The WGS84 ellipsoid is used for the transformation. The coordinates are in\n 3D\n   where the reference is the center of the Earth.\n  For an explanation, refer to\n http://en.wikipedia.org/wiki/Reference_ellipsoid\n \n\n geodeticToENU_WGS84\n\nC++: mrpt::topography::geodeticToGeocentric_WGS84(const struct mrpt::topography::TGeodeticCoords &, struct mrpt::math::TPoint3D_<double> &) --> void", pybind11::arg("in_coords"), pybind11::arg("out_point"));

	// mrpt::topography::geodeticToGeocentric(const struct mrpt::topography::TGeodeticCoords &, struct mrpt::math::TPoint3D_<double> &, const struct mrpt::topography::TEllipsoid &) file:mrpt/topography/conversions.h line:83
	M("mrpt::topography").def("geodeticToGeocentric", (void (*)(const struct mrpt::topography::TGeodeticCoords &, struct mrpt::math::TPoint3D_<double> &, const struct mrpt::topography::TEllipsoid &)) &mrpt::topography::geodeticToGeocentric, "Coordinates transformation from longitude/latitude/height to geocentric\n X/Y/Z coordinates (with an specified geoid).\n \n\n geocentricToGeodetic\n\nC++: mrpt::topography::geodeticToGeocentric(const struct mrpt::topography::TGeodeticCoords &, struct mrpt::math::TPoint3D_<double> &, const struct mrpt::topography::TEllipsoid &) --> void", pybind11::arg("in_coords"), pybind11::arg("out_point"), pybind11::arg("ellip"));

	// mrpt::topography::geocentricToGeodetic(const struct mrpt::math::TPoint3D_<double> &, struct mrpt::topography::TGeodeticCoords &, const struct mrpt::topography::TEllipsoid &) file:mrpt/topography/conversions.h line:90
	M("mrpt::topography").def("geocentricToGeodetic", [](const struct mrpt::math::TPoint3D_<double> & a0, struct mrpt::topography::TGeodeticCoords & a1) -> void { return mrpt::topography::geocentricToGeodetic(a0, a1); }, "", pybind11::arg("in_point"), pybind11::arg("out_coords"));
	M("mrpt::topography").def("geocentricToGeodetic", (void (*)(const struct mrpt::math::TPoint3D_<double> &, struct mrpt::topography::TGeodeticCoords &, const struct mrpt::topography::TEllipsoid &)) &mrpt::topography::geocentricToGeodetic, "Coordinates transformation from geocentric X/Y/Z coordinates to\n longitude/latitude/height.\n \n\n geodeticToGeocentric\n\nC++: mrpt::topography::geocentricToGeodetic(const struct mrpt::math::TPoint3D_<double> &, struct mrpt::topography::TGeodeticCoords &, const struct mrpt::topography::TEllipsoid &) --> void", pybind11::arg("in_point"), pybind11::arg("out_coords"), pybind11::arg("ellip"));

	// mrpt::topography::transform7params(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::topography::TDatum7Params &, struct mrpt::math::TPoint3D_<double> &) file:mrpt/topography/conversions.h line:100
	M("mrpt::topography").def("transform7params", (void (*)(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::topography::TDatum7Params &, struct mrpt::math::TPoint3D_<double> &)) &mrpt::topography::transform7params, "7-parameter Bursa-Wolf transformation:\n   [ X Y Z ]_WGS84 = [ dX dY dZ ] + ( 1 + dS ) [ 1 RZ -RY; -RZ 1 RX; RY -RX 1\n ] [ X Y Z ]_local\n \n\n transform10params\n\nC++: mrpt::topography::transform7params(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::topography::TDatum7Params &, struct mrpt::math::TPoint3D_<double> &) --> void", pybind11::arg("in_point"), pybind11::arg("in_datum"), pybind11::arg("out_point"));

	// mrpt::topography::transform7params_TOPCON(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::topography::TDatum7Params_TOPCON &, struct mrpt::math::TPoint3D_<double> &) file:mrpt/topography/conversions.h line:105
	M("mrpt::topography").def("transform7params_TOPCON", (void (*)(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::topography::TDatum7Params_TOPCON &, struct mrpt::math::TPoint3D_<double> &)) &mrpt::topography::transform7params_TOPCON, "C++: mrpt::topography::transform7params_TOPCON(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::topography::TDatum7Params_TOPCON &, struct mrpt::math::TPoint3D_<double> &) --> void", pybind11::arg("in_point"), pybind11::arg("in_datum"), pybind11::arg("out_point"));

	// mrpt::topography::transform10params(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::topography::TDatum10Params &, struct mrpt::math::TPoint3D_<double> &) file:mrpt/topography/conversions.h line:115
	M("mrpt::topography").def("transform10params", (void (*)(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::topography::TDatum10Params &, struct mrpt::math::TPoint3D_<double> &)) &mrpt::topography::transform10params, "10-parameter Molodensky-Badekas transformation:\n   [ X Y Z ]_WGS84 = [ dX dY dZ ] + ( 1 + dS ) [ 1 RZ -RY; -RZ 1 RX; RY -RX 1\n ] [ X-Xp Y-Yp Z-Zp ]_local  + [Xp Yp Zp]\n \n\n transform7params\n\nC++: mrpt::topography::transform10params(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::topography::TDatum10Params &, struct mrpt::math::TPoint3D_<double> &) --> void", pybind11::arg("in_point"), pybind11::arg("in_datum"), pybind11::arg("out_point"));

	// mrpt::topography::transformHelmert2D(const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::topography::TDatumHelmert2D &, struct mrpt::math::TPoint2D_<double> &) file:mrpt/topography/conversions.h line:125
	M("mrpt::topography").def("transformHelmert2D", (void (*)(const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::topography::TDatumHelmert2D &, struct mrpt::math::TPoint2D_<double> &)) &mrpt::topography::transformHelmert2D, "Helmert 2D transformation:\n   [ X Y ]_WGS84 = [ dX dY ] + ( 1 + dS ) [ cos(alpha) -sin(alpha);\n sin(alpha) cos(alpha) ] [ X-Xp Y-Yp Z-Zp ]_local + [Xp Yp Zp]\n \n\n transformHelmert3D\n\nC++: mrpt::topography::transformHelmert2D(const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::topography::TDatumHelmert2D &, struct mrpt::math::TPoint2D_<double> &) --> void", pybind11::arg("p"), pybind11::arg("d"), pybind11::arg("o"));

	// mrpt::topography::transformHelmert2D_TOPCON(const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::topography::TDatumHelmert2D_TOPCON &, struct mrpt::math::TPoint2D_<double> &) file:mrpt/topography/conversions.h line:128
	M("mrpt::topography").def("transformHelmert2D_TOPCON", (void (*)(const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::topography::TDatumHelmert2D_TOPCON &, struct mrpt::math::TPoint2D_<double> &)) &mrpt::topography::transformHelmert2D_TOPCON, "C++: mrpt::topography::transformHelmert2D_TOPCON(const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::topography::TDatumHelmert2D_TOPCON &, struct mrpt::math::TPoint2D_<double> &) --> void", pybind11::arg("p"), pybind11::arg("d"), pybind11::arg("o"));

	// mrpt::topography::transformHelmert3D(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::topography::TDatumHelmert3D &, struct mrpt::math::TPoint3D_<double> &) file:mrpt/topography/conversions.h line:136
	M("mrpt::topography").def("transformHelmert3D", (void (*)(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::topography::TDatumHelmert3D &, struct mrpt::math::TPoint3D_<double> &)) &mrpt::topography::transformHelmert3D, "Helmert3D transformation:\n   [ X Y Z ]_WGS84 = [ dX dY dZ ] + ( 1 + dS ) [ 1 -RZ RY; RZ 1 -RX; -RY RX 1\n ] [ X Y Z ]_local\n \n\n transformHelmert2D\n\nC++: mrpt::topography::transformHelmert3D(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::topography::TDatumHelmert3D &, struct mrpt::math::TPoint3D_<double> &) --> void", pybind11::arg("p"), pybind11::arg("d"), pybind11::arg("o"));

	// mrpt::topography::transformHelmert3D_TOPCON(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::topography::TDatumHelmert3D_TOPCON &, struct mrpt::math::TPoint3D_<double> &) file:mrpt/topography/conversions.h line:139
	M("mrpt::topography").def("transformHelmert3D_TOPCON", (void (*)(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::topography::TDatumHelmert3D_TOPCON &, struct mrpt::math::TPoint3D_<double> &)) &mrpt::topography::transformHelmert3D_TOPCON, "C++: mrpt::topography::transformHelmert3D_TOPCON(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::topography::TDatumHelmert3D_TOPCON &, struct mrpt::math::TPoint3D_<double> &) --> void", pybind11::arg("p"), pybind11::arg("d"), pybind11::arg("o"));

	// mrpt::topography::transform1D(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::topography::TDatum1DTransf &, struct mrpt::math::TPoint3D_<double> &) file:mrpt/topography/conversions.h line:145
	M("mrpt::topography").def("transform1D", (void (*)(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::topography::TDatum1DTransf &, struct mrpt::math::TPoint3D_<double> &)) &mrpt::topography::transform1D, "1D transformation:\n   [ Z ]_WGS84 = (dy * X - dx * Y + Z)*(1+e)+DZ\n\nC++: mrpt::topography::transform1D(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::topography::TDatum1DTransf &, struct mrpt::math::TPoint3D_<double> &) --> void", pybind11::arg("p"), pybind11::arg("d"), pybind11::arg("o"));

	// mrpt::topography::transfInterpolation(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::topography::TDatumTransfInterpolation &, struct mrpt::math::TPoint3D_<double> &) file:mrpt/topography/conversions.h line:150
	M("mrpt::topography").def("transfInterpolation", (void (*)(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::topography::TDatumTransfInterpolation &, struct mrpt::math::TPoint3D_<double> &)) &mrpt::topography::transfInterpolation, "Interpolation:\n   [ Z ]_WGS84 = (dy * X - dx * Y + Z)*(1+e)+DZ\n\nC++: mrpt::topography::transfInterpolation(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::topography::TDatumTransfInterpolation &, struct mrpt::math::TPoint3D_<double> &) --> void", pybind11::arg("p"), pybind11::arg("d"), pybind11::arg("o"));

	// mrpt::topography::UTMToGeodetic(double, double, int, char, double &, double &, const struct mrpt::topography::TEllipsoid &) file:mrpt/topography/conversions.h line:164
	M("mrpt::topography").def("UTMToGeodetic", [](double const & a0, double const & a1, int const & a2, char const & a3, double & a4, double & a5) -> void { return mrpt::topography::UTMToGeodetic(a0, a1, a2, a3, a4, a5); }, "", pybind11::arg("X"), pybind11::arg("Y"), pybind11::arg("zone"), pybind11::arg("hem"), pybind11::arg("out_lon"), pybind11::arg("out_lat"));
	M("mrpt::topography").def("UTMToGeodetic", (void (*)(double, double, int, char, double &, double &, const struct mrpt::topography::TEllipsoid &)) &mrpt::topography::UTMToGeodetic, "Returns the Geodetic coordinates of the UTM input point.\n \n\n East coordinate of the input point.\n \n\n North coordinate of the input point.\n \n\n time zone (Spanish: \"huso\").\n \n\n hemisphere ('N'/'n' for North or 'S'/s' for South ). An\n exception will be raised on any other value.\n \n\n the reference ellipsoid used for the transformation (default:\n WGS84)\n \n\n  Out latitude, in degrees.\n \n\n  Out longitude, in degrees.\n\nC++: mrpt::topography::UTMToGeodetic(double, double, int, char, double &, double &, const struct mrpt::topography::TEllipsoid &) --> void", pybind11::arg("X"), pybind11::arg("Y"), pybind11::arg("zone"), pybind11::arg("hem"), pybind11::arg("out_lon"), pybind11::arg("out_lat"), pybind11::arg("ellip"));

}
