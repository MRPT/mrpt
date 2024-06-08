#include <ios>
#include <iterator>
#include <locale>
#include <memory>
#include <mrpt/topography/data_types.h>
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

void bind_mrpt_topography_data_types(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::topography::TCoords file:mrpt/topography/data_types.h line:24
		pybind11::class_<mrpt::topography::TCoords, std::shared_ptr<mrpt::topography::TCoords>> cl(M("mrpt::topography"), "TCoords", "A coordinate that is stored as a simple \"decimal\" angle in degrees, but can\n be retrieved/set in the form of DEGREES + arc-MINUTES + arc-SECONDS.");
		cl.def( pybind11::init<const int, const int, const double>(), pybind11::arg("_deg"), pybind11::arg("_min"), pybind11::arg("_sec") );

		cl.def( pybind11::init<const double>(), pybind11::arg("dec") );

		cl.def( pybind11::init( [](){ return new mrpt::topography::TCoords(); } ) );
		cl.def( pybind11::init( [](mrpt::topography::TCoords const &o){ return new mrpt::topography::TCoords(o); } ) );
		cl.def_readwrite("decimal_value", &mrpt::topography::TCoords::decimal_value);
		cl.def("setFromDecimal", (void (mrpt::topography::TCoords::*)(const double)) &mrpt::topography::TCoords::setFromDecimal, "Set from a decimal value (XX.YYYYY) in degrees. \n\nC++: mrpt::topography::TCoords::setFromDecimal(const double) --> void", pybind11::arg("dec"));
		cl.def("getDecimalValue", (double (mrpt::topography::TCoords::*)() const) &mrpt::topography::TCoords::getDecimalValue, "Get the decimal value (XX.YYYYY), in degrees - you can also use the\n automatic conversion between TCoords and a double.  \n\nC++: mrpt::topography::TCoords::getDecimalValue() const --> double");
		cl.def("getDegMinSec", (void (mrpt::topography::TCoords::*)(int &, int &, double &) const) &mrpt::topography::TCoords::getDegMinSec, "Return the Deg Min' Sec'' representation of this value. \n\nC++: mrpt::topography::TCoords::getDegMinSec(int &, int &, double &) const --> void", pybind11::arg("degrees"), pybind11::arg("minutes"), pybind11::arg("seconds"));
		cl.def("setDegMinSec", (void (mrpt::topography::TCoords::*)(const int, const int, const double)) &mrpt::topography::TCoords::setDegMinSec, "Set the coordinate from its Deg Min' Deg'' parts. \n\nC++: mrpt::topography::TCoords::setDegMinSec(const int, const int, const double) --> void", pybind11::arg("degrees"), pybind11::arg("minutes"), pybind11::arg("seconds"));
		cl.def("getAsString", (std::string (mrpt::topography::TCoords::*)() const) &mrpt::topography::TCoords::getAsString, "Return a std::string in the format \"DEGdeg MIN' SEC''\" \n\nC++: mrpt::topography::TCoords::getAsString() const --> std::string");

		cl.def("__str__", [](mrpt::topography::TCoords const &o) -> std::string { std::ostringstream s; using namespace mrpt::topography; s << o; return s.str(); } );
	}
	{ // mrpt::topography::TEllipsoid file:mrpt/topography/data_types.h line:78
		pybind11::class_<mrpt::topography::TEllipsoid, std::shared_ptr<mrpt::topography::TEllipsoid>> cl(M("mrpt::topography"), "TEllipsoid", "");
		cl.def( pybind11::init( [](){ return new mrpt::topography::TEllipsoid(); } ) );
		cl.def( pybind11::init<const double, const double, const std::string &>(), pybind11::arg("_sa"), pybind11::arg("_sb"), pybind11::arg("_name") );

		cl.def( pybind11::init( [](mrpt::topography::TEllipsoid const &o){ return new mrpt::topography::TEllipsoid(o); } ) );
		cl.def_readwrite("sa", &mrpt::topography::TEllipsoid::sa);
		cl.def_readwrite("sb", &mrpt::topography::TEllipsoid::sb);
		cl.def_readwrite("name", &mrpt::topography::TEllipsoid::name);
		cl.def_static("Ellipsoid_WGS84", (struct mrpt::topography::TEllipsoid (*)()) &mrpt::topography::TEllipsoid::Ellipsoid_WGS84, "C++: mrpt::topography::TEllipsoid::Ellipsoid_WGS84() --> struct mrpt::topography::TEllipsoid");
		cl.def_static("Ellipsoid_WGS72", (struct mrpt::topography::TEllipsoid (*)()) &mrpt::topography::TEllipsoid::Ellipsoid_WGS72, "C++: mrpt::topography::TEllipsoid::Ellipsoid_WGS72() --> struct mrpt::topography::TEllipsoid");
		cl.def_static("Ellipsoid_WGS66", (struct mrpt::topography::TEllipsoid (*)()) &mrpt::topography::TEllipsoid::Ellipsoid_WGS66, "C++: mrpt::topography::TEllipsoid::Ellipsoid_WGS66() --> struct mrpt::topography::TEllipsoid");
		cl.def_static("Ellipsoid_Walbeck_1817", (struct mrpt::topography::TEllipsoid (*)()) &mrpt::topography::TEllipsoid::Ellipsoid_Walbeck_1817, "C++: mrpt::topography::TEllipsoid::Ellipsoid_Walbeck_1817() --> struct mrpt::topography::TEllipsoid");
		cl.def_static("Ellipsoid_Sudamericano_1969", (struct mrpt::topography::TEllipsoid (*)()) &mrpt::topography::TEllipsoid::Ellipsoid_Sudamericano_1969, "C++: mrpt::topography::TEllipsoid::Ellipsoid_Sudamericano_1969() --> struct mrpt::topography::TEllipsoid");
		cl.def_static("Ellipsoid_Nuevo_Internacional_1967", (struct mrpt::topography::TEllipsoid (*)()) &mrpt::topography::TEllipsoid::Ellipsoid_Nuevo_Internacional_1967, "C++: mrpt::topography::TEllipsoid::Ellipsoid_Nuevo_Internacional_1967() --> struct mrpt::topography::TEllipsoid");
		cl.def_static("Ellipsoid_Mercury_Modificado_1968", (struct mrpt::topography::TEllipsoid (*)()) &mrpt::topography::TEllipsoid::Ellipsoid_Mercury_Modificado_1968, "C++: mrpt::topography::TEllipsoid::Ellipsoid_Mercury_Modificado_1968() --> struct mrpt::topography::TEllipsoid");
		cl.def_static("Ellipsoid_Mercury_1960", (struct mrpt::topography::TEllipsoid (*)()) &mrpt::topography::TEllipsoid::Ellipsoid_Mercury_1960, "C++: mrpt::topography::TEllipsoid::Ellipsoid_Mercury_1960() --> struct mrpt::topography::TEllipsoid");
		cl.def_static("Ellipsoid_Krasovsky_1940", (struct mrpt::topography::TEllipsoid (*)()) &mrpt::topography::TEllipsoid::Ellipsoid_Krasovsky_1940, "C++: mrpt::topography::TEllipsoid::Ellipsoid_Krasovsky_1940() --> struct mrpt::topography::TEllipsoid");
		cl.def_static("Ellipsoid_Internacional_1924", (struct mrpt::topography::TEllipsoid (*)()) &mrpt::topography::TEllipsoid::Ellipsoid_Internacional_1924, "C++: mrpt::topography::TEllipsoid::Ellipsoid_Internacional_1924() --> struct mrpt::topography::TEllipsoid");
		cl.def_static("Ellipsoid_Internacional_1909", (struct mrpt::topography::TEllipsoid (*)()) &mrpt::topography::TEllipsoid::Ellipsoid_Internacional_1909, "C++: mrpt::topography::TEllipsoid::Ellipsoid_Internacional_1909() --> struct mrpt::topography::TEllipsoid");
		cl.def_static("Ellipsoid_Hough_1960", (struct mrpt::topography::TEllipsoid (*)()) &mrpt::topography::TEllipsoid::Ellipsoid_Hough_1960, "C++: mrpt::topography::TEllipsoid::Ellipsoid_Hough_1960() --> struct mrpt::topography::TEllipsoid");
		cl.def_static("Ellipsoid_Helmert_1906", (struct mrpt::topography::TEllipsoid (*)()) &mrpt::topography::TEllipsoid::Ellipsoid_Helmert_1906, "C++: mrpt::topography::TEllipsoid::Ellipsoid_Helmert_1906() --> struct mrpt::topography::TEllipsoid");
		cl.def_static("Ellipsoid_Hayford_1909", (struct mrpt::topography::TEllipsoid (*)()) &mrpt::topography::TEllipsoid::Ellipsoid_Hayford_1909, "C++: mrpt::topography::TEllipsoid::Ellipsoid_Hayford_1909() --> struct mrpt::topography::TEllipsoid");
		cl.def_static("Ellipsoid_GRS80", (struct mrpt::topography::TEllipsoid (*)()) &mrpt::topography::TEllipsoid::Ellipsoid_GRS80, "C++: mrpt::topography::TEllipsoid::Ellipsoid_GRS80() --> struct mrpt::topography::TEllipsoid");
		cl.def_static("Ellipsoid_Fischer_1968", (struct mrpt::topography::TEllipsoid (*)()) &mrpt::topography::TEllipsoid::Ellipsoid_Fischer_1968, "C++: mrpt::topography::TEllipsoid::Ellipsoid_Fischer_1968() --> struct mrpt::topography::TEllipsoid");
		cl.def_static("Ellipsoid_Fischer_1960", (struct mrpt::topography::TEllipsoid (*)()) &mrpt::topography::TEllipsoid::Ellipsoid_Fischer_1960, "C++: mrpt::topography::TEllipsoid::Ellipsoid_Fischer_1960() --> struct mrpt::topography::TEllipsoid");
		cl.def_static("Ellipsoid_Clarke_1880", (struct mrpt::topography::TEllipsoid (*)()) &mrpt::topography::TEllipsoid::Ellipsoid_Clarke_1880, "C++: mrpt::topography::TEllipsoid::Ellipsoid_Clarke_1880() --> struct mrpt::topography::TEllipsoid");
		cl.def_static("Ellipsoid_Clarke_1866", (struct mrpt::topography::TEllipsoid (*)()) &mrpt::topography::TEllipsoid::Ellipsoid_Clarke_1866, "C++: mrpt::topography::TEllipsoid::Ellipsoid_Clarke_1866() --> struct mrpt::topography::TEllipsoid");
		cl.def_static("Ellipsoid_Bessel_1841", (struct mrpt::topography::TEllipsoid (*)()) &mrpt::topography::TEllipsoid::Ellipsoid_Bessel_1841, "C++: mrpt::topography::TEllipsoid::Ellipsoid_Bessel_1841() --> struct mrpt::topography::TEllipsoid");
		cl.def_static("Ellipsoid_Airy_Modificado_1965", (struct mrpt::topography::TEllipsoid (*)()) &mrpt::topography::TEllipsoid::Ellipsoid_Airy_Modificado_1965, "C++: mrpt::topography::TEllipsoid::Ellipsoid_Airy_Modificado_1965() --> struct mrpt::topography::TEllipsoid");
		cl.def_static("Ellipsoid_Airy_1830", (struct mrpt::topography::TEllipsoid (*)()) &mrpt::topography::TEllipsoid::Ellipsoid_Airy_1830, "C++: mrpt::topography::TEllipsoid::Ellipsoid_Airy_1830() --> struct mrpt::topography::TEllipsoid");
		cl.def("assign", (struct mrpt::topography::TEllipsoid & (mrpt::topography::TEllipsoid::*)(const struct mrpt::topography::TEllipsoid &)) &mrpt::topography::TEllipsoid::operator=, "C++: mrpt::topography::TEllipsoid::operator=(const struct mrpt::topography::TEllipsoid &) --> struct mrpt::topography::TEllipsoid &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::topography::TGeodeticCoords file:mrpt/topography/data_types.h line:188
		pybind11::class_<mrpt::topography::TGeodeticCoords, std::shared_ptr<mrpt::topography::TGeodeticCoords>> cl(M("mrpt::topography"), "TGeodeticCoords", "A set of geodetic coordinates: latitude, longitude and height, defined over\n a given geoid (typically, WGS84)  ");
		cl.def( pybind11::init( [](){ return new mrpt::topography::TGeodeticCoords(); } ) );
		cl.def( pybind11::init<const double, const double, const double>(), pybind11::arg("_lat"), pybind11::arg("_lon"), pybind11::arg("_height") );

		cl.def_readwrite("lat", &mrpt::topography::TGeodeticCoords::lat);
		cl.def_readwrite("lon", &mrpt::topography::TGeodeticCoords::lon);
		cl.def_readwrite("height", &mrpt::topography::TGeodeticCoords::height);
		cl.def("isClear", (bool (mrpt::topography::TGeodeticCoords::*)() const) &mrpt::topography::TGeodeticCoords::isClear, "C++: mrpt::topography::TGeodeticCoords::isClear() const --> bool");
	}
	{ // mrpt::topography::TDatum7Params file:mrpt/topography/data_types.h line:215
		pybind11::class_<mrpt::topography::TDatum7Params, std::shared_ptr<mrpt::topography::TDatum7Params>> cl(M("mrpt::topography"), "TDatum7Params", "Parameters for a topographic transfomation\n \n\n TDatum10Params, transform7params");
		cl.def( pybind11::init<const double, const double, const double, const double, const double, const double, const double>(), pybind11::arg("_dX"), pybind11::arg("_dY"), pybind11::arg("_dZ"), pybind11::arg("_Rx"), pybind11::arg("_Ry"), pybind11::arg("_Rz"), pybind11::arg("_dS") );

		cl.def_readwrite("dX", &mrpt::topography::TDatum7Params::dX);
		cl.def_readwrite("dY", &mrpt::topography::TDatum7Params::dY);
		cl.def_readwrite("dZ", &mrpt::topography::TDatum7Params::dZ);
		cl.def_readwrite("Rx", &mrpt::topography::TDatum7Params::Rx);
		cl.def_readwrite("Ry", &mrpt::topography::TDatum7Params::Ry);
		cl.def_readwrite("Rz", &mrpt::topography::TDatum7Params::Rz);
		cl.def_readwrite("dS", &mrpt::topography::TDatum7Params::dS);
	}
	{ // mrpt::topography::TDatum7Params_TOPCON file:mrpt/topography/data_types.h line:241
		pybind11::class_<mrpt::topography::TDatum7Params_TOPCON, std::shared_ptr<mrpt::topography::TDatum7Params_TOPCON>> cl(M("mrpt::topography"), "TDatum7Params_TOPCON", "");
		cl.def( pybind11::init<const double, const double, const double, const double, const double, const double, const double, const double, const double, const double, const double, const double, const double>(), pybind11::arg("_dX"), pybind11::arg("_dY"), pybind11::arg("_dZ"), pybind11::arg("_m11"), pybind11::arg("_m12"), pybind11::arg("_m13"), pybind11::arg("_m21"), pybind11::arg("_m22"), pybind11::arg("_m23"), pybind11::arg("_m31"), pybind11::arg("_m32"), pybind11::arg("_m33"), pybind11::arg("_dS") );

		cl.def_readwrite("dX", &mrpt::topography::TDatum7Params_TOPCON::dX);
		cl.def_readwrite("dY", &mrpt::topography::TDatum7Params_TOPCON::dY);
		cl.def_readwrite("dZ", &mrpt::topography::TDatum7Params_TOPCON::dZ);
		cl.def_readwrite("m11", &mrpt::topography::TDatum7Params_TOPCON::m11);
		cl.def_readwrite("m12", &mrpt::topography::TDatum7Params_TOPCON::m12);
		cl.def_readwrite("m13", &mrpt::topography::TDatum7Params_TOPCON::m13);
		cl.def_readwrite("m21", &mrpt::topography::TDatum7Params_TOPCON::m21);
		cl.def_readwrite("m22", &mrpt::topography::TDatum7Params_TOPCON::m22);
		cl.def_readwrite("m23", &mrpt::topography::TDatum7Params_TOPCON::m23);
		cl.def_readwrite("m31", &mrpt::topography::TDatum7Params_TOPCON::m31);
		cl.def_readwrite("m32", &mrpt::topography::TDatum7Params_TOPCON::m32);
		cl.def_readwrite("m33", &mrpt::topography::TDatum7Params_TOPCON::m33);
		cl.def_readwrite("dS", &mrpt::topography::TDatum7Params_TOPCON::dS);
	}
	{ // mrpt::topography::TDatum10Params file:mrpt/topography/data_types.h line:283
		pybind11::class_<mrpt::topography::TDatum10Params, std::shared_ptr<mrpt::topography::TDatum10Params>> cl(M("mrpt::topography"), "TDatum10Params", "Parameters for a topographic transfomation\n \n\n TDatum7Params, transform10params");
		cl.def( pybind11::init<const double, const double, const double, const double, const double, const double, const double, const double, const double, const double>(), pybind11::arg("_dX"), pybind11::arg("_dY"), pybind11::arg("_dZ"), pybind11::arg("_Xp"), pybind11::arg("_Yp"), pybind11::arg("_Zp"), pybind11::arg("_Rx"), pybind11::arg("_Ry"), pybind11::arg("_Rz"), pybind11::arg("_dS") );

		cl.def_readwrite("dX", &mrpt::topography::TDatum10Params::dX);
		cl.def_readwrite("dY", &mrpt::topography::TDatum10Params::dY);
		cl.def_readwrite("dZ", &mrpt::topography::TDatum10Params::dZ);
		cl.def_readwrite("Xp", &mrpt::topography::TDatum10Params::Xp);
		cl.def_readwrite("Yp", &mrpt::topography::TDatum10Params::Yp);
		cl.def_readwrite("Zp", &mrpt::topography::TDatum10Params::Zp);
		cl.def_readwrite("Rx", &mrpt::topography::TDatum10Params::Rx);
		cl.def_readwrite("Ry", &mrpt::topography::TDatum10Params::Ry);
		cl.def_readwrite("Rz", &mrpt::topography::TDatum10Params::Rz);
		cl.def_readwrite("dS", &mrpt::topography::TDatum10Params::dS);
	}
	{ // mrpt::topography::TDatumHelmert2D file:mrpt/topography/data_types.h line:317
		pybind11::class_<mrpt::topography::TDatumHelmert2D, std::shared_ptr<mrpt::topography::TDatumHelmert2D>> cl(M("mrpt::topography"), "TDatumHelmert2D", "Parameters for a topographic transfomation\n \n\n TDatumHelmert3D, transformHelmert2D");
		cl.def( pybind11::init<const double, const double, const double, const double, const double, const double>(), pybind11::arg("_dX"), pybind11::arg("_dY"), pybind11::arg("_alpha"), pybind11::arg("_dS"), pybind11::arg("_Xp"), pybind11::arg("_Yp") );

		cl.def_readwrite("dX", &mrpt::topography::TDatumHelmert2D::dX);
		cl.def_readwrite("dY", &mrpt::topography::TDatumHelmert2D::dY);
		cl.def_readwrite("alpha", &mrpt::topography::TDatumHelmert2D::alpha);
		cl.def_readwrite("dS", &mrpt::topography::TDatumHelmert2D::dS);
		cl.def_readwrite("Xp", &mrpt::topography::TDatumHelmert2D::Xp);
		cl.def_readwrite("Yp", &mrpt::topography::TDatumHelmert2D::Yp);
	}
	{ // mrpt::topography::TDatumHelmert2D_TOPCON file:mrpt/topography/data_types.h line:339
		pybind11::class_<mrpt::topography::TDatumHelmert2D_TOPCON, std::shared_ptr<mrpt::topography::TDatumHelmert2D_TOPCON>> cl(M("mrpt::topography"), "TDatumHelmert2D_TOPCON", "");
		cl.def( pybind11::init<const double, const double, const double, const double>(), pybind11::arg("_a"), pybind11::arg("_b"), pybind11::arg("_c"), pybind11::arg("_d") );

		cl.def_readwrite("a", &mrpt::topography::TDatumHelmert2D_TOPCON::a);
		cl.def_readwrite("b", &mrpt::topography::TDatumHelmert2D_TOPCON::b);
		cl.def_readwrite("c", &mrpt::topography::TDatumHelmert2D_TOPCON::c);
		cl.def_readwrite("d", &mrpt::topography::TDatumHelmert2D_TOPCON::d);
	}
	{ // mrpt::topography::TDatumHelmert3D file:mrpt/topography/data_types.h line:353
		pybind11::class_<mrpt::topography::TDatumHelmert3D, std::shared_ptr<mrpt::topography::TDatumHelmert3D>> cl(M("mrpt::topography"), "TDatumHelmert3D", "Parameters for a topographic transfomation\n \n\n TDatumHelmert2D, transformHelmert3D");
		cl.def( pybind11::init<const double, const double, const double, const double, const double, const double, const double>(), pybind11::arg("_dX"), pybind11::arg("_dY"), pybind11::arg("_dZ"), pybind11::arg("_Rx"), pybind11::arg("_Ry"), pybind11::arg("_Rz"), pybind11::arg("_dS") );

		cl.def_readwrite("dX", &mrpt::topography::TDatumHelmert3D::dX);
		cl.def_readwrite("dY", &mrpt::topography::TDatumHelmert3D::dY);
		cl.def_readwrite("dZ", &mrpt::topography::TDatumHelmert3D::dZ);
		cl.def_readwrite("Rx", &mrpt::topography::TDatumHelmert3D::Rx);
		cl.def_readwrite("Ry", &mrpt::topography::TDatumHelmert3D::Ry);
		cl.def_readwrite("Rz", &mrpt::topography::TDatumHelmert3D::Rz);
		cl.def_readwrite("dS", &mrpt::topography::TDatumHelmert3D::dS);
	}
}
