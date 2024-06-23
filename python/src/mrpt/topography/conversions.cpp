#include <iterator>
#include <memory>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/obs/CAction.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/topography/conversions.h>
#include <mrpt/topography/data_types.h>
#include <mrpt/topography/path_from_rtk_gps.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ostream>
#include <sstream> // __str__
#include <string>
#include <tuple>
#include <variant>
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

void bind_mrpt_topography_conversions(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::topography::UTMToGeodetic(const struct mrpt::math::TPoint3D_<double> &, int, char, struct mrpt::topography::TGeodeticCoords &, const struct mrpt::topography::TEllipsoid &) file:mrpt/topography/conversions.h line:182
	M("mrpt::topography").def("UTMToGeodetic", [](const struct mrpt::math::TPoint3D_<double> & a0, int const & a1, char const & a2, struct mrpt::topography::TGeodeticCoords & a3) -> void { return mrpt::topography::UTMToGeodetic(a0, a1, a2, a3); }, "", pybind11::arg("UTMCoords"), pybind11::arg("zone"), pybind11::arg("hem"), pybind11::arg("GeodeticCoords"));
	M("mrpt::topography").def("UTMToGeodetic", (void (*)(const struct mrpt::math::TPoint3D_<double> &, int, char, struct mrpt::topography::TGeodeticCoords &, const struct mrpt::topography::TEllipsoid &)) &mrpt::topography::UTMToGeodetic, "Returns the Geodetic coordinates of the UTM input point.\n \n\n UTM input coordinates.\n \n\n time zone (Spanish: \"huso\").\n \n\n hemisphere ('N'/'n' for North or 'S'/s' for South ). An\n exception will be raised on any other value.\n \n\n Out geodetic coordinates.\n \n\n the reference ellipsoid used for the transformation (default:\n WGS84)\n\nC++: mrpt::topography::UTMToGeodetic(const struct mrpt::math::TPoint3D_<double> &, int, char, struct mrpt::topography::TGeodeticCoords &, const struct mrpt::topography::TEllipsoid &) --> void", pybind11::arg("UTMCoords"), pybind11::arg("zone"), pybind11::arg("hem"), pybind11::arg("GeodeticCoords"), pybind11::arg("ellip"));

	// mrpt::topography::GeodeticToUTM(double, double, double &, double &, int &, char &, const struct mrpt::topography::TEllipsoid &) file:mrpt/topography/conversions.h line:211
	M("mrpt::topography").def("GeodeticToUTM", [](double const & a0, double const & a1, double & a2, double & a3, int & a4, char & a5) -> void { return mrpt::topography::GeodeticToUTM(a0, a1, a2, a3, a4, a5); }, "", pybind11::arg("in_latitude_degrees"), pybind11::arg("in_longitude_degrees"), pybind11::arg("out_UTM_x"), pybind11::arg("out_UTM_y"), pybind11::arg("out_UTM_zone"), pybind11::arg("out_UTM_latitude_band"));
	M("mrpt::topography").def("GeodeticToUTM", (void (*)(double, double, double &, double &, int &, char &, const struct mrpt::topography::TEllipsoid &)) &mrpt::topography::GeodeticToUTM, "Convert latitude and longitude coordinates into UTM coordinates, computing\n the corresponding UTM zone and latitude band.\n   This method is based on public code by Gabriel Ruiz Martinez and Rafael\n Palacios.\n   Example:\n   \n\n\n\n\n\n\n\n\n\n   \n http://www.mathworks.com/matlabcentral/fileexchange/10915\n\nC++: mrpt::topography::GeodeticToUTM(double, double, double &, double &, int &, char &, const struct mrpt::topography::TEllipsoid &) --> void", pybind11::arg("in_latitude_degrees"), pybind11::arg("in_longitude_degrees"), pybind11::arg("out_UTM_x"), pybind11::arg("out_UTM_y"), pybind11::arg("out_UTM_zone"), pybind11::arg("out_UTM_latitude_band"), pybind11::arg("ellip"));

	// mrpt::topography::geodeticToUTM(const struct mrpt::topography::TGeodeticCoords &, struct mrpt::math::TPoint3D_<double> &, int &, char &, const struct mrpt::topography::TEllipsoid &) file:mrpt/topography/conversions.h line:220
	M("mrpt::topography").def("geodeticToUTM", [](const struct mrpt::topography::TGeodeticCoords & a0, struct mrpt::math::TPoint3D_<double> & a1, int & a2, char & a3) -> void { return mrpt::topography::geodeticToUTM(a0, a1, a2, a3); }, "", pybind11::arg("GeodeticCoords"), pybind11::arg("UTMCoords"), pybind11::arg("UTMZone"), pybind11::arg("UTMLatitudeBand"));
	M("mrpt::topography").def("geodeticToUTM", (void (*)(const struct mrpt::topography::TGeodeticCoords &, struct mrpt::math::TPoint3D_<double> &, int &, char &, const struct mrpt::topography::TEllipsoid &)) &mrpt::topography::geodeticToUTM, "C++: mrpt::topography::geodeticToUTM(const struct mrpt::topography::TGeodeticCoords &, struct mrpt::math::TPoint3D_<double> &, int &, char &, const struct mrpt::topography::TEllipsoid &) --> void", pybind11::arg("GeodeticCoords"), pybind11::arg("UTMCoords"), pybind11::arg("UTMZone"), pybind11::arg("UTMLatitudeBand"), pybind11::arg("ellip"));

	// mrpt::topography::GeodeticToUTM(const struct mrpt::topography::TGeodeticCoords &, struct mrpt::math::TPoint3D_<double> &, int &, char &, const struct mrpt::topography::TEllipsoid &) file:mrpt/topography/conversions.h line:243
	M("mrpt::topography").def("GeodeticToUTM", [](const struct mrpt::topography::TGeodeticCoords & a0, struct mrpt::math::TPoint3D_<double> & a1, int & a2, char & a3) -> void { return mrpt::topography::GeodeticToUTM(a0, a1, a2, a3); }, "", pybind11::arg("GeodeticCoords"), pybind11::arg("UTMCoords"), pybind11::arg("UTMZone"), pybind11::arg("UTMLatitudeBand"));
	M("mrpt::topography").def("GeodeticToUTM", (void (*)(const struct mrpt::topography::TGeodeticCoords &, struct mrpt::math::TPoint3D_<double> &, int &, char &, const struct mrpt::topography::TEllipsoid &)) &mrpt::topography::GeodeticToUTM, "Convert latitude and longitude coordinates into UTM coordinates, computing\n the corresponding UTM zone and latitude band.\n   This method is based on public code by Gabriel Ruiz Martinez and Rafael\n Palacios.\n   Example:\n   \n\n\n\n\n\n\n\n\n\n   \n http://www.mathworks.com/matlabcentral/fileexchange/10915\n\nC++: mrpt::topography::GeodeticToUTM(const struct mrpt::topography::TGeodeticCoords &, struct mrpt::math::TPoint3D_<double> &, int &, char &, const struct mrpt::topography::TEllipsoid &) --> void", pybind11::arg("GeodeticCoords"), pybind11::arg("UTMCoords"), pybind11::arg("UTMZone"), pybind11::arg("UTMLatitudeBand"), pybind11::arg("ellip"));

	// mrpt::topography::ENU_axes_from_WGS84(double, double, double, struct mrpt::math::TPose3D &, bool) file:mrpt/topography/conversions.h line:271
	M("mrpt::topography").def("ENU_axes_from_WGS84", [](double const & a0, double const & a1, double const & a2, struct mrpt::math::TPose3D & a3) -> void { return mrpt::topography::ENU_axes_from_WGS84(a0, a1, a2, a3); }, "", pybind11::arg("in_longitude_reference_degrees"), pybind11::arg("in_latitude_reference_degrees"), pybind11::arg("in_height_reference_meters"), pybind11::arg("out_ENU"));
	M("mrpt::topography").def("ENU_axes_from_WGS84", (void (*)(double, double, double, struct mrpt::math::TPose3D &, bool)) &mrpt::topography::ENU_axes_from_WGS84, "Returns the East-North-Up (ENU) coordinate system associated to the given\n point.\n This is the reference employed in geodeticToENU_WGS84\n \n\n If set to true, the (x,y,z) fields will be left zeroed.\n \n\n The \"Up\" (Z) direction in ENU is the normal to the ellipsoid, which\n coincides with the direction of an increasing geodetic height.\n \n\n geodeticToENU_WGS84\n\nC++: mrpt::topography::ENU_axes_from_WGS84(double, double, double, struct mrpt::math::TPose3D &, bool) --> void", pybind11::arg("in_longitude_reference_degrees"), pybind11::arg("in_latitude_reference_degrees"), pybind11::arg("in_height_reference_meters"), pybind11::arg("out_ENU"), pybind11::arg("only_angles"));

	// mrpt::topography::ENU_axes_from_WGS84(const struct mrpt::topography::TGeodeticCoords &, struct mrpt::math::TPose3D &, bool) file:mrpt/topography/conversions.h line:279
	M("mrpt::topography").def("ENU_axes_from_WGS84", [](const struct mrpt::topography::TGeodeticCoords & a0, struct mrpt::math::TPose3D & a1) -> void { return mrpt::topography::ENU_axes_from_WGS84(a0, a1); }, "", pybind11::arg("in_coords"), pybind11::arg("out_ENU"));
	M("mrpt::topography").def("ENU_axes_from_WGS84", (void (*)(const struct mrpt::topography::TGeodeticCoords &, struct mrpt::math::TPose3D &, bool)) &mrpt::topography::ENU_axes_from_WGS84, "C++: mrpt::topography::ENU_axes_from_WGS84(const struct mrpt::topography::TGeodeticCoords &, struct mrpt::math::TPose3D &, bool) --> void", pybind11::arg("in_coords"), pybind11::arg("out_ENU"), pybind11::arg("only_angles"));

	{ // mrpt::topography::TPathFromRTKInfo file:mrpt/topography/path_from_rtk_gps.h line:25
		pybind11::class_<mrpt::topography::TPathFromRTKInfo, std::shared_ptr<mrpt::topography::TPathFromRTKInfo>> cl(M("mrpt::topography"), "TPathFromRTKInfo", "Used to return optional information from mrpt::topography::path_from_rtk_gps");
		cl.def( pybind11::init( [](){ return new mrpt::topography::TPathFromRTKInfo(); } ) );
		cl.def( pybind11::init( [](mrpt::topography::TPathFromRTKInfo const &o){ return new mrpt::topography::TPathFromRTKInfo(o); } ) );
		cl.def_readwrite("best_gps_path", &mrpt::topography::TPathFromRTKInfo::best_gps_path);
		cl.def_readwrite("mahalabis_quality_measure", &mrpt::topography::TPathFromRTKInfo::mahalabis_quality_measure);
		cl.def_readwrite("vehicle_uncertainty", &mrpt::topography::TPathFromRTKInfo::vehicle_uncertainty);
		cl.def_readwrite("W_star", &mrpt::topography::TPathFromRTKInfo::W_star);
		cl.def("assign", (struct mrpt::topography::TPathFromRTKInfo & (mrpt::topography::TPathFromRTKInfo::*)(const struct mrpt::topography::TPathFromRTKInfo &)) &mrpt::topography::TPathFromRTKInfo::operator=, "C++: mrpt::topography::TPathFromRTKInfo::operator=(const struct mrpt::topography::TPathFromRTKInfo &) --> struct mrpt::topography::TPathFromRTKInfo &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	// mrpt::topography::path_from_rtk_gps(class mrpt::poses::CPose3DInterpolator &, const class mrpt::obs::CRawlog &, size_t, size_t, bool, bool, int, struct mrpt::topography::TPathFromRTKInfo *) file:mrpt/topography/path_from_rtk_gps.h line:57
	M("mrpt::topography").def("path_from_rtk_gps", [](class mrpt::poses::CPose3DInterpolator & a0, const class mrpt::obs::CRawlog & a1, size_t const & a2, size_t const & a3) -> void { return mrpt::topography::path_from_rtk_gps(a0, a1, a2, a3); }, "", pybind11::arg("robot_path"), pybind11::arg("rawlog"), pybind11::arg("rawlog_first"), pybind11::arg("rawlog_last"));
	M("mrpt::topography").def("path_from_rtk_gps", [](class mrpt::poses::CPose3DInterpolator & a0, const class mrpt::obs::CRawlog & a1, size_t const & a2, size_t const & a3, bool const & a4) -> void { return mrpt::topography::path_from_rtk_gps(a0, a1, a2, a3, a4); }, "", pybind11::arg("robot_path"), pybind11::arg("rawlog"), pybind11::arg("rawlog_first"), pybind11::arg("rawlog_last"), pybind11::arg("isGUI"));
	M("mrpt::topography").def("path_from_rtk_gps", [](class mrpt::poses::CPose3DInterpolator & a0, const class mrpt::obs::CRawlog & a1, size_t const & a2, size_t const & a3, bool const & a4, bool const & a5) -> void { return mrpt::topography::path_from_rtk_gps(a0, a1, a2, a3, a4, a5); }, "", pybind11::arg("robot_path"), pybind11::arg("rawlog"), pybind11::arg("rawlog_first"), pybind11::arg("rawlog_last"), pybind11::arg("isGUI"), pybind11::arg("disableGPSInterp"));
	M("mrpt::topography").def("path_from_rtk_gps", [](class mrpt::poses::CPose3DInterpolator & a0, const class mrpt::obs::CRawlog & a1, size_t const & a2, size_t const & a3, bool const & a4, bool const & a5, int const & a6) -> void { return mrpt::topography::path_from_rtk_gps(a0, a1, a2, a3, a4, a5, a6); }, "", pybind11::arg("robot_path"), pybind11::arg("rawlog"), pybind11::arg("rawlog_first"), pybind11::arg("rawlog_last"), pybind11::arg("isGUI"), pybind11::arg("disableGPSInterp"), pybind11::arg("path_smooth_filter_size"));
	M("mrpt::topography").def("path_from_rtk_gps", (void (*)(class mrpt::poses::CPose3DInterpolator &, const class mrpt::obs::CRawlog &, size_t, size_t, bool, bool, int, struct mrpt::topography::TPathFromRTKInfo *)) &mrpt::topography::path_from_rtk_gps, "Reconstruct the path of a vehicle equipped with 3 RTK GPSs.\n  \n\n [OUT] The reconstructed vehicle path\n  \n\n [IN] The dataset. It must contain mrpt::obs::CObservationGPS\n observations with GGA datums.\n  \n\n [IN] The index of the first entry to process (first=0)\n  \n\n [IN] The index of the last entry to process\n  \n\n [IN] If set to true, some progress dialogs will be shown\n during the computation (requires MRPT built with support for wxWidgets).\n  \n\n [IN] Whether to interpolate missing GPS readings\n between very close datums.\n  \n\n [IN] Size of the window in the pitch & roll\n noise filtering.\n  \n\n [OUT] Optional output: additional information from the\n optimization\n\n  For more details on the method, refer to the paper: (...)\n \n\n mrpt::topography\n\nC++: mrpt::topography::path_from_rtk_gps(class mrpt::poses::CPose3DInterpolator &, const class mrpt::obs::CRawlog &, size_t, size_t, bool, bool, int, struct mrpt::topography::TPathFromRTKInfo *) --> void", pybind11::arg("robot_path"), pybind11::arg("rawlog"), pybind11::arg("rawlog_first"), pybind11::arg("rawlog_last"), pybind11::arg("isGUI"), pybind11::arg("disableGPSInterp"), pybind11::arg("path_smooth_filter_size"), pybind11::arg("outInfo"));

}
