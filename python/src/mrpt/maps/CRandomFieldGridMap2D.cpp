#include <ios>
#include <iterator>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/img/CCanvas.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/img/TColor.h>
#include <mrpt/img/TPixelCoord.h>
#include <mrpt/io/CStream.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CRandomFieldGridMap2D.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/metric_map_types.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/MatrixVectorBase.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPointPDF.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPose3DQuatPDF.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/typemeta/static_string.h>
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

// mrpt::maps::CRandomFieldGridMap2D::ConnectivityDescriptor file:mrpt/maps/CRandomFieldGridMap2D.h line:355
struct PyCallBack_mrpt_maps_CRandomFieldGridMap2D_ConnectivityDescriptor : public mrpt::maps::CRandomFieldGridMap2D::ConnectivityDescriptor {
	using mrpt::maps::CRandomFieldGridMap2D::ConnectivityDescriptor::ConnectivityDescriptor;

	bool getEdgeInformation(const class mrpt::maps::CRandomFieldGridMap2D * a0, size_t a1, size_t a2, size_t a3, size_t a4, double & a5) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CRandomFieldGridMap2D::ConnectivityDescriptor *>(this), "getEdgeInformation");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4, a5);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"ConnectivityDescriptor::getEdgeInformation\"");
	}
};

void bind_mrpt_maps_CRandomFieldGridMap2D(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::maps::TRandomFieldCell file:mrpt/maps/CRandomFieldGridMap2D.h line:38
		pybind11::class_<mrpt::maps::TRandomFieldCell, std::shared_ptr<mrpt::maps::TRandomFieldCell>> cl(M("mrpt::maps"), "TRandomFieldCell", "The contents of each cell in a CRandomFieldGridMap2D map.\n \n");
		cl.def( pybind11::init( [](){ return new mrpt::maps::TRandomFieldCell(); } ), "doc" );
		cl.def( pybind11::init( [](double const & a0){ return new mrpt::maps::TRandomFieldCell(a0); } ), "doc" , pybind11::arg("kfmean_dm_mean"));
		cl.def( pybind11::init<double, double>(), pybind11::arg("kfmean_dm_mean"), pybind11::arg("kfstd_dmmeanw") );

		cl.def_readwrite("param1_", &mrpt::maps::TRandomFieldCell::param1_);
		cl.def_readwrite("param2_", &mrpt::maps::TRandomFieldCell::param2_);
		cl.def_readwrite("dmv_var_mean", &mrpt::maps::TRandomFieldCell::dmv_var_mean);
		cl.def_readwrite("last_updated", &mrpt::maps::TRandomFieldCell::last_updated);
		cl.def_readwrite("updated_std", &mrpt::maps::TRandomFieldCell::updated_std);
		cl.def("kf_mean", (double & (mrpt::maps::TRandomFieldCell::*)()) &mrpt::maps::TRandomFieldCell::kf_mean, "[KF-methods only] The mean value of this cell \n\nC++: mrpt::maps::TRandomFieldCell::kf_mean() --> double &", pybind11::return_value_policy::automatic);
		cl.def("dm_mean", (double & (mrpt::maps::TRandomFieldCell::*)()) &mrpt::maps::TRandomFieldCell::dm_mean, "[Kernel-methods only] The cumulative weighted readings of this cell\n\nC++: mrpt::maps::TRandomFieldCell::dm_mean() --> double &", pybind11::return_value_policy::automatic);
		cl.def("gmrf_mean", (double & (mrpt::maps::TRandomFieldCell::*)()) &mrpt::maps::TRandomFieldCell::gmrf_mean, "[GMRF only] The mean value of this cell \n\nC++: mrpt::maps::TRandomFieldCell::gmrf_mean() --> double &", pybind11::return_value_policy::automatic);
		cl.def("kf_std", (double & (mrpt::maps::TRandomFieldCell::*)()) &mrpt::maps::TRandomFieldCell::kf_std, "[KF-methods only] The standard deviation value of this cell \n\nC++: mrpt::maps::TRandomFieldCell::kf_std() --> double &", pybind11::return_value_policy::automatic);
		cl.def("dm_mean_w", (double & (mrpt::maps::TRandomFieldCell::*)()) &mrpt::maps::TRandomFieldCell::dm_mean_w, "[Kernel-methods only] The cumulative weights (concentration = alpha\n * dm_mean / dm_mean_w + (1-alpha)*r0 ) \n\nC++: mrpt::maps::TRandomFieldCell::dm_mean_w() --> double &", pybind11::return_value_policy::automatic);
		cl.def("gmrf_std", (double & (mrpt::maps::TRandomFieldCell::*)()) &mrpt::maps::TRandomFieldCell::gmrf_std, "C++: mrpt::maps::TRandomFieldCell::gmrf_std() --> double &", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::maps::CRandomFieldGridMap2D file:mrpt/maps/CRandomFieldGridMap2D.h line:154
		pybind11::class_<mrpt::maps::CRandomFieldGridMap2D, std::shared_ptr<mrpt::maps::CRandomFieldGridMap2D>, mrpt::maps::CMetricMap, mrpt::containers::CDynamicGrid<mrpt::maps::TRandomFieldCell>> cl(M("mrpt::maps"), "CRandomFieldGridMap2D", "CRandomFieldGridMap2D represents a 2D grid map where each cell is associated\none real-valued property which is estimated by this map, either\n   as a simple value or as a probility distribution (for each cell).\n\n  There are a number of methods available to build the MRF grid-map,\ndepending on the value of\n    `TMapRepresentation maptype` passed in the constructor.\n\n  The following papers describe the mapping alternatives implemented here:\n		- `mrKernelDM`: A Gaussian kernel-based method. See:\n			- \"Building gas concentration gridmaps with a mobile robot\",\nLilienthal,\nA. and Duckett, T., Robotics and Autonomous Systems, v.48, 2004.\n		- `mrKernelDMV`: A kernel-based method. See:\n			- \"A Statistical Approach to Gas Distribution Modelling with Mobile\nRobots--The Kernel DM+ V Algorithm\", Lilienthal, A.J. and Reggente, M. and\nTrincavelli, M. and Blanco, J.L. and Gonzalez, J., IROS 2009.\n		- `mrKalmanFilter`: A \"brute-force\" approach to estimate the entire map\nwith a dense (linear) Kalman filter. Will be very slow for mid or large maps.\nIt's provided just for comparison purposes, not useful in practice.\n		- `mrKalmanApproximate`: A compressed/sparse Kalman filter approach.\nSee:\n			- \"A Kalman Filter Based Approach to Probabilistic Gas Distribution\nMapping\", JL Blanco, JG Monroy, J Gonzalez-Jimenez, A Lilienthal, 28th\nSymposium On Applied Computing (SAC), 2013.\n		- `mrGMRF_SD`: A Gaussian Markov Random Field (GMRF) estimator, with\nthese\nconstraints:\n			- `mrGMRF_SD`: Each cell only connected to its 4 immediate neighbors\n(Up,\ndown, left, right).\n			- (Removed in MRPT 1.5.0: `mrGMRF_G`: Each cell connected to a\nsquare\narea\nof neighbors cells)\n			- See papers:\n				- \"Time-variant gas distribution mapping with obstacle\ninformation\",\nMonroy, J. G., Blanco, J. L., & Gonzalez-Jimenez, J. Autonomous Robots,\n40(1), 1-16, 2016.\n\n  Note that this class is virtual, since derived classes still have to\nimplement:\n		- mrpt::maps::CMetricMap::internal_computeObservationLikelihood()\n		- mrpt::maps::CMetricMap::internal_insertObservation()\n		- Serialization methods: writeToStream() and readFromStream()\n\n [GMRF only] A custom connectivity pattern between cells can be defined by\ncalling setCellsConnectivity().\n\n \n mrpt::maps::CGasConcentrationGridMap2D,\nmrpt::maps::CWirelessPowerGridMap2D, mrpt::maps::CMetricMap,\nmrpt::containers::CDynamicGrid, The application icp-slam,\nmrpt::maps::CMultiMetricMap\n \n\n\n ");

		pybind11::enum_<mrpt::maps::CRandomFieldGridMap2D::TMapRepresentation>(cl, "TMapRepresentation", pybind11::arithmetic(), "The type of map representation to be used, see CRandomFieldGridMap2D for\n a discussion.")
			.value("mrKernelDM", mrpt::maps::CRandomFieldGridMap2D::mrKernelDM)
			.value("mrAchim", mrpt::maps::CRandomFieldGridMap2D::mrAchim)
			.value("mrKalmanFilter", mrpt::maps::CRandomFieldGridMap2D::mrKalmanFilter)
			.value("mrKalmanApproximate", mrpt::maps::CRandomFieldGridMap2D::mrKalmanApproximate)
			.value("mrKernelDMV", mrpt::maps::CRandomFieldGridMap2D::mrKernelDMV)
			.value("mrGMRF_SD", mrpt::maps::CRandomFieldGridMap2D::mrGMRF_SD)
			.export_values();


		pybind11::enum_<mrpt::maps::CRandomFieldGridMap2D::TGridInterpolationMethod>(cl, "TGridInterpolationMethod", pybind11::arithmetic(), "")
			.value("gimNearest", mrpt::maps::CRandomFieldGridMap2D::gimNearest)
			.value("gimBilinear", mrpt::maps::CRandomFieldGridMap2D::gimBilinear)
			.export_values();

		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::maps::CRandomFieldGridMap2D::*)() const) &mrpt::maps::CRandomFieldGridMap2D::GetRuntimeClass, "C++: mrpt::maps::CRandomFieldGridMap2D::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::maps::CRandomFieldGridMap2D::GetRuntimeClassIdStatic, "C++: mrpt::maps::CRandomFieldGridMap2D::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("clear", (void (mrpt::maps::CRandomFieldGridMap2D::*)()) &mrpt::maps::CRandomFieldGridMap2D::clear, "Calls the base CMetricMap::clear\n Declared here to avoid ambiguity between the two clear() in both base\n classes.\n\nC++: mrpt::maps::CRandomFieldGridMap2D::clear() --> void");
		cl.def("cell2float", (float (mrpt::maps::CRandomFieldGridMap2D::*)(const struct mrpt::maps::TRandomFieldCell &) const) &mrpt::maps::CRandomFieldGridMap2D::cell2float, "C++: mrpt::maps::CRandomFieldGridMap2D::cell2float(const struct mrpt::maps::TRandomFieldCell &) const --> float", pybind11::arg("c"));
		cl.def("asString", (std::string (mrpt::maps::CRandomFieldGridMap2D::*)() const) &mrpt::maps::CRandomFieldGridMap2D::asString, "Returns a short description of the map. \n\nC++: mrpt::maps::CRandomFieldGridMap2D::asString() const --> std::string");
		cl.def("isEmpty", (bool (mrpt::maps::CRandomFieldGridMap2D::*)() const) &mrpt::maps::CRandomFieldGridMap2D::isEmpty, "Returns true if the map is empty/no observation has been inserted (in\n this class it always return false,\n unless redefined otherwise in base classes)\n\nC++: mrpt::maps::CRandomFieldGridMap2D::isEmpty() const --> bool");
		cl.def("saveAsBitmapFile", (void (mrpt::maps::CRandomFieldGridMap2D::*)(const std::string &) const) &mrpt::maps::CRandomFieldGridMap2D::saveAsBitmapFile, "Save the current map as a graphical file (BMP,PNG,...).\n The file format will be derived from the file extension (see\nCImage::saveToFile )\n  It depends on the map representation model:\n		mrAchim: Each pixel is the ratio \n\n\n		mrKalmanFilter: Each pixel is the mean value of the Gaussian that\nrepresents each cell.\n\n \n \n   \n\nC++: mrpt::maps::CRandomFieldGridMap2D::saveAsBitmapFile(const std::string &) const --> void", pybind11::arg("filName"));
		cl.def("getAsBitmapFile", (void (mrpt::maps::CRandomFieldGridMap2D::*)(class mrpt::img::CImage &) const) &mrpt::maps::CRandomFieldGridMap2D::getAsBitmapFile, "Returns an image just as described in  \n\nC++: mrpt::maps::CRandomFieldGridMap2D::getAsBitmapFile(class mrpt::img::CImage &) const --> void", pybind11::arg("out_img"));
		cl.def("getAsMatrix", (void (mrpt::maps::CRandomFieldGridMap2D::*)(class mrpt::math::CMatrixDynamic<double> &) const) &mrpt::maps::CRandomFieldGridMap2D::getAsMatrix, "Like saveAsBitmapFile(), but returns the data in matrix form (first row\n in the matrix is the upper (y_max) part of the map) \n\nC++: mrpt::maps::CRandomFieldGridMap2D::getAsMatrix(class mrpt::math::CMatrixDynamic<double> &) const --> void", pybind11::arg("out_mat"));
		cl.def("resize", [](mrpt::maps::CRandomFieldGridMap2D &o, double const & a0, double const & a1, double const & a2, double const & a3, const struct mrpt::maps::TRandomFieldCell & a4) -> void { return o.resize(a0, a1, a2, a3, a4); }, "", pybind11::arg("new_x_min"), pybind11::arg("new_x_max"), pybind11::arg("new_y_min"), pybind11::arg("new_y_max"), pybind11::arg("defaultValueNewCells"));
		cl.def("resize", (void (mrpt::maps::CRandomFieldGridMap2D::*)(double, double, double, double, const struct mrpt::maps::TRandomFieldCell &, double)) &mrpt::maps::CRandomFieldGridMap2D::resize, "Changes the size of the grid, maintaining previous contents. \n setSize\n\nC++: mrpt::maps::CRandomFieldGridMap2D::resize(double, double, double, double, const struct mrpt::maps::TRandomFieldCell &, double) --> void", pybind11::arg("new_x_min"), pybind11::arg("new_x_max"), pybind11::arg("new_y_min"), pybind11::arg("new_y_max"), pybind11::arg("defaultValueNewCells"), pybind11::arg("additionalMarginMeters"));
		cl.def("setSize", [](mrpt::maps::CRandomFieldGridMap2D &o, const double & a0, const double & a1, const double & a2, const double & a3, const double & a4) -> void { return o.setSize(a0, a1, a2, a3, a4); }, "", pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("resolution"));
		cl.def("setSize", (void (mrpt::maps::CRandomFieldGridMap2D::*)(const double, const double, const double, const double, const double, const struct mrpt::maps::TRandomFieldCell *)) &mrpt::maps::CRandomFieldGridMap2D::setSize, "Changes the size of the grid, erasing previous contents.\n  \n\n Optional user-supplied object that\n will visit all grid cells to define their connectivity with neighbors and\n the strength of existing edges. If present, it overrides all options in\n insertionOptions\n \n\n resize\n\nC++: mrpt::maps::CRandomFieldGridMap2D::setSize(const double, const double, const double, const double, const double, const struct mrpt::maps::TRandomFieldCell *) --> void", pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("resolution"), pybind11::arg("fill_value"));
		cl.def("setCellsConnectivity", (void (mrpt::maps::CRandomFieldGridMap2D::*)(const class std::shared_ptr<struct mrpt::maps::CRandomFieldGridMap2D::ConnectivityDescriptor> &)) &mrpt::maps::CRandomFieldGridMap2D::setCellsConnectivity, "Sets a custom object to define the connectivity between cells. Must call\n clear() or setSize() afterwards for the changes to take place. \n\nC++: mrpt::maps::CRandomFieldGridMap2D::setCellsConnectivity(const class std::shared_ptr<struct mrpt::maps::CRandomFieldGridMap2D::ConnectivityDescriptor> &) --> void", pybind11::arg("new_connectivity_descriptor"));
		cl.def("compute3DMatchingRatio", (float (mrpt::maps::CRandomFieldGridMap2D::*)(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose3D &, const struct mrpt::maps::TMatchingRatioParams &) const) &mrpt::maps::CRandomFieldGridMap2D::compute3DMatchingRatio, "See docs in base class: in this class this always returns 0 \n\nC++: mrpt::maps::CRandomFieldGridMap2D::compute3DMatchingRatio(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose3D &, const struct mrpt::maps::TMatchingRatioParams &) const --> float", pybind11::arg("otherMap"), pybind11::arg("otherMapPose"), pybind11::arg("params"));
		cl.def("saveMetricMapRepresentationToFile", (void (mrpt::maps::CRandomFieldGridMap2D::*)(const std::string &) const) &mrpt::maps::CRandomFieldGridMap2D::saveMetricMapRepresentationToFile, "The implementation in this class just calls all the corresponding method\n of the contained metric maps \n\nC++: mrpt::maps::CRandomFieldGridMap2D::saveMetricMapRepresentationToFile(const std::string &) const --> void", pybind11::arg("filNamePrefix"));
		cl.def("saveAsMatlab3DGraph", (void (mrpt::maps::CRandomFieldGridMap2D::*)(const std::string &) const) &mrpt::maps::CRandomFieldGridMap2D::saveAsMatlab3DGraph, "Save a matlab \".m\" file which represents as 3D surfaces the mean and a\n given confidence level for the concentration of each cell.\n  This method can only be called in a KF map model. \n\nC++: mrpt::maps::CRandomFieldGridMap2D::saveAsMatlab3DGraph(const std::string &) const --> void", pybind11::arg("filName"));
		cl.def("getVisualizationInto", (void (mrpt::maps::CRandomFieldGridMap2D::*)(class mrpt::opengl::CSetOfObjects &) const) &mrpt::maps::CRandomFieldGridMap2D::getVisualizationInto, "Returns a 3D object representing the map (mean) \n\nC++: mrpt::maps::CRandomFieldGridMap2D::getVisualizationInto(class mrpt::opengl::CSetOfObjects &) const --> void", pybind11::arg("outObj"));
		cl.def("getAs3DObject", (void (mrpt::maps::CRandomFieldGridMap2D::*)(class mrpt::opengl::CSetOfObjects &, class mrpt::opengl::CSetOfObjects &) const) &mrpt::maps::CRandomFieldGridMap2D::getAs3DObject, "Returns two 3D objects representing the mean and variance maps \n\nC++: mrpt::maps::CRandomFieldGridMap2D::getAs3DObject(class mrpt::opengl::CSetOfObjects &, class mrpt::opengl::CSetOfObjects &) const --> void", pybind11::arg("meanObj"), pybind11::arg("varObj"));
		cl.def("getMapType", (enum mrpt::maps::CRandomFieldGridMap2D::TMapRepresentation (mrpt::maps::CRandomFieldGridMap2D::*)()) &mrpt::maps::CRandomFieldGridMap2D::getMapType, "Return the type of the random-field grid map, according to parameters\n passed on construction. \n\nC++: mrpt::maps::CRandomFieldGridMap2D::getMapType() --> enum mrpt::maps::CRandomFieldGridMap2D::TMapRepresentation");
		cl.def("insertIndividualReading", [](mrpt::maps::CRandomFieldGridMap2D &o, const double & a0, const struct mrpt::math::TPoint2D_<double> & a1) -> void { return o.insertIndividualReading(a0, a1); }, "", pybind11::arg("sensorReading"), pybind11::arg("point"));
		cl.def("insertIndividualReading", [](mrpt::maps::CRandomFieldGridMap2D &o, const double & a0, const struct mrpt::math::TPoint2D_<double> & a1, const bool & a2) -> void { return o.insertIndividualReading(a0, a1, a2); }, "", pybind11::arg("sensorReading"), pybind11::arg("point"), pybind11::arg("update_map"));
		cl.def("insertIndividualReading", [](mrpt::maps::CRandomFieldGridMap2D &o, const double & a0, const struct mrpt::math::TPoint2D_<double> & a1, const bool & a2, const bool & a3) -> void { return o.insertIndividualReading(a0, a1, a2, a3); }, "", pybind11::arg("sensorReading"), pybind11::arg("point"), pybind11::arg("update_map"), pybind11::arg("time_invariant"));
		cl.def("insertIndividualReading", (void (mrpt::maps::CRandomFieldGridMap2D::*)(const double, const struct mrpt::math::TPoint2D_<double> &, const bool, const bool, const double)) &mrpt::maps::CRandomFieldGridMap2D::insertIndividualReading, "Direct update of the map with a reading in a given position of the map,\n using\n  the appropriate method according to mapType passed in the constructor.\n\n This is a direct way to update the map, an alternative to the generic\n insertObservation() method which works with mrpt::obs::CObservation\n objects.\n\nC++: mrpt::maps::CRandomFieldGridMap2D::insertIndividualReading(const double, const struct mrpt::math::TPoint2D_<double> &, const bool, const bool, const double) --> void", pybind11::arg("sensorReading"), pybind11::arg("point"), pybind11::arg("update_map"), pybind11::arg("time_invariant"), pybind11::arg("reading_stddev"));
		cl.def("predictMeasurement", [](mrpt::maps::CRandomFieldGridMap2D &o, const double & a0, const double & a1, double & a2, double & a3, bool const & a4) -> void { return o.predictMeasurement(a0, a1, a2, a3, a4); }, "", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("out_predict_response"), pybind11::arg("out_predict_response_variance"), pybind11::arg("do_sensor_normalization"));
		cl.def("predictMeasurement", (void (mrpt::maps::CRandomFieldGridMap2D::*)(const double, const double, double &, double &, bool, const enum mrpt::maps::CRandomFieldGridMap2D::TGridInterpolationMethod)) &mrpt::maps::CRandomFieldGridMap2D::predictMeasurement, "Returns the prediction of the measurement at some (x,y) coordinates, and\n its certainty (in the form of the expected variance).  \n\nC++: mrpt::maps::CRandomFieldGridMap2D::predictMeasurement(const double, const double, double &, double &, bool, const enum mrpt::maps::CRandomFieldGridMap2D::TGridInterpolationMethod) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("out_predict_response"), pybind11::arg("out_predict_response_variance"), pybind11::arg("do_sensor_normalization"), pybind11::arg("interp_method"));
		cl.def("getMeanAndCov", (void (mrpt::maps::CRandomFieldGridMap2D::*)(class mrpt::math::CVectorDynamic<double> &, class mrpt::math::CMatrixDynamic<double> &) const) &mrpt::maps::CRandomFieldGridMap2D::getMeanAndCov, "Return the mean and covariance vector of the full Kalman filter estimate\n (works for all KF-based methods). \n\nC++: mrpt::maps::CRandomFieldGridMap2D::getMeanAndCov(class mrpt::math::CVectorDynamic<double> &, class mrpt::math::CMatrixDynamic<double> &) const --> void", pybind11::arg("out_means"), pybind11::arg("out_cov"));
		cl.def("getMeanAndSTD", (void (mrpt::maps::CRandomFieldGridMap2D::*)(class mrpt::math::CVectorDynamic<double> &, class mrpt::math::CVectorDynamic<double> &) const) &mrpt::maps::CRandomFieldGridMap2D::getMeanAndSTD, "Return the mean and STD vectors of the full Kalman filter estimate\n (works for all KF-based methods). \n\nC++: mrpt::maps::CRandomFieldGridMap2D::getMeanAndSTD(class mrpt::math::CVectorDynamic<double> &, class mrpt::math::CVectorDynamic<double> &) const --> void", pybind11::arg("out_means"), pybind11::arg("out_STD"));
		cl.def("setMeanAndSTD", (void (mrpt::maps::CRandomFieldGridMap2D::*)(class mrpt::math::CVectorDynamic<double> &, class mrpt::math::CVectorDynamic<double> &)) &mrpt::maps::CRandomFieldGridMap2D::setMeanAndSTD, "Load the mean and STD vectors of the full Kalman filter estimate (works\n for all KF-based methods). \n\nC++: mrpt::maps::CRandomFieldGridMap2D::setMeanAndSTD(class mrpt::math::CVectorDynamic<double> &, class mrpt::math::CVectorDynamic<double> &) --> void", pybind11::arg("out_means"), pybind11::arg("out_STD"));
		cl.def("updateMapEstimation", (void (mrpt::maps::CRandomFieldGridMap2D::*)()) &mrpt::maps::CRandomFieldGridMap2D::updateMapEstimation, "Run the method-specific procedure required to ensure that the mean &\n variances are up-to-date with all inserted observations. \n\nC++: mrpt::maps::CRandomFieldGridMap2D::updateMapEstimation() --> void");
		cl.def("enableVerbose", (void (mrpt::maps::CRandomFieldGridMap2D::*)(bool)) &mrpt::maps::CRandomFieldGridMap2D::enableVerbose, "C++: mrpt::maps::CRandomFieldGridMap2D::enableVerbose(bool) --> void", pybind11::arg("enable_verbose"));
		cl.def("isEnabledVerbose", (bool (mrpt::maps::CRandomFieldGridMap2D::*)() const) &mrpt::maps::CRandomFieldGridMap2D::isEnabledVerbose, "C++: mrpt::maps::CRandomFieldGridMap2D::isEnabledVerbose() const --> bool");
		cl.def("enableProfiler", [](mrpt::maps::CRandomFieldGridMap2D &o) -> void { return o.enableProfiler(); }, "");
		cl.def("enableProfiler", (void (mrpt::maps::CRandomFieldGridMap2D::*)(bool)) &mrpt::maps::CRandomFieldGridMap2D::enableProfiler, "C++: mrpt::maps::CRandomFieldGridMap2D::enableProfiler(bool) --> void", pybind11::arg("enable"));
		cl.def("isProfilerEnabled", (bool (mrpt::maps::CRandomFieldGridMap2D::*)() const) &mrpt::maps::CRandomFieldGridMap2D::isProfilerEnabled, "C++: mrpt::maps::CRandomFieldGridMap2D::isProfilerEnabled() const --> bool");
		cl.def("assign", (class mrpt::maps::CRandomFieldGridMap2D & (mrpt::maps::CRandomFieldGridMap2D::*)(const class mrpt::maps::CRandomFieldGridMap2D &)) &mrpt::maps::CRandomFieldGridMap2D::operator=, "C++: mrpt::maps::CRandomFieldGridMap2D::operator=(const class mrpt::maps::CRandomFieldGridMap2D &) --> class mrpt::maps::CRandomFieldGridMap2D &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon file:mrpt/maps/CRandomFieldGridMap2D.h line:253
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon, std::shared_ptr<mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon>> cl(enclosing_class, "TInsertionOptionsCommon", "Parameters common to any derived class.\n  Derived classes should derive a new struct from this one, plus \"public\n CLoadableOptions\",\n  and call the internal_* methods where appropiate to deal with the\n variables declared here.\n  Derived classes instantions of their \"TInsertionOptions\" MUST set the\n pointer \"m_insertOptions_common\" upon construction.");
			cl.def( pybind11::init( [](){ return new mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon(); } ) );
			cl.def( pybind11::init( [](mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon const &o){ return new mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon(o); } ) );
			cl.def_readwrite("sigma", &mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon::sigma);
			cl.def_readwrite("cutoffRadius", &mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon::cutoffRadius);
			cl.def_readwrite("R_min", &mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon::R_min);
			cl.def_readwrite("R_max", &mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon::R_max);
			cl.def_readwrite("dm_sigma_omega", &mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon::dm_sigma_omega);
			cl.def_readwrite("KF_covSigma", &mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon::KF_covSigma);
			cl.def_readwrite("KF_initialCellStd", &mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon::KF_initialCellStd);
			cl.def_readwrite("KF_observationModelNoise", &mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon::KF_observationModelNoise);
			cl.def_readwrite("KF_defaultCellMeanValue", &mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon::KF_defaultCellMeanValue);
			cl.def_readwrite("KF_W_size", &mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon::KF_W_size);
			cl.def_readwrite("GMRF_lambdaPrior", &mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon::GMRF_lambdaPrior);
			cl.def_readwrite("GMRF_lambdaObs", &mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon::GMRF_lambdaObs);
			cl.def_readwrite("GMRF_lambdaObsLoss", &mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon::GMRF_lambdaObsLoss);
			cl.def_readwrite("GMRF_use_occupancy_information", &mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon::GMRF_use_occupancy_information);
			cl.def_readwrite("GMRF_simplemap_file", &mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon::GMRF_simplemap_file);
			cl.def_readwrite("GMRF_gridmap_image_file", &mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon::GMRF_gridmap_image_file);
			cl.def_readwrite("GMRF_gridmap_image_res", &mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon::GMRF_gridmap_image_res);
			cl.def_readwrite("GMRF_gridmap_image_cx", &mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon::GMRF_gridmap_image_cx);
			cl.def_readwrite("GMRF_gridmap_image_cy", &mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon::GMRF_gridmap_image_cy);
			cl.def_readwrite("GMRF_saturate_min", &mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon::GMRF_saturate_min);
			cl.def_readwrite("GMRF_saturate_max", &mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon::GMRF_saturate_max);
			cl.def_readwrite("GMRF_skip_variance", &mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon::GMRF_skip_variance);
			cl.def("internal_loadFromConfigFile_common", (void (mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon::internal_loadFromConfigFile_common, "See mrpt::config::CLoadableOptions \n\nC++: mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon::internal_loadFromConfigFile_common(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
			cl.def("assign", (struct mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon & (mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon::*)(const struct mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon &)) &mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon::operator=, "C++: mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon::operator=(const struct mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon &) --> struct mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::maps::CRandomFieldGridMap2D::ConnectivityDescriptor file:mrpt/maps/CRandomFieldGridMap2D.h line:355
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CRandomFieldGridMap2D::ConnectivityDescriptor, std::shared_ptr<mrpt::maps::CRandomFieldGridMap2D::ConnectivityDescriptor>, PyCallBack_mrpt_maps_CRandomFieldGridMap2D_ConnectivityDescriptor> cl(enclosing_class, "ConnectivityDescriptor", "Base class for user-supplied objects capable of describing cells\n connectivity, used to build prior factors of the MRF graph. \n\n\n setCellsConnectivity() ");
			cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_maps_CRandomFieldGridMap2D_ConnectivityDescriptor(); } ) );
			cl.def("getEdgeInformation", (bool (mrpt::maps::CRandomFieldGridMap2D::ConnectivityDescriptor::*)(const class mrpt::maps::CRandomFieldGridMap2D *, size_t, size_t, size_t, size_t, double &)) &mrpt::maps::CRandomFieldGridMap2D::ConnectivityDescriptor::getEdgeInformation, "Implement the check of whether node i=(icx,icy) is connected with\n node j=(jcx,jcy).\n This visitor method will be called only for immediate neighbors.\n \n\n true if connected (and the \"information\" value should be\n also updated in out_edge_information), false otherwise.\n\nC++: mrpt::maps::CRandomFieldGridMap2D::ConnectivityDescriptor::getEdgeInformation(const class mrpt::maps::CRandomFieldGridMap2D *, size_t, size_t, size_t, size_t, double &) --> bool", pybind11::arg("parent"), pybind11::arg("icx"), pybind11::arg("icy"), pybind11::arg("jcx"), pybind11::arg("jcy"), pybind11::arg("out_edge_information"));
			cl.def("assign", (struct mrpt::maps::CRandomFieldGridMap2D::ConnectivityDescriptor & (mrpt::maps::CRandomFieldGridMap2D::ConnectivityDescriptor::*)(const struct mrpt::maps::CRandomFieldGridMap2D::ConnectivityDescriptor &)) &mrpt::maps::CRandomFieldGridMap2D::ConnectivityDescriptor::operator=, "C++: mrpt::maps::CRandomFieldGridMap2D::ConnectivityDescriptor::operator=(const struct mrpt::maps::CRandomFieldGridMap2D::ConnectivityDescriptor &) --> struct mrpt::maps::CRandomFieldGridMap2D::ConnectivityDescriptor &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
}
