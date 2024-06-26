#include <chrono>
#include <ios>
#include <istream>
#include <iterator>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/Clock.h>
#include <mrpt/core/aligned_allocator.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/img/TColor.h>
#include <mrpt/img/color_maps.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/metric_map_types.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CPolygon.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/T2DScanProperties.h>
#include <mrpt/obs/T3DPointsTo2DScanParams.h>
#include <mrpt/obs/TRangeImageFilter.h>
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
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ostream>
#include <ratio>
#include <sstream> // __str__
#include <streambuf>
#include <string>
#include <type_traits>
#include <utility>
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

// mrpt::maps::CPointsMap::TInsertionOptions file:mrpt/maps/CPointsMap.h line:228
struct PyCallBack_mrpt_maps_CPointsMap_TInsertionOptions : public mrpt::maps::CPointsMap::TInsertionOptions {
	using mrpt::maps::CPointsMap::TInsertionOptions::TInsertionOptions;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMap::TInsertionOptions *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TInsertionOptions::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMap::TInsertionOptions *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CLoadableOptions::saveToConfigFile(a0, a1);
	}
};

// mrpt::maps::CPointsMap::TLikelihoodOptions file:mrpt/maps/CPointsMap.h line:285
struct PyCallBack_mrpt_maps_CPointsMap_TLikelihoodOptions : public mrpt::maps::CPointsMap::TLikelihoodOptions {
	using mrpt::maps::CPointsMap::TLikelihoodOptions::TLikelihoodOptions;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMap::TLikelihoodOptions *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TLikelihoodOptions::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMap::TLikelihoodOptions *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CLoadableOptions::saveToConfigFile(a0, a1);
	}
};

// mrpt::maps::CPointsMap::TRenderOptions file:mrpt/maps/CPointsMap.h line:318
struct PyCallBack_mrpt_maps_CPointsMap_TRenderOptions : public mrpt::maps::CPointsMap::TRenderOptions {
	using mrpt::maps::CPointsMap::TRenderOptions::TRenderOptions;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMap::TRenderOptions *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TRenderOptions::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMap::TRenderOptions *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CLoadableOptions::saveToConfigFile(a0, a1);
	}
};

void bind_mrpt_maps_CPointsMap(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::maps::CPointsMap file:mrpt/maps/CPointsMap.h line:71
		pybind11::class_<mrpt::maps::CPointsMap, std::shared_ptr<mrpt::maps::CPointsMap>, mrpt::maps::CMetricMap, mrpt::opengl::PLY_Importer, mrpt::opengl::PLY_Exporter, mrpt::maps::NearestNeighborsCapable> cl(M("mrpt::maps"), "CPointsMap", "A cloud of points in 2D or 3D, which can be built from a sequence of laser\n scans or other sensors.\n  This is a virtual class, thus only a derived class can be instantiated by\n the user. The user most usually wants to use CSimplePointsMap.\n\n  This class implements generic version of\n mrpt::maps::CMetric::insertObservation() accepting these types of sensory\n data:\n   - mrpt::obs::CObservation2DRangeScan: 2D range scans\n   - mrpt::obs::CObservation3DRangeScan: 3D range scans (Kinect, etc...)\n   - mrpt::obs::CObservationRange: IRs, Sonars, etc.\n   - mrpt::obs::CObservationVelodyneScan\n   - mrpt::obs::CObservationPointCloud\n\n Loading and saving in the standard LAS LiDAR point cloud format is supported\n by installing `libLAS` and including the\n header `<mrpt/maps/CPointsMaps_liblas.h>` in your program. Since MRPT 1.5.0\n there is no need to build MRPT against libLAS to use this feature.\n See LAS functions in \n\n \n CMetricMap, CPoint, CSerializable\n \n\n\n ");
		cl.def_readwrite("insertionOptions", &mrpt::maps::CPointsMap::insertionOptions);
		cl.def_readwrite("likelihoodOptions", &mrpt::maps::CPointsMap::likelihoodOptions);
		cl.def_readwrite("renderOptions", &mrpt::maps::CPointsMap::renderOptions);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::maps::CPointsMap::*)() const) &mrpt::maps::CPointsMap::GetRuntimeClass, "C++: mrpt::maps::CPointsMap::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::maps::CPointsMap::GetRuntimeClassIdStatic, "C++: mrpt::maps::CPointsMap::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("assign", (class mrpt::maps::CPointsMap & (mrpt::maps::CPointsMap::*)(const class mrpt::maps::CPointsMap &)) &mrpt::maps::CPointsMap::operator=, "C++: mrpt::maps::CPointsMap::operator=(const class mrpt::maps::CPointsMap &) --> class mrpt::maps::CPointsMap &", pybind11::return_value_policy::automatic, pybind11::arg("o"));
		cl.def("reserve", (void (mrpt::maps::CPointsMap::*)(size_t)) &mrpt::maps::CPointsMap::reserve, "Reserves memory for a given number of points: the size of the map does\n not change, it only reserves the memory.\n  This is useful for situations where it is approximately known the final\n size of the map. This method is more\n  efficient than constantly increasing the size of the buffers. Refer to\n the STL C++ library's \"reserve\" methods.\n\nC++: mrpt::maps::CPointsMap::reserve(size_t) --> void", pybind11::arg("newLength"));
		cl.def("resize", (void (mrpt::maps::CPointsMap::*)(size_t)) &mrpt::maps::CPointsMap::resize, "Resizes all point buffers so they can hold the given number of points:\n newly created points are set to default values,\n  and old contents are not changed.\n \n\n reserve, setPoint, setPointFast, setSize\n\nC++: mrpt::maps::CPointsMap::resize(size_t) --> void", pybind11::arg("newLength"));
		cl.def("setSize", (void (mrpt::maps::CPointsMap::*)(size_t)) &mrpt::maps::CPointsMap::setSize, "Resizes all point buffers so they can hold the given number of points,\n *erasing* all previous contents\n  and leaving all points to default values.\n \n\n reserve, setPoint, setPointFast, setSize\n\nC++: mrpt::maps::CPointsMap::setSize(size_t) --> void", pybind11::arg("newLength"));
		cl.def("setPointFast", (void (mrpt::maps::CPointsMap::*)(size_t, float, float, float)) &mrpt::maps::CPointsMap::setPointFast, "Changes the coordinates of the given point (0-based index), *without*\n checking for out-of-bounds and *without* calling mark_as_modified().\n Also, color, intensity, or other data is left unchanged. \n\n setPoint \n\nC++: mrpt::maps::CPointsMap::setPointFast(size_t, float, float, float) --> void", pybind11::arg("index"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("insertPointFast", [](mrpt::maps::CPointsMap &o, float const & a0, float const & a1) -> void { return o.insertPointFast(a0, a1); }, "", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("insertPointFast", (void (mrpt::maps::CPointsMap::*)(float, float, float)) &mrpt::maps::CPointsMap::insertPointFast, "The virtual method for  *without* calling\n mark_as_modified()   \n\nC++: mrpt::maps::CPointsMap::insertPointFast(float, float, float) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("squareDistanceToClosestCorrespondence", (float (mrpt::maps::CPointsMap::*)(float, float) const) &mrpt::maps::CPointsMap::squareDistanceToClosestCorrespondence, "Returns the square distance from the 2D point (x0,y0) to the closest\n correspondence in the map.\n\nC++: mrpt::maps::CPointsMap::squareDistanceToClosestCorrespondence(float, float) const --> float", pybind11::arg("x0"), pybind11::arg("y0"));
		cl.def("squareDistanceToClosestCorrespondenceT", (float (mrpt::maps::CPointsMap::*)(const struct mrpt::math::TPoint2D_<double> &) const) &mrpt::maps::CPointsMap::squareDistanceToClosestCorrespondenceT, "C++: mrpt::maps::CPointsMap::squareDistanceToClosestCorrespondenceT(const struct mrpt::math::TPoint2D_<double> &) const --> float", pybind11::arg("p0"));
		cl.def("insertAnotherMap", [](mrpt::maps::CPointsMap &o, const class mrpt::maps::CPointsMap * a0, const class mrpt::poses::CPose3D & a1) -> void { return o.insertAnotherMap(a0, a1); }, "", pybind11::arg("otherMap"), pybind11::arg("otherPose"));
		cl.def("insertAnotherMap", (void (mrpt::maps::CPointsMap::*)(const class mrpt::maps::CPointsMap *, const class mrpt::poses::CPose3D &, const bool)) &mrpt::maps::CPointsMap::insertAnotherMap, "Insert the contents of another map into this one with some geometric\n transformation, without fusing close points.\n \n\n The other map whose points are to be inserted into this\n one.\n \n\n The pose of the other map in the coordinates of THIS map\n \n\n If true, points at (0,0,0) (in the frame of\n reference of `otherMap`) will be assumed to be invalid and will not be\n copied.\n\n \n fuseWith, addFrom\n\nC++: mrpt::maps::CPointsMap::insertAnotherMap(const class mrpt::maps::CPointsMap *, const class mrpt::poses::CPose3D &, const bool) --> void", pybind11::arg("otherMap"), pybind11::arg("otherPose"), pybind11::arg("filterOutPointsAtZero"));
		cl.def("__iadd__", (void (mrpt::maps::CPointsMap::*)(const class mrpt::maps::CPointsMap &)) &mrpt::maps::CPointsMap::operator+=, "Inserts another map into this one. \n insertAnotherMap() \n\nC++: mrpt::maps::CPointsMap::operator+=(const class mrpt::maps::CPointsMap &) --> void", pybind11::arg("anotherMap"));
		cl.def("load2D_from_text_file", (bool (mrpt::maps::CPointsMap::*)(const std::string &)) &mrpt::maps::CPointsMap::load2D_from_text_file, "Load from a text file. Each line should contain an \"X Y\" coordinate\n pair, separated by whitespaces.\n   Returns false if any error occured, true elsewere.\n\nC++: mrpt::maps::CPointsMap::load2D_from_text_file(const std::string &) --> bool", pybind11::arg("file"));
		cl.def("load3D_from_text_file", (bool (mrpt::maps::CPointsMap::*)(const std::string &)) &mrpt::maps::CPointsMap::load3D_from_text_file, "Load from a text file. Each line should contain an \"X Y Z\" coordinate\n tuple, separated by whitespaces.\n   Returns false if any error occured, true elsewere.\n\nC++: mrpt::maps::CPointsMap::load3D_from_text_file(const std::string &) --> bool", pybind11::arg("file"));
		cl.def("load2Dor3D_from_text_file", (bool (mrpt::maps::CPointsMap::*)(const std::string &, const bool)) &mrpt::maps::CPointsMap::load2Dor3D_from_text_file, "2D or 3D generic implementation of  and\n load3D_from_text_file \n\nC++: mrpt::maps::CPointsMap::load2Dor3D_from_text_file(const std::string &, const bool) --> bool", pybind11::arg("file"), pybind11::arg("is_3D"));
		cl.def("save2D_to_text_file", (bool (mrpt::maps::CPointsMap::*)(const std::string &) const) &mrpt::maps::CPointsMap::save2D_to_text_file, "Save to a text file. Each line will contain \"X Y\" point coordinates.\n		Returns false if any error occured, true elsewere.\n\nC++: mrpt::maps::CPointsMap::save2D_to_text_file(const std::string &) const --> bool", pybind11::arg("file"));
		cl.def("save3D_to_text_file", (bool (mrpt::maps::CPointsMap::*)(const std::string &) const) &mrpt::maps::CPointsMap::save3D_to_text_file, "Save to a text file. Each line will contain \"X Y Z\" point coordinates.\n     Returns false if any error occured, true elsewere.\n\nC++: mrpt::maps::CPointsMap::save3D_to_text_file(const std::string &) const --> bool", pybind11::arg("file"));
		cl.def("saveMetricMapRepresentationToFile", (void (mrpt::maps::CPointsMap::*)(const std::string &) const) &mrpt::maps::CPointsMap::saveMetricMapRepresentationToFile, "This virtual method saves the map to a file \"filNamePrefix\"+<\n some_file_extension >, as an image or in any other applicable way (Notice\n that other methods to save the map may be implemented in classes\n implementing this virtual interface) \n\nC++: mrpt::maps::CPointsMap::saveMetricMapRepresentationToFile(const std::string &) const --> void", pybind11::arg("filNamePrefix"));
		cl.def("size", (size_t (mrpt::maps::CPointsMap::*)() const) &mrpt::maps::CPointsMap::size, "Returns the number of stored points in the map.\n\nC++: mrpt::maps::CPointsMap::size() const --> size_t");
		cl.def("getPoint", (void (mrpt::maps::CPointsMap::*)(size_t, float &, float &, float &) const) &mrpt::maps::CPointsMap::getPoint, "Access to a given point from map, as a 2D point. First index is 0.\n \n\n Throws std::exception on index out of bound.\n \n\n setPoint, getPointFast\n\nC++: mrpt::maps::CPointsMap::getPoint(size_t, float &, float &, float &) const --> void", pybind11::arg("index"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("getPoint", (void (mrpt::maps::CPointsMap::*)(size_t, float &, float &) const) &mrpt::maps::CPointsMap::getPoint, "C++: mrpt::maps::CPointsMap::getPoint(size_t, float &, float &) const --> void", pybind11::arg("index"), pybind11::arg("x"), pybind11::arg("y"));
		cl.def("getPoint", (void (mrpt::maps::CPointsMap::*)(size_t, double &, double &, double &) const) &mrpt::maps::CPointsMap::getPoint, "C++: mrpt::maps::CPointsMap::getPoint(size_t, double &, double &, double &) const --> void", pybind11::arg("index"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("getPoint", (void (mrpt::maps::CPointsMap::*)(size_t, double &, double &) const) &mrpt::maps::CPointsMap::getPoint, "C++: mrpt::maps::CPointsMap::getPoint(size_t, double &, double &) const --> void", pybind11::arg("index"), pybind11::arg("x"), pybind11::arg("y"));
		cl.def("getPoint", (void (mrpt::maps::CPointsMap::*)(size_t, struct mrpt::math::TPoint2D_<double> &) const) &mrpt::maps::CPointsMap::getPoint, "C++: mrpt::maps::CPointsMap::getPoint(size_t, struct mrpt::math::TPoint2D_<double> &) const --> void", pybind11::arg("index"), pybind11::arg("p"));
		cl.def("getPoint", (void (mrpt::maps::CPointsMap::*)(size_t, struct mrpt::math::TPoint3D_<double> &) const) &mrpt::maps::CPointsMap::getPoint, "C++: mrpt::maps::CPointsMap::getPoint(size_t, struct mrpt::math::TPoint3D_<double> &) const --> void", pybind11::arg("index"), pybind11::arg("p"));
		cl.def("getPointRGB", (void (mrpt::maps::CPointsMap::*)(size_t, float &, float &, float &, float &, float &, float &) const) &mrpt::maps::CPointsMap::getPointRGB, "Access to a given point from map, and its colors, if the map defines\n them (othersise, R=G=B=1.0). First index is 0.\n \n\n The return value is the weight of the point (the times it has\n been fused)\n \n\n Throws std::exception on index out of bound.\n\nC++: mrpt::maps::CPointsMap::getPointRGB(size_t, float &, float &, float &, float &, float &, float &) const --> void", pybind11::arg("index"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("R"), pybind11::arg("G"), pybind11::arg("B"));
		cl.def("getPointFast", (void (mrpt::maps::CPointsMap::*)(size_t, float &, float &, float &) const) &mrpt::maps::CPointsMap::getPointFast, "Just like  but without checking out-of-bound index and\n without returning the point weight, just XYZ.\n\nC++: mrpt::maps::CPointsMap::getPointFast(size_t, float &, float &, float &) const --> void", pybind11::arg("index"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("hasColorPoints", (bool (mrpt::maps::CPointsMap::*)() const) &mrpt::maps::CPointsMap::hasColorPoints, "Returns true if the point map has a color field for each point \n\nC++: mrpt::maps::CPointsMap::hasColorPoints() const --> bool");
		cl.def("setPoint", (void (mrpt::maps::CPointsMap::*)(size_t, float, float, float)) &mrpt::maps::CPointsMap::setPoint, "Changes a given point from map, with Z defaulting to 0 if not provided.\n \n\n Throws std::exception on index out of bound.\n\nC++: mrpt::maps::CPointsMap::setPoint(size_t, float, float, float) --> void", pybind11::arg("index"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("setPoint", (void (mrpt::maps::CPointsMap::*)(size_t, const struct mrpt::math::TPoint2D_<double> &)) &mrpt::maps::CPointsMap::setPoint, "C++: mrpt::maps::CPointsMap::setPoint(size_t, const struct mrpt::math::TPoint2D_<double> &) --> void", pybind11::arg("index"), pybind11::arg("p"));
		cl.def("setPoint", (void (mrpt::maps::CPointsMap::*)(size_t, const struct mrpt::math::TPoint3D_<double> &)) &mrpt::maps::CPointsMap::setPoint, "C++: mrpt::maps::CPointsMap::setPoint(size_t, const struct mrpt::math::TPoint3D_<double> &) --> void", pybind11::arg("index"), pybind11::arg("p"));
		cl.def("setPoint", (void (mrpt::maps::CPointsMap::*)(size_t, float, float)) &mrpt::maps::CPointsMap::setPoint, "C++: mrpt::maps::CPointsMap::setPoint(size_t, float, float) --> void", pybind11::arg("index"), pybind11::arg("x"), pybind11::arg("y"));
		cl.def("setPointRGB", (void (mrpt::maps::CPointsMap::*)(size_t, float, float, float, float, float, float)) &mrpt::maps::CPointsMap::setPointRGB, "overload (RGB data is ignored in classes without color information)\n\nC++: mrpt::maps::CPointsMap::setPointRGB(size_t, float, float, float, float, float, float) --> void", pybind11::arg("index"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("R"), pybind11::arg("G"), pybind11::arg("B"));
		cl.def("setPointWeight", (void (mrpt::maps::CPointsMap::*)(size_t, unsigned long)) &mrpt::maps::CPointsMap::setPointWeight, "Sets the point weight, which is ignored in all classes but those which\n actually store that field (Note: No checks are done for out-of-bounds\n index). \n\n getPointWeight\n\nC++: mrpt::maps::CPointsMap::setPointWeight(size_t, unsigned long) --> void", pybind11::arg("index"), pybind11::arg("w"));
		cl.def("getPointWeight", (unsigned long (mrpt::maps::CPointsMap::*)(size_t) const) &mrpt::maps::CPointsMap::getPointWeight, "Gets the point weight, which is ignored in all classes (defaults to 1)\n but in those which actually store that field (Note: No checks are done\n for out-of-bounds index).  \n\n setPointWeight\n\nC++: mrpt::maps::CPointsMap::getPointWeight(size_t) const --> unsigned long", pybind11::arg("index"));
		cl.def("hasField_Intensity", (bool (mrpt::maps::CPointsMap::*)() const) &mrpt::maps::CPointsMap::hasField_Intensity, "C++: mrpt::maps::CPointsMap::hasField_Intensity() const --> bool");
		cl.def("hasField_Ring", (bool (mrpt::maps::CPointsMap::*)() const) &mrpt::maps::CPointsMap::hasField_Ring, "C++: mrpt::maps::CPointsMap::hasField_Ring() const --> bool");
		cl.def("hasField_Timestamp", (bool (mrpt::maps::CPointsMap::*)() const) &mrpt::maps::CPointsMap::hasField_Timestamp, "C++: mrpt::maps::CPointsMap::hasField_Timestamp() const --> bool");
		cl.def("hasField_color_R", (bool (mrpt::maps::CPointsMap::*)() const) &mrpt::maps::CPointsMap::hasField_color_R, "C++: mrpt::maps::CPointsMap::hasField_color_R() const --> bool");
		cl.def("hasField_color_G", (bool (mrpt::maps::CPointsMap::*)() const) &mrpt::maps::CPointsMap::hasField_color_G, "C++: mrpt::maps::CPointsMap::hasField_color_G() const --> bool");
		cl.def("hasField_color_B", (bool (mrpt::maps::CPointsMap::*)() const) &mrpt::maps::CPointsMap::hasField_color_B, "C++: mrpt::maps::CPointsMap::hasField_color_B() const --> bool");
		cl.def("insertPointField_Intensity", (void (mrpt::maps::CPointsMap::*)(float)) &mrpt::maps::CPointsMap::insertPointField_Intensity, "C++: mrpt::maps::CPointsMap::insertPointField_Intensity(float) --> void", pybind11::arg("i"));
		cl.def("insertPointField_Ring", (void (mrpt::maps::CPointsMap::*)(uint16_t)) &mrpt::maps::CPointsMap::insertPointField_Ring, "C++: mrpt::maps::CPointsMap::insertPointField_Ring(uint16_t) --> void", pybind11::arg("r"));
		cl.def("insertPointField_Timestamp", (void (mrpt::maps::CPointsMap::*)(float)) &mrpt::maps::CPointsMap::insertPointField_Timestamp, "C++: mrpt::maps::CPointsMap::insertPointField_Timestamp(float) --> void", pybind11::arg("t"));
		cl.def("insertPointField_color_R", (void (mrpt::maps::CPointsMap::*)(float)) &mrpt::maps::CPointsMap::insertPointField_color_R, "C++: mrpt::maps::CPointsMap::insertPointField_color_R(float) --> void", pybind11::arg("v"));
		cl.def("insertPointField_color_G", (void (mrpt::maps::CPointsMap::*)(float)) &mrpt::maps::CPointsMap::insertPointField_color_G, "C++: mrpt::maps::CPointsMap::insertPointField_color_G(float) --> void", pybind11::arg("v"));
		cl.def("insertPointField_color_B", (void (mrpt::maps::CPointsMap::*)(float)) &mrpt::maps::CPointsMap::insertPointField_color_B, "C++: mrpt::maps::CPointsMap::insertPointField_color_B(float) --> void", pybind11::arg("v"));
		cl.def("insertPoint", [](mrpt::maps::CPointsMap &o, float const & a0, float const & a1) -> void { return o.insertPoint(a0, a1); }, "", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("insertPoint", (void (mrpt::maps::CPointsMap::*)(float, float, float)) &mrpt::maps::CPointsMap::insertPoint, "Provides a way to insert (append) individual points into the map: the\n missing fields of child\n classes (color, weight, etc) are left to their default values\n\nC++: mrpt::maps::CPointsMap::insertPoint(float, float, float) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("insertPoint", (void (mrpt::maps::CPointsMap::*)(const struct mrpt::math::TPoint3D_<double> &)) &mrpt::maps::CPointsMap::insertPoint, "C++: mrpt::maps::CPointsMap::insertPoint(const struct mrpt::math::TPoint3D_<double> &) --> void", pybind11::arg("p"));
		cl.def("insertPointRGB", (void (mrpt::maps::CPointsMap::*)(float, float, float, float, float, float)) &mrpt::maps::CPointsMap::insertPointRGB, "overload (RGB data is ignored in classes without color information)\n\nC++: mrpt::maps::CPointsMap::insertPointRGB(float, float, float, float, float, float) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("R"), pybind11::arg("G"), pybind11::arg("B"));
		cl.def("insertPointFrom", (void (mrpt::maps::CPointsMap::*)(const class mrpt::maps::CPointsMap &, size_t)) &mrpt::maps::CPointsMap::insertPointFrom, "Generic method to copy *all* applicable point properties from\n  one map to another, e.g. timestamp, intensity, etc.\n\nC++: mrpt::maps::CPointsMap::insertPointFrom(const class mrpt::maps::CPointsMap &, size_t) --> void", pybind11::arg("source"), pybind11::arg("sourcePointIndex"));
		cl.def("clipOutOfRangeInZ", (void (mrpt::maps::CPointsMap::*)(float, float)) &mrpt::maps::CPointsMap::clipOutOfRangeInZ, "Delete points out of the given \"z\" axis range have been removed.\n\nC++: mrpt::maps::CPointsMap::clipOutOfRangeInZ(float, float) --> void", pybind11::arg("zMin"), pybind11::arg("zMax"));
		cl.def("clipOutOfRange", (void (mrpt::maps::CPointsMap::*)(const struct mrpt::math::TPoint2D_<double> &, float)) &mrpt::maps::CPointsMap::clipOutOfRange, "Delete points which are more far than \"maxRange\" away from the given\n \"point\".\n\nC++: mrpt::maps::CPointsMap::clipOutOfRange(const struct mrpt::math::TPoint2D_<double> &, float) --> void", pybind11::arg("point"), pybind11::arg("maxRange"));
		cl.def("determineMatching2D", (void (mrpt::maps::CPointsMap::*)(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose2D &, class mrpt::tfest::TMatchingPairListTempl<float> &, const struct mrpt::maps::TMatchingParams &, struct mrpt::maps::TMatchingExtraResults &) const) &mrpt::maps::CPointsMap::determineMatching2D, "C++: mrpt::maps::CPointsMap::determineMatching2D(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose2D &, class mrpt::tfest::TMatchingPairListTempl<float> &, const struct mrpt::maps::TMatchingParams &, struct mrpt::maps::TMatchingExtraResults &) const --> void", pybind11::arg("otherMap"), pybind11::arg("otherMapPose"), pybind11::arg("correspondences"), pybind11::arg("params"), pybind11::arg("extraResults"));
		cl.def("determineMatching3D", (void (mrpt::maps::CPointsMap::*)(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose3D &, class mrpt::tfest::TMatchingPairListTempl<float> &, const struct mrpt::maps::TMatchingParams &, struct mrpt::maps::TMatchingExtraResults &) const) &mrpt::maps::CPointsMap::determineMatching3D, "C++: mrpt::maps::CPointsMap::determineMatching3D(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose3D &, class mrpt::tfest::TMatchingPairListTempl<float> &, const struct mrpt::maps::TMatchingParams &, struct mrpt::maps::TMatchingExtraResults &) const --> void", pybind11::arg("otherMap"), pybind11::arg("otherMapPose"), pybind11::arg("correspondences"), pybind11::arg("params"), pybind11::arg("extraResults"));
		cl.def("compute3DMatchingRatio", (float (mrpt::maps::CPointsMap::*)(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose3D &, const struct mrpt::maps::TMatchingRatioParams &) const) &mrpt::maps::CPointsMap::compute3DMatchingRatio, "C++: mrpt::maps::CPointsMap::compute3DMatchingRatio(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose3D &, const struct mrpt::maps::TMatchingRatioParams &) const --> float", pybind11::arg("otherMap"), pybind11::arg("otherMapPose"), pybind11::arg("params"));
		cl.def("compute3DDistanceToMesh", (void (mrpt::maps::CPointsMap::*)(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose3D &, float, class mrpt::tfest::TMatchingPairListTempl<float> &, float &)) &mrpt::maps::CPointsMap::compute3DDistanceToMesh, "Computes the matchings between this and another 3D points map.\n	   This method matches each point in the other map with the centroid of the\n	 3 closest points in 3D\n	   from this map (if the distance is below a defined threshold).\n \n\n					  [IN] The other map to compute the\n	 matching with.\n \n\n				  [IN] The pose of the other map as seen\n	 from \"this\".\n \n\n   [IN] Maximum 2D linear distance\n	 between two points to be matched.\n \n\n			  [OUT] The detected matchings pairs.\n \n\n		  [OUT] The ratio [0,1] of points in\n	 otherMap with at least one correspondence.\n\n \n determineMatching3D\n\nC++: mrpt::maps::CPointsMap::compute3DDistanceToMesh(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose3D &, float, class mrpt::tfest::TMatchingPairListTempl<float> &, float &) --> void", pybind11::arg("otherMap2"), pybind11::arg("otherMapPose"), pybind11::arg("maxDistForCorrespondence"), pybind11::arg("correspondences"), pybind11::arg("correspondencesRatio"));
		cl.def("changeCoordinatesReference", (void (mrpt::maps::CPointsMap::*)(const class mrpt::poses::CPose2D &)) &mrpt::maps::CPointsMap::changeCoordinatesReference, "Replace each point \n by \n (pose\n compounding operator).\n\nC++: mrpt::maps::CPointsMap::changeCoordinatesReference(const class mrpt::poses::CPose2D &) --> void", pybind11::arg("b"));
		cl.def("changeCoordinatesReference", (void (mrpt::maps::CPointsMap::*)(const class mrpt::poses::CPose3D &)) &mrpt::maps::CPointsMap::changeCoordinatesReference, "Replace each point \n by \n (pose\n compounding operator).\n\nC++: mrpt::maps::CPointsMap::changeCoordinatesReference(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("b"));
		cl.def("changeCoordinatesReference", (void (mrpt::maps::CPointsMap::*)(const class mrpt::maps::CPointsMap &, const class mrpt::poses::CPose3D &)) &mrpt::maps::CPointsMap::changeCoordinatesReference, "Copy all the points from \"other\" map to \"this\", replacing each point \n\n\n by \n (pose compounding operator).\n\nC++: mrpt::maps::CPointsMap::changeCoordinatesReference(const class mrpt::maps::CPointsMap &, const class mrpt::poses::CPose3D &) --> void", pybind11::arg("other"), pybind11::arg("b"));
		cl.def("isEmpty", (bool (mrpt::maps::CPointsMap::*)() const) &mrpt::maps::CPointsMap::isEmpty, "Returns true if the map is empty/no observation has been inserted.\n\nC++: mrpt::maps::CPointsMap::isEmpty() const --> bool");
		cl.def("empty", (bool (mrpt::maps::CPointsMap::*)() const) &mrpt::maps::CPointsMap::empty, "STL-like method to check whether the map is empty: \n\nC++: mrpt::maps::CPointsMap::empty() const --> bool");
		cl.def("getVisualizationInto", (void (mrpt::maps::CPointsMap::*)(class mrpt::opengl::CSetOfObjects &) const) &mrpt::maps::CPointsMap::getVisualizationInto, "Returns a 3D object representing the map.\n  The color of the points is controlled by renderOptions\n\nC++: mrpt::maps::CPointsMap::getVisualizationInto(class mrpt::opengl::CSetOfObjects &) const --> void", pybind11::arg("outObj"));
		cl.def("getLargestDistanceFromOrigin", (float (mrpt::maps::CPointsMap::*)() const) &mrpt::maps::CPointsMap::getLargestDistanceFromOrigin, "This method returns the largest distance from the origin to any of the\n points, such as a sphere centered at the origin with this radius cover\n ALL the points in the map (the results are buffered, such as, if the map\n is not modified, the second call will be much faster than the first one).\n\nC++: mrpt::maps::CPointsMap::getLargestDistanceFromOrigin() const --> float");
		cl.def("getLargestDistanceFromOriginNoRecompute", (float (mrpt::maps::CPointsMap::*)(bool &) const) &mrpt::maps::CPointsMap::getLargestDistanceFromOriginNoRecompute, "Like  but returns in \n = false if the distance was not already computed, skipping its\n computation then, unlike getLargestDistanceFromOrigin() \n\nC++: mrpt::maps::CPointsMap::getLargestDistanceFromOriginNoRecompute(bool &) const --> float", pybind11::arg("output_is_valid"));
		cl.def("boundingBox", (struct mrpt::math::TBoundingBox_<float> (mrpt::maps::CPointsMap::*)() const) &mrpt::maps::CPointsMap::boundingBox, "Computes the bounding box of all the points, or (0,0 ,0,0, 0,0) if there\n are no points.\n  Results are cached unless the map is somehow modified to avoid repeated\n calculations.\n\nC++: mrpt::maps::CPointsMap::boundingBox() const --> struct mrpt::math::TBoundingBox_<float>");
		cl.def("extractCylinder", (void (mrpt::maps::CPointsMap::*)(const struct mrpt::math::TPoint2D_<double> &, const double, const double, const double, class mrpt::maps::CPointsMap *)) &mrpt::maps::CPointsMap::extractCylinder, "Extracts the points in the map within a cylinder in 3D defined the\n provided radius and zmin/zmax values.\n\nC++: mrpt::maps::CPointsMap::extractCylinder(const struct mrpt::math::TPoint2D_<double> &, const double, const double, const double, class mrpt::maps::CPointsMap *) --> void", pybind11::arg("center"), pybind11::arg("radius"), pybind11::arg("zmin"), pybind11::arg("zmax"), pybind11::arg("outMap"));
		cl.def("extractPoints", [](mrpt::maps::CPointsMap &o, const struct mrpt::math::TPoint3D_<double> & a0, const struct mrpt::math::TPoint3D_<double> & a1, class mrpt::maps::CPointsMap * a2) -> void { return o.extractPoints(a0, a1, a2); }, "", pybind11::arg("corner1"), pybind11::arg("corner2"), pybind11::arg("outMap"));
		cl.def("extractPoints", [](mrpt::maps::CPointsMap &o, const struct mrpt::math::TPoint3D_<double> & a0, const struct mrpt::math::TPoint3D_<double> & a1, class mrpt::maps::CPointsMap * a2, double const & a3) -> void { return o.extractPoints(a0, a1, a2, a3); }, "", pybind11::arg("corner1"), pybind11::arg("corner2"), pybind11::arg("outMap"), pybind11::arg("R"));
		cl.def("extractPoints", [](mrpt::maps::CPointsMap &o, const struct mrpt::math::TPoint3D_<double> & a0, const struct mrpt::math::TPoint3D_<double> & a1, class mrpt::maps::CPointsMap * a2, double const & a3, double const & a4) -> void { return o.extractPoints(a0, a1, a2, a3, a4); }, "", pybind11::arg("corner1"), pybind11::arg("corner2"), pybind11::arg("outMap"), pybind11::arg("R"), pybind11::arg("G"));
		cl.def("extractPoints", (void (mrpt::maps::CPointsMap::*)(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &, class mrpt::maps::CPointsMap *, double, double, double)) &mrpt::maps::CPointsMap::extractPoints, "Extracts the points in the map within the area defined by two corners.\n  The points are coloured according the R,G,B input data.\n\nC++: mrpt::maps::CPointsMap::extractPoints(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &, class mrpt::maps::CPointsMap *, double, double, double) --> void", pybind11::arg("corner1"), pybind11::arg("corner2"), pybind11::arg("outMap"), pybind11::arg("R"), pybind11::arg("G"), pybind11::arg("B"));
		cl.def("enableFilterByHeight", [](mrpt::maps::CPointsMap &o) -> void { return o.enableFilterByHeight(); }, "");
		cl.def("enableFilterByHeight", (void (mrpt::maps::CPointsMap::*)(bool)) &mrpt::maps::CPointsMap::enableFilterByHeight, "Enable/disable the filter-by-height functionality \n\n setHeightFilterLevels \n\n Default upon construction is disabled. \n\nC++: mrpt::maps::CPointsMap::enableFilterByHeight(bool) --> void", pybind11::arg("enable"));
		cl.def("isFilterByHeightEnabled", (bool (mrpt::maps::CPointsMap::*)() const) &mrpt::maps::CPointsMap::isFilterByHeightEnabled, "Return whether filter-by-height is enabled \n enableFilterByHeight \n\nC++: mrpt::maps::CPointsMap::isFilterByHeightEnabled() const --> bool");
		cl.def("setHeightFilterLevels", (void (mrpt::maps::CPointsMap::*)(const double, const double)) &mrpt::maps::CPointsMap::setHeightFilterLevels, "Set the min/max Z levels for points to be actually inserted in the map\n (only if  was called before). \n\nC++: mrpt::maps::CPointsMap::setHeightFilterLevels(const double, const double) --> void", pybind11::arg("_z_min"), pybind11::arg("_z_max"));
		cl.def("getHeightFilterLevels", (void (mrpt::maps::CPointsMap::*)(double &, double &) const) &mrpt::maps::CPointsMap::getHeightFilterLevels, "Get the min/max Z levels for points to be actually inserted in the map\n \n\n enableFilterByHeight, setHeightFilterLevels \n\nC++: mrpt::maps::CPointsMap::getHeightFilterLevels(double &, double &) const --> void", pybind11::arg("_z_min"), pybind11::arg("_z_max"));
		cl.def("internal_computeObservationLikelihood", (double (mrpt::maps::CPointsMap::*)(const class mrpt::obs::CObservation &, const class mrpt::poses::CPose3D &) const) &mrpt::maps::CPointsMap::internal_computeObservationLikelihood, "@} \n\nC++: mrpt::maps::CPointsMap::internal_computeObservationLikelihood(const class mrpt::obs::CObservation &, const class mrpt::poses::CPose3D &) const --> double", pybind11::arg("obs"), pybind11::arg("takenFrom"));
		cl.def("internal_computeObservationLikelihoodPointCloud3D", (double (mrpt::maps::CPointsMap::*)(const class mrpt::poses::CPose3D &, const float *, const float *, const float *, const size_t) const) &mrpt::maps::CPointsMap::internal_computeObservationLikelihoodPointCloud3D, "C++: mrpt::maps::CPointsMap::internal_computeObservationLikelihoodPointCloud3D(const class mrpt::poses::CPose3D &, const float *, const float *, const float *, const size_t) const --> double", pybind11::arg("pc_in_map"), pybind11::arg("xs"), pybind11::arg("ys"), pybind11::arg("zs"), pybind11::arg("num_pts"));
		cl.def("kdtree_get_point_count", (size_t (mrpt::maps::CPointsMap::*)() const) &mrpt::maps::CPointsMap::kdtree_get_point_count, "Must return the number of data points\n\nC++: mrpt::maps::CPointsMap::kdtree_get_point_count() const --> size_t");
		cl.def("kdtree_get_pt", (float (mrpt::maps::CPointsMap::*)(size_t, int) const) &mrpt::maps::CPointsMap::kdtree_get_pt, "Returns the dim'th component of the idx'th point in the class:\n\nC++: mrpt::maps::CPointsMap::kdtree_get_pt(size_t, int) const --> float", pybind11::arg("idx"), pybind11::arg("dim"));
		cl.def("kdtree_distance", (float (mrpt::maps::CPointsMap::*)(const float *, size_t, size_t) const) &mrpt::maps::CPointsMap::kdtree_distance, "Returns the distance between the vector \"p1[0:size-1]\" and the data\n point with index \"idx_p2\" stored in the class:\n\nC++: mrpt::maps::CPointsMap::kdtree_distance(const float *, size_t, size_t) const --> float", pybind11::arg("p1"), pybind11::arg("idx_p2"), pybind11::arg("size"));
		cl.def("mark_as_modified", (void (mrpt::maps::CPointsMap::*)() const) &mrpt::maps::CPointsMap::mark_as_modified, "Users normally don't need to call this. Called by this class or children\n classes, set m_largestDistanceFromOriginIsUpdated=false, invalidates the\n kd-tree cache, and such. \n\nC++: mrpt::maps::CPointsMap::mark_as_modified() const --> void");
		cl.def("asString", (std::string (mrpt::maps::CPointsMap::*)() const) &mrpt::maps::CPointsMap::asString, "Returns a short description of the map. \n\nC++: mrpt::maps::CPointsMap::asString() const --> std::string");
		cl.def("nn_prepare_for_2d_queries", (void (mrpt::maps::CPointsMap::*)() const) &mrpt::maps::CPointsMap::nn_prepare_for_2d_queries, "		@{ \n\nC++: mrpt::maps::CPointsMap::nn_prepare_for_2d_queries() const --> void");
		cl.def("nn_prepare_for_3d_queries", (void (mrpt::maps::CPointsMap::*)() const) &mrpt::maps::CPointsMap::nn_prepare_for_3d_queries, "C++: mrpt::maps::CPointsMap::nn_prepare_for_3d_queries() const --> void");
		cl.def("nn_has_indices_or_ids", (bool (mrpt::maps::CPointsMap::*)() const) &mrpt::maps::CPointsMap::nn_has_indices_or_ids, "C++: mrpt::maps::CPointsMap::nn_has_indices_or_ids() const --> bool");
		cl.def("nn_index_count", (size_t (mrpt::maps::CPointsMap::*)() const) &mrpt::maps::CPointsMap::nn_index_count, "C++: mrpt::maps::CPointsMap::nn_index_count() const --> size_t");
		cl.def("nn_single_search", (bool (mrpt::maps::CPointsMap::*)(const struct mrpt::math::TPoint3D_<float> &, struct mrpt::math::TPoint3D_<float> &, float &, uint64_t &) const) &mrpt::maps::CPointsMap::nn_single_search, "C++: mrpt::maps::CPointsMap::nn_single_search(const struct mrpt::math::TPoint3D_<float> &, struct mrpt::math::TPoint3D_<float> &, float &, uint64_t &) const --> bool", pybind11::arg("query"), pybind11::arg("result"), pybind11::arg("out_dist_sqr"), pybind11::arg("resultIndexOrID"));
		cl.def("nn_single_search", (bool (mrpt::maps::CPointsMap::*)(const struct mrpt::math::TPoint2D_<float> &, struct mrpt::math::TPoint2D_<float> &, float &, uint64_t &) const) &mrpt::maps::CPointsMap::nn_single_search, "C++: mrpt::maps::CPointsMap::nn_single_search(const struct mrpt::math::TPoint2D_<float> &, struct mrpt::math::TPoint2D_<float> &, float &, uint64_t &) const --> bool", pybind11::arg("query"), pybind11::arg("result"), pybind11::arg("out_dist_sqr"), pybind11::arg("resultIndexOrID"));

		{ // mrpt::maps::CPointsMap::TInsertionOptions file:mrpt/maps/CPointsMap.h line:228
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CPointsMap::TInsertionOptions, std::shared_ptr<mrpt::maps::CPointsMap::TInsertionOptions>, PyCallBack_mrpt_maps_CPointsMap_TInsertionOptions, mrpt::config::CLoadableOptions> cl(enclosing_class, "TInsertionOptions", "With this struct options are provided to the observation insertion\n process.\n \n\n CObservation::insertIntoPointsMap");
			cl.def( pybind11::init( [](){ return new mrpt::maps::CPointsMap::TInsertionOptions(); }, [](){ return new PyCallBack_mrpt_maps_CPointsMap_TInsertionOptions(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_maps_CPointsMap_TInsertionOptions const &o){ return new PyCallBack_mrpt_maps_CPointsMap_TInsertionOptions(o); } ) );
			cl.def( pybind11::init( [](mrpt::maps::CPointsMap::TInsertionOptions const &o){ return new mrpt::maps::CPointsMap::TInsertionOptions(o); } ) );
			cl.def_readwrite("minDistBetweenLaserPoints", &mrpt::maps::CPointsMap::TInsertionOptions::minDistBetweenLaserPoints);
			cl.def_readwrite("addToExistingPointsMap", &mrpt::maps::CPointsMap::TInsertionOptions::addToExistingPointsMap);
			cl.def_readwrite("also_interpolate", &mrpt::maps::CPointsMap::TInsertionOptions::also_interpolate);
			cl.def_readwrite("disableDeletion", &mrpt::maps::CPointsMap::TInsertionOptions::disableDeletion);
			cl.def_readwrite("fuseWithExisting", &mrpt::maps::CPointsMap::TInsertionOptions::fuseWithExisting);
			cl.def_readwrite("isPlanarMap", &mrpt::maps::CPointsMap::TInsertionOptions::isPlanarMap);
			cl.def_readwrite("horizontalTolerance", &mrpt::maps::CPointsMap::TInsertionOptions::horizontalTolerance);
			cl.def_readwrite("maxDistForInterpolatePoints", &mrpt::maps::CPointsMap::TInsertionOptions::maxDistForInterpolatePoints);
			cl.def_readwrite("insertInvalidPoints", &mrpt::maps::CPointsMap::TInsertionOptions::insertInvalidPoints);
			cl.def("loadFromConfigFile", (void (mrpt::maps::CPointsMap::TInsertionOptions::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::maps::CPointsMap::TInsertionOptions::loadFromConfigFile, "C++: mrpt::maps::CPointsMap::TInsertionOptions::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
			cl.def("writeToStream", (void (mrpt::maps::CPointsMap::TInsertionOptions::*)(class mrpt::serialization::CArchive &) const) &mrpt::maps::CPointsMap::TInsertionOptions::writeToStream, "Binary dump to stream - for usage in derived classes' serialization\n\nC++: mrpt::maps::CPointsMap::TInsertionOptions::writeToStream(class mrpt::serialization::CArchive &) const --> void", pybind11::arg("out"));
			cl.def("readFromStream", (void (mrpt::maps::CPointsMap::TInsertionOptions::*)(class mrpt::serialization::CArchive &)) &mrpt::maps::CPointsMap::TInsertionOptions::readFromStream, "Binary dump to stream - for usage in derived classes' serialization\n\nC++: mrpt::maps::CPointsMap::TInsertionOptions::readFromStream(class mrpt::serialization::CArchive &) --> void", pybind11::arg("in"));
			cl.def("assign", (struct mrpt::maps::CPointsMap::TInsertionOptions & (mrpt::maps::CPointsMap::TInsertionOptions::*)(const struct mrpt::maps::CPointsMap::TInsertionOptions &)) &mrpt::maps::CPointsMap::TInsertionOptions::operator=, "C++: mrpt::maps::CPointsMap::TInsertionOptions::operator=(const struct mrpt::maps::CPointsMap::TInsertionOptions &) --> struct mrpt::maps::CPointsMap::TInsertionOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::maps::CPointsMap::TLikelihoodOptions file:mrpt/maps/CPointsMap.h line:285
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CPointsMap::TLikelihoodOptions, std::shared_ptr<mrpt::maps::CPointsMap::TLikelihoodOptions>, PyCallBack_mrpt_maps_CPointsMap_TLikelihoodOptions, mrpt::config::CLoadableOptions> cl(enclosing_class, "TLikelihoodOptions", "Options used when evaluating \"computeObservationLikelihood\" in the\n derived classes.\n \n\n CObservation::computeObservationLikelihood");
			cl.def( pybind11::init( [](){ return new mrpt::maps::CPointsMap::TLikelihoodOptions(); }, [](){ return new PyCallBack_mrpt_maps_CPointsMap_TLikelihoodOptions(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_maps_CPointsMap_TLikelihoodOptions const &o){ return new PyCallBack_mrpt_maps_CPointsMap_TLikelihoodOptions(o); } ) );
			cl.def( pybind11::init( [](mrpt::maps::CPointsMap::TLikelihoodOptions const &o){ return new mrpt::maps::CPointsMap::TLikelihoodOptions(o); } ) );
			cl.def_readwrite("sigma_dist", &mrpt::maps::CPointsMap::TLikelihoodOptions::sigma_dist);
			cl.def_readwrite("max_corr_distance", &mrpt::maps::CPointsMap::TLikelihoodOptions::max_corr_distance);
			cl.def_readwrite("decimation", &mrpt::maps::CPointsMap::TLikelihoodOptions::decimation);
			cl.def("loadFromConfigFile", (void (mrpt::maps::CPointsMap::TLikelihoodOptions::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::maps::CPointsMap::TLikelihoodOptions::loadFromConfigFile, "C++: mrpt::maps::CPointsMap::TLikelihoodOptions::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
			cl.def("writeToStream", (void (mrpt::maps::CPointsMap::TLikelihoodOptions::*)(class mrpt::serialization::CArchive &) const) &mrpt::maps::CPointsMap::TLikelihoodOptions::writeToStream, "Binary dump to stream - for usage in derived classes' serialization\n\nC++: mrpt::maps::CPointsMap::TLikelihoodOptions::writeToStream(class mrpt::serialization::CArchive &) const --> void", pybind11::arg("out"));
			cl.def("readFromStream", (void (mrpt::maps::CPointsMap::TLikelihoodOptions::*)(class mrpt::serialization::CArchive &)) &mrpt::maps::CPointsMap::TLikelihoodOptions::readFromStream, "Binary dump to stream - for usage in derived classes' serialization\n\nC++: mrpt::maps::CPointsMap::TLikelihoodOptions::readFromStream(class mrpt::serialization::CArchive &) --> void", pybind11::arg("in"));
			cl.def("assign", (struct mrpt::maps::CPointsMap::TLikelihoodOptions & (mrpt::maps::CPointsMap::TLikelihoodOptions::*)(const struct mrpt::maps::CPointsMap::TLikelihoodOptions &)) &mrpt::maps::CPointsMap::TLikelihoodOptions::operator=, "C++: mrpt::maps::CPointsMap::TLikelihoodOptions::operator=(const struct mrpt::maps::CPointsMap::TLikelihoodOptions &) --> struct mrpt::maps::CPointsMap::TLikelihoodOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::maps::CPointsMap::TRenderOptions file:mrpt/maps/CPointsMap.h line:318
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CPointsMap::TRenderOptions, std::shared_ptr<mrpt::maps::CPointsMap::TRenderOptions>, PyCallBack_mrpt_maps_CPointsMap_TRenderOptions, mrpt::config::CLoadableOptions> cl(enclosing_class, "TRenderOptions", "Rendering options, used in getAs3DObject()");
			cl.def( pybind11::init( [](){ return new mrpt::maps::CPointsMap::TRenderOptions(); }, [](){ return new PyCallBack_mrpt_maps_CPointsMap_TRenderOptions(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_maps_CPointsMap_TRenderOptions const &o){ return new PyCallBack_mrpt_maps_CPointsMap_TRenderOptions(o); } ) );
			cl.def( pybind11::init( [](mrpt::maps::CPointsMap::TRenderOptions const &o){ return new mrpt::maps::CPointsMap::TRenderOptions(o); } ) );
			cl.def_readwrite("point_size", &mrpt::maps::CPointsMap::TRenderOptions::point_size);
			cl.def_readwrite("color", &mrpt::maps::CPointsMap::TRenderOptions::color);
			cl.def_readwrite("colormap", &mrpt::maps::CPointsMap::TRenderOptions::colormap);
			cl.def("loadFromConfigFile", (void (mrpt::maps::CPointsMap::TRenderOptions::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::maps::CPointsMap::TRenderOptions::loadFromConfigFile, "C++: mrpt::maps::CPointsMap::TRenderOptions::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
			cl.def("writeToStream", (void (mrpt::maps::CPointsMap::TRenderOptions::*)(class mrpt::serialization::CArchive &) const) &mrpt::maps::CPointsMap::TRenderOptions::writeToStream, "Binary dump to stream - used in derived classes' serialization \n\nC++: mrpt::maps::CPointsMap::TRenderOptions::writeToStream(class mrpt::serialization::CArchive &) const --> void", pybind11::arg("out"));
			cl.def("readFromStream", (void (mrpt::maps::CPointsMap::TRenderOptions::*)(class mrpt::serialization::CArchive &)) &mrpt::maps::CPointsMap::TRenderOptions::readFromStream, "Binary dump to stream - used in derived classes' serialization \n\nC++: mrpt::maps::CPointsMap::TRenderOptions::readFromStream(class mrpt::serialization::CArchive &) --> void", pybind11::arg("in"));
			cl.def("assign", (struct mrpt::maps::CPointsMap::TRenderOptions & (mrpt::maps::CPointsMap::TRenderOptions::*)(const struct mrpt::maps::CPointsMap::TRenderOptions &)) &mrpt::maps::CPointsMap::TRenderOptions::operator=, "C++: mrpt::maps::CPointsMap::TRenderOptions::operator=(const struct mrpt::maps::CPointsMap::TRenderOptions &) --> struct mrpt::maps::CPointsMap::TRenderOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
}
