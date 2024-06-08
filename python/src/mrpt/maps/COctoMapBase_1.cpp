#include <ios>
#include <istream>
#include <iterator>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/aligned_allocator.h>
#include <mrpt/img/TColor.h>
#include <mrpt/img/color_maps.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/COctoMap.h>
#include <mrpt/maps/COctoMapBase.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/metric_map_types.h>
#include <mrpt/math/CMatrixFixed.h>
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
#include <mrpt/opengl/COctoMapVoxels.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/poses/CPointPDF.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DQuatPDF.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ostream>
#include <sstream> // __str__
#include <streambuf>
#include <string>
#include <type_traits>
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

// mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions file:mrpt/maps/COctoMapBase.h line:67
struct PyCallBack_mrpt_maps_COctoMapBase_octomap_OcTree_octomap_OcTreeNode__TInsertionOptions : public mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions {
	using mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::TInsertionOptions;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions *>(this), "loadFromConfigFile");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions *>(this), "saveToConfigFile");
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

// mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TLikelihoodOptions file:mrpt/maps/COctoMapBase.h line:213
struct PyCallBack_mrpt_maps_COctoMapBase_octomap_OcTree_octomap_OcTreeNode__TLikelihoodOptions : public mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TLikelihoodOptions {
	using mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TLikelihoodOptions::TLikelihoodOptions;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TLikelihoodOptions *>(this), "loadFromConfigFile");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TLikelihoodOptions *>(this), "saveToConfigFile");
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

void bind_mrpt_maps_COctoMapBase_1(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::maps::COctoMapBase file:mrpt/maps/COctoMapBase.h line:39
		pybind11::class_<mrpt::maps::COctoMapBase<octomap::OcTree,octomap::OcTreeNode>, std::shared_ptr<mrpt::maps::COctoMapBase<octomap::OcTree,octomap::OcTreeNode>>, mrpt::maps::CMetricMap> cl(M("mrpt::maps"), "COctoMapBase_octomap_OcTree_octomap_OcTreeNode_t", "");
		cl.def_readwrite("insertionOptions", &mrpt::maps::COctoMapBase<octomap::OcTree,octomap::OcTreeNode>::insertionOptions);
		cl.def_readwrite("likelihoodOptions", &mrpt::maps::COctoMapBase<octomap::OcTree,octomap::OcTreeNode>::likelihoodOptions);
		cl.def_readwrite("renderingOptions", &mrpt::maps::COctoMapBase<octomap::OcTree,octomap::OcTreeNode>::renderingOptions);
		cl.def("asString", (std::string (mrpt::maps::COctoMapBase<octomap::OcTree,octomap::OcTreeNode>::*)() const) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::asString, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::asString() const --> std::string");
		cl.def("saveMetricMapRepresentationToFile", (void (mrpt::maps::COctoMapBase<octomap::OcTree,octomap::OcTreeNode>::*)(const std::string &) const) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::saveMetricMapRepresentationToFile, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::saveMetricMapRepresentationToFile(const std::string &) const --> void", pybind11::arg("filNamePrefix"));
		cl.def("getVisualizationInto", (void (mrpt::maps::COctoMapBase<octomap::OcTree,octomap::OcTreeNode>::*)(class mrpt::opengl::CSetOfObjects &) const) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::getVisualizationInto, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::getVisualizationInto(class mrpt::opengl::CSetOfObjects &) const --> void", pybind11::arg("o"));
		cl.def("getAsOctoMapVoxels", (void (mrpt::maps::COctoMapBase<octomap::OcTree,octomap::OcTreeNode>::*)(class mrpt::opengl::COctoMapVoxels &) const) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::getAsOctoMapVoxels, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::getAsOctoMapVoxels(class mrpt::opengl::COctoMapVoxels &) const --> void", pybind11::arg("gl_obj"));
		cl.def("getPointOccupancy", (bool (mrpt::maps::COctoMapBase<octomap::OcTree,octomap::OcTreeNode>::*)(const float, const float, const float, double &) const) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::getPointOccupancy, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::getPointOccupancy(const float, const float, const float, double &) const --> bool", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("prob_occupancy"));
		cl.def("insertPointCloud", (void (mrpt::maps::COctoMapBase<octomap::OcTree,octomap::OcTreeNode>::*)(const class mrpt::maps::CPointsMap &, const float, const float, const float)) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::insertPointCloud, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::insertPointCloud(const class mrpt::maps::CPointsMap &, const float, const float, const float) --> void", pybind11::arg("ptMap"), pybind11::arg("sensor_x"), pybind11::arg("sensor_y"), pybind11::arg("sensor_z"));
		cl.def("castRay", [](mrpt::maps::COctoMapBase<octomap::OcTree,octomap::OcTreeNode> const &o, const struct mrpt::math::TPoint3D_<double> & a0, const struct mrpt::math::TPoint3D_<double> & a1, struct mrpt::math::TPoint3D_<double> & a2) -> bool { return o.castRay(a0, a1, a2); }, "", pybind11::arg("origin"), pybind11::arg("direction"), pybind11::arg("end"));
		cl.def("castRay", [](mrpt::maps::COctoMapBase<octomap::OcTree,octomap::OcTreeNode> const &o, const struct mrpt::math::TPoint3D_<double> & a0, const struct mrpt::math::TPoint3D_<double> & a1, struct mrpt::math::TPoint3D_<double> & a2, bool const & a3) -> bool { return o.castRay(a0, a1, a2, a3); }, "", pybind11::arg("origin"), pybind11::arg("direction"), pybind11::arg("end"), pybind11::arg("ignoreUnknownCells"));
		cl.def("castRay", (bool (mrpt::maps::COctoMapBase<octomap::OcTree,octomap::OcTreeNode>::*)(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &, struct mrpt::math::TPoint3D_<double> &, bool, double) const) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::castRay, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::castRay(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &, struct mrpt::math::TPoint3D_<double> &, bool, double) const --> bool", pybind11::arg("origin"), pybind11::arg("direction"), pybind11::arg("end"), pybind11::arg("ignoreUnknownCells"), pybind11::arg("maxRange"));
		cl.def("setOccupancyThres", (void (mrpt::maps::COctoMapBase<octomap::OcTree,octomap::OcTreeNode>::*)(double)) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::setOccupancyThres, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::setOccupancyThres(double) --> void", pybind11::arg("prob"));
		cl.def("setProbHit", (void (mrpt::maps::COctoMapBase<octomap::OcTree,octomap::OcTreeNode>::*)(double)) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::setProbHit, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::setProbHit(double) --> void", pybind11::arg("prob"));
		cl.def("setProbMiss", (void (mrpt::maps::COctoMapBase<octomap::OcTree,octomap::OcTreeNode>::*)(double)) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::setProbMiss, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::setProbMiss(double) --> void", pybind11::arg("prob"));
		cl.def("setClampingThresMin", (void (mrpt::maps::COctoMapBase<octomap::OcTree,octomap::OcTreeNode>::*)(double)) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::setClampingThresMin, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::setClampingThresMin(double) --> void", pybind11::arg("thresProb"));
		cl.def("setClampingThresMax", (void (mrpt::maps::COctoMapBase<octomap::OcTree,octomap::OcTreeNode>::*)(double)) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::setClampingThresMax, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::setClampingThresMax(double) --> void", pybind11::arg("thresProb"));
		cl.def("getOccupancyThres", (double (mrpt::maps::COctoMapBase<octomap::OcTree,octomap::OcTreeNode>::*)() const) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::getOccupancyThres, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::getOccupancyThres() const --> double");
		cl.def("getOccupancyThresLog", (float (mrpt::maps::COctoMapBase<octomap::OcTree,octomap::OcTreeNode>::*)() const) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::getOccupancyThresLog, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::getOccupancyThresLog() const --> float");
		cl.def("getProbHit", (double (mrpt::maps::COctoMapBase<octomap::OcTree,octomap::OcTreeNode>::*)() const) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::getProbHit, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::getProbHit() const --> double");
		cl.def("getProbHitLog", (float (mrpt::maps::COctoMapBase<octomap::OcTree,octomap::OcTreeNode>::*)() const) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::getProbHitLog, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::getProbHitLog() const --> float");
		cl.def("getProbMiss", (double (mrpt::maps::COctoMapBase<octomap::OcTree,octomap::OcTreeNode>::*)() const) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::getProbMiss, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::getProbMiss() const --> double");
		cl.def("getProbMissLog", (float (mrpt::maps::COctoMapBase<octomap::OcTree,octomap::OcTreeNode>::*)() const) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::getProbMissLog, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::getProbMissLog() const --> float");
		cl.def("getClampingThresMin", (double (mrpt::maps::COctoMapBase<octomap::OcTree,octomap::OcTreeNode>::*)() const) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::getClampingThresMin, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::getClampingThresMin() const --> double");
		cl.def("getClampingThresMinLog", (float (mrpt::maps::COctoMapBase<octomap::OcTree,octomap::OcTreeNode>::*)() const) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::getClampingThresMinLog, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::getClampingThresMinLog() const --> float");
		cl.def("getClampingThresMax", (double (mrpt::maps::COctoMapBase<octomap::OcTree,octomap::OcTreeNode>::*)() const) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::getClampingThresMax, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::getClampingThresMax() const --> double");
		cl.def("getClampingThresMaxLog", (float (mrpt::maps::COctoMapBase<octomap::OcTree,octomap::OcTreeNode>::*)() const) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::getClampingThresMaxLog, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::getClampingThresMaxLog() const --> float");
		cl.def("assign", (class mrpt::maps::COctoMapBase<class octomap::OcTree, class octomap::OcTreeNode> & (mrpt::maps::COctoMapBase<octomap::OcTree,octomap::OcTreeNode>::*)(const class mrpt::maps::COctoMapBase<class octomap::OcTree, class octomap::OcTreeNode> &)) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::operator=, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::operator=(const class mrpt::maps::COctoMapBase<class octomap::OcTree, class octomap::OcTreeNode> &) --> class mrpt::maps::COctoMapBase<class octomap::OcTree, class octomap::OcTreeNode> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		cl.def_readwrite("genericMapParams", &mrpt::maps::CMetricMap::genericMapParams);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::maps::CMetricMap::*)() const) &mrpt::maps::CMetricMap::GetRuntimeClass, "C++: mrpt::maps::CMetricMap::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::maps::CMetricMap::GetRuntimeClassIdStatic, "C++: mrpt::maps::CMetricMap::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("clear", (void (mrpt::maps::CMetricMap::*)()) &mrpt::maps::CMetricMap::clear, "Erase all the contents of the map \n\nC++: mrpt::maps::CMetricMap::clear() --> void");
		cl.def("isEmpty", (bool (mrpt::maps::CMetricMap::*)() const) &mrpt::maps::CMetricMap::isEmpty, "Returns true if the map is empty/no observation has been inserted.\n\nC++: mrpt::maps::CMetricMap::isEmpty() const --> bool");
		cl.def("boundingBox", (struct mrpt::math::TBoundingBox_<float> (mrpt::maps::CMetricMap::*)() const) &mrpt::maps::CMetricMap::boundingBox, "Returns the bounding box of the metric map, or (0,0,0)-(0,0,0) (the\n default value of mrpt::math::TBoundingBoxf() if not implemented in the\n derived class or the map is empty.\n\nC++: mrpt::maps::CMetricMap::boundingBox() const --> struct mrpt::math::TBoundingBox_<float>");
		cl.def("loadFromSimpleMap", (void (mrpt::maps::CMetricMap::*)(const class mrpt::maps::CSimpleMap &)) &mrpt::maps::CMetricMap::loadFromSimpleMap, "Load the map contents from a CSimpleMap object, erasing all previous\n content of the map. This is done invoking `insertObservation()` for each\n observation at the mean 3D robot pose of each pose-observations pair in\n the CSimpleMap object.\n\n \n insertObservation, CSimpleMap\n \n\n std::exception Some internal steps in invoked methods can\n raise exceptions on invalid parameters, etc...\n\nC++: mrpt::maps::CMetricMap::loadFromSimpleMap(const class mrpt::maps::CSimpleMap &) --> void", pybind11::arg("Map"));
		cl.def("insertObs", [](mrpt::maps::CMetricMap &o, const class mrpt::obs::CObservation & a0) -> bool { return o.insertObs(a0); }, "", pybind11::arg("obs"));
		cl.def("insertObs", (bool (mrpt::maps::CMetricMap::*)(const class mrpt::obs::CObservation &, const class mrpt::poses::CPose3D *)) &mrpt::maps::CMetricMap::insertObs, "C++: mrpt::maps::CMetricMap::insertObs(const class mrpt::obs::CObservation &, const class mrpt::poses::CPose3D *) --> bool", pybind11::arg("obs"), pybind11::arg("robotPose"));
		cl.def("insertObs", [](mrpt::maps::CMetricMap &o, const class mrpt::obs::CSensoryFrame & a0) -> bool { return o.insertObs(a0); }, "", pybind11::arg("sf"));
		cl.def("insertObs", (bool (mrpt::maps::CMetricMap::*)(const class mrpt::obs::CSensoryFrame &, const class mrpt::poses::CPose3D *)) &mrpt::maps::CMetricMap::insertObs, "C++: mrpt::maps::CMetricMap::insertObs(const class mrpt::obs::CSensoryFrame &, const class mrpt::poses::CPose3D *) --> bool", pybind11::arg("sf"), pybind11::arg("robotPose"));
		cl.def("computeObservationLikelihood", (double (mrpt::maps::CMetricMap::*)(const class mrpt::obs::CObservation &, const class mrpt::poses::CPose3D &) const) &mrpt::maps::CMetricMap::computeObservationLikelihood, "Computes the log-likelihood of a given observation given an arbitrary\n robot 3D pose.\n See: \n\n \n The robot's pose the observation is supposed to be taken\n from.\n \n\n The observation.\n \n\n This method returns a log-likelihood.\n\n \n Used in particle filter algorithms, see: CMultiMetricMapPDF::update\n\nC++: mrpt::maps::CMetricMap::computeObservationLikelihood(const class mrpt::obs::CObservation &, const class mrpt::poses::CPose3D &) const --> double", pybind11::arg("obs"), pybind11::arg("takenFrom"));
		cl.def("canComputeObservationLikelihood", (bool (mrpt::maps::CMetricMap::*)(const class mrpt::obs::CObservation &) const) &mrpt::maps::CMetricMap::canComputeObservationLikelihood, "Returns true if this map is able to compute a sensible likelihood\n function for this observation (i.e. an occupancy grid map cannot with an\n image).\n See: \n\n \n The observation.\n \n\n computeObservationLikelihood,\n genericMapParams.enableObservationLikelihood\n\nC++: mrpt::maps::CMetricMap::canComputeObservationLikelihood(const class mrpt::obs::CObservation &) const --> bool", pybind11::arg("obs"));
		cl.def("computeObservationsLikelihood", (double (mrpt::maps::CMetricMap::*)(const class mrpt::obs::CSensoryFrame &, const class mrpt::poses::CPose3D &)) &mrpt::maps::CMetricMap::computeObservationsLikelihood, "Returns the sum of the log-likelihoods of each individual observation\n within a mrpt::obs::CSensoryFrame.\n See: \n\n \n The robot's pose the observation is supposed to be taken\n from.\n \n\n The set of observations in a CSensoryFrame.\n \n\n This method returns a log-likelihood.\n \n\n canComputeObservationsLikelihood\n\nC++: mrpt::maps::CMetricMap::computeObservationsLikelihood(const class mrpt::obs::CSensoryFrame &, const class mrpt::poses::CPose3D &) --> double", pybind11::arg("sf"), pybind11::arg("takenFrom"));
		cl.def("canComputeObservationsLikelihood", (bool (mrpt::maps::CMetricMap::*)(const class mrpt::obs::CSensoryFrame &) const) &mrpt::maps::CMetricMap::canComputeObservationsLikelihood, "Returns true if this map is able to compute a sensible likelihood\n function for this observation (i.e. an occupancy grid map cannot with an\n image).\n See: \n\n \n The observations.\n \n\n canComputeObservationLikelihood\n\nC++: mrpt::maps::CMetricMap::canComputeObservationsLikelihood(const class mrpt::obs::CSensoryFrame &) const --> bool", pybind11::arg("sf"));
		cl.def("determineMatching2D", (void (mrpt::maps::CMetricMap::*)(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose2D &, class mrpt::tfest::TMatchingPairListTempl<float> &, const struct mrpt::maps::TMatchingParams &, struct mrpt::maps::TMatchingExtraResults &) const) &mrpt::maps::CMetricMap::determineMatching2D, "Computes the matching between this and another 2D point map, which\nincludes finding:\n   - The set of points pairs in each map\n   - The mean squared distance between corresponding pairs.\n\n   The algorithm is:\n		- For each point in \"otherMap\":\n			- Transform the point according to otherMapPose\n			- Search with a KD-TREE the closest correspondences in \"this\"\nmap.\n			- Add to the set of candidate matchings, if it passes all the\nthresholds in params.\n\n   This method is the most time critical one into ICP-like algorithms.\n\n \n        [IN] The other map to compute the matching with.\n \n\n    [IN] The pose of the other map as seen from\n\"this\".\n \n\n          [IN] Parameters for the determination of\npairings.\n \n\n [OUT] The detected matchings pairs.\n \n\n    [OUT] Other results.\n \n\n compute3DMatchingRatio\n\nC++: mrpt::maps::CMetricMap::determineMatching2D(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose2D &, class mrpt::tfest::TMatchingPairListTempl<float> &, const struct mrpt::maps::TMatchingParams &, struct mrpt::maps::TMatchingExtraResults &) const --> void", pybind11::arg("otherMap"), pybind11::arg("otherMapPose"), pybind11::arg("correspondences"), pybind11::arg("params"), pybind11::arg("extraResults"));
		cl.def("determineMatching3D", (void (mrpt::maps::CMetricMap::*)(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose3D &, class mrpt::tfest::TMatchingPairListTempl<float> &, const struct mrpt::maps::TMatchingParams &, struct mrpt::maps::TMatchingExtraResults &) const) &mrpt::maps::CMetricMap::determineMatching3D, "Computes the matchings between this and another 3D points map - method\nused in 3D-ICP.\n  This method finds the set of point pairs in each map.\n\n  The method is the most time critical one into ICP-like algorithms.\n\n  The algorithm is:\n		- For each point in \"otherMap\":\n			- Transform the point according to otherMapPose\n			- Search with a KD-TREE the closest correspondences in \"this\"\nmap.\n			- Add to the set of candidate matchings, if it passes all the\nthresholds in params.\n\n \n        [IN] The other map to compute the matching with.\n \n\n    [IN] The pose of the other map as seen from\n\"this\".\n \n\n          [IN] Parameters for the determination of\npairings.\n \n\n [OUT] The detected matchings pairs.\n \n\n    [OUT] Other results.\n \n\n compute3DMatchingRatio\n\nC++: mrpt::maps::CMetricMap::determineMatching3D(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose3D &, class mrpt::tfest::TMatchingPairListTempl<float> &, const struct mrpt::maps::TMatchingParams &, struct mrpt::maps::TMatchingExtraResults &) const --> void", pybind11::arg("otherMap"), pybind11::arg("otherMapPose"), pybind11::arg("correspondences"), pybind11::arg("params"), pybind11::arg("extraResults"));
		cl.def("compute3DMatchingRatio", (float (mrpt::maps::CMetricMap::*)(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose3D &, const struct mrpt::maps::TMatchingRatioParams &) const) &mrpt::maps::CMetricMap::compute3DMatchingRatio, "Computes the ratio in [0,1] of correspondences between \"this\" and the\n \"otherMap\" map, whose 6D pose relative to \"this\" is \"otherMapPose\"\n   In the case of a multi-metric map, this returns the average between the\n maps. This method always return 0 for grid maps.\n \n\n      [IN] The other map to compute the matching with.\n \n\n  [IN] The 6D pose of the other map as seen from\n \"this\".\n \n\n        [IN] Matching parameters\n \n\n The matching ratio [0,1]\n \n\n determineMatching2D\n\nC++: mrpt::maps::CMetricMap::compute3DMatchingRatio(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose3D &, const struct mrpt::maps::TMatchingRatioParams &) const --> float", pybind11::arg("otherMap"), pybind11::arg("otherMapPose"), pybind11::arg("params"));
		cl.def("saveMetricMapRepresentationToFile", (void (mrpt::maps::CMetricMap::*)(const std::string &) const) &mrpt::maps::CMetricMap::saveMetricMapRepresentationToFile, "This virtual method saves the map to a file \"filNamePrefix\"+<\n some_file_extension >, as an image or in any other applicable way (Notice\n that other methods to save the map may be implemented in classes\n implementing this virtual interface). \n\nC++: mrpt::maps::CMetricMap::saveMetricMapRepresentationToFile(const std::string &) const --> void", pybind11::arg("filNamePrefix"));
		cl.def("auxParticleFilterCleanUp", (void (mrpt::maps::CMetricMap::*)()) &mrpt::maps::CMetricMap::auxParticleFilterCleanUp, "This method is called at the end of each \"prediction-update-map\n insertion\" cycle within\n \"mrpt::slam::CMetricMapBuilderRBPF::processActionObservation\".\n  This method should normally do nothing, but in some cases can be used\n to free auxiliary cached variables.\n\nC++: mrpt::maps::CMetricMap::auxParticleFilterCleanUp() --> void");
		cl.def("squareDistanceToClosestCorrespondence", (float (mrpt::maps::CMetricMap::*)(float, float) const) &mrpt::maps::CMetricMap::squareDistanceToClosestCorrespondence, "Returns the square distance from the 2D point (x0,y0) to the closest\n correspondence in the map. \n\nC++: mrpt::maps::CMetricMap::squareDistanceToClosestCorrespondence(float, float) const --> float", pybind11::arg("x0"), pybind11::arg("y0"));
		cl.def("getAsSimplePointsMap", (class mrpt::maps::CSimplePointsMap * (mrpt::maps::CMetricMap::*)()) &mrpt::maps::CMetricMap::getAsSimplePointsMap, "C++: mrpt::maps::CMetricMap::getAsSimplePointsMap() --> class mrpt::maps::CSimplePointsMap *", pybind11::return_value_policy::automatic);
		cl.def("assign", (class mrpt::maps::CMetricMap & (mrpt::maps::CMetricMap::*)(const class mrpt::maps::CMetricMap &)) &mrpt::maps::CMetricMap::operator=, "C++: mrpt::maps::CMetricMap::operator=(const class mrpt::maps::CMetricMap &) --> class mrpt::maps::CMetricMap &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions file:mrpt/maps/COctoMapBase.h line:67
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions, std::shared_ptr<mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions>, PyCallBack_mrpt_maps_COctoMapBase_octomap_OcTree_octomap_OcTreeNode__TInsertionOptions, mrpt::config::CLoadableOptions> cl(enclosing_class, "TInsertionOptions", "");
			cl.def( pybind11::init<class mrpt::maps::COctoMapBase<class octomap::OcTree, class octomap::OcTreeNode> &>(), pybind11::arg("parent") );

			cl.def( pybind11::init( [](){ return new mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions(); }, [](){ return new PyCallBack_mrpt_maps_COctoMapBase_octomap_OcTree_octomap_OcTreeNode__TInsertionOptions(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_maps_COctoMapBase_octomap_OcTree_octomap_OcTreeNode__TInsertionOptions const &o){ return new PyCallBack_mrpt_maps_COctoMapBase_octomap_OcTree_octomap_OcTreeNode__TInsertionOptions(o); } ) );
			cl.def( pybind11::init( [](mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions const &o){ return new mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions(o); } ) );
			cl.def_readwrite("maxrange", &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::maxrange);
			cl.def_readwrite("pruning", &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::pruning);
			cl.def("assign", (struct mrpt::maps::COctoMapBase<class octomap::OcTree, class octomap::OcTreeNode>::TInsertionOptions & (mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::*)(const struct mrpt::maps::COctoMapBase<class octomap::OcTree, class octomap::OcTreeNode>::TInsertionOptions &)) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::operator=, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::operator=(const struct mrpt::maps::COctoMapBase<class octomap::OcTree, class octomap::OcTreeNode>::TInsertionOptions &) --> struct mrpt::maps::COctoMapBase<class octomap::OcTree, class octomap::OcTreeNode>::TInsertionOptions &", pybind11::return_value_policy::automatic, pybind11::arg("o"));
			cl.def("loadFromConfigFile", (void (mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::loadFromConfigFile, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
			cl.def("setOccupancyThres", (void (mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::*)(double)) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::setOccupancyThres, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::setOccupancyThres(double) --> void", pybind11::arg("prob"));
			cl.def("setProbHit", (void (mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::*)(double)) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::setProbHit, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::setProbHit(double) --> void", pybind11::arg("prob"));
			cl.def("setProbMiss", (void (mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::*)(double)) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::setProbMiss, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::setProbMiss(double) --> void", pybind11::arg("prob"));
			cl.def("setClampingThresMin", (void (mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::*)(double)) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::setClampingThresMin, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::setClampingThresMin(double) --> void", pybind11::arg("thresProb"));
			cl.def("setClampingThresMax", (void (mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::*)(double)) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::setClampingThresMax, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::setClampingThresMax(double) --> void", pybind11::arg("thresProb"));
			cl.def("getOccupancyThres", (double (mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::*)() const) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::getOccupancyThres, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::getOccupancyThres() const --> double");
			cl.def("getOccupancyThresLog", (float (mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::*)() const) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::getOccupancyThresLog, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::getOccupancyThresLog() const --> float");
			cl.def("getProbHit", (double (mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::*)() const) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::getProbHit, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::getProbHit() const --> double");
			cl.def("getProbHitLog", (float (mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::*)() const) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::getProbHitLog, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::getProbHitLog() const --> float");
			cl.def("getProbMiss", (double (mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::*)() const) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::getProbMiss, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::getProbMiss() const --> double");
			cl.def("getProbMissLog", (float (mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::*)() const) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::getProbMissLog, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::getProbMissLog() const --> float");
			cl.def("getClampingThresMin", (double (mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::*)() const) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::getClampingThresMin, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::getClampingThresMin() const --> double");
			cl.def("getClampingThresMinLog", (float (mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::*)() const) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::getClampingThresMinLog, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::getClampingThresMinLog() const --> float");
			cl.def("getClampingThresMax", (double (mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::*)() const) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::getClampingThresMax, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::getClampingThresMax() const --> double");
			cl.def("getClampingThresMaxLog", (float (mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::*)() const) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::getClampingThresMaxLog, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TInsertionOptions::getClampingThresMaxLog() const --> float");
		}

		{ // mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TLikelihoodOptions file:mrpt/maps/COctoMapBase.h line:213
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TLikelihoodOptions, std::shared_ptr<mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TLikelihoodOptions>, PyCallBack_mrpt_maps_COctoMapBase_octomap_OcTree_octomap_OcTreeNode__TLikelihoodOptions, mrpt::config::CLoadableOptions> cl(enclosing_class, "TLikelihoodOptions", "");
			cl.def( pybind11::init( [](){ return new mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TLikelihoodOptions(); }, [](){ return new PyCallBack_mrpt_maps_COctoMapBase_octomap_OcTree_octomap_OcTreeNode__TLikelihoodOptions(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_maps_COctoMapBase_octomap_OcTree_octomap_OcTreeNode__TLikelihoodOptions const &o){ return new PyCallBack_mrpt_maps_COctoMapBase_octomap_OcTree_octomap_OcTreeNode__TLikelihoodOptions(o); } ) );
			cl.def( pybind11::init( [](mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TLikelihoodOptions const &o){ return new mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TLikelihoodOptions(o); } ) );
			cl.def_readwrite("decimation", &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TLikelihoodOptions::decimation);
			cl.def("loadFromConfigFile", (void (mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TLikelihoodOptions::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TLikelihoodOptions::loadFromConfigFile, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TLikelihoodOptions::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
			cl.def("writeToStream", (void (mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TLikelihoodOptions::*)(class mrpt::serialization::CArchive &) const) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TLikelihoodOptions::writeToStream, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TLikelihoodOptions::writeToStream(class mrpt::serialization::CArchive &) const --> void", pybind11::arg("out"));
			cl.def("readFromStream", (void (mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TLikelihoodOptions::*)(class mrpt::serialization::CArchive &)) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TLikelihoodOptions::readFromStream, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TLikelihoodOptions::readFromStream(class mrpt::serialization::CArchive &) --> void", pybind11::arg("in"));
			cl.def("assign", (struct mrpt::maps::COctoMapBase<class octomap::OcTree, class octomap::OcTreeNode>::TLikelihoodOptions & (mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TLikelihoodOptions::*)(const struct mrpt::maps::COctoMapBase<class octomap::OcTree, class octomap::OcTreeNode>::TLikelihoodOptions &)) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TLikelihoodOptions::operator=, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TLikelihoodOptions::operator=(const struct mrpt::maps::COctoMapBase<class octomap::OcTree, class octomap::OcTreeNode>::TLikelihoodOptions &) --> struct mrpt::maps::COctoMapBase<class octomap::OcTree, class octomap::OcTreeNode>::TLikelihoodOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TRenderingOptions file:mrpt/maps/COctoMapBase.h line:239
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TRenderingOptions, std::shared_ptr<mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TRenderingOptions>> cl(enclosing_class, "TRenderingOptions", "");
			cl.def( pybind11::init( [](){ return new mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TRenderingOptions(); } ) );
			cl.def( pybind11::init( [](mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TRenderingOptions const &o){ return new mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TRenderingOptions(o); } ) );
			cl.def_readwrite("generateGridLines", &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TRenderingOptions::generateGridLines);
			cl.def_readwrite("generateOccupiedVoxels", &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TRenderingOptions::generateOccupiedVoxels);
			cl.def_readwrite("visibleOccupiedVoxels", &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TRenderingOptions::visibleOccupiedVoxels);
			cl.def_readwrite("generateFreeVoxels", &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TRenderingOptions::generateFreeVoxels);
			cl.def_readwrite("visibleFreeVoxels", &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TRenderingOptions::visibleFreeVoxels);
			cl.def("writeToStream", (void (mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TRenderingOptions::*)(class mrpt::serialization::CArchive &) const) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TRenderingOptions::writeToStream, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TRenderingOptions::writeToStream(class mrpt::serialization::CArchive &) const --> void", pybind11::arg("out"));
			cl.def("readFromStream", (void (mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TRenderingOptions::*)(class mrpt::serialization::CArchive &)) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TRenderingOptions::readFromStream, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TRenderingOptions::readFromStream(class mrpt::serialization::CArchive &) --> void", pybind11::arg("in"));
			cl.def("assign", (struct mrpt::maps::COctoMapBase<class octomap::OcTree, class octomap::OcTreeNode>::TRenderingOptions & (mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TRenderingOptions::*)(const struct mrpt::maps::COctoMapBase<class octomap::OcTree, class octomap::OcTreeNode>::TRenderingOptions &)) &mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TRenderingOptions::operator=, "C++: mrpt::maps::COctoMapBase<octomap::OcTree, octomap::OcTreeNode>::TRenderingOptions::operator=(const struct mrpt::maps::COctoMapBase<class octomap::OcTree, class octomap::OcTreeNode>::TRenderingOptions &) --> struct mrpt::maps::COctoMapBase<class octomap::OcTree, class octomap::OcTreeNode>::TRenderingOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
}
