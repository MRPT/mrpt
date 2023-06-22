#include <chrono>
#include <deque>
#include <iterator>
#include <memory>
#include <mrpt/core/Clock.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/TMetricMapInitializer.h>
#include <mrpt/maps/metric_map_types.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ostream>
#include <ratio>
#include <sstream> // __str__
#include <string>
#include <tuple>
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

void bind_mrpt_maps_CMetricMap_1(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::maps::CMetricMap file:mrpt/maps/CMetricMap.h line:71
		pybind11::class_<mrpt::maps::CMetricMap, std::shared_ptr<mrpt::maps::CMetricMap>, mrpt::serialization::CSerializable, mrpt::system::CObservable, mrpt::Stringifyable, mrpt::opengl::Visualizable> cl(M("mrpt::maps"), "CMetricMap", "Declares a virtual base class for all metric maps storage classes.\n  In this class virtual methods are provided to allow the insertion\n  of any type of \"CObservation\" objects into the metric map, thus\n  updating the map (doesn't matter if it is a 2D/3D grid, a point map, etc.).\n\n  Observations don't include any information about the\n  robot pose, just the raw observation and information about\n  the sensor pose relative to the robot mobile base coordinates origin.\n\n  Note that all metric maps implement this mrpt::system::CObservable\ninterface,\n   emitting the following events:\n	  - mrpt::obs::mrptEventMetricMapClear: Upon call of the ::clear() method.\n    - mrpt::obs::mrptEventMetricMapInsert: Upon insertion of an observation\nthat effectively modifies the map (e.g. inserting an image into a grid map\nwill NOT raise an event, inserting a laser scan will).\n\n To check what observations are supported by each metric map, see\n \n\n \n All derived class must implement a static class factory\n`<metric_map_class>::MapDefinition()` that builds a default\nTMetricMapInitializer\n\n \n CObservation, CSensoryFrame, CMultiMetricMap\n \n\n\n ");
		cl.def_readwrite("genericMapParams", &mrpt::maps::CMetricMap::genericMapParams);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::maps::CMetricMap::*)() const) &mrpt::maps::CMetricMap::GetRuntimeClass, "C++: mrpt::maps::CMetricMap::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::maps::CMetricMap::GetRuntimeClassIdStatic, "C++: mrpt::maps::CMetricMap::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("clear", (void (mrpt::maps::CMetricMap::*)()) &mrpt::maps::CMetricMap::clear, "Erase all the contents of the map \n\nC++: mrpt::maps::CMetricMap::clear() --> void");
		cl.def("isEmpty", (bool (mrpt::maps::CMetricMap::*)() const) &mrpt::maps::CMetricMap::isEmpty, "Returns true if the map is empty/no observation has been inserted.\n\nC++: mrpt::maps::CMetricMap::isEmpty() const --> bool");
		cl.def("loadFromProbabilisticPosesAndObservations", (void (mrpt::maps::CMetricMap::*)(const class mrpt::maps::CSimpleMap &)) &mrpt::maps::CMetricMap::loadFromProbabilisticPosesAndObservations, "Load the map contents from a CSimpleMap object, erasing all previous\n content of the map. This is done invoking `insertObservation()` for each\n observation at the mean 3D robot pose of each pose-observations pair in\n the CSimpleMap object.\n\n \n insertObservation, CSimpleMap\n \n\n std::exception Some internal steps in invoked methods can\n raise exceptions on invalid parameters, etc...\n\nC++: mrpt::maps::CMetricMap::loadFromProbabilisticPosesAndObservations(const class mrpt::maps::CSimpleMap &) --> void", pybind11::arg("Map"));
		cl.def("loadFromSimpleMap", (void (mrpt::maps::CMetricMap::*)(const class mrpt::maps::CSimpleMap &)) &mrpt::maps::CMetricMap::loadFromSimpleMap, "! \n\nC++: mrpt::maps::CMetricMap::loadFromSimpleMap(const class mrpt::maps::CSimpleMap &) --> void", pybind11::arg("Map"));
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
	}
}
