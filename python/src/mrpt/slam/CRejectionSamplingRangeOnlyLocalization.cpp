#include <chrono>
#include <ios>
#include <iterator>
#include <memory>
#include <mrpt/core/Clock.h>
#include <mrpt/maps/CLandmarksMap.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/TMetricMapInitializer.h>
#include <mrpt/maps/metric_map_types.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TPose3DQuat.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/slam/CRejectionSamplingRangeOnlyLocalization.h>
#include <mrpt/slam/observations_overlap.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/typemeta/static_string.h>
#include <mrpt/vision/CFeatureExtraction.h>
#include <optional>
#include <ostream>
#include <ratio>
#include <sstream> // __str__
#include <streambuf>
#include <string>
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

// mrpt::slam::CRejectionSamplingRangeOnlyLocalization file:mrpt/slam/CRejectionSamplingRangeOnlyLocalization.h line:34
struct PyCallBack_mrpt_slam_CRejectionSamplingRangeOnlyLocalization : public mrpt::slam::CRejectionSamplingRangeOnlyLocalization {
	using mrpt::slam::CRejectionSamplingRangeOnlyLocalization::CRejectionSamplingRangeOnlyLocalization;

	void RS_drawFromProposal(class mrpt::poses::CPose2D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CRejectionSamplingRangeOnlyLocalization *>(this), "RS_drawFromProposal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRejectionSamplingRangeOnlyLocalization::RS_drawFromProposal(a0);
	}
	double RS_observationLikelihood(const class mrpt::poses::CPose2D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CRejectionSamplingRangeOnlyLocalization *>(this), "RS_observationLikelihood");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CRejectionSamplingRangeOnlyLocalization::RS_observationLikelihood(a0);
	}
};

void bind_mrpt_slam_CRejectionSamplingRangeOnlyLocalization(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::slam::CRejectionSamplingRangeOnlyLocalization file:mrpt/slam/CRejectionSamplingRangeOnlyLocalization.h line:34
		pybind11::class_<mrpt::slam::CRejectionSamplingRangeOnlyLocalization, std::shared_ptr<mrpt::slam::CRejectionSamplingRangeOnlyLocalization>, PyCallBack_mrpt_slam_CRejectionSamplingRangeOnlyLocalization, mrpt::bayes::CRejectionSamplingCapable<mrpt::poses::CPose2D,mrpt::bayes::particle_storage_mode::POINTER>> cl(M("mrpt::slam"), "CRejectionSamplingRangeOnlyLocalization", "An implementation of rejection sampling for generating 2D robot pose from\n range-only measurements within a landmarks (beacons) map.\n    Before calling the method \"rejectionSampling\" to generate the samples, you\n must call \"setParams\".\n    It is assumed a planar scenario, where the robot is at a fixed height\n (default=0).\n \n\n bayes::CRejectionSamplingCapable  \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::slam::CRejectionSamplingRangeOnlyLocalization(); }, [](){ return new PyCallBack_mrpt_slam_CRejectionSamplingRangeOnlyLocalization(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_slam_CRejectionSamplingRangeOnlyLocalization const &o){ return new PyCallBack_mrpt_slam_CRejectionSamplingRangeOnlyLocalization(o); } ) );
		cl.def( pybind11::init( [](mrpt::slam::CRejectionSamplingRangeOnlyLocalization const &o){ return new mrpt::slam::CRejectionSamplingRangeOnlyLocalization(o); } ) );
		cl.def("setParams", [](mrpt::slam::CRejectionSamplingRangeOnlyLocalization &o, const class mrpt::maps::CLandmarksMap & a0, const class mrpt::obs::CObservationBeaconRanges & a1, float const & a2, const class mrpt::poses::CPose2D & a3) -> bool { return o.setParams(a0, a1, a2, a3); }, "", pybind11::arg("beaconsMap"), pybind11::arg("observation"), pybind11::arg("sigmaRanges"), pybind11::arg("oldPose"));
		cl.def("setParams", [](mrpt::slam::CRejectionSamplingRangeOnlyLocalization &o, const class mrpt::maps::CLandmarksMap & a0, const class mrpt::obs::CObservationBeaconRanges & a1, float const & a2, const class mrpt::poses::CPose2D & a3, float const & a4) -> bool { return o.setParams(a0, a1, a2, a3, a4); }, "", pybind11::arg("beaconsMap"), pybind11::arg("observation"), pybind11::arg("sigmaRanges"), pybind11::arg("oldPose"), pybind11::arg("robot_z"));
		cl.def("setParams", (bool (mrpt::slam::CRejectionSamplingRangeOnlyLocalization::*)(const class mrpt::maps::CLandmarksMap &, const class mrpt::obs::CObservationBeaconRanges &, float, const class mrpt::poses::CPose2D &, float, bool)) &mrpt::slam::CRejectionSamplingRangeOnlyLocalization::setParams, "The parameters used in the generation of random samples:\n \n\n The map containing the N beacons (indexed by their\n \"beacon ID\"s). Only the mean 3D position of the beacons is used, the\n covariance is ignored.\n \n\n An observation with, at least ONE range measurement.\n \n\n The standard deviation of the \"range measurement\n noise\".\n \n\n The height of the robot on the floor (default=0). Note\n that the beacon sensor on the robot may be at a different height,\n according to data within the observation object.\n \n\n Whether to make a simple check for potential\n good angles from the beacons to generate samples (disable to speed-up the\n preparation vs. making slower the drawn).\n  This method fills out the member \"m_dataPerBeacon\".\n \n\n true if at least ONE beacon has been successfully loaded, false\n otherwise. In this case do not call \"rejectionSampling\" or an exception\n will be launch, since there is no information to generate samples.\n\nC++: mrpt::slam::CRejectionSamplingRangeOnlyLocalization::setParams(const class mrpt::maps::CLandmarksMap &, const class mrpt::obs::CObservationBeaconRanges &, float, const class mrpt::poses::CPose2D &, float, bool) --> bool", pybind11::arg("beaconsMap"), pybind11::arg("observation"), pybind11::arg("sigmaRanges"), pybind11::arg("oldPose"), pybind11::arg("robot_z"), pybind11::arg("autoCheckAngleRanges"));
		cl.def("assign", (class mrpt::slam::CRejectionSamplingRangeOnlyLocalization & (mrpt::slam::CRejectionSamplingRangeOnlyLocalization::*)(const class mrpt::slam::CRejectionSamplingRangeOnlyLocalization &)) &mrpt::slam::CRejectionSamplingRangeOnlyLocalization::operator=, "C++: mrpt::slam::CRejectionSamplingRangeOnlyLocalization::operator=(const class mrpt::slam::CRejectionSamplingRangeOnlyLocalization &) --> class mrpt::slam::CRejectionSamplingRangeOnlyLocalization &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	// mrpt::slam::observationsOverlap(const class mrpt::obs::CObservation *, const class mrpt::obs::CObservation *, const class mrpt::poses::CPose3D *) file:mrpt/slam/observations_overlap.h line:26
	M("mrpt::slam").def("observationsOverlap", [](const class mrpt::obs::CObservation * a0, const class mrpt::obs::CObservation * a1) -> double { return mrpt::slam::observationsOverlap(a0, a1); }, "", pybind11::arg("o1"), pybind11::arg("o2"));
	M("mrpt::slam").def("observationsOverlap", (double (*)(const class mrpt::obs::CObservation *, const class mrpt::obs::CObservation *, const class mrpt::poses::CPose3D *)) &mrpt::slam::observationsOverlap, "Estimates the \"overlap\" or \"matching ratio\" of two observations (range\n [0,1]), possibly taking into account their relative positions.\n  \n\n This is used in mrpt::slam::CIncrementalMapPartitioner\n\nC++: mrpt::slam::observationsOverlap(const class mrpt::obs::CObservation *, const class mrpt::obs::CObservation *, const class mrpt::poses::CPose3D *) --> double", pybind11::arg("o1"), pybind11::arg("o2"), pybind11::arg("pose_o2_wrt_o1"));

	// mrpt::slam::observationsOverlap(const class std::shared_ptr<class mrpt::obs::CObservation> &, const class std::shared_ptr<class mrpt::obs::CObservation> &, const class mrpt::poses::CPose3D *) file:mrpt/slam/observations_overlap.h line:35
	M("mrpt::slam").def("observationsOverlap", [](const class std::shared_ptr<class mrpt::obs::CObservation> & a0, const class std::shared_ptr<class mrpt::obs::CObservation> & a1) -> double { return mrpt::slam::observationsOverlap(a0, a1); }, "", pybind11::arg("o1"), pybind11::arg("o2"));
	M("mrpt::slam").def("observationsOverlap", (double (*)(const class std::shared_ptr<class mrpt::obs::CObservation> &, const class std::shared_ptr<class mrpt::obs::CObservation> &, const class mrpt::poses::CPose3D *)) &mrpt::slam::observationsOverlap, "Estimates the \"overlap\" or \"matching ratio\" of two observations (range\n [0,1]), possibly taking into account their relative positions.\n  \n\n This is used in mrpt::slam::CIncrementalMapPartitioner\n\nC++: mrpt::slam::observationsOverlap(const class std::shared_ptr<class mrpt::obs::CObservation> &, const class std::shared_ptr<class mrpt::obs::CObservation> &, const class mrpt::poses::CPose3D *) --> double", pybind11::arg("o1"), pybind11::arg("o2"), pybind11::arg("pose_o2_wrt_o1"));

	// mrpt::slam::observationsOverlap(const class mrpt::obs::CSensoryFrame &, const class mrpt::obs::CSensoryFrame &, const class mrpt::poses::CPose3D *) file:mrpt/slam/observations_overlap.h line:49
	M("mrpt::slam").def("observationsOverlap", [](const class mrpt::obs::CSensoryFrame & a0, const class mrpt::obs::CSensoryFrame & a1) -> double { return mrpt::slam::observationsOverlap(a0, a1); }, "", pybind11::arg("sf1"), pybind11::arg("sf2"));
	M("mrpt::slam").def("observationsOverlap", (double (*)(const class mrpt::obs::CSensoryFrame &, const class mrpt::obs::CSensoryFrame &, const class mrpt::poses::CPose3D *)) &mrpt::slam::observationsOverlap, "Estimates the \"overlap\" or \"matching ratio\" of two set of observations\n (range [0,1]), possibly taking into account their relative positions.\n   This method computes the average between each of the observations in each\n SF.\n  \n\n This is used in mrpt::slam::CIncrementalMapPartitioner\n\nC++: mrpt::slam::observationsOverlap(const class mrpt::obs::CSensoryFrame &, const class mrpt::obs::CSensoryFrame &, const class mrpt::poses::CPose3D *) --> double", pybind11::arg("sf1"), pybind11::arg("sf2"), pybind11::arg("pose_sf2_wrt_sf1"));

	// mrpt::slam::observationsOverlap(const class std::shared_ptr<class mrpt::obs::CSensoryFrame> &, const class std::shared_ptr<class mrpt::obs::CSensoryFrame> &, const class mrpt::poses::CPose3D *) file:mrpt/slam/observations_overlap.h line:60
	M("mrpt::slam").def("observationsOverlap", [](const class std::shared_ptr<class mrpt::obs::CSensoryFrame> & a0, const class std::shared_ptr<class mrpt::obs::CSensoryFrame> & a1) -> double { return mrpt::slam::observationsOverlap(a0, a1); }, "", pybind11::arg("sf1"), pybind11::arg("sf2"));
	M("mrpt::slam").def("observationsOverlap", (double (*)(const class std::shared_ptr<class mrpt::obs::CSensoryFrame> &, const class std::shared_ptr<class mrpt::obs::CSensoryFrame> &, const class mrpt::poses::CPose3D *)) &mrpt::slam::observationsOverlap, "Estimates the \"overlap\" or \"matching ratio\" of two set of observations\n (range [0,1]), possibly taking into account their relative positions.\n   This method computes the average between each of the observations in each\n SF.\n  \n\n This is used in mrpt::slam::CIncrementalMapPartitioner\n\nC++: mrpt::slam::observationsOverlap(const class std::shared_ptr<class mrpt::obs::CSensoryFrame> &, const class std::shared_ptr<class mrpt::obs::CSensoryFrame> &, const class mrpt::poses::CPose3D *) --> double", pybind11::arg("sf1"), pybind11::arg("sf2"), pybind11::arg("pose_sf2_wrt_sf1"));

}
