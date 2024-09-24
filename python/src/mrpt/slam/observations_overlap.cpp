#include <chrono>
#include <ios>
#include <iterator>
#include <memory>
#include <mrpt/core/Clock.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
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
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/slam/observations_overlap.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ostream>
#include <ratio>
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

void bind_mrpt_slam_observations_overlap(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
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
