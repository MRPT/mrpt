#include <chrono>
#include <iterator>
#include <memory>
#include <mrpt/core/Clock.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TTwist2D.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/poses/CPoses2DSequence.h>
#include <mrpt/poses/CPoses3DSequence.h>
#include <mrpt/poses/CRobot2DPoseEstimator.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ratio>
#include <sstream> // __str__
#include <string>
#include <variant>

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

// mrpt::poses::CPoses2DSequence file:mrpt/poses/CPoses2DSequence.h line:25
struct PyCallBack_mrpt_poses_CPoses2DSequence : public mrpt::poses::CPoses2DSequence {
	using mrpt::poses::CPoses2DSequence::CPoses2DSequence;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoses2DSequence *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPoses2DSequence::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoses2DSequence *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CPoses2DSequence::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoses2DSequence *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CPoses2DSequence::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoses2DSequence *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPoses2DSequence::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoses2DSequence *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPoses2DSequence::serializeFrom(a0, a1);
	}
};

// mrpt::poses::CPoses3DSequence file:mrpt/poses/CPoses3DSequence.h line:23
struct PyCallBack_mrpt_poses_CPoses3DSequence : public mrpt::poses::CPoses3DSequence {
	using mrpt::poses::CPoses3DSequence::CPoses3DSequence;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoses3DSequence *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPoses3DSequence::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoses3DSequence *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CPoses3DSequence::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoses3DSequence *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CPoses3DSequence::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoses3DSequence *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPoses3DSequence::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoses3DSequence *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPoses3DSequence::serializeFrom(a0, a1);
	}
};

void bind_mrpt_poses_CPoses2DSequence(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::poses::CPoses2DSequence file:mrpt/poses/CPoses2DSequence.h line:25
		pybind11::class_<mrpt::poses::CPoses2DSequence, std::shared_ptr<mrpt::poses::CPoses2DSequence>, PyCallBack_mrpt_poses_CPoses2DSequence, mrpt::serialization::CSerializable> cl(M("mrpt::poses"), "CPoses2DSequence", "This class stores a sequence of relative, incremental 2D poses. It is useful\n as the bases storing unit for more complex probability particles and for\n computing the absolute pose of any intermediate pose.\n\n \n CPose2D, CMultiMetricMap\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::poses::CPoses2DSequence(); }, [](){ return new PyCallBack_mrpt_poses_CPoses2DSequence(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_poses_CPoses2DSequence const &o){ return new PyCallBack_mrpt_poses_CPoses2DSequence(o); } ) );
		cl.def( pybind11::init( [](mrpt::poses::CPoses2DSequence const &o){ return new mrpt::poses::CPoses2DSequence(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::poses::CPoses2DSequence::GetRuntimeClassIdStatic, "C++: mrpt::poses::CPoses2DSequence::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::poses::CPoses2DSequence::*)() const) &mrpt::poses::CPoses2DSequence::GetRuntimeClass, "C++: mrpt::poses::CPoses2DSequence::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::poses::CPoses2DSequence::*)() const) &mrpt::poses::CPoses2DSequence::clone, "C++: mrpt::poses::CPoses2DSequence::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::poses::CPoses2DSequence::CreateObject, "C++: mrpt::poses::CPoses2DSequence::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("posesCount", (size_t (mrpt::poses::CPoses2DSequence::*)()) &mrpt::poses::CPoses2DSequence::posesCount, "Returns the poses count in the sequence:\n\nC++: mrpt::poses::CPoses2DSequence::posesCount() --> size_t");
		cl.def("getPose", (void (mrpt::poses::CPoses2DSequence::*)(unsigned int, class mrpt::poses::CPose2D &)) &mrpt::poses::CPoses2DSequence::getPose, "Reads the stored pose at index \"ind\", where the first one is 0, the last\n \"posesCount() - 1\"\n \n\n std::exception On invalid index value\n\nC++: mrpt::poses::CPoses2DSequence::getPose(unsigned int, class mrpt::poses::CPose2D &) --> void", pybind11::arg("ind"), pybind11::arg("outPose"));
		cl.def("changePose", (void (mrpt::poses::CPoses2DSequence::*)(unsigned int, class mrpt::poses::CPose2D &)) &mrpt::poses::CPoses2DSequence::changePose, "Changes the stored pose at index \"ind\", where the first one is 0, the\n last \"posesCount() - 1\"\n \n\n std::exception On invalid index value\n\nC++: mrpt::poses::CPoses2DSequence::changePose(unsigned int, class mrpt::poses::CPose2D &) --> void", pybind11::arg("ind"), pybind11::arg("inPose"));
		cl.def("appendPose", (void (mrpt::poses::CPoses2DSequence::*)(class mrpt::poses::CPose2D &)) &mrpt::poses::CPoses2DSequence::appendPose, "Appends a new pose at the end of sequence. Remember that poses are\n relative, incremental to the last one.\n\nC++: mrpt::poses::CPoses2DSequence::appendPose(class mrpt::poses::CPose2D &) --> void", pybind11::arg("newPose"));
		cl.def("clear", (void (mrpt::poses::CPoses2DSequence::*)()) &mrpt::poses::CPoses2DSequence::clear, "Clears the sequence.\n\nC++: mrpt::poses::CPoses2DSequence::clear() --> void");
		cl.def("absolutePoseOf", (class mrpt::poses::CPose2D (mrpt::poses::CPoses2DSequence::*)(unsigned int)) &mrpt::poses::CPoses2DSequence::absolutePoseOf, "Returns the absolute pose of a robot after moving \"n\" poses, so for\n \"n=0\" the origin pose (0,0,0deg) is returned, for \"n=1\" the first pose is\n returned, and for \"n=posesCount()\", the pose\n  of robot after moving ALL poses is returned, all of them relative to the\n starting pose.\n \n\n std::exception On invalid index value\n \n\n absolutePoseAfterAll\n\nC++: mrpt::poses::CPoses2DSequence::absolutePoseOf(unsigned int) --> class mrpt::poses::CPose2D", pybind11::arg("n"));
		cl.def("absolutePoseAfterAll", (class mrpt::poses::CPose2D (mrpt::poses::CPoses2DSequence::*)()) &mrpt::poses::CPoses2DSequence::absolutePoseAfterAll, "A shortcut for \"absolutePoseOf( posesCount() )\".\n \n\n absolutePoseOf, posesCount\n\nC++: mrpt::poses::CPoses2DSequence::absolutePoseAfterAll() --> class mrpt::poses::CPose2D");
		cl.def("computeTraveledDistanceAfter", (double (mrpt::poses::CPoses2DSequence::*)(size_t)) &mrpt::poses::CPoses2DSequence::computeTraveledDistanceAfter, "Returns the traveled distance after moving \"n\" poses, so for \"n=0\" it\n returns 0, for \"n=1\" the first traveled distance, and for\n \"n=posesCount()\", the total\n  distance after ALL movements.\n \n\n std::exception On invalid index value\n \n\n computeTraveledDistanceAfterAll\n\nC++: mrpt::poses::CPoses2DSequence::computeTraveledDistanceAfter(size_t) --> double", pybind11::arg("n"));
		cl.def("computeTraveledDistanceAfterAll", (double (mrpt::poses::CPoses2DSequence::*)()) &mrpt::poses::CPoses2DSequence::computeTraveledDistanceAfterAll, "Returns the traveled distance after ALL movements.\n   A shortcut for \"computeTraveledDistanceAfter( posesCount() )\".\n \n\n computeTraveledDistanceAfter\n\nC++: mrpt::poses::CPoses2DSequence::computeTraveledDistanceAfterAll() --> double");
		cl.def("assign", (class mrpt::poses::CPoses2DSequence & (mrpt::poses::CPoses2DSequence::*)(const class mrpt::poses::CPoses2DSequence &)) &mrpt::poses::CPoses2DSequence::operator=, "C++: mrpt::poses::CPoses2DSequence::operator=(const class mrpt::poses::CPoses2DSequence &) --> class mrpt::poses::CPoses2DSequence &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::poses::CPoses3DSequence file:mrpt/poses/CPoses3DSequence.h line:23
		pybind11::class_<mrpt::poses::CPoses3DSequence, std::shared_ptr<mrpt::poses::CPoses3DSequence>, PyCallBack_mrpt_poses_CPoses3DSequence, mrpt::serialization::CSerializable> cl(M("mrpt::poses"), "CPoses3DSequence", "This class stores a sequence of relative, incremental 3D poses. It is useful\n as the bases storing unit for more complex probability particles and for\n computing the absolute pose of any intermediate pose.\n\n \n CPose3D, CMultiMetricMap\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::poses::CPoses3DSequence(); }, [](){ return new PyCallBack_mrpt_poses_CPoses3DSequence(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_poses_CPoses3DSequence const &o){ return new PyCallBack_mrpt_poses_CPoses3DSequence(o); } ) );
		cl.def( pybind11::init( [](mrpt::poses::CPoses3DSequence const &o){ return new mrpt::poses::CPoses3DSequence(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::poses::CPoses3DSequence::GetRuntimeClassIdStatic, "C++: mrpt::poses::CPoses3DSequence::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::poses::CPoses3DSequence::*)() const) &mrpt::poses::CPoses3DSequence::GetRuntimeClass, "C++: mrpt::poses::CPoses3DSequence::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::poses::CPoses3DSequence::*)() const) &mrpt::poses::CPoses3DSequence::clone, "C++: mrpt::poses::CPoses3DSequence::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::poses::CPoses3DSequence::CreateObject, "C++: mrpt::poses::CPoses3DSequence::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("posesCount", (size_t (mrpt::poses::CPoses3DSequence::*)()) &mrpt::poses::CPoses3DSequence::posesCount, "Returns the poses count in the sequence:\n\nC++: mrpt::poses::CPoses3DSequence::posesCount() --> size_t");
		cl.def("getPose", (void (mrpt::poses::CPoses3DSequence::*)(unsigned int, class mrpt::poses::CPose3D &)) &mrpt::poses::CPoses3DSequence::getPose, "Reads the stored pose at index \"ind\", where the first one is 0, the last\n \"posesCount() - 1\"\n \n\n std::exception On invalid index value\n\nC++: mrpt::poses::CPoses3DSequence::getPose(unsigned int, class mrpt::poses::CPose3D &) --> void", pybind11::arg("ind"), pybind11::arg("outPose"));
		cl.def("changePose", (void (mrpt::poses::CPoses3DSequence::*)(unsigned int, class mrpt::poses::CPose3D &)) &mrpt::poses::CPoses3DSequence::changePose, "Changes the stored pose at index \"ind\", where the first one is 0, the\n last \"posesCount() - 1\"\n \n\n std::exception On invalid index value\n\nC++: mrpt::poses::CPoses3DSequence::changePose(unsigned int, class mrpt::poses::CPose3D &) --> void", pybind11::arg("ind"), pybind11::arg("inPose"));
		cl.def("appendPose", (void (mrpt::poses::CPoses3DSequence::*)(class mrpt::poses::CPose3D &)) &mrpt::poses::CPoses3DSequence::appendPose, "Appends a new pose at the end of sequence. Remember that poses are\n relative, incremental to the last one.\n\nC++: mrpt::poses::CPoses3DSequence::appendPose(class mrpt::poses::CPose3D &) --> void", pybind11::arg("newPose"));
		cl.def("clear", (void (mrpt::poses::CPoses3DSequence::*)()) &mrpt::poses::CPoses3DSequence::clear, "Clears the sequence.\n\nC++: mrpt::poses::CPoses3DSequence::clear() --> void");
		cl.def("absolutePoseOf", (class mrpt::poses::CPose3D (mrpt::poses::CPoses3DSequence::*)(unsigned int)) &mrpt::poses::CPoses3DSequence::absolutePoseOf, "Returns the absolute pose of a robot after moving \"n\" poses, so for\n \"n=0\" the origin pose (0,0,0deg) is returned, for \"n=1\" the first pose is\n returned, and for \"n=posesCount()\", the pose\n  of robot after moving ALL poses is returned, all of them relative to the\n starting pose.\n \n\n std::exception On invalid index value\n \n\n absolutePoseAfterAll\n\nC++: mrpt::poses::CPoses3DSequence::absolutePoseOf(unsigned int) --> class mrpt::poses::CPose3D", pybind11::arg("n"));
		cl.def("absolutePoseAfterAll", (class mrpt::poses::CPose3D (mrpt::poses::CPoses3DSequence::*)()) &mrpt::poses::CPoses3DSequence::absolutePoseAfterAll, "A shortcut for \"absolutePoseOf( posesCount() )\".\n \n\n absolutePoseOf, posesCount\n\nC++: mrpt::poses::CPoses3DSequence::absolutePoseAfterAll() --> class mrpt::poses::CPose3D");
		cl.def("computeTraveledDistanceAfter", (double (mrpt::poses::CPoses3DSequence::*)(size_t)) &mrpt::poses::CPoses3DSequence::computeTraveledDistanceAfter, "Returns the traveled distance after moving \"n\" poses, so for \"n=0\" it\n returns 0, for \"n=1\" the first traveled distance, and for\n \"n=posesCount()\", the total\n  distance after ALL movements.\n \n\n std::exception On invalid index value\n \n\n computeTraveledDistanceAfterAll\n\nC++: mrpt::poses::CPoses3DSequence::computeTraveledDistanceAfter(size_t) --> double", pybind11::arg("n"));
		cl.def("computeTraveledDistanceAfterAll", (double (mrpt::poses::CPoses3DSequence::*)()) &mrpt::poses::CPoses3DSequence::computeTraveledDistanceAfterAll, "Returns the traveled distance after ALL movements.\n   A shortcut for \"computeTraveledDistanceAfter( posesCount() )\".\n \n\n computeTraveledDistanceAfter\n\nC++: mrpt::poses::CPoses3DSequence::computeTraveledDistanceAfterAll() --> double");
		cl.def("assign", (class mrpt::poses::CPoses3DSequence & (mrpt::poses::CPoses3DSequence::*)(const class mrpt::poses::CPoses3DSequence &)) &mrpt::poses::CPoses3DSequence::operator=, "C++: mrpt::poses::CPoses3DSequence::operator=(const class mrpt::poses::CPoses3DSequence &) --> class mrpt::poses::CPoses3DSequence &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::poses::CRobot2DPoseEstimator file:mrpt/poses/CRobot2DPoseEstimator.h line:30
		pybind11::class_<mrpt::poses::CRobot2DPoseEstimator, std::shared_ptr<mrpt::poses::CRobot2DPoseEstimator>> cl(M("mrpt::poses"), "CRobot2DPoseEstimator", "A simple filter to estimate and extrapolate the robot 2D (x,y,phi) pose from\nasynchronous odometry and localization/SLAM data.\n  The implemented model is a state vector:\n		- TPose2D (x,y,phi) + TTwist2D (vx,vy,omega)\n  The filter can be asked for an extrapolation for some arbitrary time `t'`,\nand it'll do a simple linear prediction.\n  **All methods are thread-safe**.\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::poses::CRobot2DPoseEstimator(); } ) );
		cl.def_readwrite("params", &mrpt::poses::CRobot2DPoseEstimator::params);
		cl.def("reset", (void (mrpt::poses::CRobot2DPoseEstimator::*)()) &mrpt::poses::CRobot2DPoseEstimator::reset, "Resets all internal state. \n\nC++: mrpt::poses::CRobot2DPoseEstimator::reset() --> void");
		cl.def("processUpdateNewPoseLocalization", (void (mrpt::poses::CRobot2DPoseEstimator::*)(const struct mrpt::math::TPose2D &, mrpt::Clock::time_point)) &mrpt::poses::CRobot2DPoseEstimator::processUpdateNewPoseLocalization, "Updates the filter with new global-coordinates localization data from a\n localization or SLAM source.\n \n\n The timestamp of the sensor readings used to evaluate\n localization / SLAM.\n\nC++: mrpt::poses::CRobot2DPoseEstimator::processUpdateNewPoseLocalization(const struct mrpt::math::TPose2D &, mrpt::Clock::time_point) --> void", pybind11::arg("newPose"), pybind11::arg("tim"));
		cl.def("processUpdateNewOdometry", [](mrpt::poses::CRobot2DPoseEstimator &o, const struct mrpt::math::TPose2D & a0, mrpt::Clock::time_point const & a1) -> void { return o.processUpdateNewOdometry(a0, a1); }, "", pybind11::arg("newGlobalOdometry"), pybind11::arg("cur_tim"));
		cl.def("processUpdateNewOdometry", [](mrpt::poses::CRobot2DPoseEstimator &o, const struct mrpt::math::TPose2D & a0, mrpt::Clock::time_point const & a1, bool const & a2) -> void { return o.processUpdateNewOdometry(a0, a1, a2); }, "", pybind11::arg("newGlobalOdometry"), pybind11::arg("cur_tim"), pybind11::arg("hasVelocities"));
		cl.def("processUpdateNewOdometry", (void (mrpt::poses::CRobot2DPoseEstimator::*)(const struct mrpt::math::TPose2D &, mrpt::Clock::time_point, bool, const struct mrpt::math::TTwist2D &)) &mrpt::poses::CRobot2DPoseEstimator::processUpdateNewOdometry, "Updates the filter with new odometry readings. \n\nC++: mrpt::poses::CRobot2DPoseEstimator::processUpdateNewOdometry(const struct mrpt::math::TPose2D &, mrpt::Clock::time_point, bool, const struct mrpt::math::TTwist2D &) --> void", pybind11::arg("newGlobalOdometry"), pybind11::arg("cur_tim"), pybind11::arg("hasVelocities"), pybind11::arg("newRobotVelLocal"));
		cl.def("getCurrentEstimate", [](mrpt::poses::CRobot2DPoseEstimator const &o, struct mrpt::math::TPose2D & a0, struct mrpt::math::TTwist2D & a1, struct mrpt::math::TTwist2D & a2) -> bool { return o.getCurrentEstimate(a0, a1, a2); }, "", pybind11::arg("pose"), pybind11::arg("velLocal"), pybind11::arg("velGlobal"));
		cl.def("getCurrentEstimate", (bool (mrpt::poses::CRobot2DPoseEstimator::*)(struct mrpt::math::TPose2D &, struct mrpt::math::TTwist2D &, struct mrpt::math::TTwist2D &, mrpt::Clock::time_point) const) &mrpt::poses::CRobot2DPoseEstimator::getCurrentEstimate, "Get the estimate for a given timestamp (defaults to `now()`), obtained\n as:\n\n   last_loc (+) [ last_odo (-) odo_ref ] (+) extrapolation_from_vw\n\n \n true is the estimate can be trusted. False if the real observed\n data is too old or there is no valid data yet.\n \n\n getLatestRobotPose\n\nC++: mrpt::poses::CRobot2DPoseEstimator::getCurrentEstimate(struct mrpt::math::TPose2D &, struct mrpt::math::TTwist2D &, struct mrpt::math::TTwist2D &, mrpt::Clock::time_point) const --> bool", pybind11::arg("pose"), pybind11::arg("velLocal"), pybind11::arg("velGlobal"), pybind11::arg("tim_query"));
		cl.def("getLatestRobotPose", (bool (mrpt::poses::CRobot2DPoseEstimator::*)(struct mrpt::math::TPose2D &) const) &mrpt::poses::CRobot2DPoseEstimator::getLatestRobotPose, "Get the latest known robot pose, either from odometry or localization.\n  This differs from getCurrentEstimate() in that this method does NOT\n extrapolate as getCurrentEstimate() does.\n \n\n false if there is not estimation yet.\n \n\n getCurrentEstimate\n\nC++: mrpt::poses::CRobot2DPoseEstimator::getLatestRobotPose(struct mrpt::math::TPose2D &) const --> bool", pybind11::arg("pose"));
		cl.def("getLatestRobotPose", (bool (mrpt::poses::CRobot2DPoseEstimator::*)(class mrpt::poses::CPose2D &) const) &mrpt::poses::CRobot2DPoseEstimator::getLatestRobotPose, "C++: mrpt::poses::CRobot2DPoseEstimator::getLatestRobotPose(class mrpt::poses::CPose2D &) const --> bool", pybind11::arg("pose"));
		cl.def_static("extrapolateRobotPose", (void (*)(const struct mrpt::math::TPose2D &, const struct mrpt::math::TTwist2D &, const double, struct mrpt::math::TPose2D &)) &mrpt::poses::CRobot2DPoseEstimator::extrapolateRobotPose, "Auxiliary static method to extrapolate the pose of a robot located at\n \"p\" with velocities (v,w) after a time delay \"delta_time\". \n\nC++: mrpt::poses::CRobot2DPoseEstimator::extrapolateRobotPose(const struct mrpt::math::TPose2D &, const struct mrpt::math::TTwist2D &, const double, struct mrpt::math::TPose2D &) --> void", pybind11::arg("p"), pybind11::arg("robot_vel_local"), pybind11::arg("delta_time"), pybind11::arg("new_p"));

		{ // mrpt::poses::CRobot2DPoseEstimator::TOptions file:mrpt/poses/CRobot2DPoseEstimator.h line:81
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::poses::CRobot2DPoseEstimator::TOptions, std::shared_ptr<mrpt::poses::CRobot2DPoseEstimator::TOptions>> cl(enclosing_class, "TOptions", "");
			cl.def( pybind11::init( [](){ return new mrpt::poses::CRobot2DPoseEstimator::TOptions(); } ) );
			cl.def( pybind11::init( [](mrpt::poses::CRobot2DPoseEstimator::TOptions const &o){ return new mrpt::poses::CRobot2DPoseEstimator::TOptions(o); } ) );
			cl.def_readwrite("max_odometry_age", &mrpt::poses::CRobot2DPoseEstimator::TOptions::max_odometry_age);
			cl.def_readwrite("max_localiz_age", &mrpt::poses::CRobot2DPoseEstimator::TOptions::max_localiz_age);
			cl.def("assign", (struct mrpt::poses::CRobot2DPoseEstimator::TOptions & (mrpt::poses::CRobot2DPoseEstimator::TOptions::*)(const struct mrpt::poses::CRobot2DPoseEstimator::TOptions &)) &mrpt::poses::CRobot2DPoseEstimator::TOptions::operator=, "C++: mrpt::poses::CRobot2DPoseEstimator::TOptions::operator=(const struct mrpt::poses::CRobot2DPoseEstimator::TOptions &) --> struct mrpt::poses::CRobot2DPoseEstimator::TOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
}
