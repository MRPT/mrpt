#include <chrono>
#include <istream>
#include <iterator>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/core/Clock.h>
#include <mrpt/core/aligned_allocator.h>
#include <mrpt/core/safe_pointers.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/img/color_maps.h>
#include <mrpt/kinematics/CVehicleVelCmd.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/TMetricMapInitializer.h>
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
#include <mrpt/math/TTwist2D.h>
#include <mrpt/nav/reactive/CRobot2NavInterface.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/T2DScanProperties.h>
#include <mrpt/obs/T3DPointsTo2DScanParams.h>
#include <mrpt/obs/TRangeImageFilter.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ostream>
#include <ratio>
#include <sstream> // __str__
#include <string>
#include <utility>
#include <vector>

#include <functional>
#include <pybind11/pybind11.h>
#include <string>
#include <stl_binders.hpp>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

// mrpt::nav::CRobot2NavInterface file:mrpt/nav/reactive/CRobot2NavInterface.h line:43
struct PyCallBack_mrpt_nav_CRobot2NavInterface : public mrpt::nav::CRobot2NavInterface {
	using mrpt::nav::CRobot2NavInterface::CRobot2NavInterface;

	bool getCurrentPoseAndSpeeds(struct mrpt::math::TPose2D & a0, struct mrpt::math::TTwist2D & a1, struct std::chrono::time_point<class mrpt::Clock, struct std::chrono::duration<long, struct std::ratio<1, 10000000> > > & a2, struct mrpt::math::TPose2D & a3, std::string & a4) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "getCurrentPoseAndSpeeds");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CRobot2NavInterface::getCurrentPoseAndSpeeds\"");
	}
	bool changeSpeeds(const class mrpt::kinematics::CVehicleVelCmd & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "changeSpeeds");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CRobot2NavInterface::changeSpeeds\"");
	}
	bool changeSpeedsNOP() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "changeSpeedsNOP");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CRobot2NavInterface::changeSpeedsNOP();
	}
	bool stop(bool a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "stop");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CRobot2NavInterface::stop\"");
	}
	class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> getEmergencyStopCmd() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "getEmergencyStopCmd");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>::value) {
				static pybind11::detail::override_caster_t<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>> caster;
				return pybind11::detail::cast_ref<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CRobot2NavInterface::getEmergencyStopCmd\"");
	}
	class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> getStopCmd() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "getStopCmd");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>::value) {
				static pybind11::detail::override_caster_t<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>> caster;
				return pybind11::detail::cast_ref<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CRobot2NavInterface::getStopCmd\"");
	}
	class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> getAlignCmd(const double a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "getAlignCmd");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>::value) {
				static pybind11::detail::override_caster_t<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>> caster;
				return pybind11::detail::cast_ref<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o));
		}
		return CRobot2NavInterface::getAlignCmd(a0);
	}
	bool startWatchdog(float a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "startWatchdog");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CRobot2NavInterface::startWatchdog(a0);
	}
	bool stopWatchdog() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "stopWatchdog");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CRobot2NavInterface::stopWatchdog();
	}
	bool senseObstacles(class mrpt::maps::CSimplePointsMap & a0, struct std::chrono::time_point<class mrpt::Clock, struct std::chrono::duration<long, struct std::ratio<1, 10000000> > > & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "senseObstacles");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CRobot2NavInterface::senseObstacles\"");
	}
	void sendNavigationStartEvent() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "sendNavigationStartEvent");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRobot2NavInterface::sendNavigationStartEvent();
	}
	void sendNavigationEndEvent() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "sendNavigationEndEvent");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRobot2NavInterface::sendNavigationEndEvent();
	}
	void sendWaypointReachedEvent(int a0, bool a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "sendWaypointReachedEvent");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRobot2NavInterface::sendWaypointReachedEvent(a0, a1);
	}
	void sendNewWaypointTargetEvent(int a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "sendNewWaypointTargetEvent");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRobot2NavInterface::sendNewWaypointTargetEvent(a0);
	}
	void sendNavigationEndDueToErrorEvent() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "sendNavigationEndDueToErrorEvent");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRobot2NavInterface::sendNavigationEndDueToErrorEvent();
	}
	void sendWaySeemsBlockedEvent() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "sendWaySeemsBlockedEvent");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRobot2NavInterface::sendWaySeemsBlockedEvent();
	}
	void sendApparentCollisionEvent() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "sendApparentCollisionEvent");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRobot2NavInterface::sendApparentCollisionEvent();
	}
	void sendCannotGetCloserToBlockedTargetEvent() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "sendCannotGetCloserToBlockedTargetEvent");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRobot2NavInterface::sendCannotGetCloserToBlockedTargetEvent();
	}
	double getNavigationTime() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "getNavigationTime");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CRobot2NavInterface::getNavigationTime();
	}
	void resetNavigationTimer() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "resetNavigationTimer");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRobot2NavInterface::resetNavigationTimer();
	}
};

void bind_mrpt_nav_reactive_CRobot2NavInterface(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::nav::CRobot2NavInterface file:mrpt/nav/reactive/CRobot2NavInterface.h line:43
		pybind11::class_<mrpt::nav::CRobot2NavInterface, std::shared_ptr<mrpt::nav::CRobot2NavInterface>, PyCallBack_mrpt_nav_CRobot2NavInterface> cl(M("mrpt::nav"), "CRobot2NavInterface", "The pure virtual interface between a real or simulated robot and any\n `CAbstractNavigator`-derived class.\n\n  The user must define a new class derived from `CRobot2NavInterface` and\n reimplement\n   all pure virtual and the desired virtual methods according to the\n documentation in this class.\n\n This class does not make assumptions about the kinematic\n model of the robot, so it can work with either\n Ackermann, differential-driven or holonomic robots. It will depend on the\n used PTGs, so checkout\n each PTG documentation for the length and meaning of velocity commands.\n\n If used for a simulator, users may prefer to inherit from one of these\n classes, which already provide partial implementations:\n  - mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven\n  - mrpt::nav::CRobot2NavInterfaceForSimulator_Holo\n\n \n CReactiveNavigationSystem, CAbstractNavigator\n  \n\n\n ");
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_nav_CRobot2NavInterface(); } ) );
		cl.def(pybind11::init<PyCallBack_mrpt_nav_CRobot2NavInterface const &>());
		cl.def("getCurrentPoseAndSpeeds", (bool (mrpt::nav::CRobot2NavInterface::*)(struct mrpt::math::TPose2D &, struct mrpt::math::TTwist2D &, struct std::chrono::time_point<class mrpt::Clock, struct std::chrono::duration<long, struct std::ratio<1, 10000000> > > &, struct mrpt::math::TPose2D &, std::string &)) &mrpt::nav::CRobot2NavInterface::getCurrentPoseAndSpeeds, "Get the current pose and velocity of the robot. The implementation\n should not take too much time to return,\n   so if it might take more than ~10ms to ask the robot for the\n instantaneous data, it may be good enough to\n   return the latest values from a cache which is updated in a parallel\n thread.\n \n\n false on any error retrieving these values from the robot.\n  \n\nC++: mrpt::nav::CRobot2NavInterface::getCurrentPoseAndSpeeds(struct mrpt::math::TPose2D &, struct mrpt::math::TTwist2D &, struct std::chrono::time_point<class mrpt::Clock, struct std::chrono::duration<long, struct std::ratio<1, 10000000> > > &, struct mrpt::math::TPose2D &, std::string &) --> bool", pybind11::arg("curPose"), pybind11::arg("curVelGlobal"), pybind11::arg("timestamp"), pybind11::arg("curOdometry"), pybind11::arg("frame_id"));
		cl.def("changeSpeeds", (bool (mrpt::nav::CRobot2NavInterface::*)(const class mrpt::kinematics::CVehicleVelCmd &)) &mrpt::nav::CRobot2NavInterface::changeSpeeds, "Sends a velocity command to the robot.\n The number components in each command depends on children classes of\n mrpt::kinematics::CVehicleVelCmd.\n One robot may accept one or more different CVehicleVelCmd classes.\n This method resets the watchdog timer (that may be or may be not\n implemented in a particular robotic platform) started with\n startWatchdog()\n \n\n false on any error.\n \n\n startWatchdog\n \n\nC++: mrpt::nav::CRobot2NavInterface::changeSpeeds(const class mrpt::kinematics::CVehicleVelCmd &) --> bool", pybind11::arg("vel_cmd"));
		cl.def("changeSpeedsNOP", (bool (mrpt::nav::CRobot2NavInterface::*)()) &mrpt::nav::CRobot2NavInterface::changeSpeedsNOP, "Just like changeSpeeds(), but will be called when the last velocity\n command is still the preferred solution,\n so there is no need to change that past command. The unique effect of\n this callback would be resetting the watchdog timer.\n \n\n false on any error.\n \n\n changeSpeeds(), startWatchdog()\n  \n\nC++: mrpt::nav::CRobot2NavInterface::changeSpeedsNOP() --> bool");
		cl.def("stop", [](mrpt::nav::CRobot2NavInterface &o) -> bool { return o.stop(); }, "");
		cl.def("stop", (bool (mrpt::nav::CRobot2NavInterface::*)(bool)) &mrpt::nav::CRobot2NavInterface::stop, "Stop the robot right now.\n  \n\n true if stop is due to some unexpected error.\n false if \"stop\" happens as part of a normal operation (e.g. target\n reached).\n \n\n false on any error.\n\nC++: mrpt::nav::CRobot2NavInterface::stop(bool) --> bool", pybind11::arg("isEmergencyStop"));
		cl.def("getEmergencyStopCmd", (class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> (mrpt::nav::CRobot2NavInterface::*)()) &mrpt::nav::CRobot2NavInterface::getEmergencyStopCmd, "Gets the emergency stop command for the current robot\n \n\n the emergency stop command\n\nC++: mrpt::nav::CRobot2NavInterface::getEmergencyStopCmd() --> class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>");
		cl.def("getStopCmd", (class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> (mrpt::nav::CRobot2NavInterface::*)()) &mrpt::nav::CRobot2NavInterface::getStopCmd, "Gets the emergency stop command for the current robot\n \n\n the emergency stop command\n\nC++: mrpt::nav::CRobot2NavInterface::getStopCmd() --> class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>");
		cl.def("getAlignCmd", (class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> (mrpt::nav::CRobot2NavInterface::*)(const double)) &mrpt::nav::CRobot2NavInterface::getAlignCmd, "Gets a motion command to make the robot to align with a given *relative*\n heading, without translating.\n Only for circular robots that can rotate in place; otherwise, return an\n empty smart pointer to indicate\n that the operation is not possible (this is what the default\n implementation does). \n\nC++: mrpt::nav::CRobot2NavInterface::getAlignCmd(const double) --> class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>", pybind11::arg("relative_heading_radians"));
		cl.def("startWatchdog", (bool (mrpt::nav::CRobot2NavInterface::*)(float)) &mrpt::nav::CRobot2NavInterface::startWatchdog, "Start the watchdog timer of the robot platform, if any, for maximum\n expected delay between consecutive calls to changeSpeeds().\n \n\n Period, in ms.\n \n\n false on any error. \n\nC++: mrpt::nav::CRobot2NavInterface::startWatchdog(float) --> bool", pybind11::arg("T_ms"));
		cl.def("stopWatchdog", (bool (mrpt::nav::CRobot2NavInterface::*)()) &mrpt::nav::CRobot2NavInterface::stopWatchdog, "Stop the watchdog timer.\n \n\n false on any error. \n startWatchdog \n\nC++: mrpt::nav::CRobot2NavInterface::stopWatchdog() --> bool");
		cl.def("senseObstacles", (bool (mrpt::nav::CRobot2NavInterface::*)(class mrpt::maps::CSimplePointsMap &, struct std::chrono::time_point<class mrpt::Clock, struct std::chrono::duration<long, struct std::ratio<1, 10000000> > > &)) &mrpt::nav::CRobot2NavInterface::senseObstacles, "Return the current set of obstacle points, as seen from the local\n coordinate frame of the robot.\n \n\n false on any error.\n \n\n  A representation of obstacles in robot-centric\n coordinates.\n \n\n  The timestamp for the read obstacles. Use\n mrpt::system::now() unless you have something more accurate.\n\nC++: mrpt::nav::CRobot2NavInterface::senseObstacles(class mrpt::maps::CSimplePointsMap &, struct std::chrono::time_point<class mrpt::Clock, struct std::chrono::duration<long, struct std::ratio<1, 10000000> > > &) --> bool", pybind11::arg("obstacles"), pybind11::arg("timestamp"));
		cl.def("sendNavigationStartEvent", (void (mrpt::nav::CRobot2NavInterface::*)()) &mrpt::nav::CRobot2NavInterface::sendNavigationStartEvent, "@{ \n\n Callback: Start of navigation command \n\nC++: mrpt::nav::CRobot2NavInterface::sendNavigationStartEvent() --> void");
		cl.def("sendNavigationEndEvent", (void (mrpt::nav::CRobot2NavInterface::*)()) &mrpt::nav::CRobot2NavInterface::sendNavigationEndEvent, "Callback: End of navigation command (reach of single goal, or final\n waypoint of waypoint list) \n\nC++: mrpt::nav::CRobot2NavInterface::sendNavigationEndEvent() --> void");
		cl.def("sendWaypointReachedEvent", (void (mrpt::nav::CRobot2NavInterface::*)(int, bool)) &mrpt::nav::CRobot2NavInterface::sendWaypointReachedEvent, "Callback: Reached an intermediary waypoint in waypoint list navigation.\n reached_nSkipped will be `true` if the waypoint was physically reached;\n `false` if it was actually \"skipped\".\n\nC++: mrpt::nav::CRobot2NavInterface::sendWaypointReachedEvent(int, bool) --> void", pybind11::arg("waypoint_index"), pybind11::arg("reached_nSkipped"));
		cl.def("sendNewWaypointTargetEvent", (void (mrpt::nav::CRobot2NavInterface::*)(int)) &mrpt::nav::CRobot2NavInterface::sendNewWaypointTargetEvent, "Callback: Heading towards a new intermediary/final waypoint in waypoint\n list navigation \n\nC++: mrpt::nav::CRobot2NavInterface::sendNewWaypointTargetEvent(int) --> void", pybind11::arg("waypoint_index"));
		cl.def("sendNavigationEndDueToErrorEvent", (void (mrpt::nav::CRobot2NavInterface::*)()) &mrpt::nav::CRobot2NavInterface::sendNavigationEndDueToErrorEvent, "Callback: Error asking sensory data from robot or sending motor\n commands. \n\nC++: mrpt::nav::CRobot2NavInterface::sendNavigationEndDueToErrorEvent() --> void");
		cl.def("sendWaySeemsBlockedEvent", (void (mrpt::nav::CRobot2NavInterface::*)()) &mrpt::nav::CRobot2NavInterface::sendWaySeemsBlockedEvent, "Callback: No progression made towards target for a predefined period of\n time. \n\nC++: mrpt::nav::CRobot2NavInterface::sendWaySeemsBlockedEvent() --> void");
		cl.def("sendApparentCollisionEvent", (void (mrpt::nav::CRobot2NavInterface::*)()) &mrpt::nav::CRobot2NavInterface::sendApparentCollisionEvent, "Callback: Apparent collision event (i.e. there is at least one obstacle\n point inside the robot shape) \n\nC++: mrpt::nav::CRobot2NavInterface::sendApparentCollisionEvent() --> void");
		cl.def("sendCannotGetCloserToBlockedTargetEvent", (void (mrpt::nav::CRobot2NavInterface::*)()) &mrpt::nav::CRobot2NavInterface::sendCannotGetCloserToBlockedTargetEvent, "Callback: Target seems to be blocked by an obstacle. \n\nC++: mrpt::nav::CRobot2NavInterface::sendCannotGetCloserToBlockedTargetEvent() --> void");
		cl.def("getNavigationTime", (double (mrpt::nav::CRobot2NavInterface::*)()) &mrpt::nav::CRobot2NavInterface::getNavigationTime, "Returns the number of seconds ellapsed since the constructor of this\n class was invoked, or since\n the last call of resetNavigationTimer(). This will be normally\n wall-clock time, except in simulators where this method will return\n simulation time. \n\nC++: mrpt::nav::CRobot2NavInterface::getNavigationTime() --> double");
		cl.def("resetNavigationTimer", (void (mrpt::nav::CRobot2NavInterface::*)()) &mrpt::nav::CRobot2NavInterface::resetNavigationTimer, "see getNavigationTime() \n\nC++: mrpt::nav::CRobot2NavInterface::resetNavigationTimer() --> void");
		cl.def("assign", (class mrpt::nav::CRobot2NavInterface & (mrpt::nav::CRobot2NavInterface::*)(const class mrpt::nav::CRobot2NavInterface &)) &mrpt::nav::CRobot2NavInterface::operator=, "C++: mrpt::nav::CRobot2NavInterface::operator=(const class mrpt::nav::CRobot2NavInterface &) --> class mrpt::nav::CRobot2NavInterface &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
