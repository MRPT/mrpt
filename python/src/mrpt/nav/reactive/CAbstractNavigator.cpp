#include <chrono>
#include <iterator>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/core/Clock.h>
#include <mrpt/kinematics/CVehicleVelCmd.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TTwist2D.h>
#include <mrpt/nav/reactive/CAbstractNavigator.h>
#include <mrpt/nav/reactive/CRobot2NavInterface.h>
#include <mrpt/poses/FrameTransformer.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTimeLogger.h>
#include <optional>
#include <ratio>
#include <sstream> // __str__
#include <string>
#include <string_view>
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

// mrpt::nav::CAbstractNavigator file:mrpt/nav/reactive/CAbstractNavigator.h line:58
struct PyCallBack_mrpt_nav_CAbstractNavigator : public mrpt::nav::CAbstractNavigator {
	using mrpt::nav::CAbstractNavigator::CAbstractNavigator;

	void loadConfigFile(const class mrpt::config::CConfigFileBase & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractNavigator *>(this), "loadConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CAbstractNavigator::loadConfigFile(a0);
	}
	void saveConfigFile(class mrpt::config::CConfigFileBase & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractNavigator *>(this), "saveConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CAbstractNavigator::saveConfigFile(a0);
	}
	void initialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractNavigator *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CAbstractNavigator::initialize\"");
	}
	void navigationStep() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractNavigator *>(this), "navigationStep");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CAbstractNavigator::navigationStep();
	}
	void cancel() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractNavigator *>(this), "cancel");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CAbstractNavigator::cancel();
	}
	void resume() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractNavigator *>(this), "resume");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CAbstractNavigator::resume();
	}
	void suspend() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractNavigator *>(this), "suspend");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CAbstractNavigator::suspend();
	}
	void resetNavError() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractNavigator *>(this), "resetNavError");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CAbstractNavigator::resetNavError();
	}
	void performNavigationStep() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractNavigator *>(this), "performNavigationStep");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CAbstractNavigator::performNavigationStep\"");
	}
	void onStartNewNavigation() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractNavigator *>(this), "onStartNewNavigation");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CAbstractNavigator::onStartNewNavigation\"");
	}
	void onNavigateCommandReceived() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractNavigator *>(this), "onNavigateCommandReceived");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CAbstractNavigator::onNavigateCommandReceived();
	}
	void updateCurrentPoseAndSpeeds() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractNavigator *>(this), "updateCurrentPoseAndSpeeds");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CAbstractNavigator::updateCurrentPoseAndSpeeds();
	}
	void performNavigationStepNavigating(bool a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractNavigator *>(this), "performNavigationStepNavigating");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CAbstractNavigator::performNavigationStepNavigating(a0);
	}
	void doEmergencyStop(const std::string & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractNavigator *>(this), "doEmergencyStop");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CAbstractNavigator::doEmergencyStop(a0);
	}
	bool changeSpeeds(const class mrpt::kinematics::CVehicleVelCmd & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractNavigator *>(this), "changeSpeeds");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CAbstractNavigator::changeSpeeds(a0);
	}
	bool changeSpeedsNOP() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractNavigator *>(this), "changeSpeedsNOP");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CAbstractNavigator::changeSpeedsNOP();
	}
	bool stop(bool a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractNavigator *>(this), "stop");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CAbstractNavigator::stop(a0);
	}
	bool checkHasReachedTarget(const double a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractNavigator *>(this), "checkHasReachedTarget");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CAbstractNavigator::checkHasReachedTarget(a0);
	}
	bool checkCollisionWithLatestObstacles(const struct mrpt::math::TPose2D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractNavigator *>(this), "checkCollisionWithLatestObstacles");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CAbstractNavigator::checkCollisionWithLatestObstacles(a0);
	}
};

// mrpt::nav::CAbstractNavigator::TNavigationParamsBase file:mrpt/nav/reactive/CAbstractNavigator.h line:103
struct PyCallBack_mrpt_nav_CAbstractNavigator_TNavigationParamsBase : public mrpt::nav::CAbstractNavigator::TNavigationParamsBase {
	using mrpt::nav::CAbstractNavigator::TNavigationParamsBase::TNavigationParamsBase;

	std::string getAsText() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractNavigator::TNavigationParamsBase *>(this), "getAsText");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"TNavigationParamsBase::getAsText\"");
	}
	bool isEqual(const struct mrpt::nav::CAbstractNavigator::TNavigationParamsBase & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractNavigator::TNavigationParamsBase *>(this), "isEqual");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"TNavigationParamsBase::isEqual\"");
	}
};

// mrpt::nav::CAbstractNavigator::TNavigationParams file:mrpt/nav/reactive/CAbstractNavigator.h line:116
struct PyCallBack_mrpt_nav_CAbstractNavigator_TNavigationParams : public mrpt::nav::CAbstractNavigator::TNavigationParams {
	using mrpt::nav::CAbstractNavigator::TNavigationParams::TNavigationParams;

	std::string getAsText() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractNavigator::TNavigationParams *>(this), "getAsText");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return TNavigationParams::getAsText();
	}
	bool isEqual(const struct mrpt::nav::CAbstractNavigator::TNavigationParamsBase & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractNavigator::TNavigationParams *>(this), "isEqual");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return TNavigationParams::isEqual(a0);
	}
};

// mrpt::nav::CAbstractNavigator::TAbstractNavigatorParams file:mrpt/nav/reactive/CAbstractNavigator.h line:223
struct PyCallBack_mrpt_nav_CAbstractNavigator_TAbstractNavigatorParams : public mrpt::nav::CAbstractNavigator::TAbstractNavigatorParams {
	using mrpt::nav::CAbstractNavigator::TAbstractNavigatorParams::TAbstractNavigatorParams;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractNavigator::TAbstractNavigatorParams *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TAbstractNavigatorParams::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractNavigator::TAbstractNavigatorParams *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TAbstractNavigatorParams::saveToConfigFile(a0, a1);
	}
};

void bind_mrpt_nav_reactive_CAbstractNavigator(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::nav::CAbstractNavigator file:mrpt/nav/reactive/CAbstractNavigator.h line:58
		pybind11::class_<mrpt::nav::CAbstractNavigator, std::shared_ptr<mrpt::nav::CAbstractNavigator>, PyCallBack_mrpt_nav_CAbstractNavigator> cl(M("mrpt::nav"), "CAbstractNavigator", "This is the base class for any reactive/planned navigation system. See\n derived classes.\n\n How to use:\n  - A class derived from `CRobot2NavInterface` with callbacks must be defined\n by the user and provided to the constructor.\n  - `navigationStep()` must be called periodically in order to effectively run\n the navigation. This method will internally call the callbacks to gather\n sensor data and robot positioning data.\n\n It implements the following state machine (see\n CAbstractNavigator::getCurrentState() ), taking into account the extensions\n described in CWaypointsNavigator\n  \n\n\n\n\n\n\n\n\n\n\n\n\n\n \n CWaypointsNavigator, CReactiveNavigationSystem, CRobot2NavInterface, all\n children classes\n  \n\n\n ");
		cl.def( pybind11::init<class mrpt::nav::CRobot2NavInterface &>(), pybind11::arg("robot_interface_impl") );


		pybind11::enum_<mrpt::nav::CAbstractNavigator::TState>(cl, "TState", pybind11::arithmetic(), "The different states for the navigation system. ")
			.value("IDLE", mrpt::nav::CAbstractNavigator::IDLE)
			.value("NAVIGATING", mrpt::nav::CAbstractNavigator::NAVIGATING)
			.value("SUSPENDED", mrpt::nav::CAbstractNavigator::SUSPENDED)
			.value("NAV_ERROR", mrpt::nav::CAbstractNavigator::NAV_ERROR)
			.export_values();


		pybind11::enum_<mrpt::nav::CAbstractNavigator::TErrorCode>(cl, "TErrorCode", pybind11::arithmetic(), "Explains the reason for the navigation error. ")
			.value("ERR_NONE", mrpt::nav::CAbstractNavigator::ERR_NONE)
			.value("ERR_EMERGENCY_STOP", mrpt::nav::CAbstractNavigator::ERR_EMERGENCY_STOP)
			.value("ERR_CANNOT_REACH_TARGET", mrpt::nav::CAbstractNavigator::ERR_CANNOT_REACH_TARGET)
			.value("ERR_OTHER", mrpt::nav::CAbstractNavigator::ERR_OTHER)
			.export_values();

		cl.def_readwrite("params_abstract_navigator", &mrpt::nav::CAbstractNavigator::params_abstract_navigator);
		cl.def_readwrite("m_navProfiler", &mrpt::nav::CAbstractNavigator::m_navProfiler);
		cl.def("loadConfigFile", (void (mrpt::nav::CAbstractNavigator::*)(const class mrpt::config::CConfigFileBase &)) &mrpt::nav::CAbstractNavigator::loadConfigFile, "Loads all params from a file. To be called before initialize().\n Each derived class *MUST* load its own parameters, and then call *ITS\n PARENT'S* overriden method to ensure all params are loaded. \n\nC++: mrpt::nav::CAbstractNavigator::loadConfigFile(const class mrpt::config::CConfigFileBase &) --> void", pybind11::arg("c"));
		cl.def("saveConfigFile", (void (mrpt::nav::CAbstractNavigator::*)(class mrpt::config::CConfigFileBase &) const) &mrpt::nav::CAbstractNavigator::saveConfigFile, "Saves all current options to a config file.\n Each derived class *MUST* save its own parameters, and then call *ITS\n PARENT'S* overriden method to ensure all params are saved. \n\nC++: mrpt::nav::CAbstractNavigator::saveConfigFile(class mrpt::config::CConfigFileBase &) const --> void", pybind11::arg("c"));
		cl.def("initialize", (void (mrpt::nav::CAbstractNavigator::*)()) &mrpt::nav::CAbstractNavigator::initialize, "Must be called before any other navigation command \n\nC++: mrpt::nav::CAbstractNavigator::initialize() --> void");
		cl.def("navigationStep", (void (mrpt::nav::CAbstractNavigator::*)()) &mrpt::nav::CAbstractNavigator::navigationStep, "This method must be called periodically in order to effectively run the\n navigation \n\nC++: mrpt::nav::CAbstractNavigator::navigationStep() --> void");
		cl.def("cancel", (void (mrpt::nav::CAbstractNavigator::*)()) &mrpt::nav::CAbstractNavigator::cancel, "Cancel current navegation. \n\nC++: mrpt::nav::CAbstractNavigator::cancel() --> void");
		cl.def("resume", (void (mrpt::nav::CAbstractNavigator::*)()) &mrpt::nav::CAbstractNavigator::resume, "Continues with suspended navigation. \n suspend \n\nC++: mrpt::nav::CAbstractNavigator::resume() --> void");
		cl.def("suspend", (void (mrpt::nav::CAbstractNavigator::*)()) &mrpt::nav::CAbstractNavigator::suspend, "Suspend current navegation. \n resume \n\nC++: mrpt::nav::CAbstractNavigator::suspend() --> void");
		cl.def("resetNavError", (void (mrpt::nav::CAbstractNavigator::*)()) &mrpt::nav::CAbstractNavigator::resetNavError, "Resets a `NAV_ERROR` state back to `IDLE` \n\nC++: mrpt::nav::CAbstractNavigator::resetNavError() --> void");
		cl.def("getCurrentState", (enum mrpt::nav::CAbstractNavigator::TState (mrpt::nav::CAbstractNavigator::*)() const) &mrpt::nav::CAbstractNavigator::getCurrentState, "Returns the current navigator state. \n\nC++: mrpt::nav::CAbstractNavigator::getCurrentState() const --> enum mrpt::nav::CAbstractNavigator::TState");
		cl.def("getErrorReason", (const struct mrpt::nav::CAbstractNavigator::TErrorReason & (mrpt::nav::CAbstractNavigator::*)() const) &mrpt::nav::CAbstractNavigator::getErrorReason, "In case of state=NAV_ERROR, this returns the reason for the error.\n Error state is reseted every time a new navigation starts with\n a call to navigate(), or when resetNavError() is called.\n\nC++: mrpt::nav::CAbstractNavigator::getErrorReason() const --> const struct mrpt::nav::CAbstractNavigator::TErrorReason &", pybind11::return_value_policy::automatic);
		cl.def("enableRethrowNavExceptions", (void (mrpt::nav::CAbstractNavigator::*)(const bool)) &mrpt::nav::CAbstractNavigator::enableRethrowNavExceptions, "By default, error exceptions on navigationStep() will dump an error\n message to the output logger interface. If rethrow is enabled\n (default=false), the error message will be reported as well, but\n exceptions will be re-thrown.\n\nC++: mrpt::nav::CAbstractNavigator::enableRethrowNavExceptions(const bool) --> void", pybind11::arg("enable"));
		cl.def("isRethrowNavExceptionsEnabled", (bool (mrpt::nav::CAbstractNavigator::*)() const) &mrpt::nav::CAbstractNavigator::isRethrowNavExceptionsEnabled, "C++: mrpt::nav::CAbstractNavigator::isRethrowNavExceptionsEnabled() const --> bool");
		cl.def("getDelaysTimeLogger", (const class mrpt::system::CTimeLogger & (mrpt::nav::CAbstractNavigator::*)() const) &mrpt::nav::CAbstractNavigator::getDelaysTimeLogger, "Gives access to a const-ref to the internal time logger used to estimate\n delays \n\n getTimeLogger() in derived classes \n\nC++: mrpt::nav::CAbstractNavigator::getDelaysTimeLogger() const --> const class mrpt::system::CTimeLogger &", pybind11::return_value_policy::automatic);

		{ // mrpt::nav::CAbstractNavigator::TargetInfo file:mrpt/nav/reactive/CAbstractNavigator.h line:68
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::nav::CAbstractNavigator::TargetInfo, std::shared_ptr<mrpt::nav::CAbstractNavigator::TargetInfo>> cl(enclosing_class, "TargetInfo", "Individual target info in CAbstractNavigator::TNavigationParamsBase and\n derived classes ");
			cl.def( pybind11::init( [](){ return new mrpt::nav::CAbstractNavigator::TargetInfo(); } ) );
			cl.def( pybind11::init( [](mrpt::nav::CAbstractNavigator::TargetInfo const &o){ return new mrpt::nav::CAbstractNavigator::TargetInfo(o); } ) );
			cl.def_readwrite("target_coords", &mrpt::nav::CAbstractNavigator::TargetInfo::target_coords);
			cl.def_readwrite("target_frame_id", &mrpt::nav::CAbstractNavigator::TargetInfo::target_frame_id);
			cl.def_readwrite("targetAllowedDistance", &mrpt::nav::CAbstractNavigator::TargetInfo::targetAllowedDistance);
			cl.def_readwrite("targetIsRelative", &mrpt::nav::CAbstractNavigator::TargetInfo::targetIsRelative);
			cl.def_readwrite("targetDesiredRelSpeed", &mrpt::nav::CAbstractNavigator::TargetInfo::targetDesiredRelSpeed);
			cl.def_readwrite("targetIsIntermediaryWaypoint", &mrpt::nav::CAbstractNavigator::TargetInfo::targetIsIntermediaryWaypoint);
			cl.def("getAsText", (std::string (mrpt::nav::CAbstractNavigator::TargetInfo::*)() const) &mrpt::nav::CAbstractNavigator::TargetInfo::getAsText, "Gets navigation params as a human-readable format \n\nC++: mrpt::nav::CAbstractNavigator::TargetInfo::getAsText() const --> std::string");
			cl.def("__eq__", (bool (mrpt::nav::CAbstractNavigator::TargetInfo::*)(const struct mrpt::nav::CAbstractNavigator::TargetInfo &) const) &mrpt::nav::CAbstractNavigator::TargetInfo::operator==, "C++: mrpt::nav::CAbstractNavigator::TargetInfo::operator==(const struct mrpt::nav::CAbstractNavigator::TargetInfo &) const --> bool", pybind11::arg("o"));
			cl.def("__ne__", (bool (mrpt::nav::CAbstractNavigator::TargetInfo::*)(const struct mrpt::nav::CAbstractNavigator::TargetInfo &) const) &mrpt::nav::CAbstractNavigator::TargetInfo::operator!=, "C++: mrpt::nav::CAbstractNavigator::TargetInfo::operator!=(const struct mrpt::nav::CAbstractNavigator::TargetInfo &) const --> bool", pybind11::arg("o"));
			cl.def("assign", (struct mrpt::nav::CAbstractNavigator::TargetInfo & (mrpt::nav::CAbstractNavigator::TargetInfo::*)(const struct mrpt::nav::CAbstractNavigator::TargetInfo &)) &mrpt::nav::CAbstractNavigator::TargetInfo::operator=, "C++: mrpt::nav::CAbstractNavigator::TargetInfo::operator=(const struct mrpt::nav::CAbstractNavigator::TargetInfo &) --> struct mrpt::nav::CAbstractNavigator::TargetInfo &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::nav::CAbstractNavigator::TNavigationParamsBase file:mrpt/nav/reactive/CAbstractNavigator.h line:103
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::nav::CAbstractNavigator::TNavigationParamsBase, std::shared_ptr<mrpt::nav::CAbstractNavigator::TNavigationParamsBase>, PyCallBack_mrpt_nav_CAbstractNavigator_TNavigationParamsBase> cl(enclosing_class, "TNavigationParamsBase", "Base for all high-level navigation commands. See derived classes ");
			cl.def(pybind11::init<PyCallBack_mrpt_nav_CAbstractNavigator_TNavigationParamsBase const &>());
			cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_nav_CAbstractNavigator_TNavigationParamsBase(); } ) );
			cl.def("getAsText", (std::string (mrpt::nav::CAbstractNavigator::TNavigationParamsBase::*)() const) &mrpt::nav::CAbstractNavigator::TNavigationParamsBase::getAsText, "Gets navigation params as a human-readable format \n\nC++: mrpt::nav::CAbstractNavigator::TNavigationParamsBase::getAsText() const --> std::string");
			cl.def("assign", (struct mrpt::nav::CAbstractNavigator::TNavigationParamsBase & (mrpt::nav::CAbstractNavigator::TNavigationParamsBase::*)(const struct mrpt::nav::CAbstractNavigator::TNavigationParamsBase &)) &mrpt::nav::CAbstractNavigator::TNavigationParamsBase::operator=, "C++: mrpt::nav::CAbstractNavigator::TNavigationParamsBase::operator=(const struct mrpt::nav::CAbstractNavigator::TNavigationParamsBase &) --> struct mrpt::nav::CAbstractNavigator::TNavigationParamsBase &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::nav::CAbstractNavigator::TNavigationParams file:mrpt/nav/reactive/CAbstractNavigator.h line:116
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::nav::CAbstractNavigator::TNavigationParams, std::shared_ptr<mrpt::nav::CAbstractNavigator::TNavigationParams>, PyCallBack_mrpt_nav_CAbstractNavigator_TNavigationParams, mrpt::nav::CAbstractNavigator::TNavigationParamsBase> cl(enclosing_class, "TNavigationParams", "The struct for configuring navigation requests. Used in\n CAbstractPTGBasedReactive::navigate() ");
			cl.def( pybind11::init( [](){ return new mrpt::nav::CAbstractNavigator::TNavigationParams(); }, [](){ return new PyCallBack_mrpt_nav_CAbstractNavigator_TNavigationParams(); } ) );
			cl.def_readwrite("target", &mrpt::nav::CAbstractNavigator::TNavigationParams::target);
			cl.def("getAsText", (std::string (mrpt::nav::CAbstractNavigator::TNavigationParams::*)() const) &mrpt::nav::CAbstractNavigator::TNavigationParams::getAsText, "Gets navigation params as a human-readable format \n\nC++: mrpt::nav::CAbstractNavigator::TNavigationParams::getAsText() const --> std::string");
		}

		{ // mrpt::nav::CAbstractNavigator::TErrorReason file:mrpt/nav/reactive/CAbstractNavigator.h line:189
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::nav::CAbstractNavigator::TErrorReason, std::shared_ptr<mrpt::nav::CAbstractNavigator::TErrorReason>> cl(enclosing_class, "TErrorReason", "");
			cl.def( pybind11::init( [](){ return new mrpt::nav::CAbstractNavigator::TErrorReason(); } ) );
			cl.def( pybind11::init( [](mrpt::nav::CAbstractNavigator::TErrorReason const &o){ return new mrpt::nav::CAbstractNavigator::TErrorReason(o); } ) );
			cl.def_readwrite("error_code", &mrpt::nav::CAbstractNavigator::TErrorReason::error_code);
			cl.def_readwrite("error_msg", &mrpt::nav::CAbstractNavigator::TErrorReason::error_msg);
			cl.def("assign", (struct mrpt::nav::CAbstractNavigator::TErrorReason & (mrpt::nav::CAbstractNavigator::TErrorReason::*)(const struct mrpt::nav::CAbstractNavigator::TErrorReason &)) &mrpt::nav::CAbstractNavigator::TErrorReason::operator=, "C++: mrpt::nav::CAbstractNavigator::TErrorReason::operator=(const struct mrpt::nav::CAbstractNavigator::TErrorReason &) --> struct mrpt::nav::CAbstractNavigator::TErrorReason &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::nav::CAbstractNavigator::TAbstractNavigatorParams file:mrpt/nav/reactive/CAbstractNavigator.h line:223
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::nav::CAbstractNavigator::TAbstractNavigatorParams, std::shared_ptr<mrpt::nav::CAbstractNavigator::TAbstractNavigatorParams>, PyCallBack_mrpt_nav_CAbstractNavigator_TAbstractNavigatorParams, mrpt::config::CLoadableOptions> cl(enclosing_class, "TAbstractNavigatorParams", "@}");
			cl.def( pybind11::init( [](){ return new mrpt::nav::CAbstractNavigator::TAbstractNavigatorParams(); }, [](){ return new PyCallBack_mrpt_nav_CAbstractNavigator_TAbstractNavigatorParams(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_nav_CAbstractNavigator_TAbstractNavigatorParams const &o){ return new PyCallBack_mrpt_nav_CAbstractNavigator_TAbstractNavigatorParams(o); } ) );
			cl.def( pybind11::init( [](mrpt::nav::CAbstractNavigator::TAbstractNavigatorParams const &o){ return new mrpt::nav::CAbstractNavigator::TAbstractNavigatorParams(o); } ) );
			cl.def_readwrite("dist_to_target_for_sending_event", &mrpt::nav::CAbstractNavigator::TAbstractNavigatorParams::dist_to_target_for_sending_event);
			cl.def_readwrite("alarm_seems_not_approaching_target_timeout", &mrpt::nav::CAbstractNavigator::TAbstractNavigatorParams::alarm_seems_not_approaching_target_timeout);
			cl.def_readwrite("dist_check_target_is_blocked", &mrpt::nav::CAbstractNavigator::TAbstractNavigatorParams::dist_check_target_is_blocked);
			cl.def_readwrite("hysteresis_check_target_is_blocked", &mrpt::nav::CAbstractNavigator::TAbstractNavigatorParams::hysteresis_check_target_is_blocked);
			cl.def("loadFromConfigFile", (void (mrpt::nav::CAbstractNavigator::TAbstractNavigatorParams::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::nav::CAbstractNavigator::TAbstractNavigatorParams::loadFromConfigFile, "C++: mrpt::nav::CAbstractNavigator::TAbstractNavigatorParams::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("c"), pybind11::arg("s"));
			cl.def("saveToConfigFile", (void (mrpt::nav::CAbstractNavigator::TAbstractNavigatorParams::*)(class mrpt::config::CConfigFileBase &, const std::string &) const) &mrpt::nav::CAbstractNavigator::TAbstractNavigatorParams::saveToConfigFile, "C++: mrpt::nav::CAbstractNavigator::TAbstractNavigatorParams::saveToConfigFile(class mrpt::config::CConfigFileBase &, const std::string &) const --> void", pybind11::arg("c"), pybind11::arg("s"));
			cl.def("assign", (struct mrpt::nav::CAbstractNavigator::TAbstractNavigatorParams & (mrpt::nav::CAbstractNavigator::TAbstractNavigatorParams::*)(const struct mrpt::nav::CAbstractNavigator::TAbstractNavigatorParams &)) &mrpt::nav::CAbstractNavigator::TAbstractNavigatorParams::operator=, "C++: mrpt::nav::CAbstractNavigator::TAbstractNavigatorParams::operator=(const struct mrpt::nav::CAbstractNavigator::TAbstractNavigatorParams &) --> struct mrpt::nav::CAbstractNavigator::TAbstractNavigatorParams &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
}
