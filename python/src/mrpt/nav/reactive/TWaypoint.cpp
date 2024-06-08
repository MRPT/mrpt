#include <chrono>
#include <iterator>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/Clock.h>
#include <mrpt/img/TColor.h>
#include <mrpt/kinematics/CVehicleVelCmd.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TTwist2D.h>
#include <mrpt/nav/reactive/CAbstractNavigator.h>
#include <mrpt/nav/reactive/CRobot2NavInterface.h>
#include <mrpt/nav/reactive/CWaypointsNavigator.h>
#include <mrpt/nav/reactive/TWaypoint.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/poses/CPointPDF.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DQuatPDF.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ratio>
#include <sstream> // __str__
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

// mrpt::nav::CWaypointsNavigator file:mrpt/nav/reactive/CWaypointsNavigator.h line:38
struct PyCallBack_mrpt_nav_CWaypointsNavigator : public mrpt::nav::CWaypointsNavigator {
	using mrpt::nav::CWaypointsNavigator::CWaypointsNavigator;

	void navigationStep() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CWaypointsNavigator *>(this), "navigationStep");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CWaypointsNavigator::navigationStep();
	}
	void cancel() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CWaypointsNavigator *>(this), "cancel");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CWaypointsNavigator::cancel();
	}
	void navigateWaypoints(const struct mrpt::nav::TWaypointSequence & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CWaypointsNavigator *>(this), "navigateWaypoints");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CWaypointsNavigator::navigateWaypoints(a0);
	}
	void getWaypointNavStatus(struct mrpt::nav::TWaypointStatusSequence & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CWaypointsNavigator *>(this), "getWaypointNavStatus");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CWaypointsNavigator::getWaypointNavStatus(a0);
	}
	void loadConfigFile(const class mrpt::config::CConfigFileBase & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CWaypointsNavigator *>(this), "loadConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CWaypointsNavigator::loadConfigFile(a0);
	}
	void saveConfigFile(class mrpt::config::CConfigFileBase & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CWaypointsNavigator *>(this), "saveConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CWaypointsNavigator::saveConfigFile(a0);
	}
	bool impl_waypoint_is_reachable(const struct mrpt::math::TPoint2D_<double> & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CWaypointsNavigator *>(this), "impl_waypoint_is_reachable");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CWaypointsNavigator::impl_waypoint_is_reachable\"");
	}
	void onStartNewNavigation() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CWaypointsNavigator *>(this), "onStartNewNavigation");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CWaypointsNavigator::onStartNewNavigation();
	}
	void onNavigateCommandReceived() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CWaypointsNavigator *>(this), "onNavigateCommandReceived");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CWaypointsNavigator::onNavigateCommandReceived();
	}
	bool checkHasReachedTarget(const double a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CWaypointsNavigator *>(this), "checkHasReachedTarget");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CWaypointsNavigator::checkHasReachedTarget(a0);
	}
	void waypoints_navigationStep() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CWaypointsNavigator *>(this), "waypoints_navigationStep");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CWaypointsNavigator::waypoints_navigationStep();
	}
	void initialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CWaypointsNavigator *>(this), "initialize");
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
	void resume() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CWaypointsNavigator *>(this), "resume");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CWaypointsNavigator *>(this), "suspend");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CWaypointsNavigator *>(this), "resetNavError");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CWaypointsNavigator *>(this), "performNavigationStep");
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
	void updateCurrentPoseAndSpeeds() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CWaypointsNavigator *>(this), "updateCurrentPoseAndSpeeds");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CWaypointsNavigator *>(this), "performNavigationStepNavigating");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CWaypointsNavigator *>(this), "doEmergencyStop");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CWaypointsNavigator *>(this), "changeSpeeds");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CWaypointsNavigator *>(this), "changeSpeedsNOP");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CWaypointsNavigator *>(this), "stop");
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
	bool checkCollisionWithLatestObstacles(const struct mrpt::math::TPose2D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CWaypointsNavigator *>(this), "checkCollisionWithLatestObstacles");
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

// mrpt::nav::CWaypointsNavigator::TNavigationParamsWaypoints file:mrpt/nav/reactive/CWaypointsNavigator.h line:43
struct PyCallBack_mrpt_nav_CWaypointsNavigator_TNavigationParamsWaypoints : public mrpt::nav::CWaypointsNavigator::TNavigationParamsWaypoints {
	using mrpt::nav::CWaypointsNavigator::TNavigationParamsWaypoints::TNavigationParamsWaypoints;

	std::string getAsText() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CWaypointsNavigator::TNavigationParamsWaypoints *>(this), "getAsText");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return TNavigationParamsWaypoints::getAsText();
	}
	bool isEqual(const struct mrpt::nav::CAbstractNavigator::TNavigationParamsBase & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CWaypointsNavigator::TNavigationParamsWaypoints *>(this), "isEqual");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return TNavigationParamsWaypoints::isEqual(a0);
	}
};

// mrpt::nav::CWaypointsNavigator::TWaypointsNavigatorParams file:mrpt/nav/reactive/CWaypointsNavigator.h line:120
struct PyCallBack_mrpt_nav_CWaypointsNavigator_TWaypointsNavigatorParams : public mrpt::nav::CWaypointsNavigator::TWaypointsNavigatorParams {
	using mrpt::nav::CWaypointsNavigator::TWaypointsNavigatorParams::TWaypointsNavigatorParams;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CWaypointsNavigator::TWaypointsNavigatorParams *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TWaypointsNavigatorParams::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CWaypointsNavigator::TWaypointsNavigatorParams *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TWaypointsNavigatorParams::saveToConfigFile(a0, a1);
	}
};

void bind_mrpt_nav_reactive_TWaypoint(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::nav::TWaypointsRenderingParams file:mrpt/nav/reactive/TWaypoint.h line:92
		pybind11::class_<mrpt::nav::TWaypointsRenderingParams, std::shared_ptr<mrpt::nav::TWaypointsRenderingParams>> cl(M("mrpt::nav"), "TWaypointsRenderingParams", "used in getAsOpenglVisualization() ");
		cl.def( pybind11::init( [](){ return new mrpt::nav::TWaypointsRenderingParams(); } ) );
		cl.def( pybind11::init( [](mrpt::nav::TWaypointsRenderingParams const &o){ return new mrpt::nav::TWaypointsRenderingParams(o); } ) );
		cl.def_readwrite("outer_radius", &mrpt::nav::TWaypointsRenderingParams::outer_radius);
		cl.def_readwrite("inner_radius", &mrpt::nav::TWaypointsRenderingParams::inner_radius);
		cl.def_readwrite("outer_radius_non_skippable", &mrpt::nav::TWaypointsRenderingParams::outer_radius_non_skippable);
		cl.def_readwrite("inner_radius_non_skippable", &mrpt::nav::TWaypointsRenderingParams::inner_radius_non_skippable);
		cl.def_readwrite("outer_radius_reached", &mrpt::nav::TWaypointsRenderingParams::outer_radius_reached);
		cl.def_readwrite("inner_radius_reached", &mrpt::nav::TWaypointsRenderingParams::inner_radius_reached);
		cl.def_readwrite("heading_arrow_len", &mrpt::nav::TWaypointsRenderingParams::heading_arrow_len);
		cl.def_readwrite("color_regular", &mrpt::nav::TWaypointsRenderingParams::color_regular);
		cl.def_readwrite("color_current_goal", &mrpt::nav::TWaypointsRenderingParams::color_current_goal);
		cl.def_readwrite("color_reached", &mrpt::nav::TWaypointsRenderingParams::color_reached);
		cl.def_readwrite("show_labels", &mrpt::nav::TWaypointsRenderingParams::show_labels);
		cl.def("assign", (struct mrpt::nav::TWaypointsRenderingParams & (mrpt::nav::TWaypointsRenderingParams::*)(const struct mrpt::nav::TWaypointsRenderingParams &)) &mrpt::nav::TWaypointsRenderingParams::operator=, "C++: mrpt::nav::TWaypointsRenderingParams::operator=(const struct mrpt::nav::TWaypointsRenderingParams &) --> struct mrpt::nav::TWaypointsRenderingParams &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::nav::TWaypointSequence file:mrpt/nav/reactive/TWaypoint.h line:109
		pybind11::class_<mrpt::nav::TWaypointSequence, std::shared_ptr<mrpt::nav::TWaypointSequence>> cl(M("mrpt::nav"), "TWaypointSequence", "The struct for requesting navigation requests for a sequence of waypoints.\n Used in CWaypointsNavigator::navigateWaypoints().\n Users can directly fill in the list of waypoints manipulating the public\n field `waypoints`.\n  \n");
		cl.def( pybind11::init( [](){ return new mrpt::nav::TWaypointSequence(); } ) );
		cl.def( pybind11::init( [](mrpt::nav::TWaypointSequence const &o){ return new mrpt::nav::TWaypointSequence(o); } ) );
		cl.def_readwrite("waypoints", &mrpt::nav::TWaypointSequence::waypoints);
		cl.def("clear", (void (mrpt::nav::TWaypointSequence::*)()) &mrpt::nav::TWaypointSequence::clear, "C++: mrpt::nav::TWaypointSequence::clear() --> void");
		cl.def("getAsText", (std::string (mrpt::nav::TWaypointSequence::*)() const) &mrpt::nav::TWaypointSequence::getAsText, "Gets navigation params as a human-readable format \n\nC++: mrpt::nav::TWaypointSequence::getAsText() const --> std::string");
		cl.def("getAsOpenglVisualization", [](mrpt::nav::TWaypointSequence const &o, class mrpt::opengl::CSetOfObjects & a0) -> void { return o.getAsOpenglVisualization(a0); }, "", pybind11::arg("obj"));
		cl.def("getAsOpenglVisualization", (void (mrpt::nav::TWaypointSequence::*)(class mrpt::opengl::CSetOfObjects &, const struct mrpt::nav::TWaypointsRenderingParams &) const) &mrpt::nav::TWaypointSequence::getAsOpenglVisualization, "Renders the sequence of waypoints (previous contents of `obj` are\n cleared) \n\nC++: mrpt::nav::TWaypointSequence::getAsOpenglVisualization(class mrpt::opengl::CSetOfObjects &, const struct mrpt::nav::TWaypointsRenderingParams &) const --> void", pybind11::arg("obj"), pybind11::arg("params"));
		cl.def("save", (void (mrpt::nav::TWaypointSequence::*)(class mrpt::config::CConfigFileBase &, const std::string &) const) &mrpt::nav::TWaypointSequence::save, "Saves waypoints to a config file section \n\nC++: mrpt::nav::TWaypointSequence::save(class mrpt::config::CConfigFileBase &, const std::string &) const --> void", pybind11::arg("c"), pybind11::arg("s"));
		cl.def("load", (void (mrpt::nav::TWaypointSequence::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::nav::TWaypointSequence::load, "Loads waypoints to a config file section \n\nC++: mrpt::nav::TWaypointSequence::load(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("c"), pybind11::arg("s"));
		cl.def("assign", (struct mrpt::nav::TWaypointSequence & (mrpt::nav::TWaypointSequence::*)(const struct mrpt::nav::TWaypointSequence &)) &mrpt::nav::TWaypointSequence::operator=, "C++: mrpt::nav::TWaypointSequence::operator=(const struct mrpt::nav::TWaypointSequence &) --> struct mrpt::nav::TWaypointSequence &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::nav::TWaypointStatus file:mrpt/nav/reactive/TWaypoint.h line:131
		pybind11::class_<mrpt::nav::TWaypointStatus, std::shared_ptr<mrpt::nav::TWaypointStatus>, mrpt::nav::TWaypoint> cl(M("mrpt::nav"), "TWaypointStatus", "A waypoint with an execution status. ");
		cl.def( pybind11::init( [](){ return new mrpt::nav::TWaypointStatus(); } ) );
		cl.def( pybind11::init( [](mrpt::nav::TWaypointStatus const &o){ return new mrpt::nav::TWaypointStatus(o); } ) );
		cl.def_readwrite("reached", &mrpt::nav::TWaypointStatus::reached);
		cl.def_readwrite("skipped", &mrpt::nav::TWaypointStatus::skipped);
		cl.def_readwrite("timestamp_reach", &mrpt::nav::TWaypointStatus::timestamp_reach);
		cl.def_readwrite("counter_seen_reachable", &mrpt::nav::TWaypointStatus::counter_seen_reachable);
		cl.def_readwrite("user_status_data", &mrpt::nav::TWaypointStatus::user_status_data);
		cl.def("assign", (struct mrpt::nav::TWaypointStatus & (mrpt::nav::TWaypointStatus::*)(const struct mrpt::nav::TWaypoint &)) &mrpt::nav::TWaypointStatus::operator=, "Only copies the base class TWaypoint data fields \n\nC++: mrpt::nav::TWaypointStatus::operator=(const struct mrpt::nav::TWaypoint &) --> struct mrpt::nav::TWaypointStatus &", pybind11::return_value_policy::automatic, pybind11::arg("wp"));
		cl.def("getAsText", (std::string (mrpt::nav::TWaypointStatus::*)() const) &mrpt::nav::TWaypointStatus::getAsText, "Gets navigation params as a human-readable format \n\nC++: mrpt::nav::TWaypointStatus::getAsText() const --> std::string");
		cl.def("assign", (struct mrpt::nav::TWaypointStatus & (mrpt::nav::TWaypointStatus::*)(const struct mrpt::nav::TWaypointStatus &)) &mrpt::nav::TWaypointStatus::operator=, "C++: mrpt::nav::TWaypointStatus::operator=(const struct mrpt::nav::TWaypointStatus &) --> struct mrpt::nav::TWaypointStatus &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::nav::TWaypointStatusSequence file:mrpt/nav/reactive/TWaypoint.h line:165
		pybind11::class_<mrpt::nav::TWaypointStatusSequence, std::shared_ptr<mrpt::nav::TWaypointStatusSequence>> cl(M("mrpt::nav"), "TWaypointStatusSequence", "The struct for querying the status of waypoints navigation. Used in\n CWaypointsNavigator::getWaypointNavStatus().\n  \n");
		cl.def( pybind11::init( [](){ return new mrpt::nav::TWaypointStatusSequence(); } ) );
		cl.def( pybind11::init( [](mrpt::nav::TWaypointStatusSequence const &o){ return new mrpt::nav::TWaypointStatusSequence(o); } ) );
		cl.def_readwrite("waypoints", &mrpt::nav::TWaypointStatusSequence::waypoints);
		cl.def_readwrite("timestamp_nav_started", &mrpt::nav::TWaypointStatusSequence::timestamp_nav_started);
		cl.def_readwrite("final_goal_reached", &mrpt::nav::TWaypointStatusSequence::final_goal_reached);
		cl.def_readwrite("waypoint_index_current_goal", &mrpt::nav::TWaypointStatusSequence::waypoint_index_current_goal);
		cl.def_readwrite("last_robot_pose", &mrpt::nav::TWaypointStatusSequence::last_robot_pose);
		cl.def("getAsText", (std::string (mrpt::nav::TWaypointStatusSequence::*)() const) &mrpt::nav::TWaypointStatusSequence::getAsText, "Ctor with default values \n\n Gets navigation params as a human-readable format \n\nC++: mrpt::nav::TWaypointStatusSequence::getAsText() const --> std::string");
		cl.def("getAsOpenglVisualization", [](mrpt::nav::TWaypointStatusSequence const &o, class mrpt::opengl::CSetOfObjects & a0) -> void { return o.getAsOpenglVisualization(a0); }, "", pybind11::arg("obj"));
		cl.def("getAsOpenglVisualization", (void (mrpt::nav::TWaypointStatusSequence::*)(class mrpt::opengl::CSetOfObjects &, const struct mrpt::nav::TWaypointsRenderingParams &) const) &mrpt::nav::TWaypointStatusSequence::getAsOpenglVisualization, "Renders the sequence of waypoints (previous contents of `obj` are\n cleared) \n\nC++: mrpt::nav::TWaypointStatusSequence::getAsOpenglVisualization(class mrpt::opengl::CSetOfObjects &, const struct mrpt::nav::TWaypointsRenderingParams &) const --> void", pybind11::arg("obj"), pybind11::arg("params"));
		cl.def("assign", (struct mrpt::nav::TWaypointStatusSequence & (mrpt::nav::TWaypointStatusSequence::*)(const struct mrpt::nav::TWaypointStatusSequence &)) &mrpt::nav::TWaypointStatusSequence::operator=, "C++: mrpt::nav::TWaypointStatusSequence::operator=(const struct mrpt::nav::TWaypointStatusSequence &) --> struct mrpt::nav::TWaypointStatusSequence &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::nav::CWaypointsNavigator file:mrpt/nav/reactive/CWaypointsNavigator.h line:38
		pybind11::class_<mrpt::nav::CWaypointsNavigator, std::shared_ptr<mrpt::nav::CWaypointsNavigator>, PyCallBack_mrpt_nav_CWaypointsNavigator, mrpt::nav::CAbstractNavigator> cl(M("mrpt::nav"), "CWaypointsNavigator", "This class extends `CAbstractNavigator` with the capability of following a\n list of waypoints. By default, waypoints are followed one by one,\n  but, if they are tagged with `allow_skip=true` **and** the derived navigator\n class supports it, the navigator may choose to skip some to\n  make a smoother, safer and shorter navigation.\n\n Waypoints have an optional `target_heading` field, which will be honored only\n for waypoints that are skipped, and if the underlying robot\n interface supports the pure-rotation methods.\n\n Notes on navigation status and event dispatchment:\n  - Navigation state may briefly pass by the `IDLE` status between a waypoint\n is reached and a new navigation command is issued towards the next waypoint.\n  - `sendNavigationEndEvent()` will be called only when the last waypoint is\n reached.\n  - Reaching an intermediary waypoint (or skipping it if considered so by the\n navigator) generates a call to `sendWaypointReachedEvent()` instead.\n\n \n Base class CAbstractNavigator, CWaypointsNavigator::navigateWaypoints(),\n and derived classes.\n  \n\n\n ");
		cl.def( pybind11::init<class mrpt::nav::CRobot2NavInterface &>(), pybind11::arg("robot_interface_impl") );

		cl.def_readwrite("params_waypoints_navigator", &mrpt::nav::CWaypointsNavigator::params_waypoints_navigator);
		cl.def("navigationStep", (void (mrpt::nav::CWaypointsNavigator::*)()) &mrpt::nav::CWaypointsNavigator::navigationStep, "C++: mrpt::nav::CWaypointsNavigator::navigationStep() --> void");
		cl.def("cancel", (void (mrpt::nav::CWaypointsNavigator::*)()) &mrpt::nav::CWaypointsNavigator::cancel, "Cancel current navegation. \n\nC++: mrpt::nav::CWaypointsNavigator::cancel() --> void");
		cl.def("navigateWaypoints", (void (mrpt::nav::CWaypointsNavigator::*)(const struct mrpt::nav::TWaypointSequence &)) &mrpt::nav::CWaypointsNavigator::navigateWaypoints, "Waypoint navigation request. This immediately cancels any other previous\n on-going navigation.\n \n\n CAbstractNavigator::navigate() for single waypoint navigation\n requests.\n\nC++: mrpt::nav::CWaypointsNavigator::navigateWaypoints(const struct mrpt::nav::TWaypointSequence &) --> void", pybind11::arg("nav_request"));
		cl.def("getWaypointNavStatus", (void (mrpt::nav::CWaypointsNavigator::*)(struct mrpt::nav::TWaypointStatusSequence &) const) &mrpt::nav::CWaypointsNavigator::getWaypointNavStatus, "Get a copy of the control structure which describes the progress status\n of the waypoint navigation. \n\nC++: mrpt::nav::CWaypointsNavigator::getWaypointNavStatus(struct mrpt::nav::TWaypointStatusSequence &) const --> void", pybind11::arg("out_nav_status"));
		cl.def("getWaypointNavStatus", (struct mrpt::nav::TWaypointStatusSequence (mrpt::nav::CWaypointsNavigator::*)() const) &mrpt::nav::CWaypointsNavigator::getWaypointNavStatus, "Get a copy of the control structure which describes the progress status\n of the waypoint navigation.\n \n   \n\nC++: mrpt::nav::CWaypointsNavigator::getWaypointNavStatus() const --> struct mrpt::nav::TWaypointStatusSequence");
		cl.def("beginWaypointsAccess", (struct mrpt::nav::TWaypointStatusSequence & (mrpt::nav::CWaypointsNavigator::*)()) &mrpt::nav::CWaypointsNavigator::beginWaypointsAccess, "Gets a write-enabled reference to the list of waypoints, simultanously\n acquiring the critical section mutex.\n Caller must call endWaypointsAccess() when done editing the waypoints.\n\nC++: mrpt::nav::CWaypointsNavigator::beginWaypointsAccess() --> struct mrpt::nav::TWaypointStatusSequence &", pybind11::return_value_policy::automatic);
		cl.def("endWaypointsAccess", (void (mrpt::nav::CWaypointsNavigator::*)()) &mrpt::nav::CWaypointsNavigator::endWaypointsAccess, "Must be called after beginWaypointsAccess() \n\nC++: mrpt::nav::CWaypointsNavigator::endWaypointsAccess() --> void");
		cl.def("isRelativePointReachable", (bool (mrpt::nav::CWaypointsNavigator::*)(const struct mrpt::math::TPoint2D_<double> &) const) &mrpt::nav::CWaypointsNavigator::isRelativePointReachable, "Returns `true` if, according to the information gathered at the last\n navigation step,\n there is a free path to the given point; `false` otherwise: if way is\n blocked or there is missing information,\n the point is out of range for the existing PTGs, etc. \n\nC++: mrpt::nav::CWaypointsNavigator::isRelativePointReachable(const struct mrpt::math::TPoint2D_<double> &) const --> bool", pybind11::arg("wp_local_wrt_robot"));
		cl.def("loadConfigFile", (void (mrpt::nav::CWaypointsNavigator::*)(const class mrpt::config::CConfigFileBase &)) &mrpt::nav::CWaypointsNavigator::loadConfigFile, "C++: mrpt::nav::CWaypointsNavigator::loadConfigFile(const class mrpt::config::CConfigFileBase &) --> void", pybind11::arg("c"));
		cl.def("saveConfigFile", (void (mrpt::nav::CWaypointsNavigator::*)(class mrpt::config::CConfigFileBase &) const) &mrpt::nav::CWaypointsNavigator::saveConfigFile, "C++: mrpt::nav::CWaypointsNavigator::saveConfigFile(class mrpt::config::CConfigFileBase &) const --> void", pybind11::arg("c"));

		{ // mrpt::nav::CWaypointsNavigator::TNavigationParamsWaypoints file:mrpt/nav/reactive/CWaypointsNavigator.h line:43
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::nav::CWaypointsNavigator::TNavigationParamsWaypoints, std::shared_ptr<mrpt::nav::CWaypointsNavigator::TNavigationParamsWaypoints>, PyCallBack_mrpt_nav_CWaypointsNavigator_TNavigationParamsWaypoints> cl(enclosing_class, "TNavigationParamsWaypoints", "The struct for configuring navigation requests to CWaypointsNavigator\n and derived classes. ");
			cl.def( pybind11::init( [](PyCallBack_mrpt_nav_CWaypointsNavigator_TNavigationParamsWaypoints const &o){ return new PyCallBack_mrpt_nav_CWaypointsNavigator_TNavigationParamsWaypoints(o); } ) );
			cl.def( pybind11::init( [](mrpt::nav::CWaypointsNavigator::TNavigationParamsWaypoints const &o){ return new mrpt::nav::CWaypointsNavigator::TNavigationParamsWaypoints(o); } ) );
			cl.def( pybind11::init( [](){ return new mrpt::nav::CWaypointsNavigator::TNavigationParamsWaypoints(); }, [](){ return new PyCallBack_mrpt_nav_CWaypointsNavigator_TNavigationParamsWaypoints(); } ) );
			cl.def_readwrite("multiple_targets", &mrpt::nav::CWaypointsNavigator::TNavigationParamsWaypoints::multiple_targets);
			cl.def("getAsText", (std::string (mrpt::nav::CWaypointsNavigator::TNavigationParamsWaypoints::*)() const) &mrpt::nav::CWaypointsNavigator::TNavigationParamsWaypoints::getAsText, "C++: mrpt::nav::CWaypointsNavigator::TNavigationParamsWaypoints::getAsText() const --> std::string");
			cl.def("assign", (struct mrpt::nav::CWaypointsNavigator::TNavigationParamsWaypoints & (mrpt::nav::CWaypointsNavigator::TNavigationParamsWaypoints::*)(const struct mrpt::nav::CWaypointsNavigator::TNavigationParamsWaypoints &)) &mrpt::nav::CWaypointsNavigator::TNavigationParamsWaypoints::operator=, "C++: mrpt::nav::CWaypointsNavigator::TNavigationParamsWaypoints::operator=(const struct mrpt::nav::CWaypointsNavigator::TNavigationParamsWaypoints &) --> struct mrpt::nav::CWaypointsNavigator::TNavigationParamsWaypoints &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::nav::CWaypointsNavigator::TWaypointsNavigatorParams file:mrpt/nav/reactive/CWaypointsNavigator.h line:120
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::nav::CWaypointsNavigator::TWaypointsNavigatorParams, std::shared_ptr<mrpt::nav::CWaypointsNavigator::TWaypointsNavigatorParams>, PyCallBack_mrpt_nav_CWaypointsNavigator_TWaypointsNavigatorParams, mrpt::config::CLoadableOptions> cl(enclosing_class, "TWaypointsNavigatorParams", "");
			cl.def( pybind11::init( [](){ return new mrpt::nav::CWaypointsNavigator::TWaypointsNavigatorParams(); }, [](){ return new PyCallBack_mrpt_nav_CWaypointsNavigator_TWaypointsNavigatorParams(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_nav_CWaypointsNavigator_TWaypointsNavigatorParams const &o){ return new PyCallBack_mrpt_nav_CWaypointsNavigator_TWaypointsNavigatorParams(o); } ) );
			cl.def( pybind11::init( [](mrpt::nav::CWaypointsNavigator::TWaypointsNavigatorParams const &o){ return new mrpt::nav::CWaypointsNavigator::TWaypointsNavigatorParams(o); } ) );
			cl.def_readwrite("max_distance_to_allow_skip_waypoint", &mrpt::nav::CWaypointsNavigator::TWaypointsNavigatorParams::max_distance_to_allow_skip_waypoint);
			cl.def_readwrite("min_timesteps_confirm_skip_waypoints", &mrpt::nav::CWaypointsNavigator::TWaypointsNavigatorParams::min_timesteps_confirm_skip_waypoints);
			cl.def_readwrite("waypoint_angle_tolerance", &mrpt::nav::CWaypointsNavigator::TWaypointsNavigatorParams::waypoint_angle_tolerance);
			cl.def_readwrite("multitarget_look_ahead", &mrpt::nav::CWaypointsNavigator::TWaypointsNavigatorParams::multitarget_look_ahead);
			cl.def("loadFromConfigFile", (void (mrpt::nav::CWaypointsNavigator::TWaypointsNavigatorParams::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::nav::CWaypointsNavigator::TWaypointsNavigatorParams::loadFromConfigFile, "C++: mrpt::nav::CWaypointsNavigator::TWaypointsNavigatorParams::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("c"), pybind11::arg("s"));
			cl.def("saveToConfigFile", (void (mrpt::nav::CWaypointsNavigator::TWaypointsNavigatorParams::*)(class mrpt::config::CConfigFileBase &, const std::string &) const) &mrpt::nav::CWaypointsNavigator::TWaypointsNavigatorParams::saveToConfigFile, "C++: mrpt::nav::CWaypointsNavigator::TWaypointsNavigatorParams::saveToConfigFile(class mrpt::config::CConfigFileBase &, const std::string &) const --> void", pybind11::arg("c"), pybind11::arg("s"));
			cl.def("assign", (struct mrpt::nav::CWaypointsNavigator::TWaypointsNavigatorParams & (mrpt::nav::CWaypointsNavigator::TWaypointsNavigatorParams::*)(const struct mrpt::nav::CWaypointsNavigator::TWaypointsNavigatorParams &)) &mrpt::nav::CWaypointsNavigator::TWaypointsNavigatorParams::operator=, "C++: mrpt::nav::CWaypointsNavigator::TWaypointsNavigatorParams::operator=(const struct mrpt::nav::CWaypointsNavigator::TWaypointsNavigatorParams &) --> struct mrpt::nav::CWaypointsNavigator::TWaypointsNavigatorParams &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
}
