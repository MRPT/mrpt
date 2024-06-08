#include <chrono>
#include <functional>
#include <istream>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/Clock.h>
#include <mrpt/core/aligned_allocator.h>
#include <mrpt/core/safe_pointers.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/img/TColor.h>
#include <mrpt/img/color_maps.h>
#include <mrpt/kinematics/CVehicleSimul_DiffDriven.h>
#include <mrpt/kinematics/CVehicleSimul_Holo.h>
#include <mrpt/kinematics/CVehicleVelCmd.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CPointsMap.h>
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
#include <mrpt/math/TSegment3D.h>
#include <mrpt/math/TTwist2D.h>
#include <mrpt/nav/holonomic/CAbstractHolonomicReactiveMethod.h>
#include <mrpt/nav/holonomic/ClearanceDiagram.h>
#include <mrpt/nav/reactive/CAbstractNavigator.h>
#include <mrpt/nav/reactive/CAbstractPTGBasedReactive.h>
#include <mrpt/nav/reactive/CLogFileRecord.h>
#include <mrpt/nav/reactive/CReactiveNavigationSystem3D.h>
#include <mrpt/nav/reactive/CRobot2NavInterface.h>
#include <mrpt/nav/reactive/CRobot2NavInterfaceForSimulator.h>
#include <mrpt/nav/reactive/CWaypointsNavigator.h>
#include <mrpt/nav/reactive/TCandidateMovementPTG.h>
#include <mrpt/nav/reactive/TWaypoint.h>
#include <mrpt/nav/tpspace/CPTG_DiffDrive_CollisionGridBased.h>
#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/obs/T2DScanProperties.h>
#include <mrpt/obs/T3DPointsTo2DScanParams.h>
#include <mrpt/obs/TRangeImageFilter.h>
#include <mrpt/opengl/CMesh.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPointPDF.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDF.h>
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
#include <string>
#include <type_traits>
#include <utility>
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

// mrpt::nav::CReactiveNavigationSystem3D file:mrpt/nav/reactive/CReactiveNavigationSystem3D.h line:80
struct PyCallBack_mrpt_nav_CReactiveNavigationSystem3D : public mrpt::nav::CReactiveNavigationSystem3D {
	using mrpt::nav::CReactiveNavigationSystem3D::CReactiveNavigationSystem3D;

	bool checkCollisionWithLatestObstacles(const struct mrpt::math::TPose2D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem3D *>(this), "checkCollisionWithLatestObstacles");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CReactiveNavigationSystem3D::checkCollisionWithLatestObstacles(a0);
	}
	size_t getPTG_count() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem3D *>(this), "getPTG_count");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CReactiveNavigationSystem3D::getPTG_count();
	}
	class mrpt::nav::CParameterizedTrajectoryGenerator * getPTG(size_t a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem3D *>(this), "getPTG");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::nav::CParameterizedTrajectoryGenerator *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::nav::CParameterizedTrajectoryGenerator *> caster;
				return pybind11::detail::cast_ref<class mrpt::nav::CParameterizedTrajectoryGenerator *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::nav::CParameterizedTrajectoryGenerator *>(std::move(o));
		}
		return CReactiveNavigationSystem3D::getPTG(a0);
	}
	void loadConfigFile(const class mrpt::config::CConfigFileBase & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem3D *>(this), "loadConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CReactiveNavigationSystem3D::loadConfigFile(a0);
	}
	void saveConfigFile(class mrpt::config::CConfigFileBase & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem3D *>(this), "saveConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CReactiveNavigationSystem3D::saveConfigFile(a0);
	}
	void initialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem3D *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CAbstractPTGBasedReactive::initialize();
	}
	void performNavigationStep() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem3D *>(this), "performNavigationStep");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CAbstractPTGBasedReactive::performNavigationStep();
	}
	class mrpt::nav::CAbstractHolonomicReactiveMethod * getHoloMethod(int a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem3D *>(this), "getHoloMethod");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::nav::CAbstractHolonomicReactiveMethod *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::nav::CAbstractHolonomicReactiveMethod *> caster;
				return pybind11::detail::cast_ref<class mrpt::nav::CAbstractHolonomicReactiveMethod *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::nav::CAbstractHolonomicReactiveMethod *>(std::move(o));
		}
		return CAbstractPTGBasedReactive::getHoloMethod(a0);
	}
	bool impl_waypoint_is_reachable(const struct mrpt::math::TPoint2D_<double> & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem3D *>(this), "impl_waypoint_is_reachable");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CAbstractPTGBasedReactive::impl_waypoint_is_reachable(a0);
	}
	void STEP1_InitPTGs() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem3D *>(this), "STEP1_InitPTGs");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CAbstractPTGBasedReactive::STEP1_InitPTGs\"");
	}
	bool implementSenseObstacles(mrpt::Clock::time_point & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem3D *>(this), "implementSenseObstacles");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CAbstractPTGBasedReactive::implementSenseObstacles\"");
	}
	void loggingGetWSObstaclesAndShape(class mrpt::nav::CLogFileRecord & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem3D *>(this), "loggingGetWSObstaclesAndShape");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CAbstractPTGBasedReactive::loggingGetWSObstaclesAndShape\"");
	}
	double generate_vel_cmd(const struct mrpt::nav::TCandidateMovementPTG & a0, class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem3D *>(this), "generate_vel_cmd");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CAbstractPTGBasedReactive::generate_vel_cmd(a0, a1);
	}
	void onStartNewNavigation() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem3D *>(this), "onStartNewNavigation");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CAbstractPTGBasedReactive::onStartNewNavigation();
	}
	void navigationStep() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem3D *>(this), "navigationStep");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem3D *>(this), "cancel");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem3D *>(this), "navigateWaypoints");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem3D *>(this), "getWaypointNavStatus");
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
	void onNavigateCommandReceived() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem3D *>(this), "onNavigateCommandReceived");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem3D *>(this), "checkHasReachedTarget");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem3D *>(this), "waypoints_navigationStep");
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
	void resume() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem3D *>(this), "resume");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem3D *>(this), "suspend");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem3D *>(this), "resetNavError");
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
	void updateCurrentPoseAndSpeeds() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem3D *>(this), "updateCurrentPoseAndSpeeds");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem3D *>(this), "performNavigationStepNavigating");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem3D *>(this), "doEmergencyStop");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem3D *>(this), "changeSpeeds");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem3D *>(this), "changeSpeedsNOP");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem3D *>(this), "stop");
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
};

// mrpt::nav::CRobot2NavInterfaceForSimulator_Holo file:mrpt/nav/reactive/CRobot2NavInterfaceForSimulator.h line:25
struct PyCallBack_mrpt_nav_CRobot2NavInterfaceForSimulator_Holo : public mrpt::nav::CRobot2NavInterfaceForSimulator_Holo {
	using mrpt::nav::CRobot2NavInterfaceForSimulator_Holo::CRobot2NavInterfaceForSimulator_Holo;

	bool getCurrentPoseAndSpeeds(struct mrpt::math::TPose2D & a0, struct mrpt::math::TTwist2D & a1, mrpt::Clock::time_point & a2, struct mrpt::math::TPose2D & a3, std::string & a4) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_Holo *>(this), "getCurrentPoseAndSpeeds");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CRobot2NavInterfaceForSimulator_Holo::getCurrentPoseAndSpeeds(a0, a1, a2, a3, a4);
	}
	bool changeSpeeds(const class mrpt::kinematics::CVehicleVelCmd & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_Holo *>(this), "changeSpeeds");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CRobot2NavInterfaceForSimulator_Holo::changeSpeeds(a0);
	}
	bool stop(bool a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_Holo *>(this), "stop");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CRobot2NavInterfaceForSimulator_Holo::stop(a0);
	}
	class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> getEmergencyStopCmd() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_Holo *>(this), "getEmergencyStopCmd");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>::value) {
				static pybind11::detail::override_caster_t<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>> caster;
				return pybind11::detail::cast_ref<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o));
		}
		return CRobot2NavInterfaceForSimulator_Holo::getEmergencyStopCmd();
	}
	class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> getStopCmd() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_Holo *>(this), "getStopCmd");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>::value) {
				static pybind11::detail::override_caster_t<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>> caster;
				return pybind11::detail::cast_ref<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o));
		}
		return CRobot2NavInterfaceForSimulator_Holo::getStopCmd();
	}
	class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> getAlignCmd(const double a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_Holo *>(this), "getAlignCmd");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>::value) {
				static pybind11::detail::override_caster_t<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>> caster;
				return pybind11::detail::cast_ref<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o));
		}
		return CRobot2NavInterfaceForSimulator_Holo::getAlignCmd(a0);
	}
	double getNavigationTime() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_Holo *>(this), "getNavigationTime");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CRobot2NavInterfaceForSimulator_Holo::getNavigationTime();
	}
	void resetNavigationTimer() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_Holo *>(this), "resetNavigationTimer");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRobot2NavInterfaceForSimulator_Holo::resetNavigationTimer();
	}
	bool changeSpeedsNOP() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_Holo *>(this), "changeSpeedsNOP");
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
	bool startWatchdog(float a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_Holo *>(this), "startWatchdog");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_Holo *>(this), "stopWatchdog");
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
	bool senseObstacles(class mrpt::maps::CSimplePointsMap & a0, mrpt::Clock::time_point & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_Holo *>(this), "senseObstacles");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_Holo *>(this), "sendNavigationStartEvent");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_Holo *>(this), "sendNavigationEndEvent");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_Holo *>(this), "sendWaypointReachedEvent");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_Holo *>(this), "sendNewWaypointTargetEvent");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_Holo *>(this), "sendNavigationEndDueToErrorEvent");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_Holo *>(this), "sendWaySeemsBlockedEvent");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_Holo *>(this), "sendApparentCollisionEvent");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_Holo *>(this), "sendCannotGetCloserToBlockedTargetEvent");
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
};

// mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven file:mrpt/nav/reactive/CRobot2NavInterfaceForSimulator.h line:102
struct PyCallBack_mrpt_nav_CRobot2NavInterfaceForSimulator_DiffDriven : public mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven {
	using mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven::CRobot2NavInterfaceForSimulator_DiffDriven;

	bool getCurrentPoseAndSpeeds(struct mrpt::math::TPose2D & a0, struct mrpt::math::TTwist2D & a1, mrpt::Clock::time_point & a2, struct mrpt::math::TPose2D & a3, std::string & a4) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven *>(this), "getCurrentPoseAndSpeeds");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CRobot2NavInterfaceForSimulator_DiffDriven::getCurrentPoseAndSpeeds(a0, a1, a2, a3, a4);
	}
	bool changeSpeeds(const class mrpt::kinematics::CVehicleVelCmd & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven *>(this), "changeSpeeds");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CRobot2NavInterfaceForSimulator_DiffDriven::changeSpeeds(a0);
	}
	bool stop(bool a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven *>(this), "stop");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CRobot2NavInterfaceForSimulator_DiffDriven::stop(a0);
	}
	class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> getStopCmd() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven *>(this), "getStopCmd");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>::value) {
				static pybind11::detail::override_caster_t<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>> caster;
				return pybind11::detail::cast_ref<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o));
		}
		return CRobot2NavInterfaceForSimulator_DiffDriven::getStopCmd();
	}
	class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> getEmergencyStopCmd() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven *>(this), "getEmergencyStopCmd");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>::value) {
				static pybind11::detail::override_caster_t<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>> caster;
				return pybind11::detail::cast_ref<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o));
		}
		return CRobot2NavInterfaceForSimulator_DiffDriven::getEmergencyStopCmd();
	}
	double getNavigationTime() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven *>(this), "getNavigationTime");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CRobot2NavInterfaceForSimulator_DiffDriven::getNavigationTime();
	}
	void resetNavigationTimer() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven *>(this), "resetNavigationTimer");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRobot2NavInterfaceForSimulator_DiffDriven::resetNavigationTimer();
	}
	bool changeSpeedsNOP() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven *>(this), "changeSpeedsNOP");
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
	class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> getAlignCmd(const double a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven *>(this), "getAlignCmd");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven *>(this), "startWatchdog");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven *>(this), "stopWatchdog");
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
	bool senseObstacles(class mrpt::maps::CSimplePointsMap & a0, mrpt::Clock::time_point & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven *>(this), "senseObstacles");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven *>(this), "sendNavigationStartEvent");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven *>(this), "sendNavigationEndEvent");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven *>(this), "sendWaypointReachedEvent");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven *>(this), "sendNewWaypointTargetEvent");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven *>(this), "sendNavigationEndDueToErrorEvent");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven *>(this), "sendWaySeemsBlockedEvent");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven *>(this), "sendApparentCollisionEvent");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven *>(this), "sendCannotGetCloserToBlockedTargetEvent");
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
};

void bind_mrpt_nav_reactive_CReactiveNavigationSystem3D(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::nav::TRobotShape file:mrpt/nav/reactive/CReactiveNavigationSystem3D.h line:21
		pybind11::class_<mrpt::nav::TRobotShape, std::shared_ptr<mrpt::nav::TRobotShape>> cl(M("mrpt::nav"), "TRobotShape", "A 3D robot shape stored as a \"sliced\" stack of 2D polygons, used for\n CReactiveNavigationSystem3D\n Depending on each PTG, only the 2D polygon or the circular radius will be\n taken into account\n  \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::nav::TRobotShape(); } ) );
		cl.def( pybind11::init( [](mrpt::nav::TRobotShape const &o){ return new mrpt::nav::TRobotShape(o); } ) );
		cl.def("size", (size_t (mrpt::nav::TRobotShape::*)() const) &mrpt::nav::TRobotShape::size, "C++: mrpt::nav::TRobotShape::size() const --> size_t");
		cl.def("resize", (void (mrpt::nav::TRobotShape::*)(size_t)) &mrpt::nav::TRobotShape::resize, "C++: mrpt::nav::TRobotShape::resize(size_t) --> void", pybind11::arg("num_levels"));
		cl.def("getRadius", (double (mrpt::nav::TRobotShape::*)(size_t) const) &mrpt::nav::TRobotShape::getRadius, "C++: mrpt::nav::TRobotShape::getRadius(size_t) const --> double", pybind11::arg("level"));
		cl.def("getHeight", (double (mrpt::nav::TRobotShape::*)(size_t) const) &mrpt::nav::TRobotShape::getHeight, "C++: mrpt::nav::TRobotShape::getHeight(size_t) const --> double", pybind11::arg("level"));
		cl.def("polygon", (class mrpt::math::CPolygon & (mrpt::nav::TRobotShape::*)(size_t)) &mrpt::nav::TRobotShape::polygon, "C++: mrpt::nav::TRobotShape::polygon(size_t) --> class mrpt::math::CPolygon &", pybind11::return_value_policy::automatic, pybind11::arg("level"));
		cl.def("setRadius", (void (mrpt::nav::TRobotShape::*)(size_t, double)) &mrpt::nav::TRobotShape::setRadius, "C++: mrpt::nav::TRobotShape::setRadius(size_t, double) --> void", pybind11::arg("level"), pybind11::arg("r"));
		cl.def("setHeight", (void (mrpt::nav::TRobotShape::*)(size_t, double)) &mrpt::nav::TRobotShape::setHeight, "C++: mrpt::nav::TRobotShape::setHeight(size_t, double) --> void", pybind11::arg("level"), pybind11::arg("h"));
		cl.def("assign", (struct mrpt::nav::TRobotShape & (mrpt::nav::TRobotShape::*)(const struct mrpt::nav::TRobotShape &)) &mrpt::nav::TRobotShape::operator=, "C++: mrpt::nav::TRobotShape::operator=(const struct mrpt::nav::TRobotShape &) --> struct mrpt::nav::TRobotShape &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::nav::CReactiveNavigationSystem3D file:mrpt/nav/reactive/CReactiveNavigationSystem3D.h line:80
		pybind11::class_<mrpt::nav::CReactiveNavigationSystem3D, std::shared_ptr<mrpt::nav::CReactiveNavigationSystem3D>, PyCallBack_mrpt_nav_CReactiveNavigationSystem3D, mrpt::nav::CAbstractPTGBasedReactive> cl(M("mrpt::nav"), "CReactiveNavigationSystem3D", "See base class CAbstractPTGBasedReactive for a description and instructions\n of use.\n This particular implementation assumes a 3D (or \"2.5D\") robot shape model,\n build as a vertical stack of \"2D slices\".\n\n  Paper describing the method:\n  - M. Jaimez-Tarifa, J. Gonzalez-Jimenez, J.L. Blanco,\n    \"Efficient Reactive Navigation with Exact Collision Determination for 3D\n Robot Shapes\",\n     International Journal of Advanced Robotic Systems, 2015.\n\n Class history:\n - SEP/2012: First design.\n - JUL/2013: Integrated into MRPT library.\n - DEC/2013: Code refactoring between this class and\n CAbstractHolonomicReactiveMethod\n - FEB/2017: Refactoring of all parameters for a consistent organization in\n sections by class names (MRPT 1.5.0)\n\n This class requires a number of parameters which are usually provided via an\n external config (\".ini\") file.\n Alternatively, a memory-only object can be used to avoid physical files, see\n mrpt::config::CConfigFileMemory.\n\n A template config file can be generated at any moment by the user by calling\n saveConfigFile() with a default-constructed object.\n\n Next we provide a self-documented template config file; or see it online:\n https://github.com/MRPT/mrpt/blob/master/share/mrpt/config_files/navigation-ptgs/reactive3d_config.ini\n \n\n\n  \n CAbstractNavigator, CParameterizedTrajectoryGenerator,\n CAbstractHolonomicReactiveMethod\n  \n\n\n ");
		cl.def( pybind11::init( [](class mrpt::nav::CRobot2NavInterface & a0){ return new mrpt::nav::CReactiveNavigationSystem3D(a0); }, [](class mrpt::nav::CRobot2NavInterface & a0){ return new PyCallBack_mrpt_nav_CReactiveNavigationSystem3D(a0); } ), "doc");
		cl.def( pybind11::init( [](class mrpt::nav::CRobot2NavInterface & a0, bool const & a1){ return new mrpt::nav::CReactiveNavigationSystem3D(a0, a1); }, [](class mrpt::nav::CRobot2NavInterface & a0, bool const & a1){ return new PyCallBack_mrpt_nav_CReactiveNavigationSystem3D(a0, a1); } ), "doc");
		cl.def( pybind11::init( [](class mrpt::nav::CRobot2NavInterface & a0, bool const & a1, bool const & a2){ return new mrpt::nav::CReactiveNavigationSystem3D(a0, a1, a2); }, [](class mrpt::nav::CRobot2NavInterface & a0, bool const & a1, bool const & a2){ return new PyCallBack_mrpt_nav_CReactiveNavigationSystem3D(a0, a1, a2); } ), "doc");
		cl.def( pybind11::init<class mrpt::nav::CRobot2NavInterface &, bool, bool, const std::string &>(), pybind11::arg("react_iterf_impl"), pybind11::arg("enableConsoleOutput"), pybind11::arg("enableLogFile"), pybind11::arg("logFileDirectory") );

		cl.def("changeRobotShape", (void (mrpt::nav::CReactiveNavigationSystem3D::*)(struct mrpt::nav::TRobotShape)) &mrpt::nav::CReactiveNavigationSystem3D::changeRobotShape, "Change the robot shape, which is taken into account for collision grid\n building. \n\nC++: mrpt::nav::CReactiveNavigationSystem3D::changeRobotShape(struct mrpt::nav::TRobotShape) --> void", pybind11::arg("robotShape"));
		cl.def("checkCollisionWithLatestObstacles", (bool (mrpt::nav::CReactiveNavigationSystem3D::*)(const struct mrpt::math::TPose2D &) const) &mrpt::nav::CReactiveNavigationSystem3D::checkCollisionWithLatestObstacles, "C++: mrpt::nav::CReactiveNavigationSystem3D::checkCollisionWithLatestObstacles(const struct mrpt::math::TPose2D &) const --> bool", pybind11::arg("relative_robot_pose"));
		cl.def("getPTG_count", (size_t (mrpt::nav::CReactiveNavigationSystem3D::*)() const) &mrpt::nav::CReactiveNavigationSystem3D::getPTG_count, "C++: mrpt::nav::CReactiveNavigationSystem3D::getPTG_count() const --> size_t");
		cl.def("getPTG", (class mrpt::nav::CParameterizedTrajectoryGenerator * (mrpt::nav::CReactiveNavigationSystem3D::*)(size_t)) &mrpt::nav::CReactiveNavigationSystem3D::getPTG, "C++: mrpt::nav::CReactiveNavigationSystem3D::getPTG(size_t) --> class mrpt::nav::CParameterizedTrajectoryGenerator *", pybind11::return_value_policy::automatic, pybind11::arg("i"));
		cl.def("loadConfigFile", (void (mrpt::nav::CReactiveNavigationSystem3D::*)(const class mrpt::config::CConfigFileBase &)) &mrpt::nav::CReactiveNavigationSystem3D::loadConfigFile, "C++: mrpt::nav::CReactiveNavigationSystem3D::loadConfigFile(const class mrpt::config::CConfigFileBase &) --> void", pybind11::arg("c"));
		cl.def("saveConfigFile", (void (mrpt::nav::CReactiveNavigationSystem3D::*)(class mrpt::config::CConfigFileBase &) const) &mrpt::nav::CReactiveNavigationSystem3D::saveConfigFile, "C++: mrpt::nav::CReactiveNavigationSystem3D::saveConfigFile(class mrpt::config::CConfigFileBase &) const --> void", pybind11::arg("c"));
	}
	{ // mrpt::nav::CRobot2NavInterfaceForSimulator_Holo file:mrpt/nav/reactive/CRobot2NavInterfaceForSimulator.h line:25
		pybind11::class_<mrpt::nav::CRobot2NavInterfaceForSimulator_Holo, std::shared_ptr<mrpt::nav::CRobot2NavInterfaceForSimulator_Holo>, PyCallBack_mrpt_nav_CRobot2NavInterfaceForSimulator_Holo, mrpt::nav::CRobot2NavInterface> cl(M("mrpt::nav"), "CRobot2NavInterfaceForSimulator_Holo", "CRobot2NavInterface implemented for a simulator object based on\n mrpt::kinematics::CVehicleSimul_Holo.\n Only `senseObstacles()` remains virtual for the user to implement it.\n\n \n CReactiveNavigationSystem, CAbstractNavigator,\n mrpt::kinematics::CVehicleSimulVirtualBase\n  \n\n\n ");
		cl.def( pybind11::init<class mrpt::kinematics::CVehicleSimul_Holo &>(), pybind11::arg("simul") );

		cl.def(pybind11::init<PyCallBack_mrpt_nav_CRobot2NavInterfaceForSimulator_Holo const &>());
		cl.def("getCurrentPoseAndSpeeds", (bool (mrpt::nav::CRobot2NavInterfaceForSimulator_Holo::*)(struct mrpt::math::TPose2D &, struct mrpt::math::TTwist2D &, mrpt::Clock::time_point &, struct mrpt::math::TPose2D &, std::string &)) &mrpt::nav::CRobot2NavInterfaceForSimulator_Holo::getCurrentPoseAndSpeeds, "C++: mrpt::nav::CRobot2NavInterfaceForSimulator_Holo::getCurrentPoseAndSpeeds(struct mrpt::math::TPose2D &, struct mrpt::math::TTwist2D &, mrpt::Clock::time_point &, struct mrpt::math::TPose2D &, std::string &) --> bool", pybind11::arg("curPose"), pybind11::arg("curVel"), pybind11::arg("timestamp"), pybind11::arg("curOdometry"), pybind11::arg("frame_id"));
		cl.def("changeSpeeds", (bool (mrpt::nav::CRobot2NavInterfaceForSimulator_Holo::*)(const class mrpt::kinematics::CVehicleVelCmd &)) &mrpt::nav::CRobot2NavInterfaceForSimulator_Holo::changeSpeeds, "C++: mrpt::nav::CRobot2NavInterfaceForSimulator_Holo::changeSpeeds(const class mrpt::kinematics::CVehicleVelCmd &) --> bool", pybind11::arg("vel_cmd"));
		cl.def("stop", (bool (mrpt::nav::CRobot2NavInterfaceForSimulator_Holo::*)(bool)) &mrpt::nav::CRobot2NavInterfaceForSimulator_Holo::stop, "C++: mrpt::nav::CRobot2NavInterfaceForSimulator_Holo::stop(bool) --> bool", pybind11::arg("isEmergencyStop"));
		cl.def("getEmergencyStopCmd", (class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> (mrpt::nav::CRobot2NavInterfaceForSimulator_Holo::*)()) &mrpt::nav::CRobot2NavInterfaceForSimulator_Holo::getEmergencyStopCmd, "C++: mrpt::nav::CRobot2NavInterfaceForSimulator_Holo::getEmergencyStopCmd() --> class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>");
		cl.def("getStopCmd", (class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> (mrpt::nav::CRobot2NavInterfaceForSimulator_Holo::*)()) &mrpt::nav::CRobot2NavInterfaceForSimulator_Holo::getStopCmd, "C++: mrpt::nav::CRobot2NavInterfaceForSimulator_Holo::getStopCmd() --> class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>");
		cl.def("getAlignCmd", (class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> (mrpt::nav::CRobot2NavInterfaceForSimulator_Holo::*)(const double)) &mrpt::nav::CRobot2NavInterfaceForSimulator_Holo::getAlignCmd, "C++: mrpt::nav::CRobot2NavInterfaceForSimulator_Holo::getAlignCmd(const double) --> class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>", pybind11::arg("relative_heading_radians"));
		cl.def("getNavigationTime", (double (mrpt::nav::CRobot2NavInterfaceForSimulator_Holo::*)()) &mrpt::nav::CRobot2NavInterfaceForSimulator_Holo::getNavigationTime, "See CRobot2NavInterface::getNavigationTime(). In this class, simulation\n time is returned instead of wall-clock time. \n\nC++: mrpt::nav::CRobot2NavInterfaceForSimulator_Holo::getNavigationTime() --> double");
		cl.def("resetNavigationTimer", (void (mrpt::nav::CRobot2NavInterfaceForSimulator_Holo::*)()) &mrpt::nav::CRobot2NavInterfaceForSimulator_Holo::resetNavigationTimer, "See CRobot2NavInterface::resetNavigationTimer() \n\nC++: mrpt::nav::CRobot2NavInterfaceForSimulator_Holo::resetNavigationTimer() --> void");
	}
	{ // mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven file:mrpt/nav/reactive/CRobot2NavInterfaceForSimulator.h line:102
		pybind11::class_<mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven, std::shared_ptr<mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven>, PyCallBack_mrpt_nav_CRobot2NavInterfaceForSimulator_DiffDriven, mrpt::nav::CRobot2NavInterface> cl(M("mrpt::nav"), "CRobot2NavInterfaceForSimulator_DiffDriven", "CRobot2NavInterface implemented for a simulator object based on\n mrpt::kinematics::CVehicleSimul_DiffDriven\n Only `senseObstacles()` remains virtual for the user to implement it.\n\n \n CReactiveNavigationSystem, CAbstractNavigator,\n mrpt::kinematics::CVehicleSimulVirtualBase\n  \n\n\n ");
		cl.def( pybind11::init<class mrpt::kinematics::CVehicleSimul_DiffDriven &>(), pybind11::arg("simul") );

		cl.def(pybind11::init<PyCallBack_mrpt_nav_CRobot2NavInterfaceForSimulator_DiffDriven const &>());
		cl.def("getCurrentPoseAndSpeeds", (bool (mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven::*)(struct mrpt::math::TPose2D &, struct mrpt::math::TTwist2D &, mrpt::Clock::time_point &, struct mrpt::math::TPose2D &, std::string &)) &mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven::getCurrentPoseAndSpeeds, "C++: mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven::getCurrentPoseAndSpeeds(struct mrpt::math::TPose2D &, struct mrpt::math::TTwist2D &, mrpt::Clock::time_point &, struct mrpt::math::TPose2D &, std::string &) --> bool", pybind11::arg("curPose"), pybind11::arg("curVel"), pybind11::arg("timestamp"), pybind11::arg("curOdometry"), pybind11::arg("frame_id"));
		cl.def("changeSpeeds", (bool (mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven::*)(const class mrpt::kinematics::CVehicleVelCmd &)) &mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven::changeSpeeds, "C++: mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven::changeSpeeds(const class mrpt::kinematics::CVehicleVelCmd &) --> bool", pybind11::arg("vel_cmd"));
		cl.def("stop", (bool (mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven::*)(bool)) &mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven::stop, "C++: mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven::stop(bool) --> bool", pybind11::arg("isEmergencyStop"));
		cl.def("getStopCmd", (class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> (mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven::*)()) &mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven::getStopCmd, "C++: mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven::getStopCmd() --> class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>");
		cl.def("getEmergencyStopCmd", (class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> (mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven::*)()) &mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven::getEmergencyStopCmd, "C++: mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven::getEmergencyStopCmd() --> class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>");
		cl.def("getNavigationTime", (double (mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven::*)()) &mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven::getNavigationTime, "See CRobot2NavInterface::getNavigationTime(). In this class, simulation\n time is returned instead of wall-clock time. \n\nC++: mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven::getNavigationTime() --> double");
		cl.def("resetNavigationTimer", (void (mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven::*)()) &mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven::resetNavigationTimer, "See CRobot2NavInterface::resetNavigationTimer() \n\nC++: mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven::resetNavigationTimer() --> void");
	}
	{ // mrpt::nav::TCPoint file:mrpt/nav/tpspace/CPTG_DiffDrive_CollisionGridBased.h line:25
		pybind11::class_<mrpt::nav::TCPoint, std::shared_ptr<mrpt::nav::TCPoint>> cl(M("mrpt::nav"), "TCPoint", "Trajectory points in C-Space for non-holonomic robots \n\n CPTG_DiffDrive_CollisionGridBased ");
		cl.def( pybind11::init( [](){ return new mrpt::nav::TCPoint(); } ) );
		cl.def( pybind11::init<const float, const float, const float, const float, const float, const float, const float>(), pybind11::arg("x_"), pybind11::arg("y_"), pybind11::arg("phi_"), pybind11::arg("t_"), pybind11::arg("dist_"), pybind11::arg("v_"), pybind11::arg("w_") );

		cl.def_readwrite("x", &mrpt::nav::TCPoint::x);
		cl.def_readwrite("y", &mrpt::nav::TCPoint::y);
		cl.def_readwrite("phi", &mrpt::nav::TCPoint::phi);
		cl.def_readwrite("t", &mrpt::nav::TCPoint::t);
		cl.def_readwrite("dist", &mrpt::nav::TCPoint::dist);
		cl.def_readwrite("v", &mrpt::nav::TCPoint::v);
		cl.def_readwrite("w", &mrpt::nav::TCPoint::w);
	}
	{ // mrpt::nav::CPTG_DiffDrive_CollisionGridBased file:mrpt/nav/tpspace/CPTG_DiffDrive_CollisionGridBased.h line:56
		pybind11::class_<mrpt::nav::CPTG_DiffDrive_CollisionGridBased, std::shared_ptr<mrpt::nav::CPTG_DiffDrive_CollisionGridBased>, mrpt::nav::CPTG_RobotShape_Polygonal> cl(M("mrpt::nav"), "CPTG_DiffDrive_CollisionGridBased", "Base class for all PTGs suitable to non-holonomic, differentially-driven (or\n Ackermann) vehicles\n based on numerical integration of the trajectories and collision\n look-up-table.\n Regarding `initialize()`: in this this family of PTGs, the method builds the\n collision grid or load it from a cache file.\n Collision grids must be calculated before calling getTPObstacle(). Robot\n shape must be set before initializing with setRobotShape().\n The rest of PTG parameters should have been set at the constructor.");
		cl.def("ptgDiffDriveSteeringFunction", (void (mrpt::nav::CPTG_DiffDrive_CollisionGridBased::*)(float, float, float, float, float, float &, float &) const) &mrpt::nav::CPTG_DiffDrive_CollisionGridBased::ptgDiffDriveSteeringFunction, "The main method to be implemented in derived classes: it defines the\n differential-driven differential equation \n\nC++: mrpt::nav::CPTG_DiffDrive_CollisionGridBased::ptgDiffDriveSteeringFunction(float, float, float, float, float, float &, float &) const --> void", pybind11::arg("alpha"), pybind11::arg("t"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("phi"), pybind11::arg("v"), pybind11::arg("w"));
		cl.def("inverseMap_WS2TP", [](mrpt::nav::CPTG_DiffDrive_CollisionGridBased const &o, double const & a0, double const & a1, int & a2, double & a3) -> bool { return o.inverseMap_WS2TP(a0, a1, a2, a3); }, "", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("out_k"), pybind11::arg("out_d"));
		cl.def("inverseMap_WS2TP", (bool (mrpt::nav::CPTG_DiffDrive_CollisionGridBased::*)(double, double, int &, double &, double) const) &mrpt::nav::CPTG_DiffDrive_CollisionGridBased::inverseMap_WS2TP, "The default implementation in this class relies on a look-up-table.\n Derived classes may redefine this to closed-form expressions, when they\n exist.\n See full docs in base class\n CParameterizedTrajectoryGenerator::inverseMap_WS2TP() \n\nC++: mrpt::nav::CPTG_DiffDrive_CollisionGridBased::inverseMap_WS2TP(double, double, int &, double &, double) const --> bool", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("out_k"), pybind11::arg("out_d"), pybind11::arg("tolerance_dist"));
		cl.def("directionToMotionCommand", (class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> (mrpt::nav::CPTG_DiffDrive_CollisionGridBased::*)(uint16_t) const) &mrpt::nav::CPTG_DiffDrive_CollisionGridBased::directionToMotionCommand, "In this class, `out_action_cmd` contains: [0]: linear velocity (m/s),\n [1]: angular velocity (rad/s).\n See more docs in\n CParameterizedTrajectoryGenerator::directionToMotionCommand() \n\nC++: mrpt::nav::CPTG_DiffDrive_CollisionGridBased::directionToMotionCommand(uint16_t) const --> class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>", pybind11::arg("k"));
		cl.def("getSupportedKinematicVelocityCommand", (class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> (mrpt::nav::CPTG_DiffDrive_CollisionGridBased::*)() const) &mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getSupportedKinematicVelocityCommand, "C++: mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getSupportedKinematicVelocityCommand() const --> class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>");
		cl.def("setRefDistance", (void (mrpt::nav::CPTG_DiffDrive_CollisionGridBased::*)(const double)) &mrpt::nav::CPTG_DiffDrive_CollisionGridBased::setRefDistance, "Launches an exception in this class: it is not allowed in numerical\n integration-based PTGs to change the reference distance\n after initialization. \n\nC++: mrpt::nav::CPTG_DiffDrive_CollisionGridBased::setRefDistance(const double) --> void", pybind11::arg("refDist"));
		cl.def("getPathStepCount", (size_t (mrpt::nav::CPTG_DiffDrive_CollisionGridBased::*)(uint16_t) const) &mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getPathStepCount, "C++: mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getPathStepCount(uint16_t) const --> size_t", pybind11::arg("k"));
		cl.def("getPathPose", (struct mrpt::math::TPose2D (mrpt::nav::CPTG_DiffDrive_CollisionGridBased::*)(uint16_t, uint32_t) const) &mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getPathPose, "C++: mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getPathPose(uint16_t, uint32_t) const --> struct mrpt::math::TPose2D", pybind11::arg("k"), pybind11::arg("step"));
		cl.def("getPathDist", (double (mrpt::nav::CPTG_DiffDrive_CollisionGridBased::*)(uint16_t, uint32_t) const) &mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getPathDist, "C++: mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getPathDist(uint16_t, uint32_t) const --> double", pybind11::arg("k"), pybind11::arg("step"));
		cl.def("getPathStepForDist", (bool (mrpt::nav::CPTG_DiffDrive_CollisionGridBased::*)(uint16_t, double, unsigned int &) const) &mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getPathStepForDist, "C++: mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getPathStepForDist(uint16_t, double, unsigned int &) const --> bool", pybind11::arg("k"), pybind11::arg("dist"), pybind11::arg("out_step"));
		cl.def("getPathStepDuration", (double (mrpt::nav::CPTG_DiffDrive_CollisionGridBased::*)() const) &mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getPathStepDuration, "C++: mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getPathStepDuration() const --> double");
		cl.def("getMaxLinVel", (double (mrpt::nav::CPTG_DiffDrive_CollisionGridBased::*)() const) &mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getMaxLinVel, "C++: mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getMaxLinVel() const --> double");
		cl.def("getMaxAngVel", (double (mrpt::nav::CPTG_DiffDrive_CollisionGridBased::*)() const) &mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getMaxAngVel, "C++: mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getMaxAngVel() const --> double");
		cl.def("updateTPObstacleSingle", (void (mrpt::nav::CPTG_DiffDrive_CollisionGridBased::*)(double, double, uint16_t, double &) const) &mrpt::nav::CPTG_DiffDrive_CollisionGridBased::updateTPObstacleSingle, "C++: mrpt::nav::CPTG_DiffDrive_CollisionGridBased::updateTPObstacleSingle(double, double, uint16_t, double &) const --> void", pybind11::arg("ox"), pybind11::arg("oy"), pybind11::arg("k"), pybind11::arg("tp_obstacle_k"));
		cl.def("onNewNavDynamicState", (void (mrpt::nav::CPTG_DiffDrive_CollisionGridBased::*)()) &mrpt::nav::CPTG_DiffDrive_CollisionGridBased::onNewNavDynamicState, "This family of PTGs ignores the dynamic states \n\nC++: mrpt::nav::CPTG_DiffDrive_CollisionGridBased::onNewNavDynamicState() --> void");
		cl.def("getMax_V", (double (mrpt::nav::CPTG_DiffDrive_CollisionGridBased::*)() const) &mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getMax_V, "@} \n\nC++: mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getMax_V() const --> double");
		cl.def("getMax_W", (double (mrpt::nav::CPTG_DiffDrive_CollisionGridBased::*)() const) &mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getMax_W, "C++: mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getMax_W() const --> double");
		cl.def("assign", (class mrpt::nav::CPTG_DiffDrive_CollisionGridBased & (mrpt::nav::CPTG_DiffDrive_CollisionGridBased::*)(const class mrpt::nav::CPTG_DiffDrive_CollisionGridBased &)) &mrpt::nav::CPTG_DiffDrive_CollisionGridBased::operator=, "C++: mrpt::nav::CPTG_DiffDrive_CollisionGridBased::operator=(const class mrpt::nav::CPTG_DiffDrive_CollisionGridBased &) --> class mrpt::nav::CPTG_DiffDrive_CollisionGridBased &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
