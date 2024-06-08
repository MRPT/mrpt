#include <chrono>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/Clock.h>
#include <mrpt/img/TColor.h>
#include <mrpt/kinematics/CVehicleVelCmd.h>
#include <mrpt/maps/CSimplePointsMap.h>
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
#include <mrpt/nav/reactive/CMultiObjMotionOpt_Scalarization.h>
#include <mrpt/nav/reactive/CNavigatorManualSequence.h>
#include <mrpt/nav/reactive/CReactiveNavigationSystem.h>
#include <mrpt/nav/reactive/CRobot2NavInterface.h>
#include <mrpt/nav/reactive/CWaypointsNavigator.h>
#include <mrpt/nav/reactive/TCandidateMovementPTG.h>
#include <mrpt/nav/reactive/TWaypoint.h>
#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
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
#include <mrpt/typemeta/static_string.h>
#include <optional>
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

// mrpt::nav::CMultiObjMotionOpt_Scalarization file:mrpt/nav/reactive/CMultiObjMotionOpt_Scalarization.h line:25
struct PyCallBack_mrpt_nav_CMultiObjMotionOpt_Scalarization : public mrpt::nav::CMultiObjMotionOpt_Scalarization {
	using mrpt::nav::CMultiObjMotionOpt_Scalarization::CMultiObjMotionOpt_Scalarization;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CMultiObjMotionOpt_Scalarization *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CMultiObjMotionOpt_Scalarization::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CMultiObjMotionOpt_Scalarization *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CMultiObjMotionOpt_Scalarization::clone();
	}
	void loadConfigFile(const class mrpt::config::CConfigFileBase & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CMultiObjMotionOpt_Scalarization *>(this), "loadConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMultiObjMotionOpt_Scalarization::loadConfigFile(a0);
	}
	void saveConfigFile(class mrpt::config::CConfigFileBase & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CMultiObjMotionOpt_Scalarization *>(this), "saveConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMultiObjMotionOpt_Scalarization::saveConfigFile(a0);
	}
	void clear() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CMultiObjMotionOpt_Scalarization *>(this), "clear");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMultiObjMotionOpt_Scalarization::clear();
	}
};

// mrpt::nav::CMultiObjMotionOpt_Scalarization::TParams file:mrpt/nav/reactive/CMultiObjMotionOpt_Scalarization.h line:35
struct PyCallBack_mrpt_nav_CMultiObjMotionOpt_Scalarization_TParams : public mrpt::nav::CMultiObjMotionOpt_Scalarization::TParams {
	using mrpt::nav::CMultiObjMotionOpt_Scalarization::TParams::TParams;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CMultiObjMotionOpt_Scalarization::TParams *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TParams::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CMultiObjMotionOpt_Scalarization::TParams *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TParams::saveToConfigFile(a0, a1);
	}
};

// mrpt::nav::CNavigatorManualSequence file:mrpt/nav/reactive/CNavigatorManualSequence.h line:22
struct PyCallBack_mrpt_nav_CNavigatorManualSequence : public mrpt::nav::CNavigatorManualSequence {
	using mrpt::nav::CNavigatorManualSequence::CNavigatorManualSequence;

	void loadConfigFile(const class mrpt::config::CConfigFileBase & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CNavigatorManualSequence *>(this), "loadConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CNavigatorManualSequence::loadConfigFile(a0);
	}
	void saveConfigFile(class mrpt::config::CConfigFileBase & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CNavigatorManualSequence *>(this), "saveConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CNavigatorManualSequence::saveConfigFile(a0);
	}
	void initialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CNavigatorManualSequence *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CNavigatorManualSequence::initialize();
	}
	void navigationStep() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CNavigatorManualSequence *>(this), "navigationStep");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CNavigatorManualSequence::navigationStep();
	}
	void onStartNewNavigation() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CNavigatorManualSequence *>(this), "onStartNewNavigation");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CNavigatorManualSequence::onStartNewNavigation();
	}
	void cancel() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CNavigatorManualSequence *>(this), "cancel");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CNavigatorManualSequence *>(this), "resume");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CNavigatorManualSequence *>(this), "suspend");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CNavigatorManualSequence *>(this), "resetNavError");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CNavigatorManualSequence *>(this), "performNavigationStep");
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
	void onNavigateCommandReceived() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CNavigatorManualSequence *>(this), "onNavigateCommandReceived");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CNavigatorManualSequence *>(this), "updateCurrentPoseAndSpeeds");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CNavigatorManualSequence *>(this), "performNavigationStepNavigating");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CNavigatorManualSequence *>(this), "doEmergencyStop");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CNavigatorManualSequence *>(this), "changeSpeeds");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CNavigatorManualSequence *>(this), "changeSpeedsNOP");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CNavigatorManualSequence *>(this), "stop");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CNavigatorManualSequence *>(this), "checkHasReachedTarget");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CNavigatorManualSequence *>(this), "checkCollisionWithLatestObstacles");
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

// mrpt::nav::CReactiveNavigationSystem file:mrpt/nav/reactive/CReactiveNavigationSystem.h line:59
struct PyCallBack_mrpt_nav_CReactiveNavigationSystem : public mrpt::nav::CReactiveNavigationSystem {
	using mrpt::nav::CReactiveNavigationSystem::CReactiveNavigationSystem;

	size_t getPTG_count() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem *>(this), "getPTG_count");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CReactiveNavigationSystem::getPTG_count();
	}
	class mrpt::nav::CParameterizedTrajectoryGenerator * getPTG(size_t a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem *>(this), "getPTG");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::nav::CParameterizedTrajectoryGenerator *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::nav::CParameterizedTrajectoryGenerator *> caster;
				return pybind11::detail::cast_ref<class mrpt::nav::CParameterizedTrajectoryGenerator *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::nav::CParameterizedTrajectoryGenerator *>(std::move(o));
		}
		return CReactiveNavigationSystem::getPTG(a0);
	}
	bool checkCollisionWithLatestObstacles(const struct mrpt::math::TPose2D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem *>(this), "checkCollisionWithLatestObstacles");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CReactiveNavigationSystem::checkCollisionWithLatestObstacles(a0);
	}
	void loadConfigFile(const class mrpt::config::CConfigFileBase & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem *>(this), "loadConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CReactiveNavigationSystem::loadConfigFile(a0);
	}
	void saveConfigFile(class mrpt::config::CConfigFileBase & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem *>(this), "saveConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CReactiveNavigationSystem::saveConfigFile(a0);
	}
	void loggingGetWSObstaclesAndShape(class mrpt::nav::CLogFileRecord & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem *>(this), "loggingGetWSObstaclesAndShape");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CReactiveNavigationSystem::loggingGetWSObstaclesAndShape(a0);
	}
	void initialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem *>(this), "initialize");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem *>(this), "performNavigationStep");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem *>(this), "getHoloMethod");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem *>(this), "impl_waypoint_is_reachable");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem *>(this), "STEP1_InitPTGs");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem *>(this), "implementSenseObstacles");
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
	double generate_vel_cmd(const struct mrpt::nav::TCandidateMovementPTG & a0, class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem *>(this), "generate_vel_cmd");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem *>(this), "onStartNewNavigation");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem *>(this), "navigationStep");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem *>(this), "cancel");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem *>(this), "navigateWaypoints");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem *>(this), "getWaypointNavStatus");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem *>(this), "onNavigateCommandReceived");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem *>(this), "checkHasReachedTarget");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem *>(this), "waypoints_navigationStep");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem *>(this), "resume");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem *>(this), "suspend");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem *>(this), "resetNavError");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem *>(this), "updateCurrentPoseAndSpeeds");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem *>(this), "performNavigationStepNavigating");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem *>(this), "doEmergencyStop");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem *>(this), "changeSpeeds");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem *>(this), "changeSpeedsNOP");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem *>(this), "stop");
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

// mrpt::nav::CReactiveNavigationSystem::TReactiveNavigatorParams file:mrpt/nav/reactive/CReactiveNavigationSystem.h line:95
struct PyCallBack_mrpt_nav_CReactiveNavigationSystem_TReactiveNavigatorParams : public mrpt::nav::CReactiveNavigationSystem::TReactiveNavigatorParams {
	using mrpt::nav::CReactiveNavigationSystem::TReactiveNavigatorParams::TReactiveNavigatorParams;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem::TReactiveNavigatorParams *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TReactiveNavigatorParams::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CReactiveNavigationSystem::TReactiveNavigatorParams *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TReactiveNavigatorParams::saveToConfigFile(a0, a1);
	}
};

void bind_mrpt_nav_reactive_CMultiObjMotionOpt_Scalarization(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::nav::CMultiObjMotionOpt_Scalarization file:mrpt/nav/reactive/CMultiObjMotionOpt_Scalarization.h line:25
		pybind11::class_<mrpt::nav::CMultiObjMotionOpt_Scalarization, std::shared_ptr<mrpt::nav::CMultiObjMotionOpt_Scalarization>, PyCallBack_mrpt_nav_CMultiObjMotionOpt_Scalarization, mrpt::nav::CMultiObjectiveMotionOptimizerBase> cl(M("mrpt::nav"), "CMultiObjMotionOpt_Scalarization", "Implementation of multi-objective motion chooser using scalarization: a\n user-given formula is used to\n collapse all the scores into a single scalar score. The candidate with the\n highest positive score is selected.\n Note that assert expressions are honored via the base class\n CMultiObjectiveMotionOptimizerBase\n\n \n CReactiveNavigationSystem, CReactiveNavigationSystem3D\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::nav::CMultiObjMotionOpt_Scalarization(); }, [](){ return new PyCallBack_mrpt_nav_CMultiObjMotionOpt_Scalarization(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_nav_CMultiObjMotionOpt_Scalarization const &o){ return new PyCallBack_mrpt_nav_CMultiObjMotionOpt_Scalarization(o); } ) );
		cl.def( pybind11::init( [](mrpt::nav::CMultiObjMotionOpt_Scalarization const &o){ return new mrpt::nav::CMultiObjMotionOpt_Scalarization(o); } ) );
		cl.def_readwrite("parameters", &mrpt::nav::CMultiObjMotionOpt_Scalarization::parameters);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::nav::CMultiObjMotionOpt_Scalarization::GetRuntimeClassIdStatic, "C++: mrpt::nav::CMultiObjMotionOpt_Scalarization::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::nav::CMultiObjMotionOpt_Scalarization::*)() const) &mrpt::nav::CMultiObjMotionOpt_Scalarization::GetRuntimeClass, "C++: mrpt::nav::CMultiObjMotionOpt_Scalarization::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::nav::CMultiObjMotionOpt_Scalarization::*)() const) &mrpt::nav::CMultiObjMotionOpt_Scalarization::clone, "C++: mrpt::nav::CMultiObjMotionOpt_Scalarization::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::nav::CMultiObjMotionOpt_Scalarization::CreateObject, "C++: mrpt::nav::CMultiObjMotionOpt_Scalarization::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("loadConfigFile", (void (mrpt::nav::CMultiObjMotionOpt_Scalarization::*)(const class mrpt::config::CConfigFileBase &)) &mrpt::nav::CMultiObjMotionOpt_Scalarization::loadConfigFile, "C++: mrpt::nav::CMultiObjMotionOpt_Scalarization::loadConfigFile(const class mrpt::config::CConfigFileBase &) --> void", pybind11::arg("c"));
		cl.def("saveConfigFile", (void (mrpt::nav::CMultiObjMotionOpt_Scalarization::*)(class mrpt::config::CConfigFileBase &) const) &mrpt::nav::CMultiObjMotionOpt_Scalarization::saveConfigFile, "C++: mrpt::nav::CMultiObjMotionOpt_Scalarization::saveConfigFile(class mrpt::config::CConfigFileBase &) const --> void", pybind11::arg("c"));
		cl.def("clear", (void (mrpt::nav::CMultiObjMotionOpt_Scalarization::*)()) &mrpt::nav::CMultiObjMotionOpt_Scalarization::clear, "C++: mrpt::nav::CMultiObjMotionOpt_Scalarization::clear() --> void");

		{ // mrpt::nav::CMultiObjMotionOpt_Scalarization::TParams file:mrpt/nav/reactive/CMultiObjMotionOpt_Scalarization.h line:35
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::nav::CMultiObjMotionOpt_Scalarization::TParams, std::shared_ptr<mrpt::nav::CMultiObjMotionOpt_Scalarization::TParams>, PyCallBack_mrpt_nav_CMultiObjMotionOpt_Scalarization_TParams, mrpt::nav::CMultiObjectiveMotionOptimizerBase::TParamsBase> cl(enclosing_class, "TParams", "");
			cl.def( pybind11::init( [](){ return new mrpt::nav::CMultiObjMotionOpt_Scalarization::TParams(); }, [](){ return new PyCallBack_mrpt_nav_CMultiObjMotionOpt_Scalarization_TParams(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_nav_CMultiObjMotionOpt_Scalarization_TParams const &o){ return new PyCallBack_mrpt_nav_CMultiObjMotionOpt_Scalarization_TParams(o); } ) );
			cl.def( pybind11::init( [](mrpt::nav::CMultiObjMotionOpt_Scalarization::TParams const &o){ return new mrpt::nav::CMultiObjMotionOpt_Scalarization::TParams(o); } ) );
			cl.def_readwrite("scalar_score_formula", &mrpt::nav::CMultiObjMotionOpt_Scalarization::TParams::scalar_score_formula);
			cl.def("loadFromConfigFile", (void (mrpt::nav::CMultiObjMotionOpt_Scalarization::TParams::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::nav::CMultiObjMotionOpt_Scalarization::TParams::loadFromConfigFile, "C++: mrpt::nav::CMultiObjMotionOpt_Scalarization::TParams::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
			cl.def("saveToConfigFile", (void (mrpt::nav::CMultiObjMotionOpt_Scalarization::TParams::*)(class mrpt::config::CConfigFileBase &, const std::string &) const) &mrpt::nav::CMultiObjMotionOpt_Scalarization::TParams::saveToConfigFile, "C++: mrpt::nav::CMultiObjMotionOpt_Scalarization::TParams::saveToConfigFile(class mrpt::config::CConfigFileBase &, const std::string &) const --> void", pybind11::arg("cfg"), pybind11::arg("section"));
			cl.def("assign", (struct mrpt::nav::CMultiObjMotionOpt_Scalarization::TParams & (mrpt::nav::CMultiObjMotionOpt_Scalarization::TParams::*)(const struct mrpt::nav::CMultiObjMotionOpt_Scalarization::TParams &)) &mrpt::nav::CMultiObjMotionOpt_Scalarization::TParams::operator=, "C++: mrpt::nav::CMultiObjMotionOpt_Scalarization::TParams::operator=(const struct mrpt::nav::CMultiObjMotionOpt_Scalarization::TParams &) --> struct mrpt::nav::CMultiObjMotionOpt_Scalarization::TParams &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
	{ // mrpt::nav::CNavigatorManualSequence file:mrpt/nav/reactive/CNavigatorManualSequence.h line:22
		pybind11::class_<mrpt::nav::CNavigatorManualSequence, std::shared_ptr<mrpt::nav::CNavigatorManualSequence>, PyCallBack_mrpt_nav_CNavigatorManualSequence, mrpt::nav::CAbstractNavigator> cl(M("mrpt::nav"), "CNavigatorManualSequence", "\"Fake navigator\" for tests: it just sends out a pre-programmed sequence of\n commands to the robot.\n For a short discussion of the API, see CNavigatorVirtualBase");
		cl.def( pybind11::init<class mrpt::nav::CRobot2NavInterface &>(), pybind11::arg("react_iterf_impl") );

		cl.def_readwrite("programmed_orders", &mrpt::nav::CNavigatorManualSequence::programmed_orders);
		cl.def("loadConfigFile", (void (mrpt::nav::CNavigatorManualSequence::*)(const class mrpt::config::CConfigFileBase &)) &mrpt::nav::CNavigatorManualSequence::loadConfigFile, "@{ \n\nC++: mrpt::nav::CNavigatorManualSequence::loadConfigFile(const class mrpt::config::CConfigFileBase &) --> void", pybind11::arg("c"));
		cl.def("saveConfigFile", (void (mrpt::nav::CNavigatorManualSequence::*)(class mrpt::config::CConfigFileBase &) const) &mrpt::nav::CNavigatorManualSequence::saveConfigFile, "C++: mrpt::nav::CNavigatorManualSequence::saveConfigFile(class mrpt::config::CConfigFileBase &) const --> void", pybind11::arg("c"));
		cl.def("initialize", (void (mrpt::nav::CNavigatorManualSequence::*)()) &mrpt::nav::CNavigatorManualSequence::initialize, "Must be called for loading collision grids, etc. before invoking any\n navigation command \n\nC++: mrpt::nav::CNavigatorManualSequence::initialize() --> void");
		cl.def("navigationStep", (void (mrpt::nav::CNavigatorManualSequence::*)()) &mrpt::nav::CNavigatorManualSequence::navigationStep, "Overriden in this class to ignore the cancel/pause/... commands \n\nC++: mrpt::nav::CNavigatorManualSequence::navigationStep() --> void");

		{ // mrpt::nav::CNavigatorManualSequence::TVelCmd file:mrpt/nav/reactive/CNavigatorManualSequence.h line:40
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::nav::CNavigatorManualSequence::TVelCmd, std::shared_ptr<mrpt::nav::CNavigatorManualSequence::TVelCmd>> cl(enclosing_class, "TVelCmd", "");
			cl.def( pybind11::init( [](){ return new mrpt::nav::CNavigatorManualSequence::TVelCmd(); } ) );
			cl.def( pybind11::init( [](mrpt::nav::CNavigatorManualSequence::TVelCmd const &o){ return new mrpt::nav::CNavigatorManualSequence::TVelCmd(o); } ) );
			cl.def_readwrite("cmd_vel", &mrpt::nav::CNavigatorManualSequence::TVelCmd::cmd_vel);
			cl.def("assign", (struct mrpt::nav::CNavigatorManualSequence::TVelCmd & (mrpt::nav::CNavigatorManualSequence::TVelCmd::*)(const struct mrpt::nav::CNavigatorManualSequence::TVelCmd &)) &mrpt::nav::CNavigatorManualSequence::TVelCmd::operator=, "C++: mrpt::nav::CNavigatorManualSequence::TVelCmd::operator=(const struct mrpt::nav::CNavigatorManualSequence::TVelCmd &) --> struct mrpt::nav::CNavigatorManualSequence::TVelCmd &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
	{ // mrpt::nav::CReactiveNavigationSystem file:mrpt/nav/reactive/CReactiveNavigationSystem.h line:59
		pybind11::class_<mrpt::nav::CReactiveNavigationSystem, std::shared_ptr<mrpt::nav::CReactiveNavigationSystem>, PyCallBack_mrpt_nav_CReactiveNavigationSystem, mrpt::nav::CAbstractPTGBasedReactive> cl(M("mrpt::nav"), "CReactiveNavigationSystem", "See base class CAbstractPTGBasedReactive for a description and instructions\n of use.\n This particular implementation assumes a 2D robot shape which can be\n polygonal or circular (depending on the selected PTGs).\n\n Publications:\n  - Blanco, Jose-Luis, Javier Gonzalez, and Juan-Antonio Fernandez-Madrigal.\n [\"Extending obstacle avoidance methods through multiple parameter-space\n transformations\"](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.190.4672&rep=rep1&type=pdf).\n Autonomous Robots 24.1 (2008): 29-48.\n\n Class history:\n - 17/JUN/2004: First design.\n - 16/SEP/2004: Totally redesigned, according to document \"MultiParametric\n Based Space Transformation for Reactive Navigation\"\n - 29/SEP/2005: Totally rewritten again, for integration into MRPT library and\n according to the ICRA paper.\n - 17/OCT/2007: Whole code updated to accomodate to MRPT 0.5 and make it\n portable to Linux.\n - DEC/2013: Code refactoring between this class and\n CAbstractHolonomicReactiveMethod\n - FEB/2017: Refactoring of all parameters for a consistent organization in\n sections by class names (MRPT 1.5.0)\n\n This class requires a number of parameters which are usually provided via an\n external config (\".ini\") file.\n Alternatively, a memory-only object can be used to avoid physical files, see\n mrpt::config::CConfigFileMemory.\n\n A template config file can be generated at any moment by the user by calling\n saveConfigFile() with a default-constructed object.\n\n Next we provide a self-documented template config file; or see it online:\n https://github.com/MRPT/mrpt/blob/master/share/mrpt/config_files/navigation-ptgs/reactive2d_config.ini\n \n\n\n  \n CAbstractNavigator, CParameterizedTrajectoryGenerator,\n CAbstractHolonomicReactiveMethod\n  \n\n\n ");
		cl.def( pybind11::init( [](class mrpt::nav::CRobot2NavInterface & a0){ return new mrpt::nav::CReactiveNavigationSystem(a0); }, [](class mrpt::nav::CRobot2NavInterface & a0){ return new PyCallBack_mrpt_nav_CReactiveNavigationSystem(a0); } ), "doc");
		cl.def( pybind11::init( [](class mrpt::nav::CRobot2NavInterface & a0, bool const & a1){ return new mrpt::nav::CReactiveNavigationSystem(a0, a1); }, [](class mrpt::nav::CRobot2NavInterface & a0, bool const & a1){ return new PyCallBack_mrpt_nav_CReactiveNavigationSystem(a0, a1); } ), "doc");
		cl.def( pybind11::init( [](class mrpt::nav::CRobot2NavInterface & a0, bool const & a1, bool const & a2){ return new mrpt::nav::CReactiveNavigationSystem(a0, a1, a2); }, [](class mrpt::nav::CRobot2NavInterface & a0, bool const & a1, bool const & a2){ return new PyCallBack_mrpt_nav_CReactiveNavigationSystem(a0, a1, a2); } ), "doc");
		cl.def( pybind11::init<class mrpt::nav::CRobot2NavInterface &, bool, bool, const std::string &>(), pybind11::arg("react_iterf_impl"), pybind11::arg("enableConsoleOutput"), pybind11::arg("enableLogFile"), pybind11::arg("logFileDirectory") );

		cl.def_readwrite("params_reactive_nav", &mrpt::nav::CReactiveNavigationSystem::params_reactive_nav);
		cl.def("changeRobotShape", (void (mrpt::nav::CReactiveNavigationSystem::*)(const class mrpt::math::CPolygon &)) &mrpt::nav::CReactiveNavigationSystem::changeRobotShape, "Defines the 2D polygonal robot shape, used for some PTGs for collision\n checking. \n\nC++: mrpt::nav::CReactiveNavigationSystem::changeRobotShape(const class mrpt::math::CPolygon &) --> void", pybind11::arg("shape"));
		cl.def("changeRobotCircularShapeRadius", (void (mrpt::nav::CReactiveNavigationSystem::*)(const double)) &mrpt::nav::CReactiveNavigationSystem::changeRobotCircularShapeRadius, "Defines the 2D circular robot shape radius, used for some PTGs for\n collision checking. \n\nC++: mrpt::nav::CReactiveNavigationSystem::changeRobotCircularShapeRadius(const double) --> void", pybind11::arg("R"));
		cl.def("getPTG_count", (size_t (mrpt::nav::CReactiveNavigationSystem::*)() const) &mrpt::nav::CReactiveNavigationSystem::getPTG_count, "C++: mrpt::nav::CReactiveNavigationSystem::getPTG_count() const --> size_t");
		cl.def("getPTG", (class mrpt::nav::CParameterizedTrajectoryGenerator * (mrpt::nav::CReactiveNavigationSystem::*)(size_t)) &mrpt::nav::CReactiveNavigationSystem::getPTG, "C++: mrpt::nav::CReactiveNavigationSystem::getPTG(size_t) --> class mrpt::nav::CParameterizedTrajectoryGenerator *", pybind11::return_value_policy::automatic, pybind11::arg("i"));
		cl.def("checkCollisionWithLatestObstacles", (bool (mrpt::nav::CReactiveNavigationSystem::*)(const struct mrpt::math::TPose2D &) const) &mrpt::nav::CReactiveNavigationSystem::checkCollisionWithLatestObstacles, "C++: mrpt::nav::CReactiveNavigationSystem::checkCollisionWithLatestObstacles(const struct mrpt::math::TPose2D &) const --> bool", pybind11::arg("relative_robot_pose"));
		cl.def("loadConfigFile", (void (mrpt::nav::CReactiveNavigationSystem::*)(const class mrpt::config::CConfigFileBase &)) &mrpt::nav::CReactiveNavigationSystem::loadConfigFile, "C++: mrpt::nav::CReactiveNavigationSystem::loadConfigFile(const class mrpt::config::CConfigFileBase &) --> void", pybind11::arg("c"));
		cl.def("saveConfigFile", (void (mrpt::nav::CReactiveNavigationSystem::*)(class mrpt::config::CConfigFileBase &) const) &mrpt::nav::CReactiveNavigationSystem::saveConfigFile, "C++: mrpt::nav::CReactiveNavigationSystem::saveConfigFile(class mrpt::config::CConfigFileBase &) const --> void", pybind11::arg("c"));

		{ // mrpt::nav::CReactiveNavigationSystem::TReactiveNavigatorParams file:mrpt/nav/reactive/CReactiveNavigationSystem.h line:95
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::nav::CReactiveNavigationSystem::TReactiveNavigatorParams, std::shared_ptr<mrpt::nav::CReactiveNavigationSystem::TReactiveNavigatorParams>, PyCallBack_mrpt_nav_CReactiveNavigationSystem_TReactiveNavigatorParams, mrpt::config::CLoadableOptions> cl(enclosing_class, "TReactiveNavigatorParams", "");
			cl.def( pybind11::init( [](){ return new mrpt::nav::CReactiveNavigationSystem::TReactiveNavigatorParams(); }, [](){ return new PyCallBack_mrpt_nav_CReactiveNavigationSystem_TReactiveNavigatorParams(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_nav_CReactiveNavigationSystem_TReactiveNavigatorParams const &o){ return new PyCallBack_mrpt_nav_CReactiveNavigationSystem_TReactiveNavigatorParams(o); } ) );
			cl.def( pybind11::init( [](mrpt::nav::CReactiveNavigationSystem::TReactiveNavigatorParams const &o){ return new mrpt::nav::CReactiveNavigationSystem::TReactiveNavigatorParams(o); } ) );
			cl.def_readwrite("min_obstacles_height", &mrpt::nav::CReactiveNavigationSystem::TReactiveNavigatorParams::min_obstacles_height);
			cl.def_readwrite("max_obstacles_height", &mrpt::nav::CReactiveNavigationSystem::TReactiveNavigatorParams::max_obstacles_height);
			cl.def("loadFromConfigFile", (void (mrpt::nav::CReactiveNavigationSystem::TReactiveNavigatorParams::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::nav::CReactiveNavigationSystem::TReactiveNavigatorParams::loadFromConfigFile, "C++: mrpt::nav::CReactiveNavigationSystem::TReactiveNavigatorParams::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("c"), pybind11::arg("s"));
			cl.def("saveToConfigFile", (void (mrpt::nav::CReactiveNavigationSystem::TReactiveNavigatorParams::*)(class mrpt::config::CConfigFileBase &, const std::string &) const) &mrpt::nav::CReactiveNavigationSystem::TReactiveNavigatorParams::saveToConfigFile, "C++: mrpt::nav::CReactiveNavigationSystem::TReactiveNavigatorParams::saveToConfigFile(class mrpt::config::CConfigFileBase &, const std::string &) const --> void", pybind11::arg("c"), pybind11::arg("s"));
			cl.def("assign", (struct mrpt::nav::CReactiveNavigationSystem::TReactiveNavigatorParams & (mrpt::nav::CReactiveNavigationSystem::TReactiveNavigatorParams::*)(const struct mrpt::nav::CReactiveNavigationSystem::TReactiveNavigatorParams &)) &mrpt::nav::CReactiveNavigationSystem::TReactiveNavigatorParams::operator=, "C++: mrpt::nav::CReactiveNavigationSystem::TReactiveNavigatorParams::operator=(const struct mrpt::nav::CReactiveNavigationSystem::TReactiveNavigatorParams &) --> struct mrpt::nav::CReactiveNavigationSystem::TReactiveNavigatorParams &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
}
