#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/kinematics/CVehicleVelCmd.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TSegment2D.h>
#include <mrpt/math/TSegment3D.h>
#include <mrpt/math/TTwist2D.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/nav/tpspace/CPTG_DiffDrive_C.h>
#include <mrpt/nav/tpspace/CPTG_DiffDrive_CC.h>
#include <mrpt/nav/tpspace/CPTG_DiffDrive_CCS.h>
#include <mrpt/nav/tpspace/CPTG_DiffDrive_CS.h>
#include <mrpt/nav/tpspace/CPTG_DiffDrive_CollisionGridBased.h>
#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <sstream> // __str__
#include <string>
#include <utility>
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

// mrpt::nav::CPTG_DiffDrive_C file:mrpt/nav/tpspace/CPTG_DiffDrive_C.h line:40
struct PyCallBack_mrpt_nav_CPTG_DiffDrive_C : public mrpt::nav::CPTG_DiffDrive_C {
	using mrpt::nav::CPTG_DiffDrive_C::CPTG_DiffDrive_C;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPTG_DiffDrive_C::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CPTG_DiffDrive_C::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CPTG_DiffDrive_C::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_C::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_C::serializeFrom(a0, a1);
	}
	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_C::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_C::saveToConfigFile(a0, a1);
	}
	std::string getDescription() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "getDescription");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CPTG_DiffDrive_C::getDescription();
	}
	bool inverseMap_WS2TP(double a0, double a1, int & a2, double & a3, double a4) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "inverseMap_WS2TP");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPTG_DiffDrive_C::inverseMap_WS2TP(a0, a1, a2, a3, a4);
	}
	bool PTG_IsIntoDomain(double a0, double a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "PTG_IsIntoDomain");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPTG_DiffDrive_C::PTG_IsIntoDomain(a0, a1);
	}
	void ptgDiffDriveSteeringFunction(float a0, float a1, float a2, float a3, float a4, float & a5, float & a6) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "ptgDiffDriveSteeringFunction");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4, a5, a6);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_C::ptgDiffDriveSteeringFunction(a0, a1, a2, a3, a4, a5, a6);
	}
	void loadDefaultParams() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "loadDefaultParams");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_C::loadDefaultParams();
	}
	class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> directionToMotionCommand(uint16_t a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "directionToMotionCommand");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>::value) {
				static pybind11::detail::override_caster_t<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>> caster;
				return pybind11::detail::cast_ref<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::directionToMotionCommand(a0);
	}
	class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> getSupportedKinematicVelocityCommand() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "getSupportedKinematicVelocityCommand");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>::value) {
				static pybind11::detail::override_caster_t<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>> caster;
				return pybind11::detail::cast_ref<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::getSupportedKinematicVelocityCommand();
	}
	void setRefDistance(const double a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "setRefDistance");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::setRefDistance(a0);
	}
	size_t getPathStepCount(uint16_t a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "getPathStepCount");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::getPathStepCount(a0);
	}
	struct mrpt::math::TPose2D getPathPose(uint16_t a0, uint32_t a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "getPathPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TPose2D>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TPose2D> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TPose2D>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TPose2D>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::getPathPose(a0, a1);
	}
	double getPathDist(uint16_t a0, uint32_t a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "getPathDist");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::getPathDist(a0, a1);
	}
	bool getPathStepForDist(uint16_t a0, double a1, unsigned int & a2) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "getPathStepForDist");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::getPathStepForDist(a0, a1, a2);
	}
	double getPathStepDuration() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "getPathStepDuration");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::getPathStepDuration();
	}
	double getMaxLinVel() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "getMaxLinVel");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::getMaxLinVel();
	}
	double getMaxAngVel() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "getMaxAngVel");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::getMaxAngVel();
	}
	void updateTPObstacleSingle(double a0, double a1, uint16_t a2, double & a3) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "updateTPObstacleSingle");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::updateTPObstacleSingle(a0, a1, a2, a3);
	}
	void onNewNavDynamicState() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "onNewNavDynamicState");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::onNewNavDynamicState();
	}
	void internal_processNewRobotShape() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "internal_processNewRobotShape");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::internal_processNewRobotShape();
	}
	void internal_initialize(const std::string & a0, const bool a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "internal_initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::internal_initialize(a0, a1);
	}
	void internal_deinitialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "internal_deinitialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::internal_deinitialize();
	}
	void internal_readFromStream(class mrpt::serialization::CArchive & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "internal_readFromStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::internal_readFromStream(a0);
	}
	void internal_writeToStream(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "internal_writeToStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::internal_writeToStream(a0);
	}
	struct mrpt::math::TTwist2D getPathTwist(uint16_t a0, uint32_t a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "getPathTwist");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TTwist2D>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TTwist2D> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TTwist2D>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TTwist2D>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::getPathTwist(a0, a1);
	}
	double getMaxRobotRadius() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "getMaxRobotRadius");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CPTG_RobotShape_Polygonal::getMaxRobotRadius();
	}
	double evalClearanceToRobotShape(const double a0, const double a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "evalClearanceToRobotShape");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CPTG_RobotShape_Polygonal::evalClearanceToRobotShape(a0, a1);
	}
	bool isPointInsideRobotShape(const double a0, const double a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "isPointInsideRobotShape");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPTG_RobotShape_Polygonal::isPointInsideRobotShape(a0, a1);
	}
	void add_robotShape_to_setOfLines(class mrpt::opengl::CSetOfLines & a0, const class mrpt::poses::CPose2D & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "add_robotShape_to_setOfLines");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_RobotShape_Polygonal::add_robotShape_to_setOfLines(a0, a1);
	}
	bool isBijectiveAt(uint16_t a0, uint32_t a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "isBijectiveAt");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CParameterizedTrajectoryGenerator::isBijectiveAt(a0, a1);
	}
	bool supportVelCmdNOP() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "supportVelCmdNOP");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CParameterizedTrajectoryGenerator::supportVelCmdNOP();
	}
	bool supportSpeedAtTarget() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "supportSpeedAtTarget");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CParameterizedTrajectoryGenerator::supportSpeedAtTarget();
	}
	double maxTimeInVelCmdNOP(int a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "maxTimeInVelCmdNOP");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CParameterizedTrajectoryGenerator::maxTimeInVelCmdNOP(a0);
	}
	double getActualUnloopedPathLength(uint16_t a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "getActualUnloopedPathLength");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CParameterizedTrajectoryGenerator::getActualUnloopedPathLength(a0);
	}
	double evalPathRelativePriority(uint16_t a0, double a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "evalPathRelativePriority");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CParameterizedTrajectoryGenerator::evalPathRelativePriority(a0, a1);
	}
	void renderPathAsSimpleLine(const unsigned short a0, class mrpt::opengl::CSetOfLines & a1, const double a2, const double a3) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "renderPathAsSimpleLine");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CParameterizedTrajectoryGenerator::renderPathAsSimpleLine(a0, a1, a2, a3);
	}
	void evalClearanceSingleObstacle(const double a0, const double a1, const unsigned short a2, class std::map<double, double> & a3, bool a4) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_C *>(this), "evalClearanceSingleObstacle");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CParameterizedTrajectoryGenerator::evalClearanceSingleObstacle(a0, a1, a2, a3, a4);
	}
};

// mrpt::nav::CPTG_DiffDrive_CC file:mrpt/nav/tpspace/CPTG_DiffDrive_CC.h line:25
struct PyCallBack_mrpt_nav_CPTG_DiffDrive_CC : public mrpt::nav::CPTG_DiffDrive_CC {
	using mrpt::nav::CPTG_DiffDrive_CC::CPTG_DiffDrive_CC;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPTG_DiffDrive_CC::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CPTG_DiffDrive_CC::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CPTG_DiffDrive_CC::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CC::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CC::serializeFrom(a0, a1);
	}
	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CC::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CC::saveToConfigFile(a0, a1);
	}
	std::string getDescription() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "getDescription");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CPTG_DiffDrive_CC::getDescription();
	}
	bool PTG_IsIntoDomain(double a0, double a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "PTG_IsIntoDomain");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPTG_DiffDrive_CC::PTG_IsIntoDomain(a0, a1);
	}
	void ptgDiffDriveSteeringFunction(float a0, float a1, float a2, float a3, float a4, float & a5, float & a6) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "ptgDiffDriveSteeringFunction");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4, a5, a6);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CC::ptgDiffDriveSteeringFunction(a0, a1, a2, a3, a4, a5, a6);
	}
	void loadDefaultParams() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "loadDefaultParams");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CC::loadDefaultParams();
	}
	bool inverseMap_WS2TP(double a0, double a1, int & a2, double & a3, double a4) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "inverseMap_WS2TP");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::inverseMap_WS2TP(a0, a1, a2, a3, a4);
	}
	class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> directionToMotionCommand(uint16_t a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "directionToMotionCommand");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>::value) {
				static pybind11::detail::override_caster_t<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>> caster;
				return pybind11::detail::cast_ref<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::directionToMotionCommand(a0);
	}
	class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> getSupportedKinematicVelocityCommand() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "getSupportedKinematicVelocityCommand");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>::value) {
				static pybind11::detail::override_caster_t<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>> caster;
				return pybind11::detail::cast_ref<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::getSupportedKinematicVelocityCommand();
	}
	void setRefDistance(const double a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "setRefDistance");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::setRefDistance(a0);
	}
	size_t getPathStepCount(uint16_t a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "getPathStepCount");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::getPathStepCount(a0);
	}
	struct mrpt::math::TPose2D getPathPose(uint16_t a0, uint32_t a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "getPathPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TPose2D>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TPose2D> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TPose2D>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TPose2D>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::getPathPose(a0, a1);
	}
	double getPathDist(uint16_t a0, uint32_t a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "getPathDist");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::getPathDist(a0, a1);
	}
	bool getPathStepForDist(uint16_t a0, double a1, unsigned int & a2) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "getPathStepForDist");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::getPathStepForDist(a0, a1, a2);
	}
	double getPathStepDuration() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "getPathStepDuration");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::getPathStepDuration();
	}
	double getMaxLinVel() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "getMaxLinVel");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::getMaxLinVel();
	}
	double getMaxAngVel() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "getMaxAngVel");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::getMaxAngVel();
	}
	void updateTPObstacleSingle(double a0, double a1, uint16_t a2, double & a3) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "updateTPObstacleSingle");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::updateTPObstacleSingle(a0, a1, a2, a3);
	}
	void onNewNavDynamicState() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "onNewNavDynamicState");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::onNewNavDynamicState();
	}
	void internal_processNewRobotShape() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "internal_processNewRobotShape");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::internal_processNewRobotShape();
	}
	void internal_initialize(const std::string & a0, const bool a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "internal_initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::internal_initialize(a0, a1);
	}
	void internal_deinitialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "internal_deinitialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::internal_deinitialize();
	}
	void internal_readFromStream(class mrpt::serialization::CArchive & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "internal_readFromStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::internal_readFromStream(a0);
	}
	void internal_writeToStream(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "internal_writeToStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::internal_writeToStream(a0);
	}
	struct mrpt::math::TTwist2D getPathTwist(uint16_t a0, uint32_t a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "getPathTwist");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TTwist2D>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TTwist2D> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TTwist2D>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TTwist2D>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::getPathTwist(a0, a1);
	}
	double getMaxRobotRadius() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "getMaxRobotRadius");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CPTG_RobotShape_Polygonal::getMaxRobotRadius();
	}
	double evalClearanceToRobotShape(const double a0, const double a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "evalClearanceToRobotShape");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CPTG_RobotShape_Polygonal::evalClearanceToRobotShape(a0, a1);
	}
	bool isPointInsideRobotShape(const double a0, const double a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "isPointInsideRobotShape");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPTG_RobotShape_Polygonal::isPointInsideRobotShape(a0, a1);
	}
	void add_robotShape_to_setOfLines(class mrpt::opengl::CSetOfLines & a0, const class mrpt::poses::CPose2D & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "add_robotShape_to_setOfLines");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_RobotShape_Polygonal::add_robotShape_to_setOfLines(a0, a1);
	}
	bool isBijectiveAt(uint16_t a0, uint32_t a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "isBijectiveAt");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CParameterizedTrajectoryGenerator::isBijectiveAt(a0, a1);
	}
	bool supportVelCmdNOP() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "supportVelCmdNOP");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CParameterizedTrajectoryGenerator::supportVelCmdNOP();
	}
	bool supportSpeedAtTarget() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "supportSpeedAtTarget");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CParameterizedTrajectoryGenerator::supportSpeedAtTarget();
	}
	double maxTimeInVelCmdNOP(int a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "maxTimeInVelCmdNOP");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CParameterizedTrajectoryGenerator::maxTimeInVelCmdNOP(a0);
	}
	double getActualUnloopedPathLength(uint16_t a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "getActualUnloopedPathLength");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CParameterizedTrajectoryGenerator::getActualUnloopedPathLength(a0);
	}
	double evalPathRelativePriority(uint16_t a0, double a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "evalPathRelativePriority");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CParameterizedTrajectoryGenerator::evalPathRelativePriority(a0, a1);
	}
	void renderPathAsSimpleLine(const unsigned short a0, class mrpt::opengl::CSetOfLines & a1, const double a2, const double a3) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "renderPathAsSimpleLine");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CParameterizedTrajectoryGenerator::renderPathAsSimpleLine(a0, a1, a2, a3);
	}
	void evalClearanceSingleObstacle(const double a0, const double a1, const unsigned short a2, class std::map<double, double> & a3, bool a4) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CC *>(this), "evalClearanceSingleObstacle");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CParameterizedTrajectoryGenerator::evalClearanceSingleObstacle(a0, a1, a2, a3, a4);
	}
};

// mrpt::nav::CPTG_DiffDrive_CCS file:mrpt/nav/tpspace/CPTG_DiffDrive_CCS.h line:25
struct PyCallBack_mrpt_nav_CPTG_DiffDrive_CCS : public mrpt::nav::CPTG_DiffDrive_CCS {
	using mrpt::nav::CPTG_DiffDrive_CCS::CPTG_DiffDrive_CCS;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPTG_DiffDrive_CCS::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CPTG_DiffDrive_CCS::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CPTG_DiffDrive_CCS::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CCS::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CCS::serializeFrom(a0, a1);
	}
	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CCS::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CCS::saveToConfigFile(a0, a1);
	}
	std::string getDescription() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "getDescription");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CPTG_DiffDrive_CCS::getDescription();
	}
	bool PTG_IsIntoDomain(double a0, double a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "PTG_IsIntoDomain");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPTG_DiffDrive_CCS::PTG_IsIntoDomain(a0, a1);
	}
	void ptgDiffDriveSteeringFunction(float a0, float a1, float a2, float a3, float a4, float & a5, float & a6) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "ptgDiffDriveSteeringFunction");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4, a5, a6);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CCS::ptgDiffDriveSteeringFunction(a0, a1, a2, a3, a4, a5, a6);
	}
	void loadDefaultParams() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "loadDefaultParams");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CCS::loadDefaultParams();
	}
	bool inverseMap_WS2TP(double a0, double a1, int & a2, double & a3, double a4) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "inverseMap_WS2TP");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::inverseMap_WS2TP(a0, a1, a2, a3, a4);
	}
	class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> directionToMotionCommand(uint16_t a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "directionToMotionCommand");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>::value) {
				static pybind11::detail::override_caster_t<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>> caster;
				return pybind11::detail::cast_ref<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::directionToMotionCommand(a0);
	}
	class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> getSupportedKinematicVelocityCommand() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "getSupportedKinematicVelocityCommand");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>::value) {
				static pybind11::detail::override_caster_t<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>> caster;
				return pybind11::detail::cast_ref<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::getSupportedKinematicVelocityCommand();
	}
	void setRefDistance(const double a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "setRefDistance");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::setRefDistance(a0);
	}
	size_t getPathStepCount(uint16_t a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "getPathStepCount");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::getPathStepCount(a0);
	}
	struct mrpt::math::TPose2D getPathPose(uint16_t a0, uint32_t a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "getPathPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TPose2D>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TPose2D> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TPose2D>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TPose2D>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::getPathPose(a0, a1);
	}
	double getPathDist(uint16_t a0, uint32_t a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "getPathDist");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::getPathDist(a0, a1);
	}
	bool getPathStepForDist(uint16_t a0, double a1, unsigned int & a2) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "getPathStepForDist");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::getPathStepForDist(a0, a1, a2);
	}
	double getPathStepDuration() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "getPathStepDuration");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::getPathStepDuration();
	}
	double getMaxLinVel() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "getMaxLinVel");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::getMaxLinVel();
	}
	double getMaxAngVel() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "getMaxAngVel");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::getMaxAngVel();
	}
	void updateTPObstacleSingle(double a0, double a1, uint16_t a2, double & a3) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "updateTPObstacleSingle");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::updateTPObstacleSingle(a0, a1, a2, a3);
	}
	void onNewNavDynamicState() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "onNewNavDynamicState");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::onNewNavDynamicState();
	}
	void internal_processNewRobotShape() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "internal_processNewRobotShape");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::internal_processNewRobotShape();
	}
	void internal_initialize(const std::string & a0, const bool a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "internal_initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::internal_initialize(a0, a1);
	}
	void internal_deinitialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "internal_deinitialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::internal_deinitialize();
	}
	void internal_readFromStream(class mrpt::serialization::CArchive & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "internal_readFromStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::internal_readFromStream(a0);
	}
	void internal_writeToStream(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "internal_writeToStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::internal_writeToStream(a0);
	}
	struct mrpt::math::TTwist2D getPathTwist(uint16_t a0, uint32_t a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "getPathTwist");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TTwist2D>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TTwist2D> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TTwist2D>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TTwist2D>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::getPathTwist(a0, a1);
	}
	double getMaxRobotRadius() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "getMaxRobotRadius");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CPTG_RobotShape_Polygonal::getMaxRobotRadius();
	}
	double evalClearanceToRobotShape(const double a0, const double a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "evalClearanceToRobotShape");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CPTG_RobotShape_Polygonal::evalClearanceToRobotShape(a0, a1);
	}
	bool isPointInsideRobotShape(const double a0, const double a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "isPointInsideRobotShape");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPTG_RobotShape_Polygonal::isPointInsideRobotShape(a0, a1);
	}
	void add_robotShape_to_setOfLines(class mrpt::opengl::CSetOfLines & a0, const class mrpt::poses::CPose2D & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "add_robotShape_to_setOfLines");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_RobotShape_Polygonal::add_robotShape_to_setOfLines(a0, a1);
	}
	bool isBijectiveAt(uint16_t a0, uint32_t a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "isBijectiveAt");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CParameterizedTrajectoryGenerator::isBijectiveAt(a0, a1);
	}
	bool supportVelCmdNOP() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "supportVelCmdNOP");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CParameterizedTrajectoryGenerator::supportVelCmdNOP();
	}
	bool supportSpeedAtTarget() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "supportSpeedAtTarget");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CParameterizedTrajectoryGenerator::supportSpeedAtTarget();
	}
	double maxTimeInVelCmdNOP(int a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "maxTimeInVelCmdNOP");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CParameterizedTrajectoryGenerator::maxTimeInVelCmdNOP(a0);
	}
	double getActualUnloopedPathLength(uint16_t a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "getActualUnloopedPathLength");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CParameterizedTrajectoryGenerator::getActualUnloopedPathLength(a0);
	}
	double evalPathRelativePriority(uint16_t a0, double a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "evalPathRelativePriority");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CParameterizedTrajectoryGenerator::evalPathRelativePriority(a0, a1);
	}
	void renderPathAsSimpleLine(const unsigned short a0, class mrpt::opengl::CSetOfLines & a1, const double a2, const double a3) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "renderPathAsSimpleLine");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CParameterizedTrajectoryGenerator::renderPathAsSimpleLine(a0, a1, a2, a3);
	}
	void evalClearanceSingleObstacle(const double a0, const double a1, const unsigned short a2, class std::map<double, double> & a3, bool a4) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CCS *>(this), "evalClearanceSingleObstacle");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CParameterizedTrajectoryGenerator::evalClearanceSingleObstacle(a0, a1, a2, a3, a4);
	}
};

// mrpt::nav::CPTG_DiffDrive_CS file:mrpt/nav/tpspace/CPTG_DiffDrive_CS.h line:25
struct PyCallBack_mrpt_nav_CPTG_DiffDrive_CS : public mrpt::nav::CPTG_DiffDrive_CS {
	using mrpt::nav::CPTG_DiffDrive_CS::CPTG_DiffDrive_CS;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPTG_DiffDrive_CS::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CPTG_DiffDrive_CS::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CPTG_DiffDrive_CS::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CS::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CS::serializeFrom(a0, a1);
	}
	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CS::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CS::saveToConfigFile(a0, a1);
	}
	std::string getDescription() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "getDescription");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CPTG_DiffDrive_CS::getDescription();
	}
	bool PTG_IsIntoDomain(double a0, double a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "PTG_IsIntoDomain");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPTG_DiffDrive_CS::PTG_IsIntoDomain(a0, a1);
	}
	void ptgDiffDriveSteeringFunction(float a0, float a1, float a2, float a3, float a4, float & a5, float & a6) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "ptgDiffDriveSteeringFunction");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4, a5, a6);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CS::ptgDiffDriveSteeringFunction(a0, a1, a2, a3, a4, a5, a6);
	}
	void loadDefaultParams() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "loadDefaultParams");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CS::loadDefaultParams();
	}
	bool inverseMap_WS2TP(double a0, double a1, int & a2, double & a3, double a4) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "inverseMap_WS2TP");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::inverseMap_WS2TP(a0, a1, a2, a3, a4);
	}
	class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> directionToMotionCommand(uint16_t a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "directionToMotionCommand");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>::value) {
				static pybind11::detail::override_caster_t<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>> caster;
				return pybind11::detail::cast_ref<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::directionToMotionCommand(a0);
	}
	class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> getSupportedKinematicVelocityCommand() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "getSupportedKinematicVelocityCommand");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>::value) {
				static pybind11::detail::override_caster_t<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>> caster;
				return pybind11::detail::cast_ref<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::getSupportedKinematicVelocityCommand();
	}
	void setRefDistance(const double a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "setRefDistance");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::setRefDistance(a0);
	}
	size_t getPathStepCount(uint16_t a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "getPathStepCount");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::getPathStepCount(a0);
	}
	struct mrpt::math::TPose2D getPathPose(uint16_t a0, uint32_t a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "getPathPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TPose2D>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TPose2D> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TPose2D>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TPose2D>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::getPathPose(a0, a1);
	}
	double getPathDist(uint16_t a0, uint32_t a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "getPathDist");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::getPathDist(a0, a1);
	}
	bool getPathStepForDist(uint16_t a0, double a1, unsigned int & a2) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "getPathStepForDist");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::getPathStepForDist(a0, a1, a2);
	}
	double getPathStepDuration() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "getPathStepDuration");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::getPathStepDuration();
	}
	double getMaxLinVel() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "getMaxLinVel");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::getMaxLinVel();
	}
	double getMaxAngVel() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "getMaxAngVel");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::getMaxAngVel();
	}
	void updateTPObstacleSingle(double a0, double a1, uint16_t a2, double & a3) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "updateTPObstacleSingle");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::updateTPObstacleSingle(a0, a1, a2, a3);
	}
	void onNewNavDynamicState() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "onNewNavDynamicState");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::onNewNavDynamicState();
	}
	void internal_processNewRobotShape() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "internal_processNewRobotShape");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::internal_processNewRobotShape();
	}
	void internal_initialize(const std::string & a0, const bool a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "internal_initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::internal_initialize(a0, a1);
	}
	void internal_deinitialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "internal_deinitialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::internal_deinitialize();
	}
	void internal_readFromStream(class mrpt::serialization::CArchive & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "internal_readFromStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::internal_readFromStream(a0);
	}
	void internal_writeToStream(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "internal_writeToStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::internal_writeToStream(a0);
	}
	struct mrpt::math::TTwist2D getPathTwist(uint16_t a0, uint32_t a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "getPathTwist");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TTwist2D>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TTwist2D> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TTwist2D>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TTwist2D>(std::move(o));
		}
		return CPTG_DiffDrive_CollisionGridBased::getPathTwist(a0, a1);
	}
	double getMaxRobotRadius() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "getMaxRobotRadius");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CPTG_RobotShape_Polygonal::getMaxRobotRadius();
	}
	double evalClearanceToRobotShape(const double a0, const double a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "evalClearanceToRobotShape");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CPTG_RobotShape_Polygonal::evalClearanceToRobotShape(a0, a1);
	}
	bool isPointInsideRobotShape(const double a0, const double a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "isPointInsideRobotShape");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPTG_RobotShape_Polygonal::isPointInsideRobotShape(a0, a1);
	}
	void add_robotShape_to_setOfLines(class mrpt::opengl::CSetOfLines & a0, const class mrpt::poses::CPose2D & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "add_robotShape_to_setOfLines");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_RobotShape_Polygonal::add_robotShape_to_setOfLines(a0, a1);
	}
	bool isBijectiveAt(uint16_t a0, uint32_t a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "isBijectiveAt");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CParameterizedTrajectoryGenerator::isBijectiveAt(a0, a1);
	}
	bool supportVelCmdNOP() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "supportVelCmdNOP");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CParameterizedTrajectoryGenerator::supportVelCmdNOP();
	}
	bool supportSpeedAtTarget() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "supportSpeedAtTarget");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CParameterizedTrajectoryGenerator::supportSpeedAtTarget();
	}
	double maxTimeInVelCmdNOP(int a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "maxTimeInVelCmdNOP");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CParameterizedTrajectoryGenerator::maxTimeInVelCmdNOP(a0);
	}
	double getActualUnloopedPathLength(uint16_t a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "getActualUnloopedPathLength");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CParameterizedTrajectoryGenerator::getActualUnloopedPathLength(a0);
	}
	double evalPathRelativePriority(uint16_t a0, double a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "evalPathRelativePriority");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CParameterizedTrajectoryGenerator::evalPathRelativePriority(a0, a1);
	}
	void renderPathAsSimpleLine(const unsigned short a0, class mrpt::opengl::CSetOfLines & a1, const double a2, const double a3) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "renderPathAsSimpleLine");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CParameterizedTrajectoryGenerator::renderPathAsSimpleLine(a0, a1, a2, a3);
	}
	void evalClearanceSingleObstacle(const double a0, const double a1, const unsigned short a2, class std::map<double, double> & a3, bool a4) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_CS *>(this), "evalClearanceSingleObstacle");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CParameterizedTrajectoryGenerator::evalClearanceSingleObstacle(a0, a1, a2, a3, a4);
	}
};

void bind_mrpt_nav_tpspace_CPTG_DiffDrive_C(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::nav::CPTG_DiffDrive_C file:mrpt/nav/tpspace/CPTG_DiffDrive_C.h line:40
		pybind11::class_<mrpt::nav::CPTG_DiffDrive_C, std::shared_ptr<mrpt::nav::CPTG_DiffDrive_C>, PyCallBack_mrpt_nav_CPTG_DiffDrive_C, mrpt::nav::CPTG_DiffDrive_CollisionGridBased> cl(M("mrpt::nav"), "CPTG_DiffDrive_C", "A PTG for circular paths (\"C\" type PTG in papers).\n - **Compatible kinematics**: differential-driven / Ackermann steering\n - **Compatible robot shape**: Arbitrary 2D polygon\n - **PTG parameters**: Use the app `ptg-configurator`\n\n This PT generator functions are:\n\n \n\n \n\n So, the radius of curvature of each trajectory is constant for each \"alpha\"\n value (the trajectory parameter):\n\n  \n\n\n from which a minimum radius of curvature can be set by selecting the\n appropriate values of V_MAX and W_MAX,\n knowning that \n\n.\n\n  ![C-PTG path examples](PTG1_paths.png)\n\n \n [Before MRPT 1.5.0 this was named CPTG1]\n  \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::nav::CPTG_DiffDrive_C(); }, [](){ return new PyCallBack_mrpt_nav_CPTG_DiffDrive_C(); } ) );
		cl.def( pybind11::init<const class mrpt::config::CConfigFileBase &, const std::string &>(), pybind11::arg("cfg"), pybind11::arg("sSection") );

		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::nav::CPTG_DiffDrive_C::GetRuntimeClassIdStatic, "C++: mrpt::nav::CPTG_DiffDrive_C::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::nav::CPTG_DiffDrive_C::*)() const) &mrpt::nav::CPTG_DiffDrive_C::GetRuntimeClass, "C++: mrpt::nav::CPTG_DiffDrive_C::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::nav::CPTG_DiffDrive_C::*)() const) &mrpt::nav::CPTG_DiffDrive_C::clone, "C++: mrpt::nav::CPTG_DiffDrive_C::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::nav::CPTG_DiffDrive_C::CreateObject, "C++: mrpt::nav::CPTG_DiffDrive_C::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("loadFromConfigFile", (void (mrpt::nav::CPTG_DiffDrive_C::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::nav::CPTG_DiffDrive_C::loadFromConfigFile, "C++: mrpt::nav::CPTG_DiffDrive_C::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("cfg"), pybind11::arg("sSection"));
		cl.def("saveToConfigFile", (void (mrpt::nav::CPTG_DiffDrive_C::*)(class mrpt::config::CConfigFileBase &, const std::string &) const) &mrpt::nav::CPTG_DiffDrive_C::saveToConfigFile, "C++: mrpt::nav::CPTG_DiffDrive_C::saveToConfigFile(class mrpt::config::CConfigFileBase &, const std::string &) const --> void", pybind11::arg("cfg"), pybind11::arg("sSection"));
		cl.def("getDescription", (std::string (mrpt::nav::CPTG_DiffDrive_C::*)() const) &mrpt::nav::CPTG_DiffDrive_C::getDescription, "C++: mrpt::nav::CPTG_DiffDrive_C::getDescription() const --> std::string");
		cl.def("inverseMap_WS2TP", [](mrpt::nav::CPTG_DiffDrive_C const &o, double const & a0, double const & a1, int & a2, double & a3) -> bool { return o.inverseMap_WS2TP(a0, a1, a2, a3); }, "", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("out_k"), pybind11::arg("out_d"));
		cl.def("inverseMap_WS2TP", (bool (mrpt::nav::CPTG_DiffDrive_C::*)(double, double, int &, double &, double) const) &mrpt::nav::CPTG_DiffDrive_C::inverseMap_WS2TP, "C++: mrpt::nav::CPTG_DiffDrive_C::inverseMap_WS2TP(double, double, int &, double &, double) const --> bool", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("out_k"), pybind11::arg("out_d"), pybind11::arg("tolerance_dist"));
		cl.def("PTG_IsIntoDomain", (bool (mrpt::nav::CPTG_DiffDrive_C::*)(double, double) const) &mrpt::nav::CPTG_DiffDrive_C::PTG_IsIntoDomain, "C++: mrpt::nav::CPTG_DiffDrive_C::PTG_IsIntoDomain(double, double) const --> bool", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("ptgDiffDriveSteeringFunction", (void (mrpt::nav::CPTG_DiffDrive_C::*)(float, float, float, float, float, float &, float &) const) &mrpt::nav::CPTG_DiffDrive_C::ptgDiffDriveSteeringFunction, "C++: mrpt::nav::CPTG_DiffDrive_C::ptgDiffDriveSteeringFunction(float, float, float, float, float, float &, float &) const --> void", pybind11::arg("alpha"), pybind11::arg("t"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("phi"), pybind11::arg("v"), pybind11::arg("w"));
		cl.def("loadDefaultParams", (void (mrpt::nav::CPTG_DiffDrive_C::*)()) &mrpt::nav::CPTG_DiffDrive_C::loadDefaultParams, "C++: mrpt::nav::CPTG_DiffDrive_C::loadDefaultParams() --> void");
		cl.def("assign", (class mrpt::nav::CPTG_DiffDrive_C & (mrpt::nav::CPTG_DiffDrive_C::*)(const class mrpt::nav::CPTG_DiffDrive_C &)) &mrpt::nav::CPTG_DiffDrive_C::operator=, "C++: mrpt::nav::CPTG_DiffDrive_C::operator=(const class mrpt::nav::CPTG_DiffDrive_C &) --> class mrpt::nav::CPTG_DiffDrive_C &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::nav::CPTG_DiffDrive_CC file:mrpt/nav/tpspace/CPTG_DiffDrive_CC.h line:25
		pybind11::class_<mrpt::nav::CPTG_DiffDrive_CC, std::shared_ptr<mrpt::nav::CPTG_DiffDrive_CC>, PyCallBack_mrpt_nav_CPTG_DiffDrive_CC, mrpt::nav::CPTG_DiffDrive_CollisionGridBased> cl(M("mrpt::nav"), "CPTG_DiffDrive_CC", "A PTG for optimal paths of type \"C|C\" , as named in PTG papers.\n - **Compatible kinematics**: differential-driven / Ackermann steering\n - **Compatible robot shape**: Arbitrary 2D polygon\n - **PTG parameters**: Use the app `ptg-configurator`\n\n  See also \"Obstacle Distance for Car-Like Robots\", IEEE Trans. Rob. And\n Autom, 1999.\n \n\n [Before MRPT 1.5.0 this was named CPTG4]\n  \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::nav::CPTG_DiffDrive_CC(); }, [](){ return new PyCallBack_mrpt_nav_CPTG_DiffDrive_CC(); } ) );
		cl.def( pybind11::init<const class mrpt::config::CConfigFileBase &, const std::string &>(), pybind11::arg("cfg"), pybind11::arg("sSection") );

		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::nav::CPTG_DiffDrive_CC::GetRuntimeClassIdStatic, "C++: mrpt::nav::CPTG_DiffDrive_CC::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::nav::CPTG_DiffDrive_CC::*)() const) &mrpt::nav::CPTG_DiffDrive_CC::GetRuntimeClass, "C++: mrpt::nav::CPTG_DiffDrive_CC::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::nav::CPTG_DiffDrive_CC::*)() const) &mrpt::nav::CPTG_DiffDrive_CC::clone, "C++: mrpt::nav::CPTG_DiffDrive_CC::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::nav::CPTG_DiffDrive_CC::CreateObject, "C++: mrpt::nav::CPTG_DiffDrive_CC::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("loadFromConfigFile", (void (mrpt::nav::CPTG_DiffDrive_CC::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::nav::CPTG_DiffDrive_CC::loadFromConfigFile, "C++: mrpt::nav::CPTG_DiffDrive_CC::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("cfg"), pybind11::arg("sSection"));
		cl.def("saveToConfigFile", (void (mrpt::nav::CPTG_DiffDrive_CC::*)(class mrpt::config::CConfigFileBase &, const std::string &) const) &mrpt::nav::CPTG_DiffDrive_CC::saveToConfigFile, "C++: mrpt::nav::CPTG_DiffDrive_CC::saveToConfigFile(class mrpt::config::CConfigFileBase &, const std::string &) const --> void", pybind11::arg("cfg"), pybind11::arg("sSection"));
		cl.def("getDescription", (std::string (mrpt::nav::CPTG_DiffDrive_CC::*)() const) &mrpt::nav::CPTG_DiffDrive_CC::getDescription, "C++: mrpt::nav::CPTG_DiffDrive_CC::getDescription() const --> std::string");
		cl.def("PTG_IsIntoDomain", (bool (mrpt::nav::CPTG_DiffDrive_CC::*)(double, double) const) &mrpt::nav::CPTG_DiffDrive_CC::PTG_IsIntoDomain, "C++: mrpt::nav::CPTG_DiffDrive_CC::PTG_IsIntoDomain(double, double) const --> bool", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("ptgDiffDriveSteeringFunction", (void (mrpt::nav::CPTG_DiffDrive_CC::*)(float, float, float, float, float, float &, float &) const) &mrpt::nav::CPTG_DiffDrive_CC::ptgDiffDriveSteeringFunction, "C++: mrpt::nav::CPTG_DiffDrive_CC::ptgDiffDriveSteeringFunction(float, float, float, float, float, float &, float &) const --> void", pybind11::arg("alpha"), pybind11::arg("t"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("phi"), pybind11::arg("v"), pybind11::arg("w"));
		cl.def("loadDefaultParams", (void (mrpt::nav::CPTG_DiffDrive_CC::*)()) &mrpt::nav::CPTG_DiffDrive_CC::loadDefaultParams, "C++: mrpt::nav::CPTG_DiffDrive_CC::loadDefaultParams() --> void");
		cl.def("assign", (class mrpt::nav::CPTG_DiffDrive_CC & (mrpt::nav::CPTG_DiffDrive_CC::*)(const class mrpt::nav::CPTG_DiffDrive_CC &)) &mrpt::nav::CPTG_DiffDrive_CC::operator=, "C++: mrpt::nav::CPTG_DiffDrive_CC::operator=(const class mrpt::nav::CPTG_DiffDrive_CC &) --> class mrpt::nav::CPTG_DiffDrive_CC &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::nav::CPTG_DiffDrive_CCS file:mrpt/nav/tpspace/CPTG_DiffDrive_CCS.h line:25
		pybind11::class_<mrpt::nav::CPTG_DiffDrive_CCS, std::shared_ptr<mrpt::nav::CPTG_DiffDrive_CCS>, PyCallBack_mrpt_nav_CPTG_DiffDrive_CCS, mrpt::nav::CPTG_DiffDrive_CollisionGridBased> cl(M("mrpt::nav"), "CPTG_DiffDrive_CCS", "A PTG for optimal paths of type \"C|C,S\" (as named in PTG papers).\n - **Compatible kinematics**: differential-driven / Ackermann steering\n - **Compatible robot shape**: Arbitrary 2D polygon\n - **PTG parameters**: Use the app `ptg-configurator`\n\n  See also \"Obstacle Distance for Car-Like Robots\", IEEE Trans. Rob. And\n Autom, 1999.\n \n\n [Before MRPT 1.5.0 this was named CPTG3]\n  \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::nav::CPTG_DiffDrive_CCS(); }, [](){ return new PyCallBack_mrpt_nav_CPTG_DiffDrive_CCS(); } ) );
		cl.def( pybind11::init<const class mrpt::config::CConfigFileBase &, const std::string &>(), pybind11::arg("cfg"), pybind11::arg("sSection") );

		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::nav::CPTG_DiffDrive_CCS::GetRuntimeClassIdStatic, "C++: mrpt::nav::CPTG_DiffDrive_CCS::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::nav::CPTG_DiffDrive_CCS::*)() const) &mrpt::nav::CPTG_DiffDrive_CCS::GetRuntimeClass, "C++: mrpt::nav::CPTG_DiffDrive_CCS::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::nav::CPTG_DiffDrive_CCS::*)() const) &mrpt::nav::CPTG_DiffDrive_CCS::clone, "C++: mrpt::nav::CPTG_DiffDrive_CCS::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::nav::CPTG_DiffDrive_CCS::CreateObject, "C++: mrpt::nav::CPTG_DiffDrive_CCS::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("loadFromConfigFile", (void (mrpt::nav::CPTG_DiffDrive_CCS::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::nav::CPTG_DiffDrive_CCS::loadFromConfigFile, "C++: mrpt::nav::CPTG_DiffDrive_CCS::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("cfg"), pybind11::arg("sSection"));
		cl.def("saveToConfigFile", (void (mrpt::nav::CPTG_DiffDrive_CCS::*)(class mrpt::config::CConfigFileBase &, const std::string &) const) &mrpt::nav::CPTG_DiffDrive_CCS::saveToConfigFile, "C++: mrpt::nav::CPTG_DiffDrive_CCS::saveToConfigFile(class mrpt::config::CConfigFileBase &, const std::string &) const --> void", pybind11::arg("cfg"), pybind11::arg("sSection"));
		cl.def("getDescription", (std::string (mrpt::nav::CPTG_DiffDrive_CCS::*)() const) &mrpt::nav::CPTG_DiffDrive_CCS::getDescription, "C++: mrpt::nav::CPTG_DiffDrive_CCS::getDescription() const --> std::string");
		cl.def("PTG_IsIntoDomain", (bool (mrpt::nav::CPTG_DiffDrive_CCS::*)(double, double) const) &mrpt::nav::CPTG_DiffDrive_CCS::PTG_IsIntoDomain, "C++: mrpt::nav::CPTG_DiffDrive_CCS::PTG_IsIntoDomain(double, double) const --> bool", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("ptgDiffDriveSteeringFunction", (void (mrpt::nav::CPTG_DiffDrive_CCS::*)(float, float, float, float, float, float &, float &) const) &mrpt::nav::CPTG_DiffDrive_CCS::ptgDiffDriveSteeringFunction, "C++: mrpt::nav::CPTG_DiffDrive_CCS::ptgDiffDriveSteeringFunction(float, float, float, float, float, float &, float &) const --> void", pybind11::arg("alpha"), pybind11::arg("t"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("phi"), pybind11::arg("v"), pybind11::arg("w"));
		cl.def("loadDefaultParams", (void (mrpt::nav::CPTG_DiffDrive_CCS::*)()) &mrpt::nav::CPTG_DiffDrive_CCS::loadDefaultParams, "C++: mrpt::nav::CPTG_DiffDrive_CCS::loadDefaultParams() --> void");
		cl.def("assign", (class mrpt::nav::CPTG_DiffDrive_CCS & (mrpt::nav::CPTG_DiffDrive_CCS::*)(const class mrpt::nav::CPTG_DiffDrive_CCS &)) &mrpt::nav::CPTG_DiffDrive_CCS::operator=, "C++: mrpt::nav::CPTG_DiffDrive_CCS::operator=(const class mrpt::nav::CPTG_DiffDrive_CCS &) --> class mrpt::nav::CPTG_DiffDrive_CCS &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::nav::CPTG_DiffDrive_CS file:mrpt/nav/tpspace/CPTG_DiffDrive_CS.h line:25
		pybind11::class_<mrpt::nav::CPTG_DiffDrive_CS, std::shared_ptr<mrpt::nav::CPTG_DiffDrive_CS>, PyCallBack_mrpt_nav_CPTG_DiffDrive_CS, mrpt::nav::CPTG_DiffDrive_CollisionGridBased> cl(M("mrpt::nav"), "CPTG_DiffDrive_CS", "A PTG for optimal paths of type \"CS\", as named in PTG papers.\n - **Compatible kinematics**: differential-driven / Ackermann steering\n - **Compatible robot shape**: Arbitrary 2D polygon\n - **PTG parameters**: Use the app `ptg-configurator`\n\n  See \"Obstacle Distance for Car-Like Robots\", IEEE Trans. Rob. And Autom,\n 1999.\n \n\n [Before MRPT 1.5.0 this was named CPTG5]\n  \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::nav::CPTG_DiffDrive_CS(); }, [](){ return new PyCallBack_mrpt_nav_CPTG_DiffDrive_CS(); } ) );
		cl.def( pybind11::init<const class mrpt::config::CConfigFileBase &, const std::string &>(), pybind11::arg("cfg"), pybind11::arg("sSection") );

		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::nav::CPTG_DiffDrive_CS::GetRuntimeClassIdStatic, "C++: mrpt::nav::CPTG_DiffDrive_CS::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::nav::CPTG_DiffDrive_CS::*)() const) &mrpt::nav::CPTG_DiffDrive_CS::GetRuntimeClass, "C++: mrpt::nav::CPTG_DiffDrive_CS::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::nav::CPTG_DiffDrive_CS::*)() const) &mrpt::nav::CPTG_DiffDrive_CS::clone, "C++: mrpt::nav::CPTG_DiffDrive_CS::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::nav::CPTG_DiffDrive_CS::CreateObject, "C++: mrpt::nav::CPTG_DiffDrive_CS::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("loadFromConfigFile", (void (mrpt::nav::CPTG_DiffDrive_CS::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::nav::CPTG_DiffDrive_CS::loadFromConfigFile, "C++: mrpt::nav::CPTG_DiffDrive_CS::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("cfg"), pybind11::arg("sSection"));
		cl.def("saveToConfigFile", (void (mrpt::nav::CPTG_DiffDrive_CS::*)(class mrpt::config::CConfigFileBase &, const std::string &) const) &mrpt::nav::CPTG_DiffDrive_CS::saveToConfigFile, "C++: mrpt::nav::CPTG_DiffDrive_CS::saveToConfigFile(class mrpt::config::CConfigFileBase &, const std::string &) const --> void", pybind11::arg("cfg"), pybind11::arg("sSection"));
		cl.def("getDescription", (std::string (mrpt::nav::CPTG_DiffDrive_CS::*)() const) &mrpt::nav::CPTG_DiffDrive_CS::getDescription, "C++: mrpt::nav::CPTG_DiffDrive_CS::getDescription() const --> std::string");
		cl.def("PTG_IsIntoDomain", (bool (mrpt::nav::CPTG_DiffDrive_CS::*)(double, double) const) &mrpt::nav::CPTG_DiffDrive_CS::PTG_IsIntoDomain, "C++: mrpt::nav::CPTG_DiffDrive_CS::PTG_IsIntoDomain(double, double) const --> bool", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("ptgDiffDriveSteeringFunction", (void (mrpt::nav::CPTG_DiffDrive_CS::*)(float, float, float, float, float, float &, float &) const) &mrpt::nav::CPTG_DiffDrive_CS::ptgDiffDriveSteeringFunction, "C++: mrpt::nav::CPTG_DiffDrive_CS::ptgDiffDriveSteeringFunction(float, float, float, float, float, float &, float &) const --> void", pybind11::arg("alpha"), pybind11::arg("t"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("phi"), pybind11::arg("v"), pybind11::arg("w"));
		cl.def("loadDefaultParams", (void (mrpt::nav::CPTG_DiffDrive_CS::*)()) &mrpt::nav::CPTG_DiffDrive_CS::loadDefaultParams, "C++: mrpt::nav::CPTG_DiffDrive_CS::loadDefaultParams() --> void");
		cl.def("assign", (class mrpt::nav::CPTG_DiffDrive_CS & (mrpt::nav::CPTG_DiffDrive_CS::*)(const class mrpt::nav::CPTG_DiffDrive_CS &)) &mrpt::nav::CPTG_DiffDrive_CS::operator=, "C++: mrpt::nav::CPTG_DiffDrive_CS::operator=(const class mrpt::nav::CPTG_DiffDrive_CS &) --> class mrpt::nav::CPTG_DiffDrive_CS &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
