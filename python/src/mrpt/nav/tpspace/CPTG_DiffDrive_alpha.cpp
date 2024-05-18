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
#include <mrpt/nav/tpspace/CPTG_DiffDrive_CollisionGridBased.h>
#include <mrpt/nav/tpspace/CPTG_DiffDrive_alpha.h>
#include <mrpt/nav/tpspace/CPTG_Holo_Blend.h>
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

// mrpt::nav::CPTG_DiffDrive_alpha file:mrpt/nav/tpspace/CPTG_DiffDrive_alpha.h line:35
struct PyCallBack_mrpt_nav_CPTG_DiffDrive_alpha : public mrpt::nav::CPTG_DiffDrive_alpha {
	using mrpt::nav::CPTG_DiffDrive_alpha::CPTG_DiffDrive_alpha;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPTG_DiffDrive_alpha::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CPTG_DiffDrive_alpha::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CPTG_DiffDrive_alpha::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_alpha::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_alpha::serializeFrom(a0, a1);
	}
	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_alpha::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_alpha::saveToConfigFile(a0, a1);
	}
	std::string getDescription() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "getDescription");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CPTG_DiffDrive_alpha::getDescription();
	}
	void ptgDiffDriveSteeringFunction(float a0, float a1, float a2, float a3, float a4, float & a5, float & a6) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "ptgDiffDriveSteeringFunction");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4, a5, a6);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_alpha::ptgDiffDriveSteeringFunction(a0, a1, a2, a3, a4, a5, a6);
	}
	void loadDefaultParams() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "loadDefaultParams");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_DiffDrive_alpha::loadDefaultParams();
	}
	bool inverseMap_WS2TP(double a0, double a1, int & a2, double & a3, double a4) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "inverseMap_WS2TP");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "directionToMotionCommand");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "getSupportedKinematicVelocityCommand");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "setRefDistance");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "getPathStepCount");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "getPathPose");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "getPathDist");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "getPathStepForDist");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "getPathStepDuration");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "getMaxLinVel");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "getMaxAngVel");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "updateTPObstacleSingle");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "onNewNavDynamicState");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "internal_processNewRobotShape");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "internal_initialize");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "internal_deinitialize");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "internal_readFromStream");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "internal_writeToStream");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "getPathTwist");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "getMaxRobotRadius");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "evalClearanceToRobotShape");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "isPointInsideRobotShape");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "add_robotShape_to_setOfLines");
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
	bool PTG_IsIntoDomain(double a0, double a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "PTG_IsIntoDomain");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CParameterizedTrajectoryGenerator::PTG_IsIntoDomain(a0, a1);
	}
	bool isBijectiveAt(uint16_t a0, uint32_t a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "isBijectiveAt");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "supportVelCmdNOP");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "supportSpeedAtTarget");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "maxTimeInVelCmdNOP");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "getActualUnloopedPathLength");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "evalPathRelativePriority");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "renderPathAsSimpleLine");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_DiffDrive_alpha *>(this), "evalClearanceSingleObstacle");
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

// mrpt::nav::CPTG_Holo_Blend file:mrpt/nav/tpspace/CPTG_Holo_Blend.h line:25
struct PyCallBack_mrpt_nav_CPTG_Holo_Blend : public mrpt::nav::CPTG_Holo_Blend {
	using mrpt::nav::CPTG_Holo_Blend::CPTG_Holo_Blend;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPTG_Holo_Blend::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CPTG_Holo_Blend::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CPTG_Holo_Blend::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_Holo_Blend::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_Holo_Blend::serializeFrom(a0, a1);
	}
	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_Holo_Blend::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_Holo_Blend::saveToConfigFile(a0, a1);
	}
	void loadDefaultParams() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "loadDefaultParams");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_Holo_Blend::loadDefaultParams();
	}
	bool supportVelCmdNOP() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "supportVelCmdNOP");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPTG_Holo_Blend::supportVelCmdNOP();
	}
	double maxTimeInVelCmdNOP(int a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "maxTimeInVelCmdNOP");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CPTG_Holo_Blend::maxTimeInVelCmdNOP(a0);
	}
	std::string getDescription() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "getDescription");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CPTG_Holo_Blend::getDescription();
	}
	bool inverseMap_WS2TP(double a0, double a1, int & a2, double & a3, double a4) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "inverseMap_WS2TP");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPTG_Holo_Blend::inverseMap_WS2TP(a0, a1, a2, a3, a4);
	}
	bool PTG_IsIntoDomain(double a0, double a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "PTG_IsIntoDomain");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPTG_Holo_Blend::PTG_IsIntoDomain(a0, a1);
	}
	void onNewNavDynamicState() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "onNewNavDynamicState");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_Holo_Blend::onNewNavDynamicState();
	}
	class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> directionToMotionCommand(uint16_t a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "directionToMotionCommand");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>::value) {
				static pybind11::detail::override_caster_t<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>> caster;
				return pybind11::detail::cast_ref<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o));
		}
		return CPTG_Holo_Blend::directionToMotionCommand(a0);
	}
	class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> getSupportedKinematicVelocityCommand() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "getSupportedKinematicVelocityCommand");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>::value) {
				static pybind11::detail::override_caster_t<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>> caster;
				return pybind11::detail::cast_ref<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o));
		}
		return CPTG_Holo_Blend::getSupportedKinematicVelocityCommand();
	}
	size_t getPathStepCount(uint16_t a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "getPathStepCount");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CPTG_Holo_Blend::getPathStepCount(a0);
	}
	struct mrpt::math::TPose2D getPathPose(uint16_t a0, uint32_t a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "getPathPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TPose2D>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TPose2D> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TPose2D>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TPose2D>(std::move(o));
		}
		return CPTG_Holo_Blend::getPathPose(a0, a1);
	}
	double getPathDist(uint16_t a0, uint32_t a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "getPathDist");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CPTG_Holo_Blend::getPathDist(a0, a1);
	}
	bool getPathStepForDist(uint16_t a0, double a1, unsigned int & a2) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "getPathStepForDist");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPTG_Holo_Blend::getPathStepForDist(a0, a1, a2);
	}
	double getPathStepDuration() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "getPathStepDuration");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CPTG_Holo_Blend::getPathStepDuration();
	}
	double getMaxLinVel() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "getMaxLinVel");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CPTG_Holo_Blend::getMaxLinVel();
	}
	double getMaxAngVel() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "getMaxAngVel");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CPTG_Holo_Blend::getMaxAngVel();
	}
	void updateTPObstacleSingle(double a0, double a1, uint16_t a2, double & a3) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "updateTPObstacleSingle");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_Holo_Blend::updateTPObstacleSingle(a0, a1, a2, a3);
	}
	void internal_processNewRobotShape() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "internal_processNewRobotShape");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_Holo_Blend::internal_processNewRobotShape();
	}
	void internal_initialize(const std::string & a0, const bool a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "internal_initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_Holo_Blend::internal_initialize(a0, a1);
	}
	void internal_deinitialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "internal_deinitialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_Holo_Blend::internal_deinitialize();
	}
	double getMaxRobotRadius() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "getMaxRobotRadius");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CPTG_RobotShape_Circular::getMaxRobotRadius();
	}
	double evalClearanceToRobotShape(const double a0, const double a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "evalClearanceToRobotShape");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CPTG_RobotShape_Circular::evalClearanceToRobotShape(a0, a1);
	}
	void add_robotShape_to_setOfLines(class mrpt::opengl::CSetOfLines & a0, const class mrpt::poses::CPose2D & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "add_robotShape_to_setOfLines");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPTG_RobotShape_Circular::add_robotShape_to_setOfLines(a0, a1);
	}
	bool isPointInsideRobotShape(const double a0, const double a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "isPointInsideRobotShape");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPTG_RobotShape_Circular::isPointInsideRobotShape(a0, a1);
	}
	bool isBijectiveAt(uint16_t a0, uint32_t a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "isBijectiveAt");
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
	void setRefDistance(const double a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "setRefDistance");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CParameterizedTrajectoryGenerator::setRefDistance(a0);
	}
	struct mrpt::math::TTwist2D getPathTwist(uint16_t a0, uint32_t a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "getPathTwist");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TTwist2D>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TTwist2D> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TTwist2D>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TTwist2D>(std::move(o));
		}
		return CParameterizedTrajectoryGenerator::getPathTwist(a0, a1);
	}
	bool supportSpeedAtTarget() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "supportSpeedAtTarget");
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
	double getActualUnloopedPathLength(uint16_t a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "getActualUnloopedPathLength");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "evalPathRelativePriority");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "renderPathAsSimpleLine");
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
	void internal_readFromStream(class mrpt::serialization::CArchive & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "internal_readFromStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CParameterizedTrajectoryGenerator::internal_readFromStream(a0);
	}
	void internal_writeToStream(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "internal_writeToStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CParameterizedTrajectoryGenerator::internal_writeToStream(a0);
	}
	void evalClearanceSingleObstacle(const double a0, const double a1, const unsigned short a2, class std::map<double, double> & a3, bool a4) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CPTG_Holo_Blend *>(this), "evalClearanceSingleObstacle");
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

void bind_mrpt_nav_tpspace_CPTG_DiffDrive_alpha(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::nav::CPTG_DiffDrive_alpha file:mrpt/nav/tpspace/CPTG_DiffDrive_alpha.h line:35
		pybind11::class_<mrpt::nav::CPTG_DiffDrive_alpha, std::shared_ptr<mrpt::nav::CPTG_DiffDrive_alpha>, PyCallBack_mrpt_nav_CPTG_DiffDrive_alpha, mrpt::nav::CPTG_DiffDrive_CollisionGridBased> cl(M("mrpt::nav"), "CPTG_DiffDrive_alpha", "The \"a(symptotic)-alpha PTG\", as named in PTG papers.\n - **Compatible kinematics**: differential-driven / Ackermann steering\n - **Compatible robot shape**: Arbitrary 2D polygon\n - **PTG parameters**: Use the app `ptg-configurator`\n\n This PT generator functions are:\n\n \n\n \n\n\n So, the radius of curvature of each trajectory is NOT constant for each\n \"alpha\" value in this PTG:\n\n  ![C-PTG path examples](PTG2_paths.png)\n\n \n [Before MRPT 1.5.0 this was named CPTG2]\n  \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::nav::CPTG_DiffDrive_alpha(); }, [](){ return new PyCallBack_mrpt_nav_CPTG_DiffDrive_alpha(); } ) );
		cl.def( pybind11::init<const class mrpt::config::CConfigFileBase &, const std::string &>(), pybind11::arg("cfg"), pybind11::arg("sSection") );

		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::nav::CPTG_DiffDrive_alpha::GetRuntimeClassIdStatic, "C++: mrpt::nav::CPTG_DiffDrive_alpha::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::nav::CPTG_DiffDrive_alpha::*)() const) &mrpt::nav::CPTG_DiffDrive_alpha::GetRuntimeClass, "C++: mrpt::nav::CPTG_DiffDrive_alpha::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::nav::CPTG_DiffDrive_alpha::*)() const) &mrpt::nav::CPTG_DiffDrive_alpha::clone, "C++: mrpt::nav::CPTG_DiffDrive_alpha::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::nav::CPTG_DiffDrive_alpha::CreateObject, "C++: mrpt::nav::CPTG_DiffDrive_alpha::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("loadFromConfigFile", (void (mrpt::nav::CPTG_DiffDrive_alpha::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::nav::CPTG_DiffDrive_alpha::loadFromConfigFile, "C++: mrpt::nav::CPTG_DiffDrive_alpha::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("cfg"), pybind11::arg("sSection"));
		cl.def("saveToConfigFile", (void (mrpt::nav::CPTG_DiffDrive_alpha::*)(class mrpt::config::CConfigFileBase &, const std::string &) const) &mrpt::nav::CPTG_DiffDrive_alpha::saveToConfigFile, "C++: mrpt::nav::CPTG_DiffDrive_alpha::saveToConfigFile(class mrpt::config::CConfigFileBase &, const std::string &) const --> void", pybind11::arg("cfg"), pybind11::arg("sSection"));
		cl.def("getDescription", (std::string (mrpt::nav::CPTG_DiffDrive_alpha::*)() const) &mrpt::nav::CPTG_DiffDrive_alpha::getDescription, "C++: mrpt::nav::CPTG_DiffDrive_alpha::getDescription() const --> std::string");
		cl.def("ptgDiffDriveSteeringFunction", (void (mrpt::nav::CPTG_DiffDrive_alpha::*)(float, float, float, float, float, float &, float &) const) &mrpt::nav::CPTG_DiffDrive_alpha::ptgDiffDriveSteeringFunction, "C++: mrpt::nav::CPTG_DiffDrive_alpha::ptgDiffDriveSteeringFunction(float, float, float, float, float, float &, float &) const --> void", pybind11::arg("alpha"), pybind11::arg("t"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("phi"), pybind11::arg("v"), pybind11::arg("w"));
		cl.def("loadDefaultParams", (void (mrpt::nav::CPTG_DiffDrive_alpha::*)()) &mrpt::nav::CPTG_DiffDrive_alpha::loadDefaultParams, "C++: mrpt::nav::CPTG_DiffDrive_alpha::loadDefaultParams() --> void");
		cl.def("assign", (class mrpt::nav::CPTG_DiffDrive_alpha & (mrpt::nav::CPTG_DiffDrive_alpha::*)(const class mrpt::nav::CPTG_DiffDrive_alpha &)) &mrpt::nav::CPTG_DiffDrive_alpha::operator=, "C++: mrpt::nav::CPTG_DiffDrive_alpha::operator=(const class mrpt::nav::CPTG_DiffDrive_alpha &) --> class mrpt::nav::CPTG_DiffDrive_alpha &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::nav::CPTG_Holo_Blend file:mrpt/nav/tpspace/CPTG_Holo_Blend.h line:25
		pybind11::class_<mrpt::nav::CPTG_Holo_Blend, std::shared_ptr<mrpt::nav::CPTG_Holo_Blend>, PyCallBack_mrpt_nav_CPTG_Holo_Blend, mrpt::nav::CPTG_RobotShape_Circular> cl(M("mrpt::nav"), "CPTG_Holo_Blend", "A PTG for circular-shaped robots with holonomic kinematics.\n - **Compatible kinematics**: Holonomic robot capable of velocity commands\n with a linear interpolation (\"ramp \"or \"blending\") time. See\n mrpt::kinematics::CVehicleSimul_Holo\n - **Compatible robot shape**: Circular robots\n - **PTG parameters**: Use the app `ptg-configurator`\n\n  \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::nav::CPTG_Holo_Blend(); }, [](){ return new PyCallBack_mrpt_nav_CPTG_Holo_Blend(); } ) );
		cl.def( pybind11::init<const class mrpt::config::CConfigFileBase &, const std::string &>(), pybind11::arg("cfg"), pybind11::arg("sSection") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_nav_CPTG_Holo_Blend const &o){ return new PyCallBack_mrpt_nav_CPTG_Holo_Blend(o); } ) );
		cl.def( pybind11::init( [](mrpt::nav::CPTG_Holo_Blend const &o){ return new mrpt::nav::CPTG_Holo_Blend(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::nav::CPTG_Holo_Blend::GetRuntimeClassIdStatic, "C++: mrpt::nav::CPTG_Holo_Blend::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::nav::CPTG_Holo_Blend::*)() const) &mrpt::nav::CPTG_Holo_Blend::GetRuntimeClass, "C++: mrpt::nav::CPTG_Holo_Blend::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::nav::CPTG_Holo_Blend::*)() const) &mrpt::nav::CPTG_Holo_Blend::clone, "C++: mrpt::nav::CPTG_Holo_Blend::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::nav::CPTG_Holo_Blend::CreateObject, "C++: mrpt::nav::CPTG_Holo_Blend::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("loadFromConfigFile", (void (mrpt::nav::CPTG_Holo_Blend::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::nav::CPTG_Holo_Blend::loadFromConfigFile, "C++: mrpt::nav::CPTG_Holo_Blend::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("cfg"), pybind11::arg("sSection"));
		cl.def("saveToConfigFile", (void (mrpt::nav::CPTG_Holo_Blend::*)(class mrpt::config::CConfigFileBase &, const std::string &) const) &mrpt::nav::CPTG_Holo_Blend::saveToConfigFile, "C++: mrpt::nav::CPTG_Holo_Blend::saveToConfigFile(class mrpt::config::CConfigFileBase &, const std::string &) const --> void", pybind11::arg("cfg"), pybind11::arg("sSection"));
		cl.def("loadDefaultParams", (void (mrpt::nav::CPTG_Holo_Blend::*)()) &mrpt::nav::CPTG_Holo_Blend::loadDefaultParams, "C++: mrpt::nav::CPTG_Holo_Blend::loadDefaultParams() --> void");
		cl.def("supportVelCmdNOP", (bool (mrpt::nav::CPTG_Holo_Blend::*)() const) &mrpt::nav::CPTG_Holo_Blend::supportVelCmdNOP, "C++: mrpt::nav::CPTG_Holo_Blend::supportVelCmdNOP() const --> bool");
		cl.def("maxTimeInVelCmdNOP", (double (mrpt::nav::CPTG_Holo_Blend::*)(int) const) &mrpt::nav::CPTG_Holo_Blend::maxTimeInVelCmdNOP, "C++: mrpt::nav::CPTG_Holo_Blend::maxTimeInVelCmdNOP(int) const --> double", pybind11::arg("path_k"));
		cl.def("getDescription", (std::string (mrpt::nav::CPTG_Holo_Blend::*)() const) &mrpt::nav::CPTG_Holo_Blend::getDescription, "C++: mrpt::nav::CPTG_Holo_Blend::getDescription() const --> std::string");
		cl.def("inverseMap_WS2TP", [](mrpt::nav::CPTG_Holo_Blend const &o, double const & a0, double const & a1, int & a2, double & a3) -> bool { return o.inverseMap_WS2TP(a0, a1, a2, a3); }, "", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("out_k"), pybind11::arg("out_d"));
		cl.def("inverseMap_WS2TP", (bool (mrpt::nav::CPTG_Holo_Blend::*)(double, double, int &, double &, double) const) &mrpt::nav::CPTG_Holo_Blend::inverseMap_WS2TP, "C++: mrpt::nav::CPTG_Holo_Blend::inverseMap_WS2TP(double, double, int &, double &, double) const --> bool", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("out_k"), pybind11::arg("out_d"), pybind11::arg("tolerance_dist"));
		cl.def("PTG_IsIntoDomain", (bool (mrpt::nav::CPTG_Holo_Blend::*)(double, double) const) &mrpt::nav::CPTG_Holo_Blend::PTG_IsIntoDomain, "C++: mrpt::nav::CPTG_Holo_Blend::PTG_IsIntoDomain(double, double) const --> bool", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("onNewNavDynamicState", (void (mrpt::nav::CPTG_Holo_Blend::*)()) &mrpt::nav::CPTG_Holo_Blend::onNewNavDynamicState, "C++: mrpt::nav::CPTG_Holo_Blend::onNewNavDynamicState() --> void");
		cl.def("directionToMotionCommand", (class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> (mrpt::nav::CPTG_Holo_Blend::*)(uint16_t) const) &mrpt::nav::CPTG_Holo_Blend::directionToMotionCommand, "Converts a discretized \"alpha\" value into a feasible motion command or\n action. See derived classes for the meaning of these actions \n\nC++: mrpt::nav::CPTG_Holo_Blend::directionToMotionCommand(uint16_t) const --> class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>", pybind11::arg("k"));
		cl.def("getSupportedKinematicVelocityCommand", (class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> (mrpt::nav::CPTG_Holo_Blend::*)() const) &mrpt::nav::CPTG_Holo_Blend::getSupportedKinematicVelocityCommand, "C++: mrpt::nav::CPTG_Holo_Blend::getSupportedKinematicVelocityCommand() const --> class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>");
		cl.def("getPathStepCount", (size_t (mrpt::nav::CPTG_Holo_Blend::*)(uint16_t) const) &mrpt::nav::CPTG_Holo_Blend::getPathStepCount, "C++: mrpt::nav::CPTG_Holo_Blend::getPathStepCount(uint16_t) const --> size_t", pybind11::arg("k"));
		cl.def("getPathPose", (struct mrpt::math::TPose2D (mrpt::nav::CPTG_Holo_Blend::*)(uint16_t, uint32_t) const) &mrpt::nav::CPTG_Holo_Blend::getPathPose, "C++: mrpt::nav::CPTG_Holo_Blend::getPathPose(uint16_t, uint32_t) const --> struct mrpt::math::TPose2D", pybind11::arg("k"), pybind11::arg("step"));
		cl.def("getPathDist", (double (mrpt::nav::CPTG_Holo_Blend::*)(uint16_t, uint32_t) const) &mrpt::nav::CPTG_Holo_Blend::getPathDist, "C++: mrpt::nav::CPTG_Holo_Blend::getPathDist(uint16_t, uint32_t) const --> double", pybind11::arg("k"), pybind11::arg("step"));
		cl.def("getPathStepForDist", (bool (mrpt::nav::CPTG_Holo_Blend::*)(uint16_t, double, unsigned int &) const) &mrpt::nav::CPTG_Holo_Blend::getPathStepForDist, "C++: mrpt::nav::CPTG_Holo_Blend::getPathStepForDist(uint16_t, double, unsigned int &) const --> bool", pybind11::arg("k"), pybind11::arg("dist"), pybind11::arg("out_step"));
		cl.def("getPathStepDuration", (double (mrpt::nav::CPTG_Holo_Blend::*)() const) &mrpt::nav::CPTG_Holo_Blend::getPathStepDuration, "C++: mrpt::nav::CPTG_Holo_Blend::getPathStepDuration() const --> double");
		cl.def("getMaxLinVel", (double (mrpt::nav::CPTG_Holo_Blend::*)() const) &mrpt::nav::CPTG_Holo_Blend::getMaxLinVel, "C++: mrpt::nav::CPTG_Holo_Blend::getMaxLinVel() const --> double");
		cl.def("getMaxAngVel", (double (mrpt::nav::CPTG_Holo_Blend::*)() const) &mrpt::nav::CPTG_Holo_Blend::getMaxAngVel, "C++: mrpt::nav::CPTG_Holo_Blend::getMaxAngVel() const --> double");
		cl.def("updateTPObstacleSingle", (void (mrpt::nav::CPTG_Holo_Blend::*)(double, double, uint16_t, double &) const) &mrpt::nav::CPTG_Holo_Blend::updateTPObstacleSingle, "C++: mrpt::nav::CPTG_Holo_Blend::updateTPObstacleSingle(double, double, uint16_t, double &) const --> void", pybind11::arg("ox"), pybind11::arg("oy"), pybind11::arg("k"), pybind11::arg("tp_obstacle_k"));
		cl.def_static("calc_trans_distance_t_below_Tramp", (double (*)(double, double, double, double, double)) &mrpt::nav::CPTG_Holo_Blend::calc_trans_distance_t_below_Tramp, "Axiliary function for computing the line-integral distance along the\n trajectory, handling special cases of 1/0: \n\nC++: mrpt::nav::CPTG_Holo_Blend::calc_trans_distance_t_below_Tramp(double, double, double, double, double) --> double", pybind11::arg("k2"), pybind11::arg("k4"), pybind11::arg("vxi"), pybind11::arg("vyi"), pybind11::arg("t"));
		cl.def_static("calc_trans_distance_t_below_Tramp_abc", (double (*)(double, double, double, double)) &mrpt::nav::CPTG_Holo_Blend::calc_trans_distance_t_below_Tramp_abc, "Axiliary function for calc_trans_distance_t_below_Tramp() and others \n\nC++: mrpt::nav::CPTG_Holo_Blend::calc_trans_distance_t_below_Tramp_abc(double, double, double, double) --> double", pybind11::arg("t"), pybind11::arg("a"), pybind11::arg("b"), pybind11::arg("c"));
		cl.def("assign", (class mrpt::nav::CPTG_Holo_Blend & (mrpt::nav::CPTG_Holo_Blend::*)(const class mrpt::nav::CPTG_Holo_Blend &)) &mrpt::nav::CPTG_Holo_Blend::operator=, "C++: mrpt::nav::CPTG_Holo_Blend::operator=(const class mrpt::nav::CPTG_Holo_Blend &) --> class mrpt::nav::CPTG_Holo_Blend &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
