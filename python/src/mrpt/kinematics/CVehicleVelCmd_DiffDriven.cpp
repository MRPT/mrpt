#include <iterator>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/kinematics/CVehicleSimulVirtualBase.h>
#include <mrpt/kinematics/CVehicleSimul_DiffDriven.h>
#include <mrpt/kinematics/CVehicleSimul_Holo.h>
#include <mrpt/kinematics/CVehicleVelCmd.h>
#include <mrpt/kinematics/CVehicleVelCmd_DiffDriven.h>
#include <mrpt/kinematics/CVehicleVelCmd_Holo.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/typemeta/static_string.h>
#include <sstream> // __str__
#include <string>
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

// mrpt::kinematics::CVehicleVelCmd_DiffDriven file:mrpt/kinematics/CVehicleVelCmd_DiffDriven.h line:19
struct PyCallBack_mrpt_kinematics_CVehicleVelCmd_DiffDriven : public mrpt::kinematics::CVehicleVelCmd_DiffDriven {
	using mrpt::kinematics::CVehicleVelCmd_DiffDriven::CVehicleVelCmd_DiffDriven;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd_DiffDriven *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CVehicleVelCmd_DiffDriven::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd_DiffDriven *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CVehicleVelCmd_DiffDriven::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd_DiffDriven *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CVehicleVelCmd_DiffDriven::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd_DiffDriven *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CVehicleVelCmd_DiffDriven::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd_DiffDriven *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CVehicleVelCmd_DiffDriven::serializeFrom(a0, a1);
	}
	size_t getVelCmdLength() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd_DiffDriven *>(this), "getVelCmdLength");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CVehicleVelCmd_DiffDriven::getVelCmdLength();
	}
	std::string getVelCmdDescription(const int a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd_DiffDriven *>(this), "getVelCmdDescription");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CVehicleVelCmd_DiffDriven::getVelCmdDescription(a0);
	}
	double getVelCmdElement(const int a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd_DiffDriven *>(this), "getVelCmdElement");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CVehicleVelCmd_DiffDriven::getVelCmdElement(a0);
	}
	void setVelCmdElement(const int a0, const double a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd_DiffDriven *>(this), "setVelCmdElement");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CVehicleVelCmd_DiffDriven::setVelCmdElement(a0, a1);
	}
	bool isStopCmd() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd_DiffDriven *>(this), "isStopCmd");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CVehicleVelCmd_DiffDriven::isStopCmd();
	}
	void setToStop() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd_DiffDriven *>(this), "setToStop");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CVehicleVelCmd_DiffDriven::setToStop();
	}
	void cmdVel_scale(double a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd_DiffDriven *>(this), "cmdVel_scale");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CVehicleVelCmd_DiffDriven::cmdVel_scale(a0);
	}
	double cmdVel_limits(const class mrpt::kinematics::CVehicleVelCmd & a0, const double a1, const struct mrpt::kinematics::CVehicleVelCmd::TVelCmdParams & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd_DiffDriven *>(this), "cmdVel_limits");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CVehicleVelCmd_DiffDriven::cmdVel_limits(a0, a1, a2);
	}
	std::string asString() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd_DiffDriven *>(this), "asString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CVehicleVelCmd::asString();
	}
};

// mrpt::kinematics::CVehicleSimul_DiffDriven file:mrpt/kinematics/CVehicleSimul_DiffDriven.h line:21
struct PyCallBack_mrpt_kinematics_CVehicleSimul_DiffDriven : public mrpt::kinematics::CVehicleSimul_DiffDriven {
	using mrpt::kinematics::CVehicleSimul_DiffDriven::CVehicleSimul_DiffDriven;

	void sendVelCmd(const class mrpt::kinematics::CVehicleVelCmd & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleSimul_DiffDriven *>(this), "sendVelCmd");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CVehicleSimul_DiffDriven::sendVelCmd(a0);
	}
	class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> getVelCmdType() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleSimul_DiffDriven *>(this), "getVelCmdType");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>::value) {
				static pybind11::detail::override_caster_t<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>> caster;
				return pybind11::detail::cast_ref<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o));
		}
		return CVehicleSimul_DiffDriven::getVelCmdType();
	}
	void internal_simulControlStep(const double a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleSimul_DiffDriven *>(this), "internal_simulControlStep");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CVehicleSimulVirtualBase::internal_simulControlStep\"");
	}
	void internal_clear() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleSimul_DiffDriven *>(this), "internal_clear");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CVehicleSimulVirtualBase::internal_clear\"");
	}
};

// mrpt::kinematics::CVehicleVelCmd_Holo file:mrpt/kinematics/CVehicleVelCmd_Holo.h line:19
struct PyCallBack_mrpt_kinematics_CVehicleVelCmd_Holo : public mrpt::kinematics::CVehicleVelCmd_Holo {
	using mrpt::kinematics::CVehicleVelCmd_Holo::CVehicleVelCmd_Holo;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd_Holo *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CVehicleVelCmd_Holo::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd_Holo *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CVehicleVelCmd_Holo::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd_Holo *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CVehicleVelCmd_Holo::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd_Holo *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CVehicleVelCmd_Holo::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd_Holo *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CVehicleVelCmd_Holo::serializeFrom(a0, a1);
	}
	size_t getVelCmdLength() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd_Holo *>(this), "getVelCmdLength");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CVehicleVelCmd_Holo::getVelCmdLength();
	}
	std::string getVelCmdDescription(const int a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd_Holo *>(this), "getVelCmdDescription");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CVehicleVelCmd_Holo::getVelCmdDescription(a0);
	}
	double getVelCmdElement(const int a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd_Holo *>(this), "getVelCmdElement");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CVehicleVelCmd_Holo::getVelCmdElement(a0);
	}
	void setVelCmdElement(const int a0, const double a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd_Holo *>(this), "setVelCmdElement");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CVehicleVelCmd_Holo::setVelCmdElement(a0, a1);
	}
	bool isStopCmd() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd_Holo *>(this), "isStopCmd");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CVehicleVelCmd_Holo::isStopCmd();
	}
	void setToStop() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd_Holo *>(this), "setToStop");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CVehicleVelCmd_Holo::setToStop();
	}
	void cmdVel_scale(double a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd_Holo *>(this), "cmdVel_scale");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CVehicleVelCmd_Holo::cmdVel_scale(a0);
	}
	double cmdVel_limits(const class mrpt::kinematics::CVehicleVelCmd & a0, const double a1, const struct mrpt::kinematics::CVehicleVelCmd::TVelCmdParams & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd_Holo *>(this), "cmdVel_limits");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CVehicleVelCmd_Holo::cmdVel_limits(a0, a1, a2);
	}
	std::string asString() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd_Holo *>(this), "asString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CVehicleVelCmd::asString();
	}
};

// mrpt::kinematics::CVehicleSimul_Holo file:mrpt/kinematics/CVehicleSimul_Holo.h line:23
struct PyCallBack_mrpt_kinematics_CVehicleSimul_Holo : public mrpt::kinematics::CVehicleSimul_Holo {
	using mrpt::kinematics::CVehicleSimul_Holo::CVehicleSimul_Holo;

	void sendVelCmd(const class mrpt::kinematics::CVehicleVelCmd & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleSimul_Holo *>(this), "sendVelCmd");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CVehicleSimul_Holo::sendVelCmd(a0);
	}
	class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> getVelCmdType() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleSimul_Holo *>(this), "getVelCmdType");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>::value) {
				static pybind11::detail::override_caster_t<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>> caster;
				return pybind11::detail::cast_ref<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o));
		}
		return CVehicleSimul_Holo::getVelCmdType();
	}
	void internal_simulControlStep(const double a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleSimul_Holo *>(this), "internal_simulControlStep");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CVehicleSimulVirtualBase::internal_simulControlStep\"");
	}
	void internal_clear() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleSimul_Holo *>(this), "internal_clear");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CVehicleSimulVirtualBase::internal_clear\"");
	}
};

void bind_mrpt_kinematics_CVehicleVelCmd_DiffDriven(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::kinematics::CVehicleVelCmd_DiffDriven file:mrpt/kinematics/CVehicleVelCmd_DiffDriven.h line:19
		pybind11::class_<mrpt::kinematics::CVehicleVelCmd_DiffDriven, std::shared_ptr<mrpt::kinematics::CVehicleVelCmd_DiffDriven>, PyCallBack_mrpt_kinematics_CVehicleVelCmd_DiffDriven, mrpt::kinematics::CVehicleVelCmd> cl(M("mrpt::kinematics"), "CVehicleVelCmd_DiffDriven", "Kinematic model for Ackermann-like or differential-driven vehicles.\n\n \n\n ");
		cl.def( pybind11::init( [](PyCallBack_mrpt_kinematics_CVehicleVelCmd_DiffDriven const &o){ return new PyCallBack_mrpt_kinematics_CVehicleVelCmd_DiffDriven(o); } ) );
		cl.def( pybind11::init( [](mrpt::kinematics::CVehicleVelCmd_DiffDriven const &o){ return new mrpt::kinematics::CVehicleVelCmd_DiffDriven(o); } ) );
		cl.def( pybind11::init( [](){ return new mrpt::kinematics::CVehicleVelCmd_DiffDriven(); }, [](){ return new PyCallBack_mrpt_kinematics_CVehicleVelCmd_DiffDriven(); } ) );
		cl.def_readwrite("lin_vel", &mrpt::kinematics::CVehicleVelCmd_DiffDriven::lin_vel);
		cl.def_readwrite("ang_vel", &mrpt::kinematics::CVehicleVelCmd_DiffDriven::ang_vel);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::kinematics::CVehicleVelCmd_DiffDriven::GetRuntimeClassIdStatic, "C++: mrpt::kinematics::CVehicleVelCmd_DiffDriven::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::kinematics::CVehicleVelCmd_DiffDriven::*)() const) &mrpt::kinematics::CVehicleVelCmd_DiffDriven::GetRuntimeClass, "C++: mrpt::kinematics::CVehicleVelCmd_DiffDriven::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::kinematics::CVehicleVelCmd_DiffDriven::*)() const) &mrpt::kinematics::CVehicleVelCmd_DiffDriven::clone, "C++: mrpt::kinematics::CVehicleVelCmd_DiffDriven::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::kinematics::CVehicleVelCmd_DiffDriven::CreateObject, "C++: mrpt::kinematics::CVehicleVelCmd_DiffDriven::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("getVelCmdLength", (size_t (mrpt::kinematics::CVehicleVelCmd_DiffDriven::*)() const) &mrpt::kinematics::CVehicleVelCmd_DiffDriven::getVelCmdLength, "C++: mrpt::kinematics::CVehicleVelCmd_DiffDriven::getVelCmdLength() const --> size_t");
		cl.def("getVelCmdDescription", (std::string (mrpt::kinematics::CVehicleVelCmd_DiffDriven::*)(const int) const) &mrpt::kinematics::CVehicleVelCmd_DiffDriven::getVelCmdDescription, "C++: mrpt::kinematics::CVehicleVelCmd_DiffDriven::getVelCmdDescription(const int) const --> std::string", pybind11::arg("index"));
		cl.def("getVelCmdElement", (double (mrpt::kinematics::CVehicleVelCmd_DiffDriven::*)(const int) const) &mrpt::kinematics::CVehicleVelCmd_DiffDriven::getVelCmdElement, "C++: mrpt::kinematics::CVehicleVelCmd_DiffDriven::getVelCmdElement(const int) const --> double", pybind11::arg("index"));
		cl.def("setVelCmdElement", (void (mrpt::kinematics::CVehicleVelCmd_DiffDriven::*)(const int, const double)) &mrpt::kinematics::CVehicleVelCmd_DiffDriven::setVelCmdElement, "C++: mrpt::kinematics::CVehicleVelCmd_DiffDriven::setVelCmdElement(const int, const double) --> void", pybind11::arg("index"), pybind11::arg("val"));
		cl.def("isStopCmd", (bool (mrpt::kinematics::CVehicleVelCmd_DiffDriven::*)() const) &mrpt::kinematics::CVehicleVelCmd_DiffDriven::isStopCmd, "C++: mrpt::kinematics::CVehicleVelCmd_DiffDriven::isStopCmd() const --> bool");
		cl.def("setToStop", (void (mrpt::kinematics::CVehicleVelCmd_DiffDriven::*)()) &mrpt::kinematics::CVehicleVelCmd_DiffDriven::setToStop, "C++: mrpt::kinematics::CVehicleVelCmd_DiffDriven::setToStop() --> void");
		cl.def("cmdVel_scale", (void (mrpt::kinematics::CVehicleVelCmd_DiffDriven::*)(double)) &mrpt::kinematics::CVehicleVelCmd_DiffDriven::cmdVel_scale, "See docs of method in base class. The implementation for\n differential-driven robots of this method\n just multiplies all the components of vel_cmd times vel_scale, which is\n appropriate\n  for differential-driven kinematic models (v,w).\n\nC++: mrpt::kinematics::CVehicleVelCmd_DiffDriven::cmdVel_scale(double) --> void", pybind11::arg("vel_scale"));
		cl.def("cmdVel_limits", (double (mrpt::kinematics::CVehicleVelCmd_DiffDriven::*)(const class mrpt::kinematics::CVehicleVelCmd &, const double, const struct mrpt::kinematics::CVehicleVelCmd::TVelCmdParams &)) &mrpt::kinematics::CVehicleVelCmd_DiffDriven::cmdVel_limits, "See base class docs.\n Tecognizes these parameters: `robotMax_V_mps`, `robotMax_W_degps` \n\nC++: mrpt::kinematics::CVehicleVelCmd_DiffDriven::cmdVel_limits(const class mrpt::kinematics::CVehicleVelCmd &, const double, const struct mrpt::kinematics::CVehicleVelCmd::TVelCmdParams &) --> double", pybind11::arg("prev_vel_cmd"), pybind11::arg("beta"), pybind11::arg("params"));
		cl.def("assign", (class mrpt::kinematics::CVehicleVelCmd_DiffDriven & (mrpt::kinematics::CVehicleVelCmd_DiffDriven::*)(const class mrpt::kinematics::CVehicleVelCmd_DiffDriven &)) &mrpt::kinematics::CVehicleVelCmd_DiffDriven::operator=, "C++: mrpt::kinematics::CVehicleVelCmd_DiffDriven::operator=(const class mrpt::kinematics::CVehicleVelCmd_DiffDriven &) --> class mrpt::kinematics::CVehicleVelCmd_DiffDriven &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::kinematics::CVehicleSimul_DiffDriven file:mrpt/kinematics/CVehicleSimul_DiffDriven.h line:21
		pybind11::class_<mrpt::kinematics::CVehicleSimul_DiffDriven, std::shared_ptr<mrpt::kinematics::CVehicleSimul_DiffDriven>, PyCallBack_mrpt_kinematics_CVehicleSimul_DiffDriven, mrpt::kinematics::CVehicleSimulVirtualBase> cl(M("mrpt::kinematics"), "CVehicleSimul_DiffDriven", "Simulates the kinematics of a differential-driven planar mobile\n robot/vehicle, including odometry errors and dynamics limitations.\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::kinematics::CVehicleSimul_DiffDriven(); }, [](){ return new PyCallBack_mrpt_kinematics_CVehicleSimul_DiffDriven(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_kinematics_CVehicleSimul_DiffDriven const &o){ return new PyCallBack_mrpt_kinematics_CVehicleSimul_DiffDriven(o); } ) );
		cl.def( pybind11::init( [](mrpt::kinematics::CVehicleSimul_DiffDriven const &o){ return new mrpt::kinematics::CVehicleSimul_DiffDriven(o); } ) );
		cl.def("setDelayModelParams", [](mrpt::kinematics::CVehicleSimul_DiffDriven &o) -> void { return o.setDelayModelParams(); }, "");
		cl.def("setDelayModelParams", [](mrpt::kinematics::CVehicleSimul_DiffDriven &o, double const & a0) -> void { return o.setDelayModelParams(a0); }, "", pybind11::arg("TAU_delay_sec"));
		cl.def("setDelayModelParams", (void (mrpt::kinematics::CVehicleSimul_DiffDriven::*)(double, double)) &mrpt::kinematics::CVehicleSimul_DiffDriven::setDelayModelParams, "Change the model of delays used for the orders sent to the robot \n\n movementCommand \n\nC++: mrpt::kinematics::CVehicleSimul_DiffDriven::setDelayModelParams(double, double) --> void", pybind11::arg("TAU_delay_sec"), pybind11::arg("CMD_delay_sec"));
		cl.def("setV", (void (mrpt::kinematics::CVehicleSimul_DiffDriven::*)(double)) &mrpt::kinematics::CVehicleSimul_DiffDriven::setV, "C++: mrpt::kinematics::CVehicleSimul_DiffDriven::setV(double) --> void", pybind11::arg("v"));
		cl.def("setW", (void (mrpt::kinematics::CVehicleSimul_DiffDriven::*)(double)) &mrpt::kinematics::CVehicleSimul_DiffDriven::setW, "C++: mrpt::kinematics::CVehicleSimul_DiffDriven::setW(double) --> void", pybind11::arg("w"));
		cl.def("getV", (double (mrpt::kinematics::CVehicleSimul_DiffDriven::*)()) &mrpt::kinematics::CVehicleSimul_DiffDriven::getV, "C++: mrpt::kinematics::CVehicleSimul_DiffDriven::getV() --> double");
		cl.def("getW", (double (mrpt::kinematics::CVehicleSimul_DiffDriven::*)()) &mrpt::kinematics::CVehicleSimul_DiffDriven::getW, "C++: mrpt::kinematics::CVehicleSimul_DiffDriven::getW() --> double");
		cl.def("movementCommand", (void (mrpt::kinematics::CVehicleSimul_DiffDriven::*)(double, double)) &mrpt::kinematics::CVehicleSimul_DiffDriven::movementCommand, "Used to command the robot a desired movement:\n \n\n Linar velocity (m/s)\n \n\n Angular velocity (rad/s)\n\nC++: mrpt::kinematics::CVehicleSimul_DiffDriven::movementCommand(double, double) --> void", pybind11::arg("lin_vel"), pybind11::arg("ang_vel"));
		cl.def("sendVelCmd", (void (mrpt::kinematics::CVehicleSimul_DiffDriven::*)(const class mrpt::kinematics::CVehicleVelCmd &)) &mrpt::kinematics::CVehicleSimul_DiffDriven::sendVelCmd, "C++: mrpt::kinematics::CVehicleSimul_DiffDriven::sendVelCmd(const class mrpt::kinematics::CVehicleVelCmd &) --> void", pybind11::arg("cmd_vel"));
		cl.def("getVelCmdType", (class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> (mrpt::kinematics::CVehicleSimul_DiffDriven::*)() const) &mrpt::kinematics::CVehicleSimul_DiffDriven::getVelCmdType, "C++: mrpt::kinematics::CVehicleSimul_DiffDriven::getVelCmdType() const --> class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>");
		cl.def("assign", (class mrpt::kinematics::CVehicleSimul_DiffDriven & (mrpt::kinematics::CVehicleSimul_DiffDriven::*)(const class mrpt::kinematics::CVehicleSimul_DiffDriven &)) &mrpt::kinematics::CVehicleSimul_DiffDriven::operator=, "C++: mrpt::kinematics::CVehicleSimul_DiffDriven::operator=(const class mrpt::kinematics::CVehicleSimul_DiffDriven &) --> class mrpt::kinematics::CVehicleSimul_DiffDriven &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::kinematics::CVehicleVelCmd_Holo file:mrpt/kinematics/CVehicleVelCmd_Holo.h line:19
		pybind11::class_<mrpt::kinematics::CVehicleVelCmd_Holo, std::shared_ptr<mrpt::kinematics::CVehicleVelCmd_Holo>, PyCallBack_mrpt_kinematics_CVehicleVelCmd_Holo, mrpt::kinematics::CVehicleVelCmd> cl(M("mrpt::kinematics"), "CVehicleVelCmd_Holo", "Kinematic model for\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::kinematics::CVehicleVelCmd_Holo(); }, [](){ return new PyCallBack_mrpt_kinematics_CVehicleVelCmd_Holo(); } ) );
		cl.def( pybind11::init<double, double, double, double>(), pybind11::arg("vel"), pybind11::arg("dir_local"), pybind11::arg("ramp_time"), pybind11::arg("rot_speed") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_kinematics_CVehicleVelCmd_Holo const &o){ return new PyCallBack_mrpt_kinematics_CVehicleVelCmd_Holo(o); } ) );
		cl.def( pybind11::init( [](mrpt::kinematics::CVehicleVelCmd_Holo const &o){ return new mrpt::kinematics::CVehicleVelCmd_Holo(o); } ) );
		cl.def_readwrite("vel", &mrpt::kinematics::CVehicleVelCmd_Holo::vel);
		cl.def_readwrite("dir_local", &mrpt::kinematics::CVehicleVelCmd_Holo::dir_local);
		cl.def_readwrite("ramp_time", &mrpt::kinematics::CVehicleVelCmd_Holo::ramp_time);
		cl.def_readwrite("rot_speed", &mrpt::kinematics::CVehicleVelCmd_Holo::rot_speed);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::kinematics::CVehicleVelCmd_Holo::GetRuntimeClassIdStatic, "C++: mrpt::kinematics::CVehicleVelCmd_Holo::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::kinematics::CVehicleVelCmd_Holo::*)() const) &mrpt::kinematics::CVehicleVelCmd_Holo::GetRuntimeClass, "C++: mrpt::kinematics::CVehicleVelCmd_Holo::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::kinematics::CVehicleVelCmd_Holo::*)() const) &mrpt::kinematics::CVehicleVelCmd_Holo::clone, "C++: mrpt::kinematics::CVehicleVelCmd_Holo::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::kinematics::CVehicleVelCmd_Holo::CreateObject, "C++: mrpt::kinematics::CVehicleVelCmd_Holo::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("getVelCmdLength", (size_t (mrpt::kinematics::CVehicleVelCmd_Holo::*)() const) &mrpt::kinematics::CVehicleVelCmd_Holo::getVelCmdLength, "C++: mrpt::kinematics::CVehicleVelCmd_Holo::getVelCmdLength() const --> size_t");
		cl.def("getVelCmdDescription", (std::string (mrpt::kinematics::CVehicleVelCmd_Holo::*)(const int) const) &mrpt::kinematics::CVehicleVelCmd_Holo::getVelCmdDescription, "C++: mrpt::kinematics::CVehicleVelCmd_Holo::getVelCmdDescription(const int) const --> std::string", pybind11::arg("index"));
		cl.def("getVelCmdElement", (double (mrpt::kinematics::CVehicleVelCmd_Holo::*)(const int) const) &mrpt::kinematics::CVehicleVelCmd_Holo::getVelCmdElement, "C++: mrpt::kinematics::CVehicleVelCmd_Holo::getVelCmdElement(const int) const --> double", pybind11::arg("index"));
		cl.def("setVelCmdElement", (void (mrpt::kinematics::CVehicleVelCmd_Holo::*)(const int, const double)) &mrpt::kinematics::CVehicleVelCmd_Holo::setVelCmdElement, "C++: mrpt::kinematics::CVehicleVelCmd_Holo::setVelCmdElement(const int, const double) --> void", pybind11::arg("index"), pybind11::arg("val"));
		cl.def("isStopCmd", (bool (mrpt::kinematics::CVehicleVelCmd_Holo::*)() const) &mrpt::kinematics::CVehicleVelCmd_Holo::isStopCmd, "C++: mrpt::kinematics::CVehicleVelCmd_Holo::isStopCmd() const --> bool");
		cl.def("setToStop", (void (mrpt::kinematics::CVehicleVelCmd_Holo::*)()) &mrpt::kinematics::CVehicleVelCmd_Holo::setToStop, "C++: mrpt::kinematics::CVehicleVelCmd_Holo::setToStop() --> void");
		cl.def("cmdVel_scale", (void (mrpt::kinematics::CVehicleVelCmd_Holo::*)(double)) &mrpt::kinematics::CVehicleVelCmd_Holo::cmdVel_scale, "C++: mrpt::kinematics::CVehicleVelCmd_Holo::cmdVel_scale(double) --> void", pybind11::arg("vel_scale"));
		cl.def("cmdVel_limits", (double (mrpt::kinematics::CVehicleVelCmd_Holo::*)(const class mrpt::kinematics::CVehicleVelCmd &, const double, const struct mrpt::kinematics::CVehicleVelCmd::TVelCmdParams &)) &mrpt::kinematics::CVehicleVelCmd_Holo::cmdVel_limits, "C++: mrpt::kinematics::CVehicleVelCmd_Holo::cmdVel_limits(const class mrpt::kinematics::CVehicleVelCmd &, const double, const struct mrpt::kinematics::CVehicleVelCmd::TVelCmdParams &) --> double", pybind11::arg("prev_vel_cmd"), pybind11::arg("beta"), pybind11::arg("params"));
		cl.def("assign", (class mrpt::kinematics::CVehicleVelCmd_Holo & (mrpt::kinematics::CVehicleVelCmd_Holo::*)(const class mrpt::kinematics::CVehicleVelCmd_Holo &)) &mrpt::kinematics::CVehicleVelCmd_Holo::operator=, "C++: mrpt::kinematics::CVehicleVelCmd_Holo::operator=(const class mrpt::kinematics::CVehicleVelCmd_Holo &) --> class mrpt::kinematics::CVehicleVelCmd_Holo &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::kinematics::CVehicleSimul_Holo file:mrpt/kinematics/CVehicleSimul_Holo.h line:23
		pybind11::class_<mrpt::kinematics::CVehicleSimul_Holo, std::shared_ptr<mrpt::kinematics::CVehicleSimul_Holo>, PyCallBack_mrpt_kinematics_CVehicleSimul_Holo, mrpt::kinematics::CVehicleSimulVirtualBase> cl(M("mrpt::kinematics"), "CVehicleSimul_Holo", "Kinematic simulator of a holonomic 2D robot capable of moving in any\n direction, with \"blended\"\n velocity profiles. See CVehicleSimul_Holo::sendVelCmd() for a description of\n the velocity commands in this kinematic model.\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::kinematics::CVehicleSimul_Holo(); }, [](){ return new PyCallBack_mrpt_kinematics_CVehicleSimul_Holo(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_kinematics_CVehicleSimul_Holo const &o){ return new PyCallBack_mrpt_kinematics_CVehicleSimul_Holo(o); } ) );
		cl.def( pybind11::init( [](mrpt::kinematics::CVehicleSimul_Holo const &o){ return new mrpt::kinematics::CVehicleSimul_Holo(o); } ) );
		cl.def("sendVelRampCmd", (void (mrpt::kinematics::CVehicleSimul_Holo::*)(double, double, double, double)) &mrpt::kinematics::CVehicleSimul_Holo::sendVelRampCmd, "Sends a velocity cmd to the holonomic robot.\n   \n\n Linear speed (m/s)\n   \n\n Direction (rad) (In the odometry frame of reference)\n   \n\n Blend the cmd during this period (seconds)\n   \n\n Rotational speed while there is heading error and\n to this constant (rad/s)\n\nC++: mrpt::kinematics::CVehicleSimul_Holo::sendVelRampCmd(double, double, double, double) --> void", pybind11::arg("vel"), pybind11::arg("dir"), pybind11::arg("ramp_time"), pybind11::arg("rot_speed"));
		cl.def("sendVelCmd", (void (mrpt::kinematics::CVehicleSimul_Holo::*)(const class mrpt::kinematics::CVehicleVelCmd &)) &mrpt::kinematics::CVehicleSimul_Holo::sendVelCmd, "C++: mrpt::kinematics::CVehicleSimul_Holo::sendVelCmd(const class mrpt::kinematics::CVehicleVelCmd &) --> void", pybind11::arg("cmd_vel"));
		cl.def("getVelCmdType", (class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> (mrpt::kinematics::CVehicleSimul_Holo::*)() const) &mrpt::kinematics::CVehicleSimul_Holo::getVelCmdType, "C++: mrpt::kinematics::CVehicleSimul_Holo::getVelCmdType() const --> class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>");
		cl.def("assign", (class mrpt::kinematics::CVehicleSimul_Holo & (mrpt::kinematics::CVehicleSimul_Holo::*)(const class mrpt::kinematics::CVehicleSimul_Holo &)) &mrpt::kinematics::CVehicleSimul_Holo::operator=, "C++: mrpt::kinematics::CVehicleSimul_Holo::operator=(const class mrpt::kinematics::CVehicleSimul_Holo &) --> class mrpt::kinematics::CVehicleSimul_Holo &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
