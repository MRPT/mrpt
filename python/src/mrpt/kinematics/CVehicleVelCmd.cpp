#include <iterator>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/kinematics/CVehicleSimulVirtualBase.h>
#include <mrpt/kinematics/CVehicleVelCmd.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TTwist2D.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/COutputLogger.h>
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

// mrpt::kinematics::CVehicleVelCmd file:mrpt/kinematics/CVehicleVelCmd.h line:22
struct PyCallBack_mrpt_kinematics_CVehicleVelCmd : public mrpt::kinematics::CVehicleVelCmd {
	using mrpt::kinematics::CVehicleVelCmd::CVehicleVelCmd;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CVehicleVelCmd::GetRuntimeClass();
	}
	size_t getVelCmdLength() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd *>(this), "getVelCmdLength");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CVehicleVelCmd::getVelCmdLength\"");
	}
	std::string getVelCmdDescription(const int a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd *>(this), "getVelCmdDescription");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CVehicleVelCmd::getVelCmdDescription\"");
	}
	double getVelCmdElement(const int a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd *>(this), "getVelCmdElement");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CVehicleVelCmd::getVelCmdElement\"");
	}
	void setVelCmdElement(const int a0, const double a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd *>(this), "setVelCmdElement");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CVehicleVelCmd::setVelCmdElement\"");
	}
	bool isStopCmd() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd *>(this), "isStopCmd");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CVehicleVelCmd::isStopCmd\"");
	}
	void setToStop() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd *>(this), "setToStop");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CVehicleVelCmd::setToStop\"");
	}
	std::string asString() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd *>(this), "asString");
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
	void cmdVel_scale(double a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd *>(this), "cmdVel_scale");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CVehicleVelCmd::cmdVel_scale\"");
	}
	double cmdVel_limits(const class mrpt::kinematics::CVehicleVelCmd & a0, const double a1, const struct mrpt::kinematics::CVehicleVelCmd::TVelCmdParams & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd *>(this), "cmdVel_limits");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CVehicleVelCmd::cmdVel_limits\"");
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CSerializable::serializeGetVersion\"");
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CSerializable::serializeTo\"");
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CSerializable::serializeFrom\"");
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleVelCmd *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CObject::clone\"");
	}
};

// mrpt::kinematics::CVehicleSimulVirtualBase file:mrpt/kinematics/CVehicleSimulVirtualBase.h line:32
struct PyCallBack_mrpt_kinematics_CVehicleSimulVirtualBase : public mrpt::kinematics::CVehicleSimulVirtualBase {
	using mrpt::kinematics::CVehicleSimulVirtualBase::CVehicleSimulVirtualBase;

	void sendVelCmd(const class mrpt::kinematics::CVehicleVelCmd & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleSimulVirtualBase *>(this), "sendVelCmd");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CVehicleSimulVirtualBase::sendVelCmd\"");
	}
	class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> getVelCmdType() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleSimulVirtualBase *>(this), "getVelCmdType");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>::value) {
				static pybind11::detail::override_caster_t<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>> caster;
				return pybind11::detail::cast_ref<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CVehicleSimulVirtualBase::getVelCmdType\"");
	}
	void internal_simulControlStep(const double a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleSimulVirtualBase *>(this), "internal_simulControlStep");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CVehicleSimulVirtualBase *>(this), "internal_clear");
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

void bind_mrpt_kinematics_CVehicleVelCmd(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::kinematics::CVehicleVelCmd file:mrpt/kinematics/CVehicleVelCmd.h line:22
		pybind11::class_<mrpt::kinematics::CVehicleVelCmd, std::shared_ptr<mrpt::kinematics::CVehicleVelCmd>, PyCallBack_mrpt_kinematics_CVehicleVelCmd, mrpt::serialization::CSerializable, mrpt::Stringifyable> cl(M("mrpt::kinematics"), "CVehicleVelCmd", "Virtual base for velocity commands of different kinematic models of planar\n mobile robot.\n \n");
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_kinematics_CVehicleVelCmd(); } ) );
		cl.def(pybind11::init<PyCallBack_mrpt_kinematics_CVehicleVelCmd const &>());
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::kinematics::CVehicleVelCmd::*)() const) &mrpt::kinematics::CVehicleVelCmd::GetRuntimeClass, "C++: mrpt::kinematics::CVehicleVelCmd::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::kinematics::CVehicleVelCmd::GetRuntimeClassIdStatic, "C++: mrpt::kinematics::CVehicleVelCmd::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("assign", (class mrpt::kinematics::CVehicleVelCmd & (mrpt::kinematics::CVehicleVelCmd::*)(const class mrpt::kinematics::CVehicleVelCmd &)) &mrpt::kinematics::CVehicleVelCmd::operator=, "C++: mrpt::kinematics::CVehicleVelCmd::operator=(const class mrpt::kinematics::CVehicleVelCmd &) --> class mrpt::kinematics::CVehicleVelCmd &", pybind11::return_value_policy::automatic, pybind11::arg("other"));
		cl.def("getVelCmdLength", (size_t (mrpt::kinematics::CVehicleVelCmd::*)() const) &mrpt::kinematics::CVehicleVelCmd::getVelCmdLength, "Get number of components in each velocity command \n\nC++: mrpt::kinematics::CVehicleVelCmd::getVelCmdLength() const --> size_t");
		cl.def("getVelCmdDescription", (std::string (mrpt::kinematics::CVehicleVelCmd::*)(const int) const) &mrpt::kinematics::CVehicleVelCmd::getVelCmdDescription, "Get textual, human-readable description of each velocity command\n component \n\nC++: mrpt::kinematics::CVehicleVelCmd::getVelCmdDescription(const int) const --> std::string", pybind11::arg("index"));
		cl.def("getVelCmdElement", (double (mrpt::kinematics::CVehicleVelCmd::*)(const int) const) &mrpt::kinematics::CVehicleVelCmd::getVelCmdElement, "Get each velocity command component \n\nC++: mrpt::kinematics::CVehicleVelCmd::getVelCmdElement(const int) const --> double", pybind11::arg("index"));
		cl.def("setVelCmdElement", (void (mrpt::kinematics::CVehicleVelCmd::*)(const int, const double)) &mrpt::kinematics::CVehicleVelCmd::setVelCmdElement, "Set each velocity command component \n\nC++: mrpt::kinematics::CVehicleVelCmd::setVelCmdElement(const int, const double) --> void", pybind11::arg("index"), pybind11::arg("val"));
		cl.def("isStopCmd", (bool (mrpt::kinematics::CVehicleVelCmd::*)() const) &mrpt::kinematics::CVehicleVelCmd::isStopCmd, "Returns true if the command means \"do not move\" / \"stop\". \n setToStop\n\nC++: mrpt::kinematics::CVehicleVelCmd::isStopCmd() const --> bool");
		cl.def("setToStop", (void (mrpt::kinematics::CVehicleVelCmd::*)()) &mrpt::kinematics::CVehicleVelCmd::setToStop, "Set to a command that means \"do not move\" / \"stop\". \n isStopCmd \n\nC++: mrpt::kinematics::CVehicleVelCmd::setToStop() --> void");
		cl.def("asString", (std::string (mrpt::kinematics::CVehicleVelCmd::*)() const) &mrpt::kinematics::CVehicleVelCmd::asString, "Returns a human readable description of the cmd \n\nC++: mrpt::kinematics::CVehicleVelCmd::asString() const --> std::string");
		cl.def("cmdVel_scale", (void (mrpt::kinematics::CVehicleVelCmd::*)(double)) &mrpt::kinematics::CVehicleVelCmd::cmdVel_scale, "Scale the velocity command encoded in this object.\n \n\n A scale within [0,1] reflecting how much should be\n the raw velocity command be lessen (e.g. for safety reasons,...).\n \n\n\n\n\n Users can directly inherit from existing implementations instead of\n manually redefining this method:\n  - mrpt::kinematics::CVehicleVelCmd_DiffDriven\n  - mrpt::kinematics::CVehicleVelCmd_Holo\n\nC++: mrpt::kinematics::CVehicleVelCmd::cmdVel_scale(double) --> void", pybind11::arg("vel_scale"));
		cl.def("cmdVel_limits", (double (mrpt::kinematics::CVehicleVelCmd::*)(const class mrpt::kinematics::CVehicleVelCmd &, const double, const struct mrpt::kinematics::CVehicleVelCmd::TVelCmdParams &)) &mrpt::kinematics::CVehicleVelCmd::cmdVel_limits, "Updates this command, computing a blended version of `beta` (within\n [0,1]) of `vel_cmd` and `1-beta` of `prev_vel_cmd`, simultaneously\n to honoring any user-side maximum velocities.\n \n\n The [0,1] ratio that the cmdvel had to be scaled down, or 1.0 if\n none.\n\nC++: mrpt::kinematics::CVehicleVelCmd::cmdVel_limits(const class mrpt::kinematics::CVehicleVelCmd &, const double, const struct mrpt::kinematics::CVehicleVelCmd::TVelCmdParams &) --> double", pybind11::arg("prev_vel_cmd"), pybind11::arg("beta"), pybind11::arg("params"));

		{ // mrpt::kinematics::CVehicleVelCmd::TVelCmdParams file:mrpt/kinematics/CVehicleVelCmd.h line:50
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::kinematics::CVehicleVelCmd::TVelCmdParams, std::shared_ptr<mrpt::kinematics::CVehicleVelCmd::TVelCmdParams>> cl(enclosing_class, "TVelCmdParams", "Parameters that may be used by cmdVel_limits() in any derived classes.");
			cl.def( pybind11::init( [](){ return new mrpt::kinematics::CVehicleVelCmd::TVelCmdParams(); } ) );
			cl.def( pybind11::init( [](mrpt::kinematics::CVehicleVelCmd::TVelCmdParams const &o){ return new mrpt::kinematics::CVehicleVelCmd::TVelCmdParams(o); } ) );
			cl.def_readwrite("robotMax_V_mps", &mrpt::kinematics::CVehicleVelCmd::TVelCmdParams::robotMax_V_mps);
			cl.def_readwrite("robotMax_W_radps", &mrpt::kinematics::CVehicleVelCmd::TVelCmdParams::robotMax_W_radps);
			cl.def_readwrite("robotMinCurvRadius", &mrpt::kinematics::CVehicleVelCmd::TVelCmdParams::robotMinCurvRadius);
			cl.def("loadConfigFile", (void (mrpt::kinematics::CVehicleVelCmd::TVelCmdParams::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::kinematics::CVehicleVelCmd::TVelCmdParams::loadConfigFile, "Load any parameter required by a CVehicleVelCmd derived class. \n\nC++: mrpt::kinematics::CVehicleVelCmd::TVelCmdParams::loadConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("cfg"), pybind11::arg("section"));
			cl.def("saveToConfigFile", (void (mrpt::kinematics::CVehicleVelCmd::TVelCmdParams::*)(class mrpt::config::CConfigFileBase &, const std::string &) const) &mrpt::kinematics::CVehicleVelCmd::TVelCmdParams::saveToConfigFile, "C++: mrpt::kinematics::CVehicleVelCmd::TVelCmdParams::saveToConfigFile(class mrpt::config::CConfigFileBase &, const std::string &) const --> void", pybind11::arg("c"), pybind11::arg("s"));
			cl.def("assign", (struct mrpt::kinematics::CVehicleVelCmd::TVelCmdParams & (mrpt::kinematics::CVehicleVelCmd::TVelCmdParams::*)(const struct mrpt::kinematics::CVehicleVelCmd::TVelCmdParams &)) &mrpt::kinematics::CVehicleVelCmd::TVelCmdParams::operator=, "C++: mrpt::kinematics::CVehicleVelCmd::TVelCmdParams::operator=(const struct mrpt::kinematics::CVehicleVelCmd::TVelCmdParams &) --> struct mrpt::kinematics::CVehicleVelCmd::TVelCmdParams &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
	{ // mrpt::kinematics::CVehicleSimulVirtualBase file:mrpt/kinematics/CVehicleSimulVirtualBase.h line:32
		pybind11::class_<mrpt::kinematics::CVehicleSimulVirtualBase, std::shared_ptr<mrpt::kinematics::CVehicleSimulVirtualBase>, PyCallBack_mrpt_kinematics_CVehicleSimulVirtualBase> cl(M("mrpt::kinematics"), "CVehicleSimulVirtualBase", "This class can be used to simulate the kinematics and dynamics of a\n differential driven planar mobile robot, including odometry errors and\n dynamics limitations.\n  Main API methods are:\n  - movementCommand: Call this for send a command to the robot. This comamnd\n will be\n    delayed and passed throught a first order low-pass filter to simulate\n    robot dynamics.\n  - simulateInterval: Call this for run the simulator for the desired time\n period.\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_kinematics_CVehicleSimulVirtualBase(); } ) );
		cl.def(pybind11::init<PyCallBack_mrpt_kinematics_CVehicleSimulVirtualBase const &>());
		cl.def("simulateOneTimeStep", (void (mrpt::kinematics::CVehicleSimulVirtualBase::*)(const double)) &mrpt::kinematics::CVehicleSimulVirtualBase::simulateOneTimeStep, "@{ \n\n Runs the simulator during \"dt\" seconds. It will be split into periods of\n \"m_firmware_control_period\". \n\nC++: mrpt::kinematics::CVehicleSimulVirtualBase::simulateOneTimeStep(const double) --> void", pybind11::arg("dt"));
		cl.def("getCurrentGTPose", (const struct mrpt::math::TPose2D & (mrpt::kinematics::CVehicleSimulVirtualBase::*)() const) &mrpt::kinematics::CVehicleSimulVirtualBase::getCurrentGTPose, "Returns the instantaneous, ground truth pose in world coordinates \n\nC++: mrpt::kinematics::CVehicleSimulVirtualBase::getCurrentGTPose() const --> const struct mrpt::math::TPose2D &", pybind11::return_value_policy::automatic);
		cl.def("setCurrentGTPose", (void (mrpt::kinematics::CVehicleSimulVirtualBase::*)(const struct mrpt::math::TPose2D &)) &mrpt::kinematics::CVehicleSimulVirtualBase::setCurrentGTPose, "Brute-force move robot to target coordinates (\"teleport\") \n\nC++: mrpt::kinematics::CVehicleSimulVirtualBase::setCurrentGTPose(const struct mrpt::math::TPose2D &) --> void", pybind11::arg("pose"));
		cl.def("getCurrentOdometricPose", (const struct mrpt::math::TPose2D & (mrpt::kinematics::CVehicleSimulVirtualBase::*)() const) &mrpt::kinematics::CVehicleSimulVirtualBase::getCurrentOdometricPose, "Returns the current pose according to (noisy) odometry \n\n setOdometryErrors \n\nC++: mrpt::kinematics::CVehicleSimulVirtualBase::getCurrentOdometricPose() const --> const struct mrpt::math::TPose2D &", pybind11::return_value_policy::automatic);
		cl.def("getCurrentGTVel", (const struct mrpt::math::TTwist2D & (mrpt::kinematics::CVehicleSimulVirtualBase::*)() const) &mrpt::kinematics::CVehicleSimulVirtualBase::getCurrentGTVel, "Returns the instantaneous, ground truth velocity vector (vx,vy,omega) in\n world coordinates \n\nC++: mrpt::kinematics::CVehicleSimulVirtualBase::getCurrentGTVel() const --> const struct mrpt::math::TTwist2D &", pybind11::return_value_policy::automatic);
		cl.def("getCurrentGTVelLocal", (struct mrpt::math::TTwist2D (mrpt::kinematics::CVehicleSimulVirtualBase::*)() const) &mrpt::kinematics::CVehicleSimulVirtualBase::getCurrentGTVelLocal, "Returns the instantaneous, ground truth velocity vector (vx,vy,omega) in\n the robot local frame \n\nC++: mrpt::kinematics::CVehicleSimulVirtualBase::getCurrentGTVelLocal() const --> struct mrpt::math::TTwist2D");
		cl.def("getCurrentOdometricVel", (const struct mrpt::math::TTwist2D & (mrpt::kinematics::CVehicleSimulVirtualBase::*)() const) &mrpt::kinematics::CVehicleSimulVirtualBase::getCurrentOdometricVel, "Returns the instantaneous, odometric velocity vector (vx,vy,omega) in\n world coordinates \n\nC++: mrpt::kinematics::CVehicleSimulVirtualBase::getCurrentOdometricVel() const --> const struct mrpt::math::TTwist2D &", pybind11::return_value_policy::automatic);
		cl.def("getCurrentOdometricVelLocal", (struct mrpt::math::TTwist2D (mrpt::kinematics::CVehicleSimulVirtualBase::*)() const) &mrpt::kinematics::CVehicleSimulVirtualBase::getCurrentOdometricVelLocal, "Returns the instantaneous, odometric velocity vector (vx,vy,omega) in\n the robot local frame \n\nC++: mrpt::kinematics::CVehicleSimulVirtualBase::getCurrentOdometricVelLocal() const --> struct mrpt::math::TTwist2D");
		cl.def("getTime", (double (mrpt::kinematics::CVehicleSimulVirtualBase::*)() const) &mrpt::kinematics::CVehicleSimulVirtualBase::getTime, "Get the current simulation time \n\nC++: mrpt::kinematics::CVehicleSimulVirtualBase::getTime() const --> double");
		cl.def("sendVelCmd", (void (mrpt::kinematics::CVehicleSimulVirtualBase::*)(const class mrpt::kinematics::CVehicleVelCmd &)) &mrpt::kinematics::CVehicleSimulVirtualBase::sendVelCmd, "Sends a velocity command to the robot. The number of components and\n their meaning depends\n on the vehicle-kinematics derived class \n\nC++: mrpt::kinematics::CVehicleSimulVirtualBase::sendVelCmd(const class mrpt::kinematics::CVehicleVelCmd &) --> void", pybind11::arg("cmd_vel"));
		cl.def("getVelCmdType", (class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> (mrpt::kinematics::CVehicleSimulVirtualBase::*)() const) &mrpt::kinematics::CVehicleSimulVirtualBase::getVelCmdType, "Gets an empty velocity command object that can be queried to find out\n the number of velcmd components,... \n\nC++: mrpt::kinematics::CVehicleSimulVirtualBase::getVelCmdType() const --> class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>");
		cl.def("setOdometryErrors", [](mrpt::kinematics::CVehicleSimulVirtualBase &o, bool const & a0) -> void { return o.setOdometryErrors(a0); }, "", pybind11::arg("enabled"));
		cl.def("setOdometryErrors", [](mrpt::kinematics::CVehicleSimulVirtualBase &o, bool const & a0, double const & a1) -> void { return o.setOdometryErrors(a0, a1); }, "", pybind11::arg("enabled"), pybind11::arg("Ax_err_bias"));
		cl.def("setOdometryErrors", [](mrpt::kinematics::CVehicleSimulVirtualBase &o, bool const & a0, double const & a1, double const & a2) -> void { return o.setOdometryErrors(a0, a1, a2); }, "", pybind11::arg("enabled"), pybind11::arg("Ax_err_bias"), pybind11::arg("Ax_err_std"));
		cl.def("setOdometryErrors", [](mrpt::kinematics::CVehicleSimulVirtualBase &o, bool const & a0, double const & a1, double const & a2, double const & a3) -> void { return o.setOdometryErrors(a0, a1, a2, a3); }, "", pybind11::arg("enabled"), pybind11::arg("Ax_err_bias"), pybind11::arg("Ax_err_std"), pybind11::arg("Ay_err_bias"));
		cl.def("setOdometryErrors", [](mrpt::kinematics::CVehicleSimulVirtualBase &o, bool const & a0, double const & a1, double const & a2, double const & a3, double const & a4) -> void { return o.setOdometryErrors(a0, a1, a2, a3, a4); }, "", pybind11::arg("enabled"), pybind11::arg("Ax_err_bias"), pybind11::arg("Ax_err_std"), pybind11::arg("Ay_err_bias"), pybind11::arg("Ay_err_std"));
		cl.def("setOdometryErrors", [](mrpt::kinematics::CVehicleSimulVirtualBase &o, bool const & a0, double const & a1, double const & a2, double const & a3, double const & a4, double const & a5) -> void { return o.setOdometryErrors(a0, a1, a2, a3, a4, a5); }, "", pybind11::arg("enabled"), pybind11::arg("Ax_err_bias"), pybind11::arg("Ax_err_std"), pybind11::arg("Ay_err_bias"), pybind11::arg("Ay_err_std"), pybind11::arg("Aphi_err_bias"));
		cl.def("setOdometryErrors", (void (mrpt::kinematics::CVehicleSimulVirtualBase::*)(bool, double, double, double, double, double, double)) &mrpt::kinematics::CVehicleSimulVirtualBase::setOdometryErrors, "Enable/Disable odometry errors. Errors in odometry are 1 sigma Gaussian\n values per second \n\nC++: mrpt::kinematics::CVehicleSimulVirtualBase::setOdometryErrors(bool, double, double, double, double, double, double) --> void", pybind11::arg("enabled"), pybind11::arg("Ax_err_bias"), pybind11::arg("Ax_err_std"), pybind11::arg("Ay_err_bias"), pybind11::arg("Ay_err_std"), pybind11::arg("Aphi_err_bias"), pybind11::arg("Aphi_err_std"));
		cl.def("resetStatus", (void (mrpt::kinematics::CVehicleSimulVirtualBase::*)()) &mrpt::kinematics::CVehicleSimulVirtualBase::resetStatus, "C++: mrpt::kinematics::CVehicleSimulVirtualBase::resetStatus() --> void");
		cl.def("resetTime", (void (mrpt::kinematics::CVehicleSimulVirtualBase::*)()) &mrpt::kinematics::CVehicleSimulVirtualBase::resetTime, "Reset all simulator variables to 0 (except the\n simulation time). \n\n resetTime\n Reset time counter \n\n resetStatus \n\nC++: mrpt::kinematics::CVehicleSimulVirtualBase::resetTime() --> void");
		cl.def("assign", (class mrpt::kinematics::CVehicleSimulVirtualBase & (mrpt::kinematics::CVehicleSimulVirtualBase::*)(const class mrpt::kinematics::CVehicleSimulVirtualBase &)) &mrpt::kinematics::CVehicleSimulVirtualBase::operator=, "C++: mrpt::kinematics::CVehicleSimulVirtualBase::operator=(const class mrpt::kinematics::CVehicleSimulVirtualBase &) --> class mrpt::kinematics::CVehicleSimulVirtualBase &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
