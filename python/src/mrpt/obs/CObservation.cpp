#include <chrono>
#include <ios>
#include <iterator>
#include <memory>
#include <mrpt/core/Clock.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TPose3DQuat.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ostream>
#include <ratio>
#include <sstream> // __str__
#include <streambuf>
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

// mrpt::obs::CObservation file:mrpt/obs/CObservation.h line:50
struct PyCallBack_mrpt_obs_CObservation : public mrpt::obs::CObservation {
	using mrpt::obs::CObservation::CObservation;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CObservation::GetRuntimeClass();
	}
	using _binder_ret_0 = mrpt::Clock::time_point;
	_binder_ret_0 getOriginalReceivedTimeStamp() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation *>(this), "getOriginalReceivedTimeStamp");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<_binder_ret_0>::value) {
				static pybind11::detail::override_caster_t<_binder_ret_0> caster;
				return pybind11::detail::cast_ref<_binder_ret_0>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<_binder_ret_0>(std::move(o));
		}
		return CObservation::getOriginalReceivedTimeStamp();
	}
	void getSensorPose(class mrpt::poses::CPose3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation *>(this), "getSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CObservation::getSensorPose\"");
	}
	void setSensorPose(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation *>(this), "setSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CObservation::setSensorPose\"");
	}
	std::string asString() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation *>(this), "asString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CObservation::asString();
	}
	bool exportTxtSupported() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation *>(this), "exportTxtSupported");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CObservation::exportTxtSupported();
	}
	std::string exportTxtHeader() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation *>(this), "exportTxtHeader");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CObservation::exportTxtHeader();
	}
	std::string exportTxtDataRow() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation *>(this), "exportTxtDataRow");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CObservation::exportTxtDataRow();
	}
	void unload() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation *>(this), "unload");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservation::unload();
	}
	void load_impl() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation *>(this), "load_impl");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservation::load_impl();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation *>(this), "serializeGetVersion");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation *>(this), "serializeTo");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation *>(this), "serializeFrom");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation *>(this), "clone");
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

void bind_mrpt_obs_CObservation(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::obs::CObservation file:mrpt/obs/CObservation.h line:50
		pybind11::class_<mrpt::obs::CObservation, std::shared_ptr<mrpt::obs::CObservation>, PyCallBack_mrpt_obs_CObservation, mrpt::serialization::CSerializable, mrpt::Stringifyable> cl(M("mrpt::obs"), "CObservation", "Generic sensor observation.\n\n  This is a base virtual class for all types of sensor observations.\n  Users can add new observation types creating a new class deriving from this\n one, or reuse those provided in MRPT modules. Most observations are defined\n in \n\n\n Observations do not include any information about the robot localization,\n but just raw sensory data and, where aplicable, information about the\n sensor position and orientation in the **local frame** (vehicle frame).\n\n Datasets with large number of observations can be managed with\n mrpt::obs::CRawLog.\n\n \n CSensoryFrame, CMetricMap, mrpt::obs::CRawLog\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_obs_CObservation(); } ) );
		cl.def(pybind11::init<PyCallBack_mrpt_obs_CObservation const &>());
		cl.def_readwrite("timestamp", &mrpt::obs::CObservation::timestamp);
		cl.def_readwrite("sensorLabel", &mrpt::obs::CObservation::sensorLabel);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::obs::CObservation::*)() const) &mrpt::obs::CObservation::GetRuntimeClass, "C++: mrpt::obs::CObservation::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::obs::CObservation::GetRuntimeClassIdStatic, "C++: mrpt::obs::CObservation::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("getTimeStamp", (mrpt::Clock::time_point (mrpt::obs::CObservation::*)() const) &mrpt::obs::CObservation::getTimeStamp, "Returns CObservation::timestamp for all kind of observations \n\n getOriginalReceivedTimeStamp() \n\nC++: mrpt::obs::CObservation::getTimeStamp() const --> mrpt::Clock::time_point");
		cl.def("getOriginalReceivedTimeStamp", (mrpt::Clock::time_point (mrpt::obs::CObservation::*)() const) &mrpt::obs::CObservation::getOriginalReceivedTimeStamp, "By default, returns CObservation::timestamp but in sensors capable of\n satellite (or otherwise) accurate UTC timing of readings, this contains\n the computer-based timestamp of reception, which may be slightly\n different than  \n\n getTimeStamp()  \n\nC++: mrpt::obs::CObservation::getOriginalReceivedTimeStamp() const --> mrpt::Clock::time_point");
		cl.def("getSensorPose", (void (mrpt::obs::CObservation::*)(class mrpt::poses::CPose3D &) const) &mrpt::obs::CObservation::getSensorPose, "A general method to retrieve the sensor pose on the robot.\n  Note that most sensors will return a full (6D) CPose3D, but see the\n derived classes for more details or special cases.\n \n\n setSensorPose\n\nC++: mrpt::obs::CObservation::getSensorPose(class mrpt::poses::CPose3D &) const --> void", pybind11::arg("out_sensorPose"));
		cl.def("getSensorPose", (void (mrpt::obs::CObservation::*)(struct mrpt::math::TPose3D &) const) &mrpt::obs::CObservation::getSensorPose, "A general method to retrieve the sensor pose on the robot.\n  Note that most sensors will return a full (6D) CPose3D, but see the\n derived classes for more details or special cases.\n \n\n setSensorPose\n\nC++: mrpt::obs::CObservation::getSensorPose(struct mrpt::math::TPose3D &) const --> void", pybind11::arg("out_sensorPose"));
		cl.def("sensorPose", (struct mrpt::math::TPose3D (mrpt::obs::CObservation::*)() const) &mrpt::obs::CObservation::sensorPose, "synonym with getSensorPose()\n \n\n getSensorPose\n \n\n (New in MRPT 2.3.1)\n\nC++: mrpt::obs::CObservation::sensorPose() const --> struct mrpt::math::TPose3D");
		cl.def("setSensorPose", (void (mrpt::obs::CObservation::*)(const class mrpt::poses::CPose3D &)) &mrpt::obs::CObservation::setSensorPose, "A general method to change the sensor pose on the robot.\n  Note that most sensors will use the full (6D) CPose3D, but see the\n derived classes for more details or special cases.\n \n\n getSensorPose\n\nC++: mrpt::obs::CObservation::setSensorPose(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newSensorPose"));
		cl.def("setSensorPose", (void (mrpt::obs::CObservation::*)(const struct mrpt::math::TPose3D &)) &mrpt::obs::CObservation::setSensorPose, "A general method to change the sensor pose on the robot.\n  Note that most sensors will use the full (6D) CPose3D, but see the\n derived classes for more details or special cases.\n \n\n getSensorPose\n\nC++: mrpt::obs::CObservation::setSensorPose(const struct mrpt::math::TPose3D &) --> void", pybind11::arg("newSensorPose"));
		cl.def("asString", (std::string (mrpt::obs::CObservation::*)() const) &mrpt::obs::CObservation::asString, "Return by value version of getDescriptionAsText(std::ostream&).\n\nC++: mrpt::obs::CObservation::asString() const --> std::string");
		cl.def("exportTxtSupported", (bool (mrpt::obs::CObservation::*)() const) &mrpt::obs::CObservation::exportTxtSupported, "@{ \n\n Must return true if the class is exportable to TXT/CSV files, in which\n case the other virtual methods in this group must be redefined too.\n\nC++: mrpt::obs::CObservation::exportTxtSupported() const --> bool");
		cl.def("exportTxtHeader", (std::string (mrpt::obs::CObservation::*)() const) &mrpt::obs::CObservation::exportTxtHeader, "Returns the description of the data columns. Timestamp is automatically\n included as the first column, do not list it. See example implementations\n if interested in enabling this in custom CObservation classes.\n Do not include newlines.\n\nC++: mrpt::obs::CObservation::exportTxtHeader() const --> std::string");
		cl.def("exportTxtDataRow", (std::string (mrpt::obs::CObservation::*)() const) &mrpt::obs::CObservation::exportTxtDataRow, "Returns one row of data with the data stored in this particular object.\n Do not include newlines. \n\nC++: mrpt::obs::CObservation::exportTxtDataRow() const --> std::string");
		cl.def("load", (void (mrpt::obs::CObservation::*)() const) &mrpt::obs::CObservation::load, "Makes sure all images and other fields which may be externally stored\n are loaded in memory.\n  Note that for all CImages, calling load() is not required since the\n images will be automatically loaded upon first access, so load()\n shouldn't be needed to be called in normal cases by the user.\n  If all the data were alredy loaded or this object has no externally\n stored data fields, calling this method has no effects.\n \n\n unload\n\nC++: mrpt::obs::CObservation::load() const --> void");
		cl.def("unload", (void (mrpt::obs::CObservation::*)() const) &mrpt::obs::CObservation::unload, "Unload all images, for the case they being delayed-load images stored in\n external files (othewise, has no effect).\n \n\n load\n\nC++: mrpt::obs::CObservation::unload() const --> void");
		cl.def("assign", (class mrpt::obs::CObservation & (mrpt::obs::CObservation::*)(const class mrpt::obs::CObservation &)) &mrpt::obs::CObservation::operator=, "C++: mrpt::obs::CObservation::operator=(const class mrpt::obs::CObservation &) --> class mrpt::obs::CObservation &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
