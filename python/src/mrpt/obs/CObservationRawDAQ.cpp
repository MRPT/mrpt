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
#include <mrpt/obs/CObservationRawDAQ.h>
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

// mrpt::obs::CObservationRawDAQ file:mrpt/obs/CObservationRawDAQ.h line:29
struct PyCallBack_mrpt_obs_CObservationRawDAQ : public mrpt::obs::CObservationRawDAQ {
	using mrpt::obs::CObservationRawDAQ::CObservationRawDAQ;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRawDAQ *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CObservationRawDAQ::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRawDAQ *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CObservationRawDAQ::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRawDAQ *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CObservationRawDAQ::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRawDAQ *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationRawDAQ::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRawDAQ *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationRawDAQ::serializeFrom(a0, a1);
	}
	void getSensorPose(class mrpt::poses::CPose3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRawDAQ *>(this), "getSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationRawDAQ::getSensorPose(a0);
	}
	void setSensorPose(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRawDAQ *>(this), "setSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationRawDAQ::setSensorPose(a0);
	}
	using _binder_ret_0 = mrpt::Clock::time_point;
	_binder_ret_0 getOriginalReceivedTimeStamp() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRawDAQ *>(this), "getOriginalReceivedTimeStamp");
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
	std::string asString() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRawDAQ *>(this), "asString");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRawDAQ *>(this), "exportTxtSupported");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRawDAQ *>(this), "exportTxtHeader");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRawDAQ *>(this), "exportTxtDataRow");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRawDAQ *>(this), "unload");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRawDAQ *>(this), "load_impl");
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
};

void bind_mrpt_obs_CObservationRawDAQ(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::obs::CObservationRawDAQ file:mrpt/obs/CObservationRawDAQ.h line:29
		pybind11::class_<mrpt::obs::CObservationRawDAQ, std::shared_ptr<mrpt::obs::CObservationRawDAQ>, PyCallBack_mrpt_obs_CObservationRawDAQ, mrpt::obs::CObservation> cl(M("mrpt::obs"), "CObservationRawDAQ", "Store raw data from a Data Acquisition (DAQ) device, such that input or\n output analog and digital channels, counters from encoders, etc. at one\n sampling instant.\n  All analog values are assumed to be volts.\n On timing:\n - CObservation::timestamp corresponds to the time of the first samples in\n each of the vectors.\n - CObservationRawDAQ::sample_rate is the sampling rate, in samples per second\n per channel, as stored by the source driver.\n\n \n CObservation\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::obs::CObservationRawDAQ(); }, [](){ return new PyCallBack_mrpt_obs_CObservationRawDAQ(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_CObservationRawDAQ const &o){ return new PyCallBack_mrpt_obs_CObservationRawDAQ(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::CObservationRawDAQ const &o){ return new mrpt::obs::CObservationRawDAQ(o); } ) );
		cl.def_readwrite("AIN_8bits", &mrpt::obs::CObservationRawDAQ::AIN_8bits);
		cl.def_readwrite("AIN_16bits", &mrpt::obs::CObservationRawDAQ::AIN_16bits);
		cl.def_readwrite("AIN_32bits", &mrpt::obs::CObservationRawDAQ::AIN_32bits);
		cl.def_readwrite("AIN_float", &mrpt::obs::CObservationRawDAQ::AIN_float);
		cl.def_readwrite("AIN_double", &mrpt::obs::CObservationRawDAQ::AIN_double);
		cl.def_readwrite("AIN_channel_count", &mrpt::obs::CObservationRawDAQ::AIN_channel_count);
		cl.def_readwrite("AIN_interleaved", &mrpt::obs::CObservationRawDAQ::AIN_interleaved);
		cl.def_readwrite("AOUT_8bits", &mrpt::obs::CObservationRawDAQ::AOUT_8bits);
		cl.def_readwrite("AOUT_16bits", &mrpt::obs::CObservationRawDAQ::AOUT_16bits);
		cl.def_readwrite("AOUT_float", &mrpt::obs::CObservationRawDAQ::AOUT_float);
		cl.def_readwrite("AOUT_double", &mrpt::obs::CObservationRawDAQ::AOUT_double);
		cl.def_readwrite("DIN", &mrpt::obs::CObservationRawDAQ::DIN);
		cl.def_readwrite("DOUT", &mrpt::obs::CObservationRawDAQ::DOUT);
		cl.def_readwrite("CNTRIN_32bits", &mrpt::obs::CObservationRawDAQ::CNTRIN_32bits);
		cl.def_readwrite("CNTRIN_double", &mrpt::obs::CObservationRawDAQ::CNTRIN_double);
		cl.def_readwrite("sample_rate", &mrpt::obs::CObservationRawDAQ::sample_rate);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::obs::CObservationRawDAQ::GetRuntimeClassIdStatic, "C++: mrpt::obs::CObservationRawDAQ::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::obs::CObservationRawDAQ::*)() const) &mrpt::obs::CObservationRawDAQ::GetRuntimeClass, "C++: mrpt::obs::CObservationRawDAQ::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::obs::CObservationRawDAQ::*)() const) &mrpt::obs::CObservationRawDAQ::clone, "C++: mrpt::obs::CObservationRawDAQ::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::obs::CObservationRawDAQ::CreateObject, "C++: mrpt::obs::CObservationRawDAQ::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("getSensorPose", (void (mrpt::obs::CObservationRawDAQ::*)(class mrpt::poses::CPose3D &) const) &mrpt::obs::CObservationRawDAQ::getSensorPose, "Not used in this class \n\nC++: mrpt::obs::CObservationRawDAQ::getSensorPose(class mrpt::poses::CPose3D &) const --> void", pybind11::arg(""));
		cl.def("setSensorPose", (void (mrpt::obs::CObservationRawDAQ::*)(const class mrpt::poses::CPose3D &)) &mrpt::obs::CObservationRawDAQ::setSensorPose, "Not used in this class \n\nC++: mrpt::obs::CObservationRawDAQ::setSensorPose(const class mrpt::poses::CPose3D &) --> void", pybind11::arg(""));
		cl.def("assign", (class mrpt::obs::CObservationRawDAQ & (mrpt::obs::CObservationRawDAQ::*)(const class mrpt::obs::CObservationRawDAQ &)) &mrpt::obs::CObservationRawDAQ::operator=, "C++: mrpt::obs::CObservationRawDAQ::operator=(const class mrpt::obs::CObservationRawDAQ &) --> class mrpt::obs::CObservationRawDAQ &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
