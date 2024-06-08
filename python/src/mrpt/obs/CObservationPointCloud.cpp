#include <chrono>
#include <ios>
#include <iterator>
#include <memory>
#include <mrpt/core/Clock.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/img/color_maps.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CPolygon.h>
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
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationRotatingScan.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/obs/T2DScanProperties.h>
#include <mrpt/obs/T3DPointsTo2DScanParams.h>
#include <mrpt/obs/TRangeImageFilter.h>
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

// mrpt::obs::CObservationPointCloud file:mrpt/obs/CObservationPointCloud.h line:33
struct PyCallBack_mrpt_obs_CObservationPointCloud : public mrpt::obs::CObservationPointCloud {
	using mrpt::obs::CObservationPointCloud::CObservationPointCloud;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationPointCloud *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CObservationPointCloud::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationPointCloud *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CObservationPointCloud::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationPointCloud *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CObservationPointCloud::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationPointCloud *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationPointCloud::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationPointCloud *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationPointCloud::serializeFrom(a0, a1);
	}
	void getSensorPose(class mrpt::poses::CPose3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationPointCloud *>(this), "getSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationPointCloud::getSensorPose(a0);
	}
	void setSensorPose(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationPointCloud *>(this), "setSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationPointCloud::setSensorPose(a0);
	}
	void load_impl() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationPointCloud *>(this), "load_impl");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationPointCloud::load_impl();
	}
	void unload() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationPointCloud *>(this), "unload");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationPointCloud::unload();
	}
	using _binder_ret_0 = mrpt::Clock::time_point;
	_binder_ret_0 getOriginalReceivedTimeStamp() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationPointCloud *>(this), "getOriginalReceivedTimeStamp");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationPointCloud *>(this), "asString");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationPointCloud *>(this), "exportTxtSupported");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationPointCloud *>(this), "exportTxtHeader");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationPointCloud *>(this), "exportTxtDataRow");
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
};

// mrpt::obs::CObservationRotatingScan file:mrpt/obs/CObservationRotatingScan.h line:53
struct PyCallBack_mrpt_obs_CObservationRotatingScan : public mrpt::obs::CObservationRotatingScan {
	using mrpt::obs::CObservationRotatingScan::CObservationRotatingScan;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRotatingScan *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CObservationRotatingScan::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRotatingScan *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CObservationRotatingScan::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRotatingScan *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CObservationRotatingScan::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRotatingScan *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationRotatingScan::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRotatingScan *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationRotatingScan::serializeFrom(a0, a1);
	}
	void load_impl() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRotatingScan *>(this), "load_impl");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationRotatingScan::load_impl();
	}
	void unload() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRotatingScan *>(this), "unload");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationRotatingScan::unload();
	}
	using _binder_ret_0 = mrpt::Clock::time_point;
	_binder_ret_0 getOriginalReceivedTimeStamp() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRotatingScan *>(this), "getOriginalReceivedTimeStamp");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<_binder_ret_0>::value) {
				static pybind11::detail::override_caster_t<_binder_ret_0> caster;
				return pybind11::detail::cast_ref<_binder_ret_0>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<_binder_ret_0>(std::move(o));
		}
		return CObservationRotatingScan::getOriginalReceivedTimeStamp();
	}
	void getSensorPose(class mrpt::poses::CPose3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRotatingScan *>(this), "getSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationRotatingScan::getSensorPose(a0);
	}
	void setSensorPose(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRotatingScan *>(this), "setSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationRotatingScan::setSensorPose(a0);
	}
	std::string asString() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRotatingScan *>(this), "asString");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRotatingScan *>(this), "exportTxtSupported");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRotatingScan *>(this), "exportTxtHeader");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRotatingScan *>(this), "exportTxtDataRow");
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
};

void bind_mrpt_obs_CObservationPointCloud(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::obs::CObservationPointCloud file:mrpt/obs/CObservationPointCloud.h line:33
		pybind11::class_<mrpt::obs::CObservationPointCloud, std::shared_ptr<mrpt::obs::CObservationPointCloud>, PyCallBack_mrpt_obs_CObservationPointCloud, mrpt::obs::CObservation> cl(M("mrpt::obs"), "CObservationPointCloud", "An observation from any sensor that can be summarized as a pointcloud.\n The cloud can comprise plain XYZ points, or can include intensity, or RGB\n data; in particular, the point cloud can be any of the derived classes of\n mrpt::maps::CPointsMap.\n\n The pointcloud has as frame of coordinates the `sensorPose` field, which\n may match the origin of the vehicle/robot or not.\n\n \n This is a mrpt::obs::CObservation class, but it is defined in the\n mrpt_maps_grp library, so it can use mrpt::maps::CPointsMap.\n\n \n CObservation, mrpt::maps::CPointsMap\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::obs::CObservationPointCloud(); }, [](){ return new PyCallBack_mrpt_obs_CObservationPointCloud(); } ) );
		cl.def( pybind11::init<const class mrpt::obs::CObservation3DRangeScan &>(), pybind11::arg("o") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_CObservationPointCloud const &o){ return new PyCallBack_mrpt_obs_CObservationPointCloud(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::CObservationPointCloud const &o){ return new mrpt::obs::CObservationPointCloud(o); } ) );

		pybind11::enum_<mrpt::obs::CObservationPointCloud::ExternalStorageFormat>(cl, "ExternalStorageFormat", "")
			.value("None", mrpt::obs::CObservationPointCloud::ExternalStorageFormat::None)
			.value("MRPT_Serialization", mrpt::obs::CObservationPointCloud::ExternalStorageFormat::MRPT_Serialization)
			.value("KittiBinFile", mrpt::obs::CObservationPointCloud::ExternalStorageFormat::KittiBinFile)
			.value("PlainTextFile", mrpt::obs::CObservationPointCloud::ExternalStorageFormat::PlainTextFile);

		cl.def_readwrite("pointcloud", &mrpt::obs::CObservationPointCloud::pointcloud);
		cl.def_readwrite("sensorPose", &mrpt::obs::CObservationPointCloud::sensorPose);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::obs::CObservationPointCloud::GetRuntimeClassIdStatic, "C++: mrpt::obs::CObservationPointCloud::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::obs::CObservationPointCloud::*)() const) &mrpt::obs::CObservationPointCloud::GetRuntimeClass, "C++: mrpt::obs::CObservationPointCloud::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::obs::CObservationPointCloud::*)() const) &mrpt::obs::CObservationPointCloud::clone, "C++: mrpt::obs::CObservationPointCloud::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::obs::CObservationPointCloud::CreateObject, "C++: mrpt::obs::CObservationPointCloud::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("getSensorPose", (void (mrpt::obs::CObservationPointCloud::*)(class mrpt::poses::CPose3D &) const) &mrpt::obs::CObservationPointCloud::getSensorPose, "C++: mrpt::obs::CObservationPointCloud::getSensorPose(class mrpt::poses::CPose3D &) const --> void", pybind11::arg("out_sensorPose"));
		cl.def("setSensorPose", (void (mrpt::obs::CObservationPointCloud::*)(const class mrpt::poses::CPose3D &)) &mrpt::obs::CObservationPointCloud::setSensorPose, "C++: mrpt::obs::CObservationPointCloud::setSensorPose(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("p"));
		cl.def("load_impl", (void (mrpt::obs::CObservationPointCloud::*)() const) &mrpt::obs::CObservationPointCloud::load_impl, "@{ \n\nC++: mrpt::obs::CObservationPointCloud::load_impl() const --> void");
		cl.def("unload", (void (mrpt::obs::CObservationPointCloud::*)() const) &mrpt::obs::CObservationPointCloud::unload, "C++: mrpt::obs::CObservationPointCloud::unload() const --> void");
		cl.def("isExternallyStored", (bool (mrpt::obs::CObservationPointCloud::*)() const) &mrpt::obs::CObservationPointCloud::isExternallyStored, "@{ \n\nC++: mrpt::obs::CObservationPointCloud::isExternallyStored() const --> bool");
		cl.def("getExternalStorageFile", (const std::string & (mrpt::obs::CObservationPointCloud::*)() const) &mrpt::obs::CObservationPointCloud::getExternalStorageFile, "C++: mrpt::obs::CObservationPointCloud::getExternalStorageFile() const --> const std::string &", pybind11::return_value_policy::automatic);
		cl.def("setAsExternalStorage", (void (mrpt::obs::CObservationPointCloud::*)(const std::string &, const enum mrpt::obs::CObservationPointCloud::ExternalStorageFormat)) &mrpt::obs::CObservationPointCloud::setAsExternalStorage, "C++: mrpt::obs::CObservationPointCloud::setAsExternalStorage(const std::string &, const enum mrpt::obs::CObservationPointCloud::ExternalStorageFormat) --> void", pybind11::arg("fileName"), pybind11::arg("fmt"));
		cl.def("overrideExternalStorageFormatFlag", (void (mrpt::obs::CObservationPointCloud::*)(const enum mrpt::obs::CObservationPointCloud::ExternalStorageFormat)) &mrpt::obs::CObservationPointCloud::overrideExternalStorageFormatFlag, "C++: mrpt::obs::CObservationPointCloud::overrideExternalStorageFormatFlag(const enum mrpt::obs::CObservationPointCloud::ExternalStorageFormat) --> void", pybind11::arg("fmt"));
		cl.def("assign", (class mrpt::obs::CObservationPointCloud & (mrpt::obs::CObservationPointCloud::*)(const class mrpt::obs::CObservationPointCloud &)) &mrpt::obs::CObservationPointCloud::operator=, "C++: mrpt::obs::CObservationPointCloud::operator=(const class mrpt::obs::CObservationPointCloud &) --> class mrpt::obs::CObservationPointCloud &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::obs::CObservationRotatingScan file:mrpt/obs/CObservationRotatingScan.h line:53
		pybind11::class_<mrpt::obs::CObservationRotatingScan, std::shared_ptr<mrpt::obs::CObservationRotatingScan>, PyCallBack_mrpt_obs_CObservationRotatingScan, mrpt::obs::CObservation> cl(M("mrpt::obs"), "CObservationRotatingScan", "A `CObservation`-derived class for raw range data from a 2D or 3D\n rotating scanner. This class is the preferred alternative to\n CObservationVelodyneScan and CObservation2DRangeScan in MRPT 2.x, since it\n exposes range data as an organized matrix, more convenient for feature\n detection directly on \"range images\" and on points stored as a matrix in the\n member organizedPoints.\n\n Check out the main data fields in the list of members below.\n\n  Note that this object has  timestamp fields:\n  - The standard `CObservation::timestamp` field in the base class, which\n should contain the accurate satellite-based UTC timestamp if available,\n and\n  - the field originalReceivedTimestamp, with the\n local computer-based timestamp based on the reception of the message in\n the computer.\n\n Both timestamps correspond to the firing of the **first** laser in\n the scan, i.e. the first column in organizedPoints.\n\n The reference frame for the 3D LIDAR is with +X pointing forward, +Z up.\n\n \n New in MRPT 2.0.0\n \n\n CObservation, mrpt::hwdrivers::CVelodyneScanner");
		cl.def( pybind11::init( [](){ return new mrpt::obs::CObservationRotatingScan(); }, [](){ return new PyCallBack_mrpt_obs_CObservationRotatingScan(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_CObservationRotatingScan const &o){ return new PyCallBack_mrpt_obs_CObservationRotatingScan(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::CObservationRotatingScan const &o){ return new mrpt::obs::CObservationRotatingScan(o); } ) );

		pybind11::enum_<mrpt::obs::CObservationRotatingScan::ExternalStorageFormat>(cl, "ExternalStorageFormat", "")
			.value("None", mrpt::obs::CObservationRotatingScan::ExternalStorageFormat::None)
			.value("MRPT_Serialization", mrpt::obs::CObservationRotatingScan::ExternalStorageFormat::MRPT_Serialization)
			.value("PlainTextFile", mrpt::obs::CObservationRotatingScan::ExternalStorageFormat::PlainTextFile);

		cl.def_readwrite("rowCount", &mrpt::obs::CObservationRotatingScan::rowCount);
		cl.def_readwrite("columnCount", &mrpt::obs::CObservationRotatingScan::columnCount);
		cl.def_readwrite("rangeImage", &mrpt::obs::CObservationRotatingScan::rangeImage);
		cl.def_readwrite("organizedPoints", &mrpt::obs::CObservationRotatingScan::organizedPoints);
		cl.def_readwrite("intensityImage", &mrpt::obs::CObservationRotatingScan::intensityImage);
		cl.def_readwrite("rangeOtherLayers", &mrpt::obs::CObservationRotatingScan::rangeOtherLayers);
		cl.def_readwrite("rangeResolution", &mrpt::obs::CObservationRotatingScan::rangeResolution);
		cl.def_readwrite("startAzimuth", &mrpt::obs::CObservationRotatingScan::startAzimuth);
		cl.def_readwrite("azimuthSpan", &mrpt::obs::CObservationRotatingScan::azimuthSpan);
		cl.def_readwrite("sweepDuration", &mrpt::obs::CObservationRotatingScan::sweepDuration);
		cl.def_readwrite("lidarModel", &mrpt::obs::CObservationRotatingScan::lidarModel);
		cl.def_readwrite("minRange", &mrpt::obs::CObservationRotatingScan::minRange);
		cl.def_readwrite("maxRange", &mrpt::obs::CObservationRotatingScan::maxRange);
		cl.def_readwrite("sensorPose", &mrpt::obs::CObservationRotatingScan::sensorPose);
		cl.def_readwrite("originalReceivedTimestamp", &mrpt::obs::CObservationRotatingScan::originalReceivedTimestamp);
		cl.def_readwrite("has_satellite_timestamp", &mrpt::obs::CObservationRotatingScan::has_satellite_timestamp);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::obs::CObservationRotatingScan::GetRuntimeClassIdStatic, "C++: mrpt::obs::CObservationRotatingScan::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::obs::CObservationRotatingScan::*)() const) &mrpt::obs::CObservationRotatingScan::GetRuntimeClass, "C++: mrpt::obs::CObservationRotatingScan::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::obs::CObservationRotatingScan::*)() const) &mrpt::obs::CObservationRotatingScan::clone, "C++: mrpt::obs::CObservationRotatingScan::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::obs::CObservationRotatingScan::CreateObject, "C++: mrpt::obs::CObservationRotatingScan::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("fromVelodyne", (void (mrpt::obs::CObservationRotatingScan::*)(const class mrpt::obs::CObservationVelodyneScan &)) &mrpt::obs::CObservationRotatingScan::fromVelodyne, "@{ \n\nC++: mrpt::obs::CObservationRotatingScan::fromVelodyne(const class mrpt::obs::CObservationVelodyneScan &) --> void", pybind11::arg("o"));
		cl.def("fromScan2D", (void (mrpt::obs::CObservationRotatingScan::*)(const class mrpt::obs::CObservation2DRangeScan &)) &mrpt::obs::CObservationRotatingScan::fromScan2D, "C++: mrpt::obs::CObservationRotatingScan::fromScan2D(const class mrpt::obs::CObservation2DRangeScan &) --> void", pybind11::arg("o"));
		cl.def("fromGeneric", (bool (mrpt::obs::CObservationRotatingScan::*)(const class mrpt::obs::CObservation &)) &mrpt::obs::CObservationRotatingScan::fromGeneric, "Will convert from another observation if it's any of the supported\n source types (see fromVelodyne(), fromScan2D()) and\n return true, or will return false otherwise if there is no known way to\n convert from the passed object. \n\nC++: mrpt::obs::CObservationRotatingScan::fromGeneric(const class mrpt::obs::CObservation &) --> bool", pybind11::arg("o"));
		cl.def("load_impl", (void (mrpt::obs::CObservationRotatingScan::*)() const) &mrpt::obs::CObservationRotatingScan::load_impl, "@{ \n\nC++: mrpt::obs::CObservationRotatingScan::load_impl() const --> void");
		cl.def("unload", (void (mrpt::obs::CObservationRotatingScan::*)() const) &mrpt::obs::CObservationRotatingScan::unload, "C++: mrpt::obs::CObservationRotatingScan::unload() const --> void");
		cl.def("isExternallyStored", (bool (mrpt::obs::CObservationRotatingScan::*)() const) &mrpt::obs::CObservationRotatingScan::isExternallyStored, "@{ \n\nC++: mrpt::obs::CObservationRotatingScan::isExternallyStored() const --> bool");
		cl.def("getExternalStorageFile", (const std::string & (mrpt::obs::CObservationRotatingScan::*)() const) &mrpt::obs::CObservationRotatingScan::getExternalStorageFile, "C++: mrpt::obs::CObservationRotatingScan::getExternalStorageFile() const --> const std::string &", pybind11::return_value_policy::automatic);
		cl.def("setAsExternalStorage", (void (mrpt::obs::CObservationRotatingScan::*)(const std::string &, const enum mrpt::obs::CObservationRotatingScan::ExternalStorageFormat)) &mrpt::obs::CObservationRotatingScan::setAsExternalStorage, "C++: mrpt::obs::CObservationRotatingScan::setAsExternalStorage(const std::string &, const enum mrpt::obs::CObservationRotatingScan::ExternalStorageFormat) --> void", pybind11::arg("fileName"), pybind11::arg("fmt"));
		cl.def("overrideExternalStorageFormatFlag", (void (mrpt::obs::CObservationRotatingScan::*)(const enum mrpt::obs::CObservationRotatingScan::ExternalStorageFormat)) &mrpt::obs::CObservationRotatingScan::overrideExternalStorageFormatFlag, "C++: mrpt::obs::CObservationRotatingScan::overrideExternalStorageFormatFlag(const enum mrpt::obs::CObservationRotatingScan::ExternalStorageFormat) --> void", pybind11::arg("fmt"));
		cl.def("saveToTextFile", (bool (mrpt::obs::CObservationRotatingScan::*)(const std::string &) const) &mrpt::obs::CObservationRotatingScan::saveToTextFile, "Write scan data to a plain text, each line has:\n   `x y z range intensity row_idx col_idx`\n\n For each point in the organized point cloud.\n Invalid points (e.g. no lidar return) are stored as (x,y,z)=(0,0,0) and\n range=0.\n\n \n true on success\n\nC++: mrpt::obs::CObservationRotatingScan::saveToTextFile(const std::string &) const --> bool", pybind11::arg("filename"));
		cl.def("loadFromTextFile", (bool (mrpt::obs::CObservationRotatingScan::*)(const std::string &)) &mrpt::obs::CObservationRotatingScan::loadFromTextFile, "Loads the range, intensity, and organizedPoints members from a plain\n text file in the format describd in saveToTextFile()\n\nC++: mrpt::obs::CObservationRotatingScan::loadFromTextFile(const std::string &) --> bool", pybind11::arg("filename"));
		cl.def("getOriginalReceivedTimeStamp", (mrpt::Clock::time_point (mrpt::obs::CObservationRotatingScan::*)() const) &mrpt::obs::CObservationRotatingScan::getOriginalReceivedTimeStamp, "@} \n\nC++: mrpt::obs::CObservationRotatingScan::getOriginalReceivedTimeStamp() const --> mrpt::Clock::time_point");
		cl.def("getSensorPose", (void (mrpt::obs::CObservationRotatingScan::*)(class mrpt::poses::CPose3D &) const) &mrpt::obs::CObservationRotatingScan::getSensorPose, "C++: mrpt::obs::CObservationRotatingScan::getSensorPose(class mrpt::poses::CPose3D &) const --> void", pybind11::arg("out_sensorPose"));
		cl.def("setSensorPose", (void (mrpt::obs::CObservationRotatingScan::*)(const class mrpt::poses::CPose3D &)) &mrpt::obs::CObservationRotatingScan::setSensorPose, "C++: mrpt::obs::CObservationRotatingScan::setSensorPose(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newSensorPose"));
		cl.def("assign", (class mrpt::obs::CObservationRotatingScan & (mrpt::obs::CObservationRotatingScan::*)(const class mrpt::obs::CObservationRotatingScan &)) &mrpt::obs::CObservationRotatingScan::operator=, "C++: mrpt::obs::CObservationRotatingScan::operator=(const class mrpt::obs::CObservationRotatingScan &) --> class mrpt::obs::CObservationRotatingScan &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
