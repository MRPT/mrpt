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
#include <mrpt/obs/CObservationRGBD360.h>
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

// mrpt::obs::CObservationRGBD360 file:mrpt/obs/CObservationRGBD360.h line:83
struct PyCallBack_mrpt_obs_CObservationRGBD360 : public mrpt::obs::CObservationRGBD360 {
	using mrpt::obs::CObservationRGBD360::CObservationRGBD360;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRGBD360 *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CObservationRGBD360::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRGBD360 *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CObservationRGBD360::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRGBD360 *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CObservationRGBD360::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRGBD360 *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationRGBD360::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRGBD360 *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationRGBD360::serializeFrom(a0, a1);
	}
	void getSensorPose(class mrpt::poses::CPose3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRGBD360 *>(this), "getSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationRGBD360::getSensorPose(a0);
	}
	void setSensorPose(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRGBD360 *>(this), "setSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationRGBD360::setSensorPose(a0);
	}
	using _binder_ret_0 = mrpt::Clock::time_point;
	_binder_ret_0 getOriginalReceivedTimeStamp() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRGBD360 *>(this), "getOriginalReceivedTimeStamp");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRGBD360 *>(this), "asString");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRGBD360 *>(this), "exportTxtSupported");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRGBD360 *>(this), "exportTxtHeader");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRGBD360 *>(this), "exportTxtDataRow");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRGBD360 *>(this), "unload");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRGBD360 *>(this), "load_impl");
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

void bind_mrpt_obs_CObservationRGBD360(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::obs::CObservationRGBD360 file:mrpt/obs/CObservationRGBD360.h line:83
		pybind11::class_<mrpt::obs::CObservationRGBD360, std::shared_ptr<mrpt::obs::CObservationRGBD360>, PyCallBack_mrpt_obs_CObservationRGBD360, mrpt::obs::CObservation> cl(M("mrpt::obs"), "CObservationRGBD360", "Declares a class derived from \"CObservation\" that\n      encapsules an omnidirectional RGBD measurement from a set of RGBD\nsensors.\n  This kind of observations can carry one or more of these data fields:\n    - 3D point cloud (as float's).\n    - 2D range image (as a matrix): Each entry in the matrix\n\"rangeImage(ROW,COLUMN)\" contains a distance or a depth (in meters), depending\non \n    - 2D intensity (grayscale or RGB) image (as a mrpt::img::CImage): For\nSwissRanger cameras, a logarithmic A-law compression is used to convert the\noriginal 16bit intensity to a more standard 8bit graylevel.\n\n  The coordinates of the 3D point cloud are in millimeters with respect to the\nRGB camera origin of coordinates\n\n  The 2D images and matrices are stored as common images, with an up->down\nrows order and left->right, as usual.\n   Optionally, the intensity and confidence channels can be set to\ndelayed-load images for off-rawlog storage so it saves\n   memory by having loaded in memory just the needed images. See the methods\nload() and unload().\n  Due to the intensive storage requirements of this kind of observations, this\nobservation is the only one in MRPT\n   for which it's recommended to always call \"load()\" and \"unload()\" before\nand after using the observation, *ONLY* when\n   the observation was read from a rawlog dataset, in order to make sure that\nall the externally stored data fields are\n   loaded and ready in memory.\n\n  Classes that grab observations of this type are:\n		- mrpt::hwdrivers::CSwissRanger3DCamera\n		- mrpt::hwdrivers::CKinect\n\n  3D point clouds can be generated at any moment after grabbing with\nCObservationRGBD360::unprojectInto() and\nCObservationRGBD360::unprojectInto(), provided the correct\n   calibration parameters.\n\n  \n Starting at serialization version 3 (MRPT 0.9.1+), the 3D point cloud\nand the rangeImage can both be stored externally to save rawlog space.\n  \n\n Starting at serialization version 5 (MRPT 0.9.5+), the new field \n  \n\n Starting at serialization version 6 (MRPT 0.9.5+), the new field \n\n \n mrpt::hwdrivers::CSwissRanger3DCamera, mrpt::hwdrivers::COpenNI2_RGBD360,\nCObservation\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::obs::CObservationRGBD360(); }, [](){ return new PyCallBack_mrpt_obs_CObservationRGBD360(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_CObservationRGBD360 const &o){ return new PyCallBack_mrpt_obs_CObservationRGBD360(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::CObservationRGBD360 const &o){ return new mrpt::obs::CObservationRGBD360(o); } ) );
		cl.def_readwrite("hasRangeImage", &mrpt::obs::CObservationRGBD360::hasRangeImage);
		cl.def_readwrite("rangeUnits", &mrpt::obs::CObservationRGBD360::rangeUnits);
		cl.def_readwrite("hasIntensityImage", &mrpt::obs::CObservationRGBD360::hasIntensityImage);
		cl.def_readwrite("maxRange", &mrpt::obs::CObservationRGBD360::maxRange);
		cl.def_readwrite("sensorPose", &mrpt::obs::CObservationRGBD360::sensorPose);
		cl.def_readwrite("stdError", &mrpt::obs::CObservationRGBD360::stdError);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::obs::CObservationRGBD360::GetRuntimeClassIdStatic, "C++: mrpt::obs::CObservationRGBD360::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::obs::CObservationRGBD360::*)() const) &mrpt::obs::CObservationRGBD360::GetRuntimeClass, "C++: mrpt::obs::CObservationRGBD360::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::obs::CObservationRGBD360::*)() const) &mrpt::obs::CObservationRGBD360::clone, "C++: mrpt::obs::CObservationRGBD360::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::obs::CObservationRGBD360::CreateObject, "C++: mrpt::obs::CObservationRGBD360::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("rangeImage_setSize", (void (mrpt::obs::CObservationRGBD360::*)(const int, const int, const unsigned int)) &mrpt::obs::CObservationRGBD360::rangeImage_setSize, "Similar to calling \"rangeImage.setSize(H,W)\" but this method provides\n memory pooling to speed-up the memory allocation. \n\nC++: mrpt::obs::CObservationRGBD360::rangeImage_setSize(const int, const int, const unsigned int) --> void", pybind11::arg("HEIGHT"), pybind11::arg("WIDTH"), pybind11::arg("sensor_id"));
		cl.def("getSensorPose", (void (mrpt::obs::CObservationRGBD360::*)(class mrpt::poses::CPose3D &) const) &mrpt::obs::CObservationRGBD360::getSensorPose, "C++: mrpt::obs::CObservationRGBD360::getSensorPose(class mrpt::poses::CPose3D &) const --> void", pybind11::arg("out_sensorPose"));
		cl.def("setSensorPose", (void (mrpt::obs::CObservationRGBD360::*)(const class mrpt::poses::CPose3D &)) &mrpt::obs::CObservationRGBD360::setSensorPose, "C++: mrpt::obs::CObservationRGBD360::setSensorPose(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newSensorPose"));
		cl.def("assign", (class mrpt::obs::CObservationRGBD360 & (mrpt::obs::CObservationRGBD360::*)(const class mrpt::obs::CObservationRGBD360 &)) &mrpt::obs::CObservationRGBD360::operator=, "C++: mrpt::obs::CObservationRGBD360::operator=(const class mrpt::obs::CObservationRGBD360 &) --> class mrpt::obs::CObservationRGBD360 &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
