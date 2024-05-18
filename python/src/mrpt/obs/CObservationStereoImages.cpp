#include <chrono>
#include <ios>
#include <iterator>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/core/Clock.h>
#include <mrpt/img/TStereoCamera.h>
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
#include <mrpt/obs/CObservationStereoImages.h>
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

// mrpt::obs::CObservationStereoImages file:mrpt/obs/CObservationStereoImages.h line:38
struct PyCallBack_mrpt_obs_CObservationStereoImages : public mrpt::obs::CObservationStereoImages {
	using mrpt::obs::CObservationStereoImages::CObservationStereoImages;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationStereoImages *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CObservationStereoImages::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationStereoImages *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CObservationStereoImages::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationStereoImages *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CObservationStereoImages::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationStereoImages *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationStereoImages::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationStereoImages *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationStereoImages::serializeFrom(a0, a1);
	}
	void getSensorPose(class mrpt::poses::CPose3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationStereoImages *>(this), "getSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationStereoImages::getSensorPose(a0);
	}
	void setSensorPose(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationStereoImages *>(this), "setSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationStereoImages::setSensorPose(a0);
	}
	void load_impl() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationStereoImages *>(this), "load_impl");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationStereoImages::load_impl();
	}
	using _binder_ret_0 = mrpt::Clock::time_point;
	_binder_ret_0 getOriginalReceivedTimeStamp() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationStereoImages *>(this), "getOriginalReceivedTimeStamp");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationStereoImages *>(this), "asString");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationStereoImages *>(this), "exportTxtSupported");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationStereoImages *>(this), "exportTxtHeader");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationStereoImages *>(this), "exportTxtDataRow");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationStereoImages *>(this), "unload");
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
};

void bind_mrpt_obs_CObservationStereoImages(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::obs::CObservationStereoImages file:mrpt/obs/CObservationStereoImages.h line:38
		pybind11::class_<mrpt::obs::CObservationStereoImages, std::shared_ptr<mrpt::obs::CObservationStereoImages>, PyCallBack_mrpt_obs_CObservationStereoImages, mrpt::obs::CObservation> cl(M("mrpt::obs"), "CObservationStereoImages", "Observation class for either a pair of left+right or left+disparity images\nfrom a stereo camera.\n\n  To find whether the observation contains a right image and/or a disparity\nimage, see the fields hasImageDisparity and hasImageRight, respectively.\n   This figure illustrates the coordinate frames involved in this class:\n\n	 <center>\n   \n  </center>\n\n \n The images stored in this class can be raw or undistorted images. In\nthe latter case, the \"distortion\" params of the corresponding \"leftCamera\" and\n\"rightCamera\" fields should be all zeros.\n \n\n CObservation\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::obs::CObservationStereoImages(); }, [](){ return new PyCallBack_mrpt_obs_CObservationStereoImages(); } ) );
		cl.def_readwrite("imageLeft", &mrpt::obs::CObservationStereoImages::imageLeft);
		cl.def_readwrite("imageRight", &mrpt::obs::CObservationStereoImages::imageRight);
		cl.def_readwrite("imageDisparity", &mrpt::obs::CObservationStereoImages::imageDisparity);
		cl.def_readwrite("hasImageDisparity", &mrpt::obs::CObservationStereoImages::hasImageDisparity);
		cl.def_readwrite("hasImageRight", &mrpt::obs::CObservationStereoImages::hasImageRight);
		cl.def_readwrite("leftCamera", &mrpt::obs::CObservationStereoImages::leftCamera);
		cl.def_readwrite("rightCamera", &mrpt::obs::CObservationStereoImages::rightCamera);
		cl.def_readwrite("cameraPose", &mrpt::obs::CObservationStereoImages::cameraPose);
		cl.def_readwrite("rightCameraPose", &mrpt::obs::CObservationStereoImages::rightCameraPose);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::obs::CObservationStereoImages::GetRuntimeClassIdStatic, "C++: mrpt::obs::CObservationStereoImages::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::obs::CObservationStereoImages::*)() const) &mrpt::obs::CObservationStereoImages::GetRuntimeClass, "C++: mrpt::obs::CObservationStereoImages::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::obs::CObservationStereoImages::*)() const) &mrpt::obs::CObservationStereoImages::clone, "C++: mrpt::obs::CObservationStereoImages::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::obs::CObservationStereoImages::CreateObject, "C++: mrpt::obs::CObservationStereoImages::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("getStereoCameraParams", (void (mrpt::obs::CObservationStereoImages::*)(class mrpt::img::TStereoCamera &) const) &mrpt::obs::CObservationStereoImages::getStereoCameraParams, "Populates a TStereoCamera structure with the parameters in \n  and  \n\n\n areImagesRectified() \n\nC++: mrpt::obs::CObservationStereoImages::getStereoCameraParams(class mrpt::img::TStereoCamera &) const --> void", pybind11::arg("out_params"));
		cl.def("getStereoCameraParams", (class mrpt::img::TStereoCamera (mrpt::obs::CObservationStereoImages::*)() const) &mrpt::obs::CObservationStereoImages::getStereoCameraParams, "C++: mrpt::obs::CObservationStereoImages::getStereoCameraParams() const --> class mrpt::img::TStereoCamera");
		cl.def("setStereoCameraParams", (void (mrpt::obs::CObservationStereoImages::*)(const class mrpt::img::TStereoCamera &)) &mrpt::obs::CObservationStereoImages::setStereoCameraParams, "Sets   and  from a\n TStereoCamera structure \n\nC++: mrpt::obs::CObservationStereoImages::setStereoCameraParams(const class mrpt::img::TStereoCamera &) --> void", pybind11::arg("in_params"));
		cl.def("areImagesRectified", (bool (mrpt::obs::CObservationStereoImages::*)() const) &mrpt::obs::CObservationStereoImages::areImagesRectified, "This method only checks whether ALL the distortion parameters in \n are set to zero, which is\n the convention in MRPT to denote that this pair of stereo images has\n been rectified.\n\nC++: mrpt::obs::CObservationStereoImages::areImagesRectified() const --> bool");
		cl.def("getSensorPose", (void (mrpt::obs::CObservationStereoImages::*)(class mrpt::poses::CPose3D &) const) &mrpt::obs::CObservationStereoImages::getSensorPose, "@} \n\nC++: mrpt::obs::CObservationStereoImages::getSensorPose(class mrpt::poses::CPose3D &) const --> void", pybind11::arg("out_sensorPose"));
		cl.def("setSensorPose", (void (mrpt::obs::CObservationStereoImages::*)(const class mrpt::poses::CPose3D &)) &mrpt::obs::CObservationStereoImages::setSensorPose, "C++: mrpt::obs::CObservationStereoImages::setSensorPose(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newSensorPose"));
		cl.def("swap", (void (mrpt::obs::CObservationStereoImages::*)(class mrpt::obs::CObservationStereoImages &)) &mrpt::obs::CObservationStereoImages::swap, "Do an efficient swap of all data members of this object with \"o\". \n\nC++: mrpt::obs::CObservationStereoImages::swap(class mrpt::obs::CObservationStereoImages &) --> void", pybind11::arg("o"));
		cl.def("load_impl", (void (mrpt::obs::CObservationStereoImages::*)() const) &mrpt::obs::CObservationStereoImages::load_impl, "C++: mrpt::obs::CObservationStereoImages::load_impl() const --> void");
		cl.def("assign", (class mrpt::obs::CObservationStereoImages & (mrpt::obs::CObservationStereoImages::*)(const class mrpt::obs::CObservationStereoImages &)) &mrpt::obs::CObservationStereoImages::operator=, "C++: mrpt::obs::CObservationStereoImages::operator=(const class mrpt::obs::CObservationStereoImages &) --> class mrpt::obs::CObservationStereoImages &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
