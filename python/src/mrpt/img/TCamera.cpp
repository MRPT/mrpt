#include <any>
#include <functional>
#include <istream>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/containers/CommentPosition.h>
#include <mrpt/containers/YamlEmitOptions.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/typemeta/static_string.h>
#include <ostream>
#include <sstream> // __str__
#include <string>
#include <string_view>
#include <type_traits>
#include <typeinfo>
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

// mrpt::img::TCamera file:mrpt/img/TCamera.h line:33
struct PyCallBack_mrpt_img_TCamera : public mrpt::img::TCamera {
	using mrpt::img::TCamera::TCamera;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::img::TCamera *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return TCamera::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::img::TCamera *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return TCamera::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::img::TCamera *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return TCamera::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::img::TCamera *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TCamera::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::img::TCamera *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TCamera::serializeFrom(a0, a1);
	}
};

void bind_mrpt_img_TCamera(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::img::TCamera file:mrpt/img/TCamera.h line:33
		pybind11::class_<mrpt::img::TCamera, std::shared_ptr<mrpt::img::TCamera>, PyCallBack_mrpt_img_TCamera, mrpt::serialization::CSerializable> cl(M("mrpt::img"), "TCamera", "Intrinsic parameters for a pinhole or fisheye camera model, plus its\n  associated lens distortion model. The type of camera distortion is defined\n  by the  field.\n\n Parameters for one camera resolution can be used for any other resolutions by\n means of the method TCamera::scaleToResolution()\n\n \n The application camera-calib-gui for calibrating a camera\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::img::TCamera(); }, [](){ return new PyCallBack_mrpt_img_TCamera(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_img_TCamera const &o){ return new PyCallBack_mrpt_img_TCamera(o); } ) );
		cl.def( pybind11::init( [](mrpt::img::TCamera const &o){ return new mrpt::img::TCamera(o); } ) );
		cl.def_readwrite("ncols", &mrpt::img::TCamera::ncols);
		cl.def_readwrite("nrows", &mrpt::img::TCamera::nrows);
		cl.def_readwrite("intrinsicParams", &mrpt::img::TCamera::intrinsicParams);
		cl.def_readwrite("distortion", &mrpt::img::TCamera::distortion);
		cl.def_readwrite("dist", &mrpt::img::TCamera::dist);
		cl.def_readwrite("focalLengthMeters", &mrpt::img::TCamera::focalLengthMeters);
		cl.def_readwrite("cameraName", &mrpt::img::TCamera::cameraName);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::img::TCamera::GetRuntimeClassIdStatic, "C++: mrpt::img::TCamera::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::img::TCamera::*)() const) &mrpt::img::TCamera::GetRuntimeClass, "C++: mrpt::img::TCamera::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::img::TCamera::*)() const) &mrpt::img::TCamera::clone, "C++: mrpt::img::TCamera::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::img::TCamera::CreateObject, "C++: mrpt::img::TCamera::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def_static("FromYAML", (class mrpt::img::TCamera (*)(const class mrpt::containers::yaml &)) &mrpt::img::TCamera::FromYAML, "Parse from yaml, in OpenCV calibration model.\n Refer to\n [this example](https://wiki.ros.org/camera_calibration_parsers#YAML).\n\n For known distortion models see mrpt::img::DistortionModel\n\n   \n\nC++: mrpt::img::TCamera::FromYAML(const class mrpt::containers::yaml &) --> class mrpt::img::TCamera", pybind11::arg("params"));
		cl.def("asYAML", (class mrpt::containers::yaml (mrpt::img::TCamera::*)() const) &mrpt::img::TCamera::asYAML, "Stores as yaml, in OpenCV calibration model.\n Refer to\n [this example](http://wiki.ros.org/camera_calibration_parsers#YAML).\n\nC++: mrpt::img::TCamera::asYAML() const --> class mrpt::containers::yaml");
		cl.def("scaleToResolution", (void (mrpt::img::TCamera::*)(unsigned int, unsigned int)) &mrpt::img::TCamera::scaleToResolution, "Rescale all the parameters for a new camera resolution (it raises an\n exception if the aspect ratio is modified, which is not permitted).\n\nC++: mrpt::img::TCamera::scaleToResolution(unsigned int, unsigned int) --> void", pybind11::arg("new_ncols"), pybind11::arg("new_nrows"));
		cl.def("saveToConfigFile", (void (mrpt::img::TCamera::*)(const std::string &, class mrpt::config::CConfigFileBase &) const) &mrpt::img::TCamera::saveToConfigFile, "Save as a config block:\n  \n\n\n\n\n\n\n\n\n\n\n\n   \n\nC++: mrpt::img::TCamera::saveToConfigFile(const std::string &, class mrpt::config::CConfigFileBase &) const --> void", pybind11::arg("section"), pybind11::arg("cfg"));
		cl.def("loadFromConfigFile", (void (mrpt::img::TCamera::*)(const std::string &, const class mrpt::config::CConfigFileBase &)) &mrpt::img::TCamera::loadFromConfigFile, "Load all the params from a config source, in the format used in\n saveToConfigFile(), that is:\n\n  \n\n\n\n\n\n\n\n\n\n\n\n  \n std::exception on missing fields\n\nC++: mrpt::img::TCamera::loadFromConfigFile(const std::string &, const class mrpt::config::CConfigFileBase &) --> void", pybind11::arg("section"), pybind11::arg("cfg"));
		cl.def("loadFromConfigFile", (void (mrpt::img::TCamera::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::img::TCamera::loadFromConfigFile, "overload This signature is consistent with the rest of MRPT APIs \n\nC++: mrpt::img::TCamera::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("cfg"), pybind11::arg("section"));
		cl.def("dumpAsText", (std::string (mrpt::img::TCamera::*)() const) &mrpt::img::TCamera::dumpAsText, "Returns all parameters as a text block in the INI-file format.\n  \n\n asYAML()\n\nC++: mrpt::img::TCamera::dumpAsText() const --> std::string");
		cl.def("setIntrinsicParamsFromValues", (void (mrpt::img::TCamera::*)(double, double, double, double)) &mrpt::img::TCamera::setIntrinsicParamsFromValues, "Set the matrix of intrinsic params of the camera from the individual\n values of focal length and principal point coordinates (in pixels)\n\nC++: mrpt::img::TCamera::setIntrinsicParamsFromValues(double, double, double, double) --> void", pybind11::arg("fx"), pybind11::arg("fy"), pybind11::arg("cx"), pybind11::arg("cy"));
		cl.def("getDistortionParamsAsRowVector", (class mrpt::math::CMatrixDynamic<double> (mrpt::img::TCamera::*)() const) &mrpt::img::TCamera::getDistortionParamsAsRowVector, "Equivalent to getDistortionParamsAsVector()  \n\nC++: mrpt::img::TCamera::getDistortionParamsAsRowVector() const --> class mrpt::math::CMatrixDynamic<double>");
		cl.def("setDistortionPlumbBob", [](mrpt::img::TCamera &o, double const & a0, double const & a1, double const & a2, double const & a3) -> void { return o.setDistortionPlumbBob(a0, a1, a2, a3); }, "", pybind11::arg("k1_"), pybind11::arg("k2_"), pybind11::arg("p1_"), pybind11::arg("p2_"));
		cl.def("setDistortionPlumbBob", (void (mrpt::img::TCamera::*)(double, double, double, double, double)) &mrpt::img::TCamera::setDistortionPlumbBob, "Defines the distortion model and its parameters, in one call. \n\nC++: mrpt::img::TCamera::setDistortionPlumbBob(double, double, double, double, double) --> void", pybind11::arg("k1_"), pybind11::arg("k2_"), pybind11::arg("p1_"), pybind11::arg("p2_"), pybind11::arg("k3_"));
		cl.def("setDistortionKannalaBrandt", (void (mrpt::img::TCamera::*)(double, double, double, double)) &mrpt::img::TCamera::setDistortionKannalaBrandt, "Defines the distortion model and its parameters, in one call. \n\nC++: mrpt::img::TCamera::setDistortionKannalaBrandt(double, double, double, double) --> void", pybind11::arg("k1_"), pybind11::arg("k2_"), pybind11::arg("k3_"), pybind11::arg("k4_"));
		cl.def("cx", (double (mrpt::img::TCamera::*)() const) &mrpt::img::TCamera::cx, "Get the value of the principal point x-coordinate (in pixels). \n\nC++: mrpt::img::TCamera::cx() const --> double");
		cl.def("cy", (double (mrpt::img::TCamera::*)() const) &mrpt::img::TCamera::cy, "Get the value of the principal point y-coordinate  (in pixels). \n\nC++: mrpt::img::TCamera::cy() const --> double");
		cl.def("fx", (double (mrpt::img::TCamera::*)() const) &mrpt::img::TCamera::fx, "Get the value of the focal length x-value (in pixels). \n\nC++: mrpt::img::TCamera::fx() const --> double");
		cl.def("fy", (double (mrpt::img::TCamera::*)() const) &mrpt::img::TCamera::fy, "Get the value of the focal length y-value (in pixels). \n\nC++: mrpt::img::TCamera::fy() const --> double");
		cl.def("cx", (void (mrpt::img::TCamera::*)(double)) &mrpt::img::TCamera::cx, "Set the value of the principal point x-coordinate (in pixels). \n\nC++: mrpt::img::TCamera::cx(double) --> void", pybind11::arg("val"));
		cl.def("cy", (void (mrpt::img::TCamera::*)(double)) &mrpt::img::TCamera::cy, "Set the value of the principal point y-coordinate  (in pixels). \n\nC++: mrpt::img::TCamera::cy(double) --> void", pybind11::arg("val"));
		cl.def("fx", (void (mrpt::img::TCamera::*)(double)) &mrpt::img::TCamera::fx, "Set the value of the focal length x-value (in pixels). \n\nC++: mrpt::img::TCamera::fx(double) --> void", pybind11::arg("val"));
		cl.def("fy", (void (mrpt::img::TCamera::*)(double)) &mrpt::img::TCamera::fy, "Set the value of the focal length y-value (in pixels). \n\nC++: mrpt::img::TCamera::fy(double) --> void", pybind11::arg("val"));
		cl.def("k1", (double (mrpt::img::TCamera::*)() const) &mrpt::img::TCamera::k1, "Get the value of the k1 distortion parameter.  \n\nC++: mrpt::img::TCamera::k1() const --> double");
		cl.def("k2", (double (mrpt::img::TCamera::*)() const) &mrpt::img::TCamera::k2, "Get the value of the k2 distortion parameter.  \n\nC++: mrpt::img::TCamera::k2() const --> double");
		cl.def("p1", (double (mrpt::img::TCamera::*)() const) &mrpt::img::TCamera::p1, "Get the value of the p1 distortion parameter.  \n\nC++: mrpt::img::TCamera::p1() const --> double");
		cl.def("p2", (double (mrpt::img::TCamera::*)() const) &mrpt::img::TCamera::p2, "Get the value of the p2 distortion parameter.  \n\nC++: mrpt::img::TCamera::p2() const --> double");
		cl.def("k3", (double (mrpt::img::TCamera::*)() const) &mrpt::img::TCamera::k3, "Get the value of the k3 distortion parameter.  \n\nC++: mrpt::img::TCamera::k3() const --> double");
		cl.def("k4", (double (mrpt::img::TCamera::*)() const) &mrpt::img::TCamera::k4, "Get the value of the k4 distortion parameter.  \n\nC++: mrpt::img::TCamera::k4() const --> double");
		cl.def("k5", (double (mrpt::img::TCamera::*)() const) &mrpt::img::TCamera::k5, "Get the value of the k5 distortion parameter.  \n\nC++: mrpt::img::TCamera::k5() const --> double");
		cl.def("k6", (double (mrpt::img::TCamera::*)() const) &mrpt::img::TCamera::k6, "Get the value of the k6 distortion parameter.  \n\nC++: mrpt::img::TCamera::k6() const --> double");
		cl.def("k1", (void (mrpt::img::TCamera::*)(double)) &mrpt::img::TCamera::k1, "Set the value of the k1 distortion parameter.  \n\nC++: mrpt::img::TCamera::k1(double) --> void", pybind11::arg("val"));
		cl.def("k2", (void (mrpt::img::TCamera::*)(double)) &mrpt::img::TCamera::k2, "Set the value of the k2 distortion parameter.  \n\nC++: mrpt::img::TCamera::k2(double) --> void", pybind11::arg("val"));
		cl.def("p1", (void (mrpt::img::TCamera::*)(double)) &mrpt::img::TCamera::p1, "Set the value of the p1 distortion parameter.  \n\nC++: mrpt::img::TCamera::p1(double) --> void", pybind11::arg("val"));
		cl.def("p2", (void (mrpt::img::TCamera::*)(double)) &mrpt::img::TCamera::p2, "Set the value of the p2 distortion parameter.  \n\nC++: mrpt::img::TCamera::p2(double) --> void", pybind11::arg("val"));
		cl.def("k3", (void (mrpt::img::TCamera::*)(double)) &mrpt::img::TCamera::k3, "Set the value of the k3 distortion parameter.  \n\nC++: mrpt::img::TCamera::k3(double) --> void", pybind11::arg("val"));
		cl.def("k4", (void (mrpt::img::TCamera::*)(double)) &mrpt::img::TCamera::k4, "Set the value of the k4 distortion parameter.  \n\nC++: mrpt::img::TCamera::k4(double) --> void", pybind11::arg("val"));
		cl.def("k5", (void (mrpt::img::TCamera::*)(double)) &mrpt::img::TCamera::k5, "Set the value of the k5 distortion parameter.  \n\nC++: mrpt::img::TCamera::k5(double) --> void", pybind11::arg("val"));
		cl.def("k6", (void (mrpt::img::TCamera::*)(double)) &mrpt::img::TCamera::k6, "Set the value of the k6 distortion parameter.  \n\nC++: mrpt::img::TCamera::k6(double) --> void", pybind11::arg("val"));
		cl.def("assign", (class mrpt::img::TCamera & (mrpt::img::TCamera::*)(const class mrpt::img::TCamera &)) &mrpt::img::TCamera::operator=, "C++: mrpt::img::TCamera::operator=(const class mrpt::img::TCamera &) --> class mrpt::img::TCamera &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
