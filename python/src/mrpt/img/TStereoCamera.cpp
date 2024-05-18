#include <iterator>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/img/TStereoCamera.h>
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

// mrpt::img::TStereoCamera file:mrpt/img/TStereoCamera.h line:23
struct PyCallBack_mrpt_img_TStereoCamera : public mrpt::img::TStereoCamera {
	using mrpt::img::TStereoCamera::TStereoCamera;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::img::TStereoCamera *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return TStereoCamera::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::img::TStereoCamera *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return TStereoCamera::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::img::TStereoCamera *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return TStereoCamera::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::img::TStereoCamera *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TStereoCamera::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::img::TStereoCamera *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TStereoCamera::serializeFrom(a0, a1);
	}
};

void bind_mrpt_img_TStereoCamera(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::img::TStereoCamera file:mrpt/img/TStereoCamera.h line:23
		pybind11::class_<mrpt::img::TStereoCamera, std::shared_ptr<mrpt::img::TStereoCamera>, PyCallBack_mrpt_img_TStereoCamera, mrpt::serialization::CSerializable> cl(M("mrpt::img"), "TStereoCamera", "Structure to hold the parameters of a pinhole stereo camera model.\n  The parameters obtained for one camera resolution can be used for any other\n resolution by means of the method TStereoCamera::scaleToResolution()\n\n \n mrpt::vision, the application stereo-calib-gui for calibrating a stereo\n camera");
		cl.def( pybind11::init( [](PyCallBack_mrpt_img_TStereoCamera const &o){ return new PyCallBack_mrpt_img_TStereoCamera(o); } ) );
		cl.def( pybind11::init( [](mrpt::img::TStereoCamera const &o){ return new mrpt::img::TStereoCamera(o); } ) );
		cl.def( pybind11::init( [](){ return new mrpt::img::TStereoCamera(); }, [](){ return new PyCallBack_mrpt_img_TStereoCamera(); } ) );
		cl.def_readwrite("leftCamera", &mrpt::img::TStereoCamera::leftCamera);
		cl.def_readwrite("rightCamera", &mrpt::img::TStereoCamera::rightCamera);
		cl.def_readwrite("rightCameraPose", &mrpt::img::TStereoCamera::rightCameraPose);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::img::TStereoCamera::GetRuntimeClassIdStatic, "C++: mrpt::img::TStereoCamera::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::img::TStereoCamera::*)() const) &mrpt::img::TStereoCamera::GetRuntimeClass, "C++: mrpt::img::TStereoCamera::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::img::TStereoCamera::*)() const) &mrpt::img::TStereoCamera::clone, "C++: mrpt::img::TStereoCamera::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::img::TStereoCamera::CreateObject, "C++: mrpt::img::TStereoCamera::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("saveToConfigFile", (void (mrpt::img::TStereoCamera::*)(const std::string &, class mrpt::config::CConfigFileBase &) const) &mrpt::img::TStereoCamera::saveToConfigFile, "Save all params to a plain text config file in this format:\n  \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n   Notice that 3 different sections are read, of which \"section\" is only\n the prefix.\n\nC++: mrpt::img::TStereoCamera::saveToConfigFile(const std::string &, class mrpt::config::CConfigFileBase &) const --> void", pybind11::arg("section"), pybind11::arg("cfg"));
		cl.def("loadFromConfigFile", (void (mrpt::img::TStereoCamera::*)(const std::string &, const class mrpt::config::CConfigFileBase &)) &mrpt::img::TStereoCamera::loadFromConfigFile, "Load all the params from a config source, in the same format that used\n in saveToConfigFile().\n   Notice that 3 different sections are read, of which \"section\" is only\n the prefix.\n  \n\n std::exception on missing fields\n\nC++: mrpt::img::TStereoCamera::loadFromConfigFile(const std::string &, const class mrpt::config::CConfigFileBase &) --> void", pybind11::arg("section"), pybind11::arg("cfg"));
		cl.def("loadFromConfigFile", (void (mrpt::img::TStereoCamera::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::img::TStereoCamera::loadFromConfigFile, "overload This signature is consistent with the rest of MRPT APIs \n\nC++: mrpt::img::TStereoCamera::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("cfg"), pybind11::arg("section"));
		cl.def("dumpAsText", (std::string (mrpt::img::TStereoCamera::*)() const) &mrpt::img::TStereoCamera::dumpAsText, "Dumps all the parameters as a multi-line string, with the same format\n than   \n\n saveToConfigFile \n\nC++: mrpt::img::TStereoCamera::dumpAsText() const --> std::string");
		cl.def("scaleToResolution", (void (mrpt::img::TStereoCamera::*)(unsigned int, unsigned int)) &mrpt::img::TStereoCamera::scaleToResolution, "Rescale all the parameters for a new camera resolution (it raises an\n exception if the aspect ratio is modified, which is not permitted).\n\nC++: mrpt::img::TStereoCamera::scaleToResolution(unsigned int, unsigned int) --> void", pybind11::arg("new_ncols"), pybind11::arg("new_nrows"));
		cl.def("assign", (class mrpt::img::TStereoCamera & (mrpt::img::TStereoCamera::*)(const class mrpt::img::TStereoCamera &)) &mrpt::img::TStereoCamera::operator=, "C++: mrpt::img::TStereoCamera::operator=(const class mrpt::img::TStereoCamera &) --> class mrpt::img::TStereoCamera &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
