#include <any>
#include <functional>
#include <ios>
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
#include <mrpt/img/TColor.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TPose3DQuat.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/opengl/CCamera.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CText.h>
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
#include <sstream> // __str__
#include <streambuf>
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

// mrpt::opengl::CCamera file:mrpt/opengl/CCamera.h line:47
struct PyCallBack_mrpt_opengl_CCamera : public mrpt::opengl::CCamera {
	using mrpt::opengl::CCamera::CCamera;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CCamera *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CCamera::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CCamera *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CCamera::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CCamera *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CCamera::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CCamera *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CCamera::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CCamera *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CCamera::serializeFrom(a0, a1);
	}
	void toYAMLMap(class mrpt::containers::yaml & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CCamera *>(this), "toYAMLMap");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CCamera::toYAMLMap(a0);
	}
	void renderUpdateBuffers() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CCamera *>(this), "renderUpdateBuffers");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CCamera::renderUpdateBuffers();
	}
	struct mrpt::math::TBoundingBox_<float> internalBoundingBoxLocal() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CCamera *>(this), "internalBoundingBoxLocal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TBoundingBox_<float>>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TBoundingBox_<float>> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TBoundingBox_<float>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TBoundingBox_<float>>(std::move(o));
		}
		return CCamera::internalBoundingBoxLocal();
	}
	void freeOpenGLResources() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CCamera *>(this), "freeOpenGLResources");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CCamera::freeOpenGLResources();
	}
	class mrpt::opengl::CRenderizable & setColorA_u8(const unsigned char a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CCamera *>(this), "setColorA_u8");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::opengl::CRenderizable &>::value) {
				static pybind11::detail::override_caster_t<class mrpt::opengl::CRenderizable &> caster;
				return pybind11::detail::cast_ref<class mrpt::opengl::CRenderizable &>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::opengl::CRenderizable &>(std::move(o));
		}
		return CRenderizable::setColorA_u8(a0);
	}
	class mrpt::opengl::CRenderizable & setColor_u8(const struct mrpt::img::TColor & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CCamera *>(this), "setColor_u8");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::opengl::CRenderizable &>::value) {
				static pybind11::detail::override_caster_t<class mrpt::opengl::CRenderizable &> caster;
				return pybind11::detail::cast_ref<class mrpt::opengl::CRenderizable &>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::opengl::CRenderizable &>(std::move(o));
		}
		return CRenderizable::setColor_u8(a0);
	}
	bool cullElegible() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CCamera *>(this), "cullElegible");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CRenderizable::cullElegible();
	}
	bool isCompositeObject() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CCamera *>(this), "isCompositeObject");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CRenderizable::isCompositeObject();
	}
	bool traceRay(const class mrpt::poses::CPose3D & a0, double & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CCamera *>(this), "traceRay");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CRenderizable::traceRay(a0, a1);
	}
	struct mrpt::math::TPoint3D_<float> getLocalRepresentativePoint() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CCamera *>(this), "getLocalRepresentativePoint");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TPoint3D_<float>>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TPoint3D_<float>> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TPoint3D_<float>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TPoint3D_<float>>(std::move(o));
		}
		return CRenderizable::getLocalRepresentativePoint();
	}
	void initializeTextures() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CCamera *>(this), "initializeTextures");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRenderizable::initializeTextures();
	}
};

void bind_mrpt_opengl_CCamera(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::opengl::CCamera file:mrpt/opengl/CCamera.h line:47
		pybind11::class_<mrpt::opengl::CCamera, std::shared_ptr<mrpt::opengl::CCamera>, PyCallBack_mrpt_opengl_CCamera, mrpt::opengl::CRenderizable> cl(M("mrpt::opengl"), "CCamera", "Defines the intrinsic and extrinsic camera coordinates from which to render\n a 3D scene.\n\n  By default, each viewport has its own camera, accesible via\n  Viewport::getCamera(), but if a CCamera object is added as an object\n  to be rendered, it will override the internal viewport camera.\n\n  Available projection models:\n  - Projective model, parameterized via setProjectiveFOVdeg() (vertical field\n of view, in degrees)\n  - Projective model, by means of a computer vision pinhole intrinsic\n parameter set: see setProjectiveFromPinhole()\n  - Orthogonal projection model: use setProjectiveModel(false), or\n  setOrthogonal(), or\n  - No projection mode: use `setNoProjection()`. Viewport coordinates are\n    fixed to bottomLeft=(-1,-1) to rightTop=(+1,+1).\n\n Defining the position and orientation of a camera is possible by:\n - Using an \"orbit\" model: defined by a \"pointing to\" point, a distance to\n object, and azimuth + elevation angles; or\n - Directly giving the SE(3) camera pose, setting the set6DOFMode() to `true`\n and storing the desired pose with CRenderizable::setPose(). Pose axis\n convention is +Z pointing forwards, +X right, +Y down.\n\n  \n opengl::Scene\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::CCamera(); }, [](){ return new PyCallBack_mrpt_opengl_CCamera(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_opengl_CCamera const &o){ return new PyCallBack_mrpt_opengl_CCamera(o); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::CCamera const &o){ return new mrpt::opengl::CCamera(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CCamera::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CCamera::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CCamera::*)() const) &mrpt::opengl::CCamera::GetRuntimeClass, "C++: mrpt::opengl::CCamera::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::opengl::CCamera::*)() const) &mrpt::opengl::CCamera::clone, "C++: mrpt::opengl::CCamera::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::opengl::CCamera::CreateObject, "C++: mrpt::opengl::CCamera::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("toYAMLMap", (void (mrpt::opengl::CCamera::*)(class mrpt::containers::yaml &) const) &mrpt::opengl::CCamera::toYAMLMap, "C++: mrpt::opengl::CCamera::toYAMLMap(class mrpt::containers::yaml &) const --> void", pybind11::arg("propertiesMap"));
		cl.def("setProjectiveModel", [](mrpt::opengl::CCamera &o) -> void { return o.setProjectiveModel(); }, "");
		cl.def("setProjectiveModel", (void (mrpt::opengl::CCamera::*)(bool)) &mrpt::opengl::CCamera::setProjectiveModel, "Enable/Disable projective mode (vs. orthogonal). \n\nC++: mrpt::opengl::CCamera::setProjectiveModel(bool) --> void", pybind11::arg("v"));
		cl.def("setOrthogonal", [](mrpt::opengl::CCamera &o) -> void { return o.setOrthogonal(); }, "");
		cl.def("setOrthogonal", (void (mrpt::opengl::CCamera::*)(bool)) &mrpt::opengl::CCamera::setOrthogonal, "Enable/Disable orthogonal mode (vs. projective)\n\nC++: mrpt::opengl::CCamera::setOrthogonal(bool) --> void", pybind11::arg("v"));
		cl.def("setProjectiveFromPinhole", (void (mrpt::opengl::CCamera::*)(const class mrpt::img::TCamera &)) &mrpt::opengl::CCamera::setProjectiveFromPinhole, "C++: mrpt::opengl::CCamera::setProjectiveFromPinhole(const class mrpt::img::TCamera &) --> void", pybind11::arg("camIntrinsics"));
		cl.def("setNoProjection", (void (mrpt::opengl::CCamera::*)()) &mrpt::opengl::CCamera::setNoProjection, "Disable all coordinate transformations and allow direct use of pixel\n coordinates, that is, the projection matrix is the identity.\n\n In this mode, (-1,-1) is the bottom-left corner and (+1,+1) the top-right\n corner, per OpenGL defaults. This mode can be disabled calling\n setProjectiveModel() or setOrthogonal()\n\nC++: mrpt::opengl::CCamera::setNoProjection() --> void");
		cl.def("setProjectiveFOVdeg", (void (mrpt::opengl::CCamera::*)(float)) &mrpt::opengl::CCamera::setProjectiveFOVdeg, "Vertical field-of-View in degs, only when projectiveModel=true\n (default=30 deg).\n\nC++: mrpt::opengl::CCamera::setProjectiveFOVdeg(float) --> void", pybind11::arg("ang"));
		cl.def("getProjectiveFOVdeg", (float (mrpt::opengl::CCamera::*)() const) &mrpt::opengl::CCamera::getProjectiveFOVdeg, "Field-of-View in degs, only when projectiveModel=true (default=30 deg).\n\nC++: mrpt::opengl::CCamera::getProjectiveFOVdeg() const --> float");
		cl.def("hasPinholeModel", (bool (mrpt::opengl::CCamera::*)() const) &mrpt::opengl::CCamera::hasPinholeModel, "C++: mrpt::opengl::CCamera::hasPinholeModel() const --> bool");
		cl.def("isProjective", (bool (mrpt::opengl::CCamera::*)() const) &mrpt::opengl::CCamera::isProjective, "C++: mrpt::opengl::CCamera::isProjective() const --> bool");
		cl.def("isOrthogonal", (bool (mrpt::opengl::CCamera::*)() const) &mrpt::opengl::CCamera::isOrthogonal, "C++: mrpt::opengl::CCamera::isOrthogonal() const --> bool");
		cl.def("isNoProjection", (bool (mrpt::opengl::CCamera::*)() const) &mrpt::opengl::CCamera::isNoProjection, "C++: mrpt::opengl::CCamera::isNoProjection() const --> bool");
		cl.def("setPointingAt", (void (mrpt::opengl::CCamera::*)(float, float, float)) &mrpt::opengl::CCamera::setPointingAt, "@{ \n\nC++: mrpt::opengl::CCamera::setPointingAt(float, float, float) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("setPointingAt", (void (mrpt::opengl::CCamera::*)(const struct mrpt::math::TPoint3D_<double> &)) &mrpt::opengl::CCamera::setPointingAt, "C++: mrpt::opengl::CCamera::setPointingAt(const struct mrpt::math::TPoint3D_<double> &) --> void", pybind11::arg("p"));
		cl.def("getPointingAtX", (float (mrpt::opengl::CCamera::*)() const) &mrpt::opengl::CCamera::getPointingAtX, "C++: mrpt::opengl::CCamera::getPointingAtX() const --> float");
		cl.def("getPointingAtY", (float (mrpt::opengl::CCamera::*)() const) &mrpt::opengl::CCamera::getPointingAtY, "C++: mrpt::opengl::CCamera::getPointingAtY() const --> float");
		cl.def("getPointingAtZ", (float (mrpt::opengl::CCamera::*)() const) &mrpt::opengl::CCamera::getPointingAtZ, "C++: mrpt::opengl::CCamera::getPointingAtZ() const --> float");
		cl.def("getPointingAt", (struct mrpt::math::TPoint3D_<float> (mrpt::opengl::CCamera::*)() const) &mrpt::opengl::CCamera::getPointingAt, "C++: mrpt::opengl::CCamera::getPointingAt() const --> struct mrpt::math::TPoint3D_<float>");
		cl.def("setZoomDistance", (void (mrpt::opengl::CCamera::*)(float)) &mrpt::opengl::CCamera::setZoomDistance, "C++: mrpt::opengl::CCamera::setZoomDistance(float) --> void", pybind11::arg("z"));
		cl.def("getZoomDistance", (float (mrpt::opengl::CCamera::*)() const) &mrpt::opengl::CCamera::getZoomDistance, "C++: mrpt::opengl::CCamera::getZoomDistance() const --> float");
		cl.def("getAzimuthDegrees", (float (mrpt::opengl::CCamera::*)() const) &mrpt::opengl::CCamera::getAzimuthDegrees, "C++: mrpt::opengl::CCamera::getAzimuthDegrees() const --> float");
		cl.def("getElevationDegrees", (float (mrpt::opengl::CCamera::*)() const) &mrpt::opengl::CCamera::getElevationDegrees, "C++: mrpt::opengl::CCamera::getElevationDegrees() const --> float");
		cl.def("setAzimuthDegrees", (void (mrpt::opengl::CCamera::*)(float)) &mrpt::opengl::CCamera::setAzimuthDegrees, "C++: mrpt::opengl::CCamera::setAzimuthDegrees(float) --> void", pybind11::arg("ang"));
		cl.def("setElevationDegrees", (void (mrpt::opengl::CCamera::*)(float)) &mrpt::opengl::CCamera::setElevationDegrees, "C++: mrpt::opengl::CCamera::setElevationDegrees(float) --> void", pybind11::arg("ang"));
		cl.def("set6DOFMode", (void (mrpt::opengl::CCamera::*)(bool)) &mrpt::opengl::CCamera::set6DOFMode, "Set 6DOFMode, if enabled camera is set according to its pose, set via\nCRenderizable::setPose(). (default=false).\n Conventionally, eye is set looking towards +Z axis, \"down\" is the +Y\n axis, right is \"+X\" axis. In this mode azimuth/elevation are ignored.\n\nC++: mrpt::opengl::CCamera::set6DOFMode(bool) --> void", pybind11::arg("v"));
		cl.def("is6DOFMode", (bool (mrpt::opengl::CCamera::*)() const) &mrpt::opengl::CCamera::is6DOFMode, "C++: mrpt::opengl::CCamera::is6DOFMode() const --> bool");
		cl.def("renderUpdateBuffers", (void (mrpt::opengl::CCamera::*)() const) &mrpt::opengl::CCamera::renderUpdateBuffers, "Render does nothing here. \n\nC++: mrpt::opengl::CCamera::renderUpdateBuffers() const --> void");
		cl.def("internalBoundingBoxLocal", (struct mrpt::math::TBoundingBox_<float> (mrpt::opengl::CCamera::*)() const) &mrpt::opengl::CCamera::internalBoundingBoxLocal, "In this class, returns a fixed box (max,max,max), (-max,-max,-max). \n\nC++: mrpt::opengl::CCamera::internalBoundingBoxLocal() const --> struct mrpt::math::TBoundingBox_<float>");
		cl.def("freeOpenGLResources", (void (mrpt::opengl::CCamera::*)()) &mrpt::opengl::CCamera::freeOpenGLResources, "C++: mrpt::opengl::CCamera::freeOpenGLResources() --> void");
		cl.def("assign", (class mrpt::opengl::CCamera & (mrpt::opengl::CCamera::*)(const class mrpt::opengl::CCamera &)) &mrpt::opengl::CCamera::operator=, "C++: mrpt::opengl::CCamera::operator=(const class mrpt::opengl::CCamera &) --> class mrpt::opengl::CCamera &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
