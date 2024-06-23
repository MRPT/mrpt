#include <any>
#include <functional>
#include <ios>
#include <istream>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/containers/CommentPosition.h>
#include <mrpt/containers/YamlEmitOptions.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/img/CCanvas.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/img/TColor.h>
#include <mrpt/img/TPixelCoord.h>
#include <mrpt/io/CStream.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TLine2D.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TPose3DQuat.h>
#include <mrpt/math/TSegment3D.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/opengl/CCamera.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CRenderizableShaderTexturedTriangles.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/opengl/CTexturedPlane.h>
#include <mrpt/opengl/FrameBuffer.h>
#include <mrpt/opengl/Scene.h>
#include <mrpt/opengl/Shader.h>
#include <mrpt/opengl/TLightParameters.h>
#include <mrpt/opengl/TRenderMatrices.h>
#include <mrpt/opengl/Viewport.h>
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

// mrpt::opengl::CTexturedPlane file:mrpt/opengl/CTexturedPlane.h line:26
struct PyCallBack_mrpt_opengl_CTexturedPlane : public mrpt::opengl::CTexturedPlane {
	using mrpt::opengl::CTexturedPlane::CTexturedPlane;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CTexturedPlane *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CTexturedPlane::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CTexturedPlane *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CTexturedPlane::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CTexturedPlane *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CTexturedPlane::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CTexturedPlane *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CTexturedPlane::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CTexturedPlane *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CTexturedPlane::serializeFrom(a0, a1);
	}
	void renderUpdateBuffers() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CTexturedPlane *>(this), "renderUpdateBuffers");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CTexturedPlane::renderUpdateBuffers();
	}
	void onUpdateBuffers_TexturedTriangles() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CTexturedPlane *>(this), "onUpdateBuffers_TexturedTriangles");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CTexturedPlane::onUpdateBuffers_TexturedTriangles();
	}
	void onUpdateBuffers_Triangles() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CTexturedPlane *>(this), "onUpdateBuffers_Triangles");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CTexturedPlane::onUpdateBuffers_Triangles();
	}
	void freeOpenGLResources() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CTexturedPlane *>(this), "freeOpenGLResources");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CTexturedPlane::freeOpenGLResources();
	}
	bool traceRay(const class mrpt::poses::CPose3D & a0, double & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CTexturedPlane *>(this), "traceRay");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTexturedPlane::traceRay(a0, a1);
	}
	struct mrpt::math::TBoundingBox_<float> internalBoundingBoxLocal() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CTexturedPlane *>(this), "internalBoundingBoxLocal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TBoundingBox_<float>>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TBoundingBox_<float>> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TBoundingBox_<float>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TBoundingBox_<float>>(std::move(o));
		}
		return CTexturedPlane::internalBoundingBoxLocal();
	}
	void initializeTextures() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CTexturedPlane *>(this), "initializeTextures");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRenderizableShaderTexturedTriangles::initializeTextures();
	}
	class mrpt::opengl::CRenderizable & setColorA_u8(const unsigned char a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CTexturedPlane *>(this), "setColorA_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CTexturedPlane *>(this), "setColor_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CTexturedPlane *>(this), "cullElegible");
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
	void toYAMLMap(class mrpt::containers::yaml & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CTexturedPlane *>(this), "toYAMLMap");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRenderizable::toYAMLMap(a0);
	}
	bool isCompositeObject() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CTexturedPlane *>(this), "isCompositeObject");
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
	struct mrpt::math::TPoint3D_<float> getLocalRepresentativePoint() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CTexturedPlane *>(this), "getLocalRepresentativePoint");
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
};

// mrpt::opengl::Viewport file:mrpt/opengl/Viewport.h line:64
struct PyCallBack_mrpt_opengl_Viewport : public mrpt::opengl::Viewport {
	using mrpt::opengl::Viewport::Viewport;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::Viewport *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return Viewport::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::Viewport *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return Viewport::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::Viewport *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return Viewport::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::Viewport *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Viewport::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::Viewport *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Viewport::serializeFrom(a0, a1);
	}
};

void bind_mrpt_opengl_CTexturedPlane(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::opengl::CTexturedPlane file:mrpt/opengl/CTexturedPlane.h line:26
		pybind11::class_<mrpt::opengl::CTexturedPlane, std::shared_ptr<mrpt::opengl::CTexturedPlane>, PyCallBack_mrpt_opengl_CTexturedPlane, mrpt::opengl::CRenderizableShaderTexturedTriangles, mrpt::opengl::CRenderizableShaderTriangles> cl(M("mrpt::opengl"), "CTexturedPlane", "A 2D plane in the XY plane with a texture image.\n\n Lighting is disabled by default in this class, so the plane color or texture\n will be independent of its orientation or shadows cast on it.\n This can be changed calling enableLighting(true)\n\n  \n opengl::Scene\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::CTexturedPlane(); }, [](){ return new PyCallBack_mrpt_opengl_CTexturedPlane(); } ), "doc");
		cl.def( pybind11::init( [](float const & a0){ return new mrpt::opengl::CTexturedPlane(a0); }, [](float const & a0){ return new PyCallBack_mrpt_opengl_CTexturedPlane(a0); } ), "doc");
		cl.def( pybind11::init( [](float const & a0, float const & a1){ return new mrpt::opengl::CTexturedPlane(a0, a1); }, [](float const & a0, float const & a1){ return new PyCallBack_mrpt_opengl_CTexturedPlane(a0, a1); } ), "doc");
		cl.def( pybind11::init( [](float const & a0, float const & a1, float const & a2){ return new mrpt::opengl::CTexturedPlane(a0, a1, a2); }, [](float const & a0, float const & a1, float const & a2){ return new PyCallBack_mrpt_opengl_CTexturedPlane(a0, a1, a2); } ), "doc");
		cl.def( pybind11::init<float, float, float, float>(), pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_opengl_CTexturedPlane const &o){ return new PyCallBack_mrpt_opengl_CTexturedPlane(o); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::CTexturedPlane const &o){ return new mrpt::opengl::CTexturedPlane(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CTexturedPlane::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CTexturedPlane::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CTexturedPlane::*)() const) &mrpt::opengl::CTexturedPlane::GetRuntimeClass, "C++: mrpt::opengl::CTexturedPlane::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::opengl::CTexturedPlane::*)() const) &mrpt::opengl::CTexturedPlane::clone, "C++: mrpt::opengl::CTexturedPlane::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::opengl::CTexturedPlane::CreateObject, "C++: mrpt::opengl::CTexturedPlane::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("renderUpdateBuffers", (void (mrpt::opengl::CTexturedPlane::*)() const) &mrpt::opengl::CTexturedPlane::renderUpdateBuffers, "C++: mrpt::opengl::CTexturedPlane::renderUpdateBuffers() const --> void");
		cl.def("onUpdateBuffers_TexturedTriangles", (void (mrpt::opengl::CTexturedPlane::*)()) &mrpt::opengl::CTexturedPlane::onUpdateBuffers_TexturedTriangles, "C++: mrpt::opengl::CTexturedPlane::onUpdateBuffers_TexturedTriangles() --> void");
		cl.def("onUpdateBuffers_Triangles", (void (mrpt::opengl::CTexturedPlane::*)()) &mrpt::opengl::CTexturedPlane::onUpdateBuffers_Triangles, "C++: mrpt::opengl::CTexturedPlane::onUpdateBuffers_Triangles() --> void");
		cl.def("freeOpenGLResources", (void (mrpt::opengl::CTexturedPlane::*)()) &mrpt::opengl::CTexturedPlane::freeOpenGLResources, "C++: mrpt::opengl::CTexturedPlane::freeOpenGLResources() --> void");
		cl.def("cullFaces", (void (mrpt::opengl::CTexturedPlane::*)(const enum mrpt::opengl::TCullFace &)) &mrpt::opengl::CTexturedPlane::cullFaces, "Control whether to render the FRONT, BACK, or BOTH (default) set of\n faces. Refer to docs for glCullFace() \n\nC++: mrpt::opengl::CTexturedPlane::cullFaces(const enum mrpt::opengl::TCullFace &) --> void", pybind11::arg("cf"));
		cl.def("cullFaces", (enum mrpt::opengl::TCullFace (mrpt::opengl::CTexturedPlane::*)() const) &mrpt::opengl::CTexturedPlane::cullFaces, "C++: mrpt::opengl::CTexturedPlane::cullFaces() const --> enum mrpt::opengl::TCullFace");
		cl.def("setPlaneCorners", (void (mrpt::opengl::CTexturedPlane::*)(float, float, float, float)) &mrpt::opengl::CTexturedPlane::setPlaneCorners, "Set the coordinates of the four corners that define the plane on the XY\n plane. \n\nC++: mrpt::opengl::CTexturedPlane::setPlaneCorners(float, float, float, float) --> void", pybind11::arg("xMin"), pybind11::arg("xMax"), pybind11::arg("yMin"), pybind11::arg("yMax"));
		cl.def("getPlaneCorners", (void (mrpt::opengl::CTexturedPlane::*)(float &, float &, float &, float &) const) &mrpt::opengl::CTexturedPlane::getPlaneCorners, "Get the coordinates of the four corners that define the plane on the XY\n plane. \n\nC++: mrpt::opengl::CTexturedPlane::getPlaneCorners(float &, float &, float &, float &) const --> void", pybind11::arg("xMin"), pybind11::arg("xMax"), pybind11::arg("yMin"), pybind11::arg("yMax"));
		cl.def("enableLighting", [](mrpt::opengl::CTexturedPlane &o) -> void { return o.enableLighting(); }, "");
		cl.def("enableLighting", (void (mrpt::opengl::CTexturedPlane::*)(bool)) &mrpt::opengl::CTexturedPlane::enableLighting, "C++: mrpt::opengl::CTexturedPlane::enableLighting(bool) --> void", pybind11::arg("enable"));
		cl.def("traceRay", (bool (mrpt::opengl::CTexturedPlane::*)(const class mrpt::poses::CPose3D &, double &) const) &mrpt::opengl::CTexturedPlane::traceRay, "C++: mrpt::opengl::CTexturedPlane::traceRay(const class mrpt::poses::CPose3D &, double &) const --> bool", pybind11::arg("o"), pybind11::arg("dist"));
		cl.def("internalBoundingBoxLocal", (struct mrpt::math::TBoundingBox_<float> (mrpt::opengl::CTexturedPlane::*)() const) &mrpt::opengl::CTexturedPlane::internalBoundingBoxLocal, "C++: mrpt::opengl::CTexturedPlane::internalBoundingBoxLocal() const --> struct mrpt::math::TBoundingBox_<float>");
		cl.def("assign", (class mrpt::opengl::CTexturedPlane & (mrpt::opengl::CTexturedPlane::*)(const class mrpt::opengl::CTexturedPlane &)) &mrpt::opengl::CTexturedPlane::operator=, "C++: mrpt::opengl::CTexturedPlane::operator=(const class mrpt::opengl::CTexturedPlane &) --> class mrpt::opengl::CTexturedPlane &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::opengl::FrameBufferBinding file:mrpt/opengl/FrameBuffer.h line:23
		pybind11::class_<mrpt::opengl::FrameBufferBinding, std::shared_ptr<mrpt::opengl::FrameBufferBinding>> cl(M("mrpt::opengl"), "FrameBufferBinding", "IDs of FrameBuffers, as used in FrameBuffer\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::FrameBufferBinding(); } ) );
		cl.def_readwrite("drawFbId", &mrpt::opengl::FrameBufferBinding::drawFbId);
		cl.def_readwrite("readFbId", &mrpt::opengl::FrameBufferBinding::readFbId);
	}
	{ // mrpt::opengl::FrameBuffer file:mrpt/opengl/FrameBuffer.h line:37
		pybind11::class_<mrpt::opengl::FrameBuffer, std::shared_ptr<mrpt::opengl::FrameBuffer>> cl(M("mrpt::opengl"), "FrameBuffer", "An OpenGL FrameBuffer resource (FBO) with either RGBA+depth or depth only\n render buffers.\n\n Refer to docs for glGenFramebuffers() and glGenRenderbuffers().\n\n \n Buffer, DepthMapFBO\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::FrameBuffer(); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::FrameBuffer const &o){ return new mrpt::opengl::FrameBuffer(o); } ) );
		cl.def("create", [](mrpt::opengl::FrameBuffer &o, unsigned int const & a0, unsigned int const & a1) -> void { return o.create(a0, a1); }, "", pybind11::arg("width"), pybind11::arg("height"));
		cl.def("create", (void (mrpt::opengl::FrameBuffer::*)(unsigned int, unsigned int, int)) &mrpt::opengl::FrameBuffer::create, "Creates a new FB object and the two (RGBA+depth) render buffers.\n\nC++: mrpt::opengl::FrameBuffer::create(unsigned int, unsigned int, int) --> void", pybind11::arg("width"), pybind11::arg("height"), pybind11::arg("nSamples"));
		cl.def("createDepthMap", (void (mrpt::opengl::FrameBuffer::*)(unsigned int, unsigned int)) &mrpt::opengl::FrameBuffer::createDepthMap, "Creates a new depth-only FBO.\n\nC++: mrpt::opengl::FrameBuffer::createDepthMap(unsigned int, unsigned int) --> void", pybind11::arg("width"), pybind11::arg("height"));
		cl.def("destroy", (void (mrpt::opengl::FrameBuffer::*)()) &mrpt::opengl::FrameBuffer::destroy, "Release resources \n\nC++: mrpt::opengl::FrameBuffer::destroy() --> void");
		cl.def("bind", (struct mrpt::opengl::FrameBufferBinding (mrpt::opengl::FrameBuffer::*)()) &mrpt::opengl::FrameBuffer::bind, "Bind this framebuffer object to the current context.\n  \n\n The former binding\n  \n\n Bind(), CurrentBinding()\n\nC++: mrpt::opengl::FrameBuffer::bind() --> struct mrpt::opengl::FrameBufferBinding");
		cl.def("unbind", (void (mrpt::opengl::FrameBuffer::*)()) &mrpt::opengl::FrameBuffer::unbind, "Unbind the framebuffer object from the context \n\nC++: mrpt::opengl::FrameBuffer::unbind() --> void");
		cl.def("blit", (void (mrpt::opengl::FrameBuffer::*)()) &mrpt::opengl::FrameBuffer::blit, "Blit the framebuffer object onto the screen\n\nC++: mrpt::opengl::FrameBuffer::blit() --> void");
		cl.def("initialized", (bool (mrpt::opengl::FrameBuffer::*)() const) &mrpt::opengl::FrameBuffer::initialized, "C++: mrpt::opengl::FrameBuffer::initialized() const --> bool");
		cl.def("width", (unsigned int (mrpt::opengl::FrameBuffer::*)() const) &mrpt::opengl::FrameBuffer::width, "C++: mrpt::opengl::FrameBuffer::width() const --> unsigned int");
		cl.def("height", (unsigned int (mrpt::opengl::FrameBuffer::*)() const) &mrpt::opengl::FrameBuffer::height, "C++: mrpt::opengl::FrameBuffer::height() const --> unsigned int");
		cl.def("numSamples", (int (mrpt::opengl::FrameBuffer::*)() const) &mrpt::opengl::FrameBuffer::numSamples, "C++: mrpt::opengl::FrameBuffer::numSamples() const --> int");
		cl.def("depthMapTextureId", (unsigned int (mrpt::opengl::FrameBuffer::*)() const) &mrpt::opengl::FrameBuffer::depthMapTextureId, "C++: mrpt::opengl::FrameBuffer::depthMapTextureId() const --> unsigned int");
		cl.def_static("Bind", (void (*)(const struct mrpt::opengl::FrameBufferBinding &)) &mrpt::opengl::FrameBuffer::Bind, "@{ \n\nC++: mrpt::opengl::FrameBuffer::Bind(const struct mrpt::opengl::FrameBufferBinding &) --> void", pybind11::arg("ids"));
		cl.def_static("Unbind", (void (*)()) &mrpt::opengl::FrameBuffer::Unbind, "C++: mrpt::opengl::FrameBuffer::Unbind() --> void");
		cl.def_static("CurrentBinding", (struct mrpt::opengl::FrameBufferBinding (*)()) &mrpt::opengl::FrameBuffer::CurrentBinding, "C++: mrpt::opengl::FrameBuffer::CurrentBinding() --> struct mrpt::opengl::FrameBufferBinding");
		cl.def("assign", (class mrpt::opengl::FrameBuffer & (mrpt::opengl::FrameBuffer::*)(const class mrpt::opengl::FrameBuffer &)) &mrpt::opengl::FrameBuffer::operator=, "C++: mrpt::opengl::FrameBuffer::operator=(const class mrpt::opengl::FrameBuffer &) --> class mrpt::opengl::FrameBuffer &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::opengl::Viewport file:mrpt/opengl/Viewport.h line:64
		pybind11::class_<mrpt::opengl::Viewport, std::shared_ptr<mrpt::opengl::Viewport>, PyCallBack_mrpt_opengl_Viewport, mrpt::serialization::CSerializable, mrpt::system::CObservable, mrpt::opengl::CTextMessageCapable> cl(M("mrpt::opengl"), "Viewport", "A viewport within a Scene, containing a set of OpenGL objects to\nrender.\n   This class has protected constuctor, thus it cannot be created by users.\nUse Scene::createViewport instead.\n  A viewport has these \"operation modes\":\n		- Normal (default): It renders the contained objects.\n		- Cloned: It clones the objects from another viewport. See \n		- Image mode: It renders an image (e.g. from a video stream) efficiently\nusing a textued quad. See \n\n In any case, the viewport can be resized to only fit a part of the entire\nparent viewport.\n  There will be always at least one viewport in a Scene named \"main\".\n\n This class can be observed (see mrpt::system::CObserver) for the following\nevents (see mrpt::system::mrptEvent):\n   - mrpt::opengl::mrptEventGLPreRender\n   - mrpt::opengl::mrptEventGLPostRender\n\n Two directional light sources at infinity are created by default, with\ndirections (-1,-1,-1) and (1,2,1), respectively.\n\n Lighting parameters are accessible via lightParameters().\n\n  Refer to mrpt::opengl::Scene for further details.\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::Viewport(); }, [](){ return new PyCallBack_mrpt_opengl_Viewport(); } ), "doc");
		cl.def( pybind11::init( [](class mrpt::opengl::Scene * a0){ return new mrpt::opengl::Viewport(a0); }, [](class mrpt::opengl::Scene * a0){ return new PyCallBack_mrpt_opengl_Viewport(a0); } ), "doc");
		cl.def( pybind11::init<class mrpt::opengl::Scene *, const std::string &>(), pybind11::arg("parent"), pybind11::arg("name") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_opengl_Viewport const &o){ return new PyCallBack_mrpt_opengl_Viewport(o); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::Viewport const &o){ return new mrpt::opengl::Viewport(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::Viewport::GetRuntimeClassIdStatic, "C++: mrpt::opengl::Viewport::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::Viewport::*)() const) &mrpt::opengl::Viewport::GetRuntimeClass, "C++: mrpt::opengl::Viewport::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::opengl::Viewport::*)() const) &mrpt::opengl::Viewport::clone, "C++: mrpt::opengl::Viewport::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::opengl::Viewport::CreateObject, "C++: mrpt::opengl::Viewport::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("setCloneView", (void (mrpt::opengl::Viewport::*)(const std::string &)) &mrpt::opengl::Viewport::setCloneView, "Set this viewport as a clone of some other viewport, given its name - as\n a side effect, current list of internal OpenGL objects is cleared.\n  By default, only the objects are cloned, not the camera. See\n \n\n resetCloneView\n\nC++: mrpt::opengl::Viewport::setCloneView(const std::string &) --> void", pybind11::arg("clonedViewport"));
		cl.def("setImageView", [](mrpt::opengl::Viewport &o, const class mrpt::img::CImage & a0) -> void { return o.setImageView(a0); }, "", pybind11::arg("img"));
		cl.def("setImageView", (void (mrpt::opengl::Viewport::*)(const class mrpt::img::CImage &, bool)) &mrpt::opengl::Viewport::setImageView, "Set this viewport into \"image view\"-mode, where an image is efficiently\n drawn (fitting the viewport area) using an OpenGL textured quad.\n  Call this method with the new image to update the displayed image (but\n recall to first lock the parent openglscene's critical section, then do\n the update, then release the lock, and then issue a window repaint).\n  Internally, the texture is drawn using a mrpt::opengl::CTexturedPlane\n  The viewport can be reverted to behave like a normal viewport by\n calling setNormalMode()\n\n \n This method can also make the viewport\n transparent (default), so the area not filled with the image still allows\n seeing an underlying viewport.\n\nC++: mrpt::opengl::Viewport::setImageView(const class mrpt::img::CImage &, bool) --> void", pybind11::arg("img"), pybind11::arg("transparentBackground"));
		cl.def("isImageViewMode", (bool (mrpt::opengl::Viewport::*)() const) &mrpt::opengl::Viewport::isImageViewMode, "Returns true if setImageView() has been called on this viewport \n\nC++: mrpt::opengl::Viewport::isImageViewMode() const --> bool");
		cl.def("resetCloneView", (void (mrpt::opengl::Viewport::*)()) &mrpt::opengl::Viewport::resetCloneView, "Reset the viewport to normal mode: rendering its own objects.\n \n\n setCloneView, setNormalMode\n\nC++: mrpt::opengl::Viewport::resetCloneView() --> void");
		cl.def("setCloneCamera", (void (mrpt::opengl::Viewport::*)(bool)) &mrpt::opengl::Viewport::setCloneCamera, "If set to true, and setCloneView() has been called, this viewport will\n be rendered using the camera of the cloned viewport.\n\nC++: mrpt::opengl::Viewport::setCloneCamera(bool) --> void", pybind11::arg("enable"));
		cl.def("setClonedCameraFrom", (void (mrpt::opengl::Viewport::*)(const std::string &)) &mrpt::opengl::Viewport::setClonedCameraFrom, "Use the camera of another viewport.\n  Note this works even for viewports not in \"clone\" mode, so you can\n  render different scenes but using the same camera.\n\nC++: mrpt::opengl::Viewport::setClonedCameraFrom(const std::string &) --> void", pybind11::arg("viewPortName"));
		cl.def("setNormalMode", (void (mrpt::opengl::Viewport::*)()) &mrpt::opengl::Viewport::setNormalMode, "Resets the viewport to a normal 3D viewport \n setCloneView,\n setImageView \n\nC++: mrpt::opengl::Viewport::setNormalMode() --> void");
		cl.def("setViewportVisibility", (void (mrpt::opengl::Viewport::*)(bool)) &mrpt::opengl::Viewport::setViewportVisibility, "C++: mrpt::opengl::Viewport::setViewportVisibility(bool) --> void", pybind11::arg("visible"));
		cl.def("getViewportVisibility", (bool (mrpt::opengl::Viewport::*)() const) &mrpt::opengl::Viewport::getViewportVisibility, "C++: mrpt::opengl::Viewport::getViewportVisibility() const --> bool");
		cl.def("enablePolygonNicest", [](mrpt::opengl::Viewport &o) -> void { return o.enablePolygonNicest(); }, "");
		cl.def("enablePolygonNicest", (void (mrpt::opengl::Viewport::*)(bool)) &mrpt::opengl::Viewport::enablePolygonNicest, "Sets glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST) is enabled, or GL_FASTEST\n otherwise. \n\nC++: mrpt::opengl::Viewport::enablePolygonNicest(bool) --> void", pybind11::arg("enable"));
		cl.def("isPolygonNicestEnabled", (bool (mrpt::opengl::Viewport::*)() const) &mrpt::opengl::Viewport::isPolygonNicestEnabled, "C++: mrpt::opengl::Viewport::isPolygonNicestEnabled() const --> bool");
		cl.def("lightParameters", (struct mrpt::opengl::TLightParameters & (mrpt::opengl::Viewport::*)()) &mrpt::opengl::Viewport::lightParameters, "C++: mrpt::opengl::Viewport::lightParameters() --> struct mrpt::opengl::TLightParameters &", pybind11::return_value_policy::automatic);
		cl.def("getName", (std::string (mrpt::opengl::Viewport::*)()) &mrpt::opengl::Viewport::getName, "Returns the name of the viewport \n\nC++: mrpt::opengl::Viewport::getName() --> std::string");
		cl.def("setViewportPosition", (void (mrpt::opengl::Viewport::*)(const double, const double, const double, const double)) &mrpt::opengl::Viewport::setViewportPosition, "Change the viewport position and dimension on the rendering window.\n  X & Y coordinates here can have two interpretations:\n    - If in the range [0,1], they are factors with respect to the actual\nwindow sizes (i.e. width=1 means the entire width of the rendering\nwindow).\n    - If >1, they are interpreted as pixels.\n\n  width & height can be interpreted as:\n		- If >1, they are the size of the viewport in that dimension, in\npixels.\n		- If in [0,1], they are the size of the viewport in that dimension,\nin\na factor of the width/height.\n		- If in [-1,0[, the size is computed such as the right/top border\nends\nup in the given coordinate, interpreted as a factor (e.g. -1: up to the\nend of the viewport, -0.5: up to the middle of it).\n		- If <-1 the size is computed such as the right/top border ends up\nin\nthe given absolute coordinate (e.g. -200: up to the row/column 200px).\n\n \n (x,y) specify the lower left corner of the viewport rectangle.\n \n\n getViewportPosition\n\nC++: mrpt::opengl::Viewport::setViewportPosition(const double, const double, const double, const double) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("width"), pybind11::arg("height"));
		cl.def("getViewportPosition", (void (mrpt::opengl::Viewport::*)(double &, double &, double &, double &)) &mrpt::opengl::Viewport::getViewportPosition, "Get the current viewport position and dimension on the rendering window.\n  X & Y coordinates here can have two interpretations:\n    - If in the range [0,1], they are factors with respect to the actual\n window sizes (i.e. width=1 means the entire width of the rendering\n window).\n    - If >1, they are interpreted as pixels.\n \n\n (x,y) specify the lower left corner of the viewport rectangle.\n \n\n setViewportPosition\n\nC++: mrpt::opengl::Viewport::getViewportPosition(double &, double &, double &, double &) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("width"), pybind11::arg("height"));
		cl.def("setViewportClipDistances", (void (mrpt::opengl::Viewport::*)(const float, const float)) &mrpt::opengl::Viewport::setViewportClipDistances, "Set the min/max clip depth distances of the rendering frustum (default:\n 0.1 - 1000)\n \n\n getViewportClipDistances\n\nC++: mrpt::opengl::Viewport::setViewportClipDistances(const float, const float) --> void", pybind11::arg("clip_min"), pybind11::arg("clip_max"));
		cl.def("getViewportClipDistances", (void (mrpt::opengl::Viewport::*)(float &, float &) const) &mrpt::opengl::Viewport::getViewportClipDistances, "Get the current min/max clip depth distances of the rendering frustum\n (default: 0.1 - 1000)\n \n\n setViewportClipDistances\n\nC++: mrpt::opengl::Viewport::getViewportClipDistances(float &, float &) const --> void", pybind11::arg("clip_min"), pybind11::arg("clip_max"));
		cl.def("setLightShadowClipDistances", (void (mrpt::opengl::Viewport::*)(const float, const float)) &mrpt::opengl::Viewport::setLightShadowClipDistances, "C++: mrpt::opengl::Viewport::setLightShadowClipDistances(const float, const float) --> void", pybind11::arg("clip_min"), pybind11::arg("clip_max"));
		cl.def("getLightShadowClipDistances", (void (mrpt::opengl::Viewport::*)(float &, float &) const) &mrpt::opengl::Viewport::getLightShadowClipDistances, "C++: mrpt::opengl::Viewport::getLightShadowClipDistances(float &, float &) const --> void", pybind11::arg("clip_min"), pybind11::arg("clip_max"));
		cl.def("setBorderSize", (void (mrpt::opengl::Viewport::*)(unsigned int)) &mrpt::opengl::Viewport::setBorderSize, "Set the border size (\"frame\") of the viewport (default=0) \n\nC++: mrpt::opengl::Viewport::setBorderSize(unsigned int) --> void", pybind11::arg("lineWidth"));
		cl.def("getBorderSize", (unsigned int (mrpt::opengl::Viewport::*)() const) &mrpt::opengl::Viewport::getBorderSize, "C++: mrpt::opengl::Viewport::getBorderSize() const --> unsigned int");
		cl.def("setBorderColor", (void (mrpt::opengl::Viewport::*)(const struct mrpt::img::TColor &)) &mrpt::opengl::Viewport::setBorderColor, "C++: mrpt::opengl::Viewport::setBorderColor(const struct mrpt::img::TColor &) --> void", pybind11::arg("c"));
		cl.def("getBorderColor", (const struct mrpt::img::TColor & (mrpt::opengl::Viewport::*)() const) &mrpt::opengl::Viewport::getBorderColor, "C++: mrpt::opengl::Viewport::getBorderColor() const --> const struct mrpt::img::TColor &", pybind11::return_value_policy::automatic);
		cl.def("isTransparent", (bool (mrpt::opengl::Viewport::*)()) &mrpt::opengl::Viewport::isTransparent, "Return whether the viewport will be rendered transparent over previous\n viewports.\n\nC++: mrpt::opengl::Viewport::isTransparent() --> bool");
		cl.def("setTransparent", (void (mrpt::opengl::Viewport::*)(bool)) &mrpt::opengl::Viewport::setTransparent, "Set the transparency, that is, whether the viewport will be rendered\n transparent over previous viewports (default=false).\n\nC++: mrpt::opengl::Viewport::setTransparent(bool) --> void", pybind11::arg("trans"));
		cl.def("setCustomBackgroundColor", (void (mrpt::opengl::Viewport::*)(const struct mrpt::img::TColorf &)) &mrpt::opengl::Viewport::setCustomBackgroundColor, "Defines the viewport background color \n\nC++: mrpt::opengl::Viewport::setCustomBackgroundColor(const struct mrpt::img::TColorf &) --> void", pybind11::arg("color"));
		cl.def("getCustomBackgroundColor", (struct mrpt::img::TColorf (mrpt::opengl::Viewport::*)() const) &mrpt::opengl::Viewport::getCustomBackgroundColor, "C++: mrpt::opengl::Viewport::getCustomBackgroundColor() const --> struct mrpt::img::TColorf");
		cl.def("get3DRayForPixelCoord", [](mrpt::opengl::Viewport const &o, const double & a0, const double & a1, struct mrpt::math::TLine3D & a2) -> void { return o.get3DRayForPixelCoord(a0, a1, a2); }, "", pybind11::arg("x_coord"), pybind11::arg("y_coord"), pybind11::arg("out_ray"));
		cl.def("get3DRayForPixelCoord", (void (mrpt::opengl::Viewport::*)(const double, const double, struct mrpt::math::TLine3D &, class mrpt::poses::CPose3D *) const) &mrpt::opengl::Viewport::get3DRayForPixelCoord, "Compute the 3D ray corresponding to a given pixel; this can be used to\n allow the user to pick and select 3D objects by clicking onto the 2D\n image.\n  \n\n Horizontal coordinate with the usual meaning (0:left of\n the viewport, W-1: right border).\n  \n\n Horizontal coordinate with the usual meaning (0:top of\n the viewport, H-1: right border).\n \n\n If not nullptr, will have the camera 3D pose as a\n mrpt::poses::CPose3D. See also\n \n\n (x,y) refer to VIEWPORT coordinates. Take into account this when\n viewports do not extend to the whole window size.\n \n\n x and y are double instead of integers to allow sub-pixel\n precision.\n \n\n getCurrentCameraPose\n\nC++: mrpt::opengl::Viewport::get3DRayForPixelCoord(const double, const double, struct mrpt::math::TLine3D &, class mrpt::poses::CPose3D *) const --> void", pybind11::arg("x_coord"), pybind11::arg("y_coord"), pybind11::arg("out_ray"), pybind11::arg("out_cameraPose"));
		cl.def("enableShadowCasting", [](mrpt::opengl::Viewport &o) -> void { return o.enableShadowCasting(); }, "");
		cl.def("enableShadowCasting", [](mrpt::opengl::Viewport &o, bool const & a0) -> void { return o.enableShadowCasting(a0); }, "", pybind11::arg("enabled"));
		cl.def("enableShadowCasting", [](mrpt::opengl::Viewport &o, bool const & a0, unsigned int const & a1) -> void { return o.enableShadowCasting(a0, a1); }, "", pybind11::arg("enabled"), pybind11::arg("SHADOW_MAP_SIZE_X"));
		cl.def("enableShadowCasting", (void (mrpt::opengl::Viewport::*)(bool, unsigned int, unsigned int)) &mrpt::opengl::Viewport::enableShadowCasting, "Enables or disables rendering of shadows cast by the unidirectional\n light.\n \n\n Set to true to enable shadow casting\n         (default at ctor=false).\n \n\n Width of the shadow cast map (1st pass of\n         rendering with shadows). Larger values are slower but gives\n         more precise shadows. Default=2048x2048.\n         Zero means do not change.\n \n\n Like SHADOW_MAP_SIZE_X but defines the height.\n\n   \n\nC++: mrpt::opengl::Viewport::enableShadowCasting(bool, unsigned int, unsigned int) --> void", pybind11::arg("enabled"), pybind11::arg("SHADOW_MAP_SIZE_X"), pybind11::arg("SHADOW_MAP_SIZE_Y"));
		cl.def("isShadowCastingEnabled", (bool (mrpt::opengl::Viewport::*)() const) &mrpt::opengl::Viewport::isShadowCastingEnabled, "C++: mrpt::opengl::Viewport::isShadowCastingEnabled() const --> bool");
		cl.def("clear", (void (mrpt::opengl::Viewport::*)()) &mrpt::opengl::Viewport::clear, "Delete all internal obejcts\n \n\n insert \n\nC++: mrpt::opengl::Viewport::clear() --> void");
		cl.def("insert", (void (mrpt::opengl::Viewport::*)(const class std::shared_ptr<class mrpt::opengl::CRenderizable> &)) &mrpt::opengl::Viewport::insert, "Insert a new object into the list.\n  The object MUST NOT be deleted, it will be deleted automatically by\n this object when not required anymore.\n\nC++: mrpt::opengl::Viewport::insert(const class std::shared_ptr<class mrpt::opengl::CRenderizable> &) --> void", pybind11::arg("newObject"));
		cl.def("getCurrentCameraPose", (void (mrpt::opengl::Viewport::*)(class mrpt::poses::CPose3D &) const) &mrpt::opengl::Viewport::getCurrentCameraPose, "Compute the current 3D camera pose: +Z points forward, +X to the right,\n +Y down.\n\n \n get3DRayForPixelCoord\n\nC++: mrpt::opengl::Viewport::getCurrentCameraPose(class mrpt::poses::CPose3D &) const --> void", pybind11::arg("out_cameraPose"));
		cl.def("getCurrentCameraPose", (class mrpt::poses::CPose3D (mrpt::opengl::Viewport::*)() const) &mrpt::opengl::Viewport::getCurrentCameraPose, "C++: mrpt::opengl::Viewport::getCurrentCameraPose() const --> class mrpt::poses::CPose3D");
		cl.def("setCurrentCameraFromPose", (void (mrpt::opengl::Viewport::*)(class mrpt::poses::CPose3D &)) &mrpt::opengl::Viewport::setCurrentCameraFromPose, "Changes the point of view of the camera, from a given pose.\n \n\n getCurrentCameraPose\n\nC++: mrpt::opengl::Viewport::setCurrentCameraFromPose(class mrpt::poses::CPose3D &) --> void", pybind11::arg("p"));
		cl.def("getByName", (class std::shared_ptr<class mrpt::opengl::CRenderizable> (mrpt::opengl::Viewport::*)(const std::string &)) &mrpt::opengl::Viewport::getByName, "Returns the first object with a given name, or nullptr if not found.\n\nC++: mrpt::opengl::Viewport::getByName(const std::string &) --> class std::shared_ptr<class mrpt::opengl::CRenderizable>", pybind11::arg("str"));
		cl.def("removeObject", (void (mrpt::opengl::Viewport::*)(const class std::shared_ptr<class mrpt::opengl::CRenderizable> &)) &mrpt::opengl::Viewport::removeObject, "Removes the given object from the scene (it also deletes the object to\n free its memory).\n\nC++: mrpt::opengl::Viewport::removeObject(const class std::shared_ptr<class mrpt::opengl::CRenderizable> &) --> void", pybind11::arg("obj"));
		cl.def("size", (size_t (mrpt::opengl::Viewport::*)() const) &mrpt::opengl::Viewport::size, "Number of objects contained. \n\nC++: mrpt::opengl::Viewport::size() const --> size_t");
		cl.def("empty", (bool (mrpt::opengl::Viewport::*)() const) &mrpt::opengl::Viewport::empty, "C++: mrpt::opengl::Viewport::empty() const --> bool");
		cl.def("getCamera", (class mrpt::opengl::CCamera & (mrpt::opengl::Viewport::*)()) &mrpt::opengl::Viewport::getCamera, "Get a reference to the camera associated with this viewport. \n\nC++: mrpt::opengl::Viewport::getCamera() --> class mrpt::opengl::CCamera &", pybind11::return_value_policy::automatic);
		cl.def("getBoundingBox", (struct mrpt::math::TBoundingBox_<double> (mrpt::opengl::Viewport::*)() const) &mrpt::opengl::Viewport::getBoundingBox, "C++: mrpt::opengl::Viewport::getBoundingBox() const --> struct mrpt::math::TBoundingBox_<double>");
		cl.def("getRenderMatrices", (struct mrpt::opengl::TRenderMatrices (mrpt::opengl::Viewport::*)() const) &mrpt::opengl::Viewport::getRenderMatrices, "Returns a copy of the latest render matrices structure. \n\nC++: mrpt::opengl::Viewport::getRenderMatrices() const --> struct mrpt::opengl::TRenderMatrices");
		cl.def("render", [](mrpt::opengl::Viewport const &o, const int & a0, const int & a1) -> void { return o.render(a0, a1); }, "", pybind11::arg("render_width"), pybind11::arg("render_height"));
		cl.def("render", [](mrpt::opengl::Viewport const &o, const int & a0, const int & a1, const int & a2) -> void { return o.render(a0, a1, a2); }, "", pybind11::arg("render_width"), pybind11::arg("render_height"), pybind11::arg("render_offset_x"));
		cl.def("render", [](mrpt::opengl::Viewport const &o, const int & a0, const int & a1, const int & a2, const int & a3) -> void { return o.render(a0, a1, a2, a3); }, "", pybind11::arg("render_width"), pybind11::arg("render_height"), pybind11::arg("render_offset_x"), pybind11::arg("render_offset_y"));
		cl.def("render", (void (mrpt::opengl::Viewport::*)(const int, const int, const int, const int, const class mrpt::opengl::CCamera *) const) &mrpt::opengl::Viewport::render, "Render the objects in this viewport (called from Scene) \n\nC++: mrpt::opengl::Viewport::render(const int, const int, const int, const int, const class mrpt::opengl::CCamera *) const --> void", pybind11::arg("render_width"), pybind11::arg("render_height"), pybind11::arg("render_offset_x"), pybind11::arg("render_offset_y"), pybind11::arg("forceThisCamera"));
		cl.def("updateMatricesFromCamera", (void (mrpt::opengl::Viewport::*)(const class mrpt::opengl::CCamera &) const) &mrpt::opengl::Viewport::updateMatricesFromCamera, "myCamera must come from internalResolveActiveCamera()\n\nC++: mrpt::opengl::Viewport::updateMatricesFromCamera(const class mrpt::opengl::CCamera &) const --> void", pybind11::arg("myCamera"));
		cl.def("loadDefaultShaders", (void (mrpt::opengl::Viewport::*)() const) &mrpt::opengl::Viewport::loadDefaultShaders, "Load all MPRT predefined shader programs into m_shaders \n\nC++: mrpt::opengl::Viewport::loadDefaultShaders() const --> void");
		cl.def("assign", (class mrpt::opengl::Viewport & (mrpt::opengl::Viewport::*)(const class mrpt::opengl::Viewport &)) &mrpt::opengl::Viewport::operator=, "C++: mrpt::opengl::Viewport::operator=(const class mrpt::opengl::Viewport &) --> class mrpt::opengl::Viewport &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
