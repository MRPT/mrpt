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
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TPose3DQuat.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/opengl/CCamera.h>
#include <mrpt/opengl/CEllipsoidInverseDepth2D.h>
#include <mrpt/opengl/CEllipsoidInverseDepth3D.h>
#include <mrpt/opengl/CEllipsoidRangeBearing2D.h>
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/opengl/CGeneralizedEllipsoidTemplate.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/opengl/Scene.h>
#include <mrpt/opengl/Viewport.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CMessage.h>
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
#include <stl_binders.hpp>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

// mrpt::opengl::CEllipsoidInverseDepth2D file:mrpt/opengl/CEllipsoidInverseDepth2D.h line:37
struct PyCallBack_mrpt_opengl_CEllipsoidInverseDepth2D : public mrpt::opengl::CEllipsoidInverseDepth2D {
	using mrpt::opengl::CEllipsoidInverseDepth2D::CEllipsoidInverseDepth2D;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidInverseDepth2D *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CEllipsoidInverseDepth2D::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidInverseDepth2D *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CEllipsoidInverseDepth2D::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidInverseDepth2D *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CEllipsoidInverseDepth2D::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidInverseDepth2D *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CEllipsoidInverseDepth2D::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidInverseDepth2D *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CEllipsoidInverseDepth2D::serializeFrom(a0, a1);
	}
	void freeOpenGLResources() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidInverseDepth2D *>(this), "freeOpenGLResources");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGeneralizedEllipsoidTemplate::freeOpenGLResources();
	}
	void renderUpdateBuffers() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidInverseDepth2D *>(this), "renderUpdateBuffers");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGeneralizedEllipsoidTemplate::renderUpdateBuffers();
	}
	void onUpdateBuffers_Wireframe() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidInverseDepth2D *>(this), "onUpdateBuffers_Wireframe");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGeneralizedEllipsoidTemplate::onUpdateBuffers_Wireframe();
	}
	void onUpdateBuffers_Triangles() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidInverseDepth2D *>(this), "onUpdateBuffers_Triangles");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGeneralizedEllipsoidTemplate::onUpdateBuffers_Triangles();
	}
	bool traceRay(const class mrpt::poses::CPose3D & a0, double & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidInverseDepth2D *>(this), "traceRay");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CGeneralizedEllipsoidTemplate::traceRay(a0, a1);
	}
	struct mrpt::math::TBoundingBox_<float> internalBoundingBoxLocal() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidInverseDepth2D *>(this), "internalBoundingBoxLocal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TBoundingBox_<float>>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TBoundingBox_<float>> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TBoundingBox_<float>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TBoundingBox_<float>>(std::move(o));
		}
		return CGeneralizedEllipsoidTemplate::internalBoundingBoxLocal();
	}
	class mrpt::opengl::CRenderizable & setColorA_u8(const unsigned char a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidInverseDepth2D *>(this), "setColorA_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidInverseDepth2D *>(this), "setColor_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidInverseDepth2D *>(this), "cullElegible");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidInverseDepth2D *>(this), "toYAMLMap");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidInverseDepth2D *>(this), "isCompositeObject");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidInverseDepth2D *>(this), "getLocalRepresentativePoint");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidInverseDepth2D *>(this), "initializeTextures");
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

// mrpt::opengl::CEllipsoidInverseDepth3D file:mrpt/opengl/CEllipsoidInverseDepth3D.h line:39
struct PyCallBack_mrpt_opengl_CEllipsoidInverseDepth3D : public mrpt::opengl::CEllipsoidInverseDepth3D {
	using mrpt::opengl::CEllipsoidInverseDepth3D::CEllipsoidInverseDepth3D;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidInverseDepth3D *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CEllipsoidInverseDepth3D::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidInverseDepth3D *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CEllipsoidInverseDepth3D::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidInverseDepth3D *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CEllipsoidInverseDepth3D::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidInverseDepth3D *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CEllipsoidInverseDepth3D::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidInverseDepth3D *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CEllipsoidInverseDepth3D::serializeFrom(a0, a1);
	}
	void freeOpenGLResources() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidInverseDepth3D *>(this), "freeOpenGLResources");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGeneralizedEllipsoidTemplate::freeOpenGLResources();
	}
	void renderUpdateBuffers() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidInverseDepth3D *>(this), "renderUpdateBuffers");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGeneralizedEllipsoidTemplate::renderUpdateBuffers();
	}
	void onUpdateBuffers_Wireframe() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidInverseDepth3D *>(this), "onUpdateBuffers_Wireframe");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGeneralizedEllipsoidTemplate::onUpdateBuffers_Wireframe();
	}
	void onUpdateBuffers_Triangles() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidInverseDepth3D *>(this), "onUpdateBuffers_Triangles");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGeneralizedEllipsoidTemplate::onUpdateBuffers_Triangles();
	}
	bool traceRay(const class mrpt::poses::CPose3D & a0, double & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidInverseDepth3D *>(this), "traceRay");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CGeneralizedEllipsoidTemplate::traceRay(a0, a1);
	}
	struct mrpt::math::TBoundingBox_<float> internalBoundingBoxLocal() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidInverseDepth3D *>(this), "internalBoundingBoxLocal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TBoundingBox_<float>>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TBoundingBox_<float>> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TBoundingBox_<float>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TBoundingBox_<float>>(std::move(o));
		}
		return CGeneralizedEllipsoidTemplate::internalBoundingBoxLocal();
	}
	class mrpt::opengl::CRenderizable & setColorA_u8(const unsigned char a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidInverseDepth3D *>(this), "setColorA_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidInverseDepth3D *>(this), "setColor_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidInverseDepth3D *>(this), "cullElegible");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidInverseDepth3D *>(this), "toYAMLMap");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidInverseDepth3D *>(this), "isCompositeObject");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidInverseDepth3D *>(this), "getLocalRepresentativePoint");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidInverseDepth3D *>(this), "initializeTextures");
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

// mrpt::opengl::CEllipsoidRangeBearing2D file:mrpt/opengl/CEllipsoidRangeBearing2D.h line:34
struct PyCallBack_mrpt_opengl_CEllipsoidRangeBearing2D : public mrpt::opengl::CEllipsoidRangeBearing2D {
	using mrpt::opengl::CEllipsoidRangeBearing2D::CEllipsoidRangeBearing2D;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidRangeBearing2D *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CEllipsoidRangeBearing2D::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidRangeBearing2D *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CEllipsoidRangeBearing2D::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidRangeBearing2D *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CEllipsoidRangeBearing2D::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidRangeBearing2D *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CEllipsoidRangeBearing2D::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidRangeBearing2D *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CEllipsoidRangeBearing2D::serializeFrom(a0, a1);
	}
	void freeOpenGLResources() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidRangeBearing2D *>(this), "freeOpenGLResources");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGeneralizedEllipsoidTemplate::freeOpenGLResources();
	}
	void renderUpdateBuffers() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidRangeBearing2D *>(this), "renderUpdateBuffers");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGeneralizedEllipsoidTemplate::renderUpdateBuffers();
	}
	void onUpdateBuffers_Wireframe() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidRangeBearing2D *>(this), "onUpdateBuffers_Wireframe");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGeneralizedEllipsoidTemplate::onUpdateBuffers_Wireframe();
	}
	void onUpdateBuffers_Triangles() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidRangeBearing2D *>(this), "onUpdateBuffers_Triangles");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGeneralizedEllipsoidTemplate::onUpdateBuffers_Triangles();
	}
	bool traceRay(const class mrpt::poses::CPose3D & a0, double & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidRangeBearing2D *>(this), "traceRay");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CGeneralizedEllipsoidTemplate::traceRay(a0, a1);
	}
	struct mrpt::math::TBoundingBox_<float> internalBoundingBoxLocal() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidRangeBearing2D *>(this), "internalBoundingBoxLocal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TBoundingBox_<float>>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TBoundingBox_<float>> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TBoundingBox_<float>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TBoundingBox_<float>>(std::move(o));
		}
		return CGeneralizedEllipsoidTemplate::internalBoundingBoxLocal();
	}
	class mrpt::opengl::CRenderizable & setColorA_u8(const unsigned char a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidRangeBearing2D *>(this), "setColorA_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidRangeBearing2D *>(this), "setColor_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidRangeBearing2D *>(this), "cullElegible");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidRangeBearing2D *>(this), "toYAMLMap");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidRangeBearing2D *>(this), "isCompositeObject");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidRangeBearing2D *>(this), "getLocalRepresentativePoint");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoidRangeBearing2D *>(this), "initializeTextures");
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

void bind_mrpt_opengl_CEllipsoidInverseDepth2D(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::opengl::CEllipsoidInverseDepth2D file:mrpt/opengl/CEllipsoidInverseDepth2D.h line:37
		pybind11::class_<mrpt::opengl::CEllipsoidInverseDepth2D, std::shared_ptr<mrpt::opengl::CEllipsoidInverseDepth2D>, PyCallBack_mrpt_opengl_CEllipsoidInverseDepth2D, mrpt::opengl::CGeneralizedEllipsoidTemplate<2>> cl(M("mrpt::opengl"), "CEllipsoidInverseDepth2D", "An especial \"ellipsoid\" in 3D computed as the uncertainty iso-surfaces of a\n (inv_range,yaw) variable.\n  The parameter space of this ellipsoid comprises these variables (in this\n order):\n   - inv_range: The inverse distance from the sensor to the feature.\n   - yaw: Angle for the rotation around +Z (\"azimuth\").\n\n  This parameterization is a 2D version of that presented in the paper:\n   - Civera, J. and Davison, A.J. and Montiel, J., \"Inverse depth\n parametrization for monocular SLAM\", T-RO, 2008.\n\n  This class expects you to provide a mean vector of length 4 and a 4x4\n covariance matrix, set with \n\n Please read the documentation of\n CGeneralizedEllipsoidTemplate::setQuantiles() for learning\n  the mathematical details about setting the desired confidence interval.\n\n ![mrpt::opengl::CEllipsoidInverseDepth2D](preview_CEllipsoidInverseDepth2D.png)\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::CEllipsoidInverseDepth2D(); }, [](){ return new PyCallBack_mrpt_opengl_CEllipsoidInverseDepth2D(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_opengl_CEllipsoidInverseDepth2D const &o){ return new PyCallBack_mrpt_opengl_CEllipsoidInverseDepth2D(o); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::CEllipsoidInverseDepth2D const &o){ return new mrpt::opengl::CEllipsoidInverseDepth2D(o); } ) );
		cl.def_static("getClassName", (class mrpt::typemeta::string_literal<38> (*)()) &mrpt::opengl::CEllipsoidInverseDepth2D::getClassName, "C++: mrpt::opengl::CEllipsoidInverseDepth2D::getClassName() --> class mrpt::typemeta::string_literal<38>");
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CEllipsoidInverseDepth2D::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CEllipsoidInverseDepth2D::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CEllipsoidInverseDepth2D::*)() const) &mrpt::opengl::CEllipsoidInverseDepth2D::GetRuntimeClass, "C++: mrpt::opengl::CEllipsoidInverseDepth2D::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::opengl::CEllipsoidInverseDepth2D::*)() const) &mrpt::opengl::CEllipsoidInverseDepth2D::clone, "C++: mrpt::opengl::CEllipsoidInverseDepth2D::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::opengl::CEllipsoidInverseDepth2D::CreateObject, "C++: mrpt::opengl::CEllipsoidInverseDepth2D::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("setUnderflowMaxRange", (void (mrpt::opengl::CEllipsoidInverseDepth2D::*)(const double)) &mrpt::opengl::CEllipsoidInverseDepth2D::setUnderflowMaxRange, "The maximum range to be used as a correction when a point of the\n ellipsoid falls in the negative ranges (default: 1e6) \n\nC++: mrpt::opengl::CEllipsoidInverseDepth2D::setUnderflowMaxRange(const double) --> void", pybind11::arg("maxRange"));
		cl.def("getUnderflowMaxRange", (double (mrpt::opengl::CEllipsoidInverseDepth2D::*)() const) &mrpt::opengl::CEllipsoidInverseDepth2D::getUnderflowMaxRange, "C++: mrpt::opengl::CEllipsoidInverseDepth2D::getUnderflowMaxRange() const --> double");
		cl.def("assign", (class mrpt::opengl::CEllipsoidInverseDepth2D & (mrpt::opengl::CEllipsoidInverseDepth2D::*)(const class mrpt::opengl::CEllipsoidInverseDepth2D &)) &mrpt::opengl::CEllipsoidInverseDepth2D::operator=, "C++: mrpt::opengl::CEllipsoidInverseDepth2D::operator=(const class mrpt::opengl::CEllipsoidInverseDepth2D &) --> class mrpt::opengl::CEllipsoidInverseDepth2D &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::opengl::CEllipsoidInverseDepth3D file:mrpt/opengl/CEllipsoidInverseDepth3D.h line:39
		pybind11::class_<mrpt::opengl::CEllipsoidInverseDepth3D, std::shared_ptr<mrpt::opengl::CEllipsoidInverseDepth3D>, PyCallBack_mrpt_opengl_CEllipsoidInverseDepth3D, mrpt::opengl::CGeneralizedEllipsoidTemplate<3>, mrpt::opengl::CRenderizable> cl(M("mrpt::opengl"), "CEllipsoidInverseDepth3D", "An especial \"ellipsoid\" in 3D computed as the uncertainty iso-surfaces of a\n (inv_range,yaw,pitch) variable.\n  The parameter space of this ellipsoid comprises these variables (in this\n order):\n   - inv_range: The inverse distance from the sensor to the feature.\n   - yaw: Angle for the rotation around +Z (\"azimuth\").\n   - pitch: Angle for the rotation around +Y (\"elevation\"). Positive means\n pointing below the XY plane.\n\n  This parameterization is based on the paper:\n   - Civera, J. and Davison, A.J. and Montiel, J., \"Inverse depth\n parametrization for monocular SLAM\", T-RO, 2008.\n\n  This class expects you to provide a mean vector of length 3 and a 3x3\n covariance matrix, set with \n\n Please read the documentation of\n CGeneralizedEllipsoidTemplate::setQuantiles() for learning\n  the mathematical details about setting the desired confidence interval.\n\n ![mrpt::opengl::CEllipsoidInverseDepth3D](preview_CEllipsoidInverseDepth3D.png)\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::CEllipsoidInverseDepth3D(); }, [](){ return new PyCallBack_mrpt_opengl_CEllipsoidInverseDepth3D(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_opengl_CEllipsoidInverseDepth3D const &o){ return new PyCallBack_mrpt_opengl_CEllipsoidInverseDepth3D(o); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::CEllipsoidInverseDepth3D const &o){ return new mrpt::opengl::CEllipsoidInverseDepth3D(o); } ) );
		cl.def_static("getClassName", (class mrpt::typemeta::string_literal<38> (*)()) &mrpt::opengl::CEllipsoidInverseDepth3D::getClassName, "C++: mrpt::opengl::CEllipsoidInverseDepth3D::getClassName() --> class mrpt::typemeta::string_literal<38>");
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CEllipsoidInverseDepth3D::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CEllipsoidInverseDepth3D::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CEllipsoidInverseDepth3D::*)() const) &mrpt::opengl::CEllipsoidInverseDepth3D::GetRuntimeClass, "C++: mrpt::opengl::CEllipsoidInverseDepth3D::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::opengl::CEllipsoidInverseDepth3D::*)() const) &mrpt::opengl::CEllipsoidInverseDepth3D::clone, "C++: mrpt::opengl::CEllipsoidInverseDepth3D::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::opengl::CEllipsoidInverseDepth3D::CreateObject, "C++: mrpt::opengl::CEllipsoidInverseDepth3D::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("setUnderflowMaxRange", (void (mrpt::opengl::CEllipsoidInverseDepth3D::*)(const float)) &mrpt::opengl::CEllipsoidInverseDepth3D::setUnderflowMaxRange, "The maximum range to be used as a correction when a point of the\n ellipsoid falls in the negative ranges (default: 1e6) \n\nC++: mrpt::opengl::CEllipsoidInverseDepth3D::setUnderflowMaxRange(const float) --> void", pybind11::arg("maxRange"));
		cl.def("getUnderflowMaxRange", (float (mrpt::opengl::CEllipsoidInverseDepth3D::*)() const) &mrpt::opengl::CEllipsoidInverseDepth3D::getUnderflowMaxRange, "C++: mrpt::opengl::CEllipsoidInverseDepth3D::getUnderflowMaxRange() const --> float");
		cl.def("assign", (class mrpt::opengl::CEllipsoidInverseDepth3D & (mrpt::opengl::CEllipsoidInverseDepth3D::*)(const class mrpt::opengl::CEllipsoidInverseDepth3D &)) &mrpt::opengl::CEllipsoidInverseDepth3D::operator=, "C++: mrpt::opengl::CEllipsoidInverseDepth3D::operator=(const class mrpt::opengl::CEllipsoidInverseDepth3D &) --> class mrpt::opengl::CEllipsoidInverseDepth3D &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::opengl::CEllipsoidRangeBearing2D file:mrpt/opengl/CEllipsoidRangeBearing2D.h line:34
		pybind11::class_<mrpt::opengl::CEllipsoidRangeBearing2D, std::shared_ptr<mrpt::opengl::CEllipsoidRangeBearing2D>, PyCallBack_mrpt_opengl_CEllipsoidRangeBearing2D, mrpt::opengl::CGeneralizedEllipsoidTemplate<2>, mrpt::opengl::CRenderizable> cl(M("mrpt::opengl"), "CEllipsoidRangeBearing2D", "An especial \"ellipsoid\" in 2D computed as the uncertainty iso-surfaces of a\n (range,bearing) variable.\n  The parameter space of this ellipsoid comprises these variables (in this\n order):\n   - range: Distance from sensor to feature.\n   - bearing: Angle from +X to the line that goes from the sensor towards the\n feature.\n\n  This class expects you to provide a mean vector of length 2 and a 2x2\n covariance matrix, set with \n\n Please read the documentation of\n CGeneralizedEllipsoidTemplate::setQuantiles() for learning\n  the mathematical details about setting the desired confidence interval.\n\n ![mrpt::opengl::CEllipsoidRangeBearing2D](preview_CEllipsoidRangeBearing2D.png)\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::CEllipsoidRangeBearing2D(); }, [](){ return new PyCallBack_mrpt_opengl_CEllipsoidRangeBearing2D(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_opengl_CEllipsoidRangeBearing2D const &o){ return new PyCallBack_mrpt_opengl_CEllipsoidRangeBearing2D(o); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::CEllipsoidRangeBearing2D const &o){ return new mrpt::opengl::CEllipsoidRangeBearing2D(o); } ) );
		cl.def_static("getClassName", (class mrpt::typemeta::string_literal<38> (*)()) &mrpt::opengl::CEllipsoidRangeBearing2D::getClassName, "C++: mrpt::opengl::CEllipsoidRangeBearing2D::getClassName() --> class mrpt::typemeta::string_literal<38>");
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CEllipsoidRangeBearing2D::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CEllipsoidRangeBearing2D::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CEllipsoidRangeBearing2D::*)() const) &mrpt::opengl::CEllipsoidRangeBearing2D::GetRuntimeClass, "C++: mrpt::opengl::CEllipsoidRangeBearing2D::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::opengl::CEllipsoidRangeBearing2D::*)() const) &mrpt::opengl::CEllipsoidRangeBearing2D::clone, "C++: mrpt::opengl::CEllipsoidRangeBearing2D::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::opengl::CEllipsoidRangeBearing2D::CreateObject, "C++: mrpt::opengl::CEllipsoidRangeBearing2D::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("assign", (class mrpt::opengl::CEllipsoidRangeBearing2D & (mrpt::opengl::CEllipsoidRangeBearing2D::*)(const class mrpt::opengl::CEllipsoidRangeBearing2D &)) &mrpt::opengl::CEllipsoidRangeBearing2D::operator=, "C++: mrpt::opengl::CEllipsoidRangeBearing2D::operator=(const class mrpt::opengl::CEllipsoidRangeBearing2D &) --> class mrpt::opengl::CEllipsoidRangeBearing2D &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::opengl::CFBORender file:mrpt/opengl/CFBORender.h line:37
		pybind11::class_<mrpt::opengl::CFBORender, std::shared_ptr<mrpt::opengl::CFBORender>> cl(M("mrpt::opengl"), "CFBORender", "Render 3D scenes off-screen directly to RGB and/or RGB+D images.\n\n Main methods:\n - render_RGB(): Renders a scene into an RGB image.\n - render_RGBD(): Renders a scene into an RGB and depth images.\n\n To define a background color, define it in your\n `scene.getViewport()->setCustomBackgroundColor()`. You can add overlaid text\n messages, see base class CTextMessageCapable\n\n The SE(3) pose from which the scene is rendered is defined by the scene\n `\"main\"` viewport camera pose.\n See  for code examples.\n\n \n  , \n \n\n\n ");
		cl.def( pybind11::init<const struct mrpt::opengl::CFBORender::Parameters &>(), pybind11::arg("p") );

		cl.def( pybind11::init( [](){ return new mrpt::opengl::CFBORender(); } ), "doc" );
		cl.def( pybind11::init( [](unsigned int const & a0){ return new mrpt::opengl::CFBORender(a0); } ), "doc" , pybind11::arg("width"));
		cl.def( pybind11::init<unsigned int, unsigned int>(), pybind11::arg("width"), pybind11::arg("height") );

		cl.def( pybind11::init( [](mrpt::opengl::CFBORender const &o){ return new mrpt::opengl::CFBORender(o); } ) );
		cl.def("setCamera", (void (mrpt::opengl::CFBORender::*)(const class mrpt::opengl::Scene &, const class mrpt::opengl::CCamera &)) &mrpt::opengl::CFBORender::setCamera, "Change the scene camera to be used when rendering the scene through this\n particular instance of CFBORender. \n\nC++: mrpt::opengl::CFBORender::setCamera(const class mrpt::opengl::Scene &, const class mrpt::opengl::CCamera &) --> void", pybind11::arg("scene"), pybind11::arg("camera"));
		cl.def("getCamera", (class mrpt::opengl::CCamera & (mrpt::opengl::CFBORender::*)(const class mrpt::opengl::Scene &)) &mrpt::opengl::CFBORender::getCamera, "Get a reference to the scene camera to be used when rendering the scene\n through this particular instance of CFBORender. \n\nC++: mrpt::opengl::CFBORender::getCamera(const class mrpt::opengl::Scene &) --> class mrpt::opengl::CCamera &", pybind11::return_value_policy::automatic, pybind11::arg("scene"));
		cl.def("render_RGB", (void (mrpt::opengl::CFBORender::*)(const class mrpt::opengl::Scene &, class mrpt::img::CImage &)) &mrpt::opengl::CFBORender::render_RGB, "Render the scene and get the rendered RGB image. Resizes the image\n  buffer if necessary to the configured render resolution.\n\n  \n render_RGBD()\n\nC++: mrpt::opengl::CFBORender::render_RGB(const class mrpt::opengl::Scene &, class mrpt::img::CImage &) --> void", pybind11::arg("scene"), pybind11::arg("outRGB"));
		cl.def("render_RGBD", (void (mrpt::opengl::CFBORender::*)(const class mrpt::opengl::Scene &, class mrpt::img::CImage &, class mrpt::math::CMatrixDynamic<float> &)) &mrpt::opengl::CFBORender::render_RGBD, "Render the scene and get the rendered RGB and depth images.\n Resizes the provided buffers if necessary to the configured render\n resolution.\n The output depth image is in linear depth distance units (e.g. \"meters\").\n Note that values is depth, not range, that is, it's the \"+z\" coordinate\n of a point as seen from the camera, with +Z pointing forward in the view\n direction (the common convention in computer vision).\n Pixels without any observed object in the valid viewport {clipMin,\n clipMax} range will be returned with a range of `0.0`.\n\n  \n render_RGB(), Parameters::raw_depth\n\nC++: mrpt::opengl::CFBORender::render_RGBD(const class mrpt::opengl::Scene &, class mrpt::img::CImage &, class mrpt::math::CMatrixDynamic<float> &) --> void", pybind11::arg("scene"), pybind11::arg("outRGB"), pybind11::arg("outDepth"));
		cl.def("render_depth", (void (mrpt::opengl::CFBORender::*)(const class mrpt::opengl::Scene &, class mrpt::math::CMatrixDynamic<float> &)) &mrpt::opengl::CFBORender::render_depth, "Like render_RGBD(), but only renders the depth image.\n  \n\n render_RGBD()\n\nC++: mrpt::opengl::CFBORender::render_depth(const class mrpt::opengl::Scene &, class mrpt::math::CMatrixDynamic<float> &) --> void", pybind11::arg("scene"), pybind11::arg("outDepth"));

		{ // mrpt::opengl::CFBORender::Parameters file:mrpt/opengl/CFBORender.h line:41
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::opengl::CFBORender::Parameters, std::shared_ptr<mrpt::opengl::CFBORender::Parameters>> cl(enclosing_class, "Parameters", "Parameters for CFBORender constructor ");
			cl.def( pybind11::init( [](){ return new mrpt::opengl::CFBORender::Parameters(); } ) );
			cl.def( pybind11::init<unsigned int, unsigned int>(), pybind11::arg("Width"), pybind11::arg("Height") );

			cl.def( pybind11::init( [](mrpt::opengl::CFBORender::Parameters const &o){ return new mrpt::opengl::CFBORender::Parameters(o); } ) );
			cl.def_readwrite("width", &mrpt::opengl::CFBORender::Parameters::width);
			cl.def_readwrite("height", &mrpt::opengl::CFBORender::Parameters::height);
			cl.def_readwrite("raw_depth", &mrpt::opengl::CFBORender::Parameters::raw_depth);
			cl.def_readwrite("create_EGL_context", &mrpt::opengl::CFBORender::Parameters::create_EGL_context);
			cl.def_readwrite("deviceIndexToUse", &mrpt::opengl::CFBORender::Parameters::deviceIndexToUse);
			cl.def_readwrite("blueSize", &mrpt::opengl::CFBORender::Parameters::blueSize);
			cl.def_readwrite("redSize", &mrpt::opengl::CFBORender::Parameters::redSize);
			cl.def_readwrite("greenSize", &mrpt::opengl::CFBORender::Parameters::greenSize);
			cl.def_readwrite("depthSize", &mrpt::opengl::CFBORender::Parameters::depthSize);
			cl.def_readwrite("conformantOpenGLES2", &mrpt::opengl::CFBORender::Parameters::conformantOpenGLES2);
			cl.def_readwrite("renderableOpenGLES2", &mrpt::opengl::CFBORender::Parameters::renderableOpenGLES2);
			cl.def_readwrite("bindOpenGLES_API", &mrpt::opengl::CFBORender::Parameters::bindOpenGLES_API);
			cl.def_readwrite("contextMajorVersion", &mrpt::opengl::CFBORender::Parameters::contextMajorVersion);
			cl.def_readwrite("contextMinorVersion", &mrpt::opengl::CFBORender::Parameters::contextMinorVersion);
			cl.def_readwrite("contextDebug", &mrpt::opengl::CFBORender::Parameters::contextDebug);
			cl.def("assign", (struct mrpt::opengl::CFBORender::Parameters & (mrpt::opengl::CFBORender::Parameters::*)(const struct mrpt::opengl::CFBORender::Parameters &)) &mrpt::opengl::CFBORender::Parameters::operator=, "C++: mrpt::opengl::CFBORender::Parameters::operator=(const struct mrpt::opengl::CFBORender::Parameters &) --> struct mrpt::opengl::CFBORender::Parameters &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
}
