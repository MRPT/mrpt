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
#include <mrpt/math/TPolygon3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TPose3DQuat.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/opengl/CCamera.h>
#include <mrpt/opengl/CEllipsoidRangeBearing2D.h>
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/opengl/CGeneralizedEllipsoidTemplate.h>
#include <mrpt/opengl/CPolyhedron.h>
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

// mrpt::opengl::CPolyhedron file:mrpt/opengl/CPolyhedron.h line:38
struct PyCallBack_mrpt_opengl_CPolyhedron : public mrpt::opengl::CPolyhedron {
	using mrpt::opengl::CPolyhedron::CPolyhedron;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPolyhedron *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPolyhedron::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPolyhedron *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CPolyhedron::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPolyhedron *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CPolyhedron::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPolyhedron *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPolyhedron::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPolyhedron *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPolyhedron::serializeFrom(a0, a1);
	}
	void renderUpdateBuffers() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPolyhedron *>(this), "renderUpdateBuffers");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPolyhedron::renderUpdateBuffers();
	}
	void freeOpenGLResources() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPolyhedron *>(this), "freeOpenGLResources");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPolyhedron::freeOpenGLResources();
	}
	void onUpdateBuffers_Wireframe() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPolyhedron *>(this), "onUpdateBuffers_Wireframe");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPolyhedron::onUpdateBuffers_Wireframe();
	}
	void onUpdateBuffers_Triangles() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPolyhedron *>(this), "onUpdateBuffers_Triangles");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPolyhedron::onUpdateBuffers_Triangles();
	}
	struct mrpt::math::TBoundingBox_<float> internalBoundingBoxLocal() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPolyhedron *>(this), "internalBoundingBoxLocal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TBoundingBox_<float>>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TBoundingBox_<float>> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TBoundingBox_<float>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TBoundingBox_<float>>(std::move(o));
		}
		return CPolyhedron::internalBoundingBoxLocal();
	}
	bool traceRay(const class mrpt::poses::CPose3D & a0, double & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPolyhedron *>(this), "traceRay");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPolyhedron::traceRay(a0, a1);
	}
	class mrpt::opengl::CRenderizable & setColorA_u8(const unsigned char a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPolyhedron *>(this), "setColorA_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPolyhedron *>(this), "setColor_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPolyhedron *>(this), "cullElegible");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPolyhedron *>(this), "toYAMLMap");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPolyhedron *>(this), "isCompositeObject");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPolyhedron *>(this), "getLocalRepresentativePoint");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPolyhedron *>(this), "initializeTextures");
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

void bind_mrpt_opengl_CEllipsoidRangeBearing2D(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::opengl::CEllipsoidRangeBearing2D file:mrpt/opengl/CEllipsoidRangeBearing2D.h line:34
		pybind11::class_<mrpt::opengl::CEllipsoidRangeBearing2D, std::shared_ptr<mrpt::opengl::CEllipsoidRangeBearing2D>, PyCallBack_mrpt_opengl_CEllipsoidRangeBearing2D, mrpt::opengl::CGeneralizedEllipsoidTemplate<2>, mrpt::opengl::CRenderizable> cl(M("mrpt::opengl"), "CEllipsoidRangeBearing2D", "An especial \"ellipsoid\" in 2D computed as the uncertainty iso-surfaces of a\n (range,bearing) variable.\n  The parameter space of this ellipsoid comprises these variables (in this\n order):\n   - range: Distance from sensor to feature.\n   - bearing: Angle from +X to the line that goes from the sensor towards the\n feature.\n\n  This class expects you to provide a mean vector of length 2 and a 2x2\n covariance matrix, set with \n\n Please read the documentation of\n CGeneralizedEllipsoidTemplate::setQuantiles() for learning\n  the mathematical details about setting the desired confidence interval.\n\n ![mrpt::opengl::CEllipsoidRangeBearing2D](preview_CEllipsoidRangeBearing2D.png)\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::CEllipsoidRangeBearing2D(); }, [](){ return new PyCallBack_mrpt_opengl_CEllipsoidRangeBearing2D(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_opengl_CEllipsoidRangeBearing2D const &o){ return new PyCallBack_mrpt_opengl_CEllipsoidRangeBearing2D(o); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::CEllipsoidRangeBearing2D const &o){ return new mrpt::opengl::CEllipsoidRangeBearing2D(o); } ) );
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
	{ // mrpt::opengl::CPolyhedron file:mrpt/opengl/CPolyhedron.h line:38
		pybind11::class_<mrpt::opengl::CPolyhedron, std::shared_ptr<mrpt::opengl::CPolyhedron>, PyCallBack_mrpt_opengl_CPolyhedron, mrpt::opengl::CRenderizableShaderWireFrame, mrpt::opengl::CRenderizableShaderTriangles> cl(M("mrpt::opengl"), "CPolyhedron", "This class represents arbitrary polyhedra. The class includes a set of\n static methods to create common polyhedrons. The class includes many methods\n to create standard polyhedra, not intended to be fast but to be simple. For\n example, the dodecahedron is not created efficiently: first, an icosahedron\n is created, and a duality operator is applied to it, which yields the\n dodecahedron. This way, code is much smaller, although much slower. This is\n not a big problem, since polyhedron creation does not usually take a\n significant amount of time (they are created once and rendered many times).\n Polyhedra information and models have been gotten from the Wikipedia,\n https://wikipedia.org\n\n ![mrpt::opengl::CPolyhedron](preview_CPolyhedron.png)\n\n \n opengl::Scene\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::CPolyhedron(); }, [](){ return new PyCallBack_mrpt_opengl_CPolyhedron(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_opengl_CPolyhedron const &o){ return new PyCallBack_mrpt_opengl_CPolyhedron(o); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::CPolyhedron const &o){ return new mrpt::opengl::CPolyhedron(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CPolyhedron::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CPolyhedron::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CPolyhedron::*)() const) &mrpt::opengl::CPolyhedron::GetRuntimeClass, "C++: mrpt::opengl::CPolyhedron::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::opengl::CPolyhedron::*)() const) &mrpt::opengl::CPolyhedron::clone, "C++: mrpt::opengl::CPolyhedron::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::opengl::CPolyhedron::CreateObject, "C++: mrpt::opengl::CPolyhedron::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("renderUpdateBuffers", (void (mrpt::opengl::CPolyhedron::*)() const) &mrpt::opengl::CPolyhedron::renderUpdateBuffers, "C++: mrpt::opengl::CPolyhedron::renderUpdateBuffers() const --> void");
		cl.def("freeOpenGLResources", (void (mrpt::opengl::CPolyhedron::*)()) &mrpt::opengl::CPolyhedron::freeOpenGLResources, "C++: mrpt::opengl::CPolyhedron::freeOpenGLResources() --> void");
		cl.def("onUpdateBuffers_Wireframe", (void (mrpt::opengl::CPolyhedron::*)()) &mrpt::opengl::CPolyhedron::onUpdateBuffers_Wireframe, "C++: mrpt::opengl::CPolyhedron::onUpdateBuffers_Wireframe() --> void");
		cl.def("onUpdateBuffers_Triangles", (void (mrpt::opengl::CPolyhedron::*)()) &mrpt::opengl::CPolyhedron::onUpdateBuffers_Triangles, "C++: mrpt::opengl::CPolyhedron::onUpdateBuffers_Triangles() --> void");
		cl.def("internalBoundingBoxLocal", (struct mrpt::math::TBoundingBox_<float> (mrpt::opengl::CPolyhedron::*)() const) &mrpt::opengl::CPolyhedron::internalBoundingBoxLocal, "Evaluates the bounding box of this object (including possible children)\n in the coordinate frame of the object parent. \n\nC++: mrpt::opengl::CPolyhedron::internalBoundingBoxLocal() const --> struct mrpt::math::TBoundingBox_<float>");
		cl.def_static("CreateTetrahedron", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(double)) &mrpt::opengl::CPolyhedron::CreateTetrahedron, "@{\n\n Creates a regular tetrahedron (see\n  http://en.wikipedia.org/wiki/Tetrahedron). The tetrahedron is created as a\n  triangular pyramid whose edges and vertices are transitive.\n The tetrahedron is the dual to itself.\n  \n \n\n\n  CreatePyramid,CreateJohnsonSolidWithConstantBase,CreateTruncatedTetrahedron\n\nC++: mrpt::opengl::CPolyhedron::CreateTetrahedron(double) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("radius"));
		cl.def_static("CreateHexahedron", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(double)) &mrpt::opengl::CPolyhedron::CreateHexahedron, "Creates a regular cube, also called hexahedron (see\n  http://en.wikipedia.org/wiki/Hexahedron). The hexahedron is created as a\n  cubic prism which transitive edges. Another ways to create it include:\n  Dual to an octahedron.Parallelepiped with three\n  orthogonal, equally-lengthed vectors.Triangular trapezohedron\n  with proper height.\n  \n \n\n\n  CreateOctahedron,getDual,CreateParallelepiped,CreateTrapezohedron,CreateTruncatedHexahedron,CreateTruncatedOctahedron,CreateCuboctahedron,CreateRhombicuboctahedron\n\nC++: mrpt::opengl::CPolyhedron::CreateHexahedron(double) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("radius"));
		cl.def_static("CreateOctahedron", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(double)) &mrpt::opengl::CPolyhedron::CreateOctahedron, "Creates a regular octahedron (see\n  http://en.wikipedia.org/wiki/Octahedron). The octahedron is created as a\n  square bipyramid whit transitive edges and vertices. Another ways to\n  create an octahedron are:\n  Dual to an hexahedronTriangular antiprism with transitive\n  vertices.Conveniently truncated tetrahedron.\n  \n \n\n\n  CreateHexahedron,getDual,CreateArchimedeanAntiprism,CreateTetrahedron,truncate,CreateTruncatedOctahedron,CreateTruncatedHexahedron,CreateCuboctahedron,CreateRhombicuboctahedron\n\nC++: mrpt::opengl::CPolyhedron::CreateOctahedron(double) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("radius"));
		cl.def_static("CreateDodecahedron", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(double)) &mrpt::opengl::CPolyhedron::CreateDodecahedron, "Creates a regular dodecahedron (see\n  http://en.wikipedia.org/wiki/Dodecahedron). The dodecahedron is created as\n  the dual to an icosahedron.\n  \n \n\n\n  CreateIcosahedron,getDual,CreateTruncatedDodecahedron,CreateTruncatedIcosahedron,CreateIcosidodecahedron,CreateRhombicosidodecahedron\n\nC++: mrpt::opengl::CPolyhedron::CreateDodecahedron(double) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("radius"));
		cl.def_static("CreateIcosahedron", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(double)) &mrpt::opengl::CPolyhedron::CreateIcosahedron, "Creates a regular icosahedron (see\n  http://en.wikipedia.org/wiki/Icosahedron). The icosahedron is created as a\n  gyroelongated pentagonal bipyramid with transitive edges, and it's the\n  dual to a dodecahedron.\n  \n \n\n\n  CreateJohnsonSolidWithConstantBase,CreateDodecahedron,getDual,CreateTruncatedIcosahedron,CreateTruncatedDodecahedron,CreateIcosidodecahedron,CreateRhombicosidodecahedron\n\nC++: mrpt::opengl::CPolyhedron::CreateIcosahedron(double) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("radius"));
		cl.def_static("CreateTruncatedTetrahedron", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(double)) &mrpt::opengl::CPolyhedron::CreateTruncatedTetrahedron, "@{\n\n Creates a truncated tetrahedron, consisting of four triangular faces and\n  for hexagonal ones (see\n  http://en.wikipedia.org/wiki/Truncated_tetrahedron). Its dual is the\n  triakis tetrahedron.\n  \n \n\n CreateTetrahedron,CreateTriakisTetrahedron\n\nC++: mrpt::opengl::CPolyhedron::CreateTruncatedTetrahedron(double) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("radius"));
		cl.def_static("CreateCuboctahedron", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(double)) &mrpt::opengl::CPolyhedron::CreateCuboctahedron, "Creates a cuboctahedron, consisting of six square faces and eight\n  triangular ones (see http://en.wikipedia.org/wiki/Cuboctahedron). There\n  are several ways to create a cuboctahedron:\n  Hexahedron truncated to a certain extent.Octahedron\n  truncated to a certain extent.Cantellated\n  tetrahedronDual to a rhombic dodecahedron.\n  \n \n\n\n  CreateHexahedron,CreateOctahedron,truncate,CreateTetrahedron,cantellate,CreateRhombicuboctahedron,CreateRhombicDodecahedron,\n\nC++: mrpt::opengl::CPolyhedron::CreateCuboctahedron(double) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("radius"));
		cl.def_static("CreateTruncatedHexahedron", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(double)) &mrpt::opengl::CPolyhedron::CreateTruncatedHexahedron, "Creates a truncated hexahedron, with six octogonal faces and eight\n  triangular ones (see http://en.wikipedia.org/wiki/Truncated_hexahedron).\n  The truncated octahedron is dual to the triakis octahedron.\n  \n \n\n CreateHexahedron,CreateTriakisOctahedron\n\nC++: mrpt::opengl::CPolyhedron::CreateTruncatedHexahedron(double) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("radius"));
		cl.def_static("CreateTruncatedOctahedron", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(double)) &mrpt::opengl::CPolyhedron::CreateTruncatedOctahedron, "Creates a truncated octahedron, with eight hexagons and eight squares\n  (see http://en.wikipedia.org/wiki/Truncated_octahedron). It's the dual to\n  the tetrakis hexahedron.\n  \n \n\n CreateOctahedron,CreateTetrakisHexahedron\n\nC++: mrpt::opengl::CPolyhedron::CreateTruncatedOctahedron(double) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("radius"));
		cl.def_static("CreateRhombicuboctahedron", [](double const & a0) -> std::shared_ptr<class mrpt::opengl::CPolyhedron> { return mrpt::opengl::CPolyhedron::CreateRhombicuboctahedron(a0); }, "", pybind11::arg("radius"));
		cl.def_static("CreateRhombicuboctahedron", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(double, bool)) &mrpt::opengl::CPolyhedron::CreateRhombicuboctahedron, "Creates a rhombicuboctahedron, with 18 squares and 8 triangles (see\n  http://en.wikipedia.org/wiki/Rhombicuboctahedron), calculated as an\n  elongated square bicupola. It can also be calculated as a cantellated\n  hexahedron or octahedron, and its dual is the deltoidal icositetrahedron.\n If the second argument is set to false, the lower cupola is rotated, so\n  that the objet created is an elongated square gyrobicupola (see\n  http://en.wikipedia.org/wiki/Elongated_square_gyrobicupola). This is not\n  an archimedean solid, but a Johnson one, since it hasn't got vertex\n  transitivity.\n  \n \n\n\n  CreateJohnsonSolidWithConstantBase,CreateHexahedron,CreateOctahedron,cantellate,CreateCuboctahedron,CreateDeltoidalIcositetrahedron\n\nC++: mrpt::opengl::CPolyhedron::CreateRhombicuboctahedron(double, bool) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("radius"), pybind11::arg("type"));
		cl.def_static("CreateIcosidodecahedron", [](double const & a0) -> std::shared_ptr<class mrpt::opengl::CPolyhedron> { return mrpt::opengl::CPolyhedron::CreateIcosidodecahedron(a0); }, "", pybind11::arg("radius"));
		cl.def_static("CreateIcosidodecahedron", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(double, bool)) &mrpt::opengl::CPolyhedron::CreateIcosidodecahedron, "Creates an icosidodecahedron, with 12 pentagons and 20 triangles (see\n  http://en.wikipedia.org/wiki/Icosidodecahedron). Certain truncations of\n  either a dodecahedron or an icosahedron yield an icosidodecahedron.\n The dual of the icosidodecahedron is the rhombic triacontahedron.\n If the second argument is set to false, the lower rotunda is rotated. In\n  this case, the object created is a pentagonal orthobirotunda (see\n  http://en.wikipedia.org/wiki/Pentagonal_orthobirotunda). This object\n  presents symmetry against the XY plane and is not vertex transitive, so\n  it's a Johnson's solid.\n  \n \n\n\n  CreateDodecahedron,CreateIcosahedron,truncate,CreateRhombicosidodecahedron,CreateRhombicTriacontahedron\n\nC++: mrpt::opengl::CPolyhedron::CreateIcosidodecahedron(double, bool) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("radius"), pybind11::arg("type"));
		cl.def_static("CreateTruncatedDodecahedron", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(double)) &mrpt::opengl::CPolyhedron::CreateTruncatedDodecahedron, "Creates a truncated dodecahedron, consisting of 12 dodecagons and 20\n  triangles (see http://en.wikipedia.org/wiki/Truncated_dodecahedron). The\n  truncated dodecahedron is the dual to the triakis icosahedron.\n  \n \n\n CreateDodecahedron,CreateTriakisIcosahedron\n\nC++: mrpt::opengl::CPolyhedron::CreateTruncatedDodecahedron(double) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("radius"));
		cl.def_static("CreateTruncatedIcosahedron", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(double)) &mrpt::opengl::CPolyhedron::CreateTruncatedIcosahedron, "Creates a truncated icosahedron, consisting of 20 hexagons and 12\n  pentagons. This object resembles a typical soccer ball (see\n  http://en.wikipedia.org/wiki/Truncated_icosahedron). The pentakis\n  dodecahedron is the dual to the truncated icosahedron.\n  \n \n\n CreateIcosahedron,CreatePentakisDodecahedron\n\nC++: mrpt::opengl::CPolyhedron::CreateTruncatedIcosahedron(double) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("radius"));
		cl.def_static("CreateRhombicosidodecahedron", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(double)) &mrpt::opengl::CPolyhedron::CreateRhombicosidodecahedron, "Creates a rhombicosidodecahedron, consisting of 30 squares, 12 pentagons\n  and 20 triangles (see\n  http://en.wikipedia.org/wiki/Rhombicosidodecahedron). This object can be\n  obtained as the cantellation of either a dodecahedron or an icosahedron.\n  The dual of the rhombicosidodecahedron is the deltoidal hexecontahedron.\n  \n \n\n\n  CreateDodecahedron,CreateIcosahedron,CreateIcosidodecahedron,CreateDeltoidalHexecontahedron\n\nC++: mrpt::opengl::CPolyhedron::CreateRhombicosidodecahedron(double) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("radius"));
		cl.def_static("CreatePentagonalRotunda", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(double)) &mrpt::opengl::CPolyhedron::CreatePentagonalRotunda, "@{\n\n Creates a pentagonal rotunda (half an icosidodecahedron), consisting of\n six pentagons, ten triangles and a decagon (see\n http://en.wikipedia.org/wiki/Pentagonal_rotunda).\n \n\n CreateIcosidodecahedron,CreateJohnsonSolidWithConstantBase\n\nC++: mrpt::opengl::CPolyhedron::CreatePentagonalRotunda(double) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("radius"));
		cl.def_static("CreateTriakisTetrahedron", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(double)) &mrpt::opengl::CPolyhedron::CreateTriakisTetrahedron, "@{\n\n Creates a triakis tetrahedron, dual to the truncated tetrahedron. This\n  body consists of 12 isosceles triangles (see\n  http://en.wikipedia.org/wiki/Triakis_tetrahedron).\n  \n \n\n CreateTruncatedTetrahedron\n\nC++: mrpt::opengl::CPolyhedron::CreateTriakisTetrahedron(double) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("radius"));
		cl.def_static("CreateRhombicDodecahedron", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(double)) &mrpt::opengl::CPolyhedron::CreateRhombicDodecahedron, "Creates a rhombic dodecahedron, dual to the cuboctahedron. This body\n  consists of 12 rhombi (see\n  http://en.wikipedia.org/wiki/Rhombic_dodecahedron).\n  \n \n\n CreateCuboctahedron\n\nC++: mrpt::opengl::CPolyhedron::CreateRhombicDodecahedron(double) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("radius"));
		cl.def_static("CreateTriakisOctahedron", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(double)) &mrpt::opengl::CPolyhedron::CreateTriakisOctahedron, "Creates a triakis octahedron, dual to the truncated hexahedron. This\n  body consists of 24 isosceles triangles (see\n  http://en.wikipedia.org/wiki/Triakis_octahedron).\n  \n \n\n CreateTruncatedHexahedron\n\nC++: mrpt::opengl::CPolyhedron::CreateTriakisOctahedron(double) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("radius"));
		cl.def_static("CreateTetrakisHexahedron", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(double)) &mrpt::opengl::CPolyhedron::CreateTetrakisHexahedron, "Creates a tetrakis hexahedron, dual to the truncated octahedron. This\n  body consists of 24 isosceles triangles (see\n  http://en.wikipedia.org/wiki/Tetrakis_hexahedron).\n  \n \n\n CreateTruncatedOctahedron\n\nC++: mrpt::opengl::CPolyhedron::CreateTetrakisHexahedron(double) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("radius"));
		cl.def_static("CreateDeltoidalIcositetrahedron", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(double)) &mrpt::opengl::CPolyhedron::CreateDeltoidalIcositetrahedron, "Creates a deltoidal icositetrahedron, dual to the rhombicuboctahedron.\n  This body consists of 24 kites (see\n  http://en.wikipedia.org/wiki/Deltoidal_icositetrahedron).\n  \n \n\n CreateRhombicuboctahedron\n\nC++: mrpt::opengl::CPolyhedron::CreateDeltoidalIcositetrahedron(double) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("radius"));
		cl.def_static("CreateRhombicTriacontahedron", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(double)) &mrpt::opengl::CPolyhedron::CreateRhombicTriacontahedron, "Creates a rhombic triacontahedron, dual to the icosidodecahedron. This\n  body consists of 30 rhombi (see\n  http://en.wikipedia.org/wiki/Rhombic_triacontahedron).\n  \n \n\n CreateIcosidodecahedron\n\nC++: mrpt::opengl::CPolyhedron::CreateRhombicTriacontahedron(double) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("radius"));
		cl.def_static("CreateTriakisIcosahedron", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(double)) &mrpt::opengl::CPolyhedron::CreateTriakisIcosahedron, "Creates a triakis icosahedron, dual to the truncated dodecahedron. This\n  body consists of 60 isosceles triangles\n  http://en.wikipedia.org/wiki/Triakis_icosahedron).\n  \n \n\n CreateTruncatedDodecahedron\n\nC++: mrpt::opengl::CPolyhedron::CreateTriakisIcosahedron(double) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("radius"));
		cl.def_static("CreatePentakisDodecahedron", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(double)) &mrpt::opengl::CPolyhedron::CreatePentakisDodecahedron, "Creates a pentakis dodecahedron, dual to the truncated icosahedron. This\n  body consists of 60 isosceles triangles (see\n  http://en.wikipedia.org/wiki/Pentakis_dodecahedron).\n  \n \n\n CreateTruncatedIcosahedron\n\nC++: mrpt::opengl::CPolyhedron::CreatePentakisDodecahedron(double) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("radius"));
		cl.def_static("CreateDeltoidalHexecontahedron", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(double)) &mrpt::opengl::CPolyhedron::CreateDeltoidalHexecontahedron, "Creates a deltoidal hexecontahedron, dual to the rhombicosidodecahedron.\n  This body consists of 60 kites (see\n  http://en.wikipedia.org/wiki/Deltoidal_hexecontahedron).\n  \n \n\n CreateRhombicosidodecahedron\n\nC++: mrpt::opengl::CPolyhedron::CreateDeltoidalHexecontahedron(double) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("radius"));
		cl.def_static("CreateCubicPrism", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(double, double, double, double, double, double)) &mrpt::opengl::CPolyhedron::CreateCubicPrism, "@{\n\n Creates a cubic prism, given the coordinates of two opposite vertices.\n Each edge will be parallel to one of the coordinate axes, although the\n orientation may change by assigning a pose to the object.\n \n\n CreateCubicPrism(const mrpt::math::TPoint3D &,const\n mrpt::math::TPoint3D\n &),CreateParallelepiped,CreateCustomPrism,CreateRegularPrism,CreateArchimedeanRegularPrism\n\nC++: mrpt::opengl::CPolyhedron::CreateCubicPrism(double, double, double, double, double, double) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("x1"), pybind11::arg("x2"), pybind11::arg("y1"), pybind11::arg("y2"), pybind11::arg("z1"), pybind11::arg("z2"));
		cl.def_static("CreateCubicPrism", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &)) &mrpt::opengl::CPolyhedron::CreateCubicPrism, "Creates a cubic prism, given two opposite vertices.\n \n\n\n CreateCubicPrism(double,double,double,double,double,double),CreateParallelepiped,CreateCustomPrism,CreateRegularPrism,CreateArchimedeanRegularPrism\n\nC++: mrpt::opengl::CPolyhedron::CreateCubicPrism(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("p1"), pybind11::arg("p2"));
		cl.def_static("CreateParallelepiped", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &)) &mrpt::opengl::CPolyhedron::CreateParallelepiped, "Creates a parallelepiped, given a base point and three vectors\n represented as points.\n \n\n CreateCubicPrism\n\nC++: mrpt::opengl::CPolyhedron::CreateParallelepiped(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("base"), pybind11::arg("v1"), pybind11::arg("v2"), pybind11::arg("v3"));
		cl.def_static("CreateTrapezohedron", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(uint32_t, double, double)) &mrpt::opengl::CPolyhedron::CreateTrapezohedron, "Creates a trapezohedron, consisting of 2*N kites, where N is the number\n of edges in the base. The base radius controls the polyhedron height,\n whilst the distance between bases affects the height.\n When the number of edges equals 3, the polyhedron is actually a\n parallelepiped, and it can even be a cube.\n\nC++: mrpt::opengl::CPolyhedron::CreateTrapezohedron(uint32_t, double, double) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("numBaseEdges"), pybind11::arg("baseRadius"), pybind11::arg("basesDistance"));
		cl.def_static("CreateRegularAntiprism", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(uint32_t, double, double)) &mrpt::opengl::CPolyhedron::CreateRegularAntiprism, "Creates an antiprism whose base is a regular polygon. The upper base is\n rotated \n\n with respect to the lower one, where N is the\n number of vertices in the base, and thus the lateral triangles are\n isosceles.\n \n\n CreateCustomAntiprism,CreateArchimedeanRegularAntiprism\n\nC++: mrpt::opengl::CPolyhedron::CreateRegularAntiprism(uint32_t, double, double) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("numBaseEdges"), pybind11::arg("baseRadius"), pybind11::arg("height"));
		cl.def_static("CreateRegularPrism", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(uint32_t, double, double)) &mrpt::opengl::CPolyhedron::CreateRegularPrism, "Creates a regular prism whose base is a regular polygon and whose edges\n are either parallel or perpendicular to the XY plane.\n \n\n CreateCubicPrism,CreateCustomPrism,CreateArchimedeanRegularAntiprism\n\nC++: mrpt::opengl::CPolyhedron::CreateRegularPrism(uint32_t, double, double) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("numBaseEdges"), pybind11::arg("baseRadius"), pybind11::arg("height"));
		cl.def_static("CreateRegularPyramid", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(uint32_t, double, double)) &mrpt::opengl::CPolyhedron::CreateRegularPyramid, "Creates a regular pyramid whose base is a regular polygon.\n \n\n CreatePyramid\n\nC++: mrpt::opengl::CPolyhedron::CreateRegularPyramid(uint32_t, double, double) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("numBaseEdges"), pybind11::arg("baseRadius"), pybind11::arg("height"));
		cl.def_static("CreateRegularDoublePyramid", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(uint32_t, double, double, double)) &mrpt::opengl::CPolyhedron::CreateRegularDoublePyramid, "Creates a regular double pyramid whose base is a regular polygon.\n \n\n CreateDoublePyramid\n\nC++: mrpt::opengl::CPolyhedron::CreateRegularDoublePyramid(uint32_t, double, double, double) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("numBaseEdges"), pybind11::arg("baseRadius"), pybind11::arg("height1"), pybind11::arg("height2"));
		cl.def_static("CreateArchimedeanRegularPrism", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(uint32_t, double)) &mrpt::opengl::CPolyhedron::CreateArchimedeanRegularPrism, "Creates a regular prism whose lateral area is comprised of squares, and\n so each face of its is a regular polygon. Due to vertex transitivity, the\n resulting object is always archimedean.\n \n\n CreateRegularPrism,CreateCustomPrism\n\nC++: mrpt::opengl::CPolyhedron::CreateArchimedeanRegularPrism(uint32_t, double) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("numBaseEdges"), pybind11::arg("baseRadius"));
		cl.def_static("CreateArchimedeanRegularAntiprism", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(uint32_t, double)) &mrpt::opengl::CPolyhedron::CreateArchimedeanRegularAntiprism, "Creates a regular antiprism whose lateral polygons are equilateral\n triangles, and so each face of its is a regular polygon. Due to vertex\n transitivity, the resulting object is always archimedean.\n \n\n CreateRegularAntiprism,CreateCustomAntiprism\n\nC++: mrpt::opengl::CPolyhedron::CreateArchimedeanRegularAntiprism(uint32_t, double) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("numBaseEdges"), pybind11::arg("baseRadius"));
		cl.def_static("CreateRegularTruncatedPyramid", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(uint32_t, double, double, double)) &mrpt::opengl::CPolyhedron::CreateRegularTruncatedPyramid, "Creates a regular truncated pyramid whose base is a regular polygon.\n \n\n CreateTruncatedPyramid\n\nC++: mrpt::opengl::CPolyhedron::CreateRegularTruncatedPyramid(uint32_t, double, double, double) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("numBaseEdges"), pybind11::arg("baseRadius"), pybind11::arg("height"), pybind11::arg("ratio"));
		cl.def_static("CreateRegularFrustum", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(uint32_t, double, double, double)) &mrpt::opengl::CPolyhedron::CreateRegularFrustum, "This is a synonym for CreateRegularTruncatedPyramid.\n \n\n CreateRegularTruncatedPyramid\n\nC++: mrpt::opengl::CPolyhedron::CreateRegularFrustum(uint32_t, double, double, double) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("numBaseEdges"), pybind11::arg("baseRadius"), pybind11::arg("height"), pybind11::arg("ratio"));
		cl.def_static("CreateRegularBifrustum", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(uint32_t, double, double, double, double, double)) &mrpt::opengl::CPolyhedron::CreateRegularBifrustum, "Creates a bifrustum (double truncated pyramid) whose base is a regular\n polygon lying in the XY plane.\n \n\n CreateBifrustum\n\nC++: mrpt::opengl::CPolyhedron::CreateRegularBifrustum(uint32_t, double, double, double, double, double) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("numBaseEdges"), pybind11::arg("baseRadius"), pybind11::arg("height1"), pybind11::arg("ratio1"), pybind11::arg("height2"), pybind11::arg("ratio2"));
		cl.def_static("CreateCupola", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(uint32_t, double)) &mrpt::opengl::CPolyhedron::CreateCupola, "Creates a cupola.\n \n\n std::logic_error if the number of edges is odd or less than four.\n\nC++: mrpt::opengl::CPolyhedron::CreateCupola(uint32_t, double) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("numBaseEdges"), pybind11::arg("edgeLength"));
		cl.def_static("CreateCatalanTrapezohedron", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(uint32_t, double)) &mrpt::opengl::CPolyhedron::CreateCatalanTrapezohedron, "Creates a trapezohedron whose dual is exactly an archimedean antiprism.\n Creates a cube if numBaseEdges is equal to 3.\n \n\n Actually resulting height is significantly higher than that passed\n to the algorithm.\n \n\n CreateTrapezohedron,CreateArchimedeanRegularAntiprism,getDual\n\nC++: mrpt::opengl::CPolyhedron::CreateCatalanTrapezohedron(uint32_t, double) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("numBaseEdges"), pybind11::arg("height"));
		cl.def_static("CreateCatalanDoublePyramid", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(uint32_t, double)) &mrpt::opengl::CPolyhedron::CreateCatalanDoublePyramid, "Creates a double pyramid whose dual is exactly an archimedean prism.\n Creates an octahedron if numBaseEdges is equal to 4.\n \n\n Actually resulting height is significantly higher than that passed\n to the algorithm.\n \n\n CreateDoublePyramid,CreateArchimedeanRegularPrism,getDual\n\nC++: mrpt::opengl::CPolyhedron::CreateCatalanDoublePyramid(uint32_t, double) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("numBaseEdges"), pybind11::arg("height"));
		cl.def_static("CreateJohnsonSolidWithConstantBase", [](uint32_t const & a0, double const & a1, const std::string & a2) -> std::shared_ptr<class mrpt::opengl::CPolyhedron> { return mrpt::opengl::CPolyhedron::CreateJohnsonSolidWithConstantBase(a0, a1, a2); }, "", pybind11::arg("numBaseEdges"), pybind11::arg("baseRadius"), pybind11::arg("components"));
		cl.def_static("CreateJohnsonSolidWithConstantBase", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(uint32_t, double, const std::string &, size_t)) &mrpt::opengl::CPolyhedron::CreateJohnsonSolidWithConstantBase, "Creates a series of concatenated solids (most of which are prismatoids)\n  whose base is a regular polygon with a given number of edges. Every face\n  of the resulting body will be a regular polygon, so it is a Johnson solid;\n  in special cases, it may be archimedean or even platonic.\n The shape of the body is defined by the string argument, which can\n  include one or more of the following:\n  <center>\n  StringBodyRestrictions\n  P+Upward pointing pyramidMust be the last\n  object, vertex number cannot surpass 5\n  P-Downward pointing pyramidMust be the first\n  object, vertex number cannot surpass 5\n  C+Upward pointing cupolaMust be the last object,\n  vertex number must be an even number in the range 4-10.\n  C-Downward pointing cupolaMust be the first\n  object, vertex number must be an even number in the range 4-10.\n  GC+Upward pointing cupola, rotatedMust be the\n  last object, vertex number must be an even number in the range\n  4-10.\n  GC-Downward pointing cupola, rotatedMust be the\n  first object, vertex number must be an even number in the range\n  4-10.\n  PRArchimedean prismCannot abut other\n  prism\n  AArchimedean antiprismNone\n  R+Upward pointing rotundaMust be the last\n  object, vertex number must be exactly 10\n  R-Downward pointing rotundaMust be the first\n  object, vertex number must be exactly 10\n  GR+Upward pointing rotunda, rotatedMust be the\n  last object, vertex number must be exactly 10\n  GR-Downward pointing rotundaMust be the first\n  object, vertex number must be exactly 10\n  </center>\n Some examples of bodies are:\n  <center>\n  StringVerticesResulting\n  body\n  P+3Tetrahedron\n  PR4Hexahedron\n  P-P+4Octahedron\n  A3Octahedron\n  C+PRC-\n8Rhombicuboctahedron\n  P-AP+5Icosahedron\n  R-R+10Icosidodecahedron\n  </center>\n\nC++: mrpt::opengl::CPolyhedron::CreateJohnsonSolidWithConstantBase(uint32_t, double, const std::string &, size_t) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("numBaseEdges"), pybind11::arg("baseRadius"), pybind11::arg("components"), pybind11::arg("shifts"));
		cl.def("traceRay", (bool (mrpt::opengl::CPolyhedron::*)(const class mrpt::poses::CPose3D &, double &) const) &mrpt::opengl::CPolyhedron::traceRay, "@}\n\nC++: mrpt::opengl::CPolyhedron::traceRay(const class mrpt::poses::CPose3D &, double &) const --> bool", pybind11::arg("o"), pybind11::arg("dist"));
		cl.def("getNumberOfVertices", (uint32_t (mrpt::opengl::CPolyhedron::*)() const) &mrpt::opengl::CPolyhedron::getNumberOfVertices, "Gets the amount of vertices.\n\nC++: mrpt::opengl::CPolyhedron::getNumberOfVertices() const --> uint32_t");
		cl.def("getNumberOfEdges", (uint32_t (mrpt::opengl::CPolyhedron::*)() const) &mrpt::opengl::CPolyhedron::getNumberOfEdges, "Gets the amount of edges.\n\nC++: mrpt::opengl::CPolyhedron::getNumberOfEdges() const --> uint32_t");
		cl.def("getNumberOfFaces", (uint32_t (mrpt::opengl::CPolyhedron::*)() const) &mrpt::opengl::CPolyhedron::getNumberOfFaces, "Gets the amount of faces.\n\nC++: mrpt::opengl::CPolyhedron::getNumberOfFaces() const --> uint32_t");
		cl.def("getVolume", (double (mrpt::opengl::CPolyhedron::*)() const) &mrpt::opengl::CPolyhedron::getVolume, "Gets the polyhedron volume. Won't work properly if the polyhedron is not\n convex.\n\nC++: mrpt::opengl::CPolyhedron::getVolume() const --> double");
		cl.def("isWireframe", (bool (mrpt::opengl::CPolyhedron::*)() const) &mrpt::opengl::CPolyhedron::isWireframe, "Returns whether the polyhedron will be rendered as a wireframe object.\n\nC++: mrpt::opengl::CPolyhedron::isWireframe() const --> bool");
		cl.def("setWireframe", [](mrpt::opengl::CPolyhedron &o) -> void { return o.setWireframe(); }, "");
		cl.def("setWireframe", (void (mrpt::opengl::CPolyhedron::*)(bool)) &mrpt::opengl::CPolyhedron::setWireframe, "Sets whether the polyhedron will be rendered as a wireframe object.\n\nC++: mrpt::opengl::CPolyhedron::setWireframe(bool) --> void", pybind11::arg("enabled"));
		cl.def("isClosed", (bool (mrpt::opengl::CPolyhedron::*)() const) &mrpt::opengl::CPolyhedron::isClosed, "Returns true if the polygon is a completely closed object.\n\nC++: mrpt::opengl::CPolyhedron::isClosed() const --> bool");
		cl.def("makeConvexPolygons", (void (mrpt::opengl::CPolyhedron::*)()) &mrpt::opengl::CPolyhedron::makeConvexPolygons, "Recomputes polygons, if necessary, so that each one is convex.\n\nC++: mrpt::opengl::CPolyhedron::makeConvexPolygons() --> void");
		cl.def("getCenter", (void (mrpt::opengl::CPolyhedron::*)(struct mrpt::math::TPoint3D_<double> &) const) &mrpt::opengl::CPolyhedron::getCenter, "Gets the center of the polyhedron.\n\nC++: mrpt::opengl::CPolyhedron::getCenter(struct mrpt::math::TPoint3D_<double> &) const --> void", pybind11::arg("center"));
		cl.def_static("CreateRandomPolyhedron", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)(double)) &mrpt::opengl::CPolyhedron::CreateRandomPolyhedron, "Creates a random polyhedron from the static methods.\n\nC++: mrpt::opengl::CPolyhedron::CreateRandomPolyhedron(double) --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("radius"));
		cl.def("getDual", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (mrpt::opengl::CPolyhedron::*)() const) &mrpt::opengl::CPolyhedron::getDual, "@{\n\n Given a polyhedron, creates its dual.\n \n\n truncate,cantellate,augment\n \n\n std::logic_error Can't get the dual to this polyhedron.\n\nC++: mrpt::opengl::CPolyhedron::getDual() const --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>");
		cl.def("truncate", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (mrpt::opengl::CPolyhedron::*)(double) const) &mrpt::opengl::CPolyhedron::truncate, "Truncates a polyhedron to a given factor.\n \n\n getDual,cantellate,augment\n \n\n std::logic_error Polyhedron truncation results in skew polygons\n and thus it's impossible to perform.\n\nC++: mrpt::opengl::CPolyhedron::truncate(double) const --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("factor"));
		cl.def("cantellate", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (mrpt::opengl::CPolyhedron::*)(double) const) &mrpt::opengl::CPolyhedron::cantellate, "Cantellates a polyhedron to a given factor.\n \n\n getDual,truncate,augment\n\nC++: mrpt::opengl::CPolyhedron::cantellate(double) const --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("factor"));
		cl.def("augment", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (mrpt::opengl::CPolyhedron::*)(double) const) &mrpt::opengl::CPolyhedron::augment, "Augments a polyhedron to a given height. This operation is roughly dual\n to the truncation: given a body P, the operation dtdP and aP yield\n resembling results.\n \n\n getDual,truncate,cantellate\n\nC++: mrpt::opengl::CPolyhedron::augment(double) const --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("height"));
		cl.def("augment", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (mrpt::opengl::CPolyhedron::*)(double, size_t) const) &mrpt::opengl::CPolyhedron::augment, "Augments a polyhedron to a given height. This method only affects to\n faces with certain number of vertices.\n \n\n augment(double) const\n\nC++: mrpt::opengl::CPolyhedron::augment(double, size_t) const --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("height"), pybind11::arg("numVertices"));
		cl.def("augment", [](mrpt::opengl::CPolyhedron const &o) -> std::shared_ptr<class mrpt::opengl::CPolyhedron> { return o.augment(); }, "");
		cl.def("augment", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (mrpt::opengl::CPolyhedron::*)(bool) const) &mrpt::opengl::CPolyhedron::augment, "Augments a polyhedron, so that the resulting triangles are equilateral.\n If the argument is true, triangles are \"cut\" from the polyhedron, instead\n of being added.\n \n\n std::logic_error a non-regular face has been found.\n \n\n augment(double) const\n\nC++: mrpt::opengl::CPolyhedron::augment(bool) const --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("direction"));
		cl.def("augment", [](mrpt::opengl::CPolyhedron const &o, size_t const & a0) -> std::shared_ptr<class mrpt::opengl::CPolyhedron> { return o.augment(a0); }, "", pybind11::arg("numVertices"));
		cl.def("augment", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (mrpt::opengl::CPolyhedron::*)(size_t, bool) const) &mrpt::opengl::CPolyhedron::augment, "Augments a polyhedron, so that the resulting triangles are equilateral;\n affects only faces with certain number of faces. If the second argument\n is true, triangles are \"cut\" from the polyhedron.\n \n\n std::logic_error a non-regular face has been found.\n \n\n augment(double) const\n\nC++: mrpt::opengl::CPolyhedron::augment(size_t, bool) const --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("numVertices"), pybind11::arg("direction"));
		cl.def("rotate", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (mrpt::opengl::CPolyhedron::*)(double) const) &mrpt::opengl::CPolyhedron::rotate, "Rotates a polyhedron around the Z axis a given amount of radians. In\nsome cases, this operation may be necessary to view the symmetry between\nrelated objects.\n	\n\n scale\n\nC++: mrpt::opengl::CPolyhedron::rotate(double) const --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("angle"));
		cl.def("scale", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (mrpt::opengl::CPolyhedron::*)(double) const) &mrpt::opengl::CPolyhedron::scale, "Scales a polyhedron to a given factor.\n \n\n std::logic_error factor is not a strictly positive number.\n \n\n rotate\n\nC++: mrpt::opengl::CPolyhedron::scale(double) const --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>", pybind11::arg("factor"));
		cl.def("updatePolygons", (void (mrpt::opengl::CPolyhedron::*)() const) &mrpt::opengl::CPolyhedron::updatePolygons, "@}\n\n Updates the mutable list of polygons used in rendering and ray tracing.\n\nC++: mrpt::opengl::CPolyhedron::updatePolygons() const --> void");
		cl.def_static("CreateEmpty", (class std::shared_ptr<class mrpt::opengl::CPolyhedron> (*)()) &mrpt::opengl::CPolyhedron::CreateEmpty, "Creates an empty Polyhedron. \n\nC++: mrpt::opengl::CPolyhedron::CreateEmpty() --> class std::shared_ptr<class mrpt::opengl::CPolyhedron>");
		cl.def("assign", (class mrpt::opengl::CPolyhedron & (mrpt::opengl::CPolyhedron::*)(const class mrpt::opengl::CPolyhedron &)) &mrpt::opengl::CPolyhedron::operator=, "C++: mrpt::opengl::CPolyhedron::operator=(const class mrpt::opengl::CPolyhedron &) --> class mrpt::opengl::CPolyhedron &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::opengl::CPolyhedron::TPolyhedronEdge file:mrpt/opengl/CPolyhedron.h line:65
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::opengl::CPolyhedron::TPolyhedronEdge, std::shared_ptr<mrpt::opengl::CPolyhedron::TPolyhedronEdge>> cl(enclosing_class, "TPolyhedronEdge", "Struct used to store a polyhedron edge. The struct consists only of two\n vertex indices, used to access the polyhedron vertex list.");
			cl.def( pybind11::init( [](){ return new mrpt::opengl::CPolyhedron::TPolyhedronEdge(); } ) );
			cl.def( pybind11::init( [](mrpt::opengl::CPolyhedron::TPolyhedronEdge const &o){ return new mrpt::opengl::CPolyhedron::TPolyhedronEdge(o); } ) );
			cl.def_readwrite("v1", &mrpt::opengl::CPolyhedron::TPolyhedronEdge::v1);
			cl.def_readwrite("v2", &mrpt::opengl::CPolyhedron::TPolyhedronEdge::v2);
			cl.def("__eq__", (bool (mrpt::opengl::CPolyhedron::TPolyhedronEdge::*)(const struct mrpt::opengl::CPolyhedron::TPolyhedronEdge &) const) &mrpt::opengl::CPolyhedron::TPolyhedronEdge::operator==, "Comparison agains another edge. Simmetry is taken into account.\n\nC++: mrpt::opengl::CPolyhedron::TPolyhedronEdge::operator==(const struct mrpt::opengl::CPolyhedron::TPolyhedronEdge &) const --> bool", pybind11::arg("e"));
			cl.def("assign", (struct mrpt::opengl::CPolyhedron::TPolyhedronEdge & (mrpt::opengl::CPolyhedron::TPolyhedronEdge::*)(const struct mrpt::opengl::CPolyhedron::TPolyhedronEdge &)) &mrpt::opengl::CPolyhedron::TPolyhedronEdge::operator=, "C++: mrpt::opengl::CPolyhedron::TPolyhedronEdge::operator=(const struct mrpt::opengl::CPolyhedron::TPolyhedronEdge &) --> struct mrpt::opengl::CPolyhedron::TPolyhedronEdge &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::opengl::CPolyhedron::TPolyhedronFace file:mrpt/opengl/CPolyhedron.h line:98
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::opengl::CPolyhedron::TPolyhedronFace, std::shared_ptr<mrpt::opengl::CPolyhedron::TPolyhedronFace>> cl(enclosing_class, "TPolyhedronFace", "Struct used to store a polyhedron face. Consists on a set of vertex\n indices and a normal vector.");
			cl.def( pybind11::init( [](){ return new mrpt::opengl::CPolyhedron::TPolyhedronFace(); } ) );
			cl.def( pybind11::init( [](mrpt::opengl::CPolyhedron::TPolyhedronFace const &o){ return new mrpt::opengl::CPolyhedron::TPolyhedronFace(o); } ) );
			cl.def_readwrite("vertices", &mrpt::opengl::CPolyhedron::TPolyhedronFace::vertices);
			cl.def_readwrite("normal", &mrpt::opengl::CPolyhedron::TPolyhedronFace::normal);
			cl.def("assign", (struct mrpt::opengl::CPolyhedron::TPolyhedronFace & (mrpt::opengl::CPolyhedron::TPolyhedronFace::*)(const struct mrpt::opengl::CPolyhedron::TPolyhedronFace &)) &mrpt::opengl::CPolyhedron::TPolyhedronFace::operator=, "C++: mrpt::opengl::CPolyhedron::TPolyhedronFace::operator=(const struct mrpt::opengl::CPolyhedron::TPolyhedronFace &) --> struct mrpt::opengl::CPolyhedron::TPolyhedronFace &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
}
