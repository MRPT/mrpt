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
#include <mrpt/opengl/CEllipsoid2D.h>
#include <mrpt/opengl/CEllipsoid3D.h>
#include <mrpt/opengl/CFrustum.h>
#include <mrpt/opengl/CGeneralizedEllipsoidTemplate.h>
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

// mrpt::opengl::CEllipsoid2D file:mrpt/opengl/CEllipsoid2D.h line:33
struct PyCallBack_mrpt_opengl_CEllipsoid2D : public mrpt::opengl::CEllipsoid2D {
	using mrpt::opengl::CEllipsoid2D::CEllipsoid2D;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoid2D *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CEllipsoid2D::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoid2D *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CEllipsoid2D::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoid2D *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CEllipsoid2D::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoid2D *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CEllipsoid2D::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoid2D *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CEllipsoid2D::serializeFrom(a0, a1);
	}
	bool traceRay(const class mrpt::poses::CPose3D & a0, double & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoid2D *>(this), "traceRay");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CEllipsoid2D::traceRay(a0, a1);
	}
	void freeOpenGLResources() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoid2D *>(this), "freeOpenGLResources");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoid2D *>(this), "renderUpdateBuffers");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoid2D *>(this), "onUpdateBuffers_Wireframe");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoid2D *>(this), "onUpdateBuffers_Triangles");
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
	struct mrpt::math::TBoundingBox_<float> internalBoundingBoxLocal() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoid2D *>(this), "internalBoundingBoxLocal");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoid2D *>(this), "setColorA_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoid2D *>(this), "setColor_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoid2D *>(this), "cullElegible");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoid2D *>(this), "toYAMLMap");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoid2D *>(this), "isCompositeObject");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoid2D *>(this), "getLocalRepresentativePoint");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoid2D *>(this), "initializeTextures");
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

// mrpt::opengl::CEllipsoid3D file:mrpt/opengl/CEllipsoid3D.h line:32
struct PyCallBack_mrpt_opengl_CEllipsoid3D : public mrpt::opengl::CEllipsoid3D {
	using mrpt::opengl::CEllipsoid3D::CEllipsoid3D;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoid3D *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CEllipsoid3D::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoid3D *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CEllipsoid3D::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoid3D *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CEllipsoid3D::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoid3D *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CEllipsoid3D::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoid3D *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CEllipsoid3D::serializeFrom(a0, a1);
	}
	bool traceRay(const class mrpt::poses::CPose3D & a0, double & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoid3D *>(this), "traceRay");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CEllipsoid3D::traceRay(a0, a1);
	}
	void freeOpenGLResources() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoid3D *>(this), "freeOpenGLResources");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoid3D *>(this), "renderUpdateBuffers");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoid3D *>(this), "onUpdateBuffers_Wireframe");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoid3D *>(this), "onUpdateBuffers_Triangles");
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
	struct mrpt::math::TBoundingBox_<float> internalBoundingBoxLocal() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoid3D *>(this), "internalBoundingBoxLocal");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoid3D *>(this), "setColorA_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoid3D *>(this), "setColor_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoid3D *>(this), "cullElegible");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoid3D *>(this), "toYAMLMap");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoid3D *>(this), "isCompositeObject");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoid3D *>(this), "getLocalRepresentativePoint");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CEllipsoid3D *>(this), "initializeTextures");
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

// mrpt::opengl::CFrustum file:mrpt/opengl/CFrustum.h line:47
struct PyCallBack_mrpt_opengl_CFrustum : public mrpt::opengl::CFrustum {
	using mrpt::opengl::CFrustum::CFrustum;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CFrustum *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CFrustum::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CFrustum *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CFrustum::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CFrustum *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CFrustum::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CFrustum *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CFrustum::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CFrustum *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CFrustum::serializeFrom(a0, a1);
	}
	void renderUpdateBuffers() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CFrustum *>(this), "renderUpdateBuffers");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CFrustum::renderUpdateBuffers();
	}
	void freeOpenGLResources() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CFrustum *>(this), "freeOpenGLResources");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CFrustum::freeOpenGLResources();
	}
	void onUpdateBuffers_Wireframe() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CFrustum *>(this), "onUpdateBuffers_Wireframe");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CFrustum::onUpdateBuffers_Wireframe();
	}
	void onUpdateBuffers_Triangles() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CFrustum *>(this), "onUpdateBuffers_Triangles");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CFrustum::onUpdateBuffers_Triangles();
	}
	bool traceRay(const class mrpt::poses::CPose3D & a0, double & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CFrustum *>(this), "traceRay");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CFrustum::traceRay(a0, a1);
	}
	struct mrpt::math::TBoundingBox_<float> internalBoundingBoxLocal() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CFrustum *>(this), "internalBoundingBoxLocal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TBoundingBox_<float>>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TBoundingBox_<float>> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TBoundingBox_<float>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TBoundingBox_<float>>(std::move(o));
		}
		return CFrustum::internalBoundingBoxLocal();
	}
	class mrpt::opengl::CRenderizable & setColorA_u8(const unsigned char a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CFrustum *>(this), "setColorA_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CFrustum *>(this), "setColor_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CFrustum *>(this), "cullElegible");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CFrustum *>(this), "toYAMLMap");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CFrustum *>(this), "isCompositeObject");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CFrustum *>(this), "getLocalRepresentativePoint");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CFrustum *>(this), "initializeTextures");
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

void bind_mrpt_opengl_CGeneralizedEllipsoidTemplate(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::opengl::CGeneralizedEllipsoidTemplate file:mrpt/opengl/CGeneralizedEllipsoidTemplate.h line:38
		pybind11::class_<mrpt::opengl::CGeneralizedEllipsoidTemplate<2>, mrpt::opengl::CGeneralizedEllipsoidTemplate<2>*, mrpt::opengl::CRenderizableShaderTriangles, mrpt::opengl::CRenderizableShaderWireFrame> cl(M("mrpt::opengl"), "CGeneralizedEllipsoidTemplate_2_t", "");
		cl.def("freeOpenGLResources", (void (mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::*)()) &mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::freeOpenGLResources, "C++: mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::freeOpenGLResources() --> void");
		cl.def("renderUpdateBuffers", (void (mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::*)() const) &mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::renderUpdateBuffers, "C++: mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::renderUpdateBuffers() const --> void");
		cl.def("onUpdateBuffers_Wireframe", (void (mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::*)()) &mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::onUpdateBuffers_Wireframe, "C++: mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::onUpdateBuffers_Wireframe() --> void");
		cl.def("onUpdateBuffers_Triangles", (void (mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::*)()) &mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::onUpdateBuffers_Triangles, "C++: mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::onUpdateBuffers_Triangles() --> void");
		cl.def("getCovMatrix", (class mrpt::math::CMatrixFixed<double, 2, 2> (mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::*)() const) &mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::getCovMatrix, "C++: mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::getCovMatrix() const --> class mrpt::math::CMatrixFixed<double, 2, 2>");
		cl.def("setQuantiles", (void (mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::*)(float)) &mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::setQuantiles, "C++: mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::setQuantiles(float) --> void", pybind11::arg("q"));
		cl.def("getQuantiles", (float (mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::*)() const) &mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::getQuantiles, "C++: mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::getQuantiles() const --> float");
		cl.def("setNumberOfSegments", (void (mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::*)(const unsigned int)) &mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::setNumberOfSegments, "C++: mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::setNumberOfSegments(const unsigned int) --> void", pybind11::arg("numSegments"));
		cl.def("getNumberOfSegments", (uint32_t (mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::*)() const) &mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::getNumberOfSegments, "C++: mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::getNumberOfSegments() const --> uint32_t");
		cl.def("enableDrawSolid3D", (void (mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::*)(bool)) &mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::enableDrawSolid3D, "C++: mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::enableDrawSolid3D(bool) --> void", pybind11::arg("v"));
		cl.def("traceRay", (bool (mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::*)(const class mrpt::poses::CPose3D &, double &) const) &mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::traceRay, "C++: mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::traceRay(const class mrpt::poses::CPose3D &, double &) const --> bool", pybind11::arg("o"), pybind11::arg("dist"));
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::*)() const) &mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::GetRuntimeClass, "C++: mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("assign", (class mrpt::opengl::CGeneralizedEllipsoidTemplate<2> & (mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::*)(const class mrpt::opengl::CGeneralizedEllipsoidTemplate<2> &)) &mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::operator=, "C++: mrpt::opengl::CGeneralizedEllipsoidTemplate<2>::operator=(const class mrpt::opengl::CGeneralizedEllipsoidTemplate<2> &) --> class mrpt::opengl::CGeneralizedEllipsoidTemplate<2> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::opengl::CGeneralizedEllipsoidTemplate file:mrpt/opengl/CGeneralizedEllipsoidTemplate.h line:38
		pybind11::class_<mrpt::opengl::CGeneralizedEllipsoidTemplate<3>, mrpt::opengl::CGeneralizedEllipsoidTemplate<3>*, mrpt::opengl::CRenderizableShaderTriangles, mrpt::opengl::CRenderizableShaderWireFrame> cl(M("mrpt::opengl"), "CGeneralizedEllipsoidTemplate_3_t", "");
		cl.def("setCovMatrixAndMean", (void (mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::*)(const class mrpt::math::CMatrixFixed<double, 3, 3> &, const class mrpt::math::CMatrixFixed<double, 3, 1> &)) &mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::setCovMatrixAndMean<mrpt::math::CMatrixFixed<double, 3, 3>,mrpt::math::CMatrixFixed<double, 3, 1>>, "C++: mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::setCovMatrixAndMean(const class mrpt::math::CMatrixFixed<double, 3, 3> &, const class mrpt::math::CMatrixFixed<double, 3, 1> &) --> void", pybind11::arg("new_cov"), pybind11::arg("new_mean"));
		cl.def("setCovMatrix", (void (mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::*)(const class mrpt::math::CMatrixFixed<double, 3, 3> &)) &mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::setCovMatrix<mrpt::math::CMatrixFixed<double, 3, 3>>, "C++: mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::setCovMatrix(const class mrpt::math::CMatrixFixed<double, 3, 3> &) --> void", pybind11::arg("new_cov"));
		cl.def("freeOpenGLResources", (void (mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::*)()) &mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::freeOpenGLResources, "C++: mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::freeOpenGLResources() --> void");
		cl.def("renderUpdateBuffers", (void (mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::*)() const) &mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::renderUpdateBuffers, "C++: mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::renderUpdateBuffers() const --> void");
		cl.def("onUpdateBuffers_Wireframe", (void (mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::*)()) &mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::onUpdateBuffers_Wireframe, "C++: mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::onUpdateBuffers_Wireframe() --> void");
		cl.def("onUpdateBuffers_Triangles", (void (mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::*)()) &mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::onUpdateBuffers_Triangles, "C++: mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::onUpdateBuffers_Triangles() --> void");
		cl.def("getCovMatrix", (class mrpt::math::CMatrixFixed<double, 3, 3> (mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::*)() const) &mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::getCovMatrix, "C++: mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::getCovMatrix() const --> class mrpt::math::CMatrixFixed<double, 3, 3>");
		cl.def("setQuantiles", (void (mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::*)(float)) &mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::setQuantiles, "C++: mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::setQuantiles(float) --> void", pybind11::arg("q"));
		cl.def("getQuantiles", (float (mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::*)() const) &mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::getQuantiles, "C++: mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::getQuantiles() const --> float");
		cl.def("setNumberOfSegments", (void (mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::*)(const unsigned int)) &mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::setNumberOfSegments, "C++: mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::setNumberOfSegments(const unsigned int) --> void", pybind11::arg("numSegments"));
		cl.def("getNumberOfSegments", (uint32_t (mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::*)() const) &mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::getNumberOfSegments, "C++: mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::getNumberOfSegments() const --> uint32_t");
		cl.def("enableDrawSolid3D", (void (mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::*)(bool)) &mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::enableDrawSolid3D, "C++: mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::enableDrawSolid3D(bool) --> void", pybind11::arg("v"));
		cl.def("traceRay", (bool (mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::*)(const class mrpt::poses::CPose3D &, double &) const) &mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::traceRay, "C++: mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::traceRay(const class mrpt::poses::CPose3D &, double &) const --> bool", pybind11::arg("o"), pybind11::arg("dist"));
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::*)() const) &mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::GetRuntimeClass, "C++: mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("assign", (class mrpt::opengl::CGeneralizedEllipsoidTemplate<3> & (mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::*)(const class mrpt::opengl::CGeneralizedEllipsoidTemplate<3> &)) &mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::operator=, "C++: mrpt::opengl::CGeneralizedEllipsoidTemplate<3>::operator=(const class mrpt::opengl::CGeneralizedEllipsoidTemplate<3> &) --> class mrpt::opengl::CGeneralizedEllipsoidTemplate<3> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::opengl::CEllipsoid2D file:mrpt/opengl/CEllipsoid2D.h line:33
		pybind11::class_<mrpt::opengl::CEllipsoid2D, std::shared_ptr<mrpt::opengl::CEllipsoid2D>, PyCallBack_mrpt_opengl_CEllipsoid2D, mrpt::opengl::CGeneralizedEllipsoidTemplate<2>> cl(M("mrpt::opengl"), "CEllipsoid2D", "A 2D ellipse on the XY plane, centered at the origin of this object pose.\n\n The color is determined by the RGBA fields in the class \"CRenderizable\".\n Note that a transparent ellipse can be drawn for \"0<alpha<1\" values.\n If any of the eigen value of the covariance matrix of the ellipsoid is\n zero, it will not be rendered.\n\n Please read the documentation of\n CGeneralizedEllipsoidTemplate::setQuantiles() for learning\n the mathematical details about setting the desired confidence interval.\n\n ![mrpt::opengl::CEllipsoid2D](preview_CEllipsoid.png)\n\n  \n opengl::Scene\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::CEllipsoid2D(); }, [](){ return new PyCallBack_mrpt_opengl_CEllipsoid2D(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_opengl_CEllipsoid2D const &o){ return new PyCallBack_mrpt_opengl_CEllipsoid2D(o); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::CEllipsoid2D const &o){ return new mrpt::opengl::CEllipsoid2D(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CEllipsoid2D::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CEllipsoid2D::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CEllipsoid2D::*)() const) &mrpt::opengl::CEllipsoid2D::GetRuntimeClass, "C++: mrpt::opengl::CEllipsoid2D::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::opengl::CEllipsoid2D::*)() const) &mrpt::opengl::CEllipsoid2D::clone, "C++: mrpt::opengl::CEllipsoid2D::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::opengl::CEllipsoid2D::CreateObject, "C++: mrpt::opengl::CEllipsoid2D::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("set2DsegmentsCount", (void (mrpt::opengl::CEllipsoid2D::*)(unsigned int)) &mrpt::opengl::CEllipsoid2D::set2DsegmentsCount, "The number of segments of a 2D ellipse (default=20) \n\nC++: mrpt::opengl::CEllipsoid2D::set2DsegmentsCount(unsigned int) --> void", pybind11::arg("N"));
		cl.def("traceRay", (bool (mrpt::opengl::CEllipsoid2D::*)(const class mrpt::poses::CPose3D &, double &) const) &mrpt::opengl::CEllipsoid2D::traceRay, "Ray tracing \n\nC++: mrpt::opengl::CEllipsoid2D::traceRay(const class mrpt::poses::CPose3D &, double &) const --> bool", pybind11::arg("o"), pybind11::arg("dist"));
		cl.def("assign", (class mrpt::opengl::CEllipsoid2D & (mrpt::opengl::CEllipsoid2D::*)(const class mrpt::opengl::CEllipsoid2D &)) &mrpt::opengl::CEllipsoid2D::operator=, "C++: mrpt::opengl::CEllipsoid2D::operator=(const class mrpt::opengl::CEllipsoid2D &) --> class mrpt::opengl::CEllipsoid2D &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::opengl::CEllipsoid3D file:mrpt/opengl/CEllipsoid3D.h line:32
		pybind11::class_<mrpt::opengl::CEllipsoid3D, std::shared_ptr<mrpt::opengl::CEllipsoid3D>, PyCallBack_mrpt_opengl_CEllipsoid3D, mrpt::opengl::CGeneralizedEllipsoidTemplate<3>> cl(M("mrpt::opengl"), "CEllipsoid3D", "A 3D ellipsoid, centered at zero with respect to this object pose.\n The color is determined by the RGBA fields in the class \"CRenderizable\".\n Note that a transparent ellipsoid can be drawn for \"0<alpha<1\" values.\n If any of the eigen values of the covariance matrix of the ellipsoid is\n zero, nothing will be rendered.\n\n Please read the documentation of\n CGeneralizedEllipsoidTemplate::setQuantiles() for learning\n the mathematical details about setting the desired confidence interval.\n\n ![mrpt::opengl::CEllipsoid3D](preview_CEllipsoid.png)\n\n  \n opengl::Scene\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::CEllipsoid3D(); }, [](){ return new PyCallBack_mrpt_opengl_CEllipsoid3D(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_opengl_CEllipsoid3D const &o){ return new PyCallBack_mrpt_opengl_CEllipsoid3D(o); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::CEllipsoid3D const &o){ return new mrpt::opengl::CEllipsoid3D(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CEllipsoid3D::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CEllipsoid3D::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CEllipsoid3D::*)() const) &mrpt::opengl::CEllipsoid3D::GetRuntimeClass, "C++: mrpt::opengl::CEllipsoid3D::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::opengl::CEllipsoid3D::*)() const) &mrpt::opengl::CEllipsoid3D::clone, "C++: mrpt::opengl::CEllipsoid3D::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::opengl::CEllipsoid3D::CreateObject, "C++: mrpt::opengl::CEllipsoid3D::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("set3DsegmentsCount", (void (mrpt::opengl::CEllipsoid3D::*)(unsigned int)) &mrpt::opengl::CEllipsoid3D::set3DsegmentsCount, "The number of segments of a 3D ellipse (in both \"axes\") (default=20) \n\nC++: mrpt::opengl::CEllipsoid3D::set3DsegmentsCount(unsigned int) --> void", pybind11::arg("N"));
		cl.def("traceRay", (bool (mrpt::opengl::CEllipsoid3D::*)(const class mrpt::poses::CPose3D &, double &) const) &mrpt::opengl::CEllipsoid3D::traceRay, "Ray tracing \n\nC++: mrpt::opengl::CEllipsoid3D::traceRay(const class mrpt::poses::CPose3D &, double &) const --> bool", pybind11::arg("o"), pybind11::arg("dist"));
		cl.def("assign", (class mrpt::opengl::CEllipsoid3D & (mrpt::opengl::CEllipsoid3D::*)(const class mrpt::opengl::CEllipsoid3D &)) &mrpt::opengl::CEllipsoid3D::operator=, "C++: mrpt::opengl::CEllipsoid3D::operator=(const class mrpt::opengl::CEllipsoid3D &) --> class mrpt::opengl::CEllipsoid3D &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::opengl::CFrustum file:mrpt/opengl/CFrustum.h line:47
		pybind11::class_<mrpt::opengl::CFrustum, std::shared_ptr<mrpt::opengl::CFrustum>, PyCallBack_mrpt_opengl_CFrustum, mrpt::opengl::CRenderizableShaderTriangles, mrpt::opengl::CRenderizableShaderWireFrame> cl(M("mrpt::opengl"), "CFrustum", "A solid or wireframe frustum in 3D (a rectangular truncated pyramid), with\n arbitrary (possibly assymetric) field-of-view angles.\n\n  You can switch whether to show only the lines, the surface of the frustum,\n or both.\n  By default only the lines are drawn.\n\n  The color of the object (via CRenderizable::setColor()) affects the color\n of lines.\n  To set the color of planes use \n\n  As usual in MRPT, the +X axis is assumed to by the main direction, in this\n case of the pyramid axis.\n\n  The horizontal and vertical FOVs can be set directly with \n and  if\n  they are symmetric, or with  and \n otherwise.\n\n  All FOV angles are positive numbers. FOVs must be below 90deg on each side\n (below 180deg in total).\n  If you try to set FOVs to larger values they'll truncated to 89.9deg.\n\n ![mrpt::opengl::CFrustum](preview_CFrustum.png)\n\n \n opengl::Scene,opengl::CRenderizable\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::CFrustum(); }, [](){ return new PyCallBack_mrpt_opengl_CFrustum(); } ) );
		cl.def( pybind11::init<float, float, float, float, float, bool, bool>(), pybind11::arg("near_distance"), pybind11::arg("far_distance"), pybind11::arg("horz_FOV_degrees"), pybind11::arg("vert_FOV_degrees"), pybind11::arg("lineWidth"), pybind11::arg("draw_lines"), pybind11::arg("draw_planes") );

		cl.def( pybind11::init( [](const class mrpt::img::TCamera & a0){ return new mrpt::opengl::CFrustum(a0); }, [](const class mrpt::img::TCamera & a0){ return new PyCallBack_mrpt_opengl_CFrustum(a0); } ), "doc");
		cl.def( pybind11::init<const class mrpt::img::TCamera &, double>(), pybind11::arg("intrinsics"), pybind11::arg("focalDistScale") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_opengl_CFrustum const &o){ return new PyCallBack_mrpt_opengl_CFrustum(o); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::CFrustum const &o){ return new mrpt::opengl::CFrustum(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CFrustum::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CFrustum::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CFrustum::*)() const) &mrpt::opengl::CFrustum::GetRuntimeClass, "C++: mrpt::opengl::CFrustum::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::opengl::CFrustum::*)() const) &mrpt::opengl::CFrustum::clone, "C++: mrpt::opengl::CFrustum::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::opengl::CFrustum::CreateObject, "C++: mrpt::opengl::CFrustum::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("renderUpdateBuffers", (void (mrpt::opengl::CFrustum::*)() const) &mrpt::opengl::CFrustum::renderUpdateBuffers, "C++: mrpt::opengl::CFrustum::renderUpdateBuffers() const --> void");
		cl.def("freeOpenGLResources", (void (mrpt::opengl::CFrustum::*)()) &mrpt::opengl::CFrustum::freeOpenGLResources, "C++: mrpt::opengl::CFrustum::freeOpenGLResources() --> void");
		cl.def("onUpdateBuffers_Wireframe", (void (mrpt::opengl::CFrustum::*)()) &mrpt::opengl::CFrustum::onUpdateBuffers_Wireframe, "C++: mrpt::opengl::CFrustum::onUpdateBuffers_Wireframe() --> void");
		cl.def("onUpdateBuffers_Triangles", (void (mrpt::opengl::CFrustum::*)()) &mrpt::opengl::CFrustum::onUpdateBuffers_Triangles, "C++: mrpt::opengl::CFrustum::onUpdateBuffers_Triangles() --> void");
		cl.def("setPlaneColor", (void (mrpt::opengl::CFrustum::*)(const struct mrpt::img::TColor &)) &mrpt::opengl::CFrustum::setPlaneColor, "Changes the color of the planes; to change color of lines, use\n CRenderizable base methods. \n\nC++: mrpt::opengl::CFrustum::setPlaneColor(const struct mrpt::img::TColor &) --> void", pybind11::arg("c"));
		cl.def("getPlaneColor", (const struct mrpt::img::TColor & (mrpt::opengl::CFrustum::*)() const) &mrpt::opengl::CFrustum::getPlaneColor, "C++: mrpt::opengl::CFrustum::getPlaneColor() const --> const struct mrpt::img::TColor &", pybind11::return_value_policy::automatic);
		cl.def("setNearFarPlanes", (void (mrpt::opengl::CFrustum::*)(const float, const float)) &mrpt::opengl::CFrustum::setNearFarPlanes, "Changes distance of near & far planes \n\nC++: mrpt::opengl::CFrustum::setNearFarPlanes(const float, const float) --> void", pybind11::arg("near_distance"), pybind11::arg("far_distance"));
		cl.def("getNearPlaneDistance", (float (mrpt::opengl::CFrustum::*)() const) &mrpt::opengl::CFrustum::getNearPlaneDistance, "C++: mrpt::opengl::CFrustum::getNearPlaneDistance() const --> float");
		cl.def("getFarPlaneDistance", (float (mrpt::opengl::CFrustum::*)() const) &mrpt::opengl::CFrustum::getFarPlaneDistance, "C++: mrpt::opengl::CFrustum::getFarPlaneDistance() const --> float");
		cl.def("setHorzFOV", (void (mrpt::opengl::CFrustum::*)(const float)) &mrpt::opengl::CFrustum::setHorzFOV, "Changes horizontal FOV (symmetric) \n\nC++: mrpt::opengl::CFrustum::setHorzFOV(const float) --> void", pybind11::arg("fov_horz_degrees"));
		cl.def("setVertFOV", (void (mrpt::opengl::CFrustum::*)(const float)) &mrpt::opengl::CFrustum::setVertFOV, "Changes vertical FOV (symmetric) \n\nC++: mrpt::opengl::CFrustum::setVertFOV(const float) --> void", pybind11::arg("fov_vert_degrees"));
		cl.def("setHorzFOVAsymmetric", (void (mrpt::opengl::CFrustum::*)(const float, const float)) &mrpt::opengl::CFrustum::setHorzFOVAsymmetric, "Changes horizontal FOV (asymmetric) \n\nC++: mrpt::opengl::CFrustum::setHorzFOVAsymmetric(const float, const float) --> void", pybind11::arg("fov_horz_left_degrees"), pybind11::arg("fov_horz_right_degrees"));
		cl.def("setVertFOVAsymmetric", (void (mrpt::opengl::CFrustum::*)(const float, const float)) &mrpt::opengl::CFrustum::setVertFOVAsymmetric, "Changes vertical FOV (asymmetric) \n\nC++: mrpt::opengl::CFrustum::setVertFOVAsymmetric(const float, const float) --> void", pybind11::arg("fov_vert_down_degrees"), pybind11::arg("fov_vert_up_degrees"));
		cl.def("getHorzFOV", (float (mrpt::opengl::CFrustum::*)() const) &mrpt::opengl::CFrustum::getHorzFOV, "C++: mrpt::opengl::CFrustum::getHorzFOV() const --> float");
		cl.def("getVertFOV", (float (mrpt::opengl::CFrustum::*)() const) &mrpt::opengl::CFrustum::getVertFOV, "C++: mrpt::opengl::CFrustum::getVertFOV() const --> float");
		cl.def("getHorzFOVLeft", (float (mrpt::opengl::CFrustum::*)() const) &mrpt::opengl::CFrustum::getHorzFOVLeft, "C++: mrpt::opengl::CFrustum::getHorzFOVLeft() const --> float");
		cl.def("getHorzFOVRight", (float (mrpt::opengl::CFrustum::*)() const) &mrpt::opengl::CFrustum::getHorzFOVRight, "C++: mrpt::opengl::CFrustum::getHorzFOVRight() const --> float");
		cl.def("getVertFOVDown", (float (mrpt::opengl::CFrustum::*)() const) &mrpt::opengl::CFrustum::getVertFOVDown, "C++: mrpt::opengl::CFrustum::getVertFOVDown() const --> float");
		cl.def("getVertFOVUp", (float (mrpt::opengl::CFrustum::*)() const) &mrpt::opengl::CFrustum::getVertFOVUp, "C++: mrpt::opengl::CFrustum::getVertFOVUp() const --> float");
		cl.def("traceRay", (bool (mrpt::opengl::CFrustum::*)(const class mrpt::poses::CPose3D &, double &) const) &mrpt::opengl::CFrustum::traceRay, "C++: mrpt::opengl::CFrustum::traceRay(const class mrpt::poses::CPose3D &, double &) const --> bool", pybind11::arg("o"), pybind11::arg("dist"));
		cl.def("internalBoundingBoxLocal", (struct mrpt::math::TBoundingBox_<float> (mrpt::opengl::CFrustum::*)() const) &mrpt::opengl::CFrustum::internalBoundingBoxLocal, "C++: mrpt::opengl::CFrustum::internalBoundingBoxLocal() const --> struct mrpt::math::TBoundingBox_<float>");
		cl.def("assign", (class mrpt::opengl::CFrustum & (mrpt::opengl::CFrustum::*)(const class mrpt::opengl::CFrustum &)) &mrpt::opengl::CFrustum::operator=, "C++: mrpt::opengl::CFrustum::operator=(const class mrpt::opengl::CFrustum &) --> class mrpt::opengl::CFrustum &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
