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
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CSkyBox.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/opengl/CUBE_TEXTURE_FACE.h>
#include <mrpt/opengl/CVectorField2D.h>
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

// mrpt::opengl::CSkyBox file:mrpt/opengl/CSkyBox.h line:31
struct PyCallBack_mrpt_opengl_CSkyBox : public mrpt::opengl::CSkyBox {
	using mrpt::opengl::CSkyBox::CSkyBox;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSkyBox *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CSkyBox::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSkyBox *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CSkyBox::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSkyBox *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CSkyBox::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSkyBox *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSkyBox::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSkyBox *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSkyBox::serializeFrom(a0, a1);
	}
	void renderUpdateBuffers() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSkyBox *>(this), "renderUpdateBuffers");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSkyBox::renderUpdateBuffers();
	}
	void freeOpenGLResources() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSkyBox *>(this), "freeOpenGLResources");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSkyBox::freeOpenGLResources();
	}
	void initializeTextures() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSkyBox *>(this), "initializeTextures");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSkyBox::initializeTextures();
	}
	struct mrpt::math::TBoundingBox_<float> internalBoundingBoxLocal() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSkyBox *>(this), "internalBoundingBoxLocal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TBoundingBox_<float>>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TBoundingBox_<float>> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TBoundingBox_<float>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TBoundingBox_<float>>(std::move(o));
		}
		return CSkyBox::internalBoundingBoxLocal();
	}
	bool cullElegible() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSkyBox *>(this), "cullElegible");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CSkyBox::cullElegible();
	}
	class mrpt::opengl::CRenderizable & setColorA_u8(const unsigned char a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSkyBox *>(this), "setColorA_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSkyBox *>(this), "setColor_u8");
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
	void toYAMLMap(class mrpt::containers::yaml & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSkyBox *>(this), "toYAMLMap");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSkyBox *>(this), "isCompositeObject");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSkyBox *>(this), "traceRay");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSkyBox *>(this), "getLocalRepresentativePoint");
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

// mrpt::opengl::CVectorField2D file:mrpt/opengl/CVectorField2D.h line:29
struct PyCallBack_mrpt_opengl_CVectorField2D : public mrpt::opengl::CVectorField2D {
	using mrpt::opengl::CVectorField2D::CVectorField2D;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CVectorField2D *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CVectorField2D::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CVectorField2D *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CVectorField2D::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CVectorField2D *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CVectorField2D::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CVectorField2D *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CVectorField2D::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CVectorField2D *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CVectorField2D::serializeFrom(a0, a1);
	}
	void renderUpdateBuffers() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CVectorField2D *>(this), "renderUpdateBuffers");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CVectorField2D::renderUpdateBuffers();
	}
	void freeOpenGLResources() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CVectorField2D *>(this), "freeOpenGLResources");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CVectorField2D::freeOpenGLResources();
	}
	void onUpdateBuffers_Wireframe() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CVectorField2D *>(this), "onUpdateBuffers_Wireframe");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CVectorField2D::onUpdateBuffers_Wireframe();
	}
	void onUpdateBuffers_Triangles() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CVectorField2D *>(this), "onUpdateBuffers_Triangles");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CVectorField2D::onUpdateBuffers_Triangles();
	}
	void onUpdateBuffers_Points() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CVectorField2D *>(this), "onUpdateBuffers_Points");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CVectorField2D::onUpdateBuffers_Points();
	}
	struct mrpt::math::TBoundingBox_<float> internalBoundingBoxLocal() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CVectorField2D *>(this), "internalBoundingBoxLocal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TBoundingBox_<float>>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TBoundingBox_<float>> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TBoundingBox_<float>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TBoundingBox_<float>>(std::move(o));
		}
		return CVectorField2D::internalBoundingBoxLocal();
	}
	class mrpt::opengl::CRenderizable & setColorA_u8(const unsigned char a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CVectorField2D *>(this), "setColorA_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CVectorField2D *>(this), "setColor_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CVectorField2D *>(this), "cullElegible");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CVectorField2D *>(this), "toYAMLMap");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CVectorField2D *>(this), "isCompositeObject");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CVectorField2D *>(this), "traceRay");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CVectorField2D *>(this), "getLocalRepresentativePoint");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CVectorField2D *>(this), "initializeTextures");
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

void bind_mrpt_opengl_CUBE_TEXTURE_FACE(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::opengl::CUBE_TEXTURE_FACE file:mrpt/opengl/CUBE_TEXTURE_FACE.h line:27
	pybind11::enum_<mrpt::opengl::CUBE_TEXTURE_FACE>(M("mrpt::opengl"), "CUBE_TEXTURE_FACE", "Enum type for each of the 6 faces of a Cube Texture.\n\n  Note that these enums must be defined in the same order than OpenGL API\n  constants:\n\n  #define GL_TEXTURE_CUBE_MAP_POSITIVE_X    0x8515\n  #define GL_TEXTURE_CUBE_MAP_NEGATIVE_X    0x8516\n  #define GL_TEXTURE_CUBE_MAP_POSITIVE_Y    0x8517\n  #define GL_TEXTURE_CUBE_MAP_NEGATIVE_Y    0x8518\n  #define GL_TEXTURE_CUBE_MAP_POSITIVE_Z    0x8519\n  #define GL_TEXTURE_CUBE_MAP_NEGATIVE_Z    0x851A\n\n \n\n ")
		.value("LEFT", mrpt::opengl::CUBE_TEXTURE_FACE::LEFT)
		.value("RIGHT", mrpt::opengl::CUBE_TEXTURE_FACE::RIGHT)
		.value("TOP", mrpt::opengl::CUBE_TEXTURE_FACE::TOP)
		.value("BOTTOM", mrpt::opengl::CUBE_TEXTURE_FACE::BOTTOM)
		.value("FRONT", mrpt::opengl::CUBE_TEXTURE_FACE::FRONT)
		.value("BACK", mrpt::opengl::CUBE_TEXTURE_FACE::BACK);

;

	{ // mrpt::opengl::CSkyBox file:mrpt/opengl/CSkyBox.h line:31
		pybind11::class_<mrpt::opengl::CSkyBox, std::shared_ptr<mrpt::opengl::CSkyBox>, PyCallBack_mrpt_opengl_CSkyBox, mrpt::opengl::CRenderizable> cl(M("mrpt::opengl"), "CSkyBox", "A Sky Box: 6 textures that are always rendered at \"infinity\" to give the\n  impression of the scene to be much larger.\n\n Refer to example \n\n  />\n\n \n opengl::Scene\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::CSkyBox(); }, [](){ return new PyCallBack_mrpt_opengl_CSkyBox(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_opengl_CSkyBox const &o){ return new PyCallBack_mrpt_opengl_CSkyBox(o); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::CSkyBox const &o){ return new mrpt::opengl::CSkyBox(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CSkyBox::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CSkyBox::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CSkyBox::*)() const) &mrpt::opengl::CSkyBox::GetRuntimeClass, "C++: mrpt::opengl::CSkyBox::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::opengl::CSkyBox::*)() const) &mrpt::opengl::CSkyBox::clone, "C++: mrpt::opengl::CSkyBox::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::opengl::CSkyBox::CreateObject, "C++: mrpt::opengl::CSkyBox::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("renderUpdateBuffers", (void (mrpt::opengl::CSkyBox::*)() const) &mrpt::opengl::CSkyBox::renderUpdateBuffers, "C++: mrpt::opengl::CSkyBox::renderUpdateBuffers() const --> void");
		cl.def("freeOpenGLResources", (void (mrpt::opengl::CSkyBox::*)()) &mrpt::opengl::CSkyBox::freeOpenGLResources, "C++: mrpt::opengl::CSkyBox::freeOpenGLResources() --> void");
		cl.def("initializeTextures", (void (mrpt::opengl::CSkyBox::*)() const) &mrpt::opengl::CSkyBox::initializeTextures, "C++: mrpt::opengl::CSkyBox::initializeTextures() const --> void");
		cl.def("assignImage", (void (mrpt::opengl::CSkyBox::*)(const enum mrpt::opengl::CUBE_TEXTURE_FACE, const class mrpt::img::CImage &)) &mrpt::opengl::CSkyBox::assignImage, "Assigns a texture. It is mandatory to assign all 6 faces before\n initializing/rendering the texture.\n\n \n Images are copied, the original ones can be deleted.\n\nC++: mrpt::opengl::CSkyBox::assignImage(const enum mrpt::opengl::CUBE_TEXTURE_FACE, const class mrpt::img::CImage &) --> void", pybind11::arg("face"), pybind11::arg("img"));
		cl.def("internalBoundingBoxLocal", (struct mrpt::math::TBoundingBox_<float> (mrpt::opengl::CSkyBox::*)() const) &mrpt::opengl::CSkyBox::internalBoundingBoxLocal, "C++: mrpt::opengl::CSkyBox::internalBoundingBoxLocal() const --> struct mrpt::math::TBoundingBox_<float>");
		cl.def("cullElegible", (bool (mrpt::opengl::CSkyBox::*)() const) &mrpt::opengl::CSkyBox::cullElegible, "C++: mrpt::opengl::CSkyBox::cullElegible() const --> bool");
		cl.def("assign", (class mrpt::opengl::CSkyBox & (mrpt::opengl::CSkyBox::*)(const class mrpt::opengl::CSkyBox &)) &mrpt::opengl::CSkyBox::operator=, "C++: mrpt::opengl::CSkyBox::operator=(const class mrpt::opengl::CSkyBox &) --> class mrpt::opengl::CSkyBox &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::opengl::CVectorField2D file:mrpt/opengl/CVectorField2D.h line:29
		pybind11::class_<mrpt::opengl::CVectorField2D, std::shared_ptr<mrpt::opengl::CVectorField2D>, PyCallBack_mrpt_opengl_CVectorField2D, mrpt::opengl::CRenderizableShaderPoints, mrpt::opengl::CRenderizableShaderTriangles, mrpt::opengl::CRenderizableShaderWireFrame> cl(M("mrpt::opengl"), "CVectorField2D", "A 2D vector field representation, consisting of points and arrows drawn on a\n plane (invisible grid).\n\n ![mrpt::opengl::CVectorField2D](preview_CVectorField2D.png)\n\n \n opengl::Scene\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::CVectorField2D(); }, [](){ return new PyCallBack_mrpt_opengl_CVectorField2D(); } ) );
		cl.def( pybind11::init( [](class mrpt::math::CMatrixDynamic<float> const & a0, class mrpt::math::CMatrixDynamic<float> const & a1){ return new mrpt::opengl::CVectorField2D(a0, a1); }, [](class mrpt::math::CMatrixDynamic<float> const & a0, class mrpt::math::CMatrixDynamic<float> const & a1){ return new PyCallBack_mrpt_opengl_CVectorField2D(a0, a1); } ), "doc");
		cl.def( pybind11::init( [](class mrpt::math::CMatrixDynamic<float> const & a0, class mrpt::math::CMatrixDynamic<float> const & a1, float const & a2){ return new mrpt::opengl::CVectorField2D(a0, a1, a2); }, [](class mrpt::math::CMatrixDynamic<float> const & a0, class mrpt::math::CMatrixDynamic<float> const & a1, float const & a2){ return new PyCallBack_mrpt_opengl_CVectorField2D(a0, a1, a2); } ), "doc");
		cl.def( pybind11::init( [](class mrpt::math::CMatrixDynamic<float> const & a0, class mrpt::math::CMatrixDynamic<float> const & a1, float const & a2, float const & a3){ return new mrpt::opengl::CVectorField2D(a0, a1, a2, a3); }, [](class mrpt::math::CMatrixDynamic<float> const & a0, class mrpt::math::CMatrixDynamic<float> const & a1, float const & a2, float const & a3){ return new PyCallBack_mrpt_opengl_CVectorField2D(a0, a1, a2, a3); } ), "doc");
		cl.def( pybind11::init( [](class mrpt::math::CMatrixDynamic<float> const & a0, class mrpt::math::CMatrixDynamic<float> const & a1, float const & a2, float const & a3, float const & a4){ return new mrpt::opengl::CVectorField2D(a0, a1, a2, a3, a4); }, [](class mrpt::math::CMatrixDynamic<float> const & a0, class mrpt::math::CMatrixDynamic<float> const & a1, float const & a2, float const & a3, float const & a4){ return new PyCallBack_mrpt_opengl_CVectorField2D(a0, a1, a2, a3, a4); } ), "doc");
		cl.def( pybind11::init<class mrpt::math::CMatrixDynamic<float>, class mrpt::math::CMatrixDynamic<float>, float, float, float, float>(), pybind11::arg("Matrix_x"), pybind11::arg("Matrix_y"), pybind11::arg("xmin"), pybind11::arg("xmax"), pybind11::arg("ymin"), pybind11::arg("ymax") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_opengl_CVectorField2D const &o){ return new PyCallBack_mrpt_opengl_CVectorField2D(o); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::CVectorField2D const &o){ return new mrpt::opengl::CVectorField2D(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CVectorField2D::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CVectorField2D::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CVectorField2D::*)() const) &mrpt::opengl::CVectorField2D::GetRuntimeClass, "C++: mrpt::opengl::CVectorField2D::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::opengl::CVectorField2D::*)() const) &mrpt::opengl::CVectorField2D::clone, "C++: mrpt::opengl::CVectorField2D::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::opengl::CVectorField2D::CreateObject, "C++: mrpt::opengl::CVectorField2D::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("renderUpdateBuffers", (void (mrpt::opengl::CVectorField2D::*)() const) &mrpt::opengl::CVectorField2D::renderUpdateBuffers, "C++: mrpt::opengl::CVectorField2D::renderUpdateBuffers() const --> void");
		cl.def("freeOpenGLResources", (void (mrpt::opengl::CVectorField2D::*)()) &mrpt::opengl::CVectorField2D::freeOpenGLResources, "C++: mrpt::opengl::CVectorField2D::freeOpenGLResources() --> void");
		cl.def("onUpdateBuffers_Wireframe", (void (mrpt::opengl::CVectorField2D::*)()) &mrpt::opengl::CVectorField2D::onUpdateBuffers_Wireframe, "C++: mrpt::opengl::CVectorField2D::onUpdateBuffers_Wireframe() --> void");
		cl.def("onUpdateBuffers_Triangles", (void (mrpt::opengl::CVectorField2D::*)()) &mrpt::opengl::CVectorField2D::onUpdateBuffers_Triangles, "C++: mrpt::opengl::CVectorField2D::onUpdateBuffers_Triangles() --> void");
		cl.def("onUpdateBuffers_Points", (void (mrpt::opengl::CVectorField2D::*)()) &mrpt::opengl::CVectorField2D::onUpdateBuffers_Points, "C++: mrpt::opengl::CVectorField2D::onUpdateBuffers_Points() --> void");
		cl.def("clear", (void (mrpt::opengl::CVectorField2D::*)()) &mrpt::opengl::CVectorField2D::clear, "Clear the matrices\n\nC++: mrpt::opengl::CVectorField2D::clear() --> void");
		cl.def("setPointColor", [](mrpt::opengl::CVectorField2D &o, const float & a0, const float & a1, const float & a2) -> void { return o.setPointColor(a0, a1, a2); }, "", pybind11::arg("R"), pybind11::arg("G"), pybind11::arg("B"));
		cl.def("setPointColor", (void (mrpt::opengl::CVectorField2D::*)(const float, const float, const float, const float)) &mrpt::opengl::CVectorField2D::setPointColor, "Set the point color in the range [0,1]\n\nC++: mrpt::opengl::CVectorField2D::setPointColor(const float, const float, const float, const float) --> void", pybind11::arg("R"), pybind11::arg("G"), pybind11::arg("B"), pybind11::arg("A"));
		cl.def("getPointColor", (struct mrpt::img::TColorf (mrpt::opengl::CVectorField2D::*)() const) &mrpt::opengl::CVectorField2D::getPointColor, "Get the point color in the range [0,1]\n\nC++: mrpt::opengl::CVectorField2D::getPointColor() const --> struct mrpt::img::TColorf");
		cl.def("setVectorFieldColor", [](mrpt::opengl::CVectorField2D &o, const float & a0, const float & a1, const float & a2) -> void { return o.setVectorFieldColor(a0, a1, a2); }, "", pybind11::arg("R"), pybind11::arg("G"), pybind11::arg("B"));
		cl.def("setVectorFieldColor", (void (mrpt::opengl::CVectorField2D::*)(const float, const float, const float, const float)) &mrpt::opengl::CVectorField2D::setVectorFieldColor, "Set the arrow color in the range [0,1]\n\nC++: mrpt::opengl::CVectorField2D::setVectorFieldColor(const float, const float, const float, const float) --> void", pybind11::arg("R"), pybind11::arg("G"), pybind11::arg("B"), pybind11::arg("A"));
		cl.def("getVectorFieldColor", (struct mrpt::img::TColorf (mrpt::opengl::CVectorField2D::*)() const) &mrpt::opengl::CVectorField2D::getVectorFieldColor, "Get the arrow color in the range [0,1]\n\nC++: mrpt::opengl::CVectorField2D::getVectorFieldColor() const --> struct mrpt::img::TColorf");
		cl.def("setGridCenterAndCellSize", (void (mrpt::opengl::CVectorField2D::*)(const float, const float, const float, const float)) &mrpt::opengl::CVectorField2D::setGridCenterAndCellSize, "Set the coordinates of the grid on where the vector field will be drawn\n by setting its center and the cell size.\n The number of cells is marked by the content of xcomp and ycomp.\n \n\n xcomp, ycomp\n\nC++: mrpt::opengl::CVectorField2D::setGridCenterAndCellSize(const float, const float, const float, const float) --> void", pybind11::arg("center_x"), pybind11::arg("center_y"), pybind11::arg("cellsize_x"), pybind11::arg("cellsize_y"));
		cl.def("setGridLimits", (void (mrpt::opengl::CVectorField2D::*)(const float, const float, const float, const float)) &mrpt::opengl::CVectorField2D::setGridLimits, "Set the coordinates of the grid on where the vector field will be drawn\n using x-y max and min values.\n\nC++: mrpt::opengl::CVectorField2D::setGridLimits(const float, const float, const float, const float) --> void", pybind11::arg("xmin"), pybind11::arg("xmax"), pybind11::arg("ymin"), pybind11::arg("ymax"));
		cl.def("getGridLimits", (void (mrpt::opengl::CVectorField2D::*)(float &, float &, float &, float &) const) &mrpt::opengl::CVectorField2D::getGridLimits, "Get the coordinates of the grid on where the vector field is drawn using\n the max and min values.\n\nC++: mrpt::opengl::CVectorField2D::getGridLimits(float &, float &, float &, float &) const --> void", pybind11::arg("xmin"), pybind11::arg("xmax"), pybind11::arg("ymin"), pybind11::arg("ymax"));
		cl.def("getVectorField", (void (mrpt::opengl::CVectorField2D::*)(class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &) const) &mrpt::opengl::CVectorField2D::getVectorField, "Get the vector field. Matrix_x stores the \"x\" component and Matrix_y\n stores the \"y\" component.\n\nC++: mrpt::opengl::CVectorField2D::getVectorField(class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &) const --> void", pybind11::arg("Matrix_x"), pybind11::arg("Matrix_y"));
		cl.def("getVectorField_x", (class mrpt::math::CMatrixDynamic<float> & (mrpt::opengl::CVectorField2D::*)()) &mrpt::opengl::CVectorField2D::getVectorField_x, "C++: mrpt::opengl::CVectorField2D::getVectorField_x() --> class mrpt::math::CMatrixDynamic<float> &", pybind11::return_value_policy::automatic);
		cl.def("getVectorField_y", (class mrpt::math::CMatrixDynamic<float> & (mrpt::opengl::CVectorField2D::*)()) &mrpt::opengl::CVectorField2D::getVectorField_y, "C++: mrpt::opengl::CVectorField2D::getVectorField_y() --> class mrpt::math::CMatrixDynamic<float> &", pybind11::return_value_policy::automatic);
		cl.def("setVectorField", (void (mrpt::opengl::CVectorField2D::*)(class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &)) &mrpt::opengl::CVectorField2D::setVectorField, "Set the vector field. Matrix_x contains the \"x\" component and Matrix_y\n contains the \"y\" component.\n\nC++: mrpt::opengl::CVectorField2D::setVectorField(class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &) --> void", pybind11::arg("Matrix_x"), pybind11::arg("Matrix_y"));
		cl.def("adjustVectorFieldToGrid", (void (mrpt::opengl::CVectorField2D::*)()) &mrpt::opengl::CVectorField2D::adjustVectorFieldToGrid, "Adjust the vector field in the scene (vectors magnitude) according to\n the grid size.\n\nC++: mrpt::opengl::CVectorField2D::adjustVectorFieldToGrid() --> void");
		cl.def("resize", (void (mrpt::opengl::CVectorField2D::*)(size_t, size_t)) &mrpt::opengl::CVectorField2D::resize, "Resizes the set.\n\nC++: mrpt::opengl::CVectorField2D::resize(size_t, size_t) --> void", pybind11::arg("rows"), pybind11::arg("cols"));
		cl.def("cols", (size_t (mrpt::opengl::CVectorField2D::*)() const) &mrpt::opengl::CVectorField2D::cols, "Returns the total count of rows used to represent the vector field. \n\nC++: mrpt::opengl::CVectorField2D::cols() const --> size_t");
		cl.def("rows", (size_t (mrpt::opengl::CVectorField2D::*)() const) &mrpt::opengl::CVectorField2D::rows, "Returns the total count of columns used to represent the vector field.\n\nC++: mrpt::opengl::CVectorField2D::rows() const --> size_t");
		cl.def("internalBoundingBoxLocal", (struct mrpt::math::TBoundingBox_<float> (mrpt::opengl::CVectorField2D::*)() const) &mrpt::opengl::CVectorField2D::internalBoundingBoxLocal, "C++: mrpt::opengl::CVectorField2D::internalBoundingBoxLocal() const --> struct mrpt::math::TBoundingBox_<float>");
		cl.def("enableAntiAliasing", [](mrpt::opengl::CVectorField2D &o) -> void { return o.enableAntiAliasing(); }, "");
		cl.def("enableAntiAliasing", (void (mrpt::opengl::CVectorField2D::*)(bool)) &mrpt::opengl::CVectorField2D::enableAntiAliasing, "C++: mrpt::opengl::CVectorField2D::enableAntiAliasing(bool) --> void", pybind11::arg("enable"));
		cl.def("isAntiAliasingEnabled", (bool (mrpt::opengl::CVectorField2D::*)() const) &mrpt::opengl::CVectorField2D::isAntiAliasingEnabled, "C++: mrpt::opengl::CVectorField2D::isAntiAliasingEnabled() const --> bool");
		cl.def("assign", (class mrpt::opengl::CVectorField2D & (mrpt::opengl::CVectorField2D::*)(const class mrpt::opengl::CVectorField2D &)) &mrpt::opengl::CVectorField2D::operator=, "C++: mrpt::opengl::CVectorField2D::operator=(const class mrpt::opengl::CVectorField2D &) --> class mrpt::opengl::CVectorField2D &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
