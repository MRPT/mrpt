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
#include <mrpt/img/color_maps.h>
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
#include <mrpt/opengl/CMesh.h>
#include <mrpt/opengl/CMesh3D.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CRenderizableShaderTexturedTriangles.h>
#include <mrpt/opengl/CText.h>
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
#include <pybind11/stl.h>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

// mrpt::opengl::CMesh file:mrpt/opengl/CMesh.h line:39
struct PyCallBack_mrpt_opengl_CMesh : public mrpt::opengl::CMesh {
	using mrpt::opengl::CMesh::CMesh;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMesh *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CMesh::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMesh *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CMesh::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMesh *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CMesh::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMesh *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMesh::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMesh *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMesh::serializeFrom(a0, a1);
	}
	void renderUpdateBuffers() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMesh *>(this), "renderUpdateBuffers");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMesh::renderUpdateBuffers();
	}
	void onUpdateBuffers_Wireframe() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMesh *>(this), "onUpdateBuffers_Wireframe");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMesh::onUpdateBuffers_Wireframe();
	}
	void onUpdateBuffers_TexturedTriangles() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMesh *>(this), "onUpdateBuffers_TexturedTriangles");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMesh::onUpdateBuffers_TexturedTriangles();
	}
	void freeOpenGLResources() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMesh *>(this), "freeOpenGLResources");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMesh::freeOpenGLResources();
	}
	struct mrpt::math::TBoundingBox_<float> internalBoundingBoxLocal() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMesh *>(this), "internalBoundingBoxLocal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TBoundingBox_<float>>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TBoundingBox_<float>> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TBoundingBox_<float>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TBoundingBox_<float>>(std::move(o));
		}
		return CMesh::internalBoundingBoxLocal();
	}
	bool traceRay(const class mrpt::poses::CPose3D & a0, double & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMesh *>(this), "traceRay");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CMesh::traceRay(a0, a1);
	}
	void initializeTextures() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMesh *>(this), "initializeTextures");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMesh *>(this), "setColorA_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMesh *>(this), "setColor_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMesh *>(this), "cullElegible");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMesh *>(this), "toYAMLMap");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMesh *>(this), "isCompositeObject");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMesh *>(this), "getLocalRepresentativePoint");
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

// mrpt::opengl::CMesh3D file:mrpt/opengl/CMesh3D.h line:30
struct PyCallBack_mrpt_opengl_CMesh3D : public mrpt::opengl::CMesh3D {
	using mrpt::opengl::CMesh3D::CMesh3D;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMesh3D *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CMesh3D::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMesh3D *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CMesh3D::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMesh3D *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CMesh3D::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMesh3D *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMesh3D::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMesh3D *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMesh3D::serializeFrom(a0, a1);
	}
	void renderUpdateBuffers() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMesh3D *>(this), "renderUpdateBuffers");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMesh3D::renderUpdateBuffers();
	}
	void freeOpenGLResources() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMesh3D *>(this), "freeOpenGLResources");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMesh3D::freeOpenGLResources();
	}
	void onUpdateBuffers_Wireframe() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMesh3D *>(this), "onUpdateBuffers_Wireframe");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMesh3D::onUpdateBuffers_Wireframe();
	}
	void onUpdateBuffers_Triangles() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMesh3D *>(this), "onUpdateBuffers_Triangles");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMesh3D::onUpdateBuffers_Triangles();
	}
	void onUpdateBuffers_Points() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMesh3D *>(this), "onUpdateBuffers_Points");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMesh3D::onUpdateBuffers_Points();
	}
	struct mrpt::math::TBoundingBox_<float> internalBoundingBoxLocal() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMesh3D *>(this), "internalBoundingBoxLocal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TBoundingBox_<float>>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TBoundingBox_<float>> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TBoundingBox_<float>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TBoundingBox_<float>>(std::move(o));
		}
		return CMesh3D::internalBoundingBoxLocal();
	}
	class mrpt::opengl::CRenderizable & setColorA_u8(const unsigned char a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMesh3D *>(this), "setColorA_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMesh3D *>(this), "setColor_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMesh3D *>(this), "cullElegible");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMesh3D *>(this), "toYAMLMap");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMesh3D *>(this), "isCompositeObject");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMesh3D *>(this), "traceRay");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMesh3D *>(this), "getLocalRepresentativePoint");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMesh3D *>(this), "initializeTextures");
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

void bind_mrpt_opengl_CMesh(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::opengl::CMesh file:mrpt/opengl/CMesh.h line:39
		pybind11::class_<mrpt::opengl::CMesh, std::shared_ptr<mrpt::opengl::CMesh>, PyCallBack_mrpt_opengl_CMesh, mrpt::opengl::CRenderizableShaderTexturedTriangles, mrpt::opengl::CRenderizableShaderWireFrame> cl(M("mrpt::opengl"), "CMesh", "A planar (XY) grid where each cell has an associated height and, optionally,\n a texture map. A typical usage example would be an elevation map or a 3D\n model of a terrain.\n\n The height of each cell/pixel is provided via an elevation `Z` matrix,\n where the z coordinate of the grid cell (x,y) is given by `Z(x,y)`\n (not `Z(y,x)`!!), that is:\n - Z column count = number of cells in direction \"+y\"\n - Z row count = number of cells in direction \"+x\"\n\n Since MRPT 2.7.0, the texture can be wrapped over the mesh using\n setMeshTextureExtension().\n\n ![mrpt::opengl::CMesh](preview_CMesh.png)\n\n  \n opengl::Scene\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::CMesh(); }, [](){ return new PyCallBack_mrpt_opengl_CMesh(); } ), "doc");
		cl.def( pybind11::init( [](bool const & a0){ return new mrpt::opengl::CMesh(a0); }, [](bool const & a0){ return new PyCallBack_mrpt_opengl_CMesh(a0); } ), "doc");
		cl.def( pybind11::init( [](bool const & a0, float const & a1){ return new mrpt::opengl::CMesh(a0, a1); }, [](bool const & a0, float const & a1){ return new PyCallBack_mrpt_opengl_CMesh(a0, a1); } ), "doc");
		cl.def( pybind11::init( [](bool const & a0, float const & a1, float const & a2){ return new mrpt::opengl::CMesh(a0, a1, a2); }, [](bool const & a0, float const & a1, float const & a2){ return new PyCallBack_mrpt_opengl_CMesh(a0, a1, a2); } ), "doc");
		cl.def( pybind11::init( [](bool const & a0, float const & a1, float const & a2, float const & a3){ return new mrpt::opengl::CMesh(a0, a1, a2, a3); }, [](bool const & a0, float const & a1, float const & a2, float const & a3){ return new PyCallBack_mrpt_opengl_CMesh(a0, a1, a2, a3); } ), "doc");
		cl.def( pybind11::init<bool, float, float, float, float>(), pybind11::arg("enableTransparency"), pybind11::arg("xMin"), pybind11::arg("xMax"), pybind11::arg("yMin"), pybind11::arg("yMax") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_opengl_CMesh const &o){ return new PyCallBack_mrpt_opengl_CMesh(o); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::CMesh const &o){ return new mrpt::opengl::CMesh(o); } ) );
		cl.def_static("getClassName", (class mrpt::typemeta::string_literal<19> (*)()) &mrpt::opengl::CMesh::getClassName, "C++: mrpt::opengl::CMesh::getClassName() --> class mrpt::typemeta::string_literal<19>");
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CMesh::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CMesh::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CMesh::*)() const) &mrpt::opengl::CMesh::GetRuntimeClass, "C++: mrpt::opengl::CMesh::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::opengl::CMesh::*)() const) &mrpt::opengl::CMesh::clone, "C++: mrpt::opengl::CMesh::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::opengl::CMesh::CreateObject, "C++: mrpt::opengl::CMesh::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("renderUpdateBuffers", (void (mrpt::opengl::CMesh::*)() const) &mrpt::opengl::CMesh::renderUpdateBuffers, "C++: mrpt::opengl::CMesh::renderUpdateBuffers() const --> void");
		cl.def("onUpdateBuffers_Wireframe", (void (mrpt::opengl::CMesh::*)()) &mrpt::opengl::CMesh::onUpdateBuffers_Wireframe, "C++: mrpt::opengl::CMesh::onUpdateBuffers_Wireframe() --> void");
		cl.def("onUpdateBuffers_TexturedTriangles", (void (mrpt::opengl::CMesh::*)()) &mrpt::opengl::CMesh::onUpdateBuffers_TexturedTriangles, "C++: mrpt::opengl::CMesh::onUpdateBuffers_TexturedTriangles() --> void");
		cl.def("freeOpenGLResources", (void (mrpt::opengl::CMesh::*)()) &mrpt::opengl::CMesh::freeOpenGLResources, "C++: mrpt::opengl::CMesh::freeOpenGLResources() --> void");
		cl.def("getGridLimits", (void (mrpt::opengl::CMesh::*)(float &, float &, float &, float &) const) &mrpt::opengl::CMesh::getGridLimits, "C++: mrpt::opengl::CMesh::getGridLimits(float &, float &, float &, float &) const --> void", pybind11::arg("xMin"), pybind11::arg("xMax"), pybind11::arg("yMin"), pybind11::arg("yMax"));
		cl.def("setMeshTextureExtension", (void (mrpt::opengl::CMesh::*)(float, float)) &mrpt::opengl::CMesh::setMeshTextureExtension, "Sets the texture physical size (in \"meters) using to wrap it over the\n mesh extension.\n  The default (0) means texture size is equal to whole grid extension. \n\nC++: mrpt::opengl::CMesh::setMeshTextureExtension(float, float) --> void", pybind11::arg("textureSize_x"), pybind11::arg("textureSize_y"));
		cl.def("getMeshTextureExtension", (void (mrpt::opengl::CMesh::*)(float &, float &) const) &mrpt::opengl::CMesh::getMeshTextureExtension, "C++: mrpt::opengl::CMesh::getMeshTextureExtension(float &, float &) const --> void", pybind11::arg("textureSize_x"), pybind11::arg("textureSize_y"));
		cl.def("enableTransparency", (void (mrpt::opengl::CMesh::*)(bool)) &mrpt::opengl::CMesh::enableTransparency, "C++: mrpt::opengl::CMesh::enableTransparency(bool) --> void", pybind11::arg("v"));
		cl.def("enableWireFrame", (void (mrpt::opengl::CMesh::*)(bool)) &mrpt::opengl::CMesh::enableWireFrame, "C++: mrpt::opengl::CMesh::enableWireFrame(bool) --> void", pybind11::arg("v"));
		cl.def("enableColorFromZ", [](mrpt::opengl::CMesh &o, bool const & a0) -> void { return o.enableColorFromZ(a0); }, "", pybind11::arg("v"));
		cl.def("enableColorFromZ", (void (mrpt::opengl::CMesh::*)(bool, enum mrpt::img::TColormap)) &mrpt::opengl::CMesh::enableColorFromZ, "C++: mrpt::opengl::CMesh::enableColorFromZ(bool, enum mrpt::img::TColormap) --> void", pybind11::arg("v"), pybind11::arg("colorMap"));
		cl.def("setZ", (void (mrpt::opengl::CMesh::*)(const class mrpt::math::CMatrixDynamic<float> &)) &mrpt::opengl::CMesh::setZ, "This method sets the matrix of heights for each position (cell) in the\n mesh grid \n\nC++: mrpt::opengl::CMesh::setZ(const class mrpt::math::CMatrixDynamic<float> &) --> void", pybind11::arg("in_Z"));
		cl.def("getZ", (void (mrpt::opengl::CMesh::*)(class mrpt::math::CMatrixDynamic<float> &) const) &mrpt::opengl::CMesh::getZ, "Returns a reference to the internal Z matrix, allowing changing it\n efficiently \n\nC++: mrpt::opengl::CMesh::getZ(class mrpt::math::CMatrixDynamic<float> &) const --> void", pybind11::arg("out"));
		cl.def("getMask", (void (mrpt::opengl::CMesh::*)(class mrpt::math::CMatrixDynamic<float> &) const) &mrpt::opengl::CMesh::getMask, "Returns a reference to the internal mask matrix, allowing changing it\n efficiently \n\nC++: mrpt::opengl::CMesh::getMask(class mrpt::math::CMatrixDynamic<float> &) const --> void", pybind11::arg("out"));
		cl.def("setMask", (void (mrpt::opengl::CMesh::*)(const class mrpt::math::CMatrixDynamic<float> &)) &mrpt::opengl::CMesh::setMask, "This method sets the boolean mask of valid heights for each position\n (cell) in the mesh grid \n\nC++: mrpt::opengl::CMesh::setMask(const class mrpt::math::CMatrixDynamic<float> &) --> void", pybind11::arg("in_mask"));
		cl.def("getxMin", (float (mrpt::opengl::CMesh::*)() const) &mrpt::opengl::CMesh::getxMin, "C++: mrpt::opengl::CMesh::getxMin() const --> float");
		cl.def("getxMax", (float (mrpt::opengl::CMesh::*)() const) &mrpt::opengl::CMesh::getxMax, "C++: mrpt::opengl::CMesh::getxMax() const --> float");
		cl.def("getyMin", (float (mrpt::opengl::CMesh::*)() const) &mrpt::opengl::CMesh::getyMin, "C++: mrpt::opengl::CMesh::getyMin() const --> float");
		cl.def("getyMax", (float (mrpt::opengl::CMesh::*)() const) &mrpt::opengl::CMesh::getyMax, "C++: mrpt::opengl::CMesh::getyMax() const --> float");
		cl.def("setxMin", (void (mrpt::opengl::CMesh::*)(const float)) &mrpt::opengl::CMesh::setxMin, "C++: mrpt::opengl::CMesh::setxMin(const float) --> void", pybind11::arg("nxm"));
		cl.def("setxMax", (void (mrpt::opengl::CMesh::*)(const float)) &mrpt::opengl::CMesh::setxMax, "C++: mrpt::opengl::CMesh::setxMax(const float) --> void", pybind11::arg("nxm"));
		cl.def("setyMin", (void (mrpt::opengl::CMesh::*)(const float)) &mrpt::opengl::CMesh::setyMin, "C++: mrpt::opengl::CMesh::setyMin(const float) --> void", pybind11::arg("nym"));
		cl.def("setyMax", (void (mrpt::opengl::CMesh::*)(const float)) &mrpt::opengl::CMesh::setyMax, "C++: mrpt::opengl::CMesh::setyMax(const float) --> void", pybind11::arg("nym"));
		cl.def("getXBounds", (void (mrpt::opengl::CMesh::*)(float &, float &) const) &mrpt::opengl::CMesh::getXBounds, "C++: mrpt::opengl::CMesh::getXBounds(float &, float &) const --> void", pybind11::arg("min"), pybind11::arg("max"));
		cl.def("getYBounds", (void (mrpt::opengl::CMesh::*)(float &, float &) const) &mrpt::opengl::CMesh::getYBounds, "C++: mrpt::opengl::CMesh::getYBounds(float &, float &) const --> void", pybind11::arg("min"), pybind11::arg("max"));
		cl.def("setXBounds", (void (mrpt::opengl::CMesh::*)(const float, const float)) &mrpt::opengl::CMesh::setXBounds, "C++: mrpt::opengl::CMesh::setXBounds(const float, const float) --> void", pybind11::arg("min"), pybind11::arg("max"));
		cl.def("setYBounds", (void (mrpt::opengl::CMesh::*)(const float, const float)) &mrpt::opengl::CMesh::setYBounds, "C++: mrpt::opengl::CMesh::setYBounds(const float, const float) --> void", pybind11::arg("min"), pybind11::arg("max"));
		cl.def("internalBoundingBoxLocal", (struct mrpt::math::TBoundingBox_<float> (mrpt::opengl::CMesh::*)() const) &mrpt::opengl::CMesh::internalBoundingBoxLocal, "C++: mrpt::opengl::CMesh::internalBoundingBoxLocal() const --> struct mrpt::math::TBoundingBox_<float>");
		cl.def("assignImage", (void (mrpt::opengl::CMesh::*)(const class mrpt::img::CImage &)) &mrpt::opengl::CMesh::assignImage, "Assigns a texture image.\n\nC++: mrpt::opengl::CMesh::assignImage(const class mrpt::img::CImage &) --> void", pybind11::arg("img"));
		cl.def("assignImageAndZ", (void (mrpt::opengl::CMesh::*)(const class mrpt::img::CImage &, const class mrpt::math::CMatrixDynamic<float> &)) &mrpt::opengl::CMesh::assignImageAndZ, "Assigns a texture image and Z simultaneously, and disable transparency.\n\nC++: mrpt::opengl::CMesh::assignImageAndZ(const class mrpt::img::CImage &, const class mrpt::math::CMatrixDynamic<float> &) --> void", pybind11::arg("img"), pybind11::arg("in_Z"));
		cl.def("adjustGridToImageAR", (void (mrpt::opengl::CMesh::*)()) &mrpt::opengl::CMesh::adjustGridToImageAR, "Adjust grid limits according to the image aspect ratio, maintaining the\n X limits and resizing in the Y direction.\n\nC++: mrpt::opengl::CMesh::adjustGridToImageAR() --> void");
		cl.def("traceRay", (bool (mrpt::opengl::CMesh::*)(const class mrpt::poses::CPose3D &, double &) const) &mrpt::opengl::CMesh::traceRay, "Trace ray\n\nC++: mrpt::opengl::CMesh::traceRay(const class mrpt::poses::CPose3D &, double &) const --> bool", pybind11::arg("o"), pybind11::arg("dist"));
		cl.def("assign", (class mrpt::opengl::CMesh & (mrpt::opengl::CMesh::*)(const class mrpt::opengl::CMesh &)) &mrpt::opengl::CMesh::operator=, "C++: mrpt::opengl::CMesh::operator=(const class mrpt::opengl::CMesh &) --> class mrpt::opengl::CMesh &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::opengl::CMesh::TTriangleVertexIndices file:mrpt/opengl/CMesh.h line:44
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::opengl::CMesh::TTriangleVertexIndices, std::shared_ptr<mrpt::opengl::CMesh::TTriangleVertexIndices>> cl(enclosing_class, "TTriangleVertexIndices", "");
			cl.def( pybind11::init( [](){ return new mrpt::opengl::CMesh::TTriangleVertexIndices(); } ) );
		}

	}
	{ // mrpt::opengl::CMesh3D file:mrpt/opengl/CMesh3D.h line:30
		pybind11::class_<mrpt::opengl::CMesh3D, std::shared_ptr<mrpt::opengl::CMesh3D>, PyCallBack_mrpt_opengl_CMesh3D, mrpt::opengl::CRenderizableShaderTriangles, mrpt::opengl::CRenderizableShaderWireFrame, mrpt::opengl::CRenderizableShaderPoints> cl(M("mrpt::opengl"), "CMesh3D", "A 3D mesh composed of triangles and/or quads.\n A typical usage example would be a 3D model of an object.\n\n ![mrpt::opengl::CMesh3D](preview_CMesh3D.png)\n\n \n opengl::Scene,opengl::CMesh,opengl::CAssimpModel\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::CMesh3D(); }, [](){ return new PyCallBack_mrpt_opengl_CMesh3D(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_opengl_CMesh3D const &o){ return new PyCallBack_mrpt_opengl_CMesh3D(o); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::CMesh3D const &o){ return new mrpt::opengl::CMesh3D(o); } ) );
		cl.def_static("getClassName", (class mrpt::typemeta::string_literal<21> (*)()) &mrpt::opengl::CMesh3D::getClassName, "C++: mrpt::opengl::CMesh3D::getClassName() --> class mrpt::typemeta::string_literal<21>");
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CMesh3D::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CMesh3D::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CMesh3D::*)() const) &mrpt::opengl::CMesh3D::GetRuntimeClass, "C++: mrpt::opengl::CMesh3D::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::opengl::CMesh3D::*)() const) &mrpt::opengl::CMesh3D::clone, "C++: mrpt::opengl::CMesh3D::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::opengl::CMesh3D::CreateObject, "C++: mrpt::opengl::CMesh3D::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("renderUpdateBuffers", (void (mrpt::opengl::CMesh3D::*)() const) &mrpt::opengl::CMesh3D::renderUpdateBuffers, "C++: mrpt::opengl::CMesh3D::renderUpdateBuffers() const --> void");
		cl.def("freeOpenGLResources", (void (mrpt::opengl::CMesh3D::*)()) &mrpt::opengl::CMesh3D::freeOpenGLResources, "C++: mrpt::opengl::CMesh3D::freeOpenGLResources() --> void");
		cl.def("onUpdateBuffers_Wireframe", (void (mrpt::opengl::CMesh3D::*)()) &mrpt::opengl::CMesh3D::onUpdateBuffers_Wireframe, "C++: mrpt::opengl::CMesh3D::onUpdateBuffers_Wireframe() --> void");
		cl.def("onUpdateBuffers_Triangles", (void (mrpt::opengl::CMesh3D::*)()) &mrpt::opengl::CMesh3D::onUpdateBuffers_Triangles, "C++: mrpt::opengl::CMesh3D::onUpdateBuffers_Triangles() --> void");
		cl.def("onUpdateBuffers_Points", (void (mrpt::opengl::CMesh3D::*)()) &mrpt::opengl::CMesh3D::onUpdateBuffers_Points, "C++: mrpt::opengl::CMesh3D::onUpdateBuffers_Points() --> void");
		cl.def("enableShowEdges", (void (mrpt::opengl::CMesh3D::*)(bool)) &mrpt::opengl::CMesh3D::enableShowEdges, "C++: mrpt::opengl::CMesh3D::enableShowEdges(bool) --> void", pybind11::arg("v"));
		cl.def("enableShowFaces", (void (mrpt::opengl::CMesh3D::*)(bool)) &mrpt::opengl::CMesh3D::enableShowFaces, "C++: mrpt::opengl::CMesh3D::enableShowFaces(bool) --> void", pybind11::arg("v"));
		cl.def("enableShowVertices", (void (mrpt::opengl::CMesh3D::*)(bool)) &mrpt::opengl::CMesh3D::enableShowVertices, "C++: mrpt::opengl::CMesh3D::enableShowVertices(bool) --> void", pybind11::arg("v"));
		cl.def("enableFaceNormals", (void (mrpt::opengl::CMesh3D::*)(bool)) &mrpt::opengl::CMesh3D::enableFaceNormals, "C++: mrpt::opengl::CMesh3D::enableFaceNormals(bool) --> void", pybind11::arg("v"));
		cl.def("loadMesh", (void (mrpt::opengl::CMesh3D::*)(unsigned int, unsigned int, int *, int *, float *)) &mrpt::opengl::CMesh3D::loadMesh, "Load a 3D mesh. The arguments indicate:\n		- num_verts: Number of vertices of the mesh\n		- num_faces: Number of faces of the mesh\n		- verts_per_face: An array (pointer) with the number of vertices of each\n	   face. The elements must be set either to 3 (triangle) or 4 (quad).\n		- face_verts: An array (pointer) with the vertices of each face. The\n	   vertices of each face must be consecutive in this array.\n		- vert_coords: An array (pointer) with the coordinates of each vertex.\n	   The xyz coordinates of each vertex must be consecutive in this array.\n\nC++: mrpt::opengl::CMesh3D::loadMesh(unsigned int, unsigned int, int *, int *, float *) --> void", pybind11::arg("num_verts"), pybind11::arg("num_faces"), pybind11::arg("verts_per_face"), pybind11::arg("face_verts"), pybind11::arg("vert_coords"));
		cl.def("setEdgeColor", [](mrpt::opengl::CMesh3D &o, float const & a0, float const & a1, float const & a2) -> void { return o.setEdgeColor(a0, a1, a2); }, "", pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"));
		cl.def("setEdgeColor", (void (mrpt::opengl::CMesh3D::*)(float, float, float, float)) &mrpt::opengl::CMesh3D::setEdgeColor, "C++: mrpt::opengl::CMesh3D::setEdgeColor(float, float, float, float) --> void", pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"), pybind11::arg("a"));
		cl.def("setFaceColor", [](mrpt::opengl::CMesh3D &o, float const & a0, float const & a1, float const & a2) -> void { return o.setFaceColor(a0, a1, a2); }, "", pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"));
		cl.def("setFaceColor", (void (mrpt::opengl::CMesh3D::*)(float, float, float, float)) &mrpt::opengl::CMesh3D::setFaceColor, "C++: mrpt::opengl::CMesh3D::setFaceColor(float, float, float, float) --> void", pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"), pybind11::arg("a"));
		cl.def("setVertColor", [](mrpt::opengl::CMesh3D &o, float const & a0, float const & a1, float const & a2) -> void { return o.setVertColor(a0, a1, a2); }, "", pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"));
		cl.def("setVertColor", (void (mrpt::opengl::CMesh3D::*)(float, float, float, float)) &mrpt::opengl::CMesh3D::setVertColor, "C++: mrpt::opengl::CMesh3D::setVertColor(float, float, float, float) --> void", pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"), pybind11::arg("a"));
		cl.def("internalBoundingBoxLocal", (struct mrpt::math::TBoundingBox_<float> (mrpt::opengl::CMesh3D::*)() const) &mrpt::opengl::CMesh3D::internalBoundingBoxLocal, "C++: mrpt::opengl::CMesh3D::internalBoundingBoxLocal() const --> struct mrpt::math::TBoundingBox_<float>");
		cl.def("assign", (class mrpt::opengl::CMesh3D & (mrpt::opengl::CMesh3D::*)(const class mrpt::opengl::CMesh3D &)) &mrpt::opengl::CMesh3D::operator=, "C++: mrpt::opengl::CMesh3D::operator=(const class mrpt::opengl::CMesh3D &) --> class mrpt::opengl::CMesh3D &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
