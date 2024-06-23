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
#include <mrpt/opengl/CMesh3D.h>
#include <mrpt/opengl/CMeshFast.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CRenderizableShaderPoints.h>
#include <mrpt/opengl/CRenderizableShaderWireFrame.h>
#include <mrpt/opengl/CSimpleLine.h>
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

// mrpt::opengl::CMeshFast file:mrpt/opengl/CMeshFast.h line:31
struct PyCallBack_mrpt_opengl_CMeshFast : public mrpt::opengl::CMeshFast {
	using mrpt::opengl::CMeshFast::CMeshFast;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMeshFast *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CMeshFast::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMeshFast *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CMeshFast::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMeshFast *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CMeshFast::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMeshFast *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMeshFast::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMeshFast *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMeshFast::serializeFrom(a0, a1);
	}
	void onUpdateBuffers_Points() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMeshFast *>(this), "onUpdateBuffers_Points");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMeshFast::onUpdateBuffers_Points();
	}
	struct mrpt::math::TBoundingBox_<float> internalBoundingBoxLocal() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMeshFast *>(this), "internalBoundingBoxLocal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TBoundingBox_<float>>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TBoundingBox_<float>> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TBoundingBox_<float>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TBoundingBox_<float>>(std::move(o));
		}
		return CMeshFast::internalBoundingBoxLocal();
	}
	void renderUpdateBuffers() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMeshFast *>(this), "renderUpdateBuffers");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRenderizableShaderPoints::renderUpdateBuffers();
	}
	void freeOpenGLResources() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMeshFast *>(this), "freeOpenGLResources");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRenderizableShaderPoints::freeOpenGLResources();
	}
	class mrpt::opengl::CRenderizable & setColorA_u8(const unsigned char a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMeshFast *>(this), "setColorA_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMeshFast *>(this), "setColor_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMeshFast *>(this), "cullElegible");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMeshFast *>(this), "toYAMLMap");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMeshFast *>(this), "isCompositeObject");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMeshFast *>(this), "traceRay");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMeshFast *>(this), "getLocalRepresentativePoint");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CMeshFast *>(this), "initializeTextures");
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

// mrpt::opengl::CSimpleLine file:mrpt/opengl/CSimpleLine.h line:20
struct PyCallBack_mrpt_opengl_CSimpleLine : public mrpt::opengl::CSimpleLine {
	using mrpt::opengl::CSimpleLine::CSimpleLine;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSimpleLine *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CSimpleLine::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSimpleLine *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CSimpleLine::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSimpleLine *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CSimpleLine::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSimpleLine *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSimpleLine::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSimpleLine *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSimpleLine::serializeFrom(a0, a1);
	}
	void onUpdateBuffers_Wireframe() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSimpleLine *>(this), "onUpdateBuffers_Wireframe");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSimpleLine::onUpdateBuffers_Wireframe();
	}
	struct mrpt::math::TBoundingBox_<float> internalBoundingBoxLocal() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSimpleLine *>(this), "internalBoundingBoxLocal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TBoundingBox_<float>>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TBoundingBox_<float>> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TBoundingBox_<float>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TBoundingBox_<float>>(std::move(o));
		}
		return CSimpleLine::internalBoundingBoxLocal();
	}
	void renderUpdateBuffers() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSimpleLine *>(this), "renderUpdateBuffers");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRenderizableShaderWireFrame::renderUpdateBuffers();
	}
	void freeOpenGLResources() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSimpleLine *>(this), "freeOpenGLResources");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRenderizableShaderWireFrame::freeOpenGLResources();
	}
	class mrpt::opengl::CRenderizable & setColorA_u8(const unsigned char a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSimpleLine *>(this), "setColorA_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSimpleLine *>(this), "setColor_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSimpleLine *>(this), "cullElegible");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSimpleLine *>(this), "toYAMLMap");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSimpleLine *>(this), "isCompositeObject");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSimpleLine *>(this), "traceRay");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSimpleLine *>(this), "getLocalRepresentativePoint");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSimpleLine *>(this), "initializeTextures");
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

void bind_mrpt_opengl_CMesh3D(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::opengl::CMesh3D file:mrpt/opengl/CMesh3D.h line:30
		pybind11::class_<mrpt::opengl::CMesh3D, std::shared_ptr<mrpt::opengl::CMesh3D>, PyCallBack_mrpt_opengl_CMesh3D, mrpt::opengl::CRenderizableShaderTriangles, mrpt::opengl::CRenderizableShaderWireFrame, mrpt::opengl::CRenderizableShaderPoints> cl(M("mrpt::opengl"), "CMesh3D", "A 3D mesh composed of triangles and/or quads.\n A typical usage example would be a 3D model of an object.\n\n ![mrpt::opengl::CMesh3D](preview_CMesh3D.png)\n\n \n opengl::Scene,opengl::CMesh,opengl::CAssimpModel\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::CMesh3D(); }, [](){ return new PyCallBack_mrpt_opengl_CMesh3D(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_opengl_CMesh3D const &o){ return new PyCallBack_mrpt_opengl_CMesh3D(o); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::CMesh3D const &o){ return new mrpt::opengl::CMesh3D(o); } ) );
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
		cl.def("loadMesh", (void (mrpt::opengl::CMesh3D::*)(unsigned int, unsigned int, int *, int *, float *)) &mrpt::opengl::CMesh3D::loadMesh, "Load a 3D mesh. The arguments indicate:\n    - num_verts: Number of vertices of the mesh\n    - num_faces: Number of faces of the mesh\n    - verts_per_face: An array (pointer) with the number of vertices of each\n   face. The elements must be set either to 3 (triangle) or 4 (quad).\n    - face_verts: An array (pointer) with the vertices of each face. The\n   vertices of each face must be consecutive in this array.\n    - vert_coords: An array (pointer) with the coordinates of each vertex.\n   The xyz coordinates of each vertex must be consecutive in this array.\n\nC++: mrpt::opengl::CMesh3D::loadMesh(unsigned int, unsigned int, int *, int *, float *) --> void", pybind11::arg("num_verts"), pybind11::arg("num_faces"), pybind11::arg("verts_per_face"), pybind11::arg("face_verts"), pybind11::arg("vert_coords"));
		cl.def("setEdgeColor", [](mrpt::opengl::CMesh3D &o, float const & a0, float const & a1, float const & a2) -> void { return o.setEdgeColor(a0, a1, a2); }, "", pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"));
		cl.def("setEdgeColor", (void (mrpt::opengl::CMesh3D::*)(float, float, float, float)) &mrpt::opengl::CMesh3D::setEdgeColor, "C++: mrpt::opengl::CMesh3D::setEdgeColor(float, float, float, float) --> void", pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"), pybind11::arg("a"));
		cl.def("setFaceColor", [](mrpt::opengl::CMesh3D &o, float const & a0, float const & a1, float const & a2) -> void { return o.setFaceColor(a0, a1, a2); }, "", pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"));
		cl.def("setFaceColor", (void (mrpt::opengl::CMesh3D::*)(float, float, float, float)) &mrpt::opengl::CMesh3D::setFaceColor, "C++: mrpt::opengl::CMesh3D::setFaceColor(float, float, float, float) --> void", pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"), pybind11::arg("a"));
		cl.def("setVertColor", [](mrpt::opengl::CMesh3D &o, float const & a0, float const & a1, float const & a2) -> void { return o.setVertColor(a0, a1, a2); }, "", pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"));
		cl.def("setVertColor", (void (mrpt::opengl::CMesh3D::*)(float, float, float, float)) &mrpt::opengl::CMesh3D::setVertColor, "C++: mrpt::opengl::CMesh3D::setVertColor(float, float, float, float) --> void", pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"), pybind11::arg("a"));
		cl.def("internalBoundingBoxLocal", (struct mrpt::math::TBoundingBox_<float> (mrpt::opengl::CMesh3D::*)() const) &mrpt::opengl::CMesh3D::internalBoundingBoxLocal, "C++: mrpt::opengl::CMesh3D::internalBoundingBoxLocal() const --> struct mrpt::math::TBoundingBox_<float>");
		cl.def("assign", (class mrpt::opengl::CMesh3D & (mrpt::opengl::CMesh3D::*)(const class mrpt::opengl::CMesh3D &)) &mrpt::opengl::CMesh3D::operator=, "C++: mrpt::opengl::CMesh3D::operator=(const class mrpt::opengl::CMesh3D &) --> class mrpt::opengl::CMesh3D &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::opengl::CMeshFast file:mrpt/opengl/CMeshFast.h line:31
		pybind11::class_<mrpt::opengl::CMeshFast, std::shared_ptr<mrpt::opengl::CMeshFast>, PyCallBack_mrpt_opengl_CMeshFast, mrpt::opengl::CRenderizableShaderPoints> cl(M("mrpt::opengl"), "CMeshFast", "A planar (XY) grid where each cell has an associated height and, optionally,\n a texture map.\n To make it faster to render, instead of drawing lines and triangles it draws\n a point at each gridcell.\n  A typical usage example would be an elevation map or a 3D model of a\n terrain.\n\n ![mrpt::opengl::CMeshFast](preview_CMeshFast.png)\n\n  \n opengl::Scene\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::CMeshFast(); }, [](){ return new PyCallBack_mrpt_opengl_CMeshFast(); } ), "doc");
		cl.def( pybind11::init( [](bool const & a0){ return new mrpt::opengl::CMeshFast(a0); }, [](bool const & a0){ return new PyCallBack_mrpt_opengl_CMeshFast(a0); } ), "doc");
		cl.def( pybind11::init( [](bool const & a0, float const & a1){ return new mrpt::opengl::CMeshFast(a0, a1); }, [](bool const & a0, float const & a1){ return new PyCallBack_mrpt_opengl_CMeshFast(a0, a1); } ), "doc");
		cl.def( pybind11::init( [](bool const & a0, float const & a1, float const & a2){ return new mrpt::opengl::CMeshFast(a0, a1, a2); }, [](bool const & a0, float const & a1, float const & a2){ return new PyCallBack_mrpt_opengl_CMeshFast(a0, a1, a2); } ), "doc");
		cl.def( pybind11::init( [](bool const & a0, float const & a1, float const & a2, float const & a3){ return new mrpt::opengl::CMeshFast(a0, a1, a2, a3); }, [](bool const & a0, float const & a1, float const & a2, float const & a3){ return new PyCallBack_mrpt_opengl_CMeshFast(a0, a1, a2, a3); } ), "doc");
		cl.def( pybind11::init<bool, float, float, float, float>(), pybind11::arg("enableTransparency"), pybind11::arg("xMin_p"), pybind11::arg("xMax_p"), pybind11::arg("yMin_p"), pybind11::arg("yMax_p") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_opengl_CMeshFast const &o){ return new PyCallBack_mrpt_opengl_CMeshFast(o); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::CMeshFast const &o){ return new mrpt::opengl::CMeshFast(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CMeshFast::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CMeshFast::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CMeshFast::*)() const) &mrpt::opengl::CMeshFast::GetRuntimeClass, "C++: mrpt::opengl::CMeshFast::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::opengl::CMeshFast::*)() const) &mrpt::opengl::CMeshFast::clone, "C++: mrpt::opengl::CMeshFast::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::opengl::CMeshFast::CreateObject, "C++: mrpt::opengl::CMeshFast::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("onUpdateBuffers_Points", (void (mrpt::opengl::CMeshFast::*)()) &mrpt::opengl::CMeshFast::onUpdateBuffers_Points, "@{ \n\nC++: mrpt::opengl::CMeshFast::onUpdateBuffers_Points() --> void");
		cl.def("setGridLimits", (void (mrpt::opengl::CMeshFast::*)(float, float, float, float)) &mrpt::opengl::CMeshFast::setGridLimits, "C++: mrpt::opengl::CMeshFast::setGridLimits(float, float, float, float) --> void", pybind11::arg("xmin"), pybind11::arg("xmax"), pybind11::arg("ymin"), pybind11::arg("ymax"));
		cl.def("getGridLimits", (void (mrpt::opengl::CMeshFast::*)(float &, float &, float &, float &) const) &mrpt::opengl::CMeshFast::getGridLimits, "C++: mrpt::opengl::CMeshFast::getGridLimits(float &, float &, float &, float &) const --> void", pybind11::arg("xmin"), pybind11::arg("xmax"), pybind11::arg("ymin"), pybind11::arg("ymax"));
		cl.def("enableTransparency", (void (mrpt::opengl::CMeshFast::*)(bool)) &mrpt::opengl::CMeshFast::enableTransparency, "C++: mrpt::opengl::CMeshFast::enableTransparency(bool) --> void", pybind11::arg("v"));
		cl.def("enableColorFromZ", [](mrpt::opengl::CMeshFast &o, bool const & a0) -> void { return o.enableColorFromZ(a0); }, "", pybind11::arg("v"));
		cl.def("enableColorFromZ", (void (mrpt::opengl::CMeshFast::*)(bool, enum mrpt::img::TColormap)) &mrpt::opengl::CMeshFast::enableColorFromZ, "C++: mrpt::opengl::CMeshFast::enableColorFromZ(bool, enum mrpt::img::TColormap) --> void", pybind11::arg("v"), pybind11::arg("colorMap"));
		cl.def("setZ", (void (mrpt::opengl::CMeshFast::*)(const class mrpt::math::CMatrixDynamic<float> &)) &mrpt::opengl::CMeshFast::setZ, "This method sets the matrix of heights for each position (cell) in the\n mesh grid \n\nC++: mrpt::opengl::CMeshFast::setZ(const class mrpt::math::CMatrixDynamic<float> &) --> void", pybind11::arg("in_Z"));
		cl.def("getZ", (void (mrpt::opengl::CMeshFast::*)(class mrpt::math::CMatrixDynamic<float> &) const) &mrpt::opengl::CMeshFast::getZ, "Returns a reference to the internal Z matrix, allowing changing it\n efficiently \n\nC++: mrpt::opengl::CMeshFast::getZ(class mrpt::math::CMatrixDynamic<float> &) const --> void", pybind11::arg("out"));
		cl.def("getXMin", (float (mrpt::opengl::CMeshFast::*)() const) &mrpt::opengl::CMeshFast::getXMin, "C++: mrpt::opengl::CMeshFast::getXMin() const --> float");
		cl.def("getXMax", (float (mrpt::opengl::CMeshFast::*)() const) &mrpt::opengl::CMeshFast::getXMax, "C++: mrpt::opengl::CMeshFast::getXMax() const --> float");
		cl.def("getYMin", (float (mrpt::opengl::CMeshFast::*)() const) &mrpt::opengl::CMeshFast::getYMin, "C++: mrpt::opengl::CMeshFast::getYMin() const --> float");
		cl.def("getYMax", (float (mrpt::opengl::CMeshFast::*)() const) &mrpt::opengl::CMeshFast::getYMax, "C++: mrpt::opengl::CMeshFast::getYMax() const --> float");
		cl.def("setXMin", (void (mrpt::opengl::CMeshFast::*)(float)) &mrpt::opengl::CMeshFast::setXMin, "C++: mrpt::opengl::CMeshFast::setXMin(float) --> void", pybind11::arg("nxm"));
		cl.def("setXMax", (void (mrpt::opengl::CMeshFast::*)(float)) &mrpt::opengl::CMeshFast::setXMax, "C++: mrpt::opengl::CMeshFast::setXMax(float) --> void", pybind11::arg("nxm"));
		cl.def("setYMin", (void (mrpt::opengl::CMeshFast::*)(float)) &mrpt::opengl::CMeshFast::setYMin, "C++: mrpt::opengl::CMeshFast::setYMin(float) --> void", pybind11::arg("nym"));
		cl.def("setYMax", (void (mrpt::opengl::CMeshFast::*)(float)) &mrpt::opengl::CMeshFast::setYMax, "C++: mrpt::opengl::CMeshFast::setYMax(float) --> void", pybind11::arg("nym"));
		cl.def("getXBounds", (void (mrpt::opengl::CMeshFast::*)(float &, float &) const) &mrpt::opengl::CMeshFast::getXBounds, "C++: mrpt::opengl::CMeshFast::getXBounds(float &, float &) const --> void", pybind11::arg("min"), pybind11::arg("max"));
		cl.def("getYBounds", (void (mrpt::opengl::CMeshFast::*)(float &, float &) const) &mrpt::opengl::CMeshFast::getYBounds, "C++: mrpt::opengl::CMeshFast::getYBounds(float &, float &) const --> void", pybind11::arg("min"), pybind11::arg("max"));
		cl.def("setXBounds", (void (mrpt::opengl::CMeshFast::*)(float, float)) &mrpt::opengl::CMeshFast::setXBounds, "C++: mrpt::opengl::CMeshFast::setXBounds(float, float) --> void", pybind11::arg("min"), pybind11::arg("max"));
		cl.def("setYBounds", (void (mrpt::opengl::CMeshFast::*)(float, float)) &mrpt::opengl::CMeshFast::setYBounds, "C++: mrpt::opengl::CMeshFast::setYBounds(float, float) --> void", pybind11::arg("min"), pybind11::arg("max"));
		cl.def("internalBoundingBoxLocal", (struct mrpt::math::TBoundingBox_<float> (mrpt::opengl::CMeshFast::*)() const) &mrpt::opengl::CMeshFast::internalBoundingBoxLocal, "C++: mrpt::opengl::CMeshFast::internalBoundingBoxLocal() const --> struct mrpt::math::TBoundingBox_<float>");
		cl.def("assignImage", (void (mrpt::opengl::CMeshFast::*)(const class mrpt::img::CImage &)) &mrpt::opengl::CMeshFast::assignImage, "Assigns a texture image, and disable transparency.\n\nC++: mrpt::opengl::CMeshFast::assignImage(const class mrpt::img::CImage &) --> void", pybind11::arg("img"));
		cl.def("assignImageAndZ", (void (mrpt::opengl::CMeshFast::*)(const class mrpt::img::CImage &, const class mrpt::math::CMatrixDynamic<float> &)) &mrpt::opengl::CMeshFast::assignImageAndZ, "Assigns a texture image and Z simultaneously, and disable transparency.\n\nC++: mrpt::opengl::CMeshFast::assignImageAndZ(const class mrpt::img::CImage &, const class mrpt::math::CMatrixDynamic<float> &) --> void", pybind11::arg("img"), pybind11::arg("in_Z"));
		cl.def("adjustGridToImageAR", (void (mrpt::opengl::CMeshFast::*)()) &mrpt::opengl::CMeshFast::adjustGridToImageAR, "Adjust grid limits according to the image aspect ratio, maintaining the\n X limits and resizing in the Y direction.\n\nC++: mrpt::opengl::CMeshFast::adjustGridToImageAR() --> void");
		cl.def("assign", (class mrpt::opengl::CMeshFast & (mrpt::opengl::CMeshFast::*)(const class mrpt::opengl::CMeshFast &)) &mrpt::opengl::CMeshFast::operator=, "C++: mrpt::opengl::CMeshFast::operator=(const class mrpt::opengl::CMeshFast &) --> class mrpt::opengl::CMeshFast &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::opengl::CSimpleLine file:mrpt/opengl/CSimpleLine.h line:20
		pybind11::class_<mrpt::opengl::CSimpleLine, std::shared_ptr<mrpt::opengl::CSimpleLine>, PyCallBack_mrpt_opengl_CSimpleLine, mrpt::opengl::CRenderizableShaderWireFrame> cl(M("mrpt::opengl"), "CSimpleLine", "A line segment\n  \n\n opengl::Scene\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::CSimpleLine(); }, [](){ return new PyCallBack_mrpt_opengl_CSimpleLine(); } ), "doc");
		cl.def( pybind11::init( [](float const & a0){ return new mrpt::opengl::CSimpleLine(a0); }, [](float const & a0){ return new PyCallBack_mrpt_opengl_CSimpleLine(a0); } ), "doc");
		cl.def( pybind11::init( [](float const & a0, float const & a1){ return new mrpt::opengl::CSimpleLine(a0, a1); }, [](float const & a0, float const & a1){ return new PyCallBack_mrpt_opengl_CSimpleLine(a0, a1); } ), "doc");
		cl.def( pybind11::init( [](float const & a0, float const & a1, float const & a2){ return new mrpt::opengl::CSimpleLine(a0, a1, a2); }, [](float const & a0, float const & a1, float const & a2){ return new PyCallBack_mrpt_opengl_CSimpleLine(a0, a1, a2); } ), "doc");
		cl.def( pybind11::init( [](float const & a0, float const & a1, float const & a2, float const & a3){ return new mrpt::opengl::CSimpleLine(a0, a1, a2, a3); }, [](float const & a0, float const & a1, float const & a2, float const & a3){ return new PyCallBack_mrpt_opengl_CSimpleLine(a0, a1, a2, a3); } ), "doc");
		cl.def( pybind11::init( [](float const & a0, float const & a1, float const & a2, float const & a3, float const & a4){ return new mrpt::opengl::CSimpleLine(a0, a1, a2, a3, a4); }, [](float const & a0, float const & a1, float const & a2, float const & a3, float const & a4){ return new PyCallBack_mrpt_opengl_CSimpleLine(a0, a1, a2, a3, a4); } ), "doc");
		cl.def( pybind11::init( [](float const & a0, float const & a1, float const & a2, float const & a3, float const & a4, float const & a5){ return new mrpt::opengl::CSimpleLine(a0, a1, a2, a3, a4, a5); }, [](float const & a0, float const & a1, float const & a2, float const & a3, float const & a4, float const & a5){ return new PyCallBack_mrpt_opengl_CSimpleLine(a0, a1, a2, a3, a4, a5); } ), "doc");
		cl.def( pybind11::init( [](float const & a0, float const & a1, float const & a2, float const & a3, float const & a4, float const & a5, float const & a6){ return new mrpt::opengl::CSimpleLine(a0, a1, a2, a3, a4, a5, a6); }, [](float const & a0, float const & a1, float const & a2, float const & a3, float const & a4, float const & a5, float const & a6){ return new PyCallBack_mrpt_opengl_CSimpleLine(a0, a1, a2, a3, a4, a5, a6); } ), "doc");
		cl.def( pybind11::init<float, float, float, float, float, float, float, bool>(), pybind11::arg("x0"), pybind11::arg("y0"), pybind11::arg("z0"), pybind11::arg("x1"), pybind11::arg("y1"), pybind11::arg("z1"), pybind11::arg("lineWidth"), pybind11::arg("antiAliasing") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_opengl_CSimpleLine const &o){ return new PyCallBack_mrpt_opengl_CSimpleLine(o); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::CSimpleLine const &o){ return new mrpt::opengl::CSimpleLine(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CSimpleLine::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CSimpleLine::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CSimpleLine::*)() const) &mrpt::opengl::CSimpleLine::GetRuntimeClass, "C++: mrpt::opengl::CSimpleLine::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::opengl::CSimpleLine::*)() const) &mrpt::opengl::CSimpleLine::clone, "C++: mrpt::opengl::CSimpleLine::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::opengl::CSimpleLine::CreateObject, "C++: mrpt::opengl::CSimpleLine::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("setLineCoords", (void (mrpt::opengl::CSimpleLine::*)(const struct mrpt::math::TPoint3D_<float> &, const struct mrpt::math::TPoint3D_<float> &)) &mrpt::opengl::CSimpleLine::setLineCoords, "C++: mrpt::opengl::CSimpleLine::setLineCoords(const struct mrpt::math::TPoint3D_<float> &, const struct mrpt::math::TPoint3D_<float> &) --> void", pybind11::arg("p0"), pybind11::arg("p1"));
		cl.def("getLineStart", (struct mrpt::math::TPoint3D_<float> (mrpt::opengl::CSimpleLine::*)() const) &mrpt::opengl::CSimpleLine::getLineStart, "C++: mrpt::opengl::CSimpleLine::getLineStart() const --> struct mrpt::math::TPoint3D_<float>");
		cl.def("getLineEnd", (struct mrpt::math::TPoint3D_<float> (mrpt::opengl::CSimpleLine::*)() const) &mrpt::opengl::CSimpleLine::getLineEnd, "C++: mrpt::opengl::CSimpleLine::getLineEnd() const --> struct mrpt::math::TPoint3D_<float>");
		cl.def("setLineCoords", (void (mrpt::opengl::CSimpleLine::*)(float, float, float, float, float, float)) &mrpt::opengl::CSimpleLine::setLineCoords, "(MRPT 2.3.1)\n\nC++: mrpt::opengl::CSimpleLine::setLineCoords(float, float, float, float, float, float) --> void", pybind11::arg("x0"), pybind11::arg("y0"), pybind11::arg("z0"), pybind11::arg("x1"), pybind11::arg("y1"), pybind11::arg("z1"));
		cl.def("getLineCoords", (void (mrpt::opengl::CSimpleLine::*)(float &, float &, float &, float &, float &, float &) const) &mrpt::opengl::CSimpleLine::getLineCoords, "(MRPT 2.3.1)\n\nC++: mrpt::opengl::CSimpleLine::getLineCoords(float &, float &, float &, float &, float &, float &) const --> void", pybind11::arg("x0"), pybind11::arg("y0"), pybind11::arg("z0"), pybind11::arg("x1"), pybind11::arg("y1"), pybind11::arg("z1"));
		cl.def("onUpdateBuffers_Wireframe", (void (mrpt::opengl::CSimpleLine::*)()) &mrpt::opengl::CSimpleLine::onUpdateBuffers_Wireframe, "C++: mrpt::opengl::CSimpleLine::onUpdateBuffers_Wireframe() --> void");
		cl.def("internalBoundingBoxLocal", (struct mrpt::math::TBoundingBox_<float> (mrpt::opengl::CSimpleLine::*)() const) &mrpt::opengl::CSimpleLine::internalBoundingBoxLocal, "C++: mrpt::opengl::CSimpleLine::internalBoundingBoxLocal() const --> struct mrpt::math::TBoundingBox_<float>");
		cl.def("assign", (class mrpt::opengl::CSimpleLine & (mrpt::opengl::CSimpleLine::*)(const class mrpt::opengl::CSimpleLine &)) &mrpt::opengl::CSimpleLine::operator=, "C++: mrpt::opengl::CSimpleLine::operator=(const class mrpt::opengl::CSimpleLine &) --> class mrpt::opengl::CSimpleLine &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
