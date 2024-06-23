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
#include <mrpt/core/aligned_allocator.h>
#include <mrpt/img/TColor.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/metric_map_types.h>
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
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/opengl/CAngularObservationMesh.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CRenderizableShaderTriangles.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CSetOfTriangles.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/opengl/TTriangle.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/tfest/TMatchingPair.h>
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

// mrpt::opengl::CSetOfTriangles file:mrpt/opengl/CSetOfTriangles.h line:22
struct PyCallBack_mrpt_opengl_CSetOfTriangles : public mrpt::opengl::CSetOfTriangles {
	using mrpt::opengl::CSetOfTriangles::CSetOfTriangles;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfTriangles *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CSetOfTriangles::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfTriangles *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CSetOfTriangles::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfTriangles *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CSetOfTriangles::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfTriangles *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSetOfTriangles::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfTriangles *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSetOfTriangles::serializeFrom(a0, a1);
	}
	void onUpdateBuffers_Triangles() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfTriangles *>(this), "onUpdateBuffers_Triangles");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSetOfTriangles::onUpdateBuffers_Triangles();
	}
	class mrpt::opengl::CRenderizable & setColor_u8(const struct mrpt::img::TColor & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfTriangles *>(this), "setColor_u8");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::opengl::CRenderizable &>::value) {
				static pybind11::detail::override_caster_t<class mrpt::opengl::CRenderizable &> caster;
				return pybind11::detail::cast_ref<class mrpt::opengl::CRenderizable &>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::opengl::CRenderizable &>(std::move(o));
		}
		return CSetOfTriangles::setColor_u8(a0);
	}
	class mrpt::opengl::CRenderizable & setColorA_u8(const unsigned char a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfTriangles *>(this), "setColorA_u8");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::opengl::CRenderizable &>::value) {
				static pybind11::detail::override_caster_t<class mrpt::opengl::CRenderizable &> caster;
				return pybind11::detail::cast_ref<class mrpt::opengl::CRenderizable &>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::opengl::CRenderizable &>(std::move(o));
		}
		return CSetOfTriangles::setColorA_u8(a0);
	}
	bool traceRay(const class mrpt::poses::CPose3D & a0, double & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfTriangles *>(this), "traceRay");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CSetOfTriangles::traceRay(a0, a1);
	}
	struct mrpt::math::TBoundingBox_<float> internalBoundingBoxLocal() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfTriangles *>(this), "internalBoundingBoxLocal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TBoundingBox_<float>>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TBoundingBox_<float>> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TBoundingBox_<float>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TBoundingBox_<float>>(std::move(o));
		}
		return CSetOfTriangles::internalBoundingBoxLocal();
	}
	void renderUpdateBuffers() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfTriangles *>(this), "renderUpdateBuffers");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRenderizableShaderTriangles::renderUpdateBuffers();
	}
	void freeOpenGLResources() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfTriangles *>(this), "freeOpenGLResources");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRenderizableShaderTriangles::freeOpenGLResources();
	}
	bool cullElegible() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfTriangles *>(this), "cullElegible");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfTriangles *>(this), "toYAMLMap");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfTriangles *>(this), "isCompositeObject");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfTriangles *>(this), "getLocalRepresentativePoint");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfTriangles *>(this), "initializeTextures");
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

// mrpt::opengl::CAngularObservationMesh file:mrpt/opengl/CAngularObservationMesh.h line:41
struct PyCallBack_mrpt_opengl_CAngularObservationMesh : public mrpt::opengl::CAngularObservationMesh {
	using mrpt::opengl::CAngularObservationMesh::CAngularObservationMesh;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CAngularObservationMesh *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CAngularObservationMesh::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CAngularObservationMesh *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CAngularObservationMesh::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CAngularObservationMesh *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CAngularObservationMesh::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CAngularObservationMesh *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CAngularObservationMesh::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CAngularObservationMesh *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CAngularObservationMesh::serializeFrom(a0, a1);
	}
	struct mrpt::math::TBoundingBox_<float> internalBoundingBoxLocal() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CAngularObservationMesh *>(this), "internalBoundingBoxLocal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TBoundingBox_<float>>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TBoundingBox_<float>> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TBoundingBox_<float>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TBoundingBox_<float>>(std::move(o));
		}
		return CAngularObservationMesh::internalBoundingBoxLocal();
	}
	void freeOpenGLResources() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CAngularObservationMesh *>(this), "freeOpenGLResources");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CAngularObservationMesh::freeOpenGLResources();
	}
	void renderUpdateBuffers() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CAngularObservationMesh *>(this), "renderUpdateBuffers");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CAngularObservationMesh::renderUpdateBuffers();
	}
	void onUpdateBuffers_Wireframe() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CAngularObservationMesh *>(this), "onUpdateBuffers_Wireframe");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CAngularObservationMesh::onUpdateBuffers_Wireframe();
	}
	void onUpdateBuffers_Triangles() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CAngularObservationMesh *>(this), "onUpdateBuffers_Triangles");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CAngularObservationMesh::onUpdateBuffers_Triangles();
	}
	bool traceRay(const class mrpt::poses::CPose3D & a0, double & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CAngularObservationMesh *>(this), "traceRay");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CAngularObservationMesh::traceRay(a0, a1);
	}
	class mrpt::opengl::CRenderizable & setColorA_u8(const unsigned char a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CAngularObservationMesh *>(this), "setColorA_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CAngularObservationMesh *>(this), "setColor_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CAngularObservationMesh *>(this), "cullElegible");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CAngularObservationMesh *>(this), "toYAMLMap");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CAngularObservationMesh *>(this), "isCompositeObject");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CAngularObservationMesh *>(this), "getLocalRepresentativePoint");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CAngularObservationMesh *>(this), "initializeTextures");
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

void bind_mrpt_opengl_CSetOfTriangles(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::opengl::CSetOfTriangles file:mrpt/opengl/CSetOfTriangles.h line:22
		pybind11::class_<mrpt::opengl::CSetOfTriangles, std::shared_ptr<mrpt::opengl::CSetOfTriangles>, PyCallBack_mrpt_opengl_CSetOfTriangles, mrpt::opengl::CRenderizableShaderTriangles> cl(M("mrpt::opengl"), "CSetOfTriangles", "A set of colored triangles, able to draw any solid, arbitrarily complex\n object without textures. For textures, see CSetOfTexturedTriangles\n\n \n opengl::Scene, CSetOfTexturedTriangles\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::CSetOfTriangles(); }, [](){ return new PyCallBack_mrpt_opengl_CSetOfTriangles(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_opengl_CSetOfTriangles const &o){ return new PyCallBack_mrpt_opengl_CSetOfTriangles(o); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::CSetOfTriangles const &o){ return new mrpt::opengl::CSetOfTriangles(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CSetOfTriangles::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CSetOfTriangles::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CSetOfTriangles::*)() const) &mrpt::opengl::CSetOfTriangles::GetRuntimeClass, "C++: mrpt::opengl::CSetOfTriangles::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::opengl::CSetOfTriangles::*)() const) &mrpt::opengl::CSetOfTriangles::clone, "C++: mrpt::opengl::CSetOfTriangles::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::opengl::CSetOfTriangles::CreateObject, "C++: mrpt::opengl::CSetOfTriangles::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("onUpdateBuffers_Triangles", (void (mrpt::opengl::CSetOfTriangles::*)()) &mrpt::opengl::CSetOfTriangles::onUpdateBuffers_Triangles, "@{ \n\nC++: mrpt::opengl::CSetOfTriangles::onUpdateBuffers_Triangles() --> void");
		cl.def("updatePolygons", (void (mrpt::opengl::CSetOfTriangles::*)() const) &mrpt::opengl::CSetOfTriangles::updatePolygons, "Explicitly updates the internal polygon cache, with all triangles as\n polygons. \n\n getPolygons() \n\nC++: mrpt::opengl::CSetOfTriangles::updatePolygons() const --> void");
		cl.def("clearTriangles", (void (mrpt::opengl::CSetOfTriangles::*)()) &mrpt::opengl::CSetOfTriangles::clearTriangles, "Clear this object, removing all triangles. \n\nC++: mrpt::opengl::CSetOfTriangles::clearTriangles() --> void");
		cl.def("getTrianglesCount", (size_t (mrpt::opengl::CSetOfTriangles::*)() const) &mrpt::opengl::CSetOfTriangles::getTrianglesCount, "Get triangle count \n\nC++: mrpt::opengl::CSetOfTriangles::getTrianglesCount() const --> size_t");
		cl.def("getTriangle", (void (mrpt::opengl::CSetOfTriangles::*)(size_t, struct mrpt::opengl::TTriangle &) const) &mrpt::opengl::CSetOfTriangles::getTriangle, "Gets the i-th triangle \n\nC++: mrpt::opengl::CSetOfTriangles::getTriangle(size_t, struct mrpt::opengl::TTriangle &) const --> void", pybind11::arg("idx"), pybind11::arg("t"));
		cl.def("insertTriangle", (void (mrpt::opengl::CSetOfTriangles::*)(const struct mrpt::opengl::TTriangle &)) &mrpt::opengl::CSetOfTriangles::insertTriangle, "Inserts a triangle into the set \n\nC++: mrpt::opengl::CSetOfTriangles::insertTriangle(const struct mrpt::opengl::TTriangle &) --> void", pybind11::arg("t"));
		cl.def("insertTriangles", (void (mrpt::opengl::CSetOfTriangles::*)(const class std::shared_ptr<class mrpt::opengl::CSetOfTriangles> &)) &mrpt::opengl::CSetOfTriangles::insertTriangles, "Inserts an existing CSetOfTriangles into this one \n\nC++: mrpt::opengl::CSetOfTriangles::insertTriangles(const class std::shared_ptr<class mrpt::opengl::CSetOfTriangles> &) --> void", pybind11::arg("p"));
		cl.def("reserve", (void (mrpt::opengl::CSetOfTriangles::*)(size_t)) &mrpt::opengl::CSetOfTriangles::reserve, "Reserves memory for certain number of triangles, avoiding multiple\n memory allocation calls.\n\nC++: mrpt::opengl::CSetOfTriangles::reserve(size_t) --> void", pybind11::arg("t"));
		cl.def("setColor_u8", (class mrpt::opengl::CRenderizable & (mrpt::opengl::CSetOfTriangles::*)(const struct mrpt::img::TColor &)) &mrpt::opengl::CSetOfTriangles::setColor_u8, "Overwrite all triangles colors with the one provided \n\nC++: mrpt::opengl::CSetOfTriangles::setColor_u8(const struct mrpt::img::TColor &) --> class mrpt::opengl::CRenderizable &", pybind11::return_value_policy::automatic, pybind11::arg("c"));
		cl.def("setColorA_u8", (class mrpt::opengl::CRenderizable & (mrpt::opengl::CSetOfTriangles::*)(const unsigned char)) &mrpt::opengl::CSetOfTriangles::setColorA_u8, "Overwrite all triangles colors with the one provided \n\nC++: mrpt::opengl::CSetOfTriangles::setColorA_u8(const unsigned char) --> class mrpt::opengl::CRenderizable &", pybind11::return_value_policy::automatic, pybind11::arg("a"));
		cl.def("traceRay", (bool (mrpt::opengl::CSetOfTriangles::*)(const class mrpt::poses::CPose3D &, double &) const) &mrpt::opengl::CSetOfTriangles::traceRay, "C++: mrpt::opengl::CSetOfTriangles::traceRay(const class mrpt::poses::CPose3D &, double &) const --> bool", pybind11::arg("o"), pybind11::arg("dist"));
		cl.def("internalBoundingBoxLocal", (struct mrpt::math::TBoundingBox_<float> (mrpt::opengl::CSetOfTriangles::*)() const) &mrpt::opengl::CSetOfTriangles::internalBoundingBoxLocal, "Evaluates the bounding box of this object (including possible children)\n in the coordinate frame of the object parent. \n\nC++: mrpt::opengl::CSetOfTriangles::internalBoundingBoxLocal() const --> struct mrpt::math::TBoundingBox_<float>");
		cl.def("assign", (class mrpt::opengl::CSetOfTriangles & (mrpt::opengl::CSetOfTriangles::*)(const class mrpt::opengl::CSetOfTriangles &)) &mrpt::opengl::CSetOfTriangles::operator=, "C++: mrpt::opengl::CSetOfTriangles::operator=(const class mrpt::opengl::CSetOfTriangles &) --> class mrpt::opengl::CSetOfTriangles &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::opengl::CAngularObservationMesh file:mrpt/opengl/CAngularObservationMesh.h line:41
		pybind11::class_<mrpt::opengl::CAngularObservationMesh, std::shared_ptr<mrpt::opengl::CAngularObservationMesh>, PyCallBack_mrpt_opengl_CAngularObservationMesh, mrpt::opengl::CRenderizableShaderTriangles, mrpt::opengl::CRenderizableShaderWireFrame> cl(M("mrpt::opengl"), "CAngularObservationMesh", "A mesh built from a set of 2D laser scan observations.\n Each element of this set is a single scan through the yaw, given a specific\n pitch.\n Each scan has a mrpt::poses::CPose3D identifying the origin of the scan,\n which ideally is the\n same for every one of them.\n\n  \n  \n     mrpt::opengl::CAngularObservationMesh   \n\n\n preview_CAngularObservationMesh.png  \n  \n  \n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::CAngularObservationMesh(); }, [](){ return new PyCallBack_mrpt_opengl_CAngularObservationMesh(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_opengl_CAngularObservationMesh const &o){ return new PyCallBack_mrpt_opengl_CAngularObservationMesh(o); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::CAngularObservationMesh const &o){ return new mrpt::opengl::CAngularObservationMesh(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CAngularObservationMesh::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CAngularObservationMesh::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CAngularObservationMesh::*)() const) &mrpt::opengl::CAngularObservationMesh::GetRuntimeClass, "C++: mrpt::opengl::CAngularObservationMesh::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::opengl::CAngularObservationMesh::*)() const) &mrpt::opengl::CAngularObservationMesh::clone, "C++: mrpt::opengl::CAngularObservationMesh::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::opengl::CAngularObservationMesh::CreateObject, "C++: mrpt::opengl::CAngularObservationMesh::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("internalBoundingBoxLocal", (struct mrpt::math::TBoundingBox_<float> (mrpt::opengl::CAngularObservationMesh::*)() const) &mrpt::opengl::CAngularObservationMesh::internalBoundingBoxLocal, "C++: mrpt::opengl::CAngularObservationMesh::internalBoundingBoxLocal() const --> struct mrpt::math::TBoundingBox_<float>");
		cl.def("isWireframe", (bool (mrpt::opengl::CAngularObservationMesh::*)() const) &mrpt::opengl::CAngularObservationMesh::isWireframe, "Returns whether the object is configured as wireframe or solid.\n\nC++: mrpt::opengl::CAngularObservationMesh::isWireframe() const --> bool");
		cl.def("setWireframe", [](mrpt::opengl::CAngularObservationMesh &o) -> void { return o.setWireframe(); }, "");
		cl.def("setWireframe", (void (mrpt::opengl::CAngularObservationMesh::*)(bool)) &mrpt::opengl::CAngularObservationMesh::setWireframe, "Sets the display mode for the object. True=wireframe, False=solid.\n\nC++: mrpt::opengl::CAngularObservationMesh::setWireframe(bool) --> void", pybind11::arg("enabled"));
		cl.def("isTransparencyEnabled", (bool (mrpt::opengl::CAngularObservationMesh::*)() const) &mrpt::opengl::CAngularObservationMesh::isTransparencyEnabled, "Returns whether the object may be transparent or not.\n\nC++: mrpt::opengl::CAngularObservationMesh::isTransparencyEnabled() const --> bool");
		cl.def("enableTransparency", [](mrpt::opengl::CAngularObservationMesh &o) -> void { return o.enableTransparency(); }, "");
		cl.def("enableTransparency", (void (mrpt::opengl::CAngularObservationMesh::*)(bool)) &mrpt::opengl::CAngularObservationMesh::enableTransparency, "Enables or disables transparencies.\n\nC++: mrpt::opengl::CAngularObservationMesh::enableTransparency(bool) --> void", pybind11::arg("enabled"));
		cl.def("freeOpenGLResources", (void (mrpt::opengl::CAngularObservationMesh::*)()) &mrpt::opengl::CAngularObservationMesh::freeOpenGLResources, "@{ \n\nC++: mrpt::opengl::CAngularObservationMesh::freeOpenGLResources() --> void");
		cl.def("renderUpdateBuffers", (void (mrpt::opengl::CAngularObservationMesh::*)() const) &mrpt::opengl::CAngularObservationMesh::renderUpdateBuffers, "C++: mrpt::opengl::CAngularObservationMesh::renderUpdateBuffers() const --> void");
		cl.def("onUpdateBuffers_Wireframe", (void (mrpt::opengl::CAngularObservationMesh::*)()) &mrpt::opengl::CAngularObservationMesh::onUpdateBuffers_Wireframe, "C++: mrpt::opengl::CAngularObservationMesh::onUpdateBuffers_Wireframe() --> void");
		cl.def("onUpdateBuffers_Triangles", (void (mrpt::opengl::CAngularObservationMesh::*)()) &mrpt::opengl::CAngularObservationMesh::onUpdateBuffers_Triangles, "C++: mrpt::opengl::CAngularObservationMesh::onUpdateBuffers_Triangles() --> void");
		cl.def("traceRay", (bool (mrpt::opengl::CAngularObservationMesh::*)(const class mrpt::poses::CPose3D &, double &) const) &mrpt::opengl::CAngularObservationMesh::traceRay, "Traces a ray to the object, returning the distance to a given pose\n through its X axis.\n \n\n mrpt::opengl::CRenderizable,trace2DSetOfRays,trace1DSetOfRays\n\nC++: mrpt::opengl::CAngularObservationMesh::traceRay(const class mrpt::poses::CPose3D &, double &) const --> bool", pybind11::arg("o"), pybind11::arg("dist"));
		cl.def("setPitchBounds", (void (mrpt::opengl::CAngularObservationMesh::*)(const double, const double)) &mrpt::opengl::CAngularObservationMesh::setPitchBounds, "Sets the pitch bounds for this range.\n\nC++: mrpt::opengl::CAngularObservationMesh::setPitchBounds(const double, const double) --> void", pybind11::arg("initial"), pybind11::arg("final"));
		cl.def("getPitchBounds", (void (mrpt::opengl::CAngularObservationMesh::*)(double &, double &) const) &mrpt::opengl::CAngularObservationMesh::getPitchBounds, "Gets the initial and final pitch bounds for this range.\n\nC++: mrpt::opengl::CAngularObservationMesh::getPitchBounds(double &, double &) const --> void", pybind11::arg("initial"), pybind11::arg("final"));
		cl.def("generateSetOfTriangles", (void (mrpt::opengl::CAngularObservationMesh::*)(class std::shared_ptr<class mrpt::opengl::CSetOfTriangles> &) const) &mrpt::opengl::CAngularObservationMesh::generateSetOfTriangles, "Gets the mesh as a set of triangles, for displaying them.\n \n\n generateSetOfTriangles(std::vector<TPolygon3D>\n &),mrpt::opengl::CSetOfTriangles,mrpt::opengl::mrpt::opengl::TTriangle\n\nC++: mrpt::opengl::CAngularObservationMesh::generateSetOfTriangles(class std::shared_ptr<class mrpt::opengl::CSetOfTriangles> &) const --> void", pybind11::arg("res"));
		cl.def("generatePointCloud", (void (mrpt::opengl::CAngularObservationMesh::*)(class mrpt::maps::CPointsMap *) const) &mrpt::opengl::CAngularObservationMesh::generatePointCloud, "Returns the scanned points as a 3D point cloud. The target pointmap must\n be passed as a pointer to allow the use of any derived class.\n\nC++: mrpt::opengl::CAngularObservationMesh::generatePointCloud(class mrpt::maps::CPointsMap *) const --> void", pybind11::arg("out_map"));
		cl.def("getTracedRays", (void (mrpt::opengl::CAngularObservationMesh::*)(class std::shared_ptr<class mrpt::opengl::CSetOfLines> &) const) &mrpt::opengl::CAngularObservationMesh::getTracedRays, "Gets a set of lines containing the traced rays, for displaying them.\n \n\n getUntracedRays,mrpt::opengl::CSetOfLines\n\nC++: mrpt::opengl::CAngularObservationMesh::getTracedRays(class std::shared_ptr<class mrpt::opengl::CSetOfLines> &) const --> void", pybind11::arg("res"));
		cl.def("getUntracedRays", (void (mrpt::opengl::CAngularObservationMesh::*)(class std::shared_ptr<class mrpt::opengl::CSetOfLines> &, double) const) &mrpt::opengl::CAngularObservationMesh::getUntracedRays, "Gets a set of lines containing the untraced rays, up to a specified\n distance, for displaying them.\n \n\n getTracedRays,mrpt::opengl::CSetOfLines\n\nC++: mrpt::opengl::CAngularObservationMesh::getUntracedRays(class std::shared_ptr<class mrpt::opengl::CSetOfLines> &, double) const --> void", pybind11::arg("res"), pybind11::arg("dist"));
		cl.def("assign", (class mrpt::opengl::CAngularObservationMesh & (mrpt::opengl::CAngularObservationMesh::*)(const class mrpt::opengl::CAngularObservationMesh &)) &mrpt::opengl::CAngularObservationMesh::operator=, "C++: mrpt::opengl::CAngularObservationMesh::operator=(const class mrpt::opengl::CAngularObservationMesh &) --> class mrpt::opengl::CAngularObservationMesh &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::opengl::CAngularObservationMesh::TDoubleRange file:mrpt/opengl/CAngularObservationMesh.h line:50
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::opengl::CAngularObservationMesh::TDoubleRange, std::shared_ptr<mrpt::opengl::CAngularObservationMesh::TDoubleRange>> cl(enclosing_class, "TDoubleRange", "Range specification type, with several uses.");
			cl.def( pybind11::init<double, double, double>(), pybind11::arg("a"), pybind11::arg("b"), pybind11::arg("c") );

			cl.def( pybind11::init<double, double, size_t>(), pybind11::arg("a"), pybind11::arg("b"), pybind11::arg("c") );

			cl.def( pybind11::init<double, size_t, bool>(), pybind11::arg("a"), pybind11::arg("b"), pybind11::arg("c") );

			cl.def( pybind11::init( [](mrpt::opengl::CAngularObservationMesh::TDoubleRange const &o){ return new mrpt::opengl::CAngularObservationMesh::TDoubleRange(o); } ) );
			cl.def_static("CreateFromIncrement", (struct mrpt::opengl::CAngularObservationMesh::TDoubleRange (*)(double, double, double)) &mrpt::opengl::CAngularObservationMesh::TDoubleRange::CreateFromIncrement, "Creates a range of values from the initial value, the final value\n and the increment.\n \n\n std::logic_error if the increment is zero.\n\nC++: mrpt::opengl::CAngularObservationMesh::TDoubleRange::CreateFromIncrement(double, double, double) --> struct mrpt::opengl::CAngularObservationMesh::TDoubleRange", pybind11::arg("initial"), pybind11::arg("final"), pybind11::arg("increment"));
			cl.def_static("CreateFromAmount", (struct mrpt::opengl::CAngularObservationMesh::TDoubleRange (*)(double, double, size_t)) &mrpt::opengl::CAngularObservationMesh::TDoubleRange::CreateFromAmount, "Creates a range of values from the initial value, the final value\n and a desired amount of samples.\n\nC++: mrpt::opengl::CAngularObservationMesh::TDoubleRange::CreateFromAmount(double, double, size_t) --> struct mrpt::opengl::CAngularObservationMesh::TDoubleRange", pybind11::arg("initial"), pybind11::arg("final"), pybind11::arg("amount"));
			cl.def_static("CreateFromAperture", [](double const & a0, size_t const & a1) -> mrpt::opengl::CAngularObservationMesh::TDoubleRange { return mrpt::opengl::CAngularObservationMesh::TDoubleRange::CreateFromAperture(a0, a1); }, "", pybind11::arg("aperture"), pybind11::arg("amount"));
			cl.def_static("CreateFromAperture", (struct mrpt::opengl::CAngularObservationMesh::TDoubleRange (*)(double, size_t, bool)) &mrpt::opengl::CAngularObservationMesh::TDoubleRange::CreateFromAperture, "Creates a zero-centered range of values from an aperture, an amount\n of samples and a direction.\n\nC++: mrpt::opengl::CAngularObservationMesh::TDoubleRange::CreateFromAperture(double, size_t, bool) --> struct mrpt::opengl::CAngularObservationMesh::TDoubleRange", pybind11::arg("aperture"), pybind11::arg("amount"), pybind11::arg("negToPos"));
			cl.def("aperture", (double (mrpt::opengl::CAngularObservationMesh::TDoubleRange::*)() const) &mrpt::opengl::CAngularObservationMesh::TDoubleRange::aperture, "Returns the total aperture of the range.\n \n\n std::logic_error on invalid range type.\n\nC++: mrpt::opengl::CAngularObservationMesh::TDoubleRange::aperture() const --> double");
			cl.def("initialValue", (double (mrpt::opengl::CAngularObservationMesh::TDoubleRange::*)() const) &mrpt::opengl::CAngularObservationMesh::TDoubleRange::initialValue, "Returns the first value of the range.\n \n\n std::logic_error on invalid range type.\n\nC++: mrpt::opengl::CAngularObservationMesh::TDoubleRange::initialValue() const --> double");
			cl.def("finalValue", (double (mrpt::opengl::CAngularObservationMesh::TDoubleRange::*)() const) &mrpt::opengl::CAngularObservationMesh::TDoubleRange::finalValue, "Returns the last value of the range.\n \n\n std::logic_error on invalid range type.\n\nC++: mrpt::opengl::CAngularObservationMesh::TDoubleRange::finalValue() const --> double");
			cl.def("increment", (double (mrpt::opengl::CAngularObservationMesh::TDoubleRange::*)() const) &mrpt::opengl::CAngularObservationMesh::TDoubleRange::increment, "Returns the increment between two consecutive values of the range.\n \n\n std::logic_error on invalid range type.\n\nC++: mrpt::opengl::CAngularObservationMesh::TDoubleRange::increment() const --> double");
			cl.def("amount", (size_t (mrpt::opengl::CAngularObservationMesh::TDoubleRange::*)() const) &mrpt::opengl::CAngularObservationMesh::TDoubleRange::amount, "Returns the total amount of values in this range.\n \n\n std::logic_error on invalid range type.\n\nC++: mrpt::opengl::CAngularObservationMesh::TDoubleRange::amount() const --> size_t");
			cl.def("negToPos", (bool (mrpt::opengl::CAngularObservationMesh::TDoubleRange::*)() const) &mrpt::opengl::CAngularObservationMesh::TDoubleRange::negToPos, "Returns the direction of the scan. True if the increment is\n positive, false otherwise.\n \n\n std::logic_error on invalid range type.\n\nC++: mrpt::opengl::CAngularObservationMesh::TDoubleRange::negToPos() const --> bool");
		}

	}
}
