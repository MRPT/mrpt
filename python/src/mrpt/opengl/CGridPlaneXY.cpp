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
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TPose3DQuat.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CRenderizableShaderPoints.h>
#include <mrpt/opengl/CRenderizableShaderWireFrame.h>
#include <mrpt/opengl/CSetOfObjects.h>
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

// mrpt::opengl::CGridPlaneXY file:mrpt/opengl/CGridPlaneXY.h line:23
struct PyCallBack_mrpt_opengl_CGridPlaneXY : public mrpt::opengl::CGridPlaneXY {
	using mrpt::opengl::CGridPlaneXY::CGridPlaneXY;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CGridPlaneXY *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CGridPlaneXY::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CGridPlaneXY *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CGridPlaneXY::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CGridPlaneXY *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CGridPlaneXY::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CGridPlaneXY *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGridPlaneXY::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CGridPlaneXY *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGridPlaneXY::serializeFrom(a0, a1);
	}
	void onUpdateBuffers_Wireframe() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CGridPlaneXY *>(this), "onUpdateBuffers_Wireframe");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGridPlaneXY::onUpdateBuffers_Wireframe();
	}
	struct mrpt::math::TBoundingBox_<float> internalBoundingBoxLocal() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CGridPlaneXY *>(this), "internalBoundingBoxLocal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TBoundingBox_<float>>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TBoundingBox_<float>> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TBoundingBox_<float>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TBoundingBox_<float>>(std::move(o));
		}
		return CGridPlaneXY::internalBoundingBoxLocal();
	}
	void renderUpdateBuffers() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CGridPlaneXY *>(this), "renderUpdateBuffers");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CGridPlaneXY *>(this), "freeOpenGLResources");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CGridPlaneXY *>(this), "setColorA_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CGridPlaneXY *>(this), "setColor_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CGridPlaneXY *>(this), "cullElegible");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CGridPlaneXY *>(this), "toYAMLMap");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CGridPlaneXY *>(this), "isCompositeObject");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CGridPlaneXY *>(this), "traceRay");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CGridPlaneXY *>(this), "getLocalRepresentativePoint");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CGridPlaneXY *>(this), "initializeTextures");
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

// mrpt::opengl::CPointCloud file:mrpt/opengl/CPointCloud.h line:34
struct PyCallBack_mrpt_opengl_CPointCloud : public mrpt::opengl::CPointCloud {
	using mrpt::opengl::CPointCloud::CPointCloud;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloud *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPointCloud::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloud *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CPointCloud::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloud *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CPointCloud::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloud *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointCloud::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloud *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointCloud::serializeFrom(a0, a1);
	}
	void PLY_import_set_vertex_count(size_t a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloud *>(this), "PLY_import_set_vertex_count");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointCloud::PLY_import_set_vertex_count(a0);
	}
	void PLY_import_set_vertex_timestamp(size_t a0, const double a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloud *>(this), "PLY_import_set_vertex_timestamp");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointCloud::PLY_import_set_vertex_timestamp(a0, a1);
	}
	void PLY_import_set_face_count(size_t a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloud *>(this), "PLY_import_set_face_count");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointCloud::PLY_import_set_face_count(a0);
	}
	void PLY_import_set_vertex(size_t a0, const struct mrpt::math::TPoint3D_<float> & a1, const struct mrpt::img::TColorf * a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloud *>(this), "PLY_import_set_vertex");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointCloud::PLY_import_set_vertex(a0, a1, a2);
	}
	size_t PLY_export_get_vertex_count() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloud *>(this), "PLY_export_get_vertex_count");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CPointCloud::PLY_export_get_vertex_count();
	}
	size_t PLY_export_get_face_count() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloud *>(this), "PLY_export_get_face_count");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CPointCloud::PLY_export_get_face_count();
	}
	void PLY_export_get_vertex(size_t a0, struct mrpt::math::TPoint3D_<float> & a1, bool & a2, struct mrpt::img::TColorf & a3) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloud *>(this), "PLY_export_get_vertex");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointCloud::PLY_export_get_vertex(a0, a1, a2, a3);
	}
	struct mrpt::math::TBoundingBox_<float> internalBoundingBoxLocal() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloud *>(this), "internalBoundingBoxLocal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TBoundingBox_<float>>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TBoundingBox_<float>> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TBoundingBox_<float>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TBoundingBox_<float>>(std::move(o));
		}
		return CPointCloud::internalBoundingBoxLocal();
	}
	void onUpdateBuffers_Points() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloud *>(this), "onUpdateBuffers_Points");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointCloud::onUpdateBuffers_Points();
	}
	void toYAMLMap(class mrpt::containers::yaml & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloud *>(this), "toYAMLMap");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointCloud::toYAMLMap(a0);
	}
	void renderUpdateBuffers() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloud *>(this), "renderUpdateBuffers");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloud *>(this), "freeOpenGLResources");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloud *>(this), "setColorA_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloud *>(this), "setColor_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloud *>(this), "cullElegible");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloud *>(this), "isCompositeObject");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloud *>(this), "traceRay");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloud *>(this), "getLocalRepresentativePoint");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloud *>(this), "initializeTextures");
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

void bind_mrpt_opengl_CGridPlaneXY(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::opengl::CGridPlaneXY file:mrpt/opengl/CGridPlaneXY.h line:23
		pybind11::class_<mrpt::opengl::CGridPlaneXY, std::shared_ptr<mrpt::opengl::CGridPlaneXY>, PyCallBack_mrpt_opengl_CGridPlaneXY, mrpt::opengl::CRenderizableShaderWireFrame> cl(M("mrpt::opengl"), "CGridPlaneXY", "A grid of lines over the XY plane.\n\n ![mrpt::opengl::CGridPlaneXY](preview_CGridPlaneXY.png)\n\n \n opengl::Scene\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::CGridPlaneXY(); }, [](){ return new PyCallBack_mrpt_opengl_CGridPlaneXY(); } ), "doc");
		cl.def( pybind11::init( [](float const & a0){ return new mrpt::opengl::CGridPlaneXY(a0); }, [](float const & a0){ return new PyCallBack_mrpt_opengl_CGridPlaneXY(a0); } ), "doc");
		cl.def( pybind11::init( [](float const & a0, float const & a1){ return new mrpt::opengl::CGridPlaneXY(a0, a1); }, [](float const & a0, float const & a1){ return new PyCallBack_mrpt_opengl_CGridPlaneXY(a0, a1); } ), "doc");
		cl.def( pybind11::init( [](float const & a0, float const & a1, float const & a2){ return new mrpt::opengl::CGridPlaneXY(a0, a1, a2); }, [](float const & a0, float const & a1, float const & a2){ return new PyCallBack_mrpt_opengl_CGridPlaneXY(a0, a1, a2); } ), "doc");
		cl.def( pybind11::init( [](float const & a0, float const & a1, float const & a2, float const & a3){ return new mrpt::opengl::CGridPlaneXY(a0, a1, a2, a3); }, [](float const & a0, float const & a1, float const & a2, float const & a3){ return new PyCallBack_mrpt_opengl_CGridPlaneXY(a0, a1, a2, a3); } ), "doc");
		cl.def( pybind11::init( [](float const & a0, float const & a1, float const & a2, float const & a3, float const & a4){ return new mrpt::opengl::CGridPlaneXY(a0, a1, a2, a3, a4); }, [](float const & a0, float const & a1, float const & a2, float const & a3, float const & a4){ return new PyCallBack_mrpt_opengl_CGridPlaneXY(a0, a1, a2, a3, a4); } ), "doc");
		cl.def( pybind11::init( [](float const & a0, float const & a1, float const & a2, float const & a3, float const & a4, float const & a5){ return new mrpt::opengl::CGridPlaneXY(a0, a1, a2, a3, a4, a5); }, [](float const & a0, float const & a1, float const & a2, float const & a3, float const & a4, float const & a5){ return new PyCallBack_mrpt_opengl_CGridPlaneXY(a0, a1, a2, a3, a4, a5); } ), "doc");
		cl.def( pybind11::init( [](float const & a0, float const & a1, float const & a2, float const & a3, float const & a4, float const & a5, float const & a6){ return new mrpt::opengl::CGridPlaneXY(a0, a1, a2, a3, a4, a5, a6); }, [](float const & a0, float const & a1, float const & a2, float const & a3, float const & a4, float const & a5, float const & a6){ return new PyCallBack_mrpt_opengl_CGridPlaneXY(a0, a1, a2, a3, a4, a5, a6); } ), "doc");
		cl.def( pybind11::init<float, float, float, float, float, float, float, bool>(), pybind11::arg("xMin"), pybind11::arg("xMax"), pybind11::arg("yMin"), pybind11::arg("yMax"), pybind11::arg("z"), pybind11::arg("frequency"), pybind11::arg("lineWidth"), pybind11::arg("antiAliasing") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_opengl_CGridPlaneXY const &o){ return new PyCallBack_mrpt_opengl_CGridPlaneXY(o); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::CGridPlaneXY const &o){ return new mrpt::opengl::CGridPlaneXY(o); } ) );
		cl.def_static("Create", (class std::shared_ptr<class mrpt::opengl::CGridPlaneXY> (*)(double &, double &, double &, double &, double &, const double &)) &mrpt::opengl::CGridPlaneXY::Create<double &, double &, double &, double &, double &, const double &>, "C++: mrpt::opengl::CGridPlaneXY::Create(double &, double &, double &, double &, double &, const double &) --> class std::shared_ptr<class mrpt::opengl::CGridPlaneXY>", pybind11::arg("args"), pybind11::arg("args"), pybind11::arg("args"), pybind11::arg("args"), pybind11::arg("args"), pybind11::arg("args"));
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CGridPlaneXY::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CGridPlaneXY::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CGridPlaneXY::*)() const) &mrpt::opengl::CGridPlaneXY::GetRuntimeClass, "C++: mrpt::opengl::CGridPlaneXY::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::opengl::CGridPlaneXY::*)() const) &mrpt::opengl::CGridPlaneXY::clone, "C++: mrpt::opengl::CGridPlaneXY::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::opengl::CGridPlaneXY::CreateObject, "C++: mrpt::opengl::CGridPlaneXY::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("setPlaneLimits", (void (mrpt::opengl::CGridPlaneXY::*)(float, float, float, float)) &mrpt::opengl::CGridPlaneXY::setPlaneLimits, "C++: mrpt::opengl::CGridPlaneXY::setPlaneLimits(float, float, float, float) --> void", pybind11::arg("xmin"), pybind11::arg("xmax"), pybind11::arg("ymin"), pybind11::arg("ymax"));
		cl.def("getPlaneLimits", (void (mrpt::opengl::CGridPlaneXY::*)(float &, float &, float &, float &) const) &mrpt::opengl::CGridPlaneXY::getPlaneLimits, "C++: mrpt::opengl::CGridPlaneXY::getPlaneLimits(float &, float &, float &, float &) const --> void", pybind11::arg("xmin"), pybind11::arg("xmax"), pybind11::arg("ymin"), pybind11::arg("ymax"));
		cl.def("setPlaneZcoord", (void (mrpt::opengl::CGridPlaneXY::*)(float)) &mrpt::opengl::CGridPlaneXY::setPlaneZcoord, "C++: mrpt::opengl::CGridPlaneXY::setPlaneZcoord(float) --> void", pybind11::arg("z"));
		cl.def("getPlaneZcoord", (float (mrpt::opengl::CGridPlaneXY::*)() const) &mrpt::opengl::CGridPlaneXY::getPlaneZcoord, "C++: mrpt::opengl::CGridPlaneXY::getPlaneZcoord() const --> float");
		cl.def("setGridFrequency", (void (mrpt::opengl::CGridPlaneXY::*)(float)) &mrpt::opengl::CGridPlaneXY::setGridFrequency, "C++: mrpt::opengl::CGridPlaneXY::setGridFrequency(float) --> void", pybind11::arg("freq"));
		cl.def("getGridFrequency", (float (mrpt::opengl::CGridPlaneXY::*)() const) &mrpt::opengl::CGridPlaneXY::getGridFrequency, "C++: mrpt::opengl::CGridPlaneXY::getGridFrequency() const --> float");
		cl.def("onUpdateBuffers_Wireframe", (void (mrpt::opengl::CGridPlaneXY::*)()) &mrpt::opengl::CGridPlaneXY::onUpdateBuffers_Wireframe, "C++: mrpt::opengl::CGridPlaneXY::onUpdateBuffers_Wireframe() --> void");
		cl.def("internalBoundingBoxLocal", (struct mrpt::math::TBoundingBox_<float> (mrpt::opengl::CGridPlaneXY::*)() const) &mrpt::opengl::CGridPlaneXY::internalBoundingBoxLocal, "C++: mrpt::opengl::CGridPlaneXY::internalBoundingBoxLocal() const --> struct mrpt::math::TBoundingBox_<float>");
		cl.def("assign", (class mrpt::opengl::CGridPlaneXY & (mrpt::opengl::CGridPlaneXY::*)(const class mrpt::opengl::CGridPlaneXY &)) &mrpt::opengl::CGridPlaneXY::operator=, "C++: mrpt::opengl::CGridPlaneXY::operator=(const class mrpt::opengl::CGridPlaneXY &) --> class mrpt::opengl::CGridPlaneXY &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::opengl::CPointCloud file:mrpt/opengl/CPointCloud.h line:34
		pybind11::class_<mrpt::opengl::CPointCloud, std::shared_ptr<mrpt::opengl::CPointCloud>, PyCallBack_mrpt_opengl_CPointCloud, mrpt::opengl::CRenderizableShaderPoints, mrpt::opengl::PLY_Importer, mrpt::opengl::PLY_Exporter> cl(M("mrpt::opengl"), "CPointCloud", "A cloud of points, all with the same color or each depending on its value\n along a particular coordinate axis.\n This class is just an OpenGL representation of a point cloud. For operating\n with maps of points, see mrpt::maps::CPointsMap and derived classes.\n\n To load from a points-map, CPointCloud::loadFromPointsMap().\n\n This class uses smart optimizations while rendering to efficiently draw\n clouds of millions of points, using octrees.\n\n ![mrpt::opengl::CPointCloud](preview_CPointCloud.png)\n\n  \n opengl::CPlanarLaserScan, opengl::Scene,\n opengl::CPointCloudColoured, mrpt::maps::CPointsMap \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::CPointCloud(); }, [](){ return new PyCallBack_mrpt_opengl_CPointCloud(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_opengl_CPointCloud const &o){ return new PyCallBack_mrpt_opengl_CPointCloud(o); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::CPointCloud const &o){ return new mrpt::opengl::CPointCloud(o); } ) );
		cl.def_static("Create", (class std::shared_ptr<class mrpt::opengl::CPointCloud> (*)()) &mrpt::opengl::CPointCloud::Create, "C++: mrpt::opengl::CPointCloud::Create() --> class std::shared_ptr<class mrpt::opengl::CPointCloud>");
		cl.def("loadFromPointsMap", (void (mrpt::opengl::CPointCloud::*)(const class mrpt::maps::CPointsMap *)) &mrpt::opengl::CPointCloud::loadFromPointsMap<mrpt::maps::CPointsMap>, "C++: mrpt::opengl::CPointCloud::loadFromPointsMap(const class mrpt::maps::CPointsMap *) --> void", pybind11::arg("themap"));
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CPointCloud::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CPointCloud::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CPointCloud::*)() const) &mrpt::opengl::CPointCloud::GetRuntimeClass, "C++: mrpt::opengl::CPointCloud::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::opengl::CPointCloud::*)() const) &mrpt::opengl::CPointCloud::clone, "C++: mrpt::opengl::CPointCloud::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::opengl::CPointCloud::CreateObject, "C++: mrpt::opengl::CPointCloud::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("size", (size_t (mrpt::opengl::CPointCloud::*)() const) &mrpt::opengl::CPointCloud::size, "@{ \n\nC++: mrpt::opengl::CPointCloud::size() const --> size_t");
		cl.def("size_unprotected", (size_t (mrpt::opengl::CPointCloud::*)() const) &mrpt::opengl::CPointCloud::size_unprotected, "Like size(), but without locking the data mutex (internal usage)\n\nC++: mrpt::opengl::CPointCloud::size_unprotected() const --> size_t");
		cl.def("resize", (void (mrpt::opengl::CPointCloud::*)(size_t)) &mrpt::opengl::CPointCloud::resize, "Set the number of points (with contents undefined) \n\nC++: mrpt::opengl::CPointCloud::resize(size_t) --> void", pybind11::arg("N"));
		cl.def("reserve", (void (mrpt::opengl::CPointCloud::*)(size_t)) &mrpt::opengl::CPointCloud::reserve, "Like STL std::vector's reserve \n\nC++: mrpt::opengl::CPointCloud::reserve(size_t) --> void", pybind11::arg("N"));
		cl.def("clear", (void (mrpt::opengl::CPointCloud::*)()) &mrpt::opengl::CPointCloud::clear, "Empty the list of points. \n\nC++: mrpt::opengl::CPointCloud::clear() --> void");
		cl.def("empty", (bool (mrpt::opengl::CPointCloud::*)() const) &mrpt::opengl::CPointCloud::empty, "C++: mrpt::opengl::CPointCloud::empty() const --> bool");
		cl.def("insertPoint", (void (mrpt::opengl::CPointCloud::*)(float, float, float)) &mrpt::opengl::CPointCloud::insertPoint, "Adds a new point to the cloud \n\nC++: mrpt::opengl::CPointCloud::insertPoint(float, float, float) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("insertPoint", (void (mrpt::opengl::CPointCloud::*)(const struct mrpt::math::TPoint3D_<float> &)) &mrpt::opengl::CPointCloud::insertPoint, "C++: mrpt::opengl::CPointCloud::insertPoint(const struct mrpt::math::TPoint3D_<float> &) --> void", pybind11::arg("p"));
		cl.def("insertPoint", (void (mrpt::opengl::CPointCloud::*)(const struct mrpt::math::TPoint3D_<double> &)) &mrpt::opengl::CPointCloud::insertPoint, "C++: mrpt::opengl::CPointCloud::insertPoint(const struct mrpt::math::TPoint3D_<double> &) --> void", pybind11::arg("p"));
		cl.def("__getitem__", (const struct mrpt::math::TPoint3D_<float> & (mrpt::opengl::CPointCloud::*)(size_t) const) &mrpt::opengl::CPointCloud::operator[], "Read access to each individual point (checks for \"i\" in the valid\n range only in Debug). \n\nC++: mrpt::opengl::CPointCloud::operator[](size_t) const --> const struct mrpt::math::TPoint3D_<float> &", pybind11::return_value_policy::automatic, pybind11::arg("i"));
		cl.def("getPoint3Df", (const struct mrpt::math::TPoint3D_<float> & (mrpt::opengl::CPointCloud::*)(size_t) const) &mrpt::opengl::CPointCloud::getPoint3Df, "NOTE: This method is intentionally not protected by the shared_mutex,\n since it's called in the inner loops of the octree, which acquires the\n lock once.\n\nC++: mrpt::opengl::CPointCloud::getPoint3Df(size_t) const --> const struct mrpt::math::TPoint3D_<float> &", pybind11::return_value_policy::automatic, pybind11::arg("i"));
		cl.def("setPoint", (void (mrpt::opengl::CPointCloud::*)(size_t, const float, const float, const float)) &mrpt::opengl::CPointCloud::setPoint, "Write an individual point (checks for \"i\" in the valid range only in\n Debug). \n\nC++: mrpt::opengl::CPointCloud::setPoint(size_t, const float, const float, const float) --> void", pybind11::arg("i"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("setPoint_fast", (void (mrpt::opengl::CPointCloud::*)(size_t, const float, const float, const float)) &mrpt::opengl::CPointCloud::setPoint_fast, "Write an individual point (without checking validity of the index).\n\nC++: mrpt::opengl::CPointCloud::setPoint_fast(size_t, const float, const float, const float) --> void", pybind11::arg("i"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("getActuallyRendered", (size_t (mrpt::opengl::CPointCloud::*)() const) &mrpt::opengl::CPointCloud::getActuallyRendered, "Get the number of elements actually rendered in the last render\n event.\n\nC++: mrpt::opengl::CPointCloud::getActuallyRendered() const --> size_t");
		cl.def("enableColorFromX", [](mrpt::opengl::CPointCloud &o) -> void { return o.enableColorFromX(); }, "");
		cl.def("enableColorFromX", (void (mrpt::opengl::CPointCloud::*)(bool)) &mrpt::opengl::CPointCloud::enableColorFromX, "@{ \n\nC++: mrpt::opengl::CPointCloud::enableColorFromX(bool) --> void", pybind11::arg("v"));
		cl.def("enableColorFromY", [](mrpt::opengl::CPointCloud &o) -> void { return o.enableColorFromY(); }, "");
		cl.def("enableColorFromY", (void (mrpt::opengl::CPointCloud::*)(bool)) &mrpt::opengl::CPointCloud::enableColorFromY, "C++: mrpt::opengl::CPointCloud::enableColorFromY(bool) --> void", pybind11::arg("v"));
		cl.def("enableColorFromZ", [](mrpt::opengl::CPointCloud &o) -> void { return o.enableColorFromZ(); }, "");
		cl.def("enableColorFromZ", (void (mrpt::opengl::CPointCloud::*)(bool)) &mrpt::opengl::CPointCloud::enableColorFromZ, "C++: mrpt::opengl::CPointCloud::enableColorFromZ(bool) --> void", pybind11::arg("v"));
		cl.def("enablePointSmooth", [](mrpt::opengl::CPointCloud &o) -> void { return o.enablePointSmooth(); }, "");
		cl.def("enablePointSmooth", (void (mrpt::opengl::CPointCloud::*)(bool)) &mrpt::opengl::CPointCloud::enablePointSmooth, "C++: mrpt::opengl::CPointCloud::enablePointSmooth(bool) --> void", pybind11::arg("enable"));
		cl.def("disablePointSmooth", (void (mrpt::opengl::CPointCloud::*)()) &mrpt::opengl::CPointCloud::disablePointSmooth, "C++: mrpt::opengl::CPointCloud::disablePointSmooth() --> void");
		cl.def("isPointSmoothEnabled", (bool (mrpt::opengl::CPointCloud::*)() const) &mrpt::opengl::CPointCloud::isPointSmoothEnabled, "C++: mrpt::opengl::CPointCloud::isPointSmoothEnabled() const --> bool");
		cl.def("setGradientColors", (void (mrpt::opengl::CPointCloud::*)(const struct mrpt::img::TColorf &, const struct mrpt::img::TColorf &)) &mrpt::opengl::CPointCloud::setGradientColors, "Sets the colors used as extremes when colorFromDepth is enabled. \n\nC++: mrpt::opengl::CPointCloud::setGradientColors(const struct mrpt::img::TColorf &, const struct mrpt::img::TColorf &) --> void", pybind11::arg("colorMin"), pybind11::arg("colorMax"));
		cl.def("onUpdateBuffers_Points", (void (mrpt::opengl::CPointCloud::*)()) &mrpt::opengl::CPointCloud::onUpdateBuffers_Points, "@} \n\nC++: mrpt::opengl::CPointCloud::onUpdateBuffers_Points() --> void");
		cl.def("toYAMLMap", (void (mrpt::opengl::CPointCloud::*)(class mrpt::containers::yaml &) const) &mrpt::opengl::CPointCloud::toYAMLMap, "C++: mrpt::opengl::CPointCloud::toYAMLMap(class mrpt::containers::yaml &) const --> void", pybind11::arg("propertiesMap"));
	}
}
