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
#include <mrpt/img/TColor.h>
#include <mrpt/img/color_maps.h>
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
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CRenderizableShaderPoints.h>
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

// mrpt::opengl::CPointCloudColoured file:mrpt/opengl/CPointCloudColoured.h line:34
struct PyCallBack_mrpt_opengl_CPointCloudColoured : public mrpt::opengl::CPointCloudColoured {
	using mrpt::opengl::CPointCloudColoured::CPointCloudColoured;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloudColoured *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPointCloudColoured::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloudColoured *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CPointCloudColoured::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloudColoured *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CPointCloudColoured::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloudColoured *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointCloudColoured::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloudColoured *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointCloudColoured::serializeFrom(a0, a1);
	}
	void onUpdateBuffers_Points() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloudColoured *>(this), "onUpdateBuffers_Points");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointCloudColoured::onUpdateBuffers_Points();
	}
	struct mrpt::math::TBoundingBox_<float> internalBoundingBoxLocal() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloudColoured *>(this), "internalBoundingBoxLocal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TBoundingBox_<float>>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TBoundingBox_<float>> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TBoundingBox_<float>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TBoundingBox_<float>>(std::move(o));
		}
		return CPointCloudColoured::internalBoundingBoxLocal();
	}
	void toYAMLMap(class mrpt::containers::yaml & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloudColoured *>(this), "toYAMLMap");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointCloudColoured::toYAMLMap(a0);
	}
	void PLY_import_set_vertex_count(size_t a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloudColoured *>(this), "PLY_import_set_vertex_count");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointCloudColoured::PLY_import_set_vertex_count(a0);
	}
	void PLY_import_set_face_count(size_t a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloudColoured *>(this), "PLY_import_set_face_count");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointCloudColoured::PLY_import_set_face_count(a0);
	}
	void PLY_import_set_vertex_timestamp(size_t a0, const double a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloudColoured *>(this), "PLY_import_set_vertex_timestamp");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointCloudColoured::PLY_import_set_vertex_timestamp(a0, a1);
	}
	void PLY_import_set_vertex(size_t a0, const struct mrpt::math::TPoint3D_<float> & a1, const struct mrpt::img::TColorf * a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloudColoured *>(this), "PLY_import_set_vertex");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointCloudColoured::PLY_import_set_vertex(a0, a1, a2);
	}
	size_t PLY_export_get_vertex_count() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloudColoured *>(this), "PLY_export_get_vertex_count");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CPointCloudColoured::PLY_export_get_vertex_count();
	}
	size_t PLY_export_get_face_count() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloudColoured *>(this), "PLY_export_get_face_count");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CPointCloudColoured::PLY_export_get_face_count();
	}
	void PLY_export_get_vertex(size_t a0, struct mrpt::math::TPoint3D_<float> & a1, bool & a2, struct mrpt::img::TColorf & a3) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloudColoured *>(this), "PLY_export_get_vertex");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointCloudColoured::PLY_export_get_vertex(a0, a1, a2, a3);
	}
	void renderUpdateBuffers() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloudColoured *>(this), "renderUpdateBuffers");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloudColoured *>(this), "freeOpenGLResources");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloudColoured *>(this), "setColorA_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloudColoured *>(this), "setColor_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloudColoured *>(this), "cullElegible");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloudColoured *>(this), "isCompositeObject");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloudColoured *>(this), "traceRay");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloudColoured *>(this), "getLocalRepresentativePoint");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CPointCloudColoured *>(this), "initializeTextures");
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

void bind_mrpt_opengl_CPointCloudColoured(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::opengl::CPointCloudColoured file:mrpt/opengl/CPointCloudColoured.h line:34
		pybind11::class_<mrpt::opengl::CPointCloudColoured, std::shared_ptr<mrpt::opengl::CPointCloudColoured>, PyCallBack_mrpt_opengl_CPointCloudColoured, mrpt::opengl::CRenderizableShaderPoints, mrpt::opengl::PLY_Importer, mrpt::opengl::PLY_Exporter> cl(M("mrpt::opengl"), "CPointCloudColoured", "A cloud of points, each one with an individual colour (R,G,B). The alpha\n component is shared by all the points and is stored in the base member\n m_color_A.\n\n To load from a points-map, CPointCloudColoured::loadFromPointsMap().\n\n This class uses smart optimizations while rendering to efficiently draw\n clouds of millions of points, using octrees.\n\n ![mrpt::opengl::CPointCloudColoured](preview_CPointCloudColoured.png)\n\n \n opengl::Scene, opengl::CPointCloud\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::CPointCloudColoured(); }, [](){ return new PyCallBack_mrpt_opengl_CPointCloudColoured(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_opengl_CPointCloudColoured const &o){ return new PyCallBack_mrpt_opengl_CPointCloudColoured(o); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::CPointCloudColoured const &o){ return new mrpt::opengl::CPointCloudColoured(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CPointCloudColoured::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CPointCloudColoured::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CPointCloudColoured::*)() const) &mrpt::opengl::CPointCloudColoured::GetRuntimeClass, "C++: mrpt::opengl::CPointCloudColoured::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::opengl::CPointCloudColoured::*)() const) &mrpt::opengl::CPointCloudColoured::clone, "C++: mrpt::opengl::CPointCloudColoured::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::opengl::CPointCloudColoured::CreateObject, "C++: mrpt::opengl::CPointCloudColoured::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("onUpdateBuffers_Points", (void (mrpt::opengl::CPointCloudColoured::*)()) &mrpt::opengl::CPointCloudColoured::onUpdateBuffers_Points, "C++: mrpt::opengl::CPointCloudColoured::onUpdateBuffers_Points() --> void");
		cl.def("markAllPointsAsNew", (void (mrpt::opengl::CPointCloudColoured::*)()) &mrpt::opengl::CPointCloudColoured::markAllPointsAsNew, "C++: mrpt::opengl::CPointCloudColoured::markAllPointsAsNew() --> void");
		cl.def("internalBoundingBoxLocal", (struct mrpt::math::TBoundingBox_<float> (mrpt::opengl::CPointCloudColoured::*)() const) &mrpt::opengl::CPointCloudColoured::internalBoundingBoxLocal, "C++: mrpt::opengl::CPointCloudColoured::internalBoundingBoxLocal() const --> struct mrpt::math::TBoundingBox_<float>");
		cl.def("push_back", [](mrpt::opengl::CPointCloudColoured &o, float const & a0, float const & a1, float const & a2, float const & a3, float const & a4, float const & a5) -> void { return o.push_back(a0, a1, a2, a3, a4, a5); }, "", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("R"), pybind11::arg("G"), pybind11::arg("B"));
		cl.def("push_back", (void (mrpt::opengl::CPointCloudColoured::*)(float, float, float, float, float, float, float)) &mrpt::opengl::CPointCloudColoured::push_back, "Inserts a new point into the point cloud. \n\nC++: mrpt::opengl::CPointCloudColoured::push_back(float, float, float, float, float, float, float) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("R"), pybind11::arg("G"), pybind11::arg("B"), pybind11::arg("A"));
		cl.def("insertPoint", (void (mrpt::opengl::CPointCloudColoured::*)(const struct mrpt::math::TPointXYZfRGBAu8 &)) &mrpt::opengl::CPointCloudColoured::insertPoint, "inserts a new point \n\nC++: mrpt::opengl::CPointCloudColoured::insertPoint(const struct mrpt::math::TPointXYZfRGBAu8 &) --> void", pybind11::arg("p"));
		cl.def("resize", (void (mrpt::opengl::CPointCloudColoured::*)(size_t)) &mrpt::opengl::CPointCloudColoured::resize, "Set the number of points, with undefined contents \n\nC++: mrpt::opengl::CPointCloudColoured::resize(size_t) --> void", pybind11::arg("N"));
		cl.def("reserve", (void (mrpt::opengl::CPointCloudColoured::*)(size_t)) &mrpt::opengl::CPointCloudColoured::reserve, "Like STL std::vector's reserve \n\nC++: mrpt::opengl::CPointCloudColoured::reserve(size_t) --> void", pybind11::arg("N"));
		cl.def("getPoint3Df", (const struct mrpt::math::TPoint3D_<float> & (mrpt::opengl::CPointCloudColoured::*)(size_t) const) &mrpt::opengl::CPointCloudColoured::getPoint3Df, "NOTE: This method is intentionally not protected by the shared_mutex,\n since it's called in the inner loops of the octree, which acquires the\n lock once.\n\nC++: mrpt::opengl::CPointCloudColoured::getPoint3Df(size_t) const --> const struct mrpt::math::TPoint3D_<float> &", pybind11::return_value_policy::automatic, pybind11::arg("i"));
		cl.def("setPoint", (void (mrpt::opengl::CPointCloudColoured::*)(size_t, const struct mrpt::math::TPointXYZfRGBAu8 &)) &mrpt::opengl::CPointCloudColoured::setPoint, "Write an individual point (checks for \"i\" in the valid range only in\n Debug). \n\nC++: mrpt::opengl::CPointCloudColoured::setPoint(size_t, const struct mrpt::math::TPointXYZfRGBAu8 &) --> void", pybind11::arg("i"), pybind11::arg("p"));
		cl.def("setPoint_fast", (void (mrpt::opengl::CPointCloudColoured::*)(size_t, const struct mrpt::math::TPointXYZfRGBAu8 &)) &mrpt::opengl::CPointCloudColoured::setPoint_fast, "Like  but does not check for index out of bounds \n\nC++: mrpt::opengl::CPointCloudColoured::setPoint_fast(size_t, const struct mrpt::math::TPointXYZfRGBAu8 &) --> void", pybind11::arg("i"), pybind11::arg("p"));
		cl.def("setPoint_fast", (void (mrpt::opengl::CPointCloudColoured::*)(size_t, const float, const float, const float)) &mrpt::opengl::CPointCloudColoured::setPoint_fast, "Like  but does not check for index out of bounds \n\nC++: mrpt::opengl::CPointCloudColoured::setPoint_fast(size_t, const float, const float, const float) --> void", pybind11::arg("i"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("setPointColor_fast", [](mrpt::opengl::CPointCloudColoured &o, size_t const & a0, float const & a1, float const & a2, float const & a3) -> void { return o.setPointColor_fast(a0, a1, a2, a3); }, "", pybind11::arg("index"), pybind11::arg("R"), pybind11::arg("G"), pybind11::arg("B"));
		cl.def("setPointColor_fast", (void (mrpt::opengl::CPointCloudColoured::*)(size_t, float, float, float, float)) &mrpt::opengl::CPointCloudColoured::setPointColor_fast, "Like  but without checking for out-of-index erors \n\nC++: mrpt::opengl::CPointCloudColoured::setPointColor_fast(size_t, float, float, float, float) --> void", pybind11::arg("index"), pybind11::arg("R"), pybind11::arg("G"), pybind11::arg("B"), pybind11::arg("A"));
		cl.def("setPointColor_u8_fast", [](mrpt::opengl::CPointCloudColoured &o, size_t const & a0, uint8_t const & a1, uint8_t const & a2, uint8_t const & a3) -> void { return o.setPointColor_u8_fast(a0, a1, a2, a3); }, "", pybind11::arg("index"), pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"));
		cl.def("setPointColor_u8_fast", (void (mrpt::opengl::CPointCloudColoured::*)(size_t, uint8_t, uint8_t, uint8_t, uint8_t)) &mrpt::opengl::CPointCloudColoured::setPointColor_u8_fast, "C++: mrpt::opengl::CPointCloudColoured::setPointColor_u8_fast(size_t, uint8_t, uint8_t, uint8_t, uint8_t) --> void", pybind11::arg("index"), pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"), pybind11::arg("a"));
		cl.def("getPointColor_fast", (void (mrpt::opengl::CPointCloudColoured::*)(size_t, float &, float &, float &) const) &mrpt::opengl::CPointCloudColoured::getPointColor_fast, "Like  but without checking for out-of-index erors \n\nC++: mrpt::opengl::CPointCloudColoured::getPointColor_fast(size_t, float &, float &, float &) const --> void", pybind11::arg("index"), pybind11::arg("R"), pybind11::arg("G"), pybind11::arg("B"));
		cl.def("getPointColor_fast", (void (mrpt::opengl::CPointCloudColoured::*)(size_t, unsigned char &, unsigned char &, unsigned char &) const) &mrpt::opengl::CPointCloudColoured::getPointColor_fast, "C++: mrpt::opengl::CPointCloudColoured::getPointColor_fast(size_t, unsigned char &, unsigned char &, unsigned char &) const --> void", pybind11::arg("index"), pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"));
		cl.def("getPointColor", (struct mrpt::img::TColor (mrpt::opengl::CPointCloudColoured::*)(size_t) const) &mrpt::opengl::CPointCloudColoured::getPointColor, "C++: mrpt::opengl::CPointCloudColoured::getPointColor(size_t) const --> struct mrpt::img::TColor", pybind11::arg("index"));
		cl.def("size", (size_t (mrpt::opengl::CPointCloudColoured::*)() const) &mrpt::opengl::CPointCloudColoured::size, "Return the number of points \n\nC++: mrpt::opengl::CPointCloudColoured::size() const --> size_t");
		cl.def("size_unprotected", (size_t (mrpt::opengl::CPointCloudColoured::*)() const) &mrpt::opengl::CPointCloudColoured::size_unprotected, "Like size(), but without locking the data mutex (internal usage)\n\nC++: mrpt::opengl::CPointCloudColoured::size_unprotected() const --> size_t");
		cl.def("empty", (bool (mrpt::opengl::CPointCloudColoured::*)() const) &mrpt::opengl::CPointCloudColoured::empty, "C++: mrpt::opengl::CPointCloudColoured::empty() const --> bool");
		cl.def("clear", (void (mrpt::opengl::CPointCloudColoured::*)()) &mrpt::opengl::CPointCloudColoured::clear, "Erase all the points \n\nC++: mrpt::opengl::CPointCloudColoured::clear() --> void");
		cl.def("getActuallyRendered", (size_t (mrpt::opengl::CPointCloudColoured::*)() const) &mrpt::opengl::CPointCloudColoured::getActuallyRendered, "Get the number of elements actually rendered in the last render event.\n\nC++: mrpt::opengl::CPointCloudColoured::getActuallyRendered() const --> size_t");
		cl.def("recolorizeByCoordinate", [](mrpt::opengl::CPointCloudColoured &o, const float & a0, const float & a1) -> void { return o.recolorizeByCoordinate(a0, a1); }, "", pybind11::arg("coord_min"), pybind11::arg("coord_max"));
		cl.def("recolorizeByCoordinate", [](mrpt::opengl::CPointCloudColoured &o, const float & a0, const float & a1, const int & a2) -> void { return o.recolorizeByCoordinate(a0, a1, a2); }, "", pybind11::arg("coord_min"), pybind11::arg("coord_max"), pybind11::arg("coord_index"));
		cl.def("recolorizeByCoordinate", (void (mrpt::opengl::CPointCloudColoured::*)(const float, const float, const int, const enum mrpt::img::TColormap)) &mrpt::opengl::CPointCloudColoured::recolorizeByCoordinate, "Regenerates the color of each point according the one coordinate\n (coord_index:0,1,2 for X,Y,Z) and the given color map. \n\nC++: mrpt::opengl::CPointCloudColoured::recolorizeByCoordinate(const float, const float, const int, const enum mrpt::img::TColormap) --> void", pybind11::arg("coord_min"), pybind11::arg("coord_max"), pybind11::arg("coord_index"), pybind11::arg("color_map"));
		cl.def("toYAMLMap", (void (mrpt::opengl::CPointCloudColoured::*)(class mrpt::containers::yaml &) const) &mrpt::opengl::CPointCloudColoured::toYAMLMap, "C++: mrpt::opengl::CPointCloudColoured::toYAMLMap(class mrpt::containers::yaml &) const --> void", pybind11::arg("propertiesMap"));
	}
	{ // mrpt::opengl::PointCloudAdapter file:mrpt/opengl/CPointCloudColoured.h line:264
		pybind11::class_<mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>, std::shared_ptr<mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>>> cl(M("mrpt::opengl"), "PointCloudAdapter_mrpt_opengl_CPointCloudColoured_t", "Specialization\n mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>  \n\n\n mrpt_adapters_grp");
		cl.def( pybind11::init<const class mrpt::opengl::CPointCloudColoured &>(), pybind11::arg("obj") );

		cl.def( pybind11::init( [](mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured> const &o){ return new mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>(o); } ) );
		cl.def("size", (size_t (mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>::*)() const) &mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>::size, "Get number of points \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>::size() const --> size_t");
		cl.def("resize", (void (mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>::*)(size_t)) &mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>::resize, "Set number of points (to uninitialized values) \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>::resize(size_t) --> void", pybind11::arg("N"));
		cl.def("setDimensions", (void (mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>::*)(size_t, size_t)) &mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>::setDimensions, "Does nothing as of now \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>::setDimensions(size_t, size_t) --> void", pybind11::arg("height"), pybind11::arg("width"));
		cl.def("setPointXYZ", (void (mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>::*)(size_t, const float, const float, const float)) &mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>::setPointXYZ, "Set XYZ coordinates of i'th point \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>::setPointXYZ(size_t, const float, const float, const float) --> void", pybind11::arg("idx"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("setInvalidPoint", (void (mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>::*)(size_t)) &mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>::setInvalidPoint, "C++: mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>::setInvalidPoint(size_t) --> void", pybind11::arg("idx"));
		cl.def("setPointXYZ_RGBAf", (void (mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>::*)(size_t, const float, const float, const float, const float, const float, const float, const float)) &mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>::setPointXYZ_RGBAf, "Set XYZ_RGBf coordinates of i'th point \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>::setPointXYZ_RGBAf(size_t, const float, const float, const float, const float, const float, const float, const float) --> void", pybind11::arg("idx"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("Rf"), pybind11::arg("Gf"), pybind11::arg("Bf"), pybind11::arg("Af"));
		cl.def("setPointXYZ_RGBu8", [](mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured> &o, size_t const & a0, const float & a1, const float & a2, const float & a3, const unsigned char & a4, const unsigned char & a5, const unsigned char & a6) -> void { return o.setPointXYZ_RGBu8(a0, a1, a2, a3, a4, a5, a6); }, "", pybind11::arg("idx"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"));
		cl.def("setPointXYZ_RGBu8", (void (mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>::*)(size_t, const float, const float, const float, const unsigned char, const unsigned char, const unsigned char, const unsigned char)) &mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>::setPointXYZ_RGBu8, "Set XYZ_RGBu8 coordinates of i'th point \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>::setPointXYZ_RGBu8(size_t, const float, const float, const float, const unsigned char, const unsigned char, const unsigned char, const unsigned char) --> void", pybind11::arg("idx"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"), pybind11::arg("a"));
		cl.def("getPointRGBf", (void (mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>::*)(size_t, float &, float &, float &) const) &mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>::getPointRGBf, "Get RGBf color of i'th point \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>::getPointRGBf(size_t, float &, float &, float &) const --> void", pybind11::arg("idx"), pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"));
		cl.def("setPointRGBf", (void (mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>::*)(size_t, const float, const float, const float)) &mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>::setPointRGBf, "Set XYZ_RGBf coordinates of i'th point \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>::setPointRGBf(size_t, const float, const float, const float) --> void", pybind11::arg("idx"), pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"));
		cl.def("getPointRGBu8", (void (mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>::*)(size_t, unsigned char &, unsigned char &, unsigned char &) const) &mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>::getPointRGBu8, "Get RGBu8 color of i'th point \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>::getPointRGBu8(size_t, unsigned char &, unsigned char &, unsigned char &) const --> void", pybind11::arg("idx"), pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"));
		cl.def("setPointRGBu8", (void (mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>::*)(size_t, const unsigned char, const unsigned char, const unsigned char)) &mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>::setPointRGBu8, "Set RGBu8 coordinates of i'th point \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloudColoured>::setPointRGBu8(size_t, const unsigned char, const unsigned char, const unsigned char) --> void", pybind11::arg("idx"), pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"));
	}
}
