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
#include <mrpt/opengl/COctoMapVoxels.h>
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

// mrpt::opengl::COctoMapVoxels file:mrpt/opengl/COctoMapVoxels.h line:64
struct PyCallBack_mrpt_opengl_COctoMapVoxels : public mrpt::opengl::COctoMapVoxels {
	using mrpt::opengl::COctoMapVoxels::COctoMapVoxels;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::COctoMapVoxels *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return COctoMapVoxels::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::COctoMapVoxels *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return COctoMapVoxels::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::COctoMapVoxels *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return COctoMapVoxels::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::COctoMapVoxels *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COctoMapVoxels::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::COctoMapVoxels *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COctoMapVoxels::serializeFrom(a0, a1);
	}
	void renderUpdateBuffers() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::COctoMapVoxels *>(this), "renderUpdateBuffers");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COctoMapVoxels::renderUpdateBuffers();
	}
	void onUpdateBuffers_Points() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::COctoMapVoxels *>(this), "onUpdateBuffers_Points");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COctoMapVoxels::onUpdateBuffers_Points();
	}
	void onUpdateBuffers_Wireframe() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::COctoMapVoxels *>(this), "onUpdateBuffers_Wireframe");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COctoMapVoxels::onUpdateBuffers_Wireframe();
	}
	void onUpdateBuffers_Triangles() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::COctoMapVoxels *>(this), "onUpdateBuffers_Triangles");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COctoMapVoxels::onUpdateBuffers_Triangles();
	}
	void freeOpenGLResources() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::COctoMapVoxels *>(this), "freeOpenGLResources");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COctoMapVoxels::freeOpenGLResources();
	}
	struct mrpt::math::TBoundingBox_<float> internalBoundingBoxLocal() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::COctoMapVoxels *>(this), "internalBoundingBoxLocal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TBoundingBox_<float>>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TBoundingBox_<float>> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TBoundingBox_<float>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TBoundingBox_<float>>(std::move(o));
		}
		return COctoMapVoxels::internalBoundingBoxLocal();
	}
	class mrpt::opengl::CRenderizable & setColorA_u8(const unsigned char a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::COctoMapVoxels *>(this), "setColorA_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::COctoMapVoxels *>(this), "setColor_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::COctoMapVoxels *>(this), "cullElegible");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::COctoMapVoxels *>(this), "toYAMLMap");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::COctoMapVoxels *>(this), "isCompositeObject");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::COctoMapVoxels *>(this), "traceRay");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::COctoMapVoxels *>(this), "getLocalRepresentativePoint");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::COctoMapVoxels *>(this), "initializeTextures");
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

void bind_mrpt_opengl_COctoMapVoxels(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::opengl::predefined_voxel_sets_t file:mrpt/opengl/COctoMapVoxels.h line:19
	pybind11::enum_<mrpt::opengl::predefined_voxel_sets_t>(M("mrpt::opengl"), "predefined_voxel_sets_t", pybind11::arithmetic(), "")
		.value("VOXEL_SET_OCCUPIED", mrpt::opengl::VOXEL_SET_OCCUPIED)
		.value("VOXEL_SET_FREESPACE", mrpt::opengl::VOXEL_SET_FREESPACE)
		.export_values();

;

	{ // mrpt::opengl::COctoMapVoxels file:mrpt/opengl/COctoMapVoxels.h line:64
		pybind11::class_<mrpt::opengl::COctoMapVoxels, std::shared_ptr<mrpt::opengl::COctoMapVoxels>, PyCallBack_mrpt_opengl_COctoMapVoxels, mrpt::opengl::CRenderizableShaderTriangles, mrpt::opengl::CRenderizableShaderWireFrame, mrpt::opengl::CRenderizableShaderPoints> cl(M("mrpt::opengl"), "COctoMapVoxels", "A flexible renderer of voxels, typically from a 3D octo map (see\nmrpt::maps::COctoMap).\n  This class is sort of equivalent to octovis::OcTreeDrawer from the octomap\npackage, but\n  relying on MRPT's CRenderizable so there's no need to manually\ncache the rendering of OpenGL primitives.\n\n  Normally users call mrpt::maps::COctoMap::getAs3DObject() to obtain a\ngeneric mrpt::opengl::CSetOfObjects which insides holds an instance of\nCOctoMapVoxels.\n  You can also alternativelly call COctoMapVoxels::setFromOctoMap(), so you\ncan tune the display parameters, colors, etc.\n  As with any other mrpt::opengl class, all object coordinates refer to some\nframe of reference which is relative to the object parent and can be changed\nwith mrpt::opengl::CRenderizable::setPose()\n\n  This class draws these separate elements to represent an OctoMap:\n		- A grid representation of all cubes, as simple lines (occupied/free,\nleafs/nodes,... whatever). See:\n			- showGridLines()\n			- setGridLinesColor()\n			- setGridLinesWidth()\n			- push_back_GridCube()\n		- A number of voxel collections, drawn as cubes each having a\ndifferent color (e.g. depending on the color scheme in the original\nmrpt::maps::COctoMap object).\n       The meanning of each collection is user-defined, but you can use the\nconstants VOXEL_SET_OCCUPIED, VOXEL_SET_FREESPACE for predefined meanings.\n			- showVoxels()\n			- push_back_Voxel()\n\n Several coloring schemes can be selected with setVisualizationMode(). See\nCOctoMapVoxels::visualization_mode_t\n\n ![mrpt::opengl::COctoMapVoxels](preview_COctoMapVoxels.png)\n\n \n opengl::Scene\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::COctoMapVoxels(); }, [](){ return new PyCallBack_mrpt_opengl_COctoMapVoxels(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_opengl_COctoMapVoxels const &o){ return new PyCallBack_mrpt_opengl_COctoMapVoxels(o); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::COctoMapVoxels const &o){ return new mrpt::opengl::COctoMapVoxels(o); } ) );

		pybind11::enum_<mrpt::opengl::COctoMapVoxels::visualization_mode_t>(cl, "visualization_mode_t", pybind11::arithmetic(), "The different coloring schemes, which modulate the generic\n mrpt::opengl::CRenderizable object color. Set with setVisualizationMode()")
			.value("COLOR_FROM_HEIGHT", mrpt::opengl::COctoMapVoxels::COLOR_FROM_HEIGHT)
			.value("COLOR_FROM_OCCUPANCY", mrpt::opengl::COctoMapVoxels::COLOR_FROM_OCCUPANCY)
			.value("TRANSPARENCY_FROM_OCCUPANCY", mrpt::opengl::COctoMapVoxels::TRANSPARENCY_FROM_OCCUPANCY)
			.value("TRANS_AND_COLOR_FROM_OCCUPANCY", mrpt::opengl::COctoMapVoxels::TRANS_AND_COLOR_FROM_OCCUPANCY)
			.value("MIXED", mrpt::opengl::COctoMapVoxels::MIXED)
			.value("FIXED", mrpt::opengl::COctoMapVoxels::FIXED)
			.value("COLOR_FROM_RGB_DATA", mrpt::opengl::COctoMapVoxels::COLOR_FROM_RGB_DATA)
			.export_values();

		cl.def_static("Create", (class std::shared_ptr<class mrpt::opengl::COctoMapVoxels> (*)()) &mrpt::opengl::COctoMapVoxels::Create, "C++: mrpt::opengl::COctoMapVoxels::Create() --> class std::shared_ptr<class mrpt::opengl::COctoMapVoxels>");
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::COctoMapVoxels::GetRuntimeClassIdStatic, "C++: mrpt::opengl::COctoMapVoxels::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::COctoMapVoxels::*)() const) &mrpt::opengl::COctoMapVoxels::GetRuntimeClass, "C++: mrpt::opengl::COctoMapVoxels::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::opengl::COctoMapVoxels::*)() const) &mrpt::opengl::COctoMapVoxels::clone, "C++: mrpt::opengl::COctoMapVoxels::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::opengl::COctoMapVoxels::CreateObject, "C++: mrpt::opengl::COctoMapVoxels::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("colorMap", (enum mrpt::img::TColormap (mrpt::opengl::COctoMapVoxels::*)() const) &mrpt::opengl::COctoMapVoxels::colorMap, "C++: mrpt::opengl::COctoMapVoxels::colorMap() const --> enum mrpt::img::TColormap");
		cl.def("colorMap", (void (mrpt::opengl::COctoMapVoxels::*)(const enum mrpt::img::TColormap &)) &mrpt::opengl::COctoMapVoxels::colorMap, "Changing the colormap has no effect until a source object (e.g.\n mrpt::maps::CVoxelMap) reads this property while generating the voxels\n visualization.\n\nC++: mrpt::opengl::COctoMapVoxels::colorMap(const enum mrpt::img::TColormap &) --> void", pybind11::arg("cm"));
		cl.def("renderUpdateBuffers", (void (mrpt::opengl::COctoMapVoxels::*)() const) &mrpt::opengl::COctoMapVoxels::renderUpdateBuffers, "C++: mrpt::opengl::COctoMapVoxels::renderUpdateBuffers() const --> void");
		cl.def("onUpdateBuffers_Points", (void (mrpt::opengl::COctoMapVoxels::*)()) &mrpt::opengl::COctoMapVoxels::onUpdateBuffers_Points, "C++: mrpt::opengl::COctoMapVoxels::onUpdateBuffers_Points() --> void");
		cl.def("onUpdateBuffers_Wireframe", (void (mrpt::opengl::COctoMapVoxels::*)()) &mrpt::opengl::COctoMapVoxels::onUpdateBuffers_Wireframe, "C++: mrpt::opengl::COctoMapVoxels::onUpdateBuffers_Wireframe() --> void");
		cl.def("onUpdateBuffers_Triangles", (void (mrpt::opengl::COctoMapVoxels::*)()) &mrpt::opengl::COctoMapVoxels::onUpdateBuffers_Triangles, "C++: mrpt::opengl::COctoMapVoxels::onUpdateBuffers_Triangles() --> void");
		cl.def("freeOpenGLResources", (void (mrpt::opengl::COctoMapVoxels::*)()) &mrpt::opengl::COctoMapVoxels::freeOpenGLResources, "C++: mrpt::opengl::COctoMapVoxels::freeOpenGLResources() --> void");
		cl.def("clear", (void (mrpt::opengl::COctoMapVoxels::*)()) &mrpt::opengl::COctoMapVoxels::clear, "Clears everything \n\nC++: mrpt::opengl::COctoMapVoxels::clear() --> void");
		cl.def("setVisualizationMode", (void (mrpt::opengl::COctoMapVoxels::*)(enum mrpt::opengl::COctoMapVoxels::visualization_mode_t)) &mrpt::opengl::COctoMapVoxels::setVisualizationMode, "Select the visualization mode. To have any effect, this method has to be\n called before loading the octomap. \n\nC++: mrpt::opengl::COctoMapVoxels::setVisualizationMode(enum mrpt::opengl::COctoMapVoxels::visualization_mode_t) --> void", pybind11::arg("mode"));
		cl.def("getVisualizationMode", (enum mrpt::opengl::COctoMapVoxels::visualization_mode_t (mrpt::opengl::COctoMapVoxels::*)() const) &mrpt::opengl::COctoMapVoxels::getVisualizationMode, "C++: mrpt::opengl::COctoMapVoxels::getVisualizationMode() const --> enum mrpt::opengl::COctoMapVoxels::visualization_mode_t");
		cl.def("enableLights", (void (mrpt::opengl::COctoMapVoxels::*)(bool)) &mrpt::opengl::COctoMapVoxels::enableLights, "Can be used to enable/disable the effects of lighting in this object \n\nC++: mrpt::opengl::COctoMapVoxels::enableLights(bool) --> void", pybind11::arg("enable"));
		cl.def("areLightsEnabled", (bool (mrpt::opengl::COctoMapVoxels::*)() const) &mrpt::opengl::COctoMapVoxels::areLightsEnabled, "C++: mrpt::opengl::COctoMapVoxels::areLightsEnabled() const --> bool");
		cl.def("enableCubeTransparency", (void (mrpt::opengl::COctoMapVoxels::*)(bool)) &mrpt::opengl::COctoMapVoxels::enableCubeTransparency, "By default, the alpha (transparency) component of voxel cubes is taken\n into account, but transparency can be disabled with this method. \n\nC++: mrpt::opengl::COctoMapVoxels::enableCubeTransparency(bool) --> void", pybind11::arg("enable"));
		cl.def("isCubeTransparencyEnabled", (bool (mrpt::opengl::COctoMapVoxels::*)() const) &mrpt::opengl::COctoMapVoxels::isCubeTransparencyEnabled, "C++: mrpt::opengl::COctoMapVoxels::isCubeTransparencyEnabled() const --> bool");
		cl.def("showGridLines", (void (mrpt::opengl::COctoMapVoxels::*)(bool)) &mrpt::opengl::COctoMapVoxels::showGridLines, "Shows/hides the grid lines \n\nC++: mrpt::opengl::COctoMapVoxels::showGridLines(bool) --> void", pybind11::arg("show"));
		cl.def("areGridLinesVisible", (bool (mrpt::opengl::COctoMapVoxels::*)() const) &mrpt::opengl::COctoMapVoxels::areGridLinesVisible, "C++: mrpt::opengl::COctoMapVoxels::areGridLinesVisible() const --> bool");
		cl.def("showVoxels", (void (mrpt::opengl::COctoMapVoxels::*)(unsigned int, bool)) &mrpt::opengl::COctoMapVoxels::showVoxels, "Shows/hides the voxels (voxel_set is a 0-based index for the set of\n voxels to modify, e.g. VOXEL_SET_OCCUPIED, VOXEL_SET_FREESPACE) \n\nC++: mrpt::opengl::COctoMapVoxels::showVoxels(unsigned int, bool) --> void", pybind11::arg("voxel_set"), pybind11::arg("show"));
		cl.def("areVoxelsVisible", (bool (mrpt::opengl::COctoMapVoxels::*)(unsigned int) const) &mrpt::opengl::COctoMapVoxels::areVoxelsVisible, "C++: mrpt::opengl::COctoMapVoxels::areVoxelsVisible(unsigned int) const --> bool", pybind11::arg("voxel_set"));
		cl.def("showVoxelsAsPoints", (void (mrpt::opengl::COctoMapVoxels::*)(const bool)) &mrpt::opengl::COctoMapVoxels::showVoxelsAsPoints, "For quick renders: render voxels as points instead of cubes. \n\n setVoxelAsPointsSize \n\nC++: mrpt::opengl::COctoMapVoxels::showVoxelsAsPoints(const bool) --> void", pybind11::arg("enable"));
		cl.def("areVoxelsShownAsPoints", (bool (mrpt::opengl::COctoMapVoxels::*)() const) &mrpt::opengl::COctoMapVoxels::areVoxelsShownAsPoints, "C++: mrpt::opengl::COctoMapVoxels::areVoxelsShownAsPoints() const --> bool");
		cl.def("setVoxelAsPointsSize", (void (mrpt::opengl::COctoMapVoxels::*)(float)) &mrpt::opengl::COctoMapVoxels::setVoxelAsPointsSize, "Only used when showVoxelsAsPoints() is enabled.  \n\nC++: mrpt::opengl::COctoMapVoxels::setVoxelAsPointsSize(float) --> void", pybind11::arg("pointSize"));
		cl.def("getVoxelAsPointsSize", (float (mrpt::opengl::COctoMapVoxels::*)() const) &mrpt::opengl::COctoMapVoxels::getVoxelAsPointsSize, "C++: mrpt::opengl::COctoMapVoxels::getVoxelAsPointsSize() const --> float");
		cl.def("setGridLinesWidth", (void (mrpt::opengl::COctoMapVoxels::*)(float)) &mrpt::opengl::COctoMapVoxels::setGridLinesWidth, "Sets the width of grid lines \n\nC++: mrpt::opengl::COctoMapVoxels::setGridLinesWidth(float) --> void", pybind11::arg("w"));
		cl.def("getGridLinesWidth", (float (mrpt::opengl::COctoMapVoxels::*)() const) &mrpt::opengl::COctoMapVoxels::getGridLinesWidth, "Gets the width of grid lines \n\nC++: mrpt::opengl::COctoMapVoxels::getGridLinesWidth() const --> float");
		cl.def("setGridLinesColor", (void (mrpt::opengl::COctoMapVoxels::*)(const struct mrpt::img::TColor &)) &mrpt::opengl::COctoMapVoxels::setGridLinesColor, "C++: mrpt::opengl::COctoMapVoxels::setGridLinesColor(const struct mrpt::img::TColor &) --> void", pybind11::arg("color"));
		cl.def("getGridLinesColor", (const struct mrpt::img::TColor & (mrpt::opengl::COctoMapVoxels::*)() const) &mrpt::opengl::COctoMapVoxels::getGridLinesColor, "C++: mrpt::opengl::COctoMapVoxels::getGridLinesColor() const --> const struct mrpt::img::TColor &", pybind11::return_value_policy::automatic);
		cl.def("getGridCubeCount", (size_t (mrpt::opengl::COctoMapVoxels::*)() const) &mrpt::opengl::COctoMapVoxels::getGridCubeCount, "Returns the total count of grid cubes. \n\nC++: mrpt::opengl::COctoMapVoxels::getGridCubeCount() const --> size_t");
		cl.def("getVoxelSetCount", (size_t (mrpt::opengl::COctoMapVoxels::*)() const) &mrpt::opengl::COctoMapVoxels::getVoxelSetCount, "Returns the number of voxel sets. \n\nC++: mrpt::opengl::COctoMapVoxels::getVoxelSetCount() const --> size_t");
		cl.def("getVoxelCount", (size_t (mrpt::opengl::COctoMapVoxels::*)(size_t) const) &mrpt::opengl::COctoMapVoxels::getVoxelCount, "Returns the total count of voxels in one voxel set. \n\nC++: mrpt::opengl::COctoMapVoxels::getVoxelCount(size_t) const --> size_t", pybind11::arg("set_index"));
		cl.def("setBoundingBox", (void (mrpt::opengl::COctoMapVoxels::*)(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &)) &mrpt::opengl::COctoMapVoxels::setBoundingBox, "Manually changes the bounding box (normally the user doesn't need to\n call this) \n\nC++: mrpt::opengl::COctoMapVoxels::setBoundingBox(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &) --> void", pybind11::arg("bb_min"), pybind11::arg("bb_max"));
		cl.def("resizeGridCubes", (void (mrpt::opengl::COctoMapVoxels::*)(size_t)) &mrpt::opengl::COctoMapVoxels::resizeGridCubes, "C++: mrpt::opengl::COctoMapVoxels::resizeGridCubes(size_t) --> void", pybind11::arg("nCubes"));
		cl.def("resizeVoxelSets", (void (mrpt::opengl::COctoMapVoxels::*)(size_t)) &mrpt::opengl::COctoMapVoxels::resizeVoxelSets, "C++: mrpt::opengl::COctoMapVoxels::resizeVoxelSets(size_t) --> void", pybind11::arg("nVoxelSets"));
		cl.def("resizeVoxels", (void (mrpt::opengl::COctoMapVoxels::*)(size_t, size_t)) &mrpt::opengl::COctoMapVoxels::resizeVoxels, "C++: mrpt::opengl::COctoMapVoxels::resizeVoxels(size_t, size_t) --> void", pybind11::arg("set_index"), pybind11::arg("nVoxels"));
		cl.def("reserveGridCubes", (void (mrpt::opengl::COctoMapVoxels::*)(size_t)) &mrpt::opengl::COctoMapVoxels::reserveGridCubes, "C++: mrpt::opengl::COctoMapVoxels::reserveGridCubes(size_t) --> void", pybind11::arg("nCubes"));
		cl.def("reserveVoxels", (void (mrpt::opengl::COctoMapVoxels::*)(size_t, size_t)) &mrpt::opengl::COctoMapVoxels::reserveVoxels, "C++: mrpt::opengl::COctoMapVoxels::reserveVoxels(size_t, size_t) --> void", pybind11::arg("set_index"), pybind11::arg("nVoxels"));
		cl.def("getGridCubeRef", (struct mrpt::opengl::COctoMapVoxels::TGridCube & (mrpt::opengl::COctoMapVoxels::*)(size_t)) &mrpt::opengl::COctoMapVoxels::getGridCubeRef, "C++: mrpt::opengl::COctoMapVoxels::getGridCubeRef(size_t) --> struct mrpt::opengl::COctoMapVoxels::TGridCube &", pybind11::return_value_policy::automatic, pybind11::arg("idx"));
		cl.def("getGridCube", (const struct mrpt::opengl::COctoMapVoxels::TGridCube & (mrpt::opengl::COctoMapVoxels::*)(size_t) const) &mrpt::opengl::COctoMapVoxels::getGridCube, "C++: mrpt::opengl::COctoMapVoxels::getGridCube(size_t) const --> const struct mrpt::opengl::COctoMapVoxels::TGridCube &", pybind11::return_value_policy::automatic, pybind11::arg("idx"));
		cl.def("getVoxelRef", (struct mrpt::opengl::COctoMapVoxels::TVoxel & (mrpt::opengl::COctoMapVoxels::*)(size_t, size_t)) &mrpt::opengl::COctoMapVoxels::getVoxelRef, "C++: mrpt::opengl::COctoMapVoxels::getVoxelRef(size_t, size_t) --> struct mrpt::opengl::COctoMapVoxels::TVoxel &", pybind11::return_value_policy::automatic, pybind11::arg("set_index"), pybind11::arg("idx"));
		cl.def("getVoxel", (const struct mrpt::opengl::COctoMapVoxels::TVoxel & (mrpt::opengl::COctoMapVoxels::*)(size_t, size_t) const) &mrpt::opengl::COctoMapVoxels::getVoxel, "C++: mrpt::opengl::COctoMapVoxels::getVoxel(size_t, size_t) const --> const struct mrpt::opengl::COctoMapVoxels::TVoxel &", pybind11::return_value_policy::automatic, pybind11::arg("set_index"), pybind11::arg("idx"));
		cl.def("push_back_GridCube", (void (mrpt::opengl::COctoMapVoxels::*)(const struct mrpt::opengl::COctoMapVoxels::TGridCube &)) &mrpt::opengl::COctoMapVoxels::push_back_GridCube, "C++: mrpt::opengl::COctoMapVoxels::push_back_GridCube(const struct mrpt::opengl::COctoMapVoxels::TGridCube &) --> void", pybind11::arg("c"));
		cl.def("push_back_Voxel", (void (mrpt::opengl::COctoMapVoxels::*)(size_t, const struct mrpt::opengl::COctoMapVoxels::TVoxel &)) &mrpt::opengl::COctoMapVoxels::push_back_Voxel, "C++: mrpt::opengl::COctoMapVoxels::push_back_Voxel(size_t, const struct mrpt::opengl::COctoMapVoxels::TVoxel &) --> void", pybind11::arg("set_index"), pybind11::arg("v"));
		cl.def("sort_voxels_by_z", (void (mrpt::opengl::COctoMapVoxels::*)()) &mrpt::opengl::COctoMapVoxels::sort_voxels_by_z, "C++: mrpt::opengl::COctoMapVoxels::sort_voxels_by_z() --> void");
		cl.def("internalBoundingBoxLocal", (struct mrpt::math::TBoundingBox_<float> (mrpt::opengl::COctoMapVoxels::*)() const) &mrpt::opengl::COctoMapVoxels::internalBoundingBoxLocal, "C++: mrpt::opengl::COctoMapVoxels::internalBoundingBoxLocal() const --> struct mrpt::math::TBoundingBox_<float>");
		cl.def("assign", (class mrpt::opengl::COctoMapVoxels & (mrpt::opengl::COctoMapVoxels::*)(const class mrpt::opengl::COctoMapVoxels &)) &mrpt::opengl::COctoMapVoxels::operator=, "C++: mrpt::opengl::COctoMapVoxels::operator=(const class mrpt::opengl::COctoMapVoxels &) --> class mrpt::opengl::COctoMapVoxels &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::opengl::COctoMapVoxels::TVoxel file:mrpt/opengl/COctoMapVoxels.h line:97
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::opengl::COctoMapVoxels::TVoxel, std::shared_ptr<mrpt::opengl::COctoMapVoxels::TVoxel>> cl(enclosing_class, "TVoxel", "The info of each of the voxels ");
			cl.def( pybind11::init( [](){ return new mrpt::opengl::COctoMapVoxels::TVoxel(); } ) );
			cl.def( pybind11::init<const struct mrpt::math::TPoint3D_<float> &, const double, struct mrpt::img::TColor>(), pybind11::arg("coords_"), pybind11::arg("side_length_"), pybind11::arg("color_") );

			cl.def( pybind11::init( [](mrpt::opengl::COctoMapVoxels::TVoxel const &o){ return new mrpt::opengl::COctoMapVoxels::TVoxel(o); } ) );
			cl.def_readwrite("coords", &mrpt::opengl::COctoMapVoxels::TVoxel::coords);
			cl.def_readwrite("side_length", &mrpt::opengl::COctoMapVoxels::TVoxel::side_length);
			cl.def_readwrite("color", &mrpt::opengl::COctoMapVoxels::TVoxel::color);
			cl.def("assign", (struct mrpt::opengl::COctoMapVoxels::TVoxel & (mrpt::opengl::COctoMapVoxels::TVoxel::*)(const struct mrpt::opengl::COctoMapVoxels::TVoxel &)) &mrpt::opengl::COctoMapVoxels::TVoxel::operator=, "C++: mrpt::opengl::COctoMapVoxels::TVoxel::operator=(const struct mrpt::opengl::COctoMapVoxels::TVoxel &) --> struct mrpt::opengl::COctoMapVoxels::TVoxel &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::opengl::COctoMapVoxels::TGridCube file:mrpt/opengl/COctoMapVoxels.h line:112
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::opengl::COctoMapVoxels::TGridCube, std::shared_ptr<mrpt::opengl::COctoMapVoxels::TGridCube>> cl(enclosing_class, "TGridCube", "The info of each grid block ");
			cl.def( pybind11::init( [](){ return new mrpt::opengl::COctoMapVoxels::TGridCube(); } ) );
			cl.def( pybind11::init<const struct mrpt::math::TPoint3D_<float> &, const struct mrpt::math::TPoint3D_<float> &>(), pybind11::arg("min_"), pybind11::arg("max_") );

			cl.def( pybind11::init( [](mrpt::opengl::COctoMapVoxels::TGridCube const &o){ return new mrpt::opengl::COctoMapVoxels::TGridCube(o); } ) );
			cl.def_readwrite("min", &mrpt::opengl::COctoMapVoxels::TGridCube::min);
			cl.def_readwrite("max", &mrpt::opengl::COctoMapVoxels::TGridCube::max);
			cl.def("assign", (struct mrpt::opengl::COctoMapVoxels::TGridCube & (mrpt::opengl::COctoMapVoxels::TGridCube::*)(const struct mrpt::opengl::COctoMapVoxels::TGridCube &)) &mrpt::opengl::COctoMapVoxels::TGridCube::operator=, "C++: mrpt::opengl::COctoMapVoxels::TGridCube::operator=(const struct mrpt::opengl::COctoMapVoxels::TGridCube &) --> struct mrpt::opengl::COctoMapVoxels::TGridCube &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::opengl::COctoMapVoxels::TInfoPerVoxelSet file:mrpt/opengl/COctoMapVoxels.h line:124
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::opengl::COctoMapVoxels::TInfoPerVoxelSet, std::shared_ptr<mrpt::opengl::COctoMapVoxels::TInfoPerVoxelSet>> cl(enclosing_class, "TInfoPerVoxelSet", "");
			cl.def( pybind11::init( [](){ return new mrpt::opengl::COctoMapVoxels::TInfoPerVoxelSet(); } ) );
			cl.def( pybind11::init( [](mrpt::opengl::COctoMapVoxels::TInfoPerVoxelSet const &o){ return new mrpt::opengl::COctoMapVoxels::TInfoPerVoxelSet(o); } ) );
			cl.def_readwrite("visible", &mrpt::opengl::COctoMapVoxels::TInfoPerVoxelSet::visible);
			cl.def_readwrite("voxels", &mrpt::opengl::COctoMapVoxels::TInfoPerVoxelSet::voxels);
			cl.def("assign", (struct mrpt::opengl::COctoMapVoxels::TInfoPerVoxelSet & (mrpt::opengl::COctoMapVoxels::TInfoPerVoxelSet::*)(const struct mrpt::opengl::COctoMapVoxels::TInfoPerVoxelSet &)) &mrpt::opengl::COctoMapVoxels::TInfoPerVoxelSet::operator=, "C++: mrpt::opengl::COctoMapVoxels::TInfoPerVoxelSet::operator=(const struct mrpt::opengl::COctoMapVoxels::TInfoPerVoxelSet &) --> struct mrpt::opengl::COctoMapVoxels::TInfoPerVoxelSet &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
}
