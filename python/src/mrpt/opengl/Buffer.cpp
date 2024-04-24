#include <any>
#include <functional>
#include <ios>
#include <istream>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/containers/CommentPosition.h>
#include <mrpt/containers/NonCopiableData.h>
#include <mrpt/containers/YamlEmitOptions.h>
#include <mrpt/containers/yaml.h>
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
#include <mrpt/math/TSegment2D.h>
#include <mrpt/math/TSegment3D.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/opengl/Buffer.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CRenderizableShaderPoints.h>
#include <mrpt/opengl/CRenderizableShaderWireFrame.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/opengl/VertexArrayObject.h>
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
#include <shared_mutex>
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

// mrpt::opengl::CSetOfLines file:mrpt/opengl/CSetOfLines.h line:25
struct PyCallBack_mrpt_opengl_CSetOfLines : public mrpt::opengl::CSetOfLines {
	using mrpt::opengl::CSetOfLines::CSetOfLines;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfLines *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CSetOfLines::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfLines *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CSetOfLines::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfLines *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CSetOfLines::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfLines *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSetOfLines::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfLines *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSetOfLines::serializeFrom(a0, a1);
	}
	void renderUpdateBuffers() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfLines *>(this), "renderUpdateBuffers");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSetOfLines::renderUpdateBuffers();
	}
	void freeOpenGLResources() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfLines *>(this), "freeOpenGLResources");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSetOfLines::freeOpenGLResources();
	}
	void onUpdateBuffers_Wireframe() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfLines *>(this), "onUpdateBuffers_Wireframe");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSetOfLines::onUpdateBuffers_Wireframe();
	}
	void onUpdateBuffers_Points() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfLines *>(this), "onUpdateBuffers_Points");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSetOfLines::onUpdateBuffers_Points();
	}
	struct mrpt::math::TBoundingBox_<float> internalBoundingBoxLocal() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfLines *>(this), "internalBoundingBoxLocal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TBoundingBox_<float>>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TBoundingBox_<float>> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TBoundingBox_<float>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TBoundingBox_<float>>(std::move(o));
		}
		return CSetOfLines::internalBoundingBoxLocal();
	}
	class mrpt::opengl::CRenderizable & setColorA_u8(const unsigned char a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfLines *>(this), "setColorA_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfLines *>(this), "setColor_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfLines *>(this), "cullElegible");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfLines *>(this), "toYAMLMap");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfLines *>(this), "isCompositeObject");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfLines *>(this), "traceRay");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfLines *>(this), "getLocalRepresentativePoint");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfLines *>(this), "initializeTextures");
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

void bind_mrpt_opengl_Buffer(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::opengl::Buffer file:mrpt/opengl/Buffer.h line:27
		pybind11::class_<mrpt::opengl::Buffer, std::shared_ptr<mrpt::opengl::Buffer>> cl(M("mrpt::opengl"), "Buffer", "A wrapper for an OpenGL buffer object (eg Vertex Buffer Object or VBO)\n Refer to docs for glGenBuffers() and glBufferData().\n\n \n FrameBuffer\n \n\n\n \n OpenGL Buffer Objects *can* be shared among threads.");
		cl.def( pybind11::init<const enum mrpt::opengl::Buffer::Type>(), pybind11::arg("type") );

		cl.def( pybind11::init( [](){ return new mrpt::opengl::Buffer(); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::Buffer const &o){ return new mrpt::opengl::Buffer(o); } ) );

		pybind11::enum_<mrpt::opengl::Buffer::Type>(cl, "Type", "")
			.value("Vertex", mrpt::opengl::Buffer::Type::Vertex)
			.value("ElementIndex", mrpt::opengl::Buffer::Type::ElementIndex)
			.value("PixelPack", mrpt::opengl::Buffer::Type::PixelPack)
			.value("PixelUnpack", mrpt::opengl::Buffer::Type::PixelUnpack);


		pybind11::enum_<mrpt::opengl::Buffer::Usage>(cl, "Usage", "")
			.value("StreamDraw", mrpt::opengl::Buffer::Usage::StreamDraw)
			.value("StreamRead", mrpt::opengl::Buffer::Usage::StreamRead)
			.value("StreamCopy", mrpt::opengl::Buffer::Usage::StreamCopy)
			.value("StaticDraw", mrpt::opengl::Buffer::Usage::StaticDraw)
			.value("StaticRead", mrpt::opengl::Buffer::Usage::StaticRead)
			.value("StaticCopy", mrpt::opengl::Buffer::Usage::StaticCopy)
			.value("DynamicDraw", mrpt::opengl::Buffer::Usage::DynamicDraw)
			.value("DynamicRead", mrpt::opengl::Buffer::Usage::DynamicRead)
			.value("DynamicCopy", mrpt::opengl::Buffer::Usage::DynamicCopy);

		cl.def("type", (enum mrpt::opengl::Buffer::Type (mrpt::opengl::Buffer::*)() const) &mrpt::opengl::Buffer::type, "C++: mrpt::opengl::Buffer::type() const --> enum mrpt::opengl::Buffer::Type");
		cl.def("usage", (enum mrpt::opengl::Buffer::Usage (mrpt::opengl::Buffer::*)() const) &mrpt::opengl::Buffer::usage, "C++: mrpt::opengl::Buffer::usage() const --> enum mrpt::opengl::Buffer::Usage");
		cl.def("setUsage", (void (mrpt::opengl::Buffer::*)(const enum mrpt::opengl::Buffer::Usage)) &mrpt::opengl::Buffer::setUsage, "C++: mrpt::opengl::Buffer::setUsage(const enum mrpt::opengl::Buffer::Usage) --> void", pybind11::arg("u"));
		cl.def("createOnce", (void (mrpt::opengl::Buffer::*)()) &mrpt::opengl::Buffer::createOnce, "Calls create() only if the buffer has not been created yet. \n\nC++: mrpt::opengl::Buffer::createOnce() --> void");
		cl.def("initialized", (bool (mrpt::opengl::Buffer::*)() const) &mrpt::opengl::Buffer::initialized, "C++: mrpt::opengl::Buffer::initialized() const --> bool");
		cl.def("destroy", (void (mrpt::opengl::Buffer::*)()) &mrpt::opengl::Buffer::destroy, "Automatically called upon destructor, no need for the user to call it in\n normal situations. \n\nC++: mrpt::opengl::Buffer::destroy() --> void");
		cl.def("bind", (void (mrpt::opengl::Buffer::*)()) &mrpt::opengl::Buffer::bind, "C++: mrpt::opengl::Buffer::bind() --> void");
		cl.def("unbind", (void (mrpt::opengl::Buffer::*)()) &mrpt::opengl::Buffer::unbind, "C++: mrpt::opengl::Buffer::unbind() --> void");
		cl.def("bufferId", (unsigned int (mrpt::opengl::Buffer::*)() const) &mrpt::opengl::Buffer::bufferId, "C++: mrpt::opengl::Buffer::bufferId() const --> unsigned int");
		cl.def("allocate", (void (mrpt::opengl::Buffer::*)(const void *, int)) &mrpt::opengl::Buffer::allocate, "Reserves byteCount bytes in the buffer and copy to it the provided data.\n create() and bind() must be called before using this method.\n\nC++: mrpt::opengl::Buffer::allocate(const void *, int) --> void", pybind11::arg("data"), pybind11::arg("byteCount"));
		cl.def("assign", (class mrpt::opengl::Buffer & (mrpt::opengl::Buffer::*)(const class mrpt::opengl::Buffer &)) &mrpt::opengl::Buffer::operator=, "C++: mrpt::opengl::Buffer::operator=(const class mrpt::opengl::Buffer &) --> class mrpt::opengl::Buffer &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::opengl::VertexArrayObject file:mrpt/opengl/VertexArrayObject.h line:24
		pybind11::class_<mrpt::opengl::VertexArrayObject, std::shared_ptr<mrpt::opengl::VertexArrayObject>> cl(M("mrpt::opengl"), "VertexArrayObject", "A wrapper for an OpenGL vertex array object (VAO).\n Refer to docs for glGenVertexArrays().\n\n \n\n \n OpenGL VAOs *cannot* be shared among threads/GL contexts.");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::VertexArrayObject(); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::VertexArrayObject const &o){ return new mrpt::opengl::VertexArrayObject(o); } ) );
		cl.def("createOnce", (void (mrpt::opengl::VertexArrayObject::*)()) &mrpt::opengl::VertexArrayObject::createOnce, "Calls create() only if the buffer has not been created yet. \n\nC++: mrpt::opengl::VertexArrayObject::createOnce() --> void");
		cl.def("isCreated", (bool (mrpt::opengl::VertexArrayObject::*)() const) &mrpt::opengl::VertexArrayObject::isCreated, "C++: mrpt::opengl::VertexArrayObject::isCreated() const --> bool");
		cl.def("destroy", (void (mrpt::opengl::VertexArrayObject::*)()) &mrpt::opengl::VertexArrayObject::destroy, "Automatically called upon destructor, no need for the user to call it in\n normal situations. \n\nC++: mrpt::opengl::VertexArrayObject::destroy() --> void");
		cl.def("bind", (void (mrpt::opengl::VertexArrayObject::*)()) &mrpt::opengl::VertexArrayObject::bind, "C++: mrpt::opengl::VertexArrayObject::bind() --> void");
		cl.def("release", (void (mrpt::opengl::VertexArrayObject::*)()) &mrpt::opengl::VertexArrayObject::release, "C++: mrpt::opengl::VertexArrayObject::release() --> void");
		cl.def("bufferId", (unsigned int (mrpt::opengl::VertexArrayObject::*)() const) &mrpt::opengl::VertexArrayObject::bufferId, "C++: mrpt::opengl::VertexArrayObject::bufferId() const --> unsigned int");
		cl.def("assign", (class mrpt::opengl::VertexArrayObject & (mrpt::opengl::VertexArrayObject::*)(const class mrpt::opengl::VertexArrayObject &)) &mrpt::opengl::VertexArrayObject::operator=, "C++: mrpt::opengl::VertexArrayObject::operator=(const class mrpt::opengl::VertexArrayObject &) --> class mrpt::opengl::VertexArrayObject &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::opengl::CRenderizableShaderPoints file:mrpt/opengl/CRenderizableShaderPoints.h line:38
		pybind11::class_<mrpt::opengl::CRenderizableShaderPoints, std::shared_ptr<mrpt::opengl::CRenderizableShaderPoints>, mrpt::opengl::CRenderizable> cl(M("mrpt::opengl"), "CRenderizableShaderPoints", "Renderizable generic renderer for objects using the points shader.\n\n All points may have the same point size (see setPointSize()) or a dynamic,\n depth-dependent size to emulate the effect of larger points when looked\n closely (see enableVariablePointSize()).\n\n In the latter case, point size is computed in the shader as:\n\n  gl_PointSize = vertexPointSize +\n  variablePointSize_k/(variablePointSize_DepthScale*gl_Position.z + 0.01);\n\n where the paramters vertexPointSize, variablePointSize_k, and\n variablePointSize_DepthScale can be set in this class via setPointSize(),\n setVariablePointSize_k(), and setVariablePointSize_DepthScale(),\n respectively.\n\n  \n opengl::Scene\n\n \n\n ");
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CRenderizableShaderPoints::*)() const) &mrpt::opengl::CRenderizableShaderPoints::GetRuntimeClass, "C++: mrpt::opengl::CRenderizableShaderPoints::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CRenderizableShaderPoints::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CRenderizableShaderPoints::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("renderUpdateBuffers", (void (mrpt::opengl::CRenderizableShaderPoints::*)() const) &mrpt::opengl::CRenderizableShaderPoints::renderUpdateBuffers, "C++: mrpt::opengl::CRenderizableShaderPoints::renderUpdateBuffers() const --> void");
		cl.def("onUpdateBuffers_Points", (void (mrpt::opengl::CRenderizableShaderPoints::*)()) &mrpt::opengl::CRenderizableShaderPoints::onUpdateBuffers_Points, "Must be implemented in derived classes to update the geometric entities\n to be drawn in \"m_*_buffer\" fields. \n\nC++: mrpt::opengl::CRenderizableShaderPoints::onUpdateBuffers_Points() --> void");
		cl.def("setPointSize", (void (mrpt::opengl::CRenderizableShaderPoints::*)(float)) &mrpt::opengl::CRenderizableShaderPoints::setPointSize, "By default is 1.0. \n enableVariablePointSize() \n\nC++: mrpt::opengl::CRenderizableShaderPoints::setPointSize(float) --> void", pybind11::arg("p"));
		cl.def("getPointSize", (float (mrpt::opengl::CRenderizableShaderPoints::*)() const) &mrpt::opengl::CRenderizableShaderPoints::getPointSize, "C++: mrpt::opengl::CRenderizableShaderPoints::getPointSize() const --> float");
		cl.def("enableVariablePointSize", [](mrpt::opengl::CRenderizableShaderPoints &o) -> void { return o.enableVariablePointSize(); }, "");
		cl.def("enableVariablePointSize", (void (mrpt::opengl::CRenderizableShaderPoints::*)(bool)) &mrpt::opengl::CRenderizableShaderPoints::enableVariablePointSize, "Enable/disable variable eye distance-dependent point size (default=true)\n\nC++: mrpt::opengl::CRenderizableShaderPoints::enableVariablePointSize(bool) --> void", pybind11::arg("enable"));
		cl.def("isEnabledVariablePointSize", (bool (mrpt::opengl::CRenderizableShaderPoints::*)() const) &mrpt::opengl::CRenderizableShaderPoints::isEnabledVariablePointSize, "C++: mrpt::opengl::CRenderizableShaderPoints::isEnabledVariablePointSize() const --> bool");
		cl.def("setVariablePointSize_k", (void (mrpt::opengl::CRenderizableShaderPoints::*)(float)) &mrpt::opengl::CRenderizableShaderPoints::setVariablePointSize_k, "see CRenderizableShaderPoints for a discussion of this parameter. \n\nC++: mrpt::opengl::CRenderizableShaderPoints::setVariablePointSize_k(float) --> void", pybind11::arg("v"));
		cl.def("getVariablePointSize_k", (float (mrpt::opengl::CRenderizableShaderPoints::*)() const) &mrpt::opengl::CRenderizableShaderPoints::getVariablePointSize_k, "C++: mrpt::opengl::CRenderizableShaderPoints::getVariablePointSize_k() const --> float");
		cl.def("setVariablePointSize_DepthScale", (void (mrpt::opengl::CRenderizableShaderPoints::*)(float)) &mrpt::opengl::CRenderizableShaderPoints::setVariablePointSize_DepthScale, "see CRenderizableShaderPoints for a discussion of this parameter. \n\nC++: mrpt::opengl::CRenderizableShaderPoints::setVariablePointSize_DepthScale(float) --> void", pybind11::arg("v"));
		cl.def("getVariablePointSize_DepthScale", (float (mrpt::opengl::CRenderizableShaderPoints::*)() const) &mrpt::opengl::CRenderizableShaderPoints::getVariablePointSize_DepthScale, "C++: mrpt::opengl::CRenderizableShaderPoints::getVariablePointSize_DepthScale() const --> float");
		cl.def("freeOpenGLResources", (void (mrpt::opengl::CRenderizableShaderPoints::*)()) &mrpt::opengl::CRenderizableShaderPoints::freeOpenGLResources, "C++: mrpt::opengl::CRenderizableShaderPoints::freeOpenGLResources() --> void");
		cl.def("assign", (class mrpt::opengl::CRenderizableShaderPoints & (mrpt::opengl::CRenderizableShaderPoints::*)(const class mrpt::opengl::CRenderizableShaderPoints &)) &mrpt::opengl::CRenderizableShaderPoints::operator=, "C++: mrpt::opengl::CRenderizableShaderPoints::operator=(const class mrpt::opengl::CRenderizableShaderPoints &) --> class mrpt::opengl::CRenderizableShaderPoints &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::opengl::CRenderizableShaderWireFrame file:mrpt/opengl/CRenderizableShaderWireFrame.h line:26
		pybind11::class_<mrpt::opengl::CRenderizableShaderWireFrame, std::shared_ptr<mrpt::opengl::CRenderizableShaderWireFrame>, mrpt::opengl::CRenderizable> cl(M("mrpt::opengl"), "CRenderizableShaderWireFrame", "Renderizable generic renderer for objects using the wireframe shader.\n\n  \n opengl::Scene\n\n \n\n ");
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CRenderizableShaderWireFrame::*)() const) &mrpt::opengl::CRenderizableShaderWireFrame::GetRuntimeClass, "C++: mrpt::opengl::CRenderizableShaderWireFrame::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CRenderizableShaderWireFrame::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CRenderizableShaderWireFrame::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("renderUpdateBuffers", (void (mrpt::opengl::CRenderizableShaderWireFrame::*)() const) &mrpt::opengl::CRenderizableShaderWireFrame::renderUpdateBuffers, "C++: mrpt::opengl::CRenderizableShaderWireFrame::renderUpdateBuffers() const --> void");
		cl.def("onUpdateBuffers_Wireframe", (void (mrpt::opengl::CRenderizableShaderWireFrame::*)()) &mrpt::opengl::CRenderizableShaderWireFrame::onUpdateBuffers_Wireframe, "Must be implemented in derived classes to update the geometric entities\n to be drawn in \"m_*_buffer\" fields. \n\nC++: mrpt::opengl::CRenderizableShaderWireFrame::onUpdateBuffers_Wireframe() --> void");
		cl.def("setLineWidth", (void (mrpt::opengl::CRenderizableShaderWireFrame::*)(float)) &mrpt::opengl::CRenderizableShaderWireFrame::setLineWidth, "C++: mrpt::opengl::CRenderizableShaderWireFrame::setLineWidth(float) --> void", pybind11::arg("w"));
		cl.def("getLineWidth", (float (mrpt::opengl::CRenderizableShaderWireFrame::*)() const) &mrpt::opengl::CRenderizableShaderWireFrame::getLineWidth, "C++: mrpt::opengl::CRenderizableShaderWireFrame::getLineWidth() const --> float");
		cl.def("enableAntiAliasing", [](mrpt::opengl::CRenderizableShaderWireFrame &o) -> void { return o.enableAntiAliasing(); }, "");
		cl.def("enableAntiAliasing", (void (mrpt::opengl::CRenderizableShaderWireFrame::*)(bool)) &mrpt::opengl::CRenderizableShaderWireFrame::enableAntiAliasing, "C++: mrpt::opengl::CRenderizableShaderWireFrame::enableAntiAliasing(bool) --> void", pybind11::arg("enable"));
		cl.def("isAntiAliasingEnabled", (bool (mrpt::opengl::CRenderizableShaderWireFrame::*)() const) &mrpt::opengl::CRenderizableShaderWireFrame::isAntiAliasingEnabled, "C++: mrpt::opengl::CRenderizableShaderWireFrame::isAntiAliasingEnabled() const --> bool");
		cl.def("freeOpenGLResources", (void (mrpt::opengl::CRenderizableShaderWireFrame::*)()) &mrpt::opengl::CRenderizableShaderWireFrame::freeOpenGLResources, "C++: mrpt::opengl::CRenderizableShaderWireFrame::freeOpenGLResources() --> void");
		cl.def("assign", (class mrpt::opengl::CRenderizableShaderWireFrame & (mrpt::opengl::CRenderizableShaderWireFrame::*)(const class mrpt::opengl::CRenderizableShaderWireFrame &)) &mrpt::opengl::CRenderizableShaderWireFrame::operator=, "C++: mrpt::opengl::CRenderizableShaderWireFrame::operator=(const class mrpt::opengl::CRenderizableShaderWireFrame &) --> class mrpt::opengl::CRenderizableShaderWireFrame &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::opengl::CSetOfLines file:mrpt/opengl/CSetOfLines.h line:25
		pybind11::class_<mrpt::opengl::CSetOfLines, std::shared_ptr<mrpt::opengl::CSetOfLines>, PyCallBack_mrpt_opengl_CSetOfLines, mrpt::opengl::CRenderizableShaderWireFrame, mrpt::opengl::CRenderizableShaderPoints> cl(M("mrpt::opengl"), "CSetOfLines", "A set of independent lines (or segments), one line with its own start and\n end positions (X,Y,Z). Optionally, the vertices can be also shown as dots.\n\n ![mrpt::opengl::CSetOfLines](preview_CSetOfLines.png)\n\n \n opengl::Scene\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::CSetOfLines(); }, [](){ return new PyCallBack_mrpt_opengl_CSetOfLines(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_opengl_CSetOfLines const &o){ return new PyCallBack_mrpt_opengl_CSetOfLines(o); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::CSetOfLines const &o){ return new mrpt::opengl::CSetOfLines(o); } ) );
		cl.def_static("Create", (class std::shared_ptr<class mrpt::opengl::CSetOfLines> (*)()) &mrpt::opengl::CSetOfLines::Create, "C++: mrpt::opengl::CSetOfLines::Create() --> class std::shared_ptr<class mrpt::opengl::CSetOfLines>");
		cl.def("appendLine", (void (mrpt::opengl::CSetOfLines::*)(struct mrpt::math::TPoint3D_<double>, struct mrpt::math::TPoint3D_<double>)) &mrpt::opengl::CSetOfLines::appendLine<mrpt::math::TPoint3D_<double>,mrpt::math::TPoint3D_<double>>, "C++: mrpt::opengl::CSetOfLines::appendLine(struct mrpt::math::TPoint3D_<double>, struct mrpt::math::TPoint3D_<double>) --> void", pybind11::arg("p0"), pybind11::arg("p1"));
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CSetOfLines::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CSetOfLines::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CSetOfLines::*)() const) &mrpt::opengl::CSetOfLines::GetRuntimeClass, "C++: mrpt::opengl::CSetOfLines::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::opengl::CSetOfLines::*)() const) &mrpt::opengl::CSetOfLines::clone, "C++: mrpt::opengl::CSetOfLines::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::opengl::CSetOfLines::CreateObject, "C++: mrpt::opengl::CSetOfLines::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("renderUpdateBuffers", (void (mrpt::opengl::CSetOfLines::*)() const) &mrpt::opengl::CSetOfLines::renderUpdateBuffers, "C++: mrpt::opengl::CSetOfLines::renderUpdateBuffers() const --> void");
		cl.def("freeOpenGLResources", (void (mrpt::opengl::CSetOfLines::*)()) &mrpt::opengl::CSetOfLines::freeOpenGLResources, "C++: mrpt::opengl::CSetOfLines::freeOpenGLResources() --> void");
		cl.def("onUpdateBuffers_Wireframe", (void (mrpt::opengl::CSetOfLines::*)()) &mrpt::opengl::CSetOfLines::onUpdateBuffers_Wireframe, "C++: mrpt::opengl::CSetOfLines::onUpdateBuffers_Wireframe() --> void");
		cl.def("onUpdateBuffers_Points", (void (mrpt::opengl::CSetOfLines::*)()) &mrpt::opengl::CSetOfLines::onUpdateBuffers_Points, "C++: mrpt::opengl::CSetOfLines::onUpdateBuffers_Points() --> void");
		cl.def("clear", (void (mrpt::opengl::CSetOfLines::*)()) &mrpt::opengl::CSetOfLines::clear, "Clear the list of segments \n\nC++: mrpt::opengl::CSetOfLines::clear() --> void");
		cl.def("getVerticesPointSize", (float (mrpt::opengl::CSetOfLines::*)() const) &mrpt::opengl::CSetOfLines::getVerticesPointSize, "C++: mrpt::opengl::CSetOfLines::getVerticesPointSize() const --> float");
		cl.def("setVerticesPointSize", (void (mrpt::opengl::CSetOfLines::*)(const float)) &mrpt::opengl::CSetOfLines::setVerticesPointSize, "Enable showing vertices as dots if size_points>0 \n\nC++: mrpt::opengl::CSetOfLines::setVerticesPointSize(const float) --> void", pybind11::arg("size_points"));
		cl.def("appendLine", (void (mrpt::opengl::CSetOfLines::*)(const struct mrpt::math::TSegment3D &)) &mrpt::opengl::CSetOfLines::appendLine, "Appends a line to the set.\n\nC++: mrpt::opengl::CSetOfLines::appendLine(const struct mrpt::math::TSegment3D &) --> void", pybind11::arg("sgm"));
		cl.def("appendLine", (void (mrpt::opengl::CSetOfLines::*)(double, double, double, double, double, double)) &mrpt::opengl::CSetOfLines::appendLine, "Appends a line to the set, given the coordinates of its bounds.\n\nC++: mrpt::opengl::CSetOfLines::appendLine(double, double, double, double, double, double) --> void", pybind11::arg("x0"), pybind11::arg("y0"), pybind11::arg("z0"), pybind11::arg("x1"), pybind11::arg("y1"), pybind11::arg("z1"));
		cl.def("appendLineStrip", (void (mrpt::opengl::CSetOfLines::*)(float, float, float)) &mrpt::opengl::CSetOfLines::appendLineStrip, "Appends a line whose starting point is the end point of the last line\n (similar to OpenGL's GL_LINE_STRIP)\n  \n\n std::exception If there is no previous segment \n\nC++: mrpt::opengl::CSetOfLines::appendLineStrip(float, float, float) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("resize", (void (mrpt::opengl::CSetOfLines::*)(size_t)) &mrpt::opengl::CSetOfLines::resize, "Resizes the set.\n \n\n reserve\n\nC++: mrpt::opengl::CSetOfLines::resize(size_t) --> void", pybind11::arg("nLines"));
		cl.def("reserve", (void (mrpt::opengl::CSetOfLines::*)(size_t)) &mrpt::opengl::CSetOfLines::reserve, "Reserves an amount of lines to the set. This method should be used when\n some known amount of lines is going to be inserted, so that only a memory\n allocation is needed.\n \n\n resize\n\nC++: mrpt::opengl::CSetOfLines::reserve(size_t) --> void", pybind11::arg("r"));
		cl.def("getLineCount", (size_t (mrpt::opengl::CSetOfLines::*)() const) &mrpt::opengl::CSetOfLines::getLineCount, "Returns the total count of lines in this set. \n\nC++: mrpt::opengl::CSetOfLines::getLineCount() const --> size_t");
		cl.def("size", (size_t (mrpt::opengl::CSetOfLines::*)() const) &mrpt::opengl::CSetOfLines::size, "Returns the total count of lines in this set. \n\nC++: mrpt::opengl::CSetOfLines::size() const --> size_t");
		cl.def("empty", (bool (mrpt::opengl::CSetOfLines::*)() const) &mrpt::opengl::CSetOfLines::empty, "Returns true if there are no line segments. \n\nC++: mrpt::opengl::CSetOfLines::empty() const --> bool");
		cl.def("setLineByIndex", (void (mrpt::opengl::CSetOfLines::*)(size_t, const struct mrpt::math::TSegment3D &)) &mrpt::opengl::CSetOfLines::setLineByIndex, "Sets a specific line in the set, given its index.\n \n\n appendLine\n\nC++: mrpt::opengl::CSetOfLines::setLineByIndex(size_t, const struct mrpt::math::TSegment3D &) --> void", pybind11::arg("index"), pybind11::arg("segm"));
		cl.def("setLineByIndex", (void (mrpt::opengl::CSetOfLines::*)(size_t, double, double, double, double, double, double)) &mrpt::opengl::CSetOfLines::setLineByIndex, "Sets a specific line in the set, given its index.\n \n\n appendLine\n\nC++: mrpt::opengl::CSetOfLines::setLineByIndex(size_t, double, double, double, double, double, double) --> void", pybind11::arg("index"), pybind11::arg("x0"), pybind11::arg("y0"), pybind11::arg("z0"), pybind11::arg("x1"), pybind11::arg("y1"), pybind11::arg("z1"));
		cl.def("getLineByIndex", (void (mrpt::opengl::CSetOfLines::*)(size_t, double &, double &, double &, double &, double &, double &) const) &mrpt::opengl::CSetOfLines::getLineByIndex, "Gets a specific line in the set, given its index.\n \n\n getLineByIndex\n\nC++: mrpt::opengl::CSetOfLines::getLineByIndex(size_t, double &, double &, double &, double &, double &, double &) const --> void", pybind11::arg("index"), pybind11::arg("x0"), pybind11::arg("y0"), pybind11::arg("z0"), pybind11::arg("x1"), pybind11::arg("y1"), pybind11::arg("z1"));
		cl.def("internalBoundingBoxLocal", (struct mrpt::math::TBoundingBox_<float> (mrpt::opengl::CSetOfLines::*)() const) &mrpt::opengl::CSetOfLines::internalBoundingBoxLocal, "Evaluates the bounding box of this object (including possible children)\n in the coordinate frame of the object parent. \n\nC++: mrpt::opengl::CSetOfLines::internalBoundingBoxLocal() const --> struct mrpt::math::TBoundingBox_<float>");
		cl.def("enableAntiAliasing", [](mrpt::opengl::CSetOfLines &o) -> void { return o.enableAntiAliasing(); }, "");
		cl.def("enableAntiAliasing", (void (mrpt::opengl::CSetOfLines::*)(bool)) &mrpt::opengl::CSetOfLines::enableAntiAliasing, "C++: mrpt::opengl::CSetOfLines::enableAntiAliasing(bool) --> void", pybind11::arg("enable"));
		cl.def("isAntiAliasingEnabled", (bool (mrpt::opengl::CSetOfLines::*)() const) &mrpt::opengl::CSetOfLines::isAntiAliasingEnabled, "C++: mrpt::opengl::CSetOfLines::isAntiAliasingEnabled() const --> bool");
		cl.def("assign", (class mrpt::opengl::CSetOfLines & (mrpt::opengl::CSetOfLines::*)(const class mrpt::opengl::CSetOfLines &)) &mrpt::opengl::CSetOfLines::operator=, "C++: mrpt::opengl::CSetOfLines::operator=(const class mrpt::opengl::CSetOfLines &) --> class mrpt::opengl::CSetOfLines &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
