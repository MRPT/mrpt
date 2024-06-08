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
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TPlane.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPolygon2D.h>
#include <mrpt/math/TPolygon3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TPose3DQuat.h>
#include <mrpt/math/TSegment3D.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CRenderizableShaderText.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/opengl/CTextMessageCapable.h>
#include <mrpt/opengl/TTriangle.h>
#include <mrpt/opengl/opengl_fonts.h>
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

// mrpt::opengl::CText file:mrpt/opengl/CText.h line:27
struct PyCallBack_mrpt_opengl_CText : public mrpt::opengl::CText {
	using mrpt::opengl::CText::CText;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CText *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CText::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CText *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CText::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CText *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CText::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CText *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CText::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CText *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CText::serializeFrom(a0, a1);
	}
	void onUpdateBuffers_Text() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CText *>(this), "onUpdateBuffers_Text");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CText::onUpdateBuffers_Text();
	}
	struct mrpt::math::TBoundingBox_<float> internalBoundingBoxLocal() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CText *>(this), "internalBoundingBoxLocal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TBoundingBox_<float>>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TBoundingBox_<float>> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TBoundingBox_<float>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TBoundingBox_<float>>(std::move(o));
		}
		return CText::internalBoundingBoxLocal();
	}
	void toYAMLMap(class mrpt::containers::yaml & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CText *>(this), "toYAMLMap");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CText::toYAMLMap(a0);
	}
	void renderUpdateBuffers() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CText *>(this), "renderUpdateBuffers");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRenderizableShaderText::renderUpdateBuffers();
	}
	void freeOpenGLResources() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CText *>(this), "freeOpenGLResources");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRenderizableShaderText::freeOpenGLResources();
	}
	class mrpt::opengl::CRenderizable & setColorA_u8(const unsigned char a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CText *>(this), "setColorA_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CText *>(this), "setColor_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CText *>(this), "cullElegible");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CText *>(this), "isCompositeObject");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CText *>(this), "traceRay");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CText *>(this), "getLocalRepresentativePoint");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CText *>(this), "initializeTextures");
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

void bind_mrpt_opengl_TTriangle(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::opengl::TTriangle file:mrpt/opengl/TTriangle.h line:36
		pybind11::class_<mrpt::opengl::TTriangle, std::shared_ptr<mrpt::opengl::TTriangle>> cl(M("mrpt::opengl"), "TTriangle", "A triangle (float coordinates) with RGBA colors (u8) and UV (texture\n coordinates) for each vertex. Note that not all the fields must be filled in,\n it depends on the consumer of the structure.\n\n The structure is memory packed to 1-byte, to ensure it can be used in GPU\n memory vertex arrays without unexpected paddings.\n\n \n opengl::Scene, CSetOfTexturedTriangles\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::TTriangle(); } ) );
		cl.def( pybind11::init<const class mrpt::math::TPolygon3D &>(), pybind11::arg("p") );

		cl.def( pybind11::init<const struct mrpt::math::TPoint3D_<float> &, const struct mrpt::math::TPoint3D_<float> &, const struct mrpt::math::TPoint3D_<float> &>(), pybind11::arg("p1"), pybind11::arg("p2"), pybind11::arg("p3") );

		cl.def( pybind11::init<const struct mrpt::math::TPoint3D_<float> &, const struct mrpt::math::TPoint3D_<float> &, const struct mrpt::math::TPoint3D_<float> &, const struct mrpt::math::TPoint3D_<float> &, const struct mrpt::math::TPoint3D_<float> &, const struct mrpt::math::TPoint3D_<float> &>(), pybind11::arg("p1"), pybind11::arg("p2"), pybind11::arg("p3"), pybind11::arg("n1"), pybind11::arg("n2"), pybind11::arg("n3") );

		cl.def( pybind11::init( [](mrpt::opengl::TTriangle const &o){ return new mrpt::opengl::TTriangle(o); } ) );
		cl.def_readwrite("vertices", &mrpt::opengl::TTriangle::vertices);
		cl.def("x", (float & (mrpt::opengl::TTriangle::*)(size_t)) &mrpt::opengl::TTriangle::x, "C++: mrpt::opengl::TTriangle::x(size_t) --> float &", pybind11::return_value_policy::reference, pybind11::arg("i"));
		cl.def("y", (float & (mrpt::opengl::TTriangle::*)(size_t)) &mrpt::opengl::TTriangle::y, "C++: mrpt::opengl::TTriangle::y(size_t) --> float &", pybind11::return_value_policy::reference, pybind11::arg("i"));
		cl.def("z", (float & (mrpt::opengl::TTriangle::*)(size_t)) &mrpt::opengl::TTriangle::z, "C++: mrpt::opengl::TTriangle::z(size_t) --> float &", pybind11::return_value_policy::reference, pybind11::arg("i"));
		cl.def("r", (unsigned char & (mrpt::opengl::TTriangle::*)(size_t)) &mrpt::opengl::TTriangle::r, "C++: mrpt::opengl::TTriangle::r(size_t) --> unsigned char &", pybind11::return_value_policy::automatic, pybind11::arg("i"));
		cl.def("g", (unsigned char & (mrpt::opengl::TTriangle::*)(size_t)) &mrpt::opengl::TTriangle::g, "C++: mrpt::opengl::TTriangle::g(size_t) --> unsigned char &", pybind11::return_value_policy::automatic, pybind11::arg("i"));
		cl.def("b", (unsigned char & (mrpt::opengl::TTriangle::*)(size_t)) &mrpt::opengl::TTriangle::b, "C++: mrpt::opengl::TTriangle::b(size_t) --> unsigned char &", pybind11::return_value_policy::automatic, pybind11::arg("i"));
		cl.def("a", (unsigned char & (mrpt::opengl::TTriangle::*)(size_t)) &mrpt::opengl::TTriangle::a, "C++: mrpt::opengl::TTriangle::a(size_t) --> unsigned char &", pybind11::return_value_policy::automatic, pybind11::arg("i"));
		cl.def("u", (float & (mrpt::opengl::TTriangle::*)(size_t)) &mrpt::opengl::TTriangle::u, "C++: mrpt::opengl::TTriangle::u(size_t) --> float &", pybind11::return_value_policy::reference, pybind11::arg("i"));
		cl.def("v", (float & (mrpt::opengl::TTriangle::*)(size_t)) &mrpt::opengl::TTriangle::v, "C++: mrpt::opengl::TTriangle::v(size_t) --> float &", pybind11::return_value_policy::reference, pybind11::arg("i"));
		cl.def("vertex", (struct mrpt::math::TPoint3D_<float> & (mrpt::opengl::TTriangle::*)(size_t)) &mrpt::opengl::TTriangle::vertex, "C++: mrpt::opengl::TTriangle::vertex(size_t) --> struct mrpt::math::TPoint3D_<float> &", pybind11::return_value_policy::automatic, pybind11::arg("i"));
		cl.def("setColor", (void (mrpt::opengl::TTriangle::*)(const struct mrpt::img::TColor &)) &mrpt::opengl::TTriangle::setColor, "Sets the color of all vertices \n\nC++: mrpt::opengl::TTriangle::setColor(const struct mrpt::img::TColor &) --> void", pybind11::arg("c"));
		cl.def("setColor", (void (mrpt::opengl::TTriangle::*)(const struct mrpt::img::TColorf &)) &mrpt::opengl::TTriangle::setColor, "C++: mrpt::opengl::TTriangle::setColor(const struct mrpt::img::TColorf &) --> void", pybind11::arg("c"));
		cl.def("computeNormals", (void (mrpt::opengl::TTriangle::*)()) &mrpt::opengl::TTriangle::computeNormals, "Compute the three normals from the cross-product of \"v01 x v02\".\n Note that using this default normals will not lead to interpolated\n lighting in the fragment shaders, since all vertex are equal; a derived\n class should use custom, more accurate normals to enable soft lighting.\n\nC++: mrpt::opengl::TTriangle::computeNormals() --> void");
		cl.def("readFrom", (void (mrpt::opengl::TTriangle::*)(class mrpt::serialization::CArchive &)) &mrpt::opengl::TTriangle::readFrom, "C++: mrpt::opengl::TTriangle::readFrom(class mrpt::serialization::CArchive &) --> void", pybind11::arg("i"));
		cl.def("writeTo", (void (mrpt::opengl::TTriangle::*)(class mrpt::serialization::CArchive &) const) &mrpt::opengl::TTriangle::writeTo, "C++: mrpt::opengl::TTriangle::writeTo(class mrpt::serialization::CArchive &) const --> void", pybind11::arg("o"));
		cl.def("assign", (struct mrpt::opengl::TTriangle & (mrpt::opengl::TTriangle::*)(const struct mrpt::opengl::TTriangle &)) &mrpt::opengl::TTriangle::operator=, "C++: mrpt::opengl::TTriangle::operator=(const struct mrpt::opengl::TTriangle &) --> struct mrpt::opengl::TTriangle &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::opengl::TTriangle::Vertex file:mrpt/opengl/TTriangle.h line:38
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::opengl::TTriangle::Vertex, std::shared_ptr<mrpt::opengl::TTriangle::Vertex>> cl(enclosing_class, "Vertex", "");
			cl.def( pybind11::init( [](){ return new mrpt::opengl::TTriangle::Vertex(); } ) );
			cl.def( pybind11::init( [](mrpt::opengl::TTriangle::Vertex const &o){ return new mrpt::opengl::TTriangle::Vertex(o); } ) );
			cl.def_readwrite("xyzrgba", &mrpt::opengl::TTriangle::Vertex::xyzrgba);
			cl.def_readwrite("normal", &mrpt::opengl::TTriangle::Vertex::normal);
			cl.def_readwrite("uv", &mrpt::opengl::TTriangle::Vertex::uv);
			cl.def("setColor", (void (mrpt::opengl::TTriangle::Vertex::*)(const struct mrpt::img::TColor &)) &mrpt::opengl::TTriangle::Vertex::setColor, "C++: mrpt::opengl::TTriangle::Vertex::setColor(const struct mrpt::img::TColor &) --> void", pybind11::arg("c"));
			cl.def("assign", (struct mrpt::opengl::TTriangle::Vertex & (mrpt::opengl::TTriangle::Vertex::*)(const struct mrpt::opengl::TTriangle::Vertex &)) &mrpt::opengl::TTriangle::Vertex::operator=, "C++: mrpt::opengl::TTriangle::Vertex::operator=(const struct mrpt::opengl::TTriangle::Vertex &) --> struct mrpt::opengl::TTriangle::Vertex &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
	{ // mrpt::opengl::CRenderizableShaderText file:mrpt/opengl/CRenderizableShaderText.h line:27
		pybind11::class_<mrpt::opengl::CRenderizableShaderText, std::shared_ptr<mrpt::opengl::CRenderizableShaderText>, mrpt::opengl::CRenderizable> cl(M("mrpt::opengl"), "CRenderizableShaderText", "Renderizable generic renderer for objects using the \"text shader\".\n\n  \n opengl::Scene\n\n \n\n ");
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CRenderizableShaderText::*)() const) &mrpt::opengl::CRenderizableShaderText::GetRuntimeClass, "C++: mrpt::opengl::CRenderizableShaderText::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CRenderizableShaderText::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CRenderizableShaderText::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("renderUpdateBuffers", (void (mrpt::opengl::CRenderizableShaderText::*)() const) &mrpt::opengl::CRenderizableShaderText::renderUpdateBuffers, "C++: mrpt::opengl::CRenderizableShaderText::renderUpdateBuffers() const --> void");
		cl.def("onUpdateBuffers_Text", (void (mrpt::opengl::CRenderizableShaderText::*)()) &mrpt::opengl::CRenderizableShaderText::onUpdateBuffers_Text, "Must be implemented in derived classes to update the geometric entities\n to be drawn in \"m_*_buffer\" fields. \n\nC++: mrpt::opengl::CRenderizableShaderText::onUpdateBuffers_Text() --> void");
		cl.def("freeOpenGLResources", (void (mrpt::opengl::CRenderizableShaderText::*)()) &mrpt::opengl::CRenderizableShaderText::freeOpenGLResources, "C++: mrpt::opengl::CRenderizableShaderText::freeOpenGLResources() --> void");
		cl.def("assign", (class mrpt::opengl::CRenderizableShaderText & (mrpt::opengl::CRenderizableShaderText::*)(const class mrpt::opengl::CRenderizableShaderText &)) &mrpt::opengl::CRenderizableShaderText::operator=, "C++: mrpt::opengl::CRenderizableShaderText::operator=(const class mrpt::opengl::CRenderizableShaderText &) --> class mrpt::opengl::CRenderizableShaderText &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::opengl::CText file:mrpt/opengl/CText.h line:27
		pybind11::class_<mrpt::opengl::CText, std::shared_ptr<mrpt::opengl::CText>, PyCallBack_mrpt_opengl_CText, mrpt::opengl::CRenderizableShaderText> cl(M("mrpt::opengl"), "CText", "A 2D text that always \"faces the observer\" despite it having a real 3D\n position, used to compute its position on the screen, and depth (so it can be\n occluded).\n\n Use setString() and setFont() to change the text and its appareance.\n\n ![mrpt::opengl::CText](preview_CText.png)\n\n \n CText3D, opengl::Scene\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::CText(); }, [](){ return new PyCallBack_mrpt_opengl_CText(); } ), "doc");
		cl.def( pybind11::init<const std::string &>(), pybind11::arg("str") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_opengl_CText const &o){ return new PyCallBack_mrpt_opengl_CText(o); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::CText const &o){ return new mrpt::opengl::CText(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CText::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CText::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CText::*)() const) &mrpt::opengl::CText::GetRuntimeClass, "C++: mrpt::opengl::CText::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::opengl::CText::*)() const) &mrpt::opengl::CText::clone, "C++: mrpt::opengl::CText::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::opengl::CText::CreateObject, "C++: mrpt::opengl::CText::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("setString", (void (mrpt::opengl::CText::*)(const std::string &)) &mrpt::opengl::CText::setString, "Sets the text to display \n\nC++: mrpt::opengl::CText::setString(const std::string &) --> void", pybind11::arg("s"));
		cl.def("getString", (std::string (mrpt::opengl::CText::*)() const) &mrpt::opengl::CText::getString, "Return the current text associated to this label \n\nC++: mrpt::opengl::CText::getString() const --> std::string");
		cl.def("setFont", (void (mrpt::opengl::CText::*)(const std::string &, int)) &mrpt::opengl::CText::setFont, "Sets the font among \"sans\", \"serif\", \"mono\". \n\nC++: mrpt::opengl::CText::setFont(const std::string &, int) --> void", pybind11::arg("s"), pybind11::arg("height"));
		cl.def("getFont", (std::string (mrpt::opengl::CText::*)() const) &mrpt::opengl::CText::getFont, "C++: mrpt::opengl::CText::getFont() const --> std::string");
		cl.def("internalBoundingBoxLocal", (struct mrpt::math::TBoundingBox_<float> (mrpt::opengl::CText::*)() const) &mrpt::opengl::CText::internalBoundingBoxLocal, "Evaluates the bounding box of this object (including possible children)\n in the coordinate frame of the object parent. \n\nC++: mrpt::opengl::CText::internalBoundingBoxLocal() const --> struct mrpt::math::TBoundingBox_<float>");
		cl.def("computeTextExtension", (struct std::pair<double, double> (mrpt::opengl::CText::*)() const) &mrpt::opengl::CText::computeTextExtension, "C++: mrpt::opengl::CText::computeTextExtension() const --> struct std::pair<double, double>");
		cl.def("toYAMLMap", (void (mrpt::opengl::CText::*)(class mrpt::containers::yaml &) const) &mrpt::opengl::CText::toYAMLMap, "C++: mrpt::opengl::CText::toYAMLMap(class mrpt::containers::yaml &) const --> void", pybind11::arg("propertiesMap"));
		cl.def("assign", (class mrpt::opengl::CText & (mrpt::opengl::CText::*)(const class mrpt::opengl::CText &)) &mrpt::opengl::CText::operator=, "C++: mrpt::opengl::CText::operator=(const class mrpt::opengl::CText &) --> class mrpt::opengl::CText &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::opengl::CTextMessageCapable file:mrpt/opengl/CTextMessageCapable.h line:25
		pybind11::class_<mrpt::opengl::CTextMessageCapable, std::shared_ptr<mrpt::opengl::CTextMessageCapable>> cl(M("mrpt::opengl"), "CTextMessageCapable", "Keeps a list of text messages which can be rendered to OpenGL contexts by\n graphic classes.\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::CTextMessageCapable(); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::CTextMessageCapable const &o){ return new mrpt::opengl::CTextMessageCapable(o); } ) );
		cl.def("clearTextMessages", (void (mrpt::opengl::CTextMessageCapable::*)()) &mrpt::opengl::CTextMessageCapable::clearTextMessages, "C++: mrpt::opengl::CTextMessageCapable::clearTextMessages() --> void");
		cl.def("addTextMessage", [](mrpt::opengl::CTextMessageCapable &o, const double & a0, const double & a1, const std::string & a2) -> void { return o.addTextMessage(a0, a1, a2); }, "", pybind11::arg("x_frac"), pybind11::arg("y_frac"), pybind11::arg("text"));
		cl.def("addTextMessage", [](mrpt::opengl::CTextMessageCapable &o, const double & a0, const double & a1, const std::string & a2, const unsigned long & a3) -> void { return o.addTextMessage(a0, a1, a2, a3); }, "", pybind11::arg("x_frac"), pybind11::arg("y_frac"), pybind11::arg("text"), pybind11::arg("unique_index"));
		cl.def("addTextMessage", (void (mrpt::opengl::CTextMessageCapable::*)(const double, const double, const std::string &, const unsigned long, const struct mrpt::opengl::TFontParams &)) &mrpt::opengl::CTextMessageCapable::addTextMessage, "Add 2D text messages overlapped to the 3D rendered scene. The string\n will remain displayed in the 3D window\n   until it's changed with subsequent calls to this same method, or all\n the texts are cleared with clearTextMessages().\n\n  \n The X position, interpreted as absolute pixels from the left\n if X>=1, absolute pixels from the left if X<0 or as a width factor if in\n the range [0,1[.\n  \n\n The Y position, interpreted as absolute pixels from the bottom\n if Y>=1, absolute pixels from the top if Y<0 or as a height factor if in\n the range [0,1[.\n  \n\n The text string to display.\n  \n\n The text color. For example: TColorf(1.0,1.0,1.0)\n  \n\n An \"index\" for this text message, so that\n subsequent calls with the same index will overwrite this text message\n instead of creating new ones.\n\n  You'll need to refresh the display manually with forceRepaint().\n\n \n clearTextMessages, updateTextMessage\n\nC++: mrpt::opengl::CTextMessageCapable::addTextMessage(const double, const double, const std::string &, const unsigned long, const struct mrpt::opengl::TFontParams &) --> void", pybind11::arg("x_frac"), pybind11::arg("y_frac"), pybind11::arg("text"), pybind11::arg("unique_index"), pybind11::arg("fontParams"));
		cl.def("updateTextMessage", (bool (mrpt::opengl::CTextMessageCapable::*)(size_t, const std::string &)) &mrpt::opengl::CTextMessageCapable::updateTextMessage, "Just updates the text of a given text message, without touching the\n other parameters.\n \n\n false if given ID doesn't exist.\n\nC++: mrpt::opengl::CTextMessageCapable::updateTextMessage(size_t, const std::string &) --> bool", pybind11::arg("unique_index"), pybind11::arg("text"));
		cl.def("getTextMessages", (const struct mrpt::opengl::CTextMessageCapable::TListTextMessages & (mrpt::opengl::CTextMessageCapable::*)() const) &mrpt::opengl::CTextMessageCapable::getTextMessages, "C++: mrpt::opengl::CTextMessageCapable::getTextMessages() const --> const struct mrpt::opengl::CTextMessageCapable::TListTextMessages &", pybind11::return_value_policy::automatic);
		cl.def("assign", (class mrpt::opengl::CTextMessageCapable & (mrpt::opengl::CTextMessageCapable::*)(const class mrpt::opengl::CTextMessageCapable &)) &mrpt::opengl::CTextMessageCapable::operator=, "C++: mrpt::opengl::CTextMessageCapable::operator=(const class mrpt::opengl::CTextMessageCapable &) --> class mrpt::opengl::CTextMessageCapable &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::opengl::CTextMessageCapable::DataPerText file:mrpt/opengl/CTextMessageCapable.h line:64
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::opengl::CTextMessageCapable::DataPerText, std::shared_ptr<mrpt::opengl::CTextMessageCapable::DataPerText>, mrpt::opengl::T2DTextData> cl(enclosing_class, "DataPerText", "");
			cl.def( pybind11::init( [](){ return new mrpt::opengl::CTextMessageCapable::DataPerText(); } ) );
			cl.def( pybind11::init( [](mrpt::opengl::CTextMessageCapable::DataPerText const &o){ return new mrpt::opengl::CTextMessageCapable::DataPerText(o); } ) );
			cl.def_readwrite("gl_text", &mrpt::opengl::CTextMessageCapable::DataPerText::gl_text);
			cl.def_readwrite("gl_text_shadow", &mrpt::opengl::CTextMessageCapable::DataPerText::gl_text_shadow);
			cl.def_readwrite("gl_text_outdated", &mrpt::opengl::CTextMessageCapable::DataPerText::gl_text_outdated);
			cl.def("assign", (struct mrpt::opengl::CTextMessageCapable::DataPerText & (mrpt::opengl::CTextMessageCapable::DataPerText::*)(const struct mrpt::opengl::CTextMessageCapable::DataPerText &)) &mrpt::opengl::CTextMessageCapable::DataPerText::operator=, "C++: mrpt::opengl::CTextMessageCapable::DataPerText::operator=(const struct mrpt::opengl::CTextMessageCapable::DataPerText &) --> struct mrpt::opengl::CTextMessageCapable::DataPerText &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::opengl::CTextMessageCapable::TListTextMessages file:mrpt/opengl/CTextMessageCapable.h line:70
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::opengl::CTextMessageCapable::TListTextMessages, std::shared_ptr<mrpt::opengl::CTextMessageCapable::TListTextMessages>> cl(enclosing_class, "TListTextMessages", "");
			cl.def( pybind11::init( [](){ return new mrpt::opengl::CTextMessageCapable::TListTextMessages(); } ) );
			cl.def( pybind11::init( [](mrpt::opengl::CTextMessageCapable::TListTextMessages const &o){ return new mrpt::opengl::CTextMessageCapable::TListTextMessages(o); } ) );
			cl.def_readwrite("mtx", &mrpt::opengl::CTextMessageCapable::TListTextMessages::mtx);
			cl.def_readwrite("messages", &mrpt::opengl::CTextMessageCapable::TListTextMessages::messages);
			cl.def("regenerateGLobjects", (void (mrpt::opengl::CTextMessageCapable::TListTextMessages::*)() const) &mrpt::opengl::CTextMessageCapable::TListTextMessages::regenerateGLobjects, "(re)generate all CText objects in the gl_text fields \n\nC++: mrpt::opengl::CTextMessageCapable::TListTextMessages::regenerateGLobjects() const --> void");
			cl.def("assign", (struct mrpt::opengl::CTextMessageCapable::TListTextMessages & (mrpt::opengl::CTextMessageCapable::TListTextMessages::*)(const struct mrpt::opengl::CTextMessageCapable::TListTextMessages &)) &mrpt::opengl::CTextMessageCapable::TListTextMessages::operator=, "C++: mrpt::opengl::CTextMessageCapable::TListTextMessages::operator=(const struct mrpt::opengl::CTextMessageCapable::TListTextMessages &) --> struct mrpt::opengl::CTextMessageCapable::TListTextMessages &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
}
