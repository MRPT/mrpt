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
#include <mrpt/maps/CPointsMap.h>
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
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CRenderizableShaderText.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/opengl/CText3D.h>
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

// mrpt::opengl::CText3D file:mrpt/opengl/CText3D.h line:34
struct PyCallBack_mrpt_opengl_CText3D : public mrpt::opengl::CText3D {
	using mrpt::opengl::CText3D::CText3D;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CText3D *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CText3D::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CText3D *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CText3D::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CText3D *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CText3D::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CText3D *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CText3D::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CText3D *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CText3D::serializeFrom(a0, a1);
	}
	void onUpdateBuffers_Text() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CText3D *>(this), "onUpdateBuffers_Text");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CText3D::onUpdateBuffers_Text();
	}
	struct mrpt::math::TBoundingBox_<float> internalBoundingBoxLocal() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CText3D *>(this), "internalBoundingBoxLocal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TBoundingBox_<float>>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TBoundingBox_<float>> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TBoundingBox_<float>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TBoundingBox_<float>>(std::move(o));
		}
		return CText3D::internalBoundingBoxLocal();
	}
	void toYAMLMap(class mrpt::containers::yaml & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CText3D *>(this), "toYAMLMap");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CText3D::toYAMLMap(a0);
	}
	void renderUpdateBuffers() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CText3D *>(this), "renderUpdateBuffers");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CText3D *>(this), "freeOpenGLResources");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CText3D *>(this), "setColorA_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CText3D *>(this), "setColor_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CText3D *>(this), "cullElegible");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CText3D *>(this), "isCompositeObject");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CText3D *>(this), "traceRay");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CText3D *>(this), "getLocalRepresentativePoint");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CText3D *>(this), "initializeTextures");
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

void bind_mrpt_opengl_CPointCloud(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::opengl::PointCloudAdapter file:mrpt/opengl/CPointCloud.h line:331
		pybind11::class_<mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloud>, std::shared_ptr<mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloud>>> cl(M("mrpt::opengl"), "PointCloudAdapter_mrpt_opengl_CPointCloud_t", "Specialization mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloud>\n \n");
		cl.def( pybind11::init<const class mrpt::opengl::CPointCloud &>(), pybind11::arg("obj") );

		cl.def( pybind11::init( [](mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloud> const &o){ return new mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloud>(o); } ) );
		cl.def("size", (size_t (mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloud>::*)() const) &mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloud>::size, "Get number of points \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloud>::size() const --> size_t");
		cl.def("resize", (void (mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloud>::*)(size_t)) &mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloud>::resize, "Set number of points (to uninitialized values) \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloud>::resize(size_t) --> void", pybind11::arg("N"));
		cl.def("setDimensions", (void (mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloud>::*)(size_t, size_t)) &mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloud>::setDimensions, "Does nothing as of now \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloud>::setDimensions(size_t, size_t) --> void", pybind11::arg("height"), pybind11::arg("width"));
		cl.def("setPointXYZ", (void (mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloud>::*)(size_t, const float, const float, const float)) &mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloud>::setPointXYZ, "Set XYZ coordinates of i'th point \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloud>::setPointXYZ(size_t, const float, const float, const float) --> void", pybind11::arg("idx"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("setInvalidPoint", (void (mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloud>::*)(size_t)) &mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloud>::setInvalidPoint, "Set XYZ coordinates of i'th point \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::opengl::CPointCloud>::setInvalidPoint(size_t) --> void", pybind11::arg("idx"));
	}
	{ // mrpt::opengl::CText3D file:mrpt/opengl/CText3D.h line:34
		pybind11::class_<mrpt::opengl::CText3D, std::shared_ptr<mrpt::opengl::CText3D>, PyCallBack_mrpt_opengl_CText3D, mrpt::opengl::CRenderizableShaderText> cl(M("mrpt::opengl"), "CText3D", "A 3D text (rendered with OpenGL primitives), with selectable font face and\n drawing style.\n  Use  and  to change the text displayed by this object\n (can be multi-lined).\n\n  Text is drawn along the (+X,+Y) axes.\n\n Default size of characters is \"1.0 units\". Change it with the standard\n method  as with any other 3D object.\n The color can be also changed with standard methods in the base class \n\n ![mrpt::opengl::CText3D](preview_CText3D.png)\n\n \n opengl::Scene, CText\n \n\n This class is based on code from libcvd (BSD,\n http://www.edwardrosten.com/cvd/ ) \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::CText3D(); }, [](){ return new PyCallBack_mrpt_opengl_CText3D(); } ), "doc");
		cl.def( pybind11::init( [](const std::string & a0){ return new mrpt::opengl::CText3D(a0); }, [](const std::string & a0){ return new PyCallBack_mrpt_opengl_CText3D(a0); } ), "doc");
		cl.def( pybind11::init( [](const std::string & a0, const std::string & a1){ return new mrpt::opengl::CText3D(a0, a1); }, [](const std::string & a0, const std::string & a1){ return new PyCallBack_mrpt_opengl_CText3D(a0, a1); } ), "doc");
		cl.def( pybind11::init( [](const std::string & a0, const std::string & a1, const float & a2){ return new mrpt::opengl::CText3D(a0, a1, a2); }, [](const std::string & a0, const std::string & a1, const float & a2){ return new PyCallBack_mrpt_opengl_CText3D(a0, a1, a2); } ), "doc");
		cl.def( pybind11::init( [](const std::string & a0, const std::string & a1, const float & a2, const enum mrpt::opengl::TOpenGLFontStyle & a3){ return new mrpt::opengl::CText3D(a0, a1, a2, a3); }, [](const std::string & a0, const std::string & a1, const float & a2, const enum mrpt::opengl::TOpenGLFontStyle & a3){ return new PyCallBack_mrpt_opengl_CText3D(a0, a1, a2, a3); } ), "doc");
		cl.def( pybind11::init( [](const std::string & a0, const std::string & a1, const float & a2, const enum mrpt::opengl::TOpenGLFontStyle & a3, const double & a4){ return new mrpt::opengl::CText3D(a0, a1, a2, a3, a4); }, [](const std::string & a0, const std::string & a1, const float & a2, const enum mrpt::opengl::TOpenGLFontStyle & a3, const double & a4){ return new PyCallBack_mrpt_opengl_CText3D(a0, a1, a2, a3, a4); } ), "doc");
		cl.def( pybind11::init<const std::string &, const std::string &, const float, const enum mrpt::opengl::TOpenGLFontStyle, const double, const double>(), pybind11::arg("str"), pybind11::arg("fontName"), pybind11::arg("scale"), pybind11::arg("text_style"), pybind11::arg("text_spacing"), pybind11::arg("text_kerning") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_opengl_CText3D const &o){ return new PyCallBack_mrpt_opengl_CText3D(o); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::CText3D const &o){ return new mrpt::opengl::CText3D(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CText3D::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CText3D::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CText3D::*)() const) &mrpt::opengl::CText3D::GetRuntimeClass, "C++: mrpt::opengl::CText3D::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::opengl::CText3D::*)() const) &mrpt::opengl::CText3D::clone, "C++: mrpt::opengl::CText3D::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::opengl::CText3D::CreateObject, "C++: mrpt::opengl::CText3D::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("setString", (void (mrpt::opengl::CText3D::*)(const std::string &)) &mrpt::opengl::CText3D::setString, "Sets the displayed string \n\nC++: mrpt::opengl::CText3D::setString(const std::string &) --> void", pybind11::arg("s"));
		cl.def("getString", (const std::string & (mrpt::opengl::CText3D::*)() const) &mrpt::opengl::CText3D::getString, "Returns the currently text associated to this object \n\nC++: mrpt::opengl::CText3D::getString() const --> const std::string &", pybind11::return_value_policy::automatic);
		cl.def("setFont", (void (mrpt::opengl::CText3D::*)(const std::string &)) &mrpt::opengl::CText3D::setFont, "Changes the font name, among accepted values: \"sans\", \"mono\", \"serif\" \n\nC++: mrpt::opengl::CText3D::setFont(const std::string &) --> void", pybind11::arg("font"));
		cl.def("getFont", (const std::string & (mrpt::opengl::CText3D::*)() const) &mrpt::opengl::CText3D::getFont, "Returns the text font  \n\nC++: mrpt::opengl::CText3D::getFont() const --> const std::string &", pybind11::return_value_policy::automatic);
		cl.def("setTextStyle", (void (mrpt::opengl::CText3D::*)(const enum mrpt::opengl::TOpenGLFontStyle)) &mrpt::opengl::CText3D::setTextStyle, "Change drawing style: FILL, OUTLINE, NICE \n\nC++: mrpt::opengl::CText3D::setTextStyle(const enum mrpt::opengl::TOpenGLFontStyle) --> void", pybind11::arg("text_style"));
		cl.def("getTextStyle", (enum mrpt::opengl::TOpenGLFontStyle (mrpt::opengl::CText3D::*)() const) &mrpt::opengl::CText3D::getTextStyle, "Gets the current drawing style \n\nC++: mrpt::opengl::CText3D::getTextStyle() const --> enum mrpt::opengl::TOpenGLFontStyle");
		cl.def("setTextSpacing", (void (mrpt::opengl::CText3D::*)(const double)) &mrpt::opengl::CText3D::setTextSpacing, "C++: mrpt::opengl::CText3D::setTextSpacing(const double) --> void", pybind11::arg("text_spacing"));
		cl.def("setTextSpacing", (double (mrpt::opengl::CText3D::*)() const) &mrpt::opengl::CText3D::setTextSpacing, "C++: mrpt::opengl::CText3D::setTextSpacing() const --> double");
		cl.def("setTextKerning", (void (mrpt::opengl::CText3D::*)(const double)) &mrpt::opengl::CText3D::setTextKerning, "C++: mrpt::opengl::CText3D::setTextKerning(const double) --> void", pybind11::arg("text_kerning"));
		cl.def("setTextKerning", (double (mrpt::opengl::CText3D::*)() const) &mrpt::opengl::CText3D::setTextKerning, "C++: mrpt::opengl::CText3D::setTextKerning() const --> double");
		cl.def("internalBoundingBoxLocal", (struct mrpt::math::TBoundingBox_<float> (mrpt::opengl::CText3D::*)() const) &mrpt::opengl::CText3D::internalBoundingBoxLocal, "C++: mrpt::opengl::CText3D::internalBoundingBoxLocal() const --> struct mrpt::math::TBoundingBox_<float>");
		cl.def("toYAMLMap", (void (mrpt::opengl::CText3D::*)(class mrpt::containers::yaml &) const) &mrpt::opengl::CText3D::toYAMLMap, "C++: mrpt::opengl::CText3D::toYAMLMap(class mrpt::containers::yaml &) const --> void", pybind11::arg("propertiesMap"));
		cl.def("assign", (class mrpt::opengl::CText3D & (mrpt::opengl::CText3D::*)(const class mrpt::opengl::CText3D &)) &mrpt::opengl::CText3D::operator=, "C++: mrpt::opengl::CText3D::operator=(const class mrpt::opengl::CText3D &) --> class mrpt::opengl::CText3D &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
