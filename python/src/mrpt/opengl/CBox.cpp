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
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TPose3DQuat.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/opengl/CBox.h>
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

// mrpt::opengl::CBox file:mrpt/opengl/CBox.h line:33
struct PyCallBack_mrpt_opengl_CBox : public mrpt::opengl::CBox {
	using mrpt::opengl::CBox::CBox;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CBox *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CBox::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CBox *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CBox::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CBox *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CBox::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CBox *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CBox::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CBox *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CBox::serializeFrom(a0, a1);
	}
	void renderUpdateBuffers() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CBox *>(this), "renderUpdateBuffers");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CBox::renderUpdateBuffers();
	}
	void onUpdateBuffers_Wireframe() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CBox *>(this), "onUpdateBuffers_Wireframe");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CBox::onUpdateBuffers_Wireframe();
	}
	void onUpdateBuffers_Triangles() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CBox *>(this), "onUpdateBuffers_Triangles");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CBox::onUpdateBuffers_Triangles();
	}
	void freeOpenGLResources() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CBox *>(this), "freeOpenGLResources");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CBox::freeOpenGLResources();
	}
	struct mrpt::math::TBoundingBox_<float> internalBoundingBoxLocal() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CBox *>(this), "internalBoundingBoxLocal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TBoundingBox_<float>>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TBoundingBox_<float>> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TBoundingBox_<float>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TBoundingBox_<float>>(std::move(o));
		}
		return CBox::internalBoundingBoxLocal();
	}
	bool traceRay(const class mrpt::poses::CPose3D & a0, double & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CBox *>(this), "traceRay");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CBox::traceRay(a0, a1);
	}
	class mrpt::opengl::CRenderizable & setColorA_u8(const unsigned char a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CBox *>(this), "setColorA_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CBox *>(this), "setColor_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CBox *>(this), "cullElegible");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CBox *>(this), "toYAMLMap");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CBox *>(this), "isCompositeObject");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CBox *>(this), "getLocalRepresentativePoint");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CBox *>(this), "initializeTextures");
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

void bind_mrpt_opengl_CBox(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::opengl::CBox file:mrpt/opengl/CBox.h line:33
		pybind11::class_<mrpt::opengl::CBox, std::shared_ptr<mrpt::opengl::CBox>, PyCallBack_mrpt_opengl_CBox, mrpt::opengl::CRenderizableShaderTriangles, mrpt::opengl::CRenderizableShaderWireFrame> cl(M("mrpt::opengl"), "CBox", "A solid or wireframe box in 3D, defined by 6 rectangular faces parallel to\nthe planes X, Y and Z (note that the object can be translated and rotated\nafterwards as any other CRenderizable object using the \"object pose\" in the\nbase class).\n  Three drawing modes are possible:\n	- Wireframe: setWireframe(true). Used color is the CRenderizable color\n	- Solid box: setWireframe(false). Used color is the CRenderizable color\n	- Solid box with border: setWireframe(false) + enableBoxBorder(true). Solid\ncolor is the CRenderizable color, border line can be set with\nsetBoxBorderColor().\n\n ![mrpt::opengl::CBox](preview_CBox.png)\n\n \n opengl::Scene,opengl::CRenderizable\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::CBox(); }, [](){ return new PyCallBack_mrpt_opengl_CBox(); } ) );
		cl.def( pybind11::init( [](const struct mrpt::math::TPoint3D_<double> & a0, const struct mrpt::math::TPoint3D_<double> & a1){ return new mrpt::opengl::CBox(a0, a1); }, [](const struct mrpt::math::TPoint3D_<double> & a0, const struct mrpt::math::TPoint3D_<double> & a1){ return new PyCallBack_mrpt_opengl_CBox(a0, a1); } ), "doc");
		cl.def( pybind11::init( [](const struct mrpt::math::TPoint3D_<double> & a0, const struct mrpt::math::TPoint3D_<double> & a1, bool const & a2){ return new mrpt::opengl::CBox(a0, a1, a2); }, [](const struct mrpt::math::TPoint3D_<double> & a0, const struct mrpt::math::TPoint3D_<double> & a1, bool const & a2){ return new PyCallBack_mrpt_opengl_CBox(a0, a1, a2); } ), "doc");
		cl.def( pybind11::init<const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &, bool, float>(), pybind11::arg("corner1"), pybind11::arg("corner2"), pybind11::arg("is_wireframe"), pybind11::arg("lineWidth") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_opengl_CBox const &o){ return new PyCallBack_mrpt_opengl_CBox(o); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::CBox const &o){ return new mrpt::opengl::CBox(o); } ) );
		cl.def_static("Create", (class std::shared_ptr<class mrpt::opengl::CBox> (*)()) &mrpt::opengl::CBox::Create, "C++: mrpt::opengl::CBox::Create() --> class std::shared_ptr<class mrpt::opengl::CBox>");
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CBox::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CBox::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CBox::*)() const) &mrpt::opengl::CBox::GetRuntimeClass, "C++: mrpt::opengl::CBox::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::opengl::CBox::*)() const) &mrpt::opengl::CBox::clone, "C++: mrpt::opengl::CBox::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::opengl::CBox::CreateObject, "C++: mrpt::opengl::CBox::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("renderUpdateBuffers", (void (mrpt::opengl::CBox::*)() const) &mrpt::opengl::CBox::renderUpdateBuffers, "C++: mrpt::opengl::CBox::renderUpdateBuffers() const --> void");
		cl.def("onUpdateBuffers_Wireframe", (void (mrpt::opengl::CBox::*)()) &mrpt::opengl::CBox::onUpdateBuffers_Wireframe, "C++: mrpt::opengl::CBox::onUpdateBuffers_Wireframe() --> void");
		cl.def("onUpdateBuffers_Triangles", (void (mrpt::opengl::CBox::*)()) &mrpt::opengl::CBox::onUpdateBuffers_Triangles, "C++: mrpt::opengl::CBox::onUpdateBuffers_Triangles() --> void");
		cl.def("freeOpenGLResources", (void (mrpt::opengl::CBox::*)()) &mrpt::opengl::CBox::freeOpenGLResources, "C++: mrpt::opengl::CBox::freeOpenGLResources() --> void");
		cl.def("internalBoundingBoxLocal", (struct mrpt::math::TBoundingBox_<float> (mrpt::opengl::CBox::*)() const) &mrpt::opengl::CBox::internalBoundingBoxLocal, "@} \n\nC++: mrpt::opengl::CBox::internalBoundingBoxLocal() const --> struct mrpt::math::TBoundingBox_<float>");
		cl.def("traceRay", (bool (mrpt::opengl::CBox::*)(const class mrpt::poses::CPose3D &, double &) const) &mrpt::opengl::CBox::traceRay, "Ray tracing.\n \n\n mrpt::opengl::CRenderizable\n\nC++: mrpt::opengl::CBox::traceRay(const class mrpt::poses::CPose3D &, double &) const --> bool", pybind11::arg("o"), pybind11::arg("dist"));
		cl.def("setLineWidth", (void (mrpt::opengl::CBox::*)(float)) &mrpt::opengl::CBox::setLineWidth, "C++: mrpt::opengl::CBox::setLineWidth(float) --> void", pybind11::arg("width"));
		cl.def("getLineWidth", (float (mrpt::opengl::CBox::*)() const) &mrpt::opengl::CBox::getLineWidth, "C++: mrpt::opengl::CBox::getLineWidth() const --> float");
		cl.def("setWireframe", [](mrpt::opengl::CBox &o) -> void { return o.setWireframe(); }, "");
		cl.def("setWireframe", (void (mrpt::opengl::CBox::*)(bool)) &mrpt::opengl::CBox::setWireframe, "C++: mrpt::opengl::CBox::setWireframe(bool) --> void", pybind11::arg("is_wireframe"));
		cl.def("isWireframe", (bool (mrpt::opengl::CBox::*)() const) &mrpt::opengl::CBox::isWireframe, "C++: mrpt::opengl::CBox::isWireframe() const --> bool");
		cl.def("enableBoxBorder", [](mrpt::opengl::CBox &o) -> void { return o.enableBoxBorder(); }, "");
		cl.def("enableBoxBorder", (void (mrpt::opengl::CBox::*)(bool)) &mrpt::opengl::CBox::enableBoxBorder, "C++: mrpt::opengl::CBox::enableBoxBorder(bool) --> void", pybind11::arg("drawBorder"));
		cl.def("isBoxBorderEnabled", (bool (mrpt::opengl::CBox::*)() const) &mrpt::opengl::CBox::isBoxBorderEnabled, "C++: mrpt::opengl::CBox::isBoxBorderEnabled() const --> bool");
		cl.def("setBoxBorderColor", (void (mrpt::opengl::CBox::*)(const struct mrpt::img::TColor &)) &mrpt::opengl::CBox::setBoxBorderColor, "C++: mrpt::opengl::CBox::setBoxBorderColor(const struct mrpt::img::TColor &) --> void", pybind11::arg("c"));
		cl.def("getBoxBorderColor", (struct mrpt::img::TColor (mrpt::opengl::CBox::*)() const) &mrpt::opengl::CBox::getBoxBorderColor, "C++: mrpt::opengl::CBox::getBoxBorderColor() const --> struct mrpt::img::TColor");
		cl.def("setBoxCorners", (void (mrpt::opengl::CBox::*)(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &)) &mrpt::opengl::CBox::setBoxCorners, "Set the position and size of the box, from two corners in 3D \n\nC++: mrpt::opengl::CBox::setBoxCorners(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &) --> void", pybind11::arg("corner1"), pybind11::arg("corner2"));
		cl.def("getBoxCorners", (void (mrpt::opengl::CBox::*)(struct mrpt::math::TPoint3D_<double> &, struct mrpt::math::TPoint3D_<double> &) const) &mrpt::opengl::CBox::getBoxCorners, "C++: mrpt::opengl::CBox::getBoxCorners(struct mrpt::math::TPoint3D_<double> &, struct mrpt::math::TPoint3D_<double> &) const --> void", pybind11::arg("corner1"), pybind11::arg("corner2"));
		cl.def("assign", (class mrpt::opengl::CBox & (mrpt::opengl::CBox::*)(const class mrpt::opengl::CBox &)) &mrpt::opengl::CBox::operator=, "C++: mrpt::opengl::CBox::operator=(const class mrpt::opengl::CBox &) --> class mrpt::opengl::CBox &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
