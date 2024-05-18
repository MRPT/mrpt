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
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/opengl/CVectorField3D.h>
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

// mrpt::opengl::CVectorField3D file:mrpt/opengl/CVectorField3D.h line:32
struct PyCallBack_mrpt_opengl_CVectorField3D : public mrpt::opengl::CVectorField3D {
	using mrpt::opengl::CVectorField3D::CVectorField3D;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CVectorField3D *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CVectorField3D::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CVectorField3D *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CVectorField3D::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CVectorField3D *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CVectorField3D::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CVectorField3D *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CVectorField3D::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CVectorField3D *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CVectorField3D::serializeFrom(a0, a1);
	}
	void renderUpdateBuffers() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CVectorField3D *>(this), "renderUpdateBuffers");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CVectorField3D::renderUpdateBuffers();
	}
	void freeOpenGLResources() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CVectorField3D *>(this), "freeOpenGLResources");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CVectorField3D::freeOpenGLResources();
	}
	void onUpdateBuffers_Wireframe() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CVectorField3D *>(this), "onUpdateBuffers_Wireframe");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CVectorField3D::onUpdateBuffers_Wireframe();
	}
	void onUpdateBuffers_Points() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CVectorField3D *>(this), "onUpdateBuffers_Points");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CVectorField3D::onUpdateBuffers_Points();
	}
	struct mrpt::math::TBoundingBox_<float> internalBoundingBoxLocal() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CVectorField3D *>(this), "internalBoundingBoxLocal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TBoundingBox_<float>>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TBoundingBox_<float>> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TBoundingBox_<float>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TBoundingBox_<float>>(std::move(o));
		}
		return CVectorField3D::internalBoundingBoxLocal();
	}
	class mrpt::opengl::CRenderizable & setColorA_u8(const unsigned char a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CVectorField3D *>(this), "setColorA_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CVectorField3D *>(this), "setColor_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CVectorField3D *>(this), "cullElegible");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CVectorField3D *>(this), "toYAMLMap");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CVectorField3D *>(this), "isCompositeObject");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CVectorField3D *>(this), "traceRay");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CVectorField3D *>(this), "getLocalRepresentativePoint");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CVectorField3D *>(this), "initializeTextures");
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

void bind_mrpt_opengl_CVectorField3D(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::opengl::CVectorField3D file:mrpt/opengl/CVectorField3D.h line:32
		pybind11::class_<mrpt::opengl::CVectorField3D, std::shared_ptr<mrpt::opengl::CVectorField3D>, PyCallBack_mrpt_opengl_CVectorField3D, mrpt::opengl::CRenderizableShaderPoints, mrpt::opengl::CRenderizableShaderWireFrame> cl(M("mrpt::opengl"), "CVectorField3D", "A 3D vector field representation, consisting of points and arrows drawn at\n any spatial position.\n This opengl object has been created to represent scene flow, and hence\n both the vector field and\n the coordinates of the points at which the vector field is represented\n are stored in matrices because\n they are computed from intensity and depth images.\n\n ![mrpt::opengl::CVectorField3D](preview_CVectorField3D.png)\n\n \n opengl::Scene\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::CVectorField3D(); }, [](){ return new PyCallBack_mrpt_opengl_CVectorField3D(); } ) );
		cl.def( pybind11::init<class mrpt::math::CMatrixDynamic<float>, class mrpt::math::CMatrixDynamic<float>, class mrpt::math::CMatrixDynamic<float>, class mrpt::math::CMatrixDynamic<float>, class mrpt::math::CMatrixDynamic<float>, class mrpt::math::CMatrixDynamic<float>>(), pybind11::arg("x_vf_ini"), pybind11::arg("y_vf_ini"), pybind11::arg("z_vf_ini"), pybind11::arg("x_p_ini"), pybind11::arg("y_p_ini"), pybind11::arg("z_p_ini") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_opengl_CVectorField3D const &o){ return new PyCallBack_mrpt_opengl_CVectorField3D(o); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::CVectorField3D const &o){ return new mrpt::opengl::CVectorField3D(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CVectorField3D::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CVectorField3D::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CVectorField3D::*)() const) &mrpt::opengl::CVectorField3D::GetRuntimeClass, "C++: mrpt::opengl::CVectorField3D::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::opengl::CVectorField3D::*)() const) &mrpt::opengl::CVectorField3D::clone, "C++: mrpt::opengl::CVectorField3D::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::opengl::CVectorField3D::CreateObject, "C++: mrpt::opengl::CVectorField3D::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("renderUpdateBuffers", (void (mrpt::opengl::CVectorField3D::*)() const) &mrpt::opengl::CVectorField3D::renderUpdateBuffers, "C++: mrpt::opengl::CVectorField3D::renderUpdateBuffers() const --> void");
		cl.def("freeOpenGLResources", (void (mrpt::opengl::CVectorField3D::*)()) &mrpt::opengl::CVectorField3D::freeOpenGLResources, "C++: mrpt::opengl::CVectorField3D::freeOpenGLResources() --> void");
		cl.def("onUpdateBuffers_Wireframe", (void (mrpt::opengl::CVectorField3D::*)()) &mrpt::opengl::CVectorField3D::onUpdateBuffers_Wireframe, "C++: mrpt::opengl::CVectorField3D::onUpdateBuffers_Wireframe() --> void");
		cl.def("onUpdateBuffers_Points", (void (mrpt::opengl::CVectorField3D::*)()) &mrpt::opengl::CVectorField3D::onUpdateBuffers_Points, "C++: mrpt::opengl::CVectorField3D::onUpdateBuffers_Points() --> void");
		cl.def("clear", (void (mrpt::opengl::CVectorField3D::*)()) &mrpt::opengl::CVectorField3D::clear, "Clear the matrices\n\nC++: mrpt::opengl::CVectorField3D::clear() --> void");
		cl.def("setPointColor", [](mrpt::opengl::CVectorField3D &o, const float & a0, const float & a1, const float & a2) -> void { return o.setPointColor(a0, a1, a2); }, "", pybind11::arg("R"), pybind11::arg("G"), pybind11::arg("B"));
		cl.def("setPointColor", (void (mrpt::opengl::CVectorField3D::*)(const float, const float, const float, const float)) &mrpt::opengl::CVectorField3D::setPointColor, "Set the point color in the range [0,1]\n\nC++: mrpt::opengl::CVectorField3D::setPointColor(const float, const float, const float, const float) --> void", pybind11::arg("R"), pybind11::arg("G"), pybind11::arg("B"), pybind11::arg("A"));
		cl.def("getPointColor", (struct mrpt::img::TColorf (mrpt::opengl::CVectorField3D::*)() const) &mrpt::opengl::CVectorField3D::getPointColor, "Get the point color in the range [0,1]\n\nC++: mrpt::opengl::CVectorField3D::getPointColor() const --> struct mrpt::img::TColorf");
		cl.def("setVectorFieldColor", [](mrpt::opengl::CVectorField3D &o, const float & a0, const float & a1, const float & a2) -> void { return o.setVectorFieldColor(a0, a1, a2); }, "", pybind11::arg("R"), pybind11::arg("G"), pybind11::arg("B"));
		cl.def("setVectorFieldColor", (void (mrpt::opengl::CVectorField3D::*)(const float, const float, const float, const float)) &mrpt::opengl::CVectorField3D::setVectorFieldColor, "Set the arrow color in the range [0,1]\n\nC++: mrpt::opengl::CVectorField3D::setVectorFieldColor(const float, const float, const float, const float) --> void", pybind11::arg("R"), pybind11::arg("G"), pybind11::arg("B"), pybind11::arg("A"));
		cl.def("getVectorFieldColor", (void (mrpt::opengl::CVectorField3D::*)(struct mrpt::img::TColorf, struct mrpt::img::TColorf) const) &mrpt::opengl::CVectorField3D::getVectorFieldColor, "Get the motion field min and max colors (colormap) in the range [0,1]\n\nC++: mrpt::opengl::CVectorField3D::getVectorFieldColor(struct mrpt::img::TColorf, struct mrpt::img::TColorf) const --> void", pybind11::arg("Cmin"), pybind11::arg("Cmax"));
		cl.def("setMotionFieldColormap", [](mrpt::opengl::CVectorField3D &o, const float & a0, const float & a1, const float & a2, const float & a3, const float & a4, const float & a5) -> void { return o.setMotionFieldColormap(a0, a1, a2, a3, a4, a5); }, "", pybind11::arg("Rmin"), pybind11::arg("Gmin"), pybind11::arg("Bmin"), pybind11::arg("Rmax"), pybind11::arg("Gmax"), pybind11::arg("Bmax"));
		cl.def("setMotionFieldColormap", [](mrpt::opengl::CVectorField3D &o, const float & a0, const float & a1, const float & a2, const float & a3, const float & a4, const float & a5, const float & a6) -> void { return o.setMotionFieldColormap(a0, a1, a2, a3, a4, a5, a6); }, "", pybind11::arg("Rmin"), pybind11::arg("Gmin"), pybind11::arg("Bmin"), pybind11::arg("Rmax"), pybind11::arg("Gmax"), pybind11::arg("Bmax"), pybind11::arg("Amin"));
		cl.def("setMotionFieldColormap", (void (mrpt::opengl::CVectorField3D::*)(const float, const float, const float, const float, const float, const float, const float, const float)) &mrpt::opengl::CVectorField3D::setMotionFieldColormap, "Set the motion field min and max colors (colormap) in the range [0,1]\n\nC++: mrpt::opengl::CVectorField3D::setMotionFieldColormap(const float, const float, const float, const float, const float, const float, const float, const float) --> void", pybind11::arg("Rmin"), pybind11::arg("Gmin"), pybind11::arg("Bmin"), pybind11::arg("Rmax"), pybind11::arg("Gmax"), pybind11::arg("Bmax"), pybind11::arg("Amin"), pybind11::arg("Amax"));
		cl.def("getVectorFieldColor", (struct mrpt::img::TColorf (mrpt::opengl::CVectorField3D::*)() const) &mrpt::opengl::CVectorField3D::getVectorFieldColor, "Get the arrow color in the range [0,1]\n\nC++: mrpt::opengl::CVectorField3D::getVectorFieldColor() const --> struct mrpt::img::TColorf");
		cl.def("setMaxSpeedForColor", (void (mrpt::opengl::CVectorField3D::*)(const float)) &mrpt::opengl::CVectorField3D::setMaxSpeedForColor, "Set the max speed associated for the color map ( m_still_color,\n m_maxspeed_color)\n\nC++: mrpt::opengl::CVectorField3D::setMaxSpeedForColor(const float) --> void", pybind11::arg("s"));
		cl.def("getMaxSpeedForColor", (float (mrpt::opengl::CVectorField3D::*)() const) &mrpt::opengl::CVectorField3D::getMaxSpeedForColor, "Get the max_speed  with which lines are drawn.\n\nC++: mrpt::opengl::CVectorField3D::getMaxSpeedForColor() const --> float");
		cl.def("getVectorField", (void (mrpt::opengl::CVectorField3D::*)(class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &) const) &mrpt::opengl::CVectorField3D::getVectorField, "Get the vector field in three independent matrices: Matrix_x, Matrix_y\n and Matrix_z.\n\nC++: mrpt::opengl::CVectorField3D::getVectorField(class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &) const --> void", pybind11::arg("Matrix_x"), pybind11::arg("Matrix_y"), pybind11::arg("Matrix_z"));
		cl.def("getPointCoordinates", (void (mrpt::opengl::CVectorField3D::*)(class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &) const) &mrpt::opengl::CVectorField3D::getPointCoordinates, "Get the coordiantes of the points at which the vector field is\n plotted: Coord_x, Coord_y and Coord_z.\n\nC++: mrpt::opengl::CVectorField3D::getPointCoordinates(class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &) const --> void", pybind11::arg("Coord_x"), pybind11::arg("Coord_y"), pybind11::arg("Coord_z"));
		cl.def("getVectorField_x", (class mrpt::math::CMatrixDynamic<float> & (mrpt::opengl::CVectorField3D::*)()) &mrpt::opengl::CVectorField3D::getVectorField_x, "C++: mrpt::opengl::CVectorField3D::getVectorField_x() --> class mrpt::math::CMatrixDynamic<float> &", pybind11::return_value_policy::automatic);
		cl.def("getVectorField_y", (class mrpt::math::CMatrixDynamic<float> & (mrpt::opengl::CVectorField3D::*)()) &mrpt::opengl::CVectorField3D::getVectorField_y, "C++: mrpt::opengl::CVectorField3D::getVectorField_y() --> class mrpt::math::CMatrixDynamic<float> &", pybind11::return_value_policy::automatic);
		cl.def("getVectorField_z", (class mrpt::math::CMatrixDynamic<float> & (mrpt::opengl::CVectorField3D::*)()) &mrpt::opengl::CVectorField3D::getVectorField_z, "C++: mrpt::opengl::CVectorField3D::getVectorField_z() --> class mrpt::math::CMatrixDynamic<float> &", pybind11::return_value_policy::automatic);
		cl.def("setVectorField", (void (mrpt::opengl::CVectorField3D::*)(class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &)) &mrpt::opengl::CVectorField3D::setVectorField, "Set the vector field with Matrix_x, Matrix_y and Matrix_z.\n\nC++: mrpt::opengl::CVectorField3D::setVectorField(class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &) --> void", pybind11::arg("Matrix_x"), pybind11::arg("Matrix_y"), pybind11::arg("Matrix_z"));
		cl.def("setPointCoordinates", (void (mrpt::opengl::CVectorField3D::*)(class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &)) &mrpt::opengl::CVectorField3D::setPointCoordinates, "Set the coordinates of the points at which the vector field is plotted\n with Matrix_x, Matrix_y and Matrix_z.\n\nC++: mrpt::opengl::CVectorField3D::setPointCoordinates(class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &, class mrpt::math::CMatrixDynamic<float> &) --> void", pybind11::arg("Matrix_x"), pybind11::arg("Matrix_y"), pybind11::arg("Matrix_z"));
		cl.def("resize", (void (mrpt::opengl::CVectorField3D::*)(size_t, size_t)) &mrpt::opengl::CVectorField3D::resize, "Resizes the set.\n\nC++: mrpt::opengl::CVectorField3D::resize(size_t, size_t) --> void", pybind11::arg("rows"), pybind11::arg("cols"));
		cl.def("cols", (size_t (mrpt::opengl::CVectorField3D::*)() const) &mrpt::opengl::CVectorField3D::cols, "Returns the total count of rows used to represent the vector field. \n\nC++: mrpt::opengl::CVectorField3D::cols() const --> size_t");
		cl.def("rows", (size_t (mrpt::opengl::CVectorField3D::*)() const) &mrpt::opengl::CVectorField3D::rows, "Returns the total count of columns used to represent the vector field.\n\nC++: mrpt::opengl::CVectorField3D::rows() const --> size_t");
		cl.def("internalBoundingBoxLocal", (struct mrpt::math::TBoundingBox_<float> (mrpt::opengl::CVectorField3D::*)() const) &mrpt::opengl::CVectorField3D::internalBoundingBoxLocal, "C++: mrpt::opengl::CVectorField3D::internalBoundingBoxLocal() const --> struct mrpt::math::TBoundingBox_<float>");
		cl.def("enableColorFromModule", [](mrpt::opengl::CVectorField3D &o) -> void { return o.enableColorFromModule(); }, "");
		cl.def("enableColorFromModule", (void (mrpt::opengl::CVectorField3D::*)(bool)) &mrpt::opengl::CVectorField3D::enableColorFromModule, "C++: mrpt::opengl::CVectorField3D::enableColorFromModule(bool) --> void", pybind11::arg("enable"));
		cl.def("enableShowPoints", [](mrpt::opengl::CVectorField3D &o) -> void { return o.enableShowPoints(); }, "");
		cl.def("enableShowPoints", (void (mrpt::opengl::CVectorField3D::*)(bool)) &mrpt::opengl::CVectorField3D::enableShowPoints, "C++: mrpt::opengl::CVectorField3D::enableShowPoints(bool) --> void", pybind11::arg("enable"));
		cl.def("isAntiAliasingEnabled", (bool (mrpt::opengl::CVectorField3D::*)() const) &mrpt::opengl::CVectorField3D::isAntiAliasingEnabled, "C++: mrpt::opengl::CVectorField3D::isAntiAliasingEnabled() const --> bool");
		cl.def("isColorFromModuleEnabled", (bool (mrpt::opengl::CVectorField3D::*)() const) &mrpt::opengl::CVectorField3D::isColorFromModuleEnabled, "C++: mrpt::opengl::CVectorField3D::isColorFromModuleEnabled() const --> bool");
		cl.def("assign", (class mrpt::opengl::CVectorField3D & (mrpt::opengl::CVectorField3D::*)(const class mrpt::opengl::CVectorField3D &)) &mrpt::opengl::CVectorField3D::operator=, "C++: mrpt::opengl::CVectorField3D::operator=(const class mrpt::opengl::CVectorField3D &) --> class mrpt::opengl::CVectorField3D &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
