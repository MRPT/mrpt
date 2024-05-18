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
#include <mrpt/opengl/CAxis.h>
#include <mrpt/opengl/CColorBar.h>
#include <mrpt/opengl/CDisk.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CRenderizableShaderTriangles.h>
#include <mrpt/opengl/CRenderizableShaderWireFrame.h>
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

// mrpt::opengl::CAxis file:mrpt/opengl/CAxis.h line:24
struct PyCallBack_mrpt_opengl_CAxis : public mrpt::opengl::CAxis {
	using mrpt::opengl::CAxis::CAxis;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CAxis *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CAxis::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CAxis *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CAxis::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CAxis *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CAxis::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CAxis *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CAxis::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CAxis *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CAxis::serializeFrom(a0, a1);
	}
	void onUpdateBuffers_Wireframe() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CAxis *>(this), "onUpdateBuffers_Wireframe");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CAxis::onUpdateBuffers_Wireframe();
	}
	bool isCompositeObject() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CAxis *>(this), "isCompositeObject");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CAxis::isCompositeObject();
	}
	struct mrpt::math::TBoundingBox_<float> internalBoundingBoxLocal() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CAxis *>(this), "internalBoundingBoxLocal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TBoundingBox_<float>>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TBoundingBox_<float>> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TBoundingBox_<float>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TBoundingBox_<float>>(std::move(o));
		}
		return CAxis::internalBoundingBoxLocal();
	}
	void renderUpdateBuffers() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CAxis *>(this), "renderUpdateBuffers");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CAxis *>(this), "freeOpenGLResources");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CAxis *>(this), "setColorA_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CAxis *>(this), "setColor_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CAxis *>(this), "cullElegible");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CAxis *>(this), "toYAMLMap");
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
	bool traceRay(const class mrpt::poses::CPose3D & a0, double & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CAxis *>(this), "traceRay");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CAxis *>(this), "getLocalRepresentativePoint");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CAxis *>(this), "initializeTextures");
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

// mrpt::opengl::CColorBar file:mrpt/opengl/CColorBar.h line:28
struct PyCallBack_mrpt_opengl_CColorBar : public mrpt::opengl::CColorBar {
	using mrpt::opengl::CColorBar::CColorBar;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CColorBar *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CColorBar::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CColorBar *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CColorBar::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CColorBar *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CColorBar::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CColorBar *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CColorBar::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CColorBar *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CColorBar::serializeFrom(a0, a1);
	}
	void renderUpdateBuffers() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CColorBar *>(this), "renderUpdateBuffers");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CColorBar::renderUpdateBuffers();
	}
	void onUpdateBuffers_Wireframe() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CColorBar *>(this), "onUpdateBuffers_Wireframe");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CColorBar::onUpdateBuffers_Wireframe();
	}
	void onUpdateBuffers_Triangles() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CColorBar *>(this), "onUpdateBuffers_Triangles");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CColorBar::onUpdateBuffers_Triangles();
	}
	void freeOpenGLResources() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CColorBar *>(this), "freeOpenGLResources");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CColorBar::freeOpenGLResources();
	}
	struct mrpt::math::TBoundingBox_<float> internalBoundingBoxLocal() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CColorBar *>(this), "internalBoundingBoxLocal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TBoundingBox_<float>>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TBoundingBox_<float>> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TBoundingBox_<float>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TBoundingBox_<float>>(std::move(o));
		}
		return CColorBar::internalBoundingBoxLocal();
	}
	class mrpt::opengl::CRenderizable & setColorA_u8(const unsigned char a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CColorBar *>(this), "setColorA_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CColorBar *>(this), "setColor_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CColorBar *>(this), "cullElegible");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CColorBar *>(this), "toYAMLMap");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CColorBar *>(this), "isCompositeObject");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CColorBar *>(this), "traceRay");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CColorBar *>(this), "getLocalRepresentativePoint");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CColorBar *>(this), "initializeTextures");
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

// mrpt::opengl::CDisk file:mrpt/opengl/CDisk.h line:24
struct PyCallBack_mrpt_opengl_CDisk : public mrpt::opengl::CDisk {
	using mrpt::opengl::CDisk::CDisk;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CDisk *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CDisk::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CDisk *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CDisk::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CDisk *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CDisk::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CDisk *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CDisk::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CDisk *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CDisk::serializeFrom(a0, a1);
	}
	void onUpdateBuffers_Triangles() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CDisk *>(this), "onUpdateBuffers_Triangles");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CDisk::onUpdateBuffers_Triangles();
	}
	struct mrpt::math::TBoundingBox_<float> internalBoundingBoxLocal() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CDisk *>(this), "internalBoundingBoxLocal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TBoundingBox_<float>>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TBoundingBox_<float>> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TBoundingBox_<float>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TBoundingBox_<float>>(std::move(o));
		}
		return CDisk::internalBoundingBoxLocal();
	}
	bool traceRay(const class mrpt::poses::CPose3D & a0, double & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CDisk *>(this), "traceRay");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CDisk::traceRay(a0, a1);
	}
	void renderUpdateBuffers() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CDisk *>(this), "renderUpdateBuffers");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CDisk *>(this), "freeOpenGLResources");
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
	class mrpt::opengl::CRenderizable & setColorA_u8(const unsigned char a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CDisk *>(this), "setColorA_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CDisk *>(this), "setColor_u8");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CDisk *>(this), "cullElegible");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CDisk *>(this), "toYAMLMap");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CDisk *>(this), "isCompositeObject");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CDisk *>(this), "getLocalRepresentativePoint");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CDisk *>(this), "initializeTextures");
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

void bind_mrpt_opengl_CAxis(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::opengl::CAxis file:mrpt/opengl/CAxis.h line:24
		pybind11::class_<mrpt::opengl::CAxis, std::shared_ptr<mrpt::opengl::CAxis>, PyCallBack_mrpt_opengl_CAxis, mrpt::opengl::CRenderizableShaderWireFrame> cl(M("mrpt::opengl"), "CAxis", "Draw a 3D world axis, with coordinate marks at some regular interval\n  \n\n opengl::Scene\n\n  ![mrpt::opengl::CAxis](preview_CAxis.png)\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::CAxis(); }, [](){ return new PyCallBack_mrpt_opengl_CAxis(); } ), "doc");
		cl.def( pybind11::init( [](float const & a0){ return new mrpt::opengl::CAxis(a0); }, [](float const & a0){ return new PyCallBack_mrpt_opengl_CAxis(a0); } ), "doc");
		cl.def( pybind11::init( [](float const & a0, float const & a1){ return new mrpt::opengl::CAxis(a0, a1); }, [](float const & a0, float const & a1){ return new PyCallBack_mrpt_opengl_CAxis(a0, a1); } ), "doc");
		cl.def( pybind11::init( [](float const & a0, float const & a1, float const & a2){ return new mrpt::opengl::CAxis(a0, a1, a2); }, [](float const & a0, float const & a1, float const & a2){ return new PyCallBack_mrpt_opengl_CAxis(a0, a1, a2); } ), "doc");
		cl.def( pybind11::init( [](float const & a0, float const & a1, float const & a2, float const & a3){ return new mrpt::opengl::CAxis(a0, a1, a2, a3); }, [](float const & a0, float const & a1, float const & a2, float const & a3){ return new PyCallBack_mrpt_opengl_CAxis(a0, a1, a2, a3); } ), "doc");
		cl.def( pybind11::init( [](float const & a0, float const & a1, float const & a2, float const & a3, float const & a4){ return new mrpt::opengl::CAxis(a0, a1, a2, a3, a4); }, [](float const & a0, float const & a1, float const & a2, float const & a3, float const & a4){ return new PyCallBack_mrpt_opengl_CAxis(a0, a1, a2, a3, a4); } ), "doc");
		cl.def( pybind11::init( [](float const & a0, float const & a1, float const & a2, float const & a3, float const & a4, float const & a5){ return new mrpt::opengl::CAxis(a0, a1, a2, a3, a4, a5); }, [](float const & a0, float const & a1, float const & a2, float const & a3, float const & a4, float const & a5){ return new PyCallBack_mrpt_opengl_CAxis(a0, a1, a2, a3, a4, a5); } ), "doc");
		cl.def( pybind11::init( [](float const & a0, float const & a1, float const & a2, float const & a3, float const & a4, float const & a5, float const & a6){ return new mrpt::opengl::CAxis(a0, a1, a2, a3, a4, a5, a6); }, [](float const & a0, float const & a1, float const & a2, float const & a3, float const & a4, float const & a5, float const & a6){ return new PyCallBack_mrpt_opengl_CAxis(a0, a1, a2, a3, a4, a5, a6); } ), "doc");
		cl.def( pybind11::init( [](float const & a0, float const & a1, float const & a2, float const & a3, float const & a4, float const & a5, float const & a6, float const & a7){ return new mrpt::opengl::CAxis(a0, a1, a2, a3, a4, a5, a6, a7); }, [](float const & a0, float const & a1, float const & a2, float const & a3, float const & a4, float const & a5, float const & a6, float const & a7){ return new PyCallBack_mrpt_opengl_CAxis(a0, a1, a2, a3, a4, a5, a6, a7); } ), "doc");
		cl.def( pybind11::init<float, float, float, float, float, float, float, float, bool>(), pybind11::arg("xmin"), pybind11::arg("ymin"), pybind11::arg("zmin"), pybind11::arg("xmax"), pybind11::arg("ymax"), pybind11::arg("zmax"), pybind11::arg("frecuency"), pybind11::arg("lineWidth"), pybind11::arg("marks") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_opengl_CAxis const &o){ return new PyCallBack_mrpt_opengl_CAxis(o); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::CAxis const &o){ return new mrpt::opengl::CAxis(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CAxis::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CAxis::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CAxis::*)() const) &mrpt::opengl::CAxis::GetRuntimeClass, "C++: mrpt::opengl::CAxis::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::opengl::CAxis::*)() const) &mrpt::opengl::CAxis::clone, "C++: mrpt::opengl::CAxis::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::opengl::CAxis::CreateObject, "C++: mrpt::opengl::CAxis::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("onUpdateBuffers_Wireframe", (void (mrpt::opengl::CAxis::*)()) &mrpt::opengl::CAxis::onUpdateBuffers_Wireframe, "@{ \n\nC++: mrpt::opengl::CAxis::onUpdateBuffers_Wireframe() --> void");
		cl.def("isCompositeObject", (bool (mrpt::opengl::CAxis::*)() const) &mrpt::opengl::CAxis::isCompositeObject, "C++: mrpt::opengl::CAxis::isCompositeObject() const --> bool");
		cl.def("setAxisLimits", (void (mrpt::opengl::CAxis::*)(float, float, float, float, float, float)) &mrpt::opengl::CAxis::setAxisLimits, "C++: mrpt::opengl::CAxis::setAxisLimits(float, float, float, float, float, float) --> void", pybind11::arg("xmin"), pybind11::arg("ymin"), pybind11::arg("zmin"), pybind11::arg("xmax"), pybind11::arg("ymax"), pybind11::arg("zmax"));
		cl.def("setFrequency", (void (mrpt::opengl::CAxis::*)(float)) &mrpt::opengl::CAxis::setFrequency, "Changes the frequency of the \"ticks\" \n\nC++: mrpt::opengl::CAxis::setFrequency(float) --> void", pybind11::arg("f"));
		cl.def("getFrequency", (float (mrpt::opengl::CAxis::*)() const) &mrpt::opengl::CAxis::getFrequency, "C++: mrpt::opengl::CAxis::getFrequency() const --> float");
		cl.def("setTextScale", (void (mrpt::opengl::CAxis::*)(float)) &mrpt::opengl::CAxis::setTextScale, "Changes the size of text labels (default:0.25) \n\nC++: mrpt::opengl::CAxis::setTextScale(float) --> void", pybind11::arg("f"));
		cl.def("getTextScale", (float (mrpt::opengl::CAxis::*)() const) &mrpt::opengl::CAxis::getTextScale, "C++: mrpt::opengl::CAxis::getTextScale() const --> float");
		cl.def("setTextLabelOrientation", (void (mrpt::opengl::CAxis::*)(int, float, float, float)) &mrpt::opengl::CAxis::setTextLabelOrientation, "axis: {0,1,2}=>{X,Y,Z} \n\nC++: mrpt::opengl::CAxis::setTextLabelOrientation(int, float, float, float) --> void", pybind11::arg("axis"), pybind11::arg("yaw_deg"), pybind11::arg("pitch_deg"), pybind11::arg("roll_deg"));
		cl.def("getTextLabelOrientation", (void (mrpt::opengl::CAxis::*)(int, float &, float &, float &) const) &mrpt::opengl::CAxis::getTextLabelOrientation, "axis: {0,1,2}=>{X,Y,Z} \n\nC++: mrpt::opengl::CAxis::getTextLabelOrientation(int, float &, float &, float &) const --> void", pybind11::arg("axis"), pybind11::arg("yaw_deg"), pybind11::arg("pitch_deg"), pybind11::arg("roll_deg"));
		cl.def("enableTickMarks", [](mrpt::opengl::CAxis &o) -> void { return o.enableTickMarks(); }, "");
		cl.def("enableTickMarks", (void (mrpt::opengl::CAxis::*)(bool)) &mrpt::opengl::CAxis::enableTickMarks, "C++: mrpt::opengl::CAxis::enableTickMarks(bool) --> void", pybind11::arg("v"));
		cl.def("enableTickMarks", (void (mrpt::opengl::CAxis::*)(bool, bool, bool)) &mrpt::opengl::CAxis::enableTickMarks, "C++: mrpt::opengl::CAxis::enableTickMarks(bool, bool, bool) --> void", pybind11::arg("show_x"), pybind11::arg("show_y"), pybind11::arg("show_z"));
		cl.def("setTickMarksLength", (void (mrpt::opengl::CAxis::*)(float)) &mrpt::opengl::CAxis::setTickMarksLength, "As a ratio of \"marks frequency\" (default: 0.05) \n\nC++: mrpt::opengl::CAxis::setTickMarksLength(float) --> void", pybind11::arg("len"));
		cl.def("getTickMarksLength", (float (mrpt::opengl::CAxis::*)(float)) &mrpt::opengl::CAxis::getTickMarksLength, "C++: mrpt::opengl::CAxis::getTickMarksLength(float) --> float", pybind11::arg("len"));
		cl.def("internalBoundingBoxLocal", (struct mrpt::math::TBoundingBox_<float> (mrpt::opengl::CAxis::*)() const) &mrpt::opengl::CAxis::internalBoundingBoxLocal, "C++: mrpt::opengl::CAxis::internalBoundingBoxLocal() const --> struct mrpt::math::TBoundingBox_<float>");
		cl.def("assign", (class mrpt::opengl::CAxis & (mrpt::opengl::CAxis::*)(const class mrpt::opengl::CAxis &)) &mrpt::opengl::CAxis::operator=, "C++: mrpt::opengl::CAxis::operator=(const class mrpt::opengl::CAxis &) --> class mrpt::opengl::CAxis &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::opengl::CColorBar file:mrpt/opengl/CColorBar.h line:28
		pybind11::class_<mrpt::opengl::CColorBar, std::shared_ptr<mrpt::opengl::CColorBar>, PyCallBack_mrpt_opengl_CColorBar, mrpt::opengl::CRenderizableShaderTriangles, mrpt::opengl::CRenderizableShaderWireFrame> cl(M("mrpt::opengl"), "CColorBar", "A colorbar indicator. This class renders a colorbar as a 3D object, in the\n XY plane.\n For an overlay indicator that can be easily added to any display, see\n Scene::addColorBar()\n\n ![mrpt::opengl::CColorBar](preview_CColorBar.png)\n\n \n opengl::Scene,opengl::CRenderizable, Scene::addColorBar()\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::CColorBar(); }, [](){ return new PyCallBack_mrpt_opengl_CColorBar(); } ), "doc");
		cl.def( pybind11::init( [](const enum mrpt::img::TColormap & a0){ return new mrpt::opengl::CColorBar(a0); }, [](const enum mrpt::img::TColormap & a0){ return new PyCallBack_mrpt_opengl_CColorBar(a0); } ), "doc");
		cl.def( pybind11::init( [](const enum mrpt::img::TColormap & a0, double const & a1){ return new mrpt::opengl::CColorBar(a0, a1); }, [](const enum mrpt::img::TColormap & a0, double const & a1){ return new PyCallBack_mrpt_opengl_CColorBar(a0, a1); } ), "doc");
		cl.def( pybind11::init( [](const enum mrpt::img::TColormap & a0, double const & a1, double const & a2){ return new mrpt::opengl::CColorBar(a0, a1, a2); }, [](const enum mrpt::img::TColormap & a0, double const & a1, double const & a2){ return new PyCallBack_mrpt_opengl_CColorBar(a0, a1, a2); } ), "doc");
		cl.def( pybind11::init( [](const enum mrpt::img::TColormap & a0, double const & a1, double const & a2, float const & a3){ return new mrpt::opengl::CColorBar(a0, a1, a2, a3); }, [](const enum mrpt::img::TColormap & a0, double const & a1, double const & a2, float const & a3){ return new PyCallBack_mrpt_opengl_CColorBar(a0, a1, a2, a3); } ), "doc");
		cl.def( pybind11::init( [](const enum mrpt::img::TColormap & a0, double const & a1, double const & a2, float const & a3, float const & a4){ return new mrpt::opengl::CColorBar(a0, a1, a2, a3, a4); }, [](const enum mrpt::img::TColormap & a0, double const & a1, double const & a2, float const & a3, float const & a4){ return new PyCallBack_mrpt_opengl_CColorBar(a0, a1, a2, a3, a4); } ), "doc");
		cl.def( pybind11::init( [](const enum mrpt::img::TColormap & a0, double const & a1, double const & a2, float const & a3, float const & a4, float const & a5){ return new mrpt::opengl::CColorBar(a0, a1, a2, a3, a4, a5); }, [](const enum mrpt::img::TColormap & a0, double const & a1, double const & a2, float const & a3, float const & a4, float const & a5){ return new PyCallBack_mrpt_opengl_CColorBar(a0, a1, a2, a3, a4, a5); } ), "doc");
		cl.def( pybind11::init( [](const enum mrpt::img::TColormap & a0, double const & a1, double const & a2, float const & a3, float const & a4, float const & a5, float const & a6){ return new mrpt::opengl::CColorBar(a0, a1, a2, a3, a4, a5, a6); }, [](const enum mrpt::img::TColormap & a0, double const & a1, double const & a2, float const & a3, float const & a4, float const & a5, float const & a6){ return new PyCallBack_mrpt_opengl_CColorBar(a0, a1, a2, a3, a4, a5, a6); } ), "doc");
		cl.def( pybind11::init( [](const enum mrpt::img::TColormap & a0, double const & a1, double const & a2, float const & a3, float const & a4, float const & a5, float const & a6, const std::string & a7){ return new mrpt::opengl::CColorBar(a0, a1, a2, a3, a4, a5, a6, a7); }, [](const enum mrpt::img::TColormap & a0, double const & a1, double const & a2, float const & a3, float const & a4, float const & a5, float const & a6, const std::string & a7){ return new PyCallBack_mrpt_opengl_CColorBar(a0, a1, a2, a3, a4, a5, a6, a7); } ), "doc");
		cl.def( pybind11::init<const enum mrpt::img::TColormap, double, double, float, float, float, float, const std::string &, float>(), pybind11::arg("colormap"), pybind11::arg("width"), pybind11::arg("height"), pybind11::arg("min_col"), pybind11::arg("max_col"), pybind11::arg("min_value"), pybind11::arg("max_value"), pybind11::arg("label_format"), pybind11::arg("label_font_size") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_opengl_CColorBar const &o){ return new PyCallBack_mrpt_opengl_CColorBar(o); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::CColorBar const &o){ return new mrpt::opengl::CColorBar(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CColorBar::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CColorBar::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CColorBar::*)() const) &mrpt::opengl::CColorBar::GetRuntimeClass, "C++: mrpt::opengl::CColorBar::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::opengl::CColorBar::*)() const) &mrpt::opengl::CColorBar::clone, "C++: mrpt::opengl::CColorBar::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::opengl::CColorBar::CreateObject, "C++: mrpt::opengl::CColorBar::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("renderUpdateBuffers", (void (mrpt::opengl::CColorBar::*)() const) &mrpt::opengl::CColorBar::renderUpdateBuffers, "C++: mrpt::opengl::CColorBar::renderUpdateBuffers() const --> void");
		cl.def("onUpdateBuffers_Wireframe", (void (mrpt::opengl::CColorBar::*)()) &mrpt::opengl::CColorBar::onUpdateBuffers_Wireframe, "C++: mrpt::opengl::CColorBar::onUpdateBuffers_Wireframe() --> void");
		cl.def("onUpdateBuffers_Triangles", (void (mrpt::opengl::CColorBar::*)()) &mrpt::opengl::CColorBar::onUpdateBuffers_Triangles, "C++: mrpt::opengl::CColorBar::onUpdateBuffers_Triangles() --> void");
		cl.def("onUpdateBuffers_all", (void (mrpt::opengl::CColorBar::*)()) &mrpt::opengl::CColorBar::onUpdateBuffers_all, "C++: mrpt::opengl::CColorBar::onUpdateBuffers_all() --> void");
		cl.def("freeOpenGLResources", (void (mrpt::opengl::CColorBar::*)()) &mrpt::opengl::CColorBar::freeOpenGLResources, "C++: mrpt::opengl::CColorBar::freeOpenGLResources() --> void");
		cl.def("internalBoundingBoxLocal", (struct mrpt::math::TBoundingBox_<float> (mrpt::opengl::CColorBar::*)() const) &mrpt::opengl::CColorBar::internalBoundingBoxLocal, "@} \n\nC++: mrpt::opengl::CColorBar::internalBoundingBoxLocal() const --> struct mrpt::math::TBoundingBox_<float>");
		cl.def("setColormap", (void (mrpt::opengl::CColorBar::*)(const enum mrpt::img::TColormap)) &mrpt::opengl::CColorBar::setColormap, "C++: mrpt::opengl::CColorBar::setColormap(const enum mrpt::img::TColormap) --> void", pybind11::arg("colormap"));
		cl.def("setColorAndValueLimits", (void (mrpt::opengl::CColorBar::*)(float, float, float, float)) &mrpt::opengl::CColorBar::setColorAndValueLimits, "C++: mrpt::opengl::CColorBar::setColorAndValueLimits(float, float, float, float) --> void", pybind11::arg("col_min"), pybind11::arg("col_max"), pybind11::arg("value_min"), pybind11::arg("value_max"));
		cl.def("assign", (class mrpt::opengl::CColorBar & (mrpt::opengl::CColorBar::*)(const class mrpt::opengl::CColorBar &)) &mrpt::opengl::CColorBar::operator=, "C++: mrpt::opengl::CColorBar::operator=(const class mrpt::opengl::CColorBar &) --> class mrpt::opengl::CColorBar &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::opengl::CDisk file:mrpt/opengl/CDisk.h line:24
		pybind11::class_<mrpt::opengl::CDisk, std::shared_ptr<mrpt::opengl::CDisk>, PyCallBack_mrpt_opengl_CDisk, mrpt::opengl::CRenderizableShaderTriangles> cl(M("mrpt::opengl"), "CDisk", "A planar disk in the XY plane.\n\n ![mrpt::opengl::CDisk](preview_CDisk.png)\n\n  \n opengl::Scene\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::CDisk(); }, [](){ return new PyCallBack_mrpt_opengl_CDisk(); } ) );
		cl.def( pybind11::init( [](float const & a0, float const & a1){ return new mrpt::opengl::CDisk(a0, a1); }, [](float const & a0, float const & a1){ return new PyCallBack_mrpt_opengl_CDisk(a0, a1); } ), "doc");
		cl.def( pybind11::init<float, float, uint32_t>(), pybind11::arg("rOut"), pybind11::arg("rIn"), pybind11::arg("slices") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_opengl_CDisk const &o){ return new PyCallBack_mrpt_opengl_CDisk(o); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::CDisk const &o){ return new mrpt::opengl::CDisk(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CDisk::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CDisk::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CDisk::*)() const) &mrpt::opengl::CDisk::GetRuntimeClass, "C++: mrpt::opengl::CDisk::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::opengl::CDisk::*)() const) &mrpt::opengl::CDisk::clone, "C++: mrpt::opengl::CDisk::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::opengl::CDisk::CreateObject, "C++: mrpt::opengl::CDisk::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("onUpdateBuffers_Triangles", (void (mrpt::opengl::CDisk::*)()) &mrpt::opengl::CDisk::onUpdateBuffers_Triangles, "@{ \n\nC++: mrpt::opengl::CDisk::onUpdateBuffers_Triangles() --> void");
		cl.def("setDiskRadius", [](mrpt::opengl::CDisk &o, float const & a0) -> void { return o.setDiskRadius(a0); }, "", pybind11::arg("outRadius"));
		cl.def("setDiskRadius", (void (mrpt::opengl::CDisk::*)(float, float)) &mrpt::opengl::CDisk::setDiskRadius, "@} \n\nC++: mrpt::opengl::CDisk::setDiskRadius(float, float) --> void", pybind11::arg("outRadius"), pybind11::arg("inRadius"));
		cl.def("getInRadius", (float (mrpt::opengl::CDisk::*)() const) &mrpt::opengl::CDisk::getInRadius, "C++: mrpt::opengl::CDisk::getInRadius() const --> float");
		cl.def("getOutRadius", (float (mrpt::opengl::CDisk::*)() const) &mrpt::opengl::CDisk::getOutRadius, "C++: mrpt::opengl::CDisk::getOutRadius() const --> float");
		cl.def("setSlicesCount", (void (mrpt::opengl::CDisk::*)(uint32_t)) &mrpt::opengl::CDisk::setSlicesCount, "Default=50 \n\nC++: mrpt::opengl::CDisk::setSlicesCount(uint32_t) --> void", pybind11::arg("N"));
		cl.def("internalBoundingBoxLocal", (struct mrpt::math::TBoundingBox_<float> (mrpt::opengl::CDisk::*)() const) &mrpt::opengl::CDisk::internalBoundingBoxLocal, "Evaluates the bounding box of this object (including possible children)\n in the coordinate frame of the object parent. \n\nC++: mrpt::opengl::CDisk::internalBoundingBoxLocal() const --> struct mrpt::math::TBoundingBox_<float>");
		cl.def("traceRay", (bool (mrpt::opengl::CDisk::*)(const class mrpt::poses::CPose3D &, double &) const) &mrpt::opengl::CDisk::traceRay, "Ray tracing\n\nC++: mrpt::opengl::CDisk::traceRay(const class mrpt::poses::CPose3D &, double &) const --> bool", pybind11::arg("o"), pybind11::arg("dist"));
		cl.def("assign", (class mrpt::opengl::CDisk & (mrpt::opengl::CDisk::*)(const class mrpt::opengl::CDisk &)) &mrpt::opengl::CDisk::operator=, "C++: mrpt::opengl::CDisk::operator=(const class mrpt::opengl::CDisk &) --> class mrpt::opengl::CDisk &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
