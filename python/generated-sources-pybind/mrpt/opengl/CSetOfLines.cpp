#include <any>
#include <ios>
#include <istream>
#include <iterator>
#include <memory>
#include <mrpt/containers/CommentPosition.h>
#include <mrpt/containers/YamlEmitOptions.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/img/TColor.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TPose3DQuat.h>
#include <mrpt/math/TSegment2D.h>
#include <mrpt/math/TSegment3D.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/opengl/TRenderMatrices.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPointPDF.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPose3DQuatPDF.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CMessage.h>
#include <mrpt/serialization/CSerializable.h>
#include <ostream>
#include <sstream> // __str__
#include <streambuf>
#include <string>
#include <string_view>
#include <typeinfo>
#include <utility>
#include <variant>

#include <functional>
#include <pybind11/pybind11.h>
#include <string>
#include <stl_binders.hpp>


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
	void enqueueForRenderRecursive(const struct mrpt::opengl::TRenderMatrices & a0, int & a1, bool a2, bool a3) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfLines *>(this), "enqueueForRenderRecursive");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRenderizable::enqueueForRenderRecursive(a0, a1, a2, a3);
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

// mrpt::opengl::CSetOfObjects file:mrpt/opengl/CSetOfObjects.h line:26
struct PyCallBack_mrpt_opengl_CSetOfObjects : public mrpt::opengl::CSetOfObjects {
	using mrpt::opengl::CSetOfObjects::CSetOfObjects;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfObjects *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CSetOfObjects::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfObjects *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CSetOfObjects::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfObjects *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CSetOfObjects::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfObjects *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSetOfObjects::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfObjects *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSetOfObjects::serializeFrom(a0, a1);
	}
	void renderUpdateBuffers() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfObjects *>(this), "renderUpdateBuffers");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSetOfObjects::renderUpdateBuffers();
	}
	bool isCompositeObject() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfObjects *>(this), "isCompositeObject");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CSetOfObjects::isCompositeObject();
	}
	void freeOpenGLResources() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfObjects *>(this), "freeOpenGLResources");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSetOfObjects::freeOpenGLResources();
	}
	void initializeTextures() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfObjects *>(this), "initializeTextures");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSetOfObjects::initializeTextures();
	}
	bool traceRay(const class mrpt::poses::CPose3D & a0, double & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfObjects *>(this), "traceRay");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CSetOfObjects::traceRay(a0, a1);
	}
	class mrpt::opengl::CRenderizable & setColor_u8(const struct mrpt::img::TColor & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfObjects *>(this), "setColor_u8");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::opengl::CRenderizable &>::value) {
				static pybind11::detail::override_caster_t<class mrpt::opengl::CRenderizable &> caster;
				return pybind11::detail::cast_ref<class mrpt::opengl::CRenderizable &>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::opengl::CRenderizable &>(std::move(o));
		}
		return CSetOfObjects::setColor_u8(a0);
	}
	class mrpt::opengl::CRenderizable & setColorA_u8(const unsigned char a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfObjects *>(this), "setColorA_u8");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::opengl::CRenderizable &>::value) {
				static pybind11::detail::override_caster_t<class mrpt::opengl::CRenderizable &> caster;
				return pybind11::detail::cast_ref<class mrpt::opengl::CRenderizable &>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::opengl::CRenderizable &>(std::move(o));
		}
		return CSetOfObjects::setColorA_u8(a0);
	}
	bool cullElegible() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfObjects *>(this), "cullElegible");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfObjects *>(this), "toYAMLMap");
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
	void enqueueForRenderRecursive(const struct mrpt::opengl::TRenderMatrices & a0, int & a1, bool a2, bool a3) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::opengl::CSetOfObjects *>(this), "enqueueForRenderRecursive");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRenderizable::enqueueForRenderRecursive(a0, a1, a2, a3);
	}
};

void bind_mrpt_opengl_CSetOfLines(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::opengl::CSetOfLines file:mrpt/opengl/CSetOfLines.h line:25
		pybind11::class_<mrpt::opengl::CSetOfLines, std::shared_ptr<mrpt::opengl::CSetOfLines>, PyCallBack_mrpt_opengl_CSetOfLines, mrpt::opengl::CRenderizableShaderWireFrame, mrpt::opengl::CRenderizableShaderPoints> cl(M("mrpt::opengl"), "CSetOfLines", "A set of independent lines (or segments), one line with its own start and\n end positions (X,Y,Z). Optionally, the vertices can be also shown as dots.\n\n ![mrpt::opengl::CSetOfLines](preview_CSetOfLines.png)\n\n \n opengl::Scene\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::CSetOfLines(); }, [](){ return new PyCallBack_mrpt_opengl_CSetOfLines(); } ) );
		cl.def( pybind11::init( [](const int & a0){ return new mrpt::opengl::CSetOfLines(a0); }, [](const int & a0){ return new PyCallBack_mrpt_opengl_CSetOfLines(a0); } ), "doc");
		cl.def( pybind11::init<const int &, bool>(), pybind11::arg("sgms"), pybind11::arg("antiAliasing") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_opengl_CSetOfLines const &o){ return new PyCallBack_mrpt_opengl_CSetOfLines(o); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::CSetOfLines const &o){ return new mrpt::opengl::CSetOfLines(o); } ) );
		cl.def_static("getClassName", (auto (*)()) &mrpt::opengl::CSetOfLines::getClassName, "C++: mrpt::opengl::CSetOfLines::getClassName() --> auto");
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CSetOfLines::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CSetOfLines::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CSetOfLines::*)() const) &mrpt::opengl::CSetOfLines::GetRuntimeClass, "C++: mrpt::opengl::CSetOfLines::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::opengl::CSetOfLines::*)() const) &mrpt::opengl::CSetOfLines::clone, "C++: mrpt::opengl::CSetOfLines::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::opengl::CSetOfLines::CreateObject, "C++: mrpt::opengl::CSetOfLines::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("renderUpdateBuffers", (void (mrpt::opengl::CSetOfLines::*)() const) &mrpt::opengl::CSetOfLines::renderUpdateBuffers, "C++: mrpt::opengl::CSetOfLines::renderUpdateBuffers() const --> void");
		cl.def("freeOpenGLResources", (void (mrpt::opengl::CSetOfLines::*)()) &mrpt::opengl::CSetOfLines::freeOpenGLResources, "C++: mrpt::opengl::CSetOfLines::freeOpenGLResources() --> void");
		cl.def("requiredShaders", (int (mrpt::opengl::CSetOfLines::*)() const) &mrpt::opengl::CSetOfLines::requiredShaders, "C++: mrpt::opengl::CSetOfLines::requiredShaders() const --> int");
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
		cl.def("begin", (int (mrpt::opengl::CSetOfLines::*)()) &mrpt::opengl::CSetOfLines::begin, "C++: mrpt::opengl::CSetOfLines::begin() --> int");
		cl.def("end", (int (mrpt::opengl::CSetOfLines::*)()) &mrpt::opengl::CSetOfLines::end, "C++: mrpt::opengl::CSetOfLines::end() --> int");
		cl.def("rbegin", (int (mrpt::opengl::CSetOfLines::*)() const) &mrpt::opengl::CSetOfLines::rbegin, "Beginning const reverse iterator (actually, accesses the end of the\n set).\n \n\n rend,begin,end\n\nC++: mrpt::opengl::CSetOfLines::rbegin() const --> int");
		cl.def("rend", (int (mrpt::opengl::CSetOfLines::*)() const) &mrpt::opengl::CSetOfLines::rend, "Ending const reverse iterator (actually, refers to the starting point of\n the set).\n \n\n rbegin,end,begin\n\nC++: mrpt::opengl::CSetOfLines::rend() const --> int");
		cl.def("enableAntiAliasing", [](mrpt::opengl::CSetOfLines &o) -> void { return o.enableAntiAliasing(); }, "");
		cl.def("enableAntiAliasing", (void (mrpt::opengl::CSetOfLines::*)(bool)) &mrpt::opengl::CSetOfLines::enableAntiAliasing, "C++: mrpt::opengl::CSetOfLines::enableAntiAliasing(bool) --> void", pybind11::arg("enable"));
		cl.def("isAntiAliasingEnabled", (bool (mrpt::opengl::CSetOfLines::*)() const) &mrpt::opengl::CSetOfLines::isAntiAliasingEnabled, "C++: mrpt::opengl::CSetOfLines::isAntiAliasingEnabled() const --> bool");
		cl.def("assign", (class mrpt::opengl::CSetOfLines & (mrpt::opengl::CSetOfLines::*)(const class mrpt::opengl::CSetOfLines &)) &mrpt::opengl::CSetOfLines::operator=, "C++: mrpt::opengl::CSetOfLines::operator=(const class mrpt::opengl::CSetOfLines &) --> class mrpt::opengl::CSetOfLines &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::opengl::CSetOfObjects file:mrpt/opengl/CSetOfObjects.h line:26
		pybind11::class_<mrpt::opengl::CSetOfObjects, std::shared_ptr<mrpt::opengl::CSetOfObjects>, PyCallBack_mrpt_opengl_CSetOfObjects, mrpt::opengl::CRenderizable> cl(M("mrpt::opengl"), "CSetOfObjects", "A set of object, which are referenced to the coordinates framework\n established in this object.\n It can be established a hierarchy of \"CSetOfObjects\", where the coordinates\n framework of each one will be referenced to the parent's one.\n The list of child objects is accessed directly as in the class Scene\n\n \n opengl::Scene\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::opengl::CSetOfObjects(); }, [](){ return new PyCallBack_mrpt_opengl_CSetOfObjects(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_opengl_CSetOfObjects const &o){ return new PyCallBack_mrpt_opengl_CSetOfObjects(o); } ) );
		cl.def( pybind11::init( [](mrpt::opengl::CSetOfObjects const &o){ return new mrpt::opengl::CSetOfObjects(o); } ) );
		cl.def_static("getClassName", (auto (*)()) &mrpt::opengl::CSetOfObjects::getClassName, "C++: mrpt::opengl::CSetOfObjects::getClassName() --> auto");
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::opengl::CSetOfObjects::GetRuntimeClassIdStatic, "C++: mrpt::opengl::CSetOfObjects::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::opengl::CSetOfObjects::*)() const) &mrpt::opengl::CSetOfObjects::GetRuntimeClass, "C++: mrpt::opengl::CSetOfObjects::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::opengl::CSetOfObjects::*)() const) &mrpt::opengl::CSetOfObjects::clone, "C++: mrpt::opengl::CSetOfObjects::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::opengl::CSetOfObjects::CreateObject, "C++: mrpt::opengl::CSetOfObjects::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("begin", (int (mrpt::opengl::CSetOfObjects::*)()) &mrpt::opengl::CSetOfObjects::begin, "C++: mrpt::opengl::CSetOfObjects::begin() --> int");
		cl.def("end", (int (mrpt::opengl::CSetOfObjects::*)()) &mrpt::opengl::CSetOfObjects::end, "C++: mrpt::opengl::CSetOfObjects::end() --> int");
		cl.def("insert", (void (mrpt::opengl::CSetOfObjects::*)(const class std::shared_ptr<class mrpt::opengl::CRenderizable> &)) &mrpt::opengl::CSetOfObjects::insert, "Insert a new object to the list.\n\nC++: mrpt::opengl::CSetOfObjects::insert(const class std::shared_ptr<class mrpt::opengl::CRenderizable> &) --> void", pybind11::arg("newObject"));
		cl.def("requiredShaders", (int (mrpt::opengl::CSetOfObjects::*)() const) &mrpt::opengl::CSetOfObjects::requiredShaders, "C++: mrpt::opengl::CSetOfObjects::requiredShaders() const --> int");
		cl.def("renderUpdateBuffers", (void (mrpt::opengl::CSetOfObjects::*)() const) &mrpt::opengl::CSetOfObjects::renderUpdateBuffers, "C++: mrpt::opengl::CSetOfObjects::renderUpdateBuffers() const --> void");
		cl.def("enqueueForRenderRecursive", (void (mrpt::opengl::CSetOfObjects::*)(const struct mrpt::opengl::TRenderMatrices &, int &, bool, bool) const) &mrpt::opengl::CSetOfObjects::enqueueForRenderRecursive, "C++: mrpt::opengl::CSetOfObjects::enqueueForRenderRecursive(const struct mrpt::opengl::TRenderMatrices &, int &, bool, bool) const --> void", pybind11::arg("state"), pybind11::arg("rq"), pybind11::arg("wholeInView"), pybind11::arg("is1stShadowMapPass"));
		cl.def("isCompositeObject", (bool (mrpt::opengl::CSetOfObjects::*)() const) &mrpt::opengl::CSetOfObjects::isCompositeObject, "C++: mrpt::opengl::CSetOfObjects::isCompositeObject() const --> bool");
		cl.def("freeOpenGLResources", (void (mrpt::opengl::CSetOfObjects::*)()) &mrpt::opengl::CSetOfObjects::freeOpenGLResources, "C++: mrpt::opengl::CSetOfObjects::freeOpenGLResources() --> void");
		cl.def("initializeTextures", (void (mrpt::opengl::CSetOfObjects::*)() const) &mrpt::opengl::CSetOfObjects::initializeTextures, "C++: mrpt::opengl::CSetOfObjects::initializeTextures() const --> void");
		cl.def("clear", (void (mrpt::opengl::CSetOfObjects::*)()) &mrpt::opengl::CSetOfObjects::clear, "Clear the list of objects in the scene, deleting objects' memory.\n\nC++: mrpt::opengl::CSetOfObjects::clear() --> void");
		cl.def("size", (size_t (mrpt::opengl::CSetOfObjects::*)()) &mrpt::opengl::CSetOfObjects::size, "Returns number of objects.  \n\nC++: mrpt::opengl::CSetOfObjects::size() --> size_t");
		cl.def("empty", (bool (mrpt::opengl::CSetOfObjects::*)() const) &mrpt::opengl::CSetOfObjects::empty, "Returns true if there are no objects.  \n\nC++: mrpt::opengl::CSetOfObjects::empty() const --> bool");
		cl.def("getByName", (class std::shared_ptr<class mrpt::opengl::CRenderizable> (mrpt::opengl::CSetOfObjects::*)(const std::string &)) &mrpt::opengl::CSetOfObjects::getByName, "Returns the first object with a given name, or a nullptr pointer if not\n found.\n\nC++: mrpt::opengl::CSetOfObjects::getByName(const std::string &) --> class std::shared_ptr<class mrpt::opengl::CRenderizable>", pybind11::arg("str"));
		cl.def("removeObject", (void (mrpt::opengl::CSetOfObjects::*)(const class std::shared_ptr<class mrpt::opengl::CRenderizable> &)) &mrpt::opengl::CSetOfObjects::removeObject, "Removes the given object from the scene (it also deletes the object to\n free its memory).\n\nC++: mrpt::opengl::CSetOfObjects::removeObject(const class std::shared_ptr<class mrpt::opengl::CRenderizable> &) --> void", pybind11::arg("obj"));
		cl.def("dumpListOfObjects", (void (mrpt::opengl::CSetOfObjects::*)(int &) const) &mrpt::opengl::CSetOfObjects::dumpListOfObjects, "Retrieves a list of all objects in text form\n \n\n Prefer asYAML() (since MRPT 2.1.3) \n\nC++: mrpt::opengl::CSetOfObjects::dumpListOfObjects(int &) const --> void", pybind11::arg("lst"));
		cl.def("asYAML", (class mrpt::containers::yaml (mrpt::opengl::CSetOfObjects::*)() const) &mrpt::opengl::CSetOfObjects::asYAML, "Prints all objects in human-readable YAML form.\n \n\n (New in MRPT 2.1.3) \n\nC++: mrpt::opengl::CSetOfObjects::asYAML() const --> class mrpt::containers::yaml");
		cl.def("traceRay", (bool (mrpt::opengl::CSetOfObjects::*)(const class mrpt::poses::CPose3D &, double &) const) &mrpt::opengl::CSetOfObjects::traceRay, "C++: mrpt::opengl::CSetOfObjects::traceRay(const class mrpt::poses::CPose3D &, double &) const --> bool", pybind11::arg("o"), pybind11::arg("dist"));
		cl.def("setColor_u8", (class mrpt::opengl::CRenderizable & (mrpt::opengl::CSetOfObjects::*)(const struct mrpt::img::TColor &)) &mrpt::opengl::CSetOfObjects::setColor_u8, "C++: mrpt::opengl::CSetOfObjects::setColor_u8(const struct mrpt::img::TColor &) --> class mrpt::opengl::CRenderizable &", pybind11::return_value_policy::automatic, pybind11::arg("c"));
		cl.def("setColorA_u8", (class mrpt::opengl::CRenderizable & (mrpt::opengl::CSetOfObjects::*)(const unsigned char)) &mrpt::opengl::CSetOfObjects::setColorA_u8, "C++: mrpt::opengl::CSetOfObjects::setColorA_u8(const unsigned char) --> class mrpt::opengl::CRenderizable &", pybind11::return_value_policy::automatic, pybind11::arg("a"));
		cl.def("contains", (bool (mrpt::opengl::CSetOfObjects::*)(const class std::shared_ptr<class mrpt::opengl::CRenderizable> &) const) &mrpt::opengl::CSetOfObjects::contains, "C++: mrpt::opengl::CSetOfObjects::contains(const class std::shared_ptr<class mrpt::opengl::CRenderizable> &) const --> bool", pybind11::arg("obj"));
		cl.def_static("posePDF2opengl", (class std::shared_ptr<class mrpt::opengl::CSetOfObjects> (*)(const class mrpt::poses::CPosePDF &)) &mrpt::opengl::CSetOfObjects::posePDF2opengl, "Returns a representation of a the PDF - this is just an auxiliary\n function, it's more natural to call\n    mrpt::poses::CPosePDF::getAs3DObject     \n\nC++: mrpt::opengl::CSetOfObjects::posePDF2opengl(const class mrpt::poses::CPosePDF &) --> class std::shared_ptr<class mrpt::opengl::CSetOfObjects>", pybind11::arg("o"));
		cl.def_static("posePDF2opengl", (class std::shared_ptr<class mrpt::opengl::CSetOfObjects> (*)(const class mrpt::poses::CPointPDF &)) &mrpt::opengl::CSetOfObjects::posePDF2opengl, "Returns a representation of a the PDF - this is just an auxiliary\n function, it's more natural to call\n    mrpt::poses::CPointPDF::getAs3DObject     \n\nC++: mrpt::opengl::CSetOfObjects::posePDF2opengl(const class mrpt::poses::CPointPDF &) --> class std::shared_ptr<class mrpt::opengl::CSetOfObjects>", pybind11::arg("o"));
		cl.def_static("posePDF2opengl", (class std::shared_ptr<class mrpt::opengl::CSetOfObjects> (*)(const class mrpt::poses::CPose3DPDF &)) &mrpt::opengl::CSetOfObjects::posePDF2opengl, "Returns a representation of a the PDF - this is just an auxiliary\n function, it's more natural to call\n    mrpt::poses::CPose3DPDF::getAs3DObject     \n\nC++: mrpt::opengl::CSetOfObjects::posePDF2opengl(const class mrpt::poses::CPose3DPDF &) --> class std::shared_ptr<class mrpt::opengl::CSetOfObjects>", pybind11::arg("o"));
		cl.def_static("posePDF2opengl", (class std::shared_ptr<class mrpt::opengl::CSetOfObjects> (*)(const class mrpt::poses::CPose3DQuatPDF &)) &mrpt::opengl::CSetOfObjects::posePDF2opengl, "Returns a representation of a the PDF - this is just an auxiliary\n function, it's more natural to call\n    mrpt::poses::CPose3DQuatPDF::getAs3DObject     \n\nC++: mrpt::opengl::CSetOfObjects::posePDF2opengl(const class mrpt::poses::CPose3DQuatPDF &) --> class std::shared_ptr<class mrpt::opengl::CSetOfObjects>", pybind11::arg("o"));
		cl.def("assign", (class mrpt::opengl::CSetOfObjects & (mrpt::opengl::CSetOfObjects::*)(const class mrpt::opengl::CSetOfObjects &)) &mrpt::opengl::CSetOfObjects::operator=, "C++: mrpt::opengl::CSetOfObjects::operator=(const class mrpt::opengl::CSetOfObjects &) --> class mrpt::opengl::CSetOfObjects &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
