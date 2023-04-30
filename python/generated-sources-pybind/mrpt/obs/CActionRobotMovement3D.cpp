#include <any>
#include <chrono>
#include <functional>
#include <ios>
#include <istream>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/containers/CommentPosition.h>
#include <mrpt/containers/YamlEmitOptions.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/Clock.h>
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
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation3DScene.h>
#include <mrpt/obs/CObservation6DFeatures.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CText.h>
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
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CMessage.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ostream>
#include <ratio>
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
#include <stl_binders.hpp>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

// mrpt::obs::CActionRobotMovement3D file:mrpt/obs/CActionRobotMovement3D.h line:27
struct PyCallBack_mrpt_obs_CActionRobotMovement3D : public mrpt::obs::CActionRobotMovement3D {
	using mrpt::obs::CActionRobotMovement3D::CActionRobotMovement3D;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CActionRobotMovement3D *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CActionRobotMovement3D::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CActionRobotMovement3D *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CActionRobotMovement3D::clone();
	}
	unsigned char serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CActionRobotMovement3D *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<unsigned char>::value) {
				static pybind11::detail::override_caster_t<unsigned char> caster;
				return pybind11::detail::cast_ref<unsigned char>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<unsigned char>(std::move(o));
		}
		return CActionRobotMovement3D::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CActionRobotMovement3D *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CActionRobotMovement3D::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, unsigned char a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CActionRobotMovement3D *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CActionRobotMovement3D::serializeFrom(a0, a1);
	}
};

// mrpt::obs::CObservation3DScene file:mrpt/obs/CObservation3DScene.h line:28
struct PyCallBack_mrpt_obs_CObservation3DScene : public mrpt::obs::CObservation3DScene {
	using mrpt::obs::CObservation3DScene::CObservation3DScene;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation3DScene *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CObservation3DScene::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation3DScene *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CObservation3DScene::clone();
	}
	unsigned char serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation3DScene *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<unsigned char>::value) {
				static pybind11::detail::override_caster_t<unsigned char> caster;
				return pybind11::detail::cast_ref<unsigned char>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<unsigned char>(std::move(o));
		}
		return CObservation3DScene::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation3DScene *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservation3DScene::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, unsigned char a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation3DScene *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservation3DScene::serializeFrom(a0, a1);
	}
	void getSensorPose(class mrpt::poses::CPose3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation3DScene *>(this), "getSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservation3DScene::getSensorPose(a0);
	}
	void setSensorPose(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation3DScene *>(this), "setSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservation3DScene::setSensorPose(a0);
	}
	void getVisualizationInto(class mrpt::opengl::CSetOfObjects & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation3DScene *>(this), "getVisualizationInto");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservation3DScene::getVisualizationInto(a0);
	}
	using _binder_ret_0 = struct std::chrono::time_point<class mrpt::Clock, struct std::chrono::duration<long, struct std::ratio<1, 10000000> > >;
	_binder_ret_0 getOriginalReceivedTimeStamp() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation3DScene *>(this), "getOriginalReceivedTimeStamp");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<_binder_ret_0>::value) {
				static pybind11::detail::override_caster_t<_binder_ret_0> caster;
				return pybind11::detail::cast_ref<_binder_ret_0>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<_binder_ret_0>(std::move(o));
		}
		return CObservation::getOriginalReceivedTimeStamp();
	}
	std::string asString() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation3DScene *>(this), "asString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CObservation::asString();
	}
	bool exportTxtSupported() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation3DScene *>(this), "exportTxtSupported");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CObservation::exportTxtSupported();
	}
	std::string exportTxtHeader() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation3DScene *>(this), "exportTxtHeader");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CObservation::exportTxtHeader();
	}
	std::string exportTxtDataRow() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation3DScene *>(this), "exportTxtDataRow");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CObservation::exportTxtDataRow();
	}
	void load() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation3DScene *>(this), "load");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservation::load();
	}
	void unload() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation3DScene *>(this), "unload");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservation::unload();
	}
};

// mrpt::obs::CObservation6DFeatures file:mrpt/obs/CObservation6DFeatures.h line:26
struct PyCallBack_mrpt_obs_CObservation6DFeatures : public mrpt::obs::CObservation6DFeatures {
	using mrpt::obs::CObservation6DFeatures::CObservation6DFeatures;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation6DFeatures *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CObservation6DFeatures::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation6DFeatures *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CObservation6DFeatures::clone();
	}
	unsigned char serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation6DFeatures *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<unsigned char>::value) {
				static pybind11::detail::override_caster_t<unsigned char> caster;
				return pybind11::detail::cast_ref<unsigned char>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<unsigned char>(std::move(o));
		}
		return CObservation6DFeatures::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation6DFeatures *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservation6DFeatures::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, unsigned char a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation6DFeatures *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservation6DFeatures::serializeFrom(a0, a1);
	}
	void getSensorPose(class mrpt::poses::CPose3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation6DFeatures *>(this), "getSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservation6DFeatures::getSensorPose(a0);
	}
	void setSensorPose(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation6DFeatures *>(this), "setSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservation6DFeatures::setSensorPose(a0);
	}
	using _binder_ret_0 = struct std::chrono::time_point<class mrpt::Clock, struct std::chrono::duration<long, struct std::ratio<1, 10000000> > >;
	_binder_ret_0 getOriginalReceivedTimeStamp() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation6DFeatures *>(this), "getOriginalReceivedTimeStamp");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<_binder_ret_0>::value) {
				static pybind11::detail::override_caster_t<_binder_ret_0> caster;
				return pybind11::detail::cast_ref<_binder_ret_0>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<_binder_ret_0>(std::move(o));
		}
		return CObservation::getOriginalReceivedTimeStamp();
	}
	std::string asString() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation6DFeatures *>(this), "asString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CObservation::asString();
	}
	bool exportTxtSupported() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation6DFeatures *>(this), "exportTxtSupported");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CObservation::exportTxtSupported();
	}
	std::string exportTxtHeader() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation6DFeatures *>(this), "exportTxtHeader");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CObservation::exportTxtHeader();
	}
	std::string exportTxtDataRow() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation6DFeatures *>(this), "exportTxtDataRow");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CObservation::exportTxtDataRow();
	}
	void load() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation6DFeatures *>(this), "load");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservation::load();
	}
	void unload() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation6DFeatures *>(this), "unload");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservation::unload();
	}
};

void bind_mrpt_obs_CActionRobotMovement3D(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::obs::CActionRobotMovement3D file:mrpt/obs/CActionRobotMovement3D.h line:27
		pybind11::class_<mrpt::obs::CActionRobotMovement3D, std::shared_ptr<mrpt::obs::CActionRobotMovement3D>, PyCallBack_mrpt_obs_CActionRobotMovement3D, mrpt::obs::CAction> cl(M("mrpt::obs"), "CActionRobotMovement3D", "Represents a probabilistic motion increment in SE(3).\n\n Odometry increments might be determined from visual odometry for full 3D, or\n from wheel encoders for 2D movements only.\n\n The implemented model for creating a SE(3) Gaussian from an odometry\n increment is based on  ballardini2012effective\n\n \n\n \n CAction, CActionRobotMovement3D,");
		cl.def( pybind11::init( [](){ return new mrpt::obs::CActionRobotMovement3D(); }, [](){ return new PyCallBack_mrpt_obs_CActionRobotMovement3D(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_CActionRobotMovement3D const &o){ return new PyCallBack_mrpt_obs_CActionRobotMovement3D(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::CActionRobotMovement3D const &o){ return new mrpt::obs::CActionRobotMovement3D(o); } ) );

		pybind11::enum_<mrpt::obs::CActionRobotMovement3D::TEstimationMethod>(cl, "TEstimationMethod", pybind11::arithmetic(), "A list of posible ways for estimating the content of a\n CActionRobotMovement3D object.")
			.value("emOdometry", mrpt::obs::CActionRobotMovement3D::emOdometry)
			.value("emVisualOdometry", mrpt::obs::CActionRobotMovement3D::emVisualOdometry)
			.export_values();


		pybind11::enum_<mrpt::obs::CActionRobotMovement3D::TDrawSampleMotionModel>(cl, "TDrawSampleMotionModel", pybind11::arithmetic(), "")
			.value("mmGaussian", mrpt::obs::CActionRobotMovement3D::mmGaussian)
			.value("mm6DOF", mrpt::obs::CActionRobotMovement3D::mm6DOF)
			.export_values();

		cl.def_readwrite("poseChange", &mrpt::obs::CActionRobotMovement3D::poseChange);
		cl.def_readwrite("rawOdometryIncrementReading", &mrpt::obs::CActionRobotMovement3D::rawOdometryIncrementReading);
		cl.def_readwrite("estimationMethod", &mrpt::obs::CActionRobotMovement3D::estimationMethod);
		cl.def_readwrite("motionModelConfiguration", &mrpt::obs::CActionRobotMovement3D::motionModelConfiguration);
		cl.def_readwrite("hasVelocities", &mrpt::obs::CActionRobotMovement3D::hasVelocities);
		cl.def_readwrite("velocities", &mrpt::obs::CActionRobotMovement3D::velocities);
		cl.def_static("getClassName", (class mrpt::typemeta::string_literal<33> (*)()) &mrpt::obs::CActionRobotMovement3D::getClassName, "C++: mrpt::obs::CActionRobotMovement3D::getClassName() --> class mrpt::typemeta::string_literal<33>");
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::obs::CActionRobotMovement3D::GetRuntimeClassIdStatic, "C++: mrpt::obs::CActionRobotMovement3D::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::obs::CActionRobotMovement3D::*)() const) &mrpt::obs::CActionRobotMovement3D::GetRuntimeClass, "C++: mrpt::obs::CActionRobotMovement3D::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::obs::CActionRobotMovement3D::*)() const) &mrpt::obs::CActionRobotMovement3D::clone, "C++: mrpt::obs::CActionRobotMovement3D::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::obs::CActionRobotMovement3D::CreateObject, "C++: mrpt::obs::CActionRobotMovement3D::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("computeFromOdometry", (void (mrpt::obs::CActionRobotMovement3D::*)(const class mrpt::poses::CPose3D &, const struct mrpt::obs::CActionRobotMovement3D::TMotionModelOptions &)) &mrpt::obs::CActionRobotMovement3D::computeFromOdometry, "Computes the PDF of the pose increment from an odometry reading and\n according to the given motion model (speed and encoder ticks information\n is not modified).\n According to the parameters in the passed struct, it will be called one\n the private sampling functions (see \"see also\" next).\n \n\n computeFromOdometry_model6DOF\n\nC++: mrpt::obs::CActionRobotMovement3D::computeFromOdometry(const class mrpt::poses::CPose3D &, const struct mrpt::obs::CActionRobotMovement3D::TMotionModelOptions &) --> void", pybind11::arg("odometryIncrement"), pybind11::arg("options"));
		cl.def("computeFromOdometry_model6DOF", (void (mrpt::obs::CActionRobotMovement3D::*)(const class mrpt::poses::CPose3D &, const struct mrpt::obs::CActionRobotMovement3D::TMotionModelOptions &)) &mrpt::obs::CActionRobotMovement3D::computeFromOdometry_model6DOF, "Computes the PDF of the pose increment from an odometry reading, using\n the motion model for 6 DOF.\n\n Based on:  ballardini2012effective\n\n \n computeFromOdometry\n\nC++: mrpt::obs::CActionRobotMovement3D::computeFromOdometry_model6DOF(const class mrpt::poses::CPose3D &, const struct mrpt::obs::CActionRobotMovement3D::TMotionModelOptions &) --> void", pybind11::arg("odometryIncrement"), pybind11::arg("o"));
		cl.def("assign", (class mrpt::obs::CActionRobotMovement3D & (mrpt::obs::CActionRobotMovement3D::*)(const class mrpt::obs::CActionRobotMovement3D &)) &mrpt::obs::CActionRobotMovement3D::operator=, "C++: mrpt::obs::CActionRobotMovement3D::operator=(const class mrpt::obs::CActionRobotMovement3D &) --> class mrpt::obs::CActionRobotMovement3D &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::obs::CActionRobotMovement3D::TMotionModelOptions file:mrpt/obs/CActionRobotMovement3D.h line:64
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::CActionRobotMovement3D::TMotionModelOptions, std::shared_ptr<mrpt::obs::CActionRobotMovement3D::TMotionModelOptions>> cl(enclosing_class, "TMotionModelOptions", "The parameter to be passed to \"computeFromOdometry\".\n See:  ballardini2012effective ");
			cl.def( pybind11::init( [](){ return new mrpt::obs::CActionRobotMovement3D::TMotionModelOptions(); } ) );
			cl.def( pybind11::init( [](mrpt::obs::CActionRobotMovement3D::TMotionModelOptions const &o){ return new mrpt::obs::CActionRobotMovement3D::TMotionModelOptions(o); } ) );
			cl.def_readwrite("modelSelection", &mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::modelSelection);
			cl.def_readwrite("mm6DOFModel", &mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::mm6DOFModel);
			cl.def("assign", (struct mrpt::obs::CActionRobotMovement3D::TMotionModelOptions & (mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::*)(const struct mrpt::obs::CActionRobotMovement3D::TMotionModelOptions &)) &mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::operator=, "C++: mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::operator=(const struct mrpt::obs::CActionRobotMovement3D::TMotionModelOptions &) --> struct mrpt::obs::CActionRobotMovement3D::TMotionModelOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));

			{ // mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel file:mrpt/obs/CActionRobotMovement3D.h line:71
				auto & enclosing_class = cl;
				pybind11::class_<mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel, std::shared_ptr<mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel>> cl(enclosing_class, "TOptions_6DOFModel", "");
				cl.def( pybind11::init( [](){ return new mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel(); } ) );
				cl.def( pybind11::init( [](mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel const &o){ return new mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel(o); } ) );
				cl.def_readwrite("nParticlesCount", &mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel::nParticlesCount);
				cl.def_readwrite("a1", &mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel::a1);
				cl.def_readwrite("a2", &mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel::a2);
				cl.def_readwrite("a3", &mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel::a3);
				cl.def_readwrite("a4", &mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel::a4);
				cl.def_readwrite("a5", &mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel::a5);
				cl.def_readwrite("a6", &mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel::a6);
				cl.def_readwrite("a7", &mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel::a7);
				cl.def_readwrite("a8", &mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel::a8);
				cl.def_readwrite("a9", &mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel::a9);
				cl.def_readwrite("a10", &mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel::a10);
				cl.def_readwrite("additional_std_XYZ", &mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel::additional_std_XYZ);
				cl.def_readwrite("additional_std_angle", &mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel::additional_std_angle);
				cl.def("assign", (struct mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel & (mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel::*)(const struct mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel &)) &mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel::operator=, "C++: mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel::operator=(const struct mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel &) --> struct mrpt::obs::CActionRobotMovement3D::TMotionModelOptions::TOptions_6DOFModel &", pybind11::return_value_policy::automatic, pybind11::arg(""));
			}

		}

	}
	{ // mrpt::obs::CObservation3DScene file:mrpt/obs/CObservation3DScene.h line:28
		pybind11::class_<mrpt::obs::CObservation3DScene, std::shared_ptr<mrpt::obs::CObservation3DScene>, PyCallBack_mrpt_obs_CObservation3DScene, mrpt::obs::CObservation, mrpt::opengl::Visualizable> cl(M("mrpt::obs"), "CObservation3DScene", "Not a real sensor observation, it stores a 3D scene which can be used for\n debugging or any other logging purposes.\n If stored in a .rawlog file, RawLogViewer will show the contents of\n the scene's main viewport when selecting it on the tree view.\n\n \n CObservation\n \n\n\n \n (New in MRPT 2.3.1)");
		cl.def( pybind11::init( [](){ return new mrpt::obs::CObservation3DScene(); }, [](){ return new PyCallBack_mrpt_obs_CObservation3DScene(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_CObservation3DScene const &o){ return new PyCallBack_mrpt_obs_CObservation3DScene(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::CObservation3DScene const &o){ return new mrpt::obs::CObservation3DScene(o); } ) );
		cl.def_readwrite("scene", &mrpt::obs::CObservation3DScene::scene);
		cl.def_readwrite("sensorPose", &mrpt::obs::CObservation3DScene::sensorPose);
		cl.def_static("getClassName", (class mrpt::typemeta::string_literal<30> (*)()) &mrpt::obs::CObservation3DScene::getClassName, "C++: mrpt::obs::CObservation3DScene::getClassName() --> class mrpt::typemeta::string_literal<30>");
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::obs::CObservation3DScene::GetRuntimeClassIdStatic, "C++: mrpt::obs::CObservation3DScene::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::obs::CObservation3DScene::*)() const) &mrpt::obs::CObservation3DScene::GetRuntimeClass, "C++: mrpt::obs::CObservation3DScene::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::obs::CObservation3DScene::*)() const) &mrpt::obs::CObservation3DScene::clone, "C++: mrpt::obs::CObservation3DScene::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::obs::CObservation3DScene::CreateObject, "C++: mrpt::obs::CObservation3DScene::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("getSensorPose", (void (mrpt::obs::CObservation3DScene::*)(class mrpt::poses::CPose3D &) const) &mrpt::obs::CObservation3DScene::getSensorPose, "C++: mrpt::obs::CObservation3DScene::getSensorPose(class mrpt::poses::CPose3D &) const --> void", pybind11::arg("out_sensorPose"));
		cl.def("setSensorPose", (void (mrpt::obs::CObservation3DScene::*)(const class mrpt::poses::CPose3D &)) &mrpt::obs::CObservation3DScene::setSensorPose, "C++: mrpt::obs::CObservation3DScene::setSensorPose(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newSensorPose"));
		cl.def("getVisualizationInto", (void (mrpt::obs::CObservation3DScene::*)(class mrpt::opengl::CSetOfObjects &) const) &mrpt::obs::CObservation3DScene::getVisualizationInto, "C++: mrpt::obs::CObservation3DScene::getVisualizationInto(class mrpt::opengl::CSetOfObjects &) const --> void", pybind11::arg("o"));
		cl.def("assign", (class mrpt::obs::CObservation3DScene & (mrpt::obs::CObservation3DScene::*)(const class mrpt::obs::CObservation3DScene &)) &mrpt::obs::CObservation3DScene::operator=, "C++: mrpt::obs::CObservation3DScene::operator=(const class mrpt::obs::CObservation3DScene &) --> class mrpt::obs::CObservation3DScene &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::obs::CObservation6DFeatures file:mrpt/obs/CObservation6DFeatures.h line:26
		pybind11::class_<mrpt::obs::CObservation6DFeatures, std::shared_ptr<mrpt::obs::CObservation6DFeatures>, PyCallBack_mrpt_obs_CObservation6DFeatures, mrpt::obs::CObservation> cl(M("mrpt::obs"), "CObservation6DFeatures", "An observation of one or more \"features\" or \"objects\", possibly identified\n with a unique ID, whose relative SE(3) pose is observed with respect to the\n sensor.\n The list of features is stored in \n \n\n CObservation\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::obs::CObservation6DFeatures(); }, [](){ return new PyCallBack_mrpt_obs_CObservation6DFeatures(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_CObservation6DFeatures const &o){ return new PyCallBack_mrpt_obs_CObservation6DFeatures(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::CObservation6DFeatures const &o){ return new mrpt::obs::CObservation6DFeatures(o); } ) );
		cl.def_readwrite("minSensorDistance", &mrpt::obs::CObservation6DFeatures::minSensorDistance);
		cl.def_readwrite("maxSensorDistance", &mrpt::obs::CObservation6DFeatures::maxSensorDistance);
		cl.def_readwrite("sensedFeatures", &mrpt::obs::CObservation6DFeatures::sensedFeatures);
		cl.def_readwrite("sensorPose", &mrpt::obs::CObservation6DFeatures::sensorPose);
		cl.def_static("getClassName", (class mrpt::typemeta::string_literal<33> (*)()) &mrpt::obs::CObservation6DFeatures::getClassName, "C++: mrpt::obs::CObservation6DFeatures::getClassName() --> class mrpt::typemeta::string_literal<33>");
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::obs::CObservation6DFeatures::GetRuntimeClassIdStatic, "C++: mrpt::obs::CObservation6DFeatures::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::obs::CObservation6DFeatures::*)() const) &mrpt::obs::CObservation6DFeatures::GetRuntimeClass, "C++: mrpt::obs::CObservation6DFeatures::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::obs::CObservation6DFeatures::*)() const) &mrpt::obs::CObservation6DFeatures::clone, "C++: mrpt::obs::CObservation6DFeatures::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::obs::CObservation6DFeatures::CreateObject, "C++: mrpt::obs::CObservation6DFeatures::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("getSensorPose", (void (mrpt::obs::CObservation6DFeatures::*)(class mrpt::poses::CPose3D &) const) &mrpt::obs::CObservation6DFeatures::getSensorPose, "C++: mrpt::obs::CObservation6DFeatures::getSensorPose(class mrpt::poses::CPose3D &) const --> void", pybind11::arg("out_sensorPose"));
		cl.def("setSensorPose", (void (mrpt::obs::CObservation6DFeatures::*)(const class mrpt::poses::CPose3D &)) &mrpt::obs::CObservation6DFeatures::setSensorPose, "C++: mrpt::obs::CObservation6DFeatures::setSensorPose(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newSensorPose"));
		cl.def("assign", (class mrpt::obs::CObservation6DFeatures & (mrpt::obs::CObservation6DFeatures::*)(const class mrpt::obs::CObservation6DFeatures &)) &mrpt::obs::CObservation6DFeatures::operator=, "C++: mrpt::obs::CObservation6DFeatures::operator=(const class mrpt::obs::CObservation6DFeatures &) --> class mrpt::obs::CObservation6DFeatures &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::obs::CObservation6DFeatures::TMeasurement file:mrpt/obs/CObservation6DFeatures.h line:37
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::CObservation6DFeatures::TMeasurement, std::shared_ptr<mrpt::obs::CObservation6DFeatures::TMeasurement>> cl(enclosing_class, "TMeasurement", "Each one of the measurements ");
			cl.def( pybind11::init( [](){ return new mrpt::obs::CObservation6DFeatures::TMeasurement(); } ) );
			cl.def_readwrite("pose", &mrpt::obs::CObservation6DFeatures::TMeasurement::pose);
			cl.def_readwrite("id", &mrpt::obs::CObservation6DFeatures::TMeasurement::id);
			cl.def_readwrite("inf_matrix", &mrpt::obs::CObservation6DFeatures::TMeasurement::inf_matrix);
		}

	}
}
