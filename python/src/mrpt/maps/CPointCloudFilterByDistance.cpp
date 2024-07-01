#include <any>
#include <chrono>
#include <deque>
#include <functional>
#include <ios>
#include <istream>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/containers/CommentPosition.h>
#include <mrpt/containers/YamlEmitOptions.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/Clock.h>
#include <mrpt/core/aligned_allocator.h>
#include <mrpt/core/safe_pointers.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/img/TColor.h>
#include <mrpt/img/color_maps.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CPointCloudFilterBase.h>
#include <mrpt/maps/CPointCloudFilterByDistance.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/TMetricMapInitializer.h>
#include <mrpt/maps/metric_map_types.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CPolygon.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TPose3DQuat.h>
#include <mrpt/math/TTwist3D.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/T2DScanProperties.h>
#include <mrpt/obs/T3DPointsTo2DScanParams.h>
#include <mrpt/obs/TRangeImageFilter.h>
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
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/tfest/TMatchingPair.h>
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
#include <pybind11/stl.h>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

// mrpt::maps::CPointCloudFilterByDistance file:mrpt/maps/CPointCloudFilterByDistance.h line:27
struct PyCallBack_mrpt_maps_CPointCloudFilterByDistance : public mrpt::maps::CPointCloudFilterByDistance {
	using mrpt::maps::CPointCloudFilterByDistance::CPointCloudFilterByDistance;

	void filter(class mrpt::maps::CPointsMap * a0, const mrpt::Clock::time_point a1, const class mrpt::poses::CPose3D & a2, struct mrpt::maps::CPointCloudFilterBase::TExtraFilterParams * a3) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointCloudFilterByDistance *>(this), "filter");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointCloudFilterByDistance::filter(a0, a1, a2, a3);
	}
};

// mrpt::maps::CPointCloudFilterByDistance::TOptions file:mrpt/maps/CPointCloudFilterByDistance.h line:44
struct PyCallBack_mrpt_maps_CPointCloudFilterByDistance_TOptions : public mrpt::maps::CPointCloudFilterByDistance::TOptions {
	using mrpt::maps::CPointCloudFilterByDistance::TOptions::TOptions;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointCloudFilterByDistance::TOptions *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TOptions::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointCloudFilterByDistance::TOptions *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TOptions::saveToConfigFile(a0, a1);
	}
};

// mrpt::maps::CPointsMapXYZI file:mrpt/maps/CPointsMapXYZI.h line:24
struct PyCallBack_mrpt_maps_CPointsMapXYZI : public mrpt::maps::CPointsMapXYZI {
	using mrpt::maps::CPointsMapXYZI::CPointsMapXYZI;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPointsMapXYZI::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CPointsMapXYZI::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CPointsMapXYZI::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointsMapXYZI::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointsMapXYZI::serializeFrom(a0, a1);
	}
	void reserve(size_t a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "reserve");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointsMapXYZI::reserve(a0);
	}
	void resize(size_t a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "resize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointsMapXYZI::resize(a0);
	}
	void setSize(size_t a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "setSize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointsMapXYZI::setSize(a0);
	}
	void insertPointFast(float a0, float a1, float a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "insertPointFast");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointsMapXYZI::insertPointFast(a0, a1, a2);
	}
	void addFrom_classSpecific(const class mrpt::maps::CPointsMap & a0, size_t a1, const bool a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "addFrom_classSpecific");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointsMapXYZI::addFrom_classSpecific(a0, a1, a2);
	}
	void setPointRGB(size_t a0, float a1, float a2, float a3, float a4, float a5, float a6) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "setPointRGB");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4, a5, a6);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointsMapXYZI::setPointRGB(a0, a1, a2, a3, a4, a5, a6);
	}
	void insertPointRGB(float a0, float a1, float a2, float a3, float a4, float a5) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "insertPointRGB");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4, a5);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointsMapXYZI::insertPointRGB(a0, a1, a2, a3, a4, a5);
	}
	void getPointRGB(size_t a0, float & a1, float & a2, float & a3, float & a4, float & a5, float & a6) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "getPointRGB");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4, a5, a6);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointsMapXYZI::getPointRGB(a0, a1, a2, a3, a4, a5, a6);
	}
	bool hasColorPoints() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "hasColorPoints");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPointsMapXYZI::hasColorPoints();
	}
	void getVisualizationInto(class mrpt::opengl::CSetOfObjects & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "getVisualizationInto");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointsMapXYZI::getVisualizationInto(a0);
	}
	void insertPointField_Intensity(float a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "insertPointField_Intensity");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointsMapXYZI::insertPointField_Intensity(a0);
	}
	void saveMetricMapRepresentationToFile(const std::string & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "saveMetricMapRepresentationToFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointsMapXYZI::saveMetricMapRepresentationToFile(a0);
	}
	void internal_clear() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "internal_clear");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointsMapXYZI::internal_clear();
	}
	void PLY_import_set_vertex(size_t a0, const struct mrpt::math::TPoint3D_<float> & a1, const struct mrpt::img::TColorf * a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "PLY_import_set_vertex");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointsMapXYZI::PLY_import_set_vertex(a0, a1, a2);
	}
	void PLY_import_set_vertex_count(size_t a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "PLY_import_set_vertex_count");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointsMapXYZI::PLY_import_set_vertex_count(a0);
	}
	void PLY_import_set_vertex_timestamp(size_t a0, const double a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "PLY_import_set_vertex_timestamp");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointsMapXYZI::PLY_import_set_vertex_timestamp(a0, a1);
	}
	void PLY_export_get_vertex(size_t a0, struct mrpt::math::TPoint3D_<float> & a1, bool & a2, struct mrpt::img::TColorf & a3) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "PLY_export_get_vertex");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointsMapXYZI::PLY_export_get_vertex(a0, a1, a2, a3);
	}
	float squareDistanceToClosestCorrespondence(float a0, float a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "squareDistanceToClosestCorrespondence");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<float>::value) {
				static pybind11::detail::override_caster_t<float> caster;
				return pybind11::detail::cast_ref<float>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<float>(std::move(o));
		}
		return CPointsMap::squareDistanceToClosestCorrespondence(a0, a1);
	}
	void setPointWeight(size_t a0, unsigned long a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "setPointWeight");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointsMap::setPointWeight(a0, a1);
	}
	unsigned long getPointWeight(size_t a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "getPointWeight");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<unsigned long>::value) {
				static pybind11::detail::override_caster_t<unsigned long> caster;
				return pybind11::detail::cast_ref<unsigned long>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<unsigned long>(std::move(o));
		}
		return CPointsMap::getPointWeight(a0);
	}
	void insertPointField_Ring(uint16_t a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "insertPointField_Ring");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointsMap::insertPointField_Ring(a0);
	}
	void insertPointField_Timestamp(float a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "insertPointField_Timestamp");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointsMap::insertPointField_Timestamp(a0);
	}
	void insertPointField_color_R(float a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "insertPointField_color_R");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointsMap::insertPointField_color_R(a0);
	}
	void insertPointField_color_G(float a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "insertPointField_color_G");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointsMap::insertPointField_color_G(a0);
	}
	void insertPointField_color_B(float a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "insertPointField_color_B");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointsMap::insertPointField_color_B(a0);
	}
	void determineMatching2D(const class mrpt::maps::CMetricMap * a0, const class mrpt::poses::CPose2D & a1, class mrpt::tfest::TMatchingPairListTempl<float> & a2, const struct mrpt::maps::TMatchingParams & a3, struct mrpt::maps::TMatchingExtraResults & a4) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "determineMatching2D");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointsMap::determineMatching2D(a0, a1, a2, a3, a4);
	}
	void determineMatching3D(const class mrpt::maps::CMetricMap * a0, const class mrpt::poses::CPose3D & a1, class mrpt::tfest::TMatchingPairListTempl<float> & a2, const struct mrpt::maps::TMatchingParams & a3, struct mrpt::maps::TMatchingExtraResults & a4) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "determineMatching3D");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointsMap::determineMatching3D(a0, a1, a2, a3, a4);
	}
	float compute3DMatchingRatio(const class mrpt::maps::CMetricMap * a0, const class mrpt::poses::CPose3D & a1, const struct mrpt::maps::TMatchingRatioParams & a2) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "compute3DMatchingRatio");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<float>::value) {
				static pybind11::detail::override_caster_t<float> caster;
				return pybind11::detail::cast_ref<float>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<float>(std::move(o));
		}
		return CPointsMap::compute3DMatchingRatio(a0, a1, a2);
	}
	bool isEmpty() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "isEmpty");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPointsMap::isEmpty();
	}
	struct mrpt::math::TBoundingBox_<float> boundingBox() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "boundingBox");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TBoundingBox_<float>>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TBoundingBox_<float>> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TBoundingBox_<float>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TBoundingBox_<float>>(std::move(o));
		}
		return CPointsMap::boundingBox();
	}
	double internal_computeObservationLikelihood(const class mrpt::obs::CObservation & a0, const class mrpt::poses::CPose3D & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "internal_computeObservationLikelihood");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CPointsMap::internal_computeObservationLikelihood(a0, a1);
	}
	std::string asString() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "asString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CPointsMap::asString();
	}
	void nn_prepare_for_2d_queries() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "nn_prepare_for_2d_queries");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointsMap::nn_prepare_for_2d_queries();
	}
	void nn_prepare_for_3d_queries() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "nn_prepare_for_3d_queries");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointsMap::nn_prepare_for_3d_queries();
	}
	bool nn_has_indices_or_ids() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "nn_has_indices_or_ids");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPointsMap::nn_has_indices_or_ids();
	}
	size_t nn_index_count() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "nn_index_count");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CPointsMap::nn_index_count();
	}
	bool nn_single_search(const struct mrpt::math::TPoint3D_<float> & a0, struct mrpt::math::TPoint3D_<float> & a1, float & a2, uint64_t & a3) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "nn_single_search");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPointsMap::nn_single_search(a0, a1, a2, a3);
	}
	bool nn_single_search(const struct mrpt::math::TPoint2D_<float> & a0, struct mrpt::math::TPoint2D_<float> & a1, float & a2, uint64_t & a3) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "nn_single_search");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPointsMap::nn_single_search(a0, a1, a2, a3);
	}
	void PLY_import_set_face_count(size_t a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "PLY_import_set_face_count");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointsMap::PLY_import_set_face_count(a0);
	}
	size_t PLY_export_get_vertex_count() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "PLY_export_get_vertex_count");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CPointsMap::PLY_export_get_vertex_count();
	}
	size_t PLY_export_get_face_count() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "PLY_export_get_face_count");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CPointsMap::PLY_export_get_face_count();
	}
	bool canComputeObservationLikelihood(const class mrpt::obs::CObservation & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "canComputeObservationLikelihood");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CMetricMap::canComputeObservationLikelihood(a0);
	}
	void auxParticleFilterCleanUp() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI *>(this), "auxParticleFilterCleanUp");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMetricMap::auxParticleFilterCleanUp();
	}
};

// mrpt::maps::CPointsMapXYZI::TMapDefinition file: line:78
struct PyCallBack_mrpt_maps_CPointsMapXYZI_TMapDefinition : public mrpt::maps::CPointsMapXYZI::TMapDefinition {
	using mrpt::maps::CPointsMapXYZI::TMapDefinition::TMapDefinition;

	void loadFromConfigFile_map_specific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI::TMapDefinition *>(this), "loadFromConfigFile_map_specific");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TMapDefinition::loadFromConfigFile_map_specific(a0, a1);
	}
	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI::TMapDefinition *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TMetricMapInitializer::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointsMapXYZI::TMapDefinition *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TMetricMapInitializer::saveToConfigFile(a0, a1);
	}
};

void bind_mrpt_maps_CPointCloudFilterByDistance(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::maps::CPointCloudFilterByDistance file:mrpt/maps/CPointCloudFilterByDistance.h line:27
		pybind11::class_<mrpt::maps::CPointCloudFilterByDistance, std::shared_ptr<mrpt::maps::CPointCloudFilterByDistance>, PyCallBack_mrpt_maps_CPointCloudFilterByDistance, mrpt::maps::CPointCloudFilterBase> cl(M("mrpt::maps"), "CPointCloudFilterByDistance", "Implementation of pointcloud filtering based on requisities for minimum\n neigbouring points in both,\n   the current timestamp and a previous one.\n\n \n CPointsMap\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::maps::CPointCloudFilterByDistance(); }, [](){ return new PyCallBack_mrpt_maps_CPointCloudFilterByDistance(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_maps_CPointCloudFilterByDistance const &o){ return new PyCallBack_mrpt_maps_CPointCloudFilterByDistance(o); } ) );
		cl.def( pybind11::init( [](mrpt::maps::CPointCloudFilterByDistance const &o){ return new mrpt::maps::CPointCloudFilterByDistance(o); } ) );
		cl.def_readwrite("options", &mrpt::maps::CPointCloudFilterByDistance::options);
		cl.def("filter", [](mrpt::maps::CPointCloudFilterByDistance &o, class mrpt::maps::CPointsMap * a0, const mrpt::Clock::time_point & a1, const class mrpt::poses::CPose3D & a2) -> void { return o.filter(a0, a1, a2); }, "", pybind11::arg("inout_pointcloud"), pybind11::arg("pc_timestamp"), pybind11::arg("pc_reference_pose"));
		cl.def("filter", (void (mrpt::maps::CPointCloudFilterByDistance::*)(class mrpt::maps::CPointsMap *, const mrpt::Clock::time_point, const class mrpt::poses::CPose3D &, struct mrpt::maps::CPointCloudFilterBase::TExtraFilterParams *)) &mrpt::maps::CPointCloudFilterByDistance::filter, "C++: mrpt::maps::CPointCloudFilterByDistance::filter(class mrpt::maps::CPointsMap *, const mrpt::Clock::time_point, const class mrpt::poses::CPose3D &, struct mrpt::maps::CPointCloudFilterBase::TExtraFilterParams *) --> void", pybind11::arg("inout_pointcloud"), pybind11::arg("pc_timestamp"), pybind11::arg("pc_reference_pose"), pybind11::arg("params"));
		cl.def("assign", (class mrpt::maps::CPointCloudFilterByDistance & (mrpt::maps::CPointCloudFilterByDistance::*)(const class mrpt::maps::CPointCloudFilterByDistance &)) &mrpt::maps::CPointCloudFilterByDistance::operator=, "C++: mrpt::maps::CPointCloudFilterByDistance::operator=(const class mrpt::maps::CPointCloudFilterByDistance &) --> class mrpt::maps::CPointCloudFilterByDistance &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::maps::CPointCloudFilterByDistance::TOptions file:mrpt/maps/CPointCloudFilterByDistance.h line:44
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CPointCloudFilterByDistance::TOptions, std::shared_ptr<mrpt::maps::CPointCloudFilterByDistance::TOptions>, PyCallBack_mrpt_maps_CPointCloudFilterByDistance_TOptions, mrpt::config::CLoadableOptions> cl(enclosing_class, "TOptions", "");
			cl.def( pybind11::init( [](){ return new mrpt::maps::CPointCloudFilterByDistance::TOptions(); }, [](){ return new PyCallBack_mrpt_maps_CPointCloudFilterByDistance_TOptions(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_maps_CPointCloudFilterByDistance_TOptions const &o){ return new PyCallBack_mrpt_maps_CPointCloudFilterByDistance_TOptions(o); } ) );
			cl.def( pybind11::init( [](mrpt::maps::CPointCloudFilterByDistance::TOptions const &o){ return new mrpt::maps::CPointCloudFilterByDistance::TOptions(o); } ) );
			cl.def_readwrite("min_dist", &mrpt::maps::CPointCloudFilterByDistance::TOptions::min_dist);
			cl.def_readwrite("angle_tolerance", &mrpt::maps::CPointCloudFilterByDistance::TOptions::angle_tolerance);
			cl.def_readwrite("too_old_seconds", &mrpt::maps::CPointCloudFilterByDistance::TOptions::too_old_seconds);
			cl.def_readwrite("previous_keyframes", &mrpt::maps::CPointCloudFilterByDistance::TOptions::previous_keyframes);
			cl.def_readwrite("max_deletion_ratio", &mrpt::maps::CPointCloudFilterByDistance::TOptions::max_deletion_ratio);
			cl.def("loadFromConfigFile", (void (mrpt::maps::CPointCloudFilterByDistance::TOptions::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::maps::CPointCloudFilterByDistance::TOptions::loadFromConfigFile, "C++: mrpt::maps::CPointCloudFilterByDistance::TOptions::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
			cl.def("saveToConfigFile", (void (mrpt::maps::CPointCloudFilterByDistance::TOptions::*)(class mrpt::config::CConfigFileBase &, const std::string &) const) &mrpt::maps::CPointCloudFilterByDistance::TOptions::saveToConfigFile, "C++: mrpt::maps::CPointCloudFilterByDistance::TOptions::saveToConfigFile(class mrpt::config::CConfigFileBase &, const std::string &) const --> void", pybind11::arg("c"), pybind11::arg("section"));
			cl.def("assign", (struct mrpt::maps::CPointCloudFilterByDistance::TOptions & (mrpt::maps::CPointCloudFilterByDistance::TOptions::*)(const struct mrpt::maps::CPointCloudFilterByDistance::TOptions &)) &mrpt::maps::CPointCloudFilterByDistance::TOptions::operator=, "C++: mrpt::maps::CPointCloudFilterByDistance::TOptions::operator=(const struct mrpt::maps::CPointCloudFilterByDistance::TOptions &) --> struct mrpt::maps::CPointCloudFilterByDistance::TOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
	{ // mrpt::maps::CPointsMapXYZI file:mrpt/maps/CPointsMapXYZI.h line:24
		pybind11::class_<mrpt::maps::CPointsMapXYZI, std::shared_ptr<mrpt::maps::CPointsMapXYZI>, PyCallBack_mrpt_maps_CPointsMapXYZI, mrpt::maps::CPointsMap> cl(M("mrpt::maps"), "CPointsMapXYZI", "A map of 3D points with reflectance/intensity (float).\n \n\n mrpt::maps::CPointsMap, mrpt::maps::CMetricMap\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::maps::CPointsMapXYZI(); }, [](){ return new PyCallBack_mrpt_maps_CPointsMapXYZI(); } ) );
		cl.def( pybind11::init<const class mrpt::maps::CPointsMap &>(), pybind11::arg("o") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_maps_CPointsMapXYZI const &o){ return new PyCallBack_mrpt_maps_CPointsMapXYZI(o); } ) );
		cl.def( pybind11::init( [](mrpt::maps::CPointsMapXYZI const &o){ return new mrpt::maps::CPointsMapXYZI(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::maps::CPointsMapXYZI::GetRuntimeClassIdStatic, "C++: mrpt::maps::CPointsMapXYZI::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::maps::CPointsMapXYZI::*)() const) &mrpt::maps::CPointsMapXYZI::GetRuntimeClass, "C++: mrpt::maps::CPointsMapXYZI::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::maps::CPointsMapXYZI::*)() const) &mrpt::maps::CPointsMapXYZI::clone, "C++: mrpt::maps::CPointsMapXYZI::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::maps::CPointsMapXYZI::CreateObject, "C++: mrpt::maps::CPointsMapXYZI::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("assign", (class mrpt::maps::CPointsMapXYZI & (mrpt::maps::CPointsMapXYZI::*)(const class mrpt::maps::CPointsMap &)) &mrpt::maps::CPointsMapXYZI::operator=, "C++: mrpt::maps::CPointsMapXYZI::operator=(const class mrpt::maps::CPointsMap &) --> class mrpt::maps::CPointsMapXYZI &", pybind11::return_value_policy::automatic, pybind11::arg("o"));
		cl.def("assign", (class mrpt::maps::CPointsMapXYZI & (mrpt::maps::CPointsMapXYZI::*)(const class mrpt::maps::CPointsMapXYZI &)) &mrpt::maps::CPointsMapXYZI::operator=, "C++: mrpt::maps::CPointsMapXYZI::operator=(const class mrpt::maps::CPointsMapXYZI &) --> class mrpt::maps::CPointsMapXYZI &", pybind11::return_value_policy::automatic, pybind11::arg("o"));
		cl.def("reserve", (void (mrpt::maps::CPointsMapXYZI::*)(size_t)) &mrpt::maps::CPointsMapXYZI::reserve, "from CPointsMap\n  @{ \n\nC++: mrpt::maps::CPointsMapXYZI::reserve(size_t) --> void", pybind11::arg("newLength"));
		cl.def("resize", (void (mrpt::maps::CPointsMapXYZI::*)(size_t)) &mrpt::maps::CPointsMapXYZI::resize, "C++: mrpt::maps::CPointsMapXYZI::resize(size_t) --> void", pybind11::arg("newLength"));
		cl.def("setSize", (void (mrpt::maps::CPointsMapXYZI::*)(size_t)) &mrpt::maps::CPointsMapXYZI::setSize, "C++: mrpt::maps::CPointsMapXYZI::setSize(size_t) --> void", pybind11::arg("newLength"));
		cl.def("insertPointFast", [](mrpt::maps::CPointsMapXYZI &o, float const & a0, float const & a1) -> void { return o.insertPointFast(a0, a1); }, "", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("insertPointFast", (void (mrpt::maps::CPointsMapXYZI::*)(float, float, float)) &mrpt::maps::CPointsMapXYZI::insertPointFast, "The virtual method for  *without* calling\n mark_as_modified()   \n\nC++: mrpt::maps::CPointsMapXYZI::insertPointFast(float, float, float) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("loadFromKittiVelodyneFile", (bool (mrpt::maps::CPointsMapXYZI::*)(const std::string &)) &mrpt::maps::CPointsMapXYZI::loadFromKittiVelodyneFile, "Loads from a Kitti dataset Velodyne scan binary file.\n The file can be gz compressed (only enabled if the filename ends in \".gz\"\n to prevent spurious false autodetection of gzip files).\n \n\n true on success \n\nC++: mrpt::maps::CPointsMapXYZI::loadFromKittiVelodyneFile(const std::string &) --> bool", pybind11::arg("filename"));
		cl.def("saveToKittiVelodyneFile", (bool (mrpt::maps::CPointsMapXYZI::*)(const std::string &) const) &mrpt::maps::CPointsMapXYZI::saveToKittiVelodyneFile, "C++: mrpt::maps::CPointsMapXYZI::saveToKittiVelodyneFile(const std::string &) const --> bool", pybind11::arg("filename"));
		cl.def("saveXYZI_to_text_file", (bool (mrpt::maps::CPointsMapXYZI::*)(const std::string &) const) &mrpt::maps::CPointsMapXYZI::saveXYZI_to_text_file, "Save to a text file. In each line contains X Y Z (meters) I (intensity)\n Returns false if any error occured, true elsewere.\n\nC++: mrpt::maps::CPointsMapXYZI::saveXYZI_to_text_file(const std::string &) const --> bool", pybind11::arg("file"));
		cl.def("loadXYZI_from_text_file", (bool (mrpt::maps::CPointsMapXYZI::*)(const std::string &)) &mrpt::maps::CPointsMapXYZI::loadXYZI_from_text_file, "Loads from a text file, each line having \"X Y Z I\", I in [0,1].\n Returns false if any error occured, true elsewere. \n\nC++: mrpt::maps::CPointsMapXYZI::loadXYZI_from_text_file(const std::string &) --> bool", pybind11::arg("file"));
		cl.def("setPointRGB", (void (mrpt::maps::CPointsMapXYZI::*)(size_t, float, float, float, float, float, float)) &mrpt::maps::CPointsMapXYZI::setPointRGB, "Changes a given point from map. First index is 0.\n \n\n Throws std::exception on index out of bound.\n\nC++: mrpt::maps::CPointsMapXYZI::setPointRGB(size_t, float, float, float, float, float, float) --> void", pybind11::arg("index"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("R_intensity"), pybind11::arg("G_ignored"), pybind11::arg("B_ignored"));
		cl.def("insertPointRGB", (void (mrpt::maps::CPointsMapXYZI::*)(float, float, float, float, float, float)) &mrpt::maps::CPointsMapXYZI::insertPointRGB, "Adds a new point given its coordinates and color (colors range is [0,1])\n\nC++: mrpt::maps::CPointsMapXYZI::insertPointRGB(float, float, float, float, float, float) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("R_intensity"), pybind11::arg("G_ignored"), pybind11::arg("B_ignored"));
		cl.def("setPointIntensity", (void (mrpt::maps::CPointsMapXYZI::*)(size_t, float)) &mrpt::maps::CPointsMapXYZI::setPointIntensity, "Changes the intensity of a given point from the map. First index is 0.\n \n\n Throws std::exception on index out of bound.\n\nC++: mrpt::maps::CPointsMapXYZI::setPointIntensity(size_t, float) --> void", pybind11::arg("index"), pybind11::arg("intensity"));
		cl.def("setPointColor_fast", (void (mrpt::maps::CPointsMapXYZI::*)(size_t, float, float, float)) &mrpt::maps::CPointsMapXYZI::setPointColor_fast, "Like  but without checking for out-of-index erors \n\nC++: mrpt::maps::CPointsMapXYZI::setPointColor_fast(size_t, float, float, float) --> void", pybind11::arg("index"), pybind11::arg("R"), pybind11::arg("G"), pybind11::arg("B"));
		cl.def("getPointRGB", (void (mrpt::maps::CPointsMapXYZI::*)(size_t, float &, float &, float &, float &, float &, float &) const) &mrpt::maps::CPointsMapXYZI::getPointRGB, "Retrieves a point and its color (colors range is [0,1])\n\nC++: mrpt::maps::CPointsMapXYZI::getPointRGB(size_t, float &, float &, float &, float &, float &, float &) const --> void", pybind11::arg("index"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("R_intensity"), pybind11::arg("G_intensity"), pybind11::arg("B_intensity"));
		cl.def("getPointIntensity", (float (mrpt::maps::CPointsMapXYZI::*)(size_t) const) &mrpt::maps::CPointsMapXYZI::getPointIntensity, "Retrieves a point intensity (range [0,1]) \n\nC++: mrpt::maps::CPointsMapXYZI::getPointIntensity(size_t) const --> float", pybind11::arg("index"));
		cl.def("getPointIntensity_fast", (float (mrpt::maps::CPointsMapXYZI::*)(size_t) const) &mrpt::maps::CPointsMapXYZI::getPointIntensity_fast, "Like  but without checking for out-of-index erors \n\nC++: mrpt::maps::CPointsMapXYZI::getPointIntensity_fast(size_t) const --> float", pybind11::arg("index"));
		cl.def("hasColorPoints", (bool (mrpt::maps::CPointsMapXYZI::*)() const) &mrpt::maps::CPointsMapXYZI::hasColorPoints, "Returns true if the point map has a color field for each point \n\nC++: mrpt::maps::CPointsMapXYZI::hasColorPoints() const --> bool");
		cl.def("getVisualizationInto", (void (mrpt::maps::CPointsMapXYZI::*)(class mrpt::opengl::CSetOfObjects &) const) &mrpt::maps::CPointsMapXYZI::getVisualizationInto, "Override of the default 3D scene builder to account for the individual\n points' color.\n\nC++: mrpt::maps::CPointsMapXYZI::getVisualizationInto(class mrpt::opengl::CSetOfObjects &) const --> void", pybind11::arg("outObj"));
		cl.def("insertPointField_Intensity", (void (mrpt::maps::CPointsMapXYZI::*)(float)) &mrpt::maps::CPointsMapXYZI::insertPointField_Intensity, "C++: mrpt::maps::CPointsMapXYZI::insertPointField_Intensity(float) --> void", pybind11::arg("i"));
		cl.def("saveMetricMapRepresentationToFile", (void (mrpt::maps::CPointsMapXYZI::*)(const std::string &) const) &mrpt::maps::CPointsMapXYZI::saveMetricMapRepresentationToFile, "clang-format on\n\nC++: mrpt::maps::CPointsMapXYZI::saveMetricMapRepresentationToFile(const std::string &) const --> void", pybind11::arg("filNamePrefix"));

		{ // mrpt::maps::CPointsMapXYZI::TMapDefinitionBase file: line:74
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CPointsMapXYZI::TMapDefinitionBase, std::shared_ptr<mrpt::maps::CPointsMapXYZI::TMapDefinitionBase>> cl(enclosing_class, "TMapDefinitionBase", "");
		}

		{ // mrpt::maps::CPointsMapXYZI::TMapDefinition file: line:78
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CPointsMapXYZI::TMapDefinition, std::shared_ptr<mrpt::maps::CPointsMapXYZI::TMapDefinition>, PyCallBack_mrpt_maps_CPointsMapXYZI_TMapDefinition, mrpt::maps::CPointsMapXYZI::TMapDefinitionBase> cl(enclosing_class, "TMapDefinition", "");
			cl.def( pybind11::init( [](){ return new mrpt::maps::CPointsMapXYZI::TMapDefinition(); }, [](){ return new PyCallBack_mrpt_maps_CPointsMapXYZI_TMapDefinition(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_maps_CPointsMapXYZI_TMapDefinition const &o){ return new PyCallBack_mrpt_maps_CPointsMapXYZI_TMapDefinition(o); } ) );
			cl.def( pybind11::init( [](mrpt::maps::CPointsMapXYZI::TMapDefinition const &o){ return new mrpt::maps::CPointsMapXYZI::TMapDefinition(o); } ) );
			cl.def_readwrite("insertionOpts", &mrpt::maps::CPointsMapXYZI::TMapDefinition::insertionOpts);
			cl.def_readwrite("likelihoodOpts", &mrpt::maps::CPointsMapXYZI::TMapDefinition::likelihoodOpts);
		}

	}
}
