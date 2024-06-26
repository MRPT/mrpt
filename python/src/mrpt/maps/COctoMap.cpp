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
#include <mrpt/maps/COctoMap.h>
#include <mrpt/maps/COctoMapBase.h>
#include <mrpt/maps/CPointCloudFilterBase.h>
#include <mrpt/maps/CPointsMap.h>
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
#include <mrpt/opengl/COctoMapVoxels.h>
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

// mrpt::maps::COctoMap file:mrpt/maps/COctoMap.h line:41
struct PyCallBack_mrpt_maps_COctoMap : public mrpt::maps::COctoMap {
	using mrpt::maps::COctoMap::COctoMap;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMap *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return COctoMap::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMap *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return COctoMap::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMap *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return COctoMap::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMap *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COctoMap::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMap *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COctoMap::serializeFrom(a0, a1);
	}
	void getAsOctoMapVoxels(class mrpt::opengl::COctoMapVoxels & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMap *>(this), "getAsOctoMapVoxels");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COctoMap::getAsOctoMapVoxels(a0);
	}
	bool isEmpty() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMap *>(this), "isEmpty");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return COctoMap::isEmpty();
	}
	void setOccupancyThres(double a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMap *>(this), "setOccupancyThres");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COctoMap::setOccupancyThres(a0);
	}
	void setProbHit(double a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMap *>(this), "setProbHit");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COctoMap::setProbHit(a0);
	}
	void setProbMiss(double a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMap *>(this), "setProbMiss");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COctoMap::setProbMiss(a0);
	}
	void setClampingThresMin(double a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMap *>(this), "setClampingThresMin");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COctoMap::setClampingThresMin(a0);
	}
	void setClampingThresMax(double a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMap *>(this), "setClampingThresMax");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COctoMap::setClampingThresMax(a0);
	}
	double getOccupancyThres() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMap *>(this), "getOccupancyThres");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return COctoMap::getOccupancyThres();
	}
	float getOccupancyThresLog() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMap *>(this), "getOccupancyThresLog");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<float>::value) {
				static pybind11::detail::override_caster_t<float> caster;
				return pybind11::detail::cast_ref<float>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<float>(std::move(o));
		}
		return COctoMap::getOccupancyThresLog();
	}
	double getProbHit() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMap *>(this), "getProbHit");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return COctoMap::getProbHit();
	}
	float getProbHitLog() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMap *>(this), "getProbHitLog");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<float>::value) {
				static pybind11::detail::override_caster_t<float> caster;
				return pybind11::detail::cast_ref<float>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<float>(std::move(o));
		}
		return COctoMap::getProbHitLog();
	}
	double getProbMiss() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMap *>(this), "getProbMiss");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return COctoMap::getProbMiss();
	}
	float getProbMissLog() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMap *>(this), "getProbMissLog");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<float>::value) {
				static pybind11::detail::override_caster_t<float> caster;
				return pybind11::detail::cast_ref<float>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<float>(std::move(o));
		}
		return COctoMap::getProbMissLog();
	}
	double getClampingThresMin() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMap *>(this), "getClampingThresMin");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return COctoMap::getClampingThresMin();
	}
	float getClampingThresMinLog() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMap *>(this), "getClampingThresMinLog");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<float>::value) {
				static pybind11::detail::override_caster_t<float> caster;
				return pybind11::detail::cast_ref<float>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<float>(std::move(o));
		}
		return COctoMap::getClampingThresMinLog();
	}
	double getClampingThresMax() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMap *>(this), "getClampingThresMax");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return COctoMap::getClampingThresMax();
	}
	float getClampingThresMaxLog() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMap *>(this), "getClampingThresMaxLog");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<float>::value) {
				static pybind11::detail::override_caster_t<float> caster;
				return pybind11::detail::cast_ref<float>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<float>(std::move(o));
		}
		return COctoMap::getClampingThresMaxLog();
	}
	void internal_clear() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMap *>(this), "internal_clear");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COctoMap::internal_clear();
	}
	std::string asString() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMap *>(this), "asString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return COctoMapBase::asString();
	}
	void saveMetricMapRepresentationToFile(const std::string & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMap *>(this), "saveMetricMapRepresentationToFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COctoMapBase::saveMetricMapRepresentationToFile(a0);
	}
	void getVisualizationInto(class mrpt::opengl::CSetOfObjects & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMap *>(this), "getVisualizationInto");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COctoMapBase::getVisualizationInto(a0);
	}
	struct mrpt::math::TBoundingBox_<float> boundingBox() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMap *>(this), "boundingBox");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TBoundingBox_<float>>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TBoundingBox_<float>> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TBoundingBox_<float>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TBoundingBox_<float>>(std::move(o));
		}
		return CMetricMap::boundingBox();
	}
	bool canComputeObservationLikelihood(const class mrpt::obs::CObservation & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMap *>(this), "canComputeObservationLikelihood");
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
	void determineMatching2D(const class mrpt::maps::CMetricMap * a0, const class mrpt::poses::CPose2D & a1, class mrpt::tfest::TMatchingPairListTempl<float> & a2, const struct mrpt::maps::TMatchingParams & a3, struct mrpt::maps::TMatchingExtraResults & a4) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMap *>(this), "determineMatching2D");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMetricMap::determineMatching2D(a0, a1, a2, a3, a4);
	}
	void determineMatching3D(const class mrpt::maps::CMetricMap * a0, const class mrpt::poses::CPose3D & a1, class mrpt::tfest::TMatchingPairListTempl<float> & a2, const struct mrpt::maps::TMatchingParams & a3, struct mrpt::maps::TMatchingExtraResults & a4) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMap *>(this), "determineMatching3D");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMetricMap::determineMatching3D(a0, a1, a2, a3, a4);
	}
	float compute3DMatchingRatio(const class mrpt::maps::CMetricMap * a0, const class mrpt::poses::CPose3D & a1, const struct mrpt::maps::TMatchingRatioParams & a2) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMap *>(this), "compute3DMatchingRatio");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<float>::value) {
				static pybind11::detail::override_caster_t<float> caster;
				return pybind11::detail::cast_ref<float>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<float>(std::move(o));
		}
		return CMetricMap::compute3DMatchingRatio(a0, a1, a2);
	}
	void auxParticleFilterCleanUp() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMap *>(this), "auxParticleFilterCleanUp");
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
	float squareDistanceToClosestCorrespondence(float a0, float a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMap *>(this), "squareDistanceToClosestCorrespondence");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<float>::value) {
				static pybind11::detail::override_caster_t<float> caster;
				return pybind11::detail::cast_ref<float>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<float>(std::move(o));
		}
		return CMetricMap::squareDistanceToClosestCorrespondence(a0, a1);
	}
};

// mrpt::maps::COctoMap::TMapDefinition file: line:78
struct PyCallBack_mrpt_maps_COctoMap_TMapDefinition : public mrpt::maps::COctoMap::TMapDefinition {
	using mrpt::maps::COctoMap::TMapDefinition::TMapDefinition;

	void loadFromConfigFile_map_specific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMap::TMapDefinition *>(this), "loadFromConfigFile_map_specific");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMap::TMapDefinition *>(this), "loadFromConfigFile");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COctoMap::TMapDefinition *>(this), "saveToConfigFile");
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

// mrpt::maps::CPointCloudFilterBase file:mrpt/maps/CPointCloudFilterBase.h line:34
struct PyCallBack_mrpt_maps_CPointCloudFilterBase : public mrpt::maps::CPointCloudFilterBase {
	using mrpt::maps::CPointCloudFilterBase::CPointCloudFilterBase;

	void filter(class mrpt::maps::CPointsMap * a0, const mrpt::Clock::time_point a1, const class mrpt::poses::CPose3D & a2, struct mrpt::maps::CPointCloudFilterBase::TExtraFilterParams * a3) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CPointCloudFilterBase *>(this), "filter");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CPointCloudFilterBase::filter\"");
	}
};

// mrpt::maps::CSimplePointsMap file:mrpt/maps/CSimplePointsMap.h line:30
struct PyCallBack_mrpt_maps_CSimplePointsMap : public mrpt::maps::CSimplePointsMap {
	using mrpt::maps::CSimplePointsMap::CSimplePointsMap;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CSimplePointsMap::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CSimplePointsMap::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CSimplePointsMap::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSimplePointsMap::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSimplePointsMap::serializeFrom(a0, a1);
	}
	void reserve(size_t a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "reserve");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSimplePointsMap::reserve(a0);
	}
	void resize(size_t a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "resize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSimplePointsMap::resize(a0);
	}
	void setSize(size_t a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "setSize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSimplePointsMap::setSize(a0);
	}
	void insertPointFast(float a0, float a1, float a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "insertPointFast");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSimplePointsMap::insertPointFast(a0, a1, a2);
	}
	void addFrom_classSpecific(const class mrpt::maps::CPointsMap & a0, size_t a1, const bool a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "addFrom_classSpecific");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSimplePointsMap::addFrom_classSpecific(a0, a1, a2);
	}
	const class mrpt::maps::CSimplePointsMap * getAsSimplePointsMap() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "getAsSimplePointsMap");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const class mrpt::maps::CSimplePointsMap *>::value) {
				static pybind11::detail::override_caster_t<const class mrpt::maps::CSimplePointsMap *> caster;
				return pybind11::detail::cast_ref<const class mrpt::maps::CSimplePointsMap *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const class mrpt::maps::CSimplePointsMap *>(std::move(o));
		}
		return CSimplePointsMap::getAsSimplePointsMap();
	}
	void internal_clear() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "internal_clear");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSimplePointsMap::internal_clear();
	}
	void PLY_import_set_vertex_count(size_t a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "PLY_import_set_vertex_count");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSimplePointsMap::PLY_import_set_vertex_count(a0);
	}
	void PLY_import_set_vertex_timestamp(size_t a0, const double a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "PLY_import_set_vertex_timestamp");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSimplePointsMap::PLY_import_set_vertex_timestamp(a0, a1);
	}
	float squareDistanceToClosestCorrespondence(float a0, float a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "squareDistanceToClosestCorrespondence");
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
	void saveMetricMapRepresentationToFile(const std::string & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "saveMetricMapRepresentationToFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointsMap::saveMetricMapRepresentationToFile(a0);
	}
	void getPointRGB(size_t a0, float & a1, float & a2, float & a3, float & a4, float & a5, float & a6) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "getPointRGB");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4, a5, a6);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointsMap::getPointRGB(a0, a1, a2, a3, a4, a5, a6);
	}
	bool hasColorPoints() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "hasColorPoints");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPointsMap::hasColorPoints();
	}
	void setPointRGB(size_t a0, float a1, float a2, float a3, float a4, float a5, float a6) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "setPointRGB");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4, a5, a6);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointsMap::setPointRGB(a0, a1, a2, a3, a4, a5, a6);
	}
	void setPointWeight(size_t a0, unsigned long a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "setPointWeight");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "getPointWeight");
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
	void insertPointField_Intensity(float a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "insertPointField_Intensity");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointsMap::insertPointField_Intensity(a0);
	}
	void insertPointField_Ring(uint16_t a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "insertPointField_Ring");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "insertPointField_Timestamp");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "insertPointField_color_R");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "insertPointField_color_G");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "insertPointField_color_B");
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
	void insertPointRGB(float a0, float a1, float a2, float a3, float a4, float a5) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "insertPointRGB");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4, a5);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointsMap::insertPointRGB(a0, a1, a2, a3, a4, a5);
	}
	void determineMatching2D(const class mrpt::maps::CMetricMap * a0, const class mrpt::poses::CPose2D & a1, class mrpt::tfest::TMatchingPairListTempl<float> & a2, const struct mrpt::maps::TMatchingParams & a3, struct mrpt::maps::TMatchingExtraResults & a4) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "determineMatching2D");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "determineMatching3D");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "compute3DMatchingRatio");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "isEmpty");
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
	void getVisualizationInto(class mrpt::opengl::CSetOfObjects & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "getVisualizationInto");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointsMap::getVisualizationInto(a0);
	}
	struct mrpt::math::TBoundingBox_<float> boundingBox() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "boundingBox");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "internal_computeObservationLikelihood");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "asString");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "nn_prepare_for_2d_queries");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "nn_prepare_for_3d_queries");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "nn_has_indices_or_ids");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "nn_index_count");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "nn_single_search");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "nn_single_search");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "PLY_import_set_face_count");
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
	void PLY_import_set_vertex(size_t a0, const struct mrpt::math::TPoint3D_<float> & a1, const struct mrpt::img::TColorf * a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "PLY_import_set_vertex");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointsMap::PLY_import_set_vertex(a0, a1, a2);
	}
	size_t PLY_export_get_vertex_count() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "PLY_export_get_vertex_count");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "PLY_export_get_face_count");
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
	void PLY_export_get_vertex(size_t a0, struct mrpt::math::TPoint3D_<float> & a1, bool & a2, struct mrpt::img::TColorf & a3) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "PLY_export_get_vertex");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointsMap::PLY_export_get_vertex(a0, a1, a2, a3);
	}
	bool canComputeObservationLikelihood(const class mrpt::obs::CObservation & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "canComputeObservationLikelihood");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap *>(this), "auxParticleFilterCleanUp");
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

// mrpt::maps::CSimplePointsMap::TMapDefinition file: line:78
struct PyCallBack_mrpt_maps_CSimplePointsMap_TMapDefinition : public mrpt::maps::CSimplePointsMap::TMapDefinition {
	using mrpt::maps::CSimplePointsMap::TMapDefinition::TMapDefinition;

	void loadFromConfigFile_map_specific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap::TMapDefinition *>(this), "loadFromConfigFile_map_specific");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap::TMapDefinition *>(this), "loadFromConfigFile");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CSimplePointsMap::TMapDefinition *>(this), "saveToConfigFile");
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

void bind_mrpt_maps_COctoMap(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::maps::COctoMap file:mrpt/maps/COctoMap.h line:41
		pybind11::class_<mrpt::maps::COctoMap, std::shared_ptr<mrpt::maps::COctoMap>, PyCallBack_mrpt_maps_COctoMap, mrpt::maps::COctoMapBase<octomap::OcTree,octomap::OcTreeNode>> cl(M("mrpt::maps"), "COctoMap", "A three-dimensional probabilistic occupancy grid, implemented as an\n octo-tree with the \"octomap\" C++ library.\n  This version only stores occupancy information at each octree node. See the\n base class mrpt::maps::COctoMapBase.\n\n The octomap library was presented in \n\n \n CMetricMap, the example in \"MRPT/samples/octomap_simple\"\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::maps::COctoMap(); }, [](){ return new PyCallBack_mrpt_maps_COctoMap(); } ), "doc");
		cl.def( pybind11::init<const double>(), pybind11::arg("resolution") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_maps_COctoMap const &o){ return new PyCallBack_mrpt_maps_COctoMap(o); } ) );
		cl.def( pybind11::init( [](mrpt::maps::COctoMap const &o){ return new mrpt::maps::COctoMap(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::maps::COctoMap::GetRuntimeClassIdStatic, "C++: mrpt::maps::COctoMap::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::maps::COctoMap::*)() const) &mrpt::maps::COctoMap::GetRuntimeClass, "C++: mrpt::maps::COctoMap::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::maps::COctoMap::*)() const) &mrpt::maps::COctoMap::clone, "C++: mrpt::maps::COctoMap::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::maps::COctoMap::CreateObject, "C++: mrpt::maps::COctoMap::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("getAsOctoMapVoxels", (void (mrpt::maps::COctoMap::*)(class mrpt::opengl::COctoMapVoxels &) const) &mrpt::maps::COctoMap::getAsOctoMapVoxels, "C++: mrpt::maps::COctoMap::getAsOctoMapVoxels(class mrpt::opengl::COctoMapVoxels &) const --> void", pybind11::arg("gl_obj"));
		cl.def("isEmpty", (bool (mrpt::maps::COctoMap::*)() const) &mrpt::maps::COctoMap::isEmpty, "Returns true if the map is empty/no observation has been inserted \n\nC++: mrpt::maps::COctoMap::isEmpty() const --> bool");
		cl.def("insertRay", (void (mrpt::maps::COctoMap::*)(const float, const float, const float, const float, const float, const float)) &mrpt::maps::COctoMap::insertRay, "Just like insertPointCloud but with a single ray. \n\nC++: mrpt::maps::COctoMap::insertRay(const float, const float, const float, const float, const float, const float) --> void", pybind11::arg("end_x"), pybind11::arg("end_y"), pybind11::arg("end_z"), pybind11::arg("sensor_x"), pybind11::arg("sensor_y"), pybind11::arg("sensor_z"));
		cl.def("updateVoxel", (void (mrpt::maps::COctoMap::*)(const double, const double, const double, bool)) &mrpt::maps::COctoMap::updateVoxel, "Manually updates the occupancy of the voxel at (x,y,z) as being occupied\n (true) or free (false), using the log-odds parameters in \n \n\nC++: mrpt::maps::COctoMap::updateVoxel(const double, const double, const double, bool) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("occupied"));
		cl.def("isPointWithinOctoMap", (bool (mrpt::maps::COctoMap::*)(const float, const float, const float) const) &mrpt::maps::COctoMap::isPointWithinOctoMap, "Check whether the given point lies within the volume covered by the\n octomap (that is, whether it is \"mapped\") \n\nC++: mrpt::maps::COctoMap::isPointWithinOctoMap(const float, const float, const float) const --> bool", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("getResolution", (double (mrpt::maps::COctoMap::*)() const) &mrpt::maps::COctoMap::getResolution, "C++: mrpt::maps::COctoMap::getResolution() const --> double");
		cl.def("getTreeDepth", (unsigned int (mrpt::maps::COctoMap::*)() const) &mrpt::maps::COctoMap::getTreeDepth, "C++: mrpt::maps::COctoMap::getTreeDepth() const --> unsigned int");
		cl.def("size", (size_t (mrpt::maps::COctoMap::*)() const) &mrpt::maps::COctoMap::size, "The number of nodes in the tree\n\nC++: mrpt::maps::COctoMap::size() const --> size_t");
		cl.def("memoryUsage", (size_t (mrpt::maps::COctoMap::*)() const) &mrpt::maps::COctoMap::memoryUsage, "Memory usage of the complete octree in bytes (may vary between\n architectures)\n\nC++: mrpt::maps::COctoMap::memoryUsage() const --> size_t");
		cl.def("memoryUsageNode", (size_t (mrpt::maps::COctoMap::*)() const) &mrpt::maps::COctoMap::memoryUsageNode, "Memory usage of the a single octree node\n\nC++: mrpt::maps::COctoMap::memoryUsageNode() const --> size_t");
		cl.def("memoryFullGrid", (size_t (mrpt::maps::COctoMap::*)() const) &mrpt::maps::COctoMap::memoryFullGrid, "Memory usage of a full grid of the same size as the OcTree in\n bytes (for comparison)\n\nC++: mrpt::maps::COctoMap::memoryFullGrid() const --> size_t");
		cl.def("volume", (double (mrpt::maps::COctoMap::*)()) &mrpt::maps::COctoMap::volume, "C++: mrpt::maps::COctoMap::volume() --> double");
		cl.def("getMetricSize", (void (mrpt::maps::COctoMap::*)(double &, double &, double &)) &mrpt::maps::COctoMap::getMetricSize, "Size of OcTree (all known space) in meters for x, y and z dimension\n\nC++: mrpt::maps::COctoMap::getMetricSize(double &, double &, double &) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("getMetricMin", (void (mrpt::maps::COctoMap::*)(double &, double &, double &)) &mrpt::maps::COctoMap::getMetricMin, "minimum value of the bounding box of all known space in x, y, z\n\nC++: mrpt::maps::COctoMap::getMetricMin(double &, double &, double &) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("getMetricMax", (void (mrpt::maps::COctoMap::*)(double &, double &, double &)) &mrpt::maps::COctoMap::getMetricMax, "maximum value of the bounding box of all known space in x, y, z\n\nC++: mrpt::maps::COctoMap::getMetricMax(double &, double &, double &) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("calcNumNodes", (size_t (mrpt::maps::COctoMap::*)() const) &mrpt::maps::COctoMap::calcNumNodes, "Traverses the tree to calculate the total number of nodes\n\nC++: mrpt::maps::COctoMap::calcNumNodes() const --> size_t");
		cl.def("getNumLeafNodes", (size_t (mrpt::maps::COctoMap::*)() const) &mrpt::maps::COctoMap::getNumLeafNodes, "Traverses the tree to calculate the total number of leaf nodes\n\nC++: mrpt::maps::COctoMap::getNumLeafNodes() const --> size_t");
		cl.def("setOccupancyThres", (void (mrpt::maps::COctoMap::*)(double)) &mrpt::maps::COctoMap::setOccupancyThres, "C++: mrpt::maps::COctoMap::setOccupancyThres(double) --> void", pybind11::arg("prob"));
		cl.def("setProbHit", (void (mrpt::maps::COctoMap::*)(double)) &mrpt::maps::COctoMap::setProbHit, "C++: mrpt::maps::COctoMap::setProbHit(double) --> void", pybind11::arg("prob"));
		cl.def("setProbMiss", (void (mrpt::maps::COctoMap::*)(double)) &mrpt::maps::COctoMap::setProbMiss, "C++: mrpt::maps::COctoMap::setProbMiss(double) --> void", pybind11::arg("prob"));
		cl.def("setClampingThresMin", (void (mrpt::maps::COctoMap::*)(double)) &mrpt::maps::COctoMap::setClampingThresMin, "C++: mrpt::maps::COctoMap::setClampingThresMin(double) --> void", pybind11::arg("thresProb"));
		cl.def("setClampingThresMax", (void (mrpt::maps::COctoMap::*)(double)) &mrpt::maps::COctoMap::setClampingThresMax, "C++: mrpt::maps::COctoMap::setClampingThresMax(double) --> void", pybind11::arg("thresProb"));
		cl.def("getOccupancyThres", (double (mrpt::maps::COctoMap::*)() const) &mrpt::maps::COctoMap::getOccupancyThres, "C++: mrpt::maps::COctoMap::getOccupancyThres() const --> double");
		cl.def("getOccupancyThresLog", (float (mrpt::maps::COctoMap::*)() const) &mrpt::maps::COctoMap::getOccupancyThresLog, "C++: mrpt::maps::COctoMap::getOccupancyThresLog() const --> float");
		cl.def("getProbHit", (double (mrpt::maps::COctoMap::*)() const) &mrpt::maps::COctoMap::getProbHit, "C++: mrpt::maps::COctoMap::getProbHit() const --> double");
		cl.def("getProbHitLog", (float (mrpt::maps::COctoMap::*)() const) &mrpt::maps::COctoMap::getProbHitLog, "C++: mrpt::maps::COctoMap::getProbHitLog() const --> float");
		cl.def("getProbMiss", (double (mrpt::maps::COctoMap::*)() const) &mrpt::maps::COctoMap::getProbMiss, "C++: mrpt::maps::COctoMap::getProbMiss() const --> double");
		cl.def("getProbMissLog", (float (mrpt::maps::COctoMap::*)() const) &mrpt::maps::COctoMap::getProbMissLog, "C++: mrpt::maps::COctoMap::getProbMissLog() const --> float");
		cl.def("getClampingThresMin", (double (mrpt::maps::COctoMap::*)() const) &mrpt::maps::COctoMap::getClampingThresMin, "C++: mrpt::maps::COctoMap::getClampingThresMin() const --> double");
		cl.def("getClampingThresMinLog", (float (mrpt::maps::COctoMap::*)() const) &mrpt::maps::COctoMap::getClampingThresMinLog, "C++: mrpt::maps::COctoMap::getClampingThresMinLog() const --> float");
		cl.def("getClampingThresMax", (double (mrpt::maps::COctoMap::*)() const) &mrpt::maps::COctoMap::getClampingThresMax, "C++: mrpt::maps::COctoMap::getClampingThresMax() const --> double");
		cl.def("getClampingThresMaxLog", (float (mrpt::maps::COctoMap::*)() const) &mrpt::maps::COctoMap::getClampingThresMaxLog, "C++: mrpt::maps::COctoMap::getClampingThresMaxLog() const --> float");
		cl.def("assign", (class mrpt::maps::COctoMap & (mrpt::maps::COctoMap::*)(const class mrpt::maps::COctoMap &)) &mrpt::maps::COctoMap::operator=, "C++: mrpt::maps::COctoMap::operator=(const class mrpt::maps::COctoMap &) --> class mrpt::maps::COctoMap &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::maps::COctoMap::TMapDefinitionBase file: line:74
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::COctoMap::TMapDefinitionBase, std::shared_ptr<mrpt::maps::COctoMap::TMapDefinitionBase>> cl(enclosing_class, "TMapDefinitionBase", "");
		}

		{ // mrpt::maps::COctoMap::TMapDefinition file: line:78
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::COctoMap::TMapDefinition, std::shared_ptr<mrpt::maps::COctoMap::TMapDefinition>, PyCallBack_mrpt_maps_COctoMap_TMapDefinition, mrpt::maps::COctoMap::TMapDefinitionBase> cl(enclosing_class, "TMapDefinition", "");
			cl.def( pybind11::init( [](){ return new mrpt::maps::COctoMap::TMapDefinition(); }, [](){ return new PyCallBack_mrpt_maps_COctoMap_TMapDefinition(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_maps_COctoMap_TMapDefinition const &o){ return new PyCallBack_mrpt_maps_COctoMap_TMapDefinition(o); } ) );
			cl.def( pybind11::init( [](mrpt::maps::COctoMap::TMapDefinition const &o){ return new mrpt::maps::COctoMap::TMapDefinition(o); } ) );
			cl.def_readwrite("resolution", &mrpt::maps::COctoMap::TMapDefinition::resolution);
			cl.def_readwrite("insertionOpts", &mrpt::maps::COctoMap::TMapDefinition::insertionOpts);
			cl.def_readwrite("likelihoodOpts", &mrpt::maps::COctoMap::TMapDefinition::likelihoodOpts);
		}

	}
	{ // mrpt::maps::CPointCloudFilterBase file:mrpt/maps/CPointCloudFilterBase.h line:34
		pybind11::class_<mrpt::maps::CPointCloudFilterBase, std::shared_ptr<mrpt::maps::CPointCloudFilterBase>, PyCallBack_mrpt_maps_CPointCloudFilterBase> cl(M("mrpt::maps"), "CPointCloudFilterBase", "Virtual base class for all point-cloud filtering algorithm. See derived\n classes for implementations.\n \n\n CPointsMap\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_maps_CPointCloudFilterBase(); } ) );
		cl.def(pybind11::init<PyCallBack_mrpt_maps_CPointCloudFilterBase const &>());
		cl.def("filter", [](mrpt::maps::CPointCloudFilterBase &o, class mrpt::maps::CPointsMap * a0, const mrpt::Clock::time_point & a1, const class mrpt::poses::CPose3D & a2) -> void { return o.filter(a0, a1, a2); }, "", pybind11::arg("inout_pointcloud"), pybind11::arg("pc_timestamp"), pybind11::arg("pc_reference_pose"));
		cl.def("filter", (void (mrpt::maps::CPointCloudFilterBase::*)(class mrpt::maps::CPointsMap *, const mrpt::Clock::time_point, const class mrpt::poses::CPose3D &, struct mrpt::maps::CPointCloudFilterBase::TExtraFilterParams *)) &mrpt::maps::CPointCloudFilterBase::filter, "Apply the filtering algorithm to the pointcloud. \n\nC++: mrpt::maps::CPointCloudFilterBase::filter(class mrpt::maps::CPointsMap *, const mrpt::Clock::time_point, const class mrpt::poses::CPose3D &, struct mrpt::maps::CPointCloudFilterBase::TExtraFilterParams *) --> void", pybind11::arg("inout_pointcloud"), pybind11::arg("pc_timestamp"), pybind11::arg("pc_reference_pose"), pybind11::arg("params"));
		cl.def("assign", (class mrpt::maps::CPointCloudFilterBase & (mrpt::maps::CPointCloudFilterBase::*)(const class mrpt::maps::CPointCloudFilterBase &)) &mrpt::maps::CPointCloudFilterBase::operator=, "C++: mrpt::maps::CPointCloudFilterBase::operator=(const class mrpt::maps::CPointCloudFilterBase &) --> class mrpt::maps::CPointCloudFilterBase &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::maps::CPointCloudFilterBase::TExtraFilterParams file:mrpt/maps/CPointCloudFilterBase.h line:41
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CPointCloudFilterBase::TExtraFilterParams, std::shared_ptr<mrpt::maps::CPointCloudFilterBase::TExtraFilterParams>> cl(enclosing_class, "TExtraFilterParams", "");
			cl.def( pybind11::init( [](){ return new mrpt::maps::CPointCloudFilterBase::TExtraFilterParams(); } ) );
			cl.def_readwrite("do_not_delete", &mrpt::maps::CPointCloudFilterBase::TExtraFilterParams::do_not_delete);
		}

	}
	{ // mrpt::maps::CSimplePointsMap file:mrpt/maps/CSimplePointsMap.h line:30
		pybind11::class_<mrpt::maps::CSimplePointsMap, std::shared_ptr<mrpt::maps::CSimplePointsMap>, PyCallBack_mrpt_maps_CSimplePointsMap, mrpt::maps::CPointsMap> cl(M("mrpt::maps"), "CSimplePointsMap", "A cloud of points in 2D or 3D, which can be built from a sequence of laser\n scans.\n    This class only stores the coordinates (x,y,z) of each point.\n\n  See mrpt::maps::CPointsMap and derived classes for other point cloud\n classes.\n\n \n CMetricMap, CWeightedPointsMap, CPoint,\n mrpt::serialization::CSerializable \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::maps::CSimplePointsMap(); }, [](){ return new PyCallBack_mrpt_maps_CSimplePointsMap(); } ) );
		cl.def( pybind11::init<const class mrpt::maps::CPointsMap &>(), pybind11::arg("o") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_maps_CSimplePointsMap const &o){ return new PyCallBack_mrpt_maps_CSimplePointsMap(o); } ) );
		cl.def( pybind11::init( [](mrpt::maps::CSimplePointsMap const &o){ return new mrpt::maps::CSimplePointsMap(o); } ) );
		cl.def_static("Create", (class std::shared_ptr<class mrpt::maps::CSimplePointsMap> (*)()) &mrpt::maps::CSimplePointsMap::Create, "C++: mrpt::maps::CSimplePointsMap::Create() --> class std::shared_ptr<class mrpt::maps::CSimplePointsMap>");
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::maps::CSimplePointsMap::GetRuntimeClassIdStatic, "C++: mrpt::maps::CSimplePointsMap::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::maps::CSimplePointsMap::*)() const) &mrpt::maps::CSimplePointsMap::GetRuntimeClass, "C++: mrpt::maps::CSimplePointsMap::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::maps::CSimplePointsMap::*)() const) &mrpt::maps::CSimplePointsMap::clone, "C++: mrpt::maps::CSimplePointsMap::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::maps::CSimplePointsMap::CreateObject, "C++: mrpt::maps::CSimplePointsMap::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("assign", (class mrpt::maps::CSimplePointsMap & (mrpt::maps::CSimplePointsMap::*)(const class mrpt::maps::CPointsMap &)) &mrpt::maps::CSimplePointsMap::operator=, "C++: mrpt::maps::CSimplePointsMap::operator=(const class mrpt::maps::CPointsMap &) --> class mrpt::maps::CSimplePointsMap &", pybind11::return_value_policy::automatic, pybind11::arg("o"));
		cl.def("assign", (class mrpt::maps::CSimplePointsMap & (mrpt::maps::CSimplePointsMap::*)(const class mrpt::maps::CSimplePointsMap &)) &mrpt::maps::CSimplePointsMap::operator=, "C++: mrpt::maps::CSimplePointsMap::operator=(const class mrpt::maps::CSimplePointsMap &) --> class mrpt::maps::CSimplePointsMap &", pybind11::return_value_policy::automatic, pybind11::arg("o"));
		cl.def("reserve", (void (mrpt::maps::CSimplePointsMap::*)(size_t)) &mrpt::maps::CSimplePointsMap::reserve, "from CPointsMap\n    @{ \n\nC++: mrpt::maps::CSimplePointsMap::reserve(size_t) --> void", pybind11::arg("newLength"));
		cl.def("resize", (void (mrpt::maps::CSimplePointsMap::*)(size_t)) &mrpt::maps::CSimplePointsMap::resize, "C++: mrpt::maps::CSimplePointsMap::resize(size_t) --> void", pybind11::arg("newLength"));
		cl.def("setSize", (void (mrpt::maps::CSimplePointsMap::*)(size_t)) &mrpt::maps::CSimplePointsMap::setSize, "C++: mrpt::maps::CSimplePointsMap::setSize(size_t) --> void", pybind11::arg("newLength"));
		cl.def("insertPointFast", [](mrpt::maps::CSimplePointsMap &o, float const & a0, float const & a1) -> void { return o.insertPointFast(a0, a1); }, "", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("insertPointFast", (void (mrpt::maps::CSimplePointsMap::*)(float, float, float)) &mrpt::maps::CSimplePointsMap::insertPointFast, "The virtual method for  *without* calling\n mark_as_modified()   \n\nC++: mrpt::maps::CSimplePointsMap::insertPointFast(float, float, float) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("getAsSimplePointsMap", (const class mrpt::maps::CSimplePointsMap * (mrpt::maps::CSimplePointsMap::*)() const) &mrpt::maps::CSimplePointsMap::getAsSimplePointsMap, "@} \n\nC++: mrpt::maps::CSimplePointsMap::getAsSimplePointsMap() const --> const class mrpt::maps::CSimplePointsMap *", pybind11::return_value_policy::automatic);

		{ // mrpt::maps::CSimplePointsMap::TMapDefinitionBase file: line:74
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CSimplePointsMap::TMapDefinitionBase, std::shared_ptr<mrpt::maps::CSimplePointsMap::TMapDefinitionBase>> cl(enclosing_class, "TMapDefinitionBase", "");
		}

		{ // mrpt::maps::CSimplePointsMap::TMapDefinition file: line:78
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CSimplePointsMap::TMapDefinition, std::shared_ptr<mrpt::maps::CSimplePointsMap::TMapDefinition>, PyCallBack_mrpt_maps_CSimplePointsMap_TMapDefinition, mrpt::maps::CSimplePointsMap::TMapDefinitionBase> cl(enclosing_class, "TMapDefinition", "");
			cl.def( pybind11::init( [](){ return new mrpt::maps::CSimplePointsMap::TMapDefinition(); }, [](){ return new PyCallBack_mrpt_maps_CSimplePointsMap_TMapDefinition(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_maps_CSimplePointsMap_TMapDefinition const &o){ return new PyCallBack_mrpt_maps_CSimplePointsMap_TMapDefinition(o); } ) );
			cl.def( pybind11::init( [](mrpt::maps::CSimplePointsMap::TMapDefinition const &o){ return new mrpt::maps::CSimplePointsMap::TMapDefinition(o); } ) );
			cl.def_readwrite("insertionOpts", &mrpt::maps::CSimplePointsMap::TMapDefinition::insertionOpts);
			cl.def_readwrite("likelihoodOpts", &mrpt::maps::CSimplePointsMap::TMapDefinition::likelihoodOpts);
			cl.def_readwrite("renderOpts", &mrpt::maps::CSimplePointsMap::TMapDefinition::renderOpts);
		}

	}
}
