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
#include <mrpt/maps/CColouredOctoMap.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/COctoMapBase.h>
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
#include <mrpt/obs/CObservationImage.h>
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

// mrpt::maps::CColouredOctoMap file:mrpt/maps/CColouredOctoMap.h line:38
struct PyCallBack_mrpt_maps_CColouredOctoMap : public mrpt::maps::CColouredOctoMap {
	using mrpt::maps::CColouredOctoMap::CColouredOctoMap;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredOctoMap *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CColouredOctoMap::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredOctoMap *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CColouredOctoMap::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredOctoMap *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CColouredOctoMap::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredOctoMap *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CColouredOctoMap::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredOctoMap *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CColouredOctoMap::serializeFrom(a0, a1);
	}
	void getAsOctoMapVoxels(class mrpt::opengl::COctoMapVoxels & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredOctoMap *>(this), "getAsOctoMapVoxels");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CColouredOctoMap::getAsOctoMapVoxels(a0);
	}
	bool isEmpty() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredOctoMap *>(this), "isEmpty");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CColouredOctoMap::isEmpty();
	}
	void setOccupancyThres(double a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredOctoMap *>(this), "setOccupancyThres");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CColouredOctoMap::setOccupancyThres(a0);
	}
	void setProbHit(double a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredOctoMap *>(this), "setProbHit");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CColouredOctoMap::setProbHit(a0);
	}
	void setProbMiss(double a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredOctoMap *>(this), "setProbMiss");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CColouredOctoMap::setProbMiss(a0);
	}
	void setClampingThresMin(double a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredOctoMap *>(this), "setClampingThresMin");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CColouredOctoMap::setClampingThresMin(a0);
	}
	void setClampingThresMax(double a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredOctoMap *>(this), "setClampingThresMax");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CColouredOctoMap::setClampingThresMax(a0);
	}
	double getOccupancyThres() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredOctoMap *>(this), "getOccupancyThres");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CColouredOctoMap::getOccupancyThres();
	}
	float getOccupancyThresLog() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredOctoMap *>(this), "getOccupancyThresLog");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<float>::value) {
				static pybind11::detail::override_caster_t<float> caster;
				return pybind11::detail::cast_ref<float>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<float>(std::move(o));
		}
		return CColouredOctoMap::getOccupancyThresLog();
	}
	double getProbHit() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredOctoMap *>(this), "getProbHit");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CColouredOctoMap::getProbHit();
	}
	float getProbHitLog() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredOctoMap *>(this), "getProbHitLog");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<float>::value) {
				static pybind11::detail::override_caster_t<float> caster;
				return pybind11::detail::cast_ref<float>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<float>(std::move(o));
		}
		return CColouredOctoMap::getProbHitLog();
	}
	double getProbMiss() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredOctoMap *>(this), "getProbMiss");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CColouredOctoMap::getProbMiss();
	}
	float getProbMissLog() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredOctoMap *>(this), "getProbMissLog");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<float>::value) {
				static pybind11::detail::override_caster_t<float> caster;
				return pybind11::detail::cast_ref<float>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<float>(std::move(o));
		}
		return CColouredOctoMap::getProbMissLog();
	}
	double getClampingThresMin() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredOctoMap *>(this), "getClampingThresMin");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CColouredOctoMap::getClampingThresMin();
	}
	float getClampingThresMinLog() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredOctoMap *>(this), "getClampingThresMinLog");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<float>::value) {
				static pybind11::detail::override_caster_t<float> caster;
				return pybind11::detail::cast_ref<float>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<float>(std::move(o));
		}
		return CColouredOctoMap::getClampingThresMinLog();
	}
	double getClampingThresMax() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredOctoMap *>(this), "getClampingThresMax");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CColouredOctoMap::getClampingThresMax();
	}
	float getClampingThresMaxLog() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredOctoMap *>(this), "getClampingThresMaxLog");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<float>::value) {
				static pybind11::detail::override_caster_t<float> caster;
				return pybind11::detail::cast_ref<float>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<float>(std::move(o));
		}
		return CColouredOctoMap::getClampingThresMaxLog();
	}
	void internal_clear() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredOctoMap *>(this), "internal_clear");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CColouredOctoMap::internal_clear();
	}
	std::string asString() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredOctoMap *>(this), "asString");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredOctoMap *>(this), "saveMetricMapRepresentationToFile");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredOctoMap *>(this), "getVisualizationInto");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredOctoMap *>(this), "boundingBox");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredOctoMap *>(this), "canComputeObservationLikelihood");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredOctoMap *>(this), "determineMatching2D");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredOctoMap *>(this), "determineMatching3D");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredOctoMap *>(this), "compute3DMatchingRatio");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredOctoMap *>(this), "auxParticleFilterCleanUp");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredOctoMap *>(this), "squareDistanceToClosestCorrespondence");
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

// mrpt::maps::CColouredOctoMap::TMapDefinition file: line:78
struct PyCallBack_mrpt_maps_CColouredOctoMap_TMapDefinition : public mrpt::maps::CColouredOctoMap::TMapDefinition {
	using mrpt::maps::CColouredOctoMap::TMapDefinition::TMapDefinition;

	void loadFromConfigFile_map_specific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredOctoMap::TMapDefinition *>(this), "loadFromConfigFile_map_specific");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredOctoMap::TMapDefinition *>(this), "loadFromConfigFile");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredOctoMap::TMapDefinition *>(this), "saveToConfigFile");
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

// mrpt::maps::CColouredPointsMap file:mrpt/maps/CColouredPointsMap.h line:30
struct PyCallBack_mrpt_maps_CColouredPointsMap : public mrpt::maps::CColouredPointsMap {
	using mrpt::maps::CColouredPointsMap::CColouredPointsMap;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CColouredPointsMap::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CColouredPointsMap::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CColouredPointsMap::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CColouredPointsMap::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CColouredPointsMap::serializeFrom(a0, a1);
	}
	void reserve(size_t a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "reserve");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CColouredPointsMap::reserve(a0);
	}
	void resize(size_t a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "resize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CColouredPointsMap::resize(a0);
	}
	void setSize(size_t a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "setSize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CColouredPointsMap::setSize(a0);
	}
	void insertPointFast(float a0, float a1, float a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "insertPointFast");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CColouredPointsMap::insertPointFast(a0, a1, a2);
	}
	void addFrom_classSpecific(const class mrpt::maps::CPointsMap & a0, size_t a1, const bool a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "addFrom_classSpecific");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CColouredPointsMap::addFrom_classSpecific(a0, a1, a2);
	}
	void setPointRGB(size_t a0, float a1, float a2, float a3, float a4, float a5, float a6) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "setPointRGB");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4, a5, a6);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CColouredPointsMap::setPointRGB(a0, a1, a2, a3, a4, a5, a6);
	}
	void insertPointRGB(float a0, float a1, float a2, float a3, float a4, float a5) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "insertPointRGB");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4, a5);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CColouredPointsMap::insertPointRGB(a0, a1, a2, a3, a4, a5);
	}
	void getPointRGB(size_t a0, float & a1, float & a2, float & a3, float & a4, float & a5, float & a6) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "getPointRGB");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4, a5, a6);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CColouredPointsMap::getPointRGB(a0, a1, a2, a3, a4, a5, a6);
	}
	bool hasColorPoints() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "hasColorPoints");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CColouredPointsMap::hasColorPoints();
	}
	void getVisualizationInto(class mrpt::opengl::CSetOfObjects & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "getVisualizationInto");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CColouredPointsMap::getVisualizationInto(a0);
	}
	void insertPointField_color_R(float a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "insertPointField_color_R");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CColouredPointsMap::insertPointField_color_R(a0);
	}
	void insertPointField_color_G(float a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "insertPointField_color_G");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CColouredPointsMap::insertPointField_color_G(a0);
	}
	void insertPointField_color_B(float a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "insertPointField_color_B");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CColouredPointsMap::insertPointField_color_B(a0);
	}
	void internal_clear() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "internal_clear");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CColouredPointsMap::internal_clear();
	}
	void PLY_import_set_vertex(size_t a0, const struct mrpt::math::TPoint3D_<float> & a1, const struct mrpt::img::TColorf * a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "PLY_import_set_vertex");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CColouredPointsMap::PLY_import_set_vertex(a0, a1, a2);
	}
	void PLY_import_set_vertex_count(size_t a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "PLY_import_set_vertex_count");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CColouredPointsMap::PLY_import_set_vertex_count(a0);
	}
	void PLY_import_set_vertex_timestamp(size_t a0, const double a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "PLY_import_set_vertex_timestamp");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CColouredPointsMap::PLY_import_set_vertex_timestamp(a0, a1);
	}
	void PLY_export_get_vertex(size_t a0, struct mrpt::math::TPoint3D_<float> & a1, bool & a2, struct mrpt::img::TColorf & a3) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "PLY_export_get_vertex");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CColouredPointsMap::PLY_export_get_vertex(a0, a1, a2, a3);
	}
	float squareDistanceToClosestCorrespondence(float a0, float a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "squareDistanceToClosestCorrespondence");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "saveMetricMapRepresentationToFile");
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
	void setPointWeight(size_t a0, unsigned long a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "setPointWeight");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "getPointWeight");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "insertPointField_Intensity");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "insertPointField_Ring");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "insertPointField_Timestamp");
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
	void determineMatching2D(const class mrpt::maps::CMetricMap * a0, const class mrpt::poses::CPose2D & a1, class mrpt::tfest::TMatchingPairListTempl<float> & a2, const struct mrpt::maps::TMatchingParams & a3, struct mrpt::maps::TMatchingExtraResults & a4) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "determineMatching2D");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "determineMatching3D");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "compute3DMatchingRatio");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "isEmpty");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "boundingBox");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "internal_computeObservationLikelihood");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "asString");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "nn_prepare_for_2d_queries");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "nn_prepare_for_3d_queries");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "nn_has_indices_or_ids");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "nn_index_count");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "nn_single_search");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "nn_single_search");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "PLY_import_set_face_count");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "PLY_export_get_vertex_count");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "PLY_export_get_face_count");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "canComputeObservationLikelihood");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap *>(this), "auxParticleFilterCleanUp");
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

// mrpt::maps::CColouredPointsMap::TColourOptions file:mrpt/maps/CColouredPointsMap.h line:194
struct PyCallBack_mrpt_maps_CColouredPointsMap_TColourOptions : public mrpt::maps::CColouredPointsMap::TColourOptions {
	using mrpt::maps::CColouredPointsMap::TColourOptions::TColourOptions;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap::TColourOptions *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TColourOptions::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap::TColourOptions *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CLoadableOptions::saveToConfigFile(a0, a1);
	}
};

// mrpt::maps::CColouredPointsMap::TMapDefinition file: line:78
struct PyCallBack_mrpt_maps_CColouredPointsMap_TMapDefinition : public mrpt::maps::CColouredPointsMap::TMapDefinition {
	using mrpt::maps::CColouredPointsMap::TMapDefinition::TMapDefinition;

	void loadFromConfigFile_map_specific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap::TMapDefinition *>(this), "loadFromConfigFile_map_specific");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap::TMapDefinition *>(this), "loadFromConfigFile");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CColouredPointsMap::TMapDefinition *>(this), "saveToConfigFile");
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

void bind_mrpt_maps_CColouredOctoMap(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::maps::CColouredOctoMap file:mrpt/maps/CColouredOctoMap.h line:38
		pybind11::class_<mrpt::maps::CColouredOctoMap, std::shared_ptr<mrpt::maps::CColouredOctoMap>, PyCallBack_mrpt_maps_CColouredOctoMap, mrpt::maps::COctoMapBase<octomap::ColorOcTree,octomap::ColorOcTreeNode>> cl(M("mrpt::maps"), "CColouredOctoMap", "A three-dimensional probabilistic occupancy grid, implemented as an\n octo-tree with the \"octomap\" C++ library.\n  This version stores both, occupancy information and RGB colour data at\n each octree node. See the base class mrpt::maps::COctoMapBase.\n\n The octomap library was presented in \n\n \n CMetricMap, the example in \"MRPT/samples/octomap_simple\"\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::maps::CColouredOctoMap(); }, [](){ return new PyCallBack_mrpt_maps_CColouredOctoMap(); } ), "doc");
		cl.def( pybind11::init<const double>(), pybind11::arg("resolution") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_maps_CColouredOctoMap const &o){ return new PyCallBack_mrpt_maps_CColouredOctoMap(o); } ) );
		cl.def( pybind11::init( [](mrpt::maps::CColouredOctoMap const &o){ return new mrpt::maps::CColouredOctoMap(o); } ) );

		pybind11::enum_<mrpt::maps::CColouredOctoMap::TColourUpdate>(cl, "TColourUpdate", pybind11::arithmetic(), "This allows the user to select the desired method to update voxels\n   colour.\n    SET = Set the colour of the voxel at (x,y,z) directly\n    AVERAGE = Set the colour of the voxel at (x,y,z) as the mean of\n   its previous colour and the new observed one.\n    INTEGRATE = Calculate the new colour of the voxel at (x,y,z) using\n   this formula: prev_color*node_prob +  new_color*(0.99-node_prob)\n    If there isn't any previous color, any method is equivalent to\n   SET.\n    INTEGRATE is the default option")
			.value("INTEGRATE", mrpt::maps::CColouredOctoMap::INTEGRATE)
			.value("SET", mrpt::maps::CColouredOctoMap::SET)
			.value("AVERAGE", mrpt::maps::CColouredOctoMap::AVERAGE)
			.export_values();

		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::maps::CColouredOctoMap::GetRuntimeClassIdStatic, "C++: mrpt::maps::CColouredOctoMap::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::maps::CColouredOctoMap::*)() const) &mrpt::maps::CColouredOctoMap::GetRuntimeClass, "C++: mrpt::maps::CColouredOctoMap::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::maps::CColouredOctoMap::*)() const) &mrpt::maps::CColouredOctoMap::clone, "C++: mrpt::maps::CColouredOctoMap::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::maps::CColouredOctoMap::CreateObject, "C++: mrpt::maps::CColouredOctoMap::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("getPointColour", (bool (mrpt::maps::CColouredOctoMap::*)(const float, const float, const float, unsigned char &, unsigned char &, unsigned char &) const) &mrpt::maps::CColouredOctoMap::getPointColour, "Get the RGB colour of a point\n \n\n false if the point is not mapped, in which case the\n returned colour is undefined. \n\nC++: mrpt::maps::CColouredOctoMap::getPointColour(const float, const float, const float, unsigned char &, unsigned char &, unsigned char &) const --> bool", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"));
		cl.def("updateVoxelColour", (void (mrpt::maps::CColouredOctoMap::*)(const double, const double, const double, const unsigned char, const unsigned char, const unsigned char)) &mrpt::maps::CColouredOctoMap::updateVoxelColour, "Manually update the colour of the voxel at (x,y,z) \n\nC++: mrpt::maps::CColouredOctoMap::updateVoxelColour(const double, const double, const double, const unsigned char, const unsigned char, const unsigned char) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"));
		cl.def("setVoxelColourMethod", (void (mrpt::maps::CColouredOctoMap::*)(enum mrpt::maps::CColouredOctoMap::TColourUpdate)) &mrpt::maps::CColouredOctoMap::setVoxelColourMethod, "Set the method used to update voxels colour\n\nC++: mrpt::maps::CColouredOctoMap::setVoxelColourMethod(enum mrpt::maps::CColouredOctoMap::TColourUpdate) --> void", pybind11::arg("new_method"));
		cl.def("getVoxelColourMethod", (enum mrpt::maps::CColouredOctoMap::TColourUpdate (mrpt::maps::CColouredOctoMap::*)()) &mrpt::maps::CColouredOctoMap::getVoxelColourMethod, "Get the method used to update voxels colour\n\nC++: mrpt::maps::CColouredOctoMap::getVoxelColourMethod() --> enum mrpt::maps::CColouredOctoMap::TColourUpdate");
		cl.def("getAsOctoMapVoxels", (void (mrpt::maps::CColouredOctoMap::*)(class mrpt::opengl::COctoMapVoxels &) const) &mrpt::maps::CColouredOctoMap::getAsOctoMapVoxels, "C++: mrpt::maps::CColouredOctoMap::getAsOctoMapVoxels(class mrpt::opengl::COctoMapVoxels &) const --> void", pybind11::arg("gl_obj"));
		cl.def("isEmpty", (bool (mrpt::maps::CColouredOctoMap::*)() const) &mrpt::maps::CColouredOctoMap::isEmpty, "Returns true if the map is empty/no observation has been inserted \n\nC++: mrpt::maps::CColouredOctoMap::isEmpty() const --> bool");
		cl.def("insertRay", (void (mrpt::maps::CColouredOctoMap::*)(const float, const float, const float, const float, const float, const float)) &mrpt::maps::CColouredOctoMap::insertRay, "Just like insertPointCloud but with a single ray. \n\nC++: mrpt::maps::CColouredOctoMap::insertRay(const float, const float, const float, const float, const float, const float) --> void", pybind11::arg("end_x"), pybind11::arg("end_y"), pybind11::arg("end_z"), pybind11::arg("sensor_x"), pybind11::arg("sensor_y"), pybind11::arg("sensor_z"));
		cl.def("updateVoxel", (void (mrpt::maps::CColouredOctoMap::*)(const double, const double, const double, bool)) &mrpt::maps::CColouredOctoMap::updateVoxel, "Manually updates the occupancy of the voxel at (x,y,z) as being occupied\n (true) or free (false), using the log-odds parameters in \n \n\nC++: mrpt::maps::CColouredOctoMap::updateVoxel(const double, const double, const double, bool) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("occupied"));
		cl.def("isPointWithinOctoMap", (bool (mrpt::maps::CColouredOctoMap::*)(const float, const float, const float) const) &mrpt::maps::CColouredOctoMap::isPointWithinOctoMap, "Check whether the given point lies within the volume covered by the\n octomap (that is, whether it is \"mapped\") \n\nC++: mrpt::maps::CColouredOctoMap::isPointWithinOctoMap(const float, const float, const float) const --> bool", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("getResolution", (double (mrpt::maps::CColouredOctoMap::*)() const) &mrpt::maps::CColouredOctoMap::getResolution, "C++: mrpt::maps::CColouredOctoMap::getResolution() const --> double");
		cl.def("getTreeDepth", (unsigned int (mrpt::maps::CColouredOctoMap::*)() const) &mrpt::maps::CColouredOctoMap::getTreeDepth, "C++: mrpt::maps::CColouredOctoMap::getTreeDepth() const --> unsigned int");
		cl.def("size", (size_t (mrpt::maps::CColouredOctoMap::*)() const) &mrpt::maps::CColouredOctoMap::size, "The number of nodes in the tree\n\nC++: mrpt::maps::CColouredOctoMap::size() const --> size_t");
		cl.def("memoryUsage", (size_t (mrpt::maps::CColouredOctoMap::*)() const) &mrpt::maps::CColouredOctoMap::memoryUsage, "Memory usage of the complete octree in bytes (may vary between\n architectures)\n\nC++: mrpt::maps::CColouredOctoMap::memoryUsage() const --> size_t");
		cl.def("memoryUsageNode", (size_t (mrpt::maps::CColouredOctoMap::*)() const) &mrpt::maps::CColouredOctoMap::memoryUsageNode, "Memory usage of the a single octree node\n\nC++: mrpt::maps::CColouredOctoMap::memoryUsageNode() const --> size_t");
		cl.def("memoryFullGrid", (size_t (mrpt::maps::CColouredOctoMap::*)() const) &mrpt::maps::CColouredOctoMap::memoryFullGrid, "Memory usage of a full grid of the same size as the OcTree in\n bytes (for comparison)\n\nC++: mrpt::maps::CColouredOctoMap::memoryFullGrid() const --> size_t");
		cl.def("volume", (double (mrpt::maps::CColouredOctoMap::*)()) &mrpt::maps::CColouredOctoMap::volume, "C++: mrpt::maps::CColouredOctoMap::volume() --> double");
		cl.def("getMetricSize", (void (mrpt::maps::CColouredOctoMap::*)(double &, double &, double &)) &mrpt::maps::CColouredOctoMap::getMetricSize, "Size of OcTree (all known space) in meters for x, y and z dimension\n\nC++: mrpt::maps::CColouredOctoMap::getMetricSize(double &, double &, double &) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("getMetricMin", (void (mrpt::maps::CColouredOctoMap::*)(double &, double &, double &)) &mrpt::maps::CColouredOctoMap::getMetricMin, "minimum value of the bounding box of all known space in x, y, z\n\nC++: mrpt::maps::CColouredOctoMap::getMetricMin(double &, double &, double &) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("getMetricMax", (void (mrpt::maps::CColouredOctoMap::*)(double &, double &, double &)) &mrpt::maps::CColouredOctoMap::getMetricMax, "maximum value of the bounding box of all known space in x, y, z\n\nC++: mrpt::maps::CColouredOctoMap::getMetricMax(double &, double &, double &) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("calcNumNodes", (size_t (mrpt::maps::CColouredOctoMap::*)() const) &mrpt::maps::CColouredOctoMap::calcNumNodes, "Traverses the tree to calculate the total number of nodes\n\nC++: mrpt::maps::CColouredOctoMap::calcNumNodes() const --> size_t");
		cl.def("getNumLeafNodes", (size_t (mrpt::maps::CColouredOctoMap::*)() const) &mrpt::maps::CColouredOctoMap::getNumLeafNodes, "Traverses the tree to calculate the total number of leaf nodes\n\nC++: mrpt::maps::CColouredOctoMap::getNumLeafNodes() const --> size_t");
		cl.def("setOccupancyThres", (void (mrpt::maps::CColouredOctoMap::*)(double)) &mrpt::maps::CColouredOctoMap::setOccupancyThres, "C++: mrpt::maps::CColouredOctoMap::setOccupancyThres(double) --> void", pybind11::arg("prob"));
		cl.def("setProbHit", (void (mrpt::maps::CColouredOctoMap::*)(double)) &mrpt::maps::CColouredOctoMap::setProbHit, "C++: mrpt::maps::CColouredOctoMap::setProbHit(double) --> void", pybind11::arg("prob"));
		cl.def("setProbMiss", (void (mrpt::maps::CColouredOctoMap::*)(double)) &mrpt::maps::CColouredOctoMap::setProbMiss, "C++: mrpt::maps::CColouredOctoMap::setProbMiss(double) --> void", pybind11::arg("prob"));
		cl.def("setClampingThresMin", (void (mrpt::maps::CColouredOctoMap::*)(double)) &mrpt::maps::CColouredOctoMap::setClampingThresMin, "C++: mrpt::maps::CColouredOctoMap::setClampingThresMin(double) --> void", pybind11::arg("thresProb"));
		cl.def("setClampingThresMax", (void (mrpt::maps::CColouredOctoMap::*)(double)) &mrpt::maps::CColouredOctoMap::setClampingThresMax, "C++: mrpt::maps::CColouredOctoMap::setClampingThresMax(double) --> void", pybind11::arg("thresProb"));
		cl.def("getOccupancyThres", (double (mrpt::maps::CColouredOctoMap::*)() const) &mrpt::maps::CColouredOctoMap::getOccupancyThres, "C++: mrpt::maps::CColouredOctoMap::getOccupancyThres() const --> double");
		cl.def("getOccupancyThresLog", (float (mrpt::maps::CColouredOctoMap::*)() const) &mrpt::maps::CColouredOctoMap::getOccupancyThresLog, "C++: mrpt::maps::CColouredOctoMap::getOccupancyThresLog() const --> float");
		cl.def("getProbHit", (double (mrpt::maps::CColouredOctoMap::*)() const) &mrpt::maps::CColouredOctoMap::getProbHit, "C++: mrpt::maps::CColouredOctoMap::getProbHit() const --> double");
		cl.def("getProbHitLog", (float (mrpt::maps::CColouredOctoMap::*)() const) &mrpt::maps::CColouredOctoMap::getProbHitLog, "C++: mrpt::maps::CColouredOctoMap::getProbHitLog() const --> float");
		cl.def("getProbMiss", (double (mrpt::maps::CColouredOctoMap::*)() const) &mrpt::maps::CColouredOctoMap::getProbMiss, "C++: mrpt::maps::CColouredOctoMap::getProbMiss() const --> double");
		cl.def("getProbMissLog", (float (mrpt::maps::CColouredOctoMap::*)() const) &mrpt::maps::CColouredOctoMap::getProbMissLog, "C++: mrpt::maps::CColouredOctoMap::getProbMissLog() const --> float");
		cl.def("getClampingThresMin", (double (mrpt::maps::CColouredOctoMap::*)() const) &mrpt::maps::CColouredOctoMap::getClampingThresMin, "C++: mrpt::maps::CColouredOctoMap::getClampingThresMin() const --> double");
		cl.def("getClampingThresMinLog", (float (mrpt::maps::CColouredOctoMap::*)() const) &mrpt::maps::CColouredOctoMap::getClampingThresMinLog, "C++: mrpt::maps::CColouredOctoMap::getClampingThresMinLog() const --> float");
		cl.def("getClampingThresMax", (double (mrpt::maps::CColouredOctoMap::*)() const) &mrpt::maps::CColouredOctoMap::getClampingThresMax, "C++: mrpt::maps::CColouredOctoMap::getClampingThresMax() const --> double");
		cl.def("getClampingThresMaxLog", (float (mrpt::maps::CColouredOctoMap::*)() const) &mrpt::maps::CColouredOctoMap::getClampingThresMaxLog, "C++: mrpt::maps::CColouredOctoMap::getClampingThresMaxLog() const --> float");
		cl.def("assign", (class mrpt::maps::CColouredOctoMap & (mrpt::maps::CColouredOctoMap::*)(const class mrpt::maps::CColouredOctoMap &)) &mrpt::maps::CColouredOctoMap::operator=, "C++: mrpt::maps::CColouredOctoMap::operator=(const class mrpt::maps::CColouredOctoMap &) --> class mrpt::maps::CColouredOctoMap &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::maps::CColouredOctoMap::TMapDefinitionBase file: line:74
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CColouredOctoMap::TMapDefinitionBase, std::shared_ptr<mrpt::maps::CColouredOctoMap::TMapDefinitionBase>> cl(enclosing_class, "TMapDefinitionBase", "");
		}

		{ // mrpt::maps::CColouredOctoMap::TMapDefinition file: line:78
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CColouredOctoMap::TMapDefinition, std::shared_ptr<mrpt::maps::CColouredOctoMap::TMapDefinition>, PyCallBack_mrpt_maps_CColouredOctoMap_TMapDefinition, mrpt::maps::CColouredOctoMap::TMapDefinitionBase> cl(enclosing_class, "TMapDefinition", "");
			cl.def( pybind11::init( [](){ return new mrpt::maps::CColouredOctoMap::TMapDefinition(); }, [](){ return new PyCallBack_mrpt_maps_CColouredOctoMap_TMapDefinition(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_maps_CColouredOctoMap_TMapDefinition const &o){ return new PyCallBack_mrpt_maps_CColouredOctoMap_TMapDefinition(o); } ) );
			cl.def( pybind11::init( [](mrpt::maps::CColouredOctoMap::TMapDefinition const &o){ return new mrpt::maps::CColouredOctoMap::TMapDefinition(o); } ) );
			cl.def_readwrite("resolution", &mrpt::maps::CColouredOctoMap::TMapDefinition::resolution);
			cl.def_readwrite("insertionOpts", &mrpt::maps::CColouredOctoMap::TMapDefinition::insertionOpts);
			cl.def_readwrite("likelihoodOpts", &mrpt::maps::CColouredOctoMap::TMapDefinition::likelihoodOpts);
		}

	}
	{ // mrpt::maps::CColouredPointsMap file:mrpt/maps/CColouredPointsMap.h line:30
		pybind11::class_<mrpt::maps::CColouredPointsMap, std::shared_ptr<mrpt::maps::CColouredPointsMap>, PyCallBack_mrpt_maps_CColouredPointsMap, mrpt::maps::CPointsMap> cl(M("mrpt::maps"), "CColouredPointsMap", "A map of 2D/3D points with individual colours (RGB).\n  For different color schemes, see CColouredPointsMap::colorScheme\n  Colors are defined in the range [0,1].\n \n\n mrpt::maps::CPointsMap, mrpt::maps::CMetricMap,\n mrpt::serialization::CSerializable\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::maps::CColouredPointsMap(); }, [](){ return new PyCallBack_mrpt_maps_CColouredPointsMap(); } ) );
		cl.def( pybind11::init<const class mrpt::maps::CPointsMap &>(), pybind11::arg("o") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_maps_CColouredPointsMap const &o){ return new PyCallBack_mrpt_maps_CColouredPointsMap(o); } ) );
		cl.def( pybind11::init( [](mrpt::maps::CColouredPointsMap const &o){ return new mrpt::maps::CColouredPointsMap(o); } ) );

		pybind11::enum_<mrpt::maps::CColouredPointsMap::TColouringMethod>(cl, "TColouringMethod", pybind11::arithmetic(), "The choices for coloring schemes:\n		- cmFromHeightRelativeToSensor: The Z coordinate wrt the sensor will\nbe\nused to obtain the color using the limits z_min,z_max.\n 	- cmFromIntensityImage: When inserting 3D range scans, take the\ncolor\nfrom the intensity image channel, if available.\n \n\n TColourOptions")
			.value("cmFromHeightRelativeToSensor", mrpt::maps::CColouredPointsMap::cmFromHeightRelativeToSensor)
			.value("cmFromHeightRelativeToSensorJet", mrpt::maps::CColouredPointsMap::cmFromHeightRelativeToSensorJet)
			.value("cmFromHeightRelativeToSensorGray", mrpt::maps::CColouredPointsMap::cmFromHeightRelativeToSensorGray)
			.value("cmFromIntensityImage", mrpt::maps::CColouredPointsMap::cmFromIntensityImage)
			.export_values();

		cl.def_readwrite("colorScheme", &mrpt::maps::CColouredPointsMap::colorScheme);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::maps::CColouredPointsMap::GetRuntimeClassIdStatic, "C++: mrpt::maps::CColouredPointsMap::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::maps::CColouredPointsMap::*)() const) &mrpt::maps::CColouredPointsMap::GetRuntimeClass, "C++: mrpt::maps::CColouredPointsMap::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::maps::CColouredPointsMap::*)() const) &mrpt::maps::CColouredPointsMap::clone, "C++: mrpt::maps::CColouredPointsMap::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::maps::CColouredPointsMap::CreateObject, "C++: mrpt::maps::CColouredPointsMap::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("assign", (class mrpt::maps::CColouredPointsMap & (mrpt::maps::CColouredPointsMap::*)(const class mrpt::maps::CPointsMap &)) &mrpt::maps::CColouredPointsMap::operator=, "C++: mrpt::maps::CColouredPointsMap::operator=(const class mrpt::maps::CPointsMap &) --> class mrpt::maps::CColouredPointsMap &", pybind11::return_value_policy::automatic, pybind11::arg("o"));
		cl.def("assign", (class mrpt::maps::CColouredPointsMap & (mrpt::maps::CColouredPointsMap::*)(const class mrpt::maps::CColouredPointsMap &)) &mrpt::maps::CColouredPointsMap::operator=, "C++: mrpt::maps::CColouredPointsMap::operator=(const class mrpt::maps::CColouredPointsMap &) --> class mrpt::maps::CColouredPointsMap &", pybind11::return_value_policy::automatic, pybind11::arg("o"));
		cl.def("reserve", (void (mrpt::maps::CColouredPointsMap::*)(size_t)) &mrpt::maps::CColouredPointsMap::reserve, "from CPointsMap\n    @{ \n\nC++: mrpt::maps::CColouredPointsMap::reserve(size_t) --> void", pybind11::arg("newLength"));
		cl.def("resize", (void (mrpt::maps::CColouredPointsMap::*)(size_t)) &mrpt::maps::CColouredPointsMap::resize, "C++: mrpt::maps::CColouredPointsMap::resize(size_t) --> void", pybind11::arg("newLength"));
		cl.def("setSize", (void (mrpt::maps::CColouredPointsMap::*)(size_t)) &mrpt::maps::CColouredPointsMap::setSize, "C++: mrpt::maps::CColouredPointsMap::setSize(size_t) --> void", pybind11::arg("newLength"));
		cl.def("insertPointFast", [](mrpt::maps::CColouredPointsMap &o, float const & a0, float const & a1) -> void { return o.insertPointFast(a0, a1); }, "", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("insertPointFast", (void (mrpt::maps::CColouredPointsMap::*)(float, float, float)) &mrpt::maps::CColouredPointsMap::insertPointFast, "The virtual method for  *without* calling\n mark_as_modified()   \n\nC++: mrpt::maps::CColouredPointsMap::insertPointFast(float, float, float) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("save3D_and_colour_to_text_file", (bool (mrpt::maps::CColouredPointsMap::*)(const std::string &) const) &mrpt::maps::CColouredPointsMap::save3D_and_colour_to_text_file, "Save to a text file. In each line contains X Y Z (meters) R G B (range\n [0,1]) for each point in the map.\n     Returns false if any error occured, true elsewere.\n\nC++: mrpt::maps::CColouredPointsMap::save3D_and_colour_to_text_file(const std::string &) const --> bool", pybind11::arg("file"));
		cl.def("setPointRGB", (void (mrpt::maps::CColouredPointsMap::*)(size_t, float, float, float, float, float, float)) &mrpt::maps::CColouredPointsMap::setPointRGB, "Changes a given point from map. First index is 0.\n \n\n Throws std::exception on index out of bound.\n\nC++: mrpt::maps::CColouredPointsMap::setPointRGB(size_t, float, float, float, float, float, float) --> void", pybind11::arg("index"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("R"), pybind11::arg("G"), pybind11::arg("B"));
		cl.def("insertPointRGB", (void (mrpt::maps::CColouredPointsMap::*)(float, float, float, float, float, float)) &mrpt::maps::CColouredPointsMap::insertPointRGB, "Adds a new point given its coordinates and color (colors range is [0,1])\n\nC++: mrpt::maps::CColouredPointsMap::insertPointRGB(float, float, float, float, float, float) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("R"), pybind11::arg("G"), pybind11::arg("B"));
		cl.def("setPointColor", (void (mrpt::maps::CColouredPointsMap::*)(size_t, float, float, float)) &mrpt::maps::CColouredPointsMap::setPointColor, "Changes just the color of a given point from the map. First index is 0.\n \n\n Throws std::exception on index out of bound.\n\nC++: mrpt::maps::CColouredPointsMap::setPointColor(size_t, float, float, float) --> void", pybind11::arg("index"), pybind11::arg("R"), pybind11::arg("G"), pybind11::arg("B"));
		cl.def("setPointColor_fast", (void (mrpt::maps::CColouredPointsMap::*)(size_t, float, float, float)) &mrpt::maps::CColouredPointsMap::setPointColor_fast, "Like  but without checking for out-of-index erors \n\nC++: mrpt::maps::CColouredPointsMap::setPointColor_fast(size_t, float, float, float) --> void", pybind11::arg("index"), pybind11::arg("R"), pybind11::arg("G"), pybind11::arg("B"));
		cl.def("getPointRGB", (void (mrpt::maps::CColouredPointsMap::*)(size_t, float &, float &, float &, float &, float &, float &) const) &mrpt::maps::CColouredPointsMap::getPointRGB, "Retrieves a point and its color (colors range is [0,1])\n\nC++: mrpt::maps::CColouredPointsMap::getPointRGB(size_t, float &, float &, float &, float &, float &, float &) const --> void", pybind11::arg("index"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("R"), pybind11::arg("G"), pybind11::arg("B"));
		cl.def("getPointColor", (void (mrpt::maps::CColouredPointsMap::*)(size_t, float &, float &, float &) const) &mrpt::maps::CColouredPointsMap::getPointColor, "Retrieves a point color (colors range is [0,1]) \n\nC++: mrpt::maps::CColouredPointsMap::getPointColor(size_t, float &, float &, float &) const --> void", pybind11::arg("index"), pybind11::arg("R"), pybind11::arg("G"), pybind11::arg("B"));
		cl.def("getPointColor_fast", (void (mrpt::maps::CColouredPointsMap::*)(size_t, float &, float &, float &) const) &mrpt::maps::CColouredPointsMap::getPointColor_fast, "Like  but without checking for out-of-index erors \n\nC++: mrpt::maps::CColouredPointsMap::getPointColor_fast(size_t, float &, float &, float &) const --> void", pybind11::arg("index"), pybind11::arg("R"), pybind11::arg("G"), pybind11::arg("B"));
		cl.def("hasColorPoints", (bool (mrpt::maps::CColouredPointsMap::*)() const) &mrpt::maps::CColouredPointsMap::hasColorPoints, "Returns true if the point map has a color field for each point \n\nC++: mrpt::maps::CColouredPointsMap::hasColorPoints() const --> bool");
		cl.def("getVisualizationInto", (void (mrpt::maps::CColouredPointsMap::*)(class mrpt::opengl::CSetOfObjects &) const) &mrpt::maps::CColouredPointsMap::getVisualizationInto, "Override of the default 3D scene builder to account for the individual\n points' color.\n\nC++: mrpt::maps::CColouredPointsMap::getVisualizationInto(class mrpt::opengl::CSetOfObjects &) const --> void", pybind11::arg("outObj"));
		cl.def("colourFromObservation", (bool (mrpt::maps::CColouredPointsMap::*)(const class mrpt::obs::CObservationImage &, const class mrpt::poses::CPose3D &)) &mrpt::maps::CColouredPointsMap::colourFromObservation, "Colour a set of points from a CObservationImage and the global pose of\n the robot \n\nC++: mrpt::maps::CColouredPointsMap::colourFromObservation(const class mrpt::obs::CObservationImage &, const class mrpt::poses::CPose3D &) --> bool", pybind11::arg("obs"), pybind11::arg("robotPose"));
		cl.def("resetPointsMinDist", [](mrpt::maps::CColouredPointsMap &o) -> void { return o.resetPointsMinDist(); }, "");
		cl.def("resetPointsMinDist", (void (mrpt::maps::CColouredPointsMap::*)(float)) &mrpt::maps::CColouredPointsMap::resetPointsMinDist, "Reset the minimum-observed-distance buffer for all the points to a\n predefined value \n\nC++: mrpt::maps::CColouredPointsMap::resetPointsMinDist(float) --> void", pybind11::arg("defValue"));
		cl.def("insertPointField_color_R", (void (mrpt::maps::CColouredPointsMap::*)(float)) &mrpt::maps::CColouredPointsMap::insertPointField_color_R, "C++: mrpt::maps::CColouredPointsMap::insertPointField_color_R(float) --> void", pybind11::arg("v"));
		cl.def("insertPointField_color_G", (void (mrpt::maps::CColouredPointsMap::*)(float)) &mrpt::maps::CColouredPointsMap::insertPointField_color_G, "C++: mrpt::maps::CColouredPointsMap::insertPointField_color_G(float) --> void", pybind11::arg("v"));
		cl.def("insertPointField_color_B", (void (mrpt::maps::CColouredPointsMap::*)(float)) &mrpt::maps::CColouredPointsMap::insertPointField_color_B, "C++: mrpt::maps::CColouredPointsMap::insertPointField_color_B(float) --> void", pybind11::arg("v"));

		{ // mrpt::maps::CColouredPointsMap::TColourOptions file:mrpt/maps/CColouredPointsMap.h line:194
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CColouredPointsMap::TColourOptions, std::shared_ptr<mrpt::maps::CColouredPointsMap::TColourOptions>, PyCallBack_mrpt_maps_CColouredPointsMap_TColourOptions, mrpt::config::CLoadableOptions> cl(enclosing_class, "TColourOptions", "The definition of parameters for generating colors from laser scans ");
			cl.def( pybind11::init( [](){ return new mrpt::maps::CColouredPointsMap::TColourOptions(); }, [](){ return new PyCallBack_mrpt_maps_CColouredPointsMap_TColourOptions(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_maps_CColouredPointsMap_TColourOptions const &o){ return new PyCallBack_mrpt_maps_CColouredPointsMap_TColourOptions(o); } ) );
			cl.def( pybind11::init( [](mrpt::maps::CColouredPointsMap::TColourOptions const &o){ return new mrpt::maps::CColouredPointsMap::TColourOptions(o); } ) );
			cl.def_readwrite("scheme", &mrpt::maps::CColouredPointsMap::TColourOptions::scheme);
			cl.def_readwrite("z_min", &mrpt::maps::CColouredPointsMap::TColourOptions::z_min);
			cl.def_readwrite("z_max", &mrpt::maps::CColouredPointsMap::TColourOptions::z_max);
			cl.def_readwrite("d_max", &mrpt::maps::CColouredPointsMap::TColourOptions::d_max);
			cl.def("loadFromConfigFile", (void (mrpt::maps::CColouredPointsMap::TColourOptions::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::maps::CColouredPointsMap::TColourOptions::loadFromConfigFile, "C++: mrpt::maps::CColouredPointsMap::TColourOptions::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
			cl.def("assign", (struct mrpt::maps::CColouredPointsMap::TColourOptions & (mrpt::maps::CColouredPointsMap::TColourOptions::*)(const struct mrpt::maps::CColouredPointsMap::TColourOptions &)) &mrpt::maps::CColouredPointsMap::TColourOptions::operator=, "C++: mrpt::maps::CColouredPointsMap::TColourOptions::operator=(const struct mrpt::maps::CColouredPointsMap::TColourOptions &) --> struct mrpt::maps::CColouredPointsMap::TColourOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::maps::CColouredPointsMap::TMapDefinitionBase file: line:74
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CColouredPointsMap::TMapDefinitionBase, std::shared_ptr<mrpt::maps::CColouredPointsMap::TMapDefinitionBase>> cl(enclosing_class, "TMapDefinitionBase", "");
		}

		{ // mrpt::maps::CColouredPointsMap::TMapDefinition file: line:78
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CColouredPointsMap::TMapDefinition, std::shared_ptr<mrpt::maps::CColouredPointsMap::TMapDefinition>, PyCallBack_mrpt_maps_CColouredPointsMap_TMapDefinition, mrpt::maps::CColouredPointsMap::TMapDefinitionBase> cl(enclosing_class, "TMapDefinition", "");
			cl.def( pybind11::init( [](){ return new mrpt::maps::CColouredPointsMap::TMapDefinition(); }, [](){ return new PyCallBack_mrpt_maps_CColouredPointsMap_TMapDefinition(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_maps_CColouredPointsMap_TMapDefinition const &o){ return new PyCallBack_mrpt_maps_CColouredPointsMap_TMapDefinition(o); } ) );
			cl.def( pybind11::init( [](mrpt::maps::CColouredPointsMap::TMapDefinition const &o){ return new mrpt::maps::CColouredPointsMap::TMapDefinition(o); } ) );
			cl.def_readwrite("insertionOpts", &mrpt::maps::CColouredPointsMap::TMapDefinition::insertionOpts);
			cl.def_readwrite("likelihoodOpts", &mrpt::maps::CColouredPointsMap::TMapDefinition::likelihoodOpts);
			cl.def_readwrite("colourOpts", &mrpt::maps::CColouredPointsMap::TMapDefinition::colourOpts);
		}

	}
}
