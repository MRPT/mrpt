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
#include <mrpt/containers/CDynamicGrid.h>
#include <mrpt/containers/CommentPosition.h>
#include <mrpt/containers/YamlEmitOptions.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/Clock.h>
#include <mrpt/core/aligned_allocator.h>
#include <mrpt/core/safe_pointers.h>
#include <mrpt/img/CCanvas.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/img/TColor.h>
#include <mrpt/img/TPixelCoord.h>
#include <mrpt/io/CStream.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/NearestNeighborsCapable.h>
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
#include <mrpt/obs/CObservationRange.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/T2DScanProperties.h>
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

// mrpt::maps::COccupancyGridMap2D file:mrpt/maps/COccupancyGridMap2D.h line:54
struct PyCallBack_mrpt_maps_COccupancyGridMap2D : public mrpt::maps::COccupancyGridMap2D {
	using mrpt::maps::COccupancyGridMap2D::COccupancyGridMap2D;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap2D *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return COccupancyGridMap2D::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap2D *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return COccupancyGridMap2D::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap2D *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return COccupancyGridMap2D::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap2D *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COccupancyGridMap2D::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap2D *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COccupancyGridMap2D::serializeFrom(a0, a1);
	}
	void OnPostSuccesfulInsertObs(const class mrpt::obs::CObservation & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap2D *>(this), "OnPostSuccesfulInsertObs");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COccupancyGridMap2D::OnPostSuccesfulInsertObs(a0);
	}
	void internal_clear() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap2D *>(this), "internal_clear");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COccupancyGridMap2D::internal_clear();
	}
	struct mrpt::math::TBoundingBox_<float> boundingBox() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap2D *>(this), "boundingBox");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TBoundingBox_<float>>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TBoundingBox_<float>> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TBoundingBox_<float>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TBoundingBox_<float>>(std::move(o));
		}
		return COccupancyGridMap2D::boundingBox();
	}
	void getVisualizationInto(class mrpt::opengl::CSetOfObjects & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap2D *>(this), "getVisualizationInto");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COccupancyGridMap2D::getVisualizationInto(a0);
	}
	bool isEmpty() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap2D *>(this), "isEmpty");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return COccupancyGridMap2D::isEmpty();
	}
	void determineMatching2D(const class mrpt::maps::CMetricMap * a0, const class mrpt::poses::CPose2D & a1, class mrpt::tfest::TMatchingPairListTempl<float> & a2, const struct mrpt::maps::TMatchingParams & a3, struct mrpt::maps::TMatchingExtraResults & a4) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap2D *>(this), "determineMatching2D");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COccupancyGridMap2D::determineMatching2D(a0, a1, a2, a3, a4);
	}
	float compute3DMatchingRatio(const class mrpt::maps::CMetricMap * a0, const class mrpt::poses::CPose3D & a1, const struct mrpt::maps::TMatchingRatioParams & a2) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap2D *>(this), "compute3DMatchingRatio");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<float>::value) {
				static pybind11::detail::override_caster_t<float> caster;
				return pybind11::detail::cast_ref<float>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<float>(std::move(o));
		}
		return COccupancyGridMap2D::compute3DMatchingRatio(a0, a1, a2);
	}
	void saveMetricMapRepresentationToFile(const std::string & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap2D *>(this), "saveMetricMapRepresentationToFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COccupancyGridMap2D::saveMetricMapRepresentationToFile(a0);
	}
	std::string asString() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap2D *>(this), "asString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return COccupancyGridMap2D::asString();
	}
	bool nn_has_indices_or_ids() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap2D *>(this), "nn_has_indices_or_ids");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return COccupancyGridMap2D::nn_has_indices_or_ids();
	}
	size_t nn_index_count() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap2D *>(this), "nn_index_count");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return COccupancyGridMap2D::nn_index_count();
	}
	bool nn_single_search(const struct mrpt::math::TPoint3D_<float> & a0, struct mrpt::math::TPoint3D_<float> & a1, float & a2, uint64_t & a3) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap2D *>(this), "nn_single_search");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return COccupancyGridMap2D::nn_single_search(a0, a1, a2, a3);
	}
	bool nn_single_search(const struct mrpt::math::TPoint2D_<float> & a0, struct mrpt::math::TPoint2D_<float> & a1, float & a2, uint64_t & a3) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap2D *>(this), "nn_single_search");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return COccupancyGridMap2D::nn_single_search(a0, a1, a2, a3);
	}
	bool canComputeObservationLikelihood(const class mrpt::obs::CObservation & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap2D *>(this), "canComputeObservationLikelihood");
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
	void determineMatching3D(const class mrpt::maps::CMetricMap * a0, const class mrpt::poses::CPose3D & a1, class mrpt::tfest::TMatchingPairListTempl<float> & a2, const struct mrpt::maps::TMatchingParams & a3, struct mrpt::maps::TMatchingExtraResults & a4) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap2D *>(this), "determineMatching3D");
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
	void auxParticleFilterCleanUp() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap2D *>(this), "auxParticleFilterCleanUp");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap2D *>(this), "squareDistanceToClosestCorrespondence");
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
	void nn_prepare_for_2d_queries() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap2D *>(this), "nn_prepare_for_2d_queries");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return NearestNeighborsCapable::nn_prepare_for_2d_queries();
	}
	void nn_prepare_for_3d_queries() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap2D *>(this), "nn_prepare_for_3d_queries");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return NearestNeighborsCapable::nn_prepare_for_3d_queries();
	}
};

// mrpt::maps::COccupancyGridMap2D::TInsertionOptions file:mrpt/maps/COccupancyGridMap2D.h line:435
struct PyCallBack_mrpt_maps_COccupancyGridMap2D_TInsertionOptions : public mrpt::maps::COccupancyGridMap2D::TInsertionOptions {
	using mrpt::maps::COccupancyGridMap2D::TInsertionOptions::TInsertionOptions;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap2D::TInsertionOptions *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TInsertionOptions::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap2D::TInsertionOptions *>(this), "saveToConfigFile");
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

// mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions file:mrpt/maps/COccupancyGridMap2D.h line:515
struct PyCallBack_mrpt_maps_COccupancyGridMap2D_TLikelihoodOptions : public mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions {
	using mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions::TLikelihoodOptions;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TLikelihoodOptions::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions *>(this), "saveToConfigFile");
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

// mrpt::maps::COccupancyGridMap2D::TMapDefinition file: line:78
struct PyCallBack_mrpt_maps_COccupancyGridMap2D_TMapDefinition : public mrpt::maps::COccupancyGridMap2D::TMapDefinition {
	using mrpt::maps::COccupancyGridMap2D::TMapDefinition::TMapDefinition;

	void loadFromConfigFile_map_specific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap2D::TMapDefinition *>(this), "loadFromConfigFile_map_specific");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap2D::TMapDefinition *>(this), "loadFromConfigFile");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap2D::TMapDefinition *>(this), "saveToConfigFile");
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

void bind_mrpt_maps_COccupancyGridMap2D(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::maps::COccupancyGridMap2D file:mrpt/maps/COccupancyGridMap2D.h line:54
		pybind11::class_<mrpt::maps::COccupancyGridMap2D, std::shared_ptr<mrpt::maps::COccupancyGridMap2D>, PyCallBack_mrpt_maps_COccupancyGridMap2D, mrpt::maps::CMetricMap, mrpt::maps::CLogOddsGridMap2D<signed char>, mrpt::maps::NearestNeighborsCapable> cl(M("mrpt::maps"), "COccupancyGridMap2D", "A class for storing an occupancy grid map.\n  COccupancyGridMap2D is a class for storing a metric map\n   representation in the form of a probabilistic occupancy\n   grid map: value of 0 means certainly occupied, 1 means\n   a certainly empty cell. Initially 0.5 means uncertainty.\n\n The cells keep the log-odd representation of probabilities instead of the\nprobabilities themselves.\n  More details can be found at https://www.mrpt.org/Occupancy_Grids\n\n The algorithm for updating the grid from a laser scanner can optionally take\ninto account the progressive widening of the beams, as\n   described in [this page](http://www.mrpt.org/Occupancy_Grids)\n\n   Some implemented methods are:\n		- Update of individual cells\n		- Insertion of observations\n		- Voronoi diagram and critical points (\n		- Saving and loading from/to a bitmap\n		- Laser scans simulation for the map contents\n		- Entropy and information methods (See computeEntropy)\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::maps::COccupancyGridMap2D(); }, [](){ return new PyCallBack_mrpt_maps_COccupancyGridMap2D(); } ), "doc");
		cl.def( pybind11::init( [](float const & a0){ return new mrpt::maps::COccupancyGridMap2D(a0); }, [](float const & a0){ return new PyCallBack_mrpt_maps_COccupancyGridMap2D(a0); } ), "doc");
		cl.def( pybind11::init( [](float const & a0, float const & a1){ return new mrpt::maps::COccupancyGridMap2D(a0, a1); }, [](float const & a0, float const & a1){ return new PyCallBack_mrpt_maps_COccupancyGridMap2D(a0, a1); } ), "doc");
		cl.def( pybind11::init( [](float const & a0, float const & a1, float const & a2){ return new mrpt::maps::COccupancyGridMap2D(a0, a1, a2); }, [](float const & a0, float const & a1, float const & a2){ return new PyCallBack_mrpt_maps_COccupancyGridMap2D(a0, a1, a2); } ), "doc");
		cl.def( pybind11::init( [](float const & a0, float const & a1, float const & a2, float const & a3){ return new mrpt::maps::COccupancyGridMap2D(a0, a1, a2, a3); }, [](float const & a0, float const & a1, float const & a2, float const & a3){ return new PyCallBack_mrpt_maps_COccupancyGridMap2D(a0, a1, a2, a3); } ), "doc");
		cl.def( pybind11::init<float, float, float, float, float>(), pybind11::arg("min_x"), pybind11::arg("max_x"), pybind11::arg("min_y"), pybind11::arg("max_y"), pybind11::arg("resolution") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_maps_COccupancyGridMap2D const &o){ return new PyCallBack_mrpt_maps_COccupancyGridMap2D(o); } ) );
		cl.def( pybind11::init( [](mrpt::maps::COccupancyGridMap2D const &o){ return new mrpt::maps::COccupancyGridMap2D(o); } ) );

		pybind11::enum_<mrpt::maps::COccupancyGridMap2D::TLikelihoodMethod>(cl, "TLikelihoodMethod", pybind11::arithmetic(), "The type for selecting a likelihood computation method ")
			.value("lmMeanInformation", mrpt::maps::COccupancyGridMap2D::lmMeanInformation)
			.value("lmRayTracing", mrpt::maps::COccupancyGridMap2D::lmRayTracing)
			.value("lmConsensus", mrpt::maps::COccupancyGridMap2D::lmConsensus)
			.value("lmCellsDifference", mrpt::maps::COccupancyGridMap2D::lmCellsDifference)
			.value("lmLikelihoodField_Thrun", mrpt::maps::COccupancyGridMap2D::lmLikelihoodField_Thrun)
			.value("lmLikelihoodField_II", mrpt::maps::COccupancyGridMap2D::lmLikelihoodField_II)
			.value("lmConsensusOWA", mrpt::maps::COccupancyGridMap2D::lmConsensusOWA)
			.export_values();


		pybind11::enum_<mrpt::maps::COccupancyGridMap2D::TLaserSimulUncertaintyMethod>(cl, "TLaserSimulUncertaintyMethod", pybind11::arithmetic(), "Methods for TLaserSimulUncertaintyParams in\n laserScanSimulatorWithUncertainty() ")
			.value("sumUnscented", mrpt::maps::COccupancyGridMap2D::sumUnscented)
			.value("sumMonteCarlo", mrpt::maps::COccupancyGridMap2D::sumMonteCarlo)
			.export_values();

		cl.def_readwrite("updateInfoChangeOnly", &mrpt::maps::COccupancyGridMap2D::updateInfoChangeOnly);
		cl.def_readwrite("insertionOptions", &mrpt::maps::COccupancyGridMap2D::insertionOptions);
		cl.def_readwrite("likelihoodOptions", &mrpt::maps::COccupancyGridMap2D::likelihoodOptions);
		cl.def_readwrite("likelihoodOutputs", &mrpt::maps::COccupancyGridMap2D::likelihoodOutputs);
		cl.def_readwrite("CriticalPointsList", &mrpt::maps::COccupancyGridMap2D::CriticalPointsList);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::maps::COccupancyGridMap2D::GetRuntimeClassIdStatic, "C++: mrpt::maps::COccupancyGridMap2D::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::maps::COccupancyGridMap2D::*)() const) &mrpt::maps::COccupancyGridMap2D::GetRuntimeClass, "C++: mrpt::maps::COccupancyGridMap2D::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::maps::COccupancyGridMap2D::*)() const) &mrpt::maps::COccupancyGridMap2D::clone, "C++: mrpt::maps::COccupancyGridMap2D::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::maps::COccupancyGridMap2D::CreateObject, "C++: mrpt::maps::COccupancyGridMap2D::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("updateCell", (void (mrpt::maps::COccupancyGridMap2D::*)(int, int, float)) &mrpt::maps::COccupancyGridMap2D::updateCell, "Performs the Bayesian fusion of a new observation of a cell  \n\n updateInfoChangeOnly, updateCell_fast_occupied, updateCell_fast_free \n\nC++: mrpt::maps::COccupancyGridMap2D::updateCell(int, int, float) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("v"));
		cl.def("fill", [](mrpt::maps::COccupancyGridMap2D &o) -> void { return o.fill(); }, "");
		cl.def("fill", (void (mrpt::maps::COccupancyGridMap2D::*)(float)) &mrpt::maps::COccupancyGridMap2D::fill, "Fills all the cells with a default value. \n\nC++: mrpt::maps::COccupancyGridMap2D::fill(float) --> void", pybind11::arg("default_value"));
		cl.def("setSize", [](mrpt::maps::COccupancyGridMap2D &o, float const & a0, float const & a1, float const & a2, float const & a3, float const & a4) -> void { return o.setSize(a0, a1, a2, a3, a4); }, "", pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("resolution"));
		cl.def("setSize", (void (mrpt::maps::COccupancyGridMap2D::*)(float, float, float, float, float, float)) &mrpt::maps::COccupancyGridMap2D::setSize, "Change the size of gridmap, erasing all its previous contents.\n \n\n The \"x\" coordinates of left most side of grid.\n \n\n The \"x\" coordinates of right most side of grid.\n \n\n The \"y\" coordinates of top most side of grid.\n \n\n The \"y\" coordinates of bottom most side of grid.\n \n\n The new size of cells.\n \n\n The value of cells, tipically 0.5.\n \n\n ResizeGrid\n\nC++: mrpt::maps::COccupancyGridMap2D::setSize(float, float, float, float, float, float) --> void", pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("resolution"), pybind11::arg("default_value"));
		cl.def("resizeGrid", [](mrpt::maps::COccupancyGridMap2D &o, float const & a0, float const & a1, float const & a2, float const & a3) -> void { return o.resizeGrid(a0, a1, a2, a3); }, "", pybind11::arg("new_x_min"), pybind11::arg("new_x_max"), pybind11::arg("new_y_min"), pybind11::arg("new_y_max"));
		cl.def("resizeGrid", [](mrpt::maps::COccupancyGridMap2D &o, float const & a0, float const & a1, float const & a2, float const & a3, float const & a4) -> void { return o.resizeGrid(a0, a1, a2, a3, a4); }, "", pybind11::arg("new_x_min"), pybind11::arg("new_x_max"), pybind11::arg("new_y_min"), pybind11::arg("new_y_max"), pybind11::arg("new_cells_default_value"));
		cl.def("resizeGrid", (void (mrpt::maps::COccupancyGridMap2D::*)(float, float, float, float, float, bool)) &mrpt::maps::COccupancyGridMap2D::resizeGrid, "Change the size of gridmap, maintaining previous contents.\n \n\n The \"x\" coordinates of new left most side of grid.\n \n\n The \"x\" coordinates of new right most side of grid.\n \n\n The \"y\" coordinates of new top most side of grid.\n \n\n The \"y\" coordinates of new bottom most side of grid.\n \n\n The value of the new cells, tipically 0.5.\n \n\n If set to true (default), an additional margin of\n a few meters will be added to the grid, ONLY if the new coordinates are\n larger than current ones.\n \n\n setSize\n\nC++: mrpt::maps::COccupancyGridMap2D::resizeGrid(float, float, float, float, float, bool) --> void", pybind11::arg("new_x_min"), pybind11::arg("new_x_max"), pybind11::arg("new_y_min"), pybind11::arg("new_y_max"), pybind11::arg("new_cells_default_value"), pybind11::arg("additionalMargin"));
		cl.def("getArea", (double (mrpt::maps::COccupancyGridMap2D::*)() const) &mrpt::maps::COccupancyGridMap2D::getArea, "Returns the area of the gridmap, in square meters \n\nC++: mrpt::maps::COccupancyGridMap2D::getArea() const --> double");
		cl.def("getSizeX", (unsigned int (mrpt::maps::COccupancyGridMap2D::*)() const) &mrpt::maps::COccupancyGridMap2D::getSizeX, "Returns the horizontal size of grid map in cells count \n\nC++: mrpt::maps::COccupancyGridMap2D::getSizeX() const --> unsigned int");
		cl.def("getSizeY", (unsigned int (mrpt::maps::COccupancyGridMap2D::*)() const) &mrpt::maps::COccupancyGridMap2D::getSizeY, "Returns the vertical size of grid map in cells count \n\nC++: mrpt::maps::COccupancyGridMap2D::getSizeY() const --> unsigned int");
		cl.def("getXMin", (float (mrpt::maps::COccupancyGridMap2D::*)() const) &mrpt::maps::COccupancyGridMap2D::getXMin, "Returns the \"x\" coordinate of left side of grid map \n\nC++: mrpt::maps::COccupancyGridMap2D::getXMin() const --> float");
		cl.def("getXMax", (float (mrpt::maps::COccupancyGridMap2D::*)() const) &mrpt::maps::COccupancyGridMap2D::getXMax, "Returns the \"x\" coordinate of right side of grid map \n\nC++: mrpt::maps::COccupancyGridMap2D::getXMax() const --> float");
		cl.def("getYMin", (float (mrpt::maps::COccupancyGridMap2D::*)() const) &mrpt::maps::COccupancyGridMap2D::getYMin, "Returns the \"y\" coordinate of top side of grid map \n\nC++: mrpt::maps::COccupancyGridMap2D::getYMin() const --> float");
		cl.def("getYMax", (float (mrpt::maps::COccupancyGridMap2D::*)() const) &mrpt::maps::COccupancyGridMap2D::getYMax, "Returns the \"y\" coordinate of bottom side of grid map \n\nC++: mrpt::maps::COccupancyGridMap2D::getYMax() const --> float");
		cl.def("getResolution", (float (mrpt::maps::COccupancyGridMap2D::*)() const) &mrpt::maps::COccupancyGridMap2D::getResolution, "Returns the resolution of the grid map \n\nC++: mrpt::maps::COccupancyGridMap2D::getResolution() const --> float");
		cl.def("x2idx", (int (mrpt::maps::COccupancyGridMap2D::*)(float) const) &mrpt::maps::COccupancyGridMap2D::x2idx, "Transform a coordinate value into a cell index \n\nC++: mrpt::maps::COccupancyGridMap2D::x2idx(float) const --> int", pybind11::arg("x"));
		cl.def("y2idx", (int (mrpt::maps::COccupancyGridMap2D::*)(float) const) &mrpt::maps::COccupancyGridMap2D::y2idx, "C++: mrpt::maps::COccupancyGridMap2D::y2idx(float) const --> int", pybind11::arg("y"));
		cl.def("x2idx", (int (mrpt::maps::COccupancyGridMap2D::*)(double) const) &mrpt::maps::COccupancyGridMap2D::x2idx, "C++: mrpt::maps::COccupancyGridMap2D::x2idx(double) const --> int", pybind11::arg("x"));
		cl.def("y2idx", (int (mrpt::maps::COccupancyGridMap2D::*)(double) const) &mrpt::maps::COccupancyGridMap2D::y2idx, "C++: mrpt::maps::COccupancyGridMap2D::y2idx(double) const --> int", pybind11::arg("y"));
		cl.def("idx2x", (float (mrpt::maps::COccupancyGridMap2D::*)(size_t) const) &mrpt::maps::COccupancyGridMap2D::idx2x, "Transform a cell index into a coordinate value \n\nC++: mrpt::maps::COccupancyGridMap2D::idx2x(size_t) const --> float", pybind11::arg("cx"));
		cl.def("idx2y", (float (mrpt::maps::COccupancyGridMap2D::*)(size_t) const) &mrpt::maps::COccupancyGridMap2D::idx2y, "C++: mrpt::maps::COccupancyGridMap2D::idx2y(size_t) const --> float", pybind11::arg("cy"));
		cl.def("x2idx", (int (mrpt::maps::COccupancyGridMap2D::*)(float, float) const) &mrpt::maps::COccupancyGridMap2D::x2idx, "Transform a coordinate value into a cell index, using a diferent \"x_min\"\n value \n\nC++: mrpt::maps::COccupancyGridMap2D::x2idx(float, float) const --> int", pybind11::arg("x"), pybind11::arg("xmin"));
		cl.def("y2idx", (int (mrpt::maps::COccupancyGridMap2D::*)(float, float) const) &mrpt::maps::COccupancyGridMap2D::y2idx, "C++: mrpt::maps::COccupancyGridMap2D::y2idx(float, float) const --> int", pybind11::arg("y"), pybind11::arg("ymin"));
		cl.def("boundingBox", (struct mrpt::math::TBoundingBox_<float> (mrpt::maps::COccupancyGridMap2D::*)() const) &mrpt::maps::COccupancyGridMap2D::boundingBox, "C++: mrpt::maps::COccupancyGridMap2D::boundingBox() const --> struct mrpt::math::TBoundingBox_<float>");
		cl.def_static("l2p", (float (*)(const signed char)) &mrpt::maps::COccupancyGridMap2D::l2p, "Scales an integer representation of the log-odd into a real valued\n probability in [0,1], using p=exp(l)/(1+exp(l))  \n\nC++: mrpt::maps::COccupancyGridMap2D::l2p(const signed char) --> float", pybind11::arg("l"));
		cl.def_static("l2p_255", (uint8_t (*)(const signed char)) &mrpt::maps::COccupancyGridMap2D::l2p_255, "Scales an integer representation of the log-odd into a linear scale\n [0,255], using p=exp(l)/(1+exp(l)) \n\nC++: mrpt::maps::COccupancyGridMap2D::l2p_255(const signed char) --> uint8_t", pybind11::arg("l"));
		cl.def_static("p2l", (signed char (*)(const float)) &mrpt::maps::COccupancyGridMap2D::p2l, "Scales a real valued probability in [0,1] to an integer representation\n of: log(p)-log(1-p)  in the valid range of cellType \n\nC++: mrpt::maps::COccupancyGridMap2D::p2l(const float) --> signed char", pybind11::arg("p"));
		cl.def("setCell", (void (mrpt::maps::COccupancyGridMap2D::*)(int, int, float)) &mrpt::maps::COccupancyGridMap2D::setCell, "Change the contents [0,1] of a cell, given its index \n\nC++: mrpt::maps::COccupancyGridMap2D::setCell(int, int, float) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("value"));
		cl.def("getCell", (float (mrpt::maps::COccupancyGridMap2D::*)(int, int) const) &mrpt::maps::COccupancyGridMap2D::getCell, "Read the real valued [0,1] contents of a cell, given its index \n\nC++: mrpt::maps::COccupancyGridMap2D::getCell(int, int) const --> float", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("getRow", (signed char * (mrpt::maps::COccupancyGridMap2D::*)(int)) &mrpt::maps::COccupancyGridMap2D::getRow, "Access to a \"row\": mainly used for drawing grid as a bitmap efficiently,\n do not use it normally \n\nC++: mrpt::maps::COccupancyGridMap2D::getRow(int) --> signed char *", pybind11::return_value_policy::automatic, pybind11::arg("cy"));
		cl.def("setPos", (void (mrpt::maps::COccupancyGridMap2D::*)(float, float, float)) &mrpt::maps::COccupancyGridMap2D::setPos, "Change the contents [0,1] of a cell, given its coordinates \n\nC++: mrpt::maps::COccupancyGridMap2D::setPos(float, float, float) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("value"));
		cl.def("getPos", (float (mrpt::maps::COccupancyGridMap2D::*)(float, float) const) &mrpt::maps::COccupancyGridMap2D::getPos, "Read the real valued [0,1] contents of a cell, given its coordinates \n\nC++: mrpt::maps::COccupancyGridMap2D::getPos(float, float) const --> float", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("isStaticPos", [](mrpt::maps::COccupancyGridMap2D const &o, float const & a0, float const & a1) -> bool { return o.isStaticPos(a0, a1); }, "", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("isStaticPos", (bool (mrpt::maps::COccupancyGridMap2D::*)(float, float, float) const) &mrpt::maps::COccupancyGridMap2D::isStaticPos, "Returns \"true\" if cell is \"static\", i.e.if its occupancy is below a\n given threshold \n\nC++: mrpt::maps::COccupancyGridMap2D::isStaticPos(float, float, float) const --> bool", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("threshold"));
		cl.def("isStaticCell", [](mrpt::maps::COccupancyGridMap2D const &o, int const & a0, int const & a1) -> bool { return o.isStaticCell(a0, a1); }, "", pybind11::arg("cx"), pybind11::arg("cy"));
		cl.def("isStaticCell", (bool (mrpt::maps::COccupancyGridMap2D::*)(int, int, float) const) &mrpt::maps::COccupancyGridMap2D::isStaticCell, "C++: mrpt::maps::COccupancyGridMap2D::isStaticCell(int, int, float) const --> bool", pybind11::arg("cx"), pybind11::arg("cy"), pybind11::arg("threshold"));
		cl.def("setBasisCell", (void (mrpt::maps::COccupancyGridMap2D::*)(int, int, uint8_t)) &mrpt::maps::COccupancyGridMap2D::setBasisCell, "Change a cell in the \"basis\" maps.Used for Voronoi calculation \n\nC++: mrpt::maps::COccupancyGridMap2D::setBasisCell(int, int, uint8_t) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("value"));
		cl.def("getBasisCell", (unsigned char (mrpt::maps::COccupancyGridMap2D::*)(int, int) const) &mrpt::maps::COccupancyGridMap2D::getBasisCell, "Reads a cell in the \"basis\" maps.Used for Voronoi calculation \n\nC++: mrpt::maps::COccupancyGridMap2D::getBasisCell(int, int) const --> unsigned char", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("copyMapContentFrom", (void (mrpt::maps::COccupancyGridMap2D::*)(const class mrpt::maps::COccupancyGridMap2D &)) &mrpt::maps::COccupancyGridMap2D::copyMapContentFrom, "copy the gridmap contents, but not all the options, from another map\n instance \n\nC++: mrpt::maps::COccupancyGridMap2D::copyMapContentFrom(const class mrpt::maps::COccupancyGridMap2D &) --> void", pybind11::arg("otherMap"));
		cl.def("subSample", (void (mrpt::maps::COccupancyGridMap2D::*)(int)) &mrpt::maps::COccupancyGridMap2D::subSample, "Performs a downsampling of the gridmap, by a given factor:\n resolution/=ratio \n\nC++: mrpt::maps::COccupancyGridMap2D::subSample(int) --> void", pybind11::arg("downRatio"));
		cl.def("computeEntropy", (void (mrpt::maps::COccupancyGridMap2D::*)(struct mrpt::maps::COccupancyGridMap2D::TEntropyInfo &) const) &mrpt::maps::COccupancyGridMap2D::computeEntropy, "Computes the entropy and related values of this grid map.\n  The entropy is computed as the summed entropy of each cell, taking them\n as discrete random variables following a Bernoulli distribution:\n \n\n The output information is returned here \n\nC++: mrpt::maps::COccupancyGridMap2D::computeEntropy(struct mrpt::maps::COccupancyGridMap2D::TEntropyInfo &) const --> void", pybind11::arg("info"));
		cl.def("buildVoronoiDiagram", [](mrpt::maps::COccupancyGridMap2D &o, float const & a0, float const & a1) -> void { return o.buildVoronoiDiagram(a0, a1); }, "", pybind11::arg("threshold"), pybind11::arg("robot_size"));
		cl.def("buildVoronoiDiagram", [](mrpt::maps::COccupancyGridMap2D &o, float const & a0, float const & a1, int const & a2) -> void { return o.buildVoronoiDiagram(a0, a1, a2); }, "", pybind11::arg("threshold"), pybind11::arg("robot_size"), pybind11::arg("x1"));
		cl.def("buildVoronoiDiagram", [](mrpt::maps::COccupancyGridMap2D &o, float const & a0, float const & a1, int const & a2, int const & a3) -> void { return o.buildVoronoiDiagram(a0, a1, a2, a3); }, "", pybind11::arg("threshold"), pybind11::arg("robot_size"), pybind11::arg("x1"), pybind11::arg("x2"));
		cl.def("buildVoronoiDiagram", [](mrpt::maps::COccupancyGridMap2D &o, float const & a0, float const & a1, int const & a2, int const & a3, int const & a4) -> void { return o.buildVoronoiDiagram(a0, a1, a2, a3, a4); }, "", pybind11::arg("threshold"), pybind11::arg("robot_size"), pybind11::arg("x1"), pybind11::arg("x2"), pybind11::arg("y1"));
		cl.def("buildVoronoiDiagram", (void (mrpt::maps::COccupancyGridMap2D::*)(float, float, int, int, int, int)) &mrpt::maps::COccupancyGridMap2D::buildVoronoiDiagram, "Build the Voronoi diagram of the grid map.\n \n\n The threshold for binarizing the map.\n \n\n Size in \"units\" (meters) of robot, approx.\n \n\n Left coordinate of area to be computed. Default, entire map.\n \n\n Right coordinate of area to be computed. Default, entire map.\n \n\n Top coordinate of area to be computed. Default, entire map.\n \n\n Bottom coordinate of area to be computed. Default, entire map.\n \n\n findCriticalPoints\n\nC++: mrpt::maps::COccupancyGridMap2D::buildVoronoiDiagram(float, float, int, int, int, int) --> void", pybind11::arg("threshold"), pybind11::arg("robot_size"), pybind11::arg("x1"), pybind11::arg("x2"), pybind11::arg("y1"), pybind11::arg("y2"));
		cl.def("getVoroniClearance", (uint16_t (mrpt::maps::COccupancyGridMap2D::*)(int, int) const) &mrpt::maps::COccupancyGridMap2D::getVoroniClearance, "Reads a the clearance of a cell (in centimeters), after building the\n Voronoi diagram with  \n\nC++: mrpt::maps::COccupancyGridMap2D::getVoroniClearance(int, int) const --> uint16_t", pybind11::arg("cx"), pybind11::arg("cy"));
		cl.def("getBasisMap", (const class mrpt::containers::CDynamicGrid<unsigned char> & (mrpt::maps::COccupancyGridMap2D::*)() const) &mrpt::maps::COccupancyGridMap2D::getBasisMap, "Return the auxiliary \"basis\" map built while building the Voronoi\n diagram \n\n buildVoronoiDiagram \n\nC++: mrpt::maps::COccupancyGridMap2D::getBasisMap() const --> const class mrpt::containers::CDynamicGrid<unsigned char> &", pybind11::return_value_policy::automatic);
		cl.def("getVoronoiDiagram", (const class mrpt::containers::CDynamicGrid<unsigned short> & (mrpt::maps::COccupancyGridMap2D::*)() const) &mrpt::maps::COccupancyGridMap2D::getVoronoiDiagram, "Return the Voronoi diagram; each cell contains the distance to its\n closer obstacle, or 0 if not part of the Voronoi diagram \n\n\n buildVoronoiDiagram \n\nC++: mrpt::maps::COccupancyGridMap2D::getVoronoiDiagram() const --> const class mrpt::containers::CDynamicGrid<unsigned short> &", pybind11::return_value_policy::automatic);
		cl.def("findCriticalPoints", (void (mrpt::maps::COccupancyGridMap2D::*)(float)) &mrpt::maps::COccupancyGridMap2D::findCriticalPoints, "Builds a list with the critical points from Voronoi diagram, which must\n    must be built before calling this method.\n \n\n The minimum distance between two critical points.\n \n\n buildVoronoiDiagram\n\nC++: mrpt::maps::COccupancyGridMap2D::findCriticalPoints(float) --> void", pybind11::arg("filter_distance"));
		cl.def("computeClearance", [](mrpt::maps::COccupancyGridMap2D const &o, int const & a0, int const & a1, int * a2, int * a3, int * a4) -> int { return o.computeClearance(a0, a1, a2, a3, a4); }, "", pybind11::arg("cx"), pybind11::arg("cy"), pybind11::arg("basis_x"), pybind11::arg("basis_y"), pybind11::arg("nBasis"));
		cl.def("computeClearance", (int (mrpt::maps::COccupancyGridMap2D::*)(int, int, int *, int *, int *, bool) const) &mrpt::maps::COccupancyGridMap2D::computeClearance, "Compute the clearance of a given cell, and returns its two first\n   basis (closest obstacle) points.Used to build Voronoi and critical\n points.\n \n\n The clearance of the cell, in 1/100 of \"cell\".\n \n\n The cell index\n \n\n The cell index\n \n\n Target buffer for coordinates of basis, having a size of\n two \"ints\".\n \n\n Target buffer for coordinates of basis, having a size of\n two \"ints\".\n \n\n The number of found basis: Can be 0,1 or 2.\n \n\n If \"true\" the basis are not returned, but the\n closest free cells.Default at false.\n \n\n Build_VoronoiDiagram\n\nC++: mrpt::maps::COccupancyGridMap2D::computeClearance(int, int, int *, int *, int *, bool) const --> int", pybind11::arg("cx"), pybind11::arg("cy"), pybind11::arg("basis_x"), pybind11::arg("basis_y"), pybind11::arg("nBasis"), pybind11::arg("GetContourPoint"));
		cl.def("computeClearance", (float (mrpt::maps::COccupancyGridMap2D::*)(float, float, float) const) &mrpt::maps::COccupancyGridMap2D::computeClearance, "An alternative method for computing the clearance of a given location\n (in meters).\n  \n\n The clearance (distance to closest OCCUPIED cell), in meters.\n\nC++: mrpt::maps::COccupancyGridMap2D::computeClearance(float, float, float) const --> float", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("maxSearchDistance"));
		cl.def("computePathCost", (float (mrpt::maps::COccupancyGridMap2D::*)(float, float, float, float) const) &mrpt::maps::COccupancyGridMap2D::computePathCost, "Compute the 'cost' of traversing a segment of the map according to the\n occupancy of traversed cells.\n  \n\n This returns '1-mean(traversed cells occupancy)', i.e. 0.5 for\n unknown cells, 1 for a free path.\n\nC++: mrpt::maps::COccupancyGridMap2D::computePathCost(float, float, float, float) const --> float", pybind11::arg("x1"), pybind11::arg("y1"), pybind11::arg("x2"), pybind11::arg("y2"));
		cl.def("laserScanSimulator", [](mrpt::maps::COccupancyGridMap2D const &o, class mrpt::obs::CObservation2DRangeScan & a0, const class mrpt::poses::CPose2D & a1) -> void { return o.laserScanSimulator(a0, a1); }, "", pybind11::arg("inout_Scan"), pybind11::arg("robotPose"));
		cl.def("laserScanSimulator", [](mrpt::maps::COccupancyGridMap2D const &o, class mrpt::obs::CObservation2DRangeScan & a0, const class mrpt::poses::CPose2D & a1, float const & a2) -> void { return o.laserScanSimulator(a0, a1, a2); }, "", pybind11::arg("inout_Scan"), pybind11::arg("robotPose"), pybind11::arg("threshold"));
		cl.def("laserScanSimulator", [](mrpt::maps::COccupancyGridMap2D const &o, class mrpt::obs::CObservation2DRangeScan & a0, const class mrpt::poses::CPose2D & a1, float const & a2, size_t const & a3) -> void { return o.laserScanSimulator(a0, a1, a2, a3); }, "", pybind11::arg("inout_Scan"), pybind11::arg("robotPose"), pybind11::arg("threshold"), pybind11::arg("N"));
		cl.def("laserScanSimulator", [](mrpt::maps::COccupancyGridMap2D const &o, class mrpt::obs::CObservation2DRangeScan & a0, const class mrpt::poses::CPose2D & a1, float const & a2, size_t const & a3, float const & a4) -> void { return o.laserScanSimulator(a0, a1, a2, a3, a4); }, "", pybind11::arg("inout_Scan"), pybind11::arg("robotPose"), pybind11::arg("threshold"), pybind11::arg("N"), pybind11::arg("noiseStd"));
		cl.def("laserScanSimulator", [](mrpt::maps::COccupancyGridMap2D const &o, class mrpt::obs::CObservation2DRangeScan & a0, const class mrpt::poses::CPose2D & a1, float const & a2, size_t const & a3, float const & a4, unsigned int const & a5) -> void { return o.laserScanSimulator(a0, a1, a2, a3, a4, a5); }, "", pybind11::arg("inout_Scan"), pybind11::arg("robotPose"), pybind11::arg("threshold"), pybind11::arg("N"), pybind11::arg("noiseStd"), pybind11::arg("decimation"));
		cl.def("laserScanSimulator", (void (mrpt::maps::COccupancyGridMap2D::*)(class mrpt::obs::CObservation2DRangeScan &, const class mrpt::poses::CPose2D &, float, size_t, float, unsigned int, float) const) &mrpt::maps::COccupancyGridMap2D::laserScanSimulator, "Simulates a laser range scan into the current grid map.\n   The simulated scan is stored in a CObservation2DRangeScan object, which\nis also used\n    to pass some parameters: all previously stored characteristics (as\naperture,...) are\n	  taken into account for simulation. Only a few more parameters are\nneeded. Additive gaussian noise can be optionally added to the simulated\nscan.\n \n\n [IN/OUT] This must be filled with desired parameters\nbefore calling, and will contain the scan samples on return.\n \n\n [IN] The robot pose in this map coordinates. Recall that\nsensor pose relative to this robot pose must be specified in the\nobservation object.\n \n\n [IN] The minimum occupancy threshold to consider a cell\nto be occupied (Default: 0.5f)\n \n\n [IN] The count of range scan \"rays\", by default to 361.\n \n\n [IN] The standard deviation of measurement noise. If not\ndesired, set to 0.\n \n\n [IN] The rays that will be simulated are at indexes: 0,\nD, 2D, 3D, ... Default is D=1\n \n\n [IN] The sigma of an optional Gaussian noise added\nto the angles at which ranges are measured (in radians).\n\n \n laserScanSimulatorWithUncertainty(), sonarSimulator(),\nCOccupancyGridMap2D::RAYTRACE_STEP_SIZE_IN_CELL_UNITS\n\nC++: mrpt::maps::COccupancyGridMap2D::laserScanSimulator(class mrpt::obs::CObservation2DRangeScan &, const class mrpt::poses::CPose2D &, float, size_t, float, unsigned int, float) const --> void", pybind11::arg("inout_Scan"), pybind11::arg("robotPose"), pybind11::arg("threshold"), pybind11::arg("N"), pybind11::arg("noiseStd"), pybind11::arg("decimation"), pybind11::arg("angleNoiseStd"));
		cl.def("sonarSimulator", [](mrpt::maps::COccupancyGridMap2D const &o, class mrpt::obs::CObservationRange & a0, const class mrpt::poses::CPose2D & a1) -> void { return o.sonarSimulator(a0, a1); }, "", pybind11::arg("inout_observation"), pybind11::arg("robotPose"));
		cl.def("sonarSimulator", [](mrpt::maps::COccupancyGridMap2D const &o, class mrpt::obs::CObservationRange & a0, const class mrpt::poses::CPose2D & a1, float const & a2) -> void { return o.sonarSimulator(a0, a1, a2); }, "", pybind11::arg("inout_observation"), pybind11::arg("robotPose"), pybind11::arg("threshold"));
		cl.def("sonarSimulator", [](mrpt::maps::COccupancyGridMap2D const &o, class mrpt::obs::CObservationRange & a0, const class mrpt::poses::CPose2D & a1, float const & a2, float const & a3) -> void { return o.sonarSimulator(a0, a1, a2, a3); }, "", pybind11::arg("inout_observation"), pybind11::arg("robotPose"), pybind11::arg("threshold"), pybind11::arg("rangeNoiseStd"));
		cl.def("sonarSimulator", (void (mrpt::maps::COccupancyGridMap2D::*)(class mrpt::obs::CObservationRange &, const class mrpt::poses::CPose2D &, float, float, float) const) &mrpt::maps::COccupancyGridMap2D::sonarSimulator, "Simulates the observations of a sonar rig into the current grid map.\n   The simulated ranges are stored in a CObservationRange object, which is\n also used\n    to pass in some needed parameters, as the poses of the sonar sensors\n onto the mobile robot.\n \n\n [IN/OUT] This must be filled with desired\n parameters before calling, and will contain the simulated ranges on\n return.\n \n\n [IN] The robot pose in this map coordinates. Recall that\n sensor pose relative to this robot pose must be specified in the\n observation object.\n \n\n [IN] The minimum occupancy threshold to consider a cell\n to be occupied (Default: 0.5f)\n \n\n [IN] The standard deviation of measurement noise. If\n not desired, set to 0.\n \n\n [IN] The sigma of an optional Gaussian noise added\n to the angles at which ranges are measured (in radians).\n\n \n laserScanSimulator(),\n COccupancyGridMap2D::RAYTRACE_STEP_SIZE_IN_CELL_UNITS\n\nC++: mrpt::maps::COccupancyGridMap2D::sonarSimulator(class mrpt::obs::CObservationRange &, const class mrpt::poses::CPose2D &, float, float, float) const --> void", pybind11::arg("inout_observation"), pybind11::arg("robotPose"), pybind11::arg("threshold"), pybind11::arg("rangeNoiseStd"), pybind11::arg("angleNoiseStd"));
		cl.def("simulateScanRay", [](mrpt::maps::COccupancyGridMap2D const &o, const double & a0, const double & a1, const double & a2, float & a3, bool & a4, const double & a5) -> void { return o.simulateScanRay(a0, a1, a2, a3, a4, a5); }, "", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("angle_direction"), pybind11::arg("out_range"), pybind11::arg("out_valid"), pybind11::arg("max_range_meters"));
		cl.def("simulateScanRay", [](mrpt::maps::COccupancyGridMap2D const &o, const double & a0, const double & a1, const double & a2, float & a3, bool & a4, const double & a5, const float & a6) -> void { return o.simulateScanRay(a0, a1, a2, a3, a4, a5, a6); }, "", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("angle_direction"), pybind11::arg("out_range"), pybind11::arg("out_valid"), pybind11::arg("max_range_meters"), pybind11::arg("threshold_free"));
		cl.def("simulateScanRay", [](mrpt::maps::COccupancyGridMap2D const &o, const double & a0, const double & a1, const double & a2, float & a3, bool & a4, const double & a5, const float & a6, const double & a7) -> void { return o.simulateScanRay(a0, a1, a2, a3, a4, a5, a6, a7); }, "", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("angle_direction"), pybind11::arg("out_range"), pybind11::arg("out_valid"), pybind11::arg("max_range_meters"), pybind11::arg("threshold_free"), pybind11::arg("noiseStd"));
		cl.def("simulateScanRay", (void (mrpt::maps::COccupancyGridMap2D::*)(const double, const double, const double, float &, bool &, const double, const float, const double, const double) const) &mrpt::maps::COccupancyGridMap2D::simulateScanRay, "Simulate just one \"ray\" in the grid map. This method is used internally\n to sonarSimulator and laserScanSimulator. \n\n\n COccupancyGridMap2D::RAYTRACE_STEP_SIZE_IN_CELL_UNITS \n\nC++: mrpt::maps::COccupancyGridMap2D::simulateScanRay(const double, const double, const double, float &, bool &, const double, const float, const double, const double) const --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("angle_direction"), pybind11::arg("out_range"), pybind11::arg("out_valid"), pybind11::arg("max_range_meters"), pybind11::arg("threshold_free"), pybind11::arg("noiseStd"), pybind11::arg("angleNoiseStd"));
		cl.def("laserScanSimulatorWithUncertainty", (void (mrpt::maps::COccupancyGridMap2D::*)(const struct mrpt::maps::COccupancyGridMap2D::TLaserSimulUncertaintyParams &, struct mrpt::maps::COccupancyGridMap2D::TLaserSimulUncertaintyResult &) const) &mrpt::maps::COccupancyGridMap2D::laserScanSimulatorWithUncertainty, "Like laserScanSimulatorWithUncertainty() (see it for a discussion of\n most parameters) but taking into account\n  the robot pose uncertainty and generating a vector of predicted\n variances for each ray.\n  Range uncertainty includes both, sensor noise and large non-linear\n effects caused by borders and discontinuities in the environment\n  as seen from different robot poses.\n\n \n [IN] Input settings. See TLaserSimulUncertaintyParams\n \n\n [OUT] Output range + uncertainty.\n\n \n laserScanSimulator(),\n COccupancyGridMap2D::RAYTRACE_STEP_SIZE_IN_CELL_UNITS\n\nC++: mrpt::maps::COccupancyGridMap2D::laserScanSimulatorWithUncertainty(const struct mrpt::maps::COccupancyGridMap2D::TLaserSimulUncertaintyParams &, struct mrpt::maps::COccupancyGridMap2D::TLaserSimulUncertaintyResult &) const --> void", pybind11::arg("in_params"), pybind11::arg("out_results"));
		cl.def("computeLikelihoodField_Thrun", [](mrpt::maps::COccupancyGridMap2D const &o, const class mrpt::maps::CPointsMap * a0) -> double { return o.computeLikelihoodField_Thrun(a0); }, "", pybind11::arg("pm"));
		cl.def("computeLikelihoodField_Thrun", (double (mrpt::maps::COccupancyGridMap2D::*)(const class mrpt::maps::CPointsMap *, const class mrpt::poses::CPose2D *) const) &mrpt::maps::COccupancyGridMap2D::computeLikelihoodField_Thrun, "Computes the likelihood [0,1] of a set of points, given the current grid\n map as reference.\n \n\n The points map\n \n\n The relative pose of the points map in this map's\n coordinates, or nullptr for (0,0,0).\n  See \"likelihoodOptions\" for configuration parameters.\n\nC++: mrpt::maps::COccupancyGridMap2D::computeLikelihoodField_Thrun(const class mrpt::maps::CPointsMap *, const class mrpt::poses::CPose2D *) const --> double", pybind11::arg("pm"), pybind11::arg("relativePose"));
		cl.def("computeLikelihoodField_II", [](mrpt::maps::COccupancyGridMap2D const &o, const class mrpt::maps::CPointsMap * a0) -> double { return o.computeLikelihoodField_II(a0); }, "", pybind11::arg("pm"));
		cl.def("computeLikelihoodField_II", (double (mrpt::maps::COccupancyGridMap2D::*)(const class mrpt::maps::CPointsMap *, const class mrpt::poses::CPose2D *) const) &mrpt::maps::COccupancyGridMap2D::computeLikelihoodField_II, "Computes the likelihood [0,1] of a set of points, given the current grid\n map as reference.\n \n\n The points map\n \n\n The relative pose of the points map in this map's\n coordinates, or nullptr for (0,0,0).\n  See \"likelihoodOptions\" for configuration parameters.\n\nC++: mrpt::maps::COccupancyGridMap2D::computeLikelihoodField_II(const class mrpt::maps::CPointsMap *, const class mrpt::poses::CPose2D *) const --> double", pybind11::arg("pm"), pybind11::arg("relativePose"));
		cl.def("saveAsBitmapFile", (bool (mrpt::maps::COccupancyGridMap2D::*)(const std::string &) const) &mrpt::maps::COccupancyGridMap2D::saveAsBitmapFile, "Saves the gridmap as a graphical file (BMP,PNG,...).\n The format will be derived from the file extension (see\n CImage::saveToFile )\n \n\n False on any error.\n\nC++: mrpt::maps::COccupancyGridMap2D::saveAsBitmapFile(const std::string &) const --> bool", pybind11::arg("file"));
		cl.def_static("saveAsBitmapTwoMapsWithCorrespondences", (bool (*)(const std::string &, const class mrpt::maps::COccupancyGridMap2D *, const class mrpt::maps::COccupancyGridMap2D *, const class mrpt::tfest::TMatchingPairListTempl<float> &)) &mrpt::maps::COccupancyGridMap2D::saveAsBitmapTwoMapsWithCorrespondences, "Saves a composite image with two gridmaps and lines representing a set\n of correspondences between them.\n \n\n False on any error.\n\nC++: mrpt::maps::COccupancyGridMap2D::saveAsBitmapTwoMapsWithCorrespondences(const std::string &, const class mrpt::maps::COccupancyGridMap2D *, const class mrpt::maps::COccupancyGridMap2D *, const class mrpt::tfest::TMatchingPairListTempl<float> &) --> bool", pybind11::arg("fileName"), pybind11::arg("m1"), pybind11::arg("m2"), pybind11::arg("corrs"));
		cl.def("getAsImage", [](mrpt::maps::COccupancyGridMap2D const &o, class mrpt::img::CImage & a0) -> void { return o.getAsImage(a0); }, "", pybind11::arg("img"));
		cl.def("getAsImage", [](mrpt::maps::COccupancyGridMap2D const &o, class mrpt::img::CImage & a0, bool const & a1) -> void { return o.getAsImage(a0, a1); }, "", pybind11::arg("img"), pybind11::arg("verticalFlip"));
		cl.def("getAsImage", [](mrpt::maps::COccupancyGridMap2D const &o, class mrpt::img::CImage & a0, bool const & a1, bool const & a2) -> void { return o.getAsImage(a0, a1, a2); }, "", pybind11::arg("img"), pybind11::arg("verticalFlip"), pybind11::arg("forceRGB"));
		cl.def("getAsImage", (void (mrpt::maps::COccupancyGridMap2D::*)(class mrpt::img::CImage &, bool, bool, bool) const) &mrpt::maps::COccupancyGridMap2D::getAsImage, "Returns the grid as a 8-bit graylevel image, where each pixel is a cell\n (output image is RGB only if forceRGB is true)\n  If \"tricolor\" is true, only three gray levels will appear in the image:\n gray for unobserved cells, and black/white for occupied/empty cells\n respectively.\n \n\n getAsImageFiltered\n\nC++: mrpt::maps::COccupancyGridMap2D::getAsImage(class mrpt::img::CImage &, bool, bool, bool) const --> void", pybind11::arg("img"), pybind11::arg("verticalFlip"), pybind11::arg("forceRGB"), pybind11::arg("tricolor"));
		cl.def("getAsImageFiltered", [](mrpt::maps::COccupancyGridMap2D const &o, class mrpt::img::CImage & a0) -> void { return o.getAsImageFiltered(a0); }, "", pybind11::arg("img"));
		cl.def("getAsImageFiltered", [](mrpt::maps::COccupancyGridMap2D const &o, class mrpt::img::CImage & a0, bool const & a1) -> void { return o.getAsImageFiltered(a0, a1); }, "", pybind11::arg("img"), pybind11::arg("verticalFlip"));
		cl.def("getAsImageFiltered", (void (mrpt::maps::COccupancyGridMap2D::*)(class mrpt::img::CImage &, bool, bool) const) &mrpt::maps::COccupancyGridMap2D::getAsImageFiltered, "Returns the grid as a 8-bit graylevel image, where each pixel is a cell\n (output image is RGB only if forceRGB is true) - This method filters the\n image for easy feature detection\n  If \"tricolor\" is true, only three gray levels will appear in the image:\n gray for unobserved cells, and black/white for occupied/empty cells\n respectively.\n \n\n getAsImage\n\nC++: mrpt::maps::COccupancyGridMap2D::getAsImageFiltered(class mrpt::img::CImage &, bool, bool) const --> void", pybind11::arg("img"), pybind11::arg("verticalFlip"), pybind11::arg("forceRGB"));
		cl.def("getVisualizationInto", (void (mrpt::maps::COccupancyGridMap2D::*)(class mrpt::opengl::CSetOfObjects &) const) &mrpt::maps::COccupancyGridMap2D::getVisualizationInto, "Returns a 3D plane with its texture being the occupancy grid and\n transparency proportional to \"uncertainty\" (i.e. a value of 0.5 is fully\n transparent)\n\nC++: mrpt::maps::COccupancyGridMap2D::getVisualizationInto(class mrpt::opengl::CSetOfObjects &) const --> void", pybind11::arg("outObj"));
		cl.def("getAsPointCloud", [](mrpt::maps::COccupancyGridMap2D const &o, class mrpt::maps::CSimplePointsMap & a0) -> void { return o.getAsPointCloud(a0); }, "", pybind11::arg("pm"));
		cl.def("getAsPointCloud", (void (mrpt::maps::COccupancyGridMap2D::*)(class mrpt::maps::CSimplePointsMap &, const float) const) &mrpt::maps::COccupancyGridMap2D::getAsPointCloud, "Get a point cloud with all (border) occupied cells as points \n\nC++: mrpt::maps::COccupancyGridMap2D::getAsPointCloud(class mrpt::maps::CSimplePointsMap &, const float) const --> void", pybind11::arg("pm"), pybind11::arg("occup_threshold"));
		cl.def("isEmpty", (bool (mrpt::maps::COccupancyGridMap2D::*)() const) &mrpt::maps::COccupancyGridMap2D::isEmpty, "Returns true upon map construction or after calling clear(), the return\n  changes to false upon successful insertObservation() or any other\n method to load data in the map.\n\nC++: mrpt::maps::COccupancyGridMap2D::isEmpty() const --> bool");
		cl.def("loadFromBitmapFile", [](mrpt::maps::COccupancyGridMap2D &o, const std::string & a0, float const & a1) -> bool { return o.loadFromBitmapFile(a0, a1); }, "", pybind11::arg("file"), pybind11::arg("resolution"));
		cl.def("loadFromBitmapFile", (bool (mrpt::maps::COccupancyGridMap2D::*)(const std::string &, float, const struct mrpt::math::TPoint2D_<double> &)) &mrpt::maps::COccupancyGridMap2D::loadFromBitmapFile, "Load the gridmap from a image in a file (the format can be any supported\n by CImage::loadFromFile).\n \n\n The file to be loaded.\n \n\n The size of a pixel (cell), in meters. Recall cells are\n always squared, so just a dimension is needed.\n \n\n The `x` (0=first, increases left to right on the\n image) and `y` (0=first, increases BOTTOM upwards on the image)\n coordinates for the pixel which will be taken at the origin of map\n coordinates (0,0). (Default=center of the image) \n\n False on any\n error. \n\n loadFromBitmap\n\nC++: mrpt::maps::COccupancyGridMap2D::loadFromBitmapFile(const std::string &, float, const struct mrpt::math::TPoint2D_<double> &) --> bool", pybind11::arg("file"), pybind11::arg("resolution"), pybind11::arg("origin"));
		cl.def("loadFromBitmap", [](mrpt::maps::COccupancyGridMap2D &o, const class mrpt::img::CImage & a0, float const & a1) -> bool { return o.loadFromBitmap(a0, a1); }, "", pybind11::arg("img"), pybind11::arg("resolution"));
		cl.def("loadFromBitmap", (bool (mrpt::maps::COccupancyGridMap2D::*)(const class mrpt::img::CImage &, float, const struct mrpt::math::TPoint2D_<double> &)) &mrpt::maps::COccupancyGridMap2D::loadFromBitmap, "Load the gridmap from a image in a file (the format can be any supported\n by CImage::loadFromFile).\n  See loadFromBitmapFile() for the meaning of parameters \n\nC++: mrpt::maps::COccupancyGridMap2D::loadFromBitmap(const class mrpt::img::CImage &, float, const struct mrpt::math::TPoint2D_<double> &) --> bool", pybind11::arg("img"), pybind11::arg("resolution"), pybind11::arg("origin"));
		cl.def("loadFromROSMapServerYAML", (bool (mrpt::maps::COccupancyGridMap2D::*)(const std::string &)) &mrpt::maps::COccupancyGridMap2D::loadFromROSMapServerYAML, "Loads this gridmap from a .yaml file and an accompanying image file\n  given in the\n  [map_server YAML](http://wiki.ros.org/map_server#YAML_format) file\n  format.\n\n \n Absolute or relative path to the `.yaml` file.\n\n \n false on error, true on success.\n \n\n FromROSMapServerYAML()\n\nC++: mrpt::maps::COccupancyGridMap2D::loadFromROSMapServerYAML(const std::string &) --> bool", pybind11::arg("yamlFilePath"));
		cl.def_static("FromROSMapServerYAML", (class mrpt::maps::COccupancyGridMap2D (*)(const std::string &)) &mrpt::maps::COccupancyGridMap2D::FromROSMapServerYAML, "Creates a gridmap from a .yaml file and an accompanying image file\n  given in the\n  [map_server YAML](http://wiki.ros.org/map_server#YAML_format) file\n  format.\n\n \n Absolute or relative path to the `.yaml` file.\n\n \n loadFromROSMapServerYAML()\n \n\n std::exception On error loading or parsing the files.\n\nC++: mrpt::maps::COccupancyGridMap2D::FromROSMapServerYAML(const std::string &) --> class mrpt::maps::COccupancyGridMap2D", pybind11::arg("yamlFilePath"));
		cl.def("determineMatching2D", (void (mrpt::maps::COccupancyGridMap2D::*)(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose2D &, class mrpt::tfest::TMatchingPairListTempl<float> &, const struct mrpt::maps::TMatchingParams &, struct mrpt::maps::TMatchingExtraResults &) const) &mrpt::maps::COccupancyGridMap2D::determineMatching2D, "See the base class for more details: In this class it is implemented as\n correspondences of the passed points map to occupied cells.\n NOTICE: That the \"z\" dimension is ignored in the points. Clip the points\n as appropiated if needed before calling this method.\n\n \n computeMatching3DWith\n\nC++: mrpt::maps::COccupancyGridMap2D::determineMatching2D(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose2D &, class mrpt::tfest::TMatchingPairListTempl<float> &, const struct mrpt::maps::TMatchingParams &, struct mrpt::maps::TMatchingExtraResults &) const --> void", pybind11::arg("otherMap"), pybind11::arg("otherMapPose"), pybind11::arg("correspondences"), pybind11::arg("params"), pybind11::arg("extraResults"));
		cl.def("compute3DMatchingRatio", (float (mrpt::maps::COccupancyGridMap2D::*)(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose3D &, const struct mrpt::maps::TMatchingRatioParams &) const) &mrpt::maps::COccupancyGridMap2D::compute3DMatchingRatio, "See docs in base class: in this class this always returns 0 \n\nC++: mrpt::maps::COccupancyGridMap2D::compute3DMatchingRatio(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose3D &, const struct mrpt::maps::TMatchingRatioParams &) const --> float", pybind11::arg("otherMap"), pybind11::arg("otherMapPose"), pybind11::arg("params"));
		cl.def("saveMetricMapRepresentationToFile", (void (mrpt::maps::COccupancyGridMap2D::*)(const std::string &) const) &mrpt::maps::COccupancyGridMap2D::saveMetricMapRepresentationToFile, "This virtual method saves the map to a file \"filNamePrefix\"+<\n some_file_extension >, as an image or in any other applicable way (Notice\n that other methods to save the map may be implemented in classes\n implementing this virtual interface).  \n\nC++: mrpt::maps::COccupancyGridMap2D::saveMetricMapRepresentationToFile(const std::string &) const --> void", pybind11::arg("filNamePrefix"));
		cl.def("asString", (std::string (mrpt::maps::COccupancyGridMap2D::*)() const) &mrpt::maps::COccupancyGridMap2D::asString, "Returns a short description of the map. \n\nC++: mrpt::maps::COccupancyGridMap2D::asString() const --> std::string");
		cl.def("nn_has_indices_or_ids", (bool (mrpt::maps::COccupancyGridMap2D::*)() const) &mrpt::maps::COccupancyGridMap2D::nn_has_indices_or_ids, "@{ \n\nC++: mrpt::maps::COccupancyGridMap2D::nn_has_indices_or_ids() const --> bool");
		cl.def("nn_index_count", (size_t (mrpt::maps::COccupancyGridMap2D::*)() const) &mrpt::maps::COccupancyGridMap2D::nn_index_count, "C++: mrpt::maps::COccupancyGridMap2D::nn_index_count() const --> size_t");
		cl.def("nn_single_search", (bool (mrpt::maps::COccupancyGridMap2D::*)(const struct mrpt::math::TPoint3D_<float> &, struct mrpt::math::TPoint3D_<float> &, float &, uint64_t &) const) &mrpt::maps::COccupancyGridMap2D::nn_single_search, "C++: mrpt::maps::COccupancyGridMap2D::nn_single_search(const struct mrpt::math::TPoint3D_<float> &, struct mrpt::math::TPoint3D_<float> &, float &, uint64_t &) const --> bool", pybind11::arg("query"), pybind11::arg("result"), pybind11::arg("out_dist_sqr"), pybind11::arg("resultIndexOrID"));
		cl.def("nn_single_search", (bool (mrpt::maps::COccupancyGridMap2D::*)(const struct mrpt::math::TPoint2D_<float> &, struct mrpt::math::TPoint2D_<float> &, float &, uint64_t &) const) &mrpt::maps::COccupancyGridMap2D::nn_single_search, "C++: mrpt::maps::COccupancyGridMap2D::nn_single_search(const struct mrpt::math::TPoint2D_<float> &, struct mrpt::math::TPoint2D_<float> &, float &, uint64_t &) const --> bool", pybind11::arg("query"), pybind11::arg("result"), pybind11::arg("out_dist_sqr"), pybind11::arg("resultIndexOrID"));
		cl.def("assign", (class mrpt::maps::COccupancyGridMap2D & (mrpt::maps::COccupancyGridMap2D::*)(const class mrpt::maps::COccupancyGridMap2D &)) &mrpt::maps::COccupancyGridMap2D::operator=, "C++: mrpt::maps::COccupancyGridMap2D::operator=(const class mrpt::maps::COccupancyGridMap2D &) --> class mrpt::maps::COccupancyGridMap2D &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::maps::COccupancyGridMap2D::TUpdateCellsInfoChangeOnly file:mrpt/maps/COccupancyGridMap2D.h line:200
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::COccupancyGridMap2D::TUpdateCellsInfoChangeOnly, std::shared_ptr<mrpt::maps::COccupancyGridMap2D::TUpdateCellsInfoChangeOnly>> cl(enclosing_class, "TUpdateCellsInfoChangeOnly", "An internal structure for storing data related to counting the new\n information apported by some observation ");
			cl.def( pybind11::init( [](){ return new mrpt::maps::COccupancyGridMap2D::TUpdateCellsInfoChangeOnly(); } ) );
			cl.def( pybind11::init( [](mrpt::maps::COccupancyGridMap2D::TUpdateCellsInfoChangeOnly const &o){ return new mrpt::maps::COccupancyGridMap2D::TUpdateCellsInfoChangeOnly(o); } ) );
			cl.def_readwrite("enabled", &mrpt::maps::COccupancyGridMap2D::TUpdateCellsInfoChangeOnly::enabled);
			cl.def_readwrite("I_change", &mrpt::maps::COccupancyGridMap2D::TUpdateCellsInfoChangeOnly::I_change);
			cl.def_readwrite("cellsUpdated", &mrpt::maps::COccupancyGridMap2D::TUpdateCellsInfoChangeOnly::cellsUpdated);
			cl.def_readwrite("laserRaysSkip", &mrpt::maps::COccupancyGridMap2D::TUpdateCellsInfoChangeOnly::laserRaysSkip);
			cl.def("assign", (struct mrpt::maps::COccupancyGridMap2D::TUpdateCellsInfoChangeOnly & (mrpt::maps::COccupancyGridMap2D::TUpdateCellsInfoChangeOnly::*)(const struct mrpt::maps::COccupancyGridMap2D::TUpdateCellsInfoChangeOnly &)) &mrpt::maps::COccupancyGridMap2D::TUpdateCellsInfoChangeOnly::operator=, "C++: mrpt::maps::COccupancyGridMap2D::TUpdateCellsInfoChangeOnly::operator=(const struct mrpt::maps::COccupancyGridMap2D::TUpdateCellsInfoChangeOnly &) --> struct mrpt::maps::COccupancyGridMap2D::TUpdateCellsInfoChangeOnly &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::maps::COccupancyGridMap2D::TEntropyInfo file:mrpt/maps/COccupancyGridMap2D.h line:410
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::COccupancyGridMap2D::TEntropyInfo, std::shared_ptr<mrpt::maps::COccupancyGridMap2D::TEntropyInfo>> cl(enclosing_class, "TEntropyInfo", "Used for returning entropy related information \n computeEntropy ");
			cl.def( pybind11::init( [](){ return new mrpt::maps::COccupancyGridMap2D::TEntropyInfo(); } ) );
			cl.def_readwrite("H", &mrpt::maps::COccupancyGridMap2D::TEntropyInfo::H);
			cl.def_readwrite("I", &mrpt::maps::COccupancyGridMap2D::TEntropyInfo::I);
			cl.def_readwrite("mean_H", &mrpt::maps::COccupancyGridMap2D::TEntropyInfo::mean_H);
			cl.def_readwrite("mean_I", &mrpt::maps::COccupancyGridMap2D::TEntropyInfo::mean_I);
			cl.def_readwrite("effectiveMappedArea", &mrpt::maps::COccupancyGridMap2D::TEntropyInfo::effectiveMappedArea);
			cl.def_readwrite("effectiveMappedCells", &mrpt::maps::COccupancyGridMap2D::TEntropyInfo::effectiveMappedCells);
		}

		{ // mrpt::maps::COccupancyGridMap2D::TInsertionOptions file:mrpt/maps/COccupancyGridMap2D.h line:435
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::COccupancyGridMap2D::TInsertionOptions, std::shared_ptr<mrpt::maps::COccupancyGridMap2D::TInsertionOptions>, PyCallBack_mrpt_maps_COccupancyGridMap2D_TInsertionOptions, mrpt::config::CLoadableOptions> cl(enclosing_class, "TInsertionOptions", "With this struct options are provided to the observation insertion\n process.\n \n\n CObservation::insertIntoGridMap ");
			cl.def( pybind11::init( [](){ return new mrpt::maps::COccupancyGridMap2D::TInsertionOptions(); }, [](){ return new PyCallBack_mrpt_maps_COccupancyGridMap2D_TInsertionOptions(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_maps_COccupancyGridMap2D_TInsertionOptions const &o){ return new PyCallBack_mrpt_maps_COccupancyGridMap2D_TInsertionOptions(o); } ) );
			cl.def( pybind11::init( [](mrpt::maps::COccupancyGridMap2D::TInsertionOptions const &o){ return new mrpt::maps::COccupancyGridMap2D::TInsertionOptions(o); } ) );
			cl.def_readwrite("mapAltitude", &mrpt::maps::COccupancyGridMap2D::TInsertionOptions::mapAltitude);
			cl.def_readwrite("useMapAltitude", &mrpt::maps::COccupancyGridMap2D::TInsertionOptions::useMapAltitude);
			cl.def_readwrite("maxDistanceInsertion", &mrpt::maps::COccupancyGridMap2D::TInsertionOptions::maxDistanceInsertion);
			cl.def_readwrite("maxOccupancyUpdateCertainty", &mrpt::maps::COccupancyGridMap2D::TInsertionOptions::maxOccupancyUpdateCertainty);
			cl.def_readwrite("maxFreenessUpdateCertainty", &mrpt::maps::COccupancyGridMap2D::TInsertionOptions::maxFreenessUpdateCertainty);
			cl.def_readwrite("maxFreenessInvalidRanges", &mrpt::maps::COccupancyGridMap2D::TInsertionOptions::maxFreenessInvalidRanges);
			cl.def_readwrite("considerInvalidRangesAsFreeSpace", &mrpt::maps::COccupancyGridMap2D::TInsertionOptions::considerInvalidRangesAsFreeSpace);
			cl.def_readwrite("decimation", &mrpt::maps::COccupancyGridMap2D::TInsertionOptions::decimation);
			cl.def_readwrite("horizontalTolerance", &mrpt::maps::COccupancyGridMap2D::TInsertionOptions::horizontalTolerance);
			cl.def_readwrite("CFD_features_gaussian_size", &mrpt::maps::COccupancyGridMap2D::TInsertionOptions::CFD_features_gaussian_size);
			cl.def_readwrite("CFD_features_median_size", &mrpt::maps::COccupancyGridMap2D::TInsertionOptions::CFD_features_median_size);
			cl.def_readwrite("wideningBeamsWithDistance", &mrpt::maps::COccupancyGridMap2D::TInsertionOptions::wideningBeamsWithDistance);
			cl.def("loadFromConfigFile", (void (mrpt::maps::COccupancyGridMap2D::TInsertionOptions::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::maps::COccupancyGridMap2D::TInsertionOptions::loadFromConfigFile, "This method load the options from a \".ini\" file.\n   Only those parameters found in the given \"section\" and having\n   the same name that the variable are loaded. Those not found in\n   the file will stay with their previous values (usually the default\n   values loaded at initialization). An example of an \".ini\" file:\n  \n\n\n\n\n\n     \n\nC++: mrpt::maps::COccupancyGridMap2D::TInsertionOptions::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
			cl.def("assign", (class mrpt::maps::COccupancyGridMap2D::TInsertionOptions & (mrpt::maps::COccupancyGridMap2D::TInsertionOptions::*)(const class mrpt::maps::COccupancyGridMap2D::TInsertionOptions &)) &mrpt::maps::COccupancyGridMap2D::TInsertionOptions::operator=, "C++: mrpt::maps::COccupancyGridMap2D::TInsertionOptions::operator=(const class mrpt::maps::COccupancyGridMap2D::TInsertionOptions &) --> class mrpt::maps::COccupancyGridMap2D::TInsertionOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions file:mrpt/maps/COccupancyGridMap2D.h line:515
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions, std::shared_ptr<mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions>, PyCallBack_mrpt_maps_COccupancyGridMap2D_TLikelihoodOptions, mrpt::config::CLoadableOptions> cl(enclosing_class, "TLikelihoodOptions", "With this struct options are provided to the observation likelihood\n computation process ");
			cl.def( pybind11::init( [](){ return new mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions(); }, [](){ return new PyCallBack_mrpt_maps_COccupancyGridMap2D_TLikelihoodOptions(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_maps_COccupancyGridMap2D_TLikelihoodOptions const &o){ return new PyCallBack_mrpt_maps_COccupancyGridMap2D_TLikelihoodOptions(o); } ) );
			cl.def( pybind11::init( [](mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions const &o){ return new mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions(o); } ) );
			cl.def_readwrite("likelihoodMethod", &mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions::likelihoodMethod);
			cl.def_readwrite("LF_stdHit", &mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions::LF_stdHit);
			cl.def_readwrite("LF_zHit", &mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions::LF_zHit);
			cl.def_readwrite("LF_zRandom", &mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions::LF_zRandom);
			cl.def_readwrite("LF_maxRange", &mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions::LF_maxRange);
			cl.def_readwrite("LF_decimation", &mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions::LF_decimation);
			cl.def_readwrite("LF_maxCorrsDistance", &mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions::LF_maxCorrsDistance);
			cl.def_readwrite("LF_useSquareDist", &mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions::LF_useSquareDist);
			cl.def_readwrite("LF_alternateAverageMethod", &mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions::LF_alternateAverageMethod);
			cl.def_readwrite("MI_exponent", &mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions::MI_exponent);
			cl.def_readwrite("MI_skip_rays", &mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions::MI_skip_rays);
			cl.def_readwrite("MI_ratio_max_distance", &mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions::MI_ratio_max_distance);
			cl.def_readwrite("rayTracing_useDistanceFilter", &mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions::rayTracing_useDistanceFilter);
			cl.def_readwrite("rayTracing_decimation", &mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions::rayTracing_decimation);
			cl.def_readwrite("rayTracing_stdHit", &mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions::rayTracing_stdHit);
			cl.def_readwrite("consensus_takeEachRange", &mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions::consensus_takeEachRange);
			cl.def_readwrite("consensus_pow", &mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions::consensus_pow);
			cl.def_readwrite("OWA_weights", &mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions::OWA_weights);
			cl.def_readwrite("enableLikelihoodCache", &mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions::enableLikelihoodCache);
			cl.def("loadFromConfigFile", (void (mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions::loadFromConfigFile, "This method load the options from a \".ini\" file.\n   Only those parameters found in the given \"section\" and having\n   the same name that the variable are loaded. Those not found in\n   the file will stay with their previous values (usually the default\n   values loaded at initialization). An example of an \".ini\" file:\n  \n\n\n\n\n\n     \n\nC++: mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
			cl.def("assign", (class mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions & (mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions::*)(const class mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions &)) &mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions::operator=, "C++: mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions::operator=(const class mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions &) --> class mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::maps::COccupancyGridMap2D::TLikelihoodOutput file:mrpt/maps/COccupancyGridMap2D.h line:600
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::COccupancyGridMap2D::TLikelihoodOutput, std::shared_ptr<mrpt::maps::COccupancyGridMap2D::TLikelihoodOutput>> cl(enclosing_class, "TLikelihoodOutput", "Some members of this struct will contain intermediate or output data\n after calling \"computeObservationLikelihood\" for some likelihood\n functions ");
			cl.def( pybind11::init( [](mrpt::maps::COccupancyGridMap2D::TLikelihoodOutput const &o){ return new mrpt::maps::COccupancyGridMap2D::TLikelihoodOutput(o); } ) );
			cl.def( pybind11::init( [](){ return new mrpt::maps::COccupancyGridMap2D::TLikelihoodOutput(); } ) );
			cl.def_readwrite("OWA_pairList", &mrpt::maps::COccupancyGridMap2D::TLikelihoodOutput::OWA_pairList);
			cl.def_readwrite("OWA_individualLikValues", &mrpt::maps::COccupancyGridMap2D::TLikelihoodOutput::OWA_individualLikValues);
			cl.def("assign", (struct mrpt::maps::COccupancyGridMap2D::TLikelihoodOutput & (mrpt::maps::COccupancyGridMap2D::TLikelihoodOutput::*)(const struct mrpt::maps::COccupancyGridMap2D::TLikelihoodOutput &)) &mrpt::maps::COccupancyGridMap2D::TLikelihoodOutput::operator=, "C++: mrpt::maps::COccupancyGridMap2D::TLikelihoodOutput::operator=(const struct mrpt::maps::COccupancyGridMap2D::TLikelihoodOutput &) --> struct mrpt::maps::COccupancyGridMap2D::TLikelihoodOutput &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::maps::COccupancyGridMap2D::TLaserSimulUncertaintyParams file:mrpt/maps/COccupancyGridMap2D.h line:812
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::COccupancyGridMap2D::TLaserSimulUncertaintyParams, std::shared_ptr<mrpt::maps::COccupancyGridMap2D::TLaserSimulUncertaintyParams>> cl(enclosing_class, "TLaserSimulUncertaintyParams", "Input params for laserScanSimulatorWithUncertainty() ");
			cl.def( pybind11::init( [](){ return new mrpt::maps::COccupancyGridMap2D::TLaserSimulUncertaintyParams(); } ) );
			cl.def_readwrite("method", &mrpt::maps::COccupancyGridMap2D::TLaserSimulUncertaintyParams::method);
			cl.def_readwrite("UT_alpha", &mrpt::maps::COccupancyGridMap2D::TLaserSimulUncertaintyParams::UT_alpha);
			cl.def_readwrite("UT_kappa", &mrpt::maps::COccupancyGridMap2D::TLaserSimulUncertaintyParams::UT_kappa);
			cl.def_readwrite("UT_beta", &mrpt::maps::COccupancyGridMap2D::TLaserSimulUncertaintyParams::UT_beta);
			cl.def_readwrite("MC_samples", &mrpt::maps::COccupancyGridMap2D::TLaserSimulUncertaintyParams::MC_samples);
			cl.def_readwrite("robotPose", &mrpt::maps::COccupancyGridMap2D::TLaserSimulUncertaintyParams::robotPose);
			cl.def_readwrite("aperture", &mrpt::maps::COccupancyGridMap2D::TLaserSimulUncertaintyParams::aperture);
			cl.def_readwrite("rightToLeft", &mrpt::maps::COccupancyGridMap2D::TLaserSimulUncertaintyParams::rightToLeft);
			cl.def_readwrite("maxRange", &mrpt::maps::COccupancyGridMap2D::TLaserSimulUncertaintyParams::maxRange);
			cl.def_readwrite("sensorPose", &mrpt::maps::COccupancyGridMap2D::TLaserSimulUncertaintyParams::sensorPose);
			cl.def_readwrite("nRays", &mrpt::maps::COccupancyGridMap2D::TLaserSimulUncertaintyParams::nRays);
			cl.def_readwrite("rangeNoiseStd", &mrpt::maps::COccupancyGridMap2D::TLaserSimulUncertaintyParams::rangeNoiseStd);
			cl.def_readwrite("angleNoiseStd", &mrpt::maps::COccupancyGridMap2D::TLaserSimulUncertaintyParams::angleNoiseStd);
			cl.def_readwrite("decimation", &mrpt::maps::COccupancyGridMap2D::TLaserSimulUncertaintyParams::decimation);
			cl.def_readwrite("threshold", &mrpt::maps::COccupancyGridMap2D::TLaserSimulUncertaintyParams::threshold);
		}

		{ // mrpt::maps::COccupancyGridMap2D::TLaserSimulUncertaintyResult file:mrpt/maps/COccupancyGridMap2D.h line:864
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::COccupancyGridMap2D::TLaserSimulUncertaintyResult, std::shared_ptr<mrpt::maps::COccupancyGridMap2D::TLaserSimulUncertaintyResult>> cl(enclosing_class, "TLaserSimulUncertaintyResult", "Output params for laserScanSimulatorWithUncertainty() ");
			cl.def( pybind11::init( [](){ return new mrpt::maps::COccupancyGridMap2D::TLaserSimulUncertaintyResult(); } ) );
			cl.def_readwrite("scanWithUncert", &mrpt::maps::COccupancyGridMap2D::TLaserSimulUncertaintyResult::scanWithUncert);
		}

		{ // mrpt::maps::COccupancyGridMap2D::TCriticalPointsList file:mrpt/maps/COccupancyGridMap2D.h line:1083
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::COccupancyGridMap2D::TCriticalPointsList, std::shared_ptr<mrpt::maps::COccupancyGridMap2D::TCriticalPointsList>> cl(enclosing_class, "TCriticalPointsList", "The structure used to store the set of Voronoi diagram\n    critical points.\n \n\n findCriticalPoints");
			cl.def( pybind11::init( [](){ return new mrpt::maps::COccupancyGridMap2D::TCriticalPointsList(); } ) );
			cl.def( pybind11::init( [](mrpt::maps::COccupancyGridMap2D::TCriticalPointsList const &o){ return new mrpt::maps::COccupancyGridMap2D::TCriticalPointsList(o); } ) );
			cl.def_readwrite("x", &mrpt::maps::COccupancyGridMap2D::TCriticalPointsList::x);
			cl.def_readwrite("y", &mrpt::maps::COccupancyGridMap2D::TCriticalPointsList::y);
			cl.def_readwrite("clearance", &mrpt::maps::COccupancyGridMap2D::TCriticalPointsList::clearance);
			cl.def_readwrite("x_basis1", &mrpt::maps::COccupancyGridMap2D::TCriticalPointsList::x_basis1);
			cl.def_readwrite("y_basis1", &mrpt::maps::COccupancyGridMap2D::TCriticalPointsList::y_basis1);
			cl.def_readwrite("x_basis2", &mrpt::maps::COccupancyGridMap2D::TCriticalPointsList::x_basis2);
			cl.def_readwrite("y_basis2", &mrpt::maps::COccupancyGridMap2D::TCriticalPointsList::y_basis2);
			cl.def("assign", (struct mrpt::maps::COccupancyGridMap2D::TCriticalPointsList & (mrpt::maps::COccupancyGridMap2D::TCriticalPointsList::*)(const struct mrpt::maps::COccupancyGridMap2D::TCriticalPointsList &)) &mrpt::maps::COccupancyGridMap2D::TCriticalPointsList::operator=, "C++: mrpt::maps::COccupancyGridMap2D::TCriticalPointsList::operator=(const struct mrpt::maps::COccupancyGridMap2D::TCriticalPointsList &) --> struct mrpt::maps::COccupancyGridMap2D::TCriticalPointsList &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::maps::COccupancyGridMap2D::TMapDefinitionBase file: line:74
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::COccupancyGridMap2D::TMapDefinitionBase, std::shared_ptr<mrpt::maps::COccupancyGridMap2D::TMapDefinitionBase>> cl(enclosing_class, "TMapDefinitionBase", "");
		}

		{ // mrpt::maps::COccupancyGridMap2D::TMapDefinition file: line:78
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::COccupancyGridMap2D::TMapDefinition, std::shared_ptr<mrpt::maps::COccupancyGridMap2D::TMapDefinition>, PyCallBack_mrpt_maps_COccupancyGridMap2D_TMapDefinition, mrpt::maps::COccupancyGridMap2D::TMapDefinitionBase> cl(enclosing_class, "TMapDefinition", "");
			cl.def( pybind11::init( [](){ return new mrpt::maps::COccupancyGridMap2D::TMapDefinition(); }, [](){ return new PyCallBack_mrpt_maps_COccupancyGridMap2D_TMapDefinition(); } ) );
			cl.def_readwrite("min_x", &mrpt::maps::COccupancyGridMap2D::TMapDefinition::min_x);
			cl.def_readwrite("max_x", &mrpt::maps::COccupancyGridMap2D::TMapDefinition::max_x);
			cl.def_readwrite("min_y", &mrpt::maps::COccupancyGridMap2D::TMapDefinition::min_y);
			cl.def_readwrite("max_y", &mrpt::maps::COccupancyGridMap2D::TMapDefinition::max_y);
			cl.def_readwrite("resolution", &mrpt::maps::COccupancyGridMap2D::TMapDefinition::resolution);
			cl.def_readwrite("insertionOpts", &mrpt::maps::COccupancyGridMap2D::TMapDefinition::insertionOpts);
			cl.def_readwrite("likelihoodOpts", &mrpt::maps::COccupancyGridMap2D::TMapDefinition::likelihoodOpts);
		}

	}
}
