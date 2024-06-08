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
#include <mrpt/img/TColor.h>
#include <mrpt/img/color_maps.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/COccupancyGridMap3D.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/NearestNeighborsCapable.h>
#include <mrpt/maps/TMetricMapInitializer.h>
#include <mrpt/maps/metric_map_types.h>
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
#include <mrpt/math/TTwist3D.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/obs/CSensoryFrame.h>
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

// mrpt::maps::COccupancyGridMap3D file:mrpt/maps/COccupancyGridMap3D.h line:42
struct PyCallBack_mrpt_maps_COccupancyGridMap3D : public mrpt::maps::COccupancyGridMap3D {
	using mrpt::maps::COccupancyGridMap3D::COccupancyGridMap3D;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap3D *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return COccupancyGridMap3D::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap3D *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return COccupancyGridMap3D::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap3D *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return COccupancyGridMap3D::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap3D *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COccupancyGridMap3D::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap3D *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COccupancyGridMap3D::serializeFrom(a0, a1);
	}
	void OnPostSuccesfulInsertObs(const class mrpt::obs::CObservation & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap3D *>(this), "OnPostSuccesfulInsertObs");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COccupancyGridMap3D::OnPostSuccesfulInsertObs(a0);
	}
	void internal_clear() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap3D *>(this), "internal_clear");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COccupancyGridMap3D::internal_clear();
	}
	void getVisualizationInto(class mrpt::opengl::CSetOfObjects & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap3D *>(this), "getVisualizationInto");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COccupancyGridMap3D::getVisualizationInto(a0);
	}
	struct mrpt::math::TBoundingBox_<float> boundingBox() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap3D *>(this), "boundingBox");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TBoundingBox_<float>>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TBoundingBox_<float>> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TBoundingBox_<float>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TBoundingBox_<float>>(std::move(o));
		}
		return COccupancyGridMap3D::boundingBox();
	}
	bool isEmpty() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap3D *>(this), "isEmpty");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return COccupancyGridMap3D::isEmpty();
	}
	void determineMatching2D(const class mrpt::maps::CMetricMap * a0, const class mrpt::poses::CPose2D & a1, class mrpt::tfest::TMatchingPairListTempl<float> & a2, const struct mrpt::maps::TMatchingParams & a3, struct mrpt::maps::TMatchingExtraResults & a4) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap3D *>(this), "determineMatching2D");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COccupancyGridMap3D::determineMatching2D(a0, a1, a2, a3, a4);
	}
	float compute3DMatchingRatio(const class mrpt::maps::CMetricMap * a0, const class mrpt::poses::CPose3D & a1, const struct mrpt::maps::TMatchingRatioParams & a2) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap3D *>(this), "compute3DMatchingRatio");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<float>::value) {
				static pybind11::detail::override_caster_t<float> caster;
				return pybind11::detail::cast_ref<float>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<float>(std::move(o));
		}
		return COccupancyGridMap3D::compute3DMatchingRatio(a0, a1, a2);
	}
	void saveMetricMapRepresentationToFile(const std::string & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap3D *>(this), "saveMetricMapRepresentationToFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COccupancyGridMap3D::saveMetricMapRepresentationToFile(a0);
	}
	std::string asString() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap3D *>(this), "asString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return COccupancyGridMap3D::asString();
	}
	bool nn_has_indices_or_ids() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap3D *>(this), "nn_has_indices_or_ids");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return COccupancyGridMap3D::nn_has_indices_or_ids();
	}
	size_t nn_index_count() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap3D *>(this), "nn_index_count");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return COccupancyGridMap3D::nn_index_count();
	}
	bool nn_single_search(const struct mrpt::math::TPoint3D_<float> & a0, struct mrpt::math::TPoint3D_<float> & a1, float & a2, uint64_t & a3) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap3D *>(this), "nn_single_search");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return COccupancyGridMap3D::nn_single_search(a0, a1, a2, a3);
	}
	bool nn_single_search(const struct mrpt::math::TPoint2D_<float> & a0, struct mrpt::math::TPoint2D_<float> & a1, float & a2, uint64_t & a3) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap3D *>(this), "nn_single_search");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return COccupancyGridMap3D::nn_single_search(a0, a1, a2, a3);
	}
	bool canComputeObservationLikelihood(const class mrpt::obs::CObservation & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap3D *>(this), "canComputeObservationLikelihood");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap3D *>(this), "determineMatching3D");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap3D *>(this), "auxParticleFilterCleanUp");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap3D *>(this), "squareDistanceToClosestCorrespondence");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap3D *>(this), "nn_prepare_for_2d_queries");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap3D *>(this), "nn_prepare_for_3d_queries");
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

// mrpt::maps::COccupancyGridMap3D::TInsertionOptions file:mrpt/maps/COccupancyGridMap3D.h line:202
struct PyCallBack_mrpt_maps_COccupancyGridMap3D_TInsertionOptions : public mrpt::maps::COccupancyGridMap3D::TInsertionOptions {
	using mrpt::maps::COccupancyGridMap3D::TInsertionOptions::TInsertionOptions;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap3D::TInsertionOptions *>(this), "loadFromConfigFile");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap3D::TInsertionOptions *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TInsertionOptions::saveToConfigFile(a0, a1);
	}
};

// mrpt::maps::COccupancyGridMap3D::TLikelihoodOptions file:mrpt/maps/COccupancyGridMap3D.h line:262
struct PyCallBack_mrpt_maps_COccupancyGridMap3D_TLikelihoodOptions : public mrpt::maps::COccupancyGridMap3D::TLikelihoodOptions {
	using mrpt::maps::COccupancyGridMap3D::TLikelihoodOptions::TLikelihoodOptions;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap3D::TLikelihoodOptions *>(this), "loadFromConfigFile");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap3D::TLikelihoodOptions *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TLikelihoodOptions::saveToConfigFile(a0, a1);
	}
};

// mrpt::maps::COccupancyGridMap3D::TMapDefinition file: line:78
struct PyCallBack_mrpt_maps_COccupancyGridMap3D_TMapDefinition : public mrpt::maps::COccupancyGridMap3D::TMapDefinition {
	using mrpt::maps::COccupancyGridMap3D::TMapDefinition::TMapDefinition;

	void loadFromConfigFile_map_specific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap3D::TMapDefinition *>(this), "loadFromConfigFile_map_specific");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap3D::TMapDefinition *>(this), "loadFromConfigFile");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::COccupancyGridMap3D::TMapDefinition *>(this), "saveToConfigFile");
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

void bind_mrpt_maps_COccupancyGridMap3D(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::maps::COccupancyGridMap3D file:mrpt/maps/COccupancyGridMap3D.h line:42
		pybind11::class_<mrpt::maps::COccupancyGridMap3D, std::shared_ptr<mrpt::maps::COccupancyGridMap3D>, PyCallBack_mrpt_maps_COccupancyGridMap3D, mrpt::maps::CMetricMap, mrpt::maps::CLogOddsGridMap3D<signed char>, mrpt::maps::NearestNeighborsCapable> cl(M("mrpt::maps"), "COccupancyGridMap3D", "A 3D occupancy grid map with a regular, even distribution of voxels.\n\n This is a faster alternative to COctoMap, but use with caution with limited\nmap extensions, since it could easily exaust available memory.\n\n Each voxel follows a Bernoulli probability distribution: a value of 0 means\ncertainly occupied, 1 means a certainly empty voxel. Initially 0.5 means\nuncertainty.\n\n An alternative, sparse representation of a 3D map is provided\n via mrpt::maps::CVoxelMap and mrpt::maps::CVoxelMapRGB\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::maps::COccupancyGridMap3D(); }, [](){ return new PyCallBack_mrpt_maps_COccupancyGridMap3D(); } ), "doc");
		cl.def( pybind11::init( [](const struct mrpt::math::TPoint3D_<double> & a0){ return new mrpt::maps::COccupancyGridMap3D(a0); }, [](const struct mrpt::math::TPoint3D_<double> & a0){ return new PyCallBack_mrpt_maps_COccupancyGridMap3D(a0); } ), "doc");
		cl.def( pybind11::init( [](const struct mrpt::math::TPoint3D_<double> & a0, const struct mrpt::math::TPoint3D_<double> & a1){ return new mrpt::maps::COccupancyGridMap3D(a0, a1); }, [](const struct mrpt::math::TPoint3D_<double> & a0, const struct mrpt::math::TPoint3D_<double> & a1){ return new PyCallBack_mrpt_maps_COccupancyGridMap3D(a0, a1); } ), "doc");
		cl.def( pybind11::init<const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &, float>(), pybind11::arg("corner_min"), pybind11::arg("corner_max"), pybind11::arg("resolution") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_maps_COccupancyGridMap3D const &o){ return new PyCallBack_mrpt_maps_COccupancyGridMap3D(o); } ) );
		cl.def( pybind11::init( [](mrpt::maps::COccupancyGridMap3D const &o){ return new mrpt::maps::COccupancyGridMap3D(o); } ) );

		pybind11::enum_<mrpt::maps::COccupancyGridMap3D::TLikelihoodMethod>(cl, "TLikelihoodMethod", pybind11::arithmetic(), "The type for selecting a likelihood computation method ")
			.value("lmLikelihoodField_Thrun", mrpt::maps::COccupancyGridMap3D::lmLikelihoodField_Thrun)
			.value("lmRayTracing", mrpt::maps::COccupancyGridMap3D::lmRayTracing)
			.export_values();

		cl.def_readwrite("insertionOptions", &mrpt::maps::COccupancyGridMap3D::insertionOptions);
		cl.def_readwrite("likelihoodOptions", &mrpt::maps::COccupancyGridMap3D::likelihoodOptions);
		cl.def_readwrite("renderingOptions", &mrpt::maps::COccupancyGridMap3D::renderingOptions);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::maps::COccupancyGridMap3D::GetRuntimeClassIdStatic, "C++: mrpt::maps::COccupancyGridMap3D::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::maps::COccupancyGridMap3D::*)() const) &mrpt::maps::COccupancyGridMap3D::GetRuntimeClass, "C++: mrpt::maps::COccupancyGridMap3D::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::maps::COccupancyGridMap3D::*)() const) &mrpt::maps::COccupancyGridMap3D::clone, "C++: mrpt::maps::COccupancyGridMap3D::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::maps::COccupancyGridMap3D::CreateObject, "C++: mrpt::maps::COccupancyGridMap3D::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("fill", [](mrpt::maps::COccupancyGridMap3D &o) -> void { return o.fill(); }, "");
		cl.def("fill", (void (mrpt::maps::COccupancyGridMap3D::*)(float)) &mrpt::maps::COccupancyGridMap3D::fill, "Fills all the voxels with a default value. \n\nC++: mrpt::maps::COccupancyGridMap3D::fill(float) --> void", pybind11::arg("default_value"));
		cl.def("setSize", [](mrpt::maps::COccupancyGridMap3D &o, const struct mrpt::math::TPoint3D_<double> & a0, const struct mrpt::math::TPoint3D_<double> & a1, double const & a2) -> void { return o.setSize(a0, a1, a2); }, "", pybind11::arg("corner_min"), pybind11::arg("corner_max"), pybind11::arg("resolution"));
		cl.def("setSize", (void (mrpt::maps::COccupancyGridMap3D::*)(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &, double, float)) &mrpt::maps::COccupancyGridMap3D::setSize, "Change the size of gridmap, erasing all its previous contents.\n \n\n The new size of voxels.\n \n\n The value of voxels, tipically 0.5.\n \n\n ResizeGrid\n\nC++: mrpt::maps::COccupancyGridMap3D::setSize(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &, double, float) --> void", pybind11::arg("corner_min"), pybind11::arg("corner_max"), pybind11::arg("resolution"), pybind11::arg("default_value"));
		cl.def("resizeGrid", [](mrpt::maps::COccupancyGridMap3D &o, const struct mrpt::math::TPoint3D_<double> & a0, const struct mrpt::math::TPoint3D_<double> & a1) -> void { return o.resizeGrid(a0, a1); }, "", pybind11::arg("corner_min"), pybind11::arg("corner_max"));
		cl.def("resizeGrid", (void (mrpt::maps::COccupancyGridMap3D::*)(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &, float)) &mrpt::maps::COccupancyGridMap3D::resizeGrid, "Change the size of gridmap, maintaining previous contents.\n \n\n Value of new voxels, tipically 0.5\n \n\n setSize()\n\nC++: mrpt::maps::COccupancyGridMap3D::resizeGrid(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &, float) --> void", pybind11::arg("corner_min"), pybind11::arg("corner_max"), pybind11::arg("new_voxels_default_value"));
		cl.def_static("l2p", (float (*)(const signed char)) &mrpt::maps::COccupancyGridMap3D::l2p, "Scales an integer representation of the log-odd into a real valued\n probability in [0,1], using p=exp(l)/(1+exp(l))  \n\nC++: mrpt::maps::COccupancyGridMap3D::l2p(const signed char) --> float", pybind11::arg("l"));
		cl.def_static("l2p_255", (uint8_t (*)(const signed char)) &mrpt::maps::COccupancyGridMap3D::l2p_255, "Scales an integer representation of the log-odd into a linear scale\n [0,255], using p=exp(l)/(1+exp(l)) \n\nC++: mrpt::maps::COccupancyGridMap3D::l2p_255(const signed char) --> uint8_t", pybind11::arg("l"));
		cl.def_static("p2l", (signed char (*)(const float)) &mrpt::maps::COccupancyGridMap3D::p2l, "Scales a real valued probability in [0,1] to an integer representation\n of: log(p)-log(1-p)  in the valid range of voxelType \n\nC++: mrpt::maps::COccupancyGridMap3D::p2l(const float) --> signed char", pybind11::arg("p"));
		cl.def("updateCell", (void (mrpt::maps::COccupancyGridMap3D::*)(int, int, int, float)) &mrpt::maps::COccupancyGridMap3D::updateCell, "Performs the Bayesian fusion of a new observation of a cell  \n\n updateInfoChangeOnly, updateCell_fast_occupied, updateCell_fast_free \n\nC++: mrpt::maps::COccupancyGridMap3D::updateCell(int, int, int, float) --> void", pybind11::arg("cx_idx"), pybind11::arg("cy_idx"), pybind11::arg("cz_idx"), pybind11::arg("v"));
		cl.def("setCellFreeness", (void (mrpt::maps::COccupancyGridMap3D::*)(unsigned int, unsigned int, unsigned int, float)) &mrpt::maps::COccupancyGridMap3D::setCellFreeness, "Change the contents [0,1] (0:occupied, 1:free) of a voxel, given its\n index. \n\nC++: mrpt::maps::COccupancyGridMap3D::setCellFreeness(unsigned int, unsigned int, unsigned int, float) --> void", pybind11::arg("cx"), pybind11::arg("cy"), pybind11::arg("cz"), pybind11::arg("value"));
		cl.def("getCellFreeness", (float (mrpt::maps::COccupancyGridMap3D::*)(unsigned int, unsigned int, unsigned int) const) &mrpt::maps::COccupancyGridMap3D::getCellFreeness, "Read the real valued [0,1] (0:occupied, 1:free) contents of a voxel,\n given its index \n\nC++: mrpt::maps::COccupancyGridMap3D::getCellFreeness(unsigned int, unsigned int, unsigned int) const --> float", pybind11::arg("cx"), pybind11::arg("cy"), pybind11::arg("cz"));
		cl.def("setFreenessByPos", (void (mrpt::maps::COccupancyGridMap3D::*)(float, float, float, float)) &mrpt::maps::COccupancyGridMap3D::setFreenessByPos, "Change the contents [0,1] of a voxel, given its coordinates \n\nC++: mrpt::maps::COccupancyGridMap3D::setFreenessByPos(float, float, float, float) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("value"));
		cl.def("getFreenessByPos", (float (mrpt::maps::COccupancyGridMap3D::*)(float, float, float) const) &mrpt::maps::COccupancyGridMap3D::getFreenessByPos, "Read the real valued [0,1] contents of a voxel, given its coordinates \n\nC++: mrpt::maps::COccupancyGridMap3D::getFreenessByPos(float, float, float) const --> float", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("insertRay", [](mrpt::maps::COccupancyGridMap3D &o, const struct mrpt::math::TPoint3D_<double> & a0, const struct mrpt::math::TPoint3D_<double> & a1) -> void { return o.insertRay(a0, a1); }, "", pybind11::arg("sensor"), pybind11::arg("end"));
		cl.def("insertRay", (void (mrpt::maps::COccupancyGridMap3D::*)(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &, bool)) &mrpt::maps::COccupancyGridMap3D::insertRay, "Increases the freeness of a ray segment, and the occupancy of the voxel\n at its end point (unless endIsOccupied=false).\n Normally, users would prefer the higher-level method\n CMetricMap::insertObservation()\n\nC++: mrpt::maps::COccupancyGridMap3D::insertRay(const struct mrpt::math::TPoint3D_<double> &, const struct mrpt::math::TPoint3D_<double> &, bool) --> void", pybind11::arg("sensor"), pybind11::arg("end"), pybind11::arg("endIsOccupied"));
		cl.def("getAsOctoMapVoxels", (void (mrpt::maps::COccupancyGridMap3D::*)(class mrpt::opengl::COctoMapVoxels &) const) &mrpt::maps::COccupancyGridMap3D::getAsOctoMapVoxels, "renderingOptions \n\nC++: mrpt::maps::COccupancyGridMap3D::getAsOctoMapVoxels(class mrpt::opengl::COctoMapVoxels &) const --> void", pybind11::arg("gl_obj"));
		cl.def("getVisualizationInto", (void (mrpt::maps::COccupancyGridMap3D::*)(class mrpt::opengl::CSetOfObjects &) const) &mrpt::maps::COccupancyGridMap3D::getVisualizationInto, "Returns a 3D object representing the map. \n renderingOptions \n\nC++: mrpt::maps::COccupancyGridMap3D::getVisualizationInto(class mrpt::opengl::CSetOfObjects &) const --> void", pybind11::arg("outObj"));
		cl.def("boundingBox", (struct mrpt::math::TBoundingBox_<float> (mrpt::maps::COccupancyGridMap3D::*)() const) &mrpt::maps::COccupancyGridMap3D::boundingBox, "C++: mrpt::maps::COccupancyGridMap3D::boundingBox() const --> struct mrpt::math::TBoundingBox_<float>");
		cl.def("isEmpty", (bool (mrpt::maps::COccupancyGridMap3D::*)() const) &mrpt::maps::COccupancyGridMap3D::isEmpty, "Returns true upon map construction or after calling clear(), the return\n  changes to false upon successful insertObservation() or any other\n method to load data in the map.\n\nC++: mrpt::maps::COccupancyGridMap3D::isEmpty() const --> bool");
		cl.def("determineMatching2D", (void (mrpt::maps::COccupancyGridMap3D::*)(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose2D &, class mrpt::tfest::TMatchingPairListTempl<float> &, const struct mrpt::maps::TMatchingParams &, struct mrpt::maps::TMatchingExtraResults &) const) &mrpt::maps::COccupancyGridMap3D::determineMatching2D, "See docs in base class: in this class this always returns 0 \n\nC++: mrpt::maps::COccupancyGridMap3D::determineMatching2D(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose2D &, class mrpt::tfest::TMatchingPairListTempl<float> &, const struct mrpt::maps::TMatchingParams &, struct mrpt::maps::TMatchingExtraResults &) const --> void", pybind11::arg("otherMap"), pybind11::arg("otherMapPose"), pybind11::arg("correspondences"), pybind11::arg("params"), pybind11::arg("extraResults"));
		cl.def("compute3DMatchingRatio", (float (mrpt::maps::COccupancyGridMap3D::*)(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose3D &, const struct mrpt::maps::TMatchingRatioParams &) const) &mrpt::maps::COccupancyGridMap3D::compute3DMatchingRatio, "See docs in base class: in this class this always returns 0 \n\nC++: mrpt::maps::COccupancyGridMap3D::compute3DMatchingRatio(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose3D &, const struct mrpt::maps::TMatchingRatioParams &) const --> float", pybind11::arg("otherMap"), pybind11::arg("otherMapPose"), pybind11::arg("params"));
		cl.def("saveMetricMapRepresentationToFile", (void (mrpt::maps::COccupancyGridMap3D::*)(const std::string &) const) &mrpt::maps::COccupancyGridMap3D::saveMetricMapRepresentationToFile, "C++: mrpt::maps::COccupancyGridMap3D::saveMetricMapRepresentationToFile(const std::string &) const --> void", pybind11::arg("f"));
		cl.def("asString", (std::string (mrpt::maps::COccupancyGridMap3D::*)() const) &mrpt::maps::COccupancyGridMap3D::asString, "Returns a short description of the map. \n\nC++: mrpt::maps::COccupancyGridMap3D::asString() const --> std::string");
		cl.def("nn_has_indices_or_ids", (bool (mrpt::maps::COccupancyGridMap3D::*)() const) &mrpt::maps::COccupancyGridMap3D::nn_has_indices_or_ids, "@{ \n\nC++: mrpt::maps::COccupancyGridMap3D::nn_has_indices_or_ids() const --> bool");
		cl.def("nn_index_count", (size_t (mrpt::maps::COccupancyGridMap3D::*)() const) &mrpt::maps::COccupancyGridMap3D::nn_index_count, "C++: mrpt::maps::COccupancyGridMap3D::nn_index_count() const --> size_t");
		cl.def("nn_single_search", (bool (mrpt::maps::COccupancyGridMap3D::*)(const struct mrpt::math::TPoint3D_<float> &, struct mrpt::math::TPoint3D_<float> &, float &, uint64_t &) const) &mrpt::maps::COccupancyGridMap3D::nn_single_search, "C++: mrpt::maps::COccupancyGridMap3D::nn_single_search(const struct mrpt::math::TPoint3D_<float> &, struct mrpt::math::TPoint3D_<float> &, float &, uint64_t &) const --> bool", pybind11::arg("query"), pybind11::arg("result"), pybind11::arg("out_dist_sqr"), pybind11::arg("resultIndexOrID"));
		cl.def("nn_single_search", (bool (mrpt::maps::COccupancyGridMap3D::*)(const struct mrpt::math::TPoint2D_<float> &, struct mrpt::math::TPoint2D_<float> &, float &, uint64_t &) const) &mrpt::maps::COccupancyGridMap3D::nn_single_search, "C++: mrpt::maps::COccupancyGridMap3D::nn_single_search(const struct mrpt::math::TPoint2D_<float> &, struct mrpt::math::TPoint2D_<float> &, float &, uint64_t &) const --> bool", pybind11::arg("query"), pybind11::arg("result"), pybind11::arg("out_dist_sqr"), pybind11::arg("resultIndexOrID"));
		cl.def("assign", (class mrpt::maps::COccupancyGridMap3D & (mrpt::maps::COccupancyGridMap3D::*)(const class mrpt::maps::COccupancyGridMap3D &)) &mrpt::maps::COccupancyGridMap3D::operator=, "C++: mrpt::maps::COccupancyGridMap3D::operator=(const class mrpt::maps::COccupancyGridMap3D &) --> class mrpt::maps::COccupancyGridMap3D &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::maps::COccupancyGridMap3D::TInsertionOptions file:mrpt/maps/COccupancyGridMap3D.h line:202
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::COccupancyGridMap3D::TInsertionOptions, std::shared_ptr<mrpt::maps::COccupancyGridMap3D::TInsertionOptions>, PyCallBack_mrpt_maps_COccupancyGridMap3D_TInsertionOptions, mrpt::config::CLoadableOptions> cl(enclosing_class, "TInsertionOptions", "With this struct options are provided to the observation insertion\n process.\n \n\n CObservation::insertIntoGridMap ");
			cl.def( pybind11::init( [](){ return new mrpt::maps::COccupancyGridMap3D::TInsertionOptions(); }, [](){ return new PyCallBack_mrpt_maps_COccupancyGridMap3D_TInsertionOptions(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_maps_COccupancyGridMap3D_TInsertionOptions const &o){ return new PyCallBack_mrpt_maps_COccupancyGridMap3D_TInsertionOptions(o); } ) );
			cl.def( pybind11::init( [](mrpt::maps::COccupancyGridMap3D::TInsertionOptions const &o){ return new mrpt::maps::COccupancyGridMap3D::TInsertionOptions(o); } ) );
			cl.def_readwrite("maxDistanceInsertion", &mrpt::maps::COccupancyGridMap3D::TInsertionOptions::maxDistanceInsertion);
			cl.def_readwrite("maxOccupancyUpdateCertainty", &mrpt::maps::COccupancyGridMap3D::TInsertionOptions::maxOccupancyUpdateCertainty);
			cl.def_readwrite("maxFreenessUpdateCertainty", &mrpt::maps::COccupancyGridMap3D::TInsertionOptions::maxFreenessUpdateCertainty);
			cl.def_readwrite("decimation_3d_range", &mrpt::maps::COccupancyGridMap3D::TInsertionOptions::decimation_3d_range);
			cl.def_readwrite("decimation", &mrpt::maps::COccupancyGridMap3D::TInsertionOptions::decimation);
			cl.def_readwrite("raytraceEmptyCells", &mrpt::maps::COccupancyGridMap3D::TInsertionOptions::raytraceEmptyCells);
			cl.def("loadFromConfigFile", (void (mrpt::maps::COccupancyGridMap3D::TInsertionOptions::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::maps::COccupancyGridMap3D::TInsertionOptions::loadFromConfigFile, "This method load the options from a \".ini\" file.\n   Only those parameters found in the given \"section\" and having\n   the same name that the variable are loaded. Those not found in\n   the file will stay with their previous values (usually the default\n   values loaded at initialization). An example of an \".ini\" file:\n  \n\n\n\n\n\n     \n\nC++: mrpt::maps::COccupancyGridMap3D::TInsertionOptions::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
			cl.def("saveToConfigFile", (void (mrpt::maps::COccupancyGridMap3D::TInsertionOptions::*)(class mrpt::config::CConfigFileBase &, const std::string &) const) &mrpt::maps::COccupancyGridMap3D::TInsertionOptions::saveToConfigFile, "C++: mrpt::maps::COccupancyGridMap3D::TInsertionOptions::saveToConfigFile(class mrpt::config::CConfigFileBase &, const std::string &) const --> void", pybind11::arg("target"), pybind11::arg("section"));
			cl.def("assign", (class mrpt::maps::COccupancyGridMap3D::TInsertionOptions & (mrpt::maps::COccupancyGridMap3D::TInsertionOptions::*)(const class mrpt::maps::COccupancyGridMap3D::TInsertionOptions &)) &mrpt::maps::COccupancyGridMap3D::TInsertionOptions::operator=, "C++: mrpt::maps::COccupancyGridMap3D::TInsertionOptions::operator=(const class mrpt::maps::COccupancyGridMap3D::TInsertionOptions &) --> class mrpt::maps::COccupancyGridMap3D::TInsertionOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::maps::COccupancyGridMap3D::TLikelihoodOptions file:mrpt/maps/COccupancyGridMap3D.h line:262
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::COccupancyGridMap3D::TLikelihoodOptions, std::shared_ptr<mrpt::maps::COccupancyGridMap3D::TLikelihoodOptions>, PyCallBack_mrpt_maps_COccupancyGridMap3D_TLikelihoodOptions, mrpt::config::CLoadableOptions> cl(enclosing_class, "TLikelihoodOptions", "With this struct options are provided to the observation likelihood\n computation process ");
			cl.def( pybind11::init( [](){ return new mrpt::maps::COccupancyGridMap3D::TLikelihoodOptions(); }, [](){ return new PyCallBack_mrpt_maps_COccupancyGridMap3D_TLikelihoodOptions(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_maps_COccupancyGridMap3D_TLikelihoodOptions const &o){ return new PyCallBack_mrpt_maps_COccupancyGridMap3D_TLikelihoodOptions(o); } ) );
			cl.def( pybind11::init( [](mrpt::maps::COccupancyGridMap3D::TLikelihoodOptions const &o){ return new mrpt::maps::COccupancyGridMap3D::TLikelihoodOptions(o); } ) );
			cl.def_readwrite("likelihoodMethod", &mrpt::maps::COccupancyGridMap3D::TLikelihoodOptions::likelihoodMethod);
			cl.def_readwrite("LF_stdHit", &mrpt::maps::COccupancyGridMap3D::TLikelihoodOptions::LF_stdHit);
			cl.def_readwrite("LF_zHit", &mrpt::maps::COccupancyGridMap3D::TLikelihoodOptions::LF_zHit);
			cl.def_readwrite("LF_zRandom", &mrpt::maps::COccupancyGridMap3D::TLikelihoodOptions::LF_zRandom);
			cl.def_readwrite("LF_maxRange", &mrpt::maps::COccupancyGridMap3D::TLikelihoodOptions::LF_maxRange);
			cl.def_readwrite("LF_decimation", &mrpt::maps::COccupancyGridMap3D::TLikelihoodOptions::LF_decimation);
			cl.def_readwrite("LF_maxCorrsDistance", &mrpt::maps::COccupancyGridMap3D::TLikelihoodOptions::LF_maxCorrsDistance);
			cl.def_readwrite("LF_useSquareDist", &mrpt::maps::COccupancyGridMap3D::TLikelihoodOptions::LF_useSquareDist);
			cl.def_readwrite("rayTracing_decimation", &mrpt::maps::COccupancyGridMap3D::TLikelihoodOptions::rayTracing_decimation);
			cl.def_readwrite("rayTracing_stdHit", &mrpt::maps::COccupancyGridMap3D::TLikelihoodOptions::rayTracing_stdHit);
			cl.def("loadFromConfigFile", (void (mrpt::maps::COccupancyGridMap3D::TLikelihoodOptions::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::maps::COccupancyGridMap3D::TLikelihoodOptions::loadFromConfigFile, "This method load the options from a \".ini\" file.\n   Only those parameters found in the given \"section\" and having\n   the same name that the variable are loaded. Those not found in\n   the file will stay with their previous values (usually the default\n   values loaded at initialization). An example of an \".ini\" file:\n  \n\n\n\n\n\n     \n\nC++: mrpt::maps::COccupancyGridMap3D::TLikelihoodOptions::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
			cl.def("saveToConfigFile", (void (mrpt::maps::COccupancyGridMap3D::TLikelihoodOptions::*)(class mrpt::config::CConfigFileBase &, const std::string &) const) &mrpt::maps::COccupancyGridMap3D::TLikelihoodOptions::saveToConfigFile, "C++: mrpt::maps::COccupancyGridMap3D::TLikelihoodOptions::saveToConfigFile(class mrpt::config::CConfigFileBase &, const std::string &) const --> void", pybind11::arg("c"), pybind11::arg("s"));
			cl.def("assign", (class mrpt::maps::COccupancyGridMap3D::TLikelihoodOptions & (mrpt::maps::COccupancyGridMap3D::TLikelihoodOptions::*)(const class mrpt::maps::COccupancyGridMap3D::TLikelihoodOptions &)) &mrpt::maps::COccupancyGridMap3D::TLikelihoodOptions::operator=, "C++: mrpt::maps::COccupancyGridMap3D::TLikelihoodOptions::operator=(const class mrpt::maps::COccupancyGridMap3D::TLikelihoodOptions &) --> class mrpt::maps::COccupancyGridMap3D::TLikelihoodOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::maps::COccupancyGridMap3D::TRenderingOptions file:mrpt/maps/COccupancyGridMap3D.h line:313
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::COccupancyGridMap3D::TRenderingOptions, std::shared_ptr<mrpt::maps::COccupancyGridMap3D::TRenderingOptions>> cl(enclosing_class, "TRenderingOptions", "Options for the conversion of a mrpt::maps::COccupancyGridMap3D into a\n mrpt::opengl::COctoMapVoxels ");
			cl.def( pybind11::init( [](){ return new mrpt::maps::COccupancyGridMap3D::TRenderingOptions(); } ) );
			cl.def( pybind11::init( [](mrpt::maps::COccupancyGridMap3D::TRenderingOptions const &o){ return new mrpt::maps::COccupancyGridMap3D::TRenderingOptions(o); } ) );
			cl.def_readwrite("generateGridLines", &mrpt::maps::COccupancyGridMap3D::TRenderingOptions::generateGridLines);
			cl.def_readwrite("generateOccupiedVoxels", &mrpt::maps::COccupancyGridMap3D::TRenderingOptions::generateOccupiedVoxels);
			cl.def_readwrite("visibleOccupiedVoxels", &mrpt::maps::COccupancyGridMap3D::TRenderingOptions::visibleOccupiedVoxels);
			cl.def_readwrite("generateFreeVoxels", &mrpt::maps::COccupancyGridMap3D::TRenderingOptions::generateFreeVoxels);
			cl.def_readwrite("visibleFreeVoxels", &mrpt::maps::COccupancyGridMap3D::TRenderingOptions::visibleFreeVoxels);
			cl.def("writeToStream", (void (mrpt::maps::COccupancyGridMap3D::TRenderingOptions::*)(class mrpt::serialization::CArchive &) const) &mrpt::maps::COccupancyGridMap3D::TRenderingOptions::writeToStream, "Binary dump to stream \n\nC++: mrpt::maps::COccupancyGridMap3D::TRenderingOptions::writeToStream(class mrpt::serialization::CArchive &) const --> void", pybind11::arg("out"));
			cl.def("readFromStream", (void (mrpt::maps::COccupancyGridMap3D::TRenderingOptions::*)(class mrpt::serialization::CArchive &)) &mrpt::maps::COccupancyGridMap3D::TRenderingOptions::readFromStream, "Binary dump to stream \n\nC++: mrpt::maps::COccupancyGridMap3D::TRenderingOptions::readFromStream(class mrpt::serialization::CArchive &) --> void", pybind11::arg("in"));
			cl.def("assign", (struct mrpt::maps::COccupancyGridMap3D::TRenderingOptions & (mrpt::maps::COccupancyGridMap3D::TRenderingOptions::*)(const struct mrpt::maps::COccupancyGridMap3D::TRenderingOptions &)) &mrpt::maps::COccupancyGridMap3D::TRenderingOptions::operator=, "C++: mrpt::maps::COccupancyGridMap3D::TRenderingOptions::operator=(const struct mrpt::maps::COccupancyGridMap3D::TRenderingOptions &) --> struct mrpt::maps::COccupancyGridMap3D::TRenderingOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::maps::COccupancyGridMap3D::TMapDefinitionBase file: line:74
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::COccupancyGridMap3D::TMapDefinitionBase, std::shared_ptr<mrpt::maps::COccupancyGridMap3D::TMapDefinitionBase>> cl(enclosing_class, "TMapDefinitionBase", "");
		}

		{ // mrpt::maps::COccupancyGridMap3D::TMapDefinition file: line:78
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::COccupancyGridMap3D::TMapDefinition, std::shared_ptr<mrpt::maps::COccupancyGridMap3D::TMapDefinition>, PyCallBack_mrpt_maps_COccupancyGridMap3D_TMapDefinition, mrpt::maps::COccupancyGridMap3D::TMapDefinitionBase> cl(enclosing_class, "TMapDefinition", "");
			cl.def( pybind11::init( [](){ return new mrpt::maps::COccupancyGridMap3D::TMapDefinition(); }, [](){ return new PyCallBack_mrpt_maps_COccupancyGridMap3D_TMapDefinition(); } ) );
			cl.def_readwrite("min_x", &mrpt::maps::COccupancyGridMap3D::TMapDefinition::min_x);
			cl.def_readwrite("max_x", &mrpt::maps::COccupancyGridMap3D::TMapDefinition::max_x);
			cl.def_readwrite("min_y", &mrpt::maps::COccupancyGridMap3D::TMapDefinition::min_y);
			cl.def_readwrite("max_y", &mrpt::maps::COccupancyGridMap3D::TMapDefinition::max_y);
			cl.def_readwrite("min_z", &mrpt::maps::COccupancyGridMap3D::TMapDefinition::min_z);
			cl.def_readwrite("max_z", &mrpt::maps::COccupancyGridMap3D::TMapDefinition::max_z);
			cl.def_readwrite("resolution", &mrpt::maps::COccupancyGridMap3D::TMapDefinition::resolution);
			cl.def_readwrite("insertionOpts", &mrpt::maps::COccupancyGridMap3D::TMapDefinition::insertionOpts);
			cl.def_readwrite("likelihoodOpts", &mrpt::maps::COccupancyGridMap3D::TMapDefinition::likelihoodOpts);
		}

	}
}
