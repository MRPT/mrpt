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
#include <mrpt/core/safe_pointers.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TColor.h>
#include <mrpt/img/TStereoCamera.h>
#include <mrpt/maps/CLandmark.h>
#include <mrpt/maps/CLandmarksMap.h>
#include <mrpt/maps/CMetricMap.h>
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
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/T2DScanProperties.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPointPDF.h>
#include <mrpt/poses/CPointPDFGaussian.h>
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
#include <mrpt/vision/CFeatureExtraction.h>
#include <mrpt/vision/types.h>
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

// mrpt::maps::CLandmarksMap file:mrpt/maps/CLandmarksMap.h line:74
struct PyCallBack_mrpt_maps_CLandmarksMap : public mrpt::maps::CLandmarksMap {
	using mrpt::maps::CLandmarksMap::CLandmarksMap;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CLandmarksMap *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CLandmarksMap::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CLandmarksMap *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CLandmarksMap::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CLandmarksMap *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CLandmarksMap::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CLandmarksMap *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CLandmarksMap::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CLandmarksMap *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CLandmarksMap::serializeFrom(a0, a1);
	}
	double internal_computeObservationLikelihood(const class mrpt::obs::CObservation & a0, const class mrpt::poses::CPose3D & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CLandmarksMap *>(this), "internal_computeObservationLikelihood");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CLandmarksMap::internal_computeObservationLikelihood(a0, a1);
	}
	std::string asString() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CLandmarksMap *>(this), "asString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CLandmarksMap::asString();
	}
	float compute3DMatchingRatio(const class mrpt::maps::CMetricMap * a0, const class mrpt::poses::CPose3D & a1, const struct mrpt::maps::TMatchingRatioParams & a2) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CLandmarksMap *>(this), "compute3DMatchingRatio");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<float>::value) {
				static pybind11::detail::override_caster_t<float> caster;
				return pybind11::detail::cast_ref<float>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<float>(std::move(o));
		}
		return CLandmarksMap::compute3DMatchingRatio(a0, a1, a2);
	}
	bool isEmpty() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CLandmarksMap *>(this), "isEmpty");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CLandmarksMap::isEmpty();
	}
	void saveMetricMapRepresentationToFile(const std::string & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CLandmarksMap *>(this), "saveMetricMapRepresentationToFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CLandmarksMap::saveMetricMapRepresentationToFile(a0);
	}
	void getVisualizationInto(class mrpt::opengl::CSetOfObjects & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CLandmarksMap *>(this), "getVisualizationInto");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CLandmarksMap::getVisualizationInto(a0);
	}
	void auxParticleFilterCleanUp() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CLandmarksMap *>(this), "auxParticleFilterCleanUp");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CLandmarksMap::auxParticleFilterCleanUp();
	}
	struct mrpt::math::TBoundingBox_<float> boundingBox() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CLandmarksMap *>(this), "boundingBox");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CLandmarksMap *>(this), "canComputeObservationLikelihood");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CLandmarksMap *>(this), "determineMatching2D");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CLandmarksMap *>(this), "determineMatching3D");
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
	float squareDistanceToClosestCorrespondence(float a0, float a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CLandmarksMap *>(this), "squareDistanceToClosestCorrespondence");
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

// mrpt::maps::CLandmarksMap::TInsertionOptions file:mrpt/maps/CLandmarksMap.h line:227
struct PyCallBack_mrpt_maps_CLandmarksMap_TInsertionOptions : public mrpt::maps::CLandmarksMap::TInsertionOptions {
	using mrpt::maps::CLandmarksMap::TInsertionOptions::TInsertionOptions;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CLandmarksMap::TInsertionOptions *>(this), "loadFromConfigFile");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CLandmarksMap::TInsertionOptions *>(this), "saveToConfigFile");
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

// mrpt::maps::CLandmarksMap::TLikelihoodOptions file:mrpt/maps/CLandmarksMap.h line:341
struct PyCallBack_mrpt_maps_CLandmarksMap_TLikelihoodOptions : public mrpt::maps::CLandmarksMap::TLikelihoodOptions {
	using mrpt::maps::CLandmarksMap::TLikelihoodOptions::TLikelihoodOptions;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CLandmarksMap::TLikelihoodOptions *>(this), "loadFromConfigFile");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CLandmarksMap::TLikelihoodOptions *>(this), "saveToConfigFile");
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

// mrpt::maps::CLandmarksMap::TMapDefinition file: line:78
struct PyCallBack_mrpt_maps_CLandmarksMap_TMapDefinition : public mrpt::maps::CLandmarksMap::TMapDefinition {
	using mrpt::maps::CLandmarksMap::TMapDefinition::TMapDefinition;

	void loadFromConfigFile_map_specific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CLandmarksMap::TMapDefinition *>(this), "loadFromConfigFile_map_specific");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CLandmarksMap::TMapDefinition *>(this), "loadFromConfigFile");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CLandmarksMap::TMapDefinition *>(this), "saveToConfigFile");
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

void bind_mrpt_maps_CLandmarksMap(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::maps::CLandmarksMap file:mrpt/maps/CLandmarksMap.h line:74
		pybind11::class_<mrpt::maps::CLandmarksMap, std::shared_ptr<mrpt::maps::CLandmarksMap>, PyCallBack_mrpt_maps_CLandmarksMap, mrpt::maps::CMetricMap> cl(M("mrpt::maps"), "CLandmarksMap", "A class for storing a map of 3D probabilistic landmarks.\n \n  Currently these types of landmarks are defined: (see mrpt::maps::CLandmark)\n		- For \"visual landmarks\" from images: features with associated\n descriptors.\n		- For laser scanners: each of the range measuremnts, as \"occupancy\"\n landmarks.\n		- For grid maps: \"Panoramic descriptor\" feature points.\n		- For range-only localization and SLAM: Beacons. It is also supported\n the simulation of expected beacon-to-sensor readings, observation\n likelihood,...\n \n How to load landmarks from observations:\n  When invoking CLandmarksMap::insertObservation(), the values in\n CLandmarksMap::insertionOptions will\n     determinate the kind of landmarks that will be extracted and fused into\n the map. Supported feature\n     extraction processes are listed next:\n\n  \n   Observation class: Generated Landmarks:\n Comments: \n   CObservationImage vlSIFT 1) A SIFT feature is\n created for each SIFT detected in the image,\n       2) Correspondences between SIFTs features and existing ones are\n finded by computeMatchingWith3DLandmarks,\n       3) The corresponding feaures are fused, and the new ones added,\n with an initial uncertainty according to insertionOptions \n   CObservationStereoImages vlSIFT  Each image is\n separately procesed by the method for CObservationImage observations \n \n   CObservationStereoImages vlColor  TODO... \n \n   CObservation2DRangeScan glOccupancy  A landmark is\n added for each range in the scan, with its appropiate covariance matrix derived\n from the jacobians matrixes.  \n  \n\n \n CMetricMap\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::maps::CLandmarksMap(); }, [](){ return new PyCallBack_mrpt_maps_CLandmarksMap(); } ) );
		cl.def_readwrite("landmarks", &mrpt::maps::CLandmarksMap::landmarks);
		cl.def_readwrite("insertionOptions", &mrpt::maps::CLandmarksMap::insertionOptions);
		cl.def_readwrite("likelihoodOptions", &mrpt::maps::CLandmarksMap::likelihoodOptions);
		cl.def_readwrite("insertionResults", &mrpt::maps::CLandmarksMap::insertionResults);
		cl.def_readwrite("fuseOptions", &mrpt::maps::CLandmarksMap::fuseOptions);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::maps::CLandmarksMap::GetRuntimeClassIdStatic, "C++: mrpt::maps::CLandmarksMap::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::maps::CLandmarksMap::*)() const) &mrpt::maps::CLandmarksMap::GetRuntimeClass, "C++: mrpt::maps::CLandmarksMap::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::maps::CLandmarksMap::*)() const) &mrpt::maps::CLandmarksMap::clone, "C++: mrpt::maps::CLandmarksMap::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::maps::CLandmarksMap::CreateObject, "C++: mrpt::maps::CLandmarksMap::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("internal_computeObservationLikelihood", (double (mrpt::maps::CLandmarksMap::*)(const class mrpt::obs::CObservation &, const class mrpt::poses::CPose3D &) const) &mrpt::maps::CLandmarksMap::internal_computeObservationLikelihood, "Computes the (logarithmic) likelihood that a given observation was taken\nfrom a given pose in the world being modeled with this map.\n\n  In the current implementation, this method behaves in a different way\naccording to the nature of\n   the observation's class:\n		- \"mrpt::obs::CObservation2DRangeScan\": This calls\n\"computeLikelihood_RSLC_2007\".\n		- \"mrpt::obs::CObservationStereoImages\": This calls\n\"computeLikelihood_SIFT_LandmarkMap\".\n \n  Observation class: Generated\nLandmarks: Comments: \n  CObservationImage vlSIFT 1) A SIFT feature is\ncreated for each SIFT detected in the image,\n          2) Correspondences between SIFTs features and existing ones\nare found by computeMatchingWith3DLandmarks,\n		   3) The corresponding feaures are fused, and the new ones\n added,\nwith an initial uncertainty according to insertionOptions \n  CObservationStereoImages vlSIFT  Each image is\nseparately procesed by the method for CObservationImage observations \n\n  CObservationStereoImages vlColor  TODO...\n \n  CObservation2DRangeScan glOccupancy  A\nlandmark is added for each range in the scan, with its appropiate\ncovariance matrix derived from the jacobians matrixes.  \n \n\n \n The robot's pose the observation is supposed to be taken\nfrom.\n \n\n The observation.\n \n\n This method returns a likelihood value > 0.\n\n \n Used in particle filter algorithms, see: CMultiMetricMapPDF::update\n\nC++: mrpt::maps::CLandmarksMap::internal_computeObservationLikelihood(const class mrpt::obs::CObservation &, const class mrpt::poses::CPose3D &) const --> double", pybind11::arg("obs"), pybind11::arg("takenFrom"));
		cl.def("asString", (std::string (mrpt::maps::CLandmarksMap::*)() const) &mrpt::maps::CLandmarksMap::asString, "Returns a short description of the map. \n\nC++: mrpt::maps::CLandmarksMap::asString() const --> std::string");
		cl.def("getMapMaxID", (long (mrpt::maps::CLandmarksMap::*)()) &mrpt::maps::CLandmarksMap::getMapMaxID, "C++: mrpt::maps::CLandmarksMap::getMapMaxID() --> long");
		cl.def("compute3DMatchingRatio", (float (mrpt::maps::CLandmarksMap::*)(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose3D &, const struct mrpt::maps::TMatchingRatioParams &) const) &mrpt::maps::CLandmarksMap::compute3DMatchingRatio, "** END FAMD ****\n\nC++: mrpt::maps::CLandmarksMap::compute3DMatchingRatio(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose3D &, const struct mrpt::maps::TMatchingRatioParams &) const --> float", pybind11::arg("otherMap"), pybind11::arg("otherMapPose"), pybind11::arg("params"));
		cl.def("saveToTextFile", (bool (mrpt::maps::CLandmarksMap::*)(std::string)) &mrpt::maps::CLandmarksMap::saveToTextFile, "Save to a text file.\n  In line \"i\" there are the (x,y,z) mean values of the i'th landmark +\n type of landmark + # times seen + timestamp + RGB/descriptor + ID\n\n   Returns false if any error occured, true elsewere.\n\nC++: mrpt::maps::CLandmarksMap::saveToTextFile(std::string) --> bool", pybind11::arg("file"));
		cl.def("saveToMATLABScript2D", [](mrpt::maps::CLandmarksMap &o, std::string const & a0) -> bool { return o.saveToMATLABScript2D(a0); }, "", pybind11::arg("file"));
		cl.def("saveToMATLABScript2D", [](mrpt::maps::CLandmarksMap &o, std::string const & a0, const char * a1) -> bool { return o.saveToMATLABScript2D(a0, a1); }, "", pybind11::arg("file"), pybind11::arg("style"));
		cl.def("saveToMATLABScript2D", (bool (mrpt::maps::CLandmarksMap::*)(std::string, const char *, float)) &mrpt::maps::CLandmarksMap::saveToMATLABScript2D, "Save to a MATLAB script which displays 2D error ellipses for the map\n(top-view, projection on the XY plane).\n	\n\n		The name of the file to save the script to.\n  \n\n	The MATLAB-like string for the style of the lines (see\n'help plot' in MATLAB for possibilities)\n  \n\n The ellipsoids will be drawn from the center to\n\"stdCount\" times the \"standard deviations\". (default is 2std = 95%\nconfidence intervals)\n\n  \n Returns false if any error occured, true elsewere.\n\nC++: mrpt::maps::CLandmarksMap::saveToMATLABScript2D(std::string, const char *, float) --> bool", pybind11::arg("file"), pybind11::arg("style"), pybind11::arg("stdCount"));
		cl.def("saveToMATLABScript3D", [](mrpt::maps::CLandmarksMap const &o, std::string const & a0) -> bool { return o.saveToMATLABScript3D(a0); }, "", pybind11::arg("file"));
		cl.def("saveToMATLABScript3D", [](mrpt::maps::CLandmarksMap const &o, std::string const & a0, const char * a1) -> bool { return o.saveToMATLABScript3D(a0, a1); }, "", pybind11::arg("file"), pybind11::arg("style"));
		cl.def("saveToMATLABScript3D", (bool (mrpt::maps::CLandmarksMap::*)(std::string, const char *, float) const) &mrpt::maps::CLandmarksMap::saveToMATLABScript3D, "Save to a MATLAB script which displays 3D error ellipses for the map.\n	\n\n		The name of the file to save the script to.\n  \n\n	The MATLAB-like string for the style of the lines (see\n'help plot' in MATLAB for possibilities)\n  \n\n The ellipsoids will be drawn from the center to a given\nconfidence interval in [0,1], e.g. 2 sigmas=0.95 (default is 2std = 0.95\nconfidence intervals)\n\n  \n Returns false if any error occured, true elsewere.\n\nC++: mrpt::maps::CLandmarksMap::saveToMATLABScript3D(std::string, const char *, float) const --> bool", pybind11::arg("file"), pybind11::arg("style"), pybind11::arg("confInterval"));
		cl.def("size", (size_t (mrpt::maps::CLandmarksMap::*)() const) &mrpt::maps::CLandmarksMap::size, "Returns the stored landmarks count.\n\nC++: mrpt::maps::CLandmarksMap::size() const --> size_t");
		cl.def("computeLikelihood_RSLC_2007", (double (mrpt::maps::CLandmarksMap::*)(const class mrpt::maps::CLandmarksMap *, const class mrpt::poses::CPose2D &) const) &mrpt::maps::CLandmarksMap::computeLikelihood_RSLC_2007, "Computes the (logarithmic) likelihood function for a sensed observation\n  \"o\" according to \"this\" map.\n   This is the implementation of the algorithm reported in the paper:\n      J.L. Blanco, J. Gonzalez, and J.A. Fernandez-Madrigal, \"A\n  Consensus-based Approach for Estimating the Observation Likelihood of\n  Accurate Range Sensors\", in IEEE International Conference on Robotics and\n  Automation (ICRA), Rome (Italy), Apr 10-14, 2007\n\nC++: mrpt::maps::CLandmarksMap::computeLikelihood_RSLC_2007(const class mrpt::maps::CLandmarksMap *, const class mrpt::poses::CPose2D &) const --> double", pybind11::arg("s"), pybind11::arg("sensorPose"));
		cl.def("loadSiftFeaturesFromImageObservation", [](mrpt::maps::CLandmarksMap &o, const class mrpt::obs::CObservationImage & a0) -> void { return o.loadSiftFeaturesFromImageObservation(a0); }, "", pybind11::arg("obs"));
		cl.def("loadSiftFeaturesFromImageObservation", (void (mrpt::maps::CLandmarksMap::*)(const class mrpt::obs::CObservationImage &, const struct mrpt::vision::CFeatureExtraction::TOptions &)) &mrpt::maps::CLandmarksMap::loadSiftFeaturesFromImageObservation, "Loads into this landmarks map the SIFT features extracted from an image\n observation (Previous contents of map will be erased)\n  The robot is assumed to be at the origin of the map.\n  Some options may be applicable from \"insertionOptions\"\n (insertionOptions.SIFTsLoadDistanceOfTheMean)\n\n  \n Optionally, you can pass here parameters for\n changing the default SIFT detector settings.\n\nC++: mrpt::maps::CLandmarksMap::loadSiftFeaturesFromImageObservation(const class mrpt::obs::CObservationImage &, const struct mrpt::vision::CFeatureExtraction::TOptions &) --> void", pybind11::arg("obs"), pybind11::arg("feat_options"));
		cl.def("loadSiftFeaturesFromStereoImageObservation", [](mrpt::maps::CLandmarksMap &o, const class mrpt::obs::CObservationStereoImages & a0, long const & a1) -> void { return o.loadSiftFeaturesFromStereoImageObservation(a0, a1); }, "", pybind11::arg("obs"), pybind11::arg("fID"));
		cl.def("loadSiftFeaturesFromStereoImageObservation", (void (mrpt::maps::CLandmarksMap::*)(const class mrpt::obs::CObservationStereoImages &, long, const struct mrpt::vision::CFeatureExtraction::TOptions &)) &mrpt::maps::CLandmarksMap::loadSiftFeaturesFromStereoImageObservation, "Loads into this landmarks map the SIFT features extracted from an\n observation consisting of a pair of stereo-image (Previous contents of\n map will be erased)\n  The robot is assumed to be at the origin of the map.\n  Some options may be applicable from \"insertionOptions\"\n\n  \n Optionally, you can pass here parameters for\n changing the default SIFT detector settings.\n\nC++: mrpt::maps::CLandmarksMap::loadSiftFeaturesFromStereoImageObservation(const class mrpt::obs::CObservationStereoImages &, long, const struct mrpt::vision::CFeatureExtraction::TOptions &) --> void", pybind11::arg("obs"), pybind11::arg("fID"), pybind11::arg("feat_options"));
		cl.def("computeMatchingWith2D", [](mrpt::maps::CLandmarksMap const &o, const class mrpt::maps::CMetricMap * a0, const class mrpt::poses::CPose2D & a1, float const & a2, float const & a3, const class mrpt::poses::CPose2D & a4, class mrpt::tfest::TMatchingPairListTempl<float> & a5, float & a6) -> void { return o.computeMatchingWith2D(a0, a1, a2, a3, a4, a5, a6); }, "", pybind11::arg("otherMap"), pybind11::arg("otherMapPose"), pybind11::arg("maxDistForCorrespondence"), pybind11::arg("maxAngularDistForCorrespondence"), pybind11::arg("angularDistPivotPoint"), pybind11::arg("correspondences"), pybind11::arg("correspondencesRatio"));
		cl.def("computeMatchingWith2D", [](mrpt::maps::CLandmarksMap const &o, const class mrpt::maps::CMetricMap * a0, const class mrpt::poses::CPose2D & a1, float const & a2, float const & a3, const class mrpt::poses::CPose2D & a4, class mrpt::tfest::TMatchingPairListTempl<float> & a5, float & a6, float * a7) -> void { return o.computeMatchingWith2D(a0, a1, a2, a3, a4, a5, a6, a7); }, "", pybind11::arg("otherMap"), pybind11::arg("otherMapPose"), pybind11::arg("maxDistForCorrespondence"), pybind11::arg("maxAngularDistForCorrespondence"), pybind11::arg("angularDistPivotPoint"), pybind11::arg("correspondences"), pybind11::arg("correspondencesRatio"), pybind11::arg("sumSqrDist"));
		cl.def("computeMatchingWith2D", [](mrpt::maps::CLandmarksMap const &o, const class mrpt::maps::CMetricMap * a0, const class mrpt::poses::CPose2D & a1, float const & a2, float const & a3, const class mrpt::poses::CPose2D & a4, class mrpt::tfest::TMatchingPairListTempl<float> & a5, float & a6, float * a7, bool const & a8) -> void { return o.computeMatchingWith2D(a0, a1, a2, a3, a4, a5, a6, a7, a8); }, "", pybind11::arg("otherMap"), pybind11::arg("otherMapPose"), pybind11::arg("maxDistForCorrespondence"), pybind11::arg("maxAngularDistForCorrespondence"), pybind11::arg("angularDistPivotPoint"), pybind11::arg("correspondences"), pybind11::arg("correspondencesRatio"), pybind11::arg("sumSqrDist"), pybind11::arg("onlyKeepTheClosest"));
		cl.def("computeMatchingWith2D", (void (mrpt::maps::CLandmarksMap::*)(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose2D &, float, float, const class mrpt::poses::CPose2D &, class mrpt::tfest::TMatchingPairListTempl<float> &, float &, float *, bool, bool) const) &mrpt::maps::CLandmarksMap::computeMatchingWith2D, "C++: mrpt::maps::CLandmarksMap::computeMatchingWith2D(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose2D &, float, float, const class mrpt::poses::CPose2D &, class mrpt::tfest::TMatchingPairListTempl<float> &, float &, float *, bool, bool) const --> void", pybind11::arg("otherMap"), pybind11::arg("otherMapPose"), pybind11::arg("maxDistForCorrespondence"), pybind11::arg("maxAngularDistForCorrespondence"), pybind11::arg("angularDistPivotPoint"), pybind11::arg("correspondences"), pybind11::arg("correspondencesRatio"), pybind11::arg("sumSqrDist"), pybind11::arg("onlyKeepTheClosest"), pybind11::arg("onlyUniqueRobust"));
		cl.def("changeCoordinatesReference", (void (mrpt::maps::CLandmarksMap::*)(const class mrpt::poses::CPose3D &)) &mrpt::maps::CLandmarksMap::changeCoordinatesReference, "Changes the reference system of the map to a given 3D pose.\n\nC++: mrpt::maps::CLandmarksMap::changeCoordinatesReference(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newOrg"));
		cl.def("changeCoordinatesReference", (void (mrpt::maps::CLandmarksMap::*)(const class mrpt::poses::CPose3D &, const class mrpt::maps::CLandmarksMap *)) &mrpt::maps::CLandmarksMap::changeCoordinatesReference, "Changes the reference system of the map \"otherMap\" and save the result\n in \"this\" map.\n\nC++: mrpt::maps::CLandmarksMap::changeCoordinatesReference(const class mrpt::poses::CPose3D &, const class mrpt::maps::CLandmarksMap *) --> void", pybind11::arg("newOrg"), pybind11::arg("otherMap"));
		cl.def("fuseWith", [](mrpt::maps::CLandmarksMap &o, class mrpt::maps::CLandmarksMap & a0) -> void { return o.fuseWith(a0); }, "", pybind11::arg("other"));
		cl.def("fuseWith", (void (mrpt::maps::CLandmarksMap::*)(class mrpt::maps::CLandmarksMap &, bool)) &mrpt::maps::CLandmarksMap::fuseWith, "Fuses the contents of another map with this one, updating \"this\" object\n with the result.\n  This process involves fusing corresponding landmarks, then adding the\n new ones.\n  \n\n The other landmarkmap, whose landmarks will be inserted\n into \"this\"\n  \n\n If set to \"true\", all the landmarks in\n \"other\" will be inserted into \"this\" without checking for possible\n correspondences (may appear duplicates ones, etc...)\n\nC++: mrpt::maps::CLandmarksMap::fuseWith(class mrpt::maps::CLandmarksMap &, bool) --> void", pybind11::arg("other"), pybind11::arg("justInsertAllOfThem"));
		cl.def("isEmpty", (bool (mrpt::maps::CLandmarksMap::*)() const) &mrpt::maps::CLandmarksMap::isEmpty, "Returns true if the map is empty/no observation has been inserted.\n\nC++: mrpt::maps::CLandmarksMap::isEmpty() const --> bool");
		cl.def("simulateBeaconReadings", (void (mrpt::maps::CLandmarksMap::*)(const class mrpt::poses::CPose3D &, const class mrpt::poses::CPoint3D &, class mrpt::obs::CObservationBeaconRanges &) const) &mrpt::maps::CLandmarksMap::simulateBeaconReadings, "Simulates a noisy reading toward each of the beacons in the landmarks\n map, if any.\n \n\n This robot pose is used to simulate the ranges to\n each beacon.\n \n\n The 3D position of the sensor on the\n robot\n \n\n The results will be stored here. NOTICE that the\n fields\n \"CObservationBeaconRanges::minSensorDistance\",\"CObservationBeaconRanges::maxSensorDistance\"\n and \"CObservationBeaconRanges::stdError\" MUST BE FILLED OUT before\n calling this function.\n An observation will be generated for each beacon in the map, but notice\n that some of them may be missed if out of the sensor maximum range.\n\nC++: mrpt::maps::CLandmarksMap::simulateBeaconReadings(const class mrpt::poses::CPose3D &, const class mrpt::poses::CPoint3D &, class mrpt::obs::CObservationBeaconRanges &) const --> void", pybind11::arg("in_robotPose"), pybind11::arg("in_sensorLocationOnRobot"), pybind11::arg("out_Observations"));
		cl.def("saveMetricMapRepresentationToFile", (void (mrpt::maps::CLandmarksMap::*)(const std::string &) const) &mrpt::maps::CLandmarksMap::saveMetricMapRepresentationToFile, "This virtual method saves the map to a file \"filNamePrefix\"+<\nsome_file_extension >, as an image or in any other applicable way (Notice\nthat other methods to save the map may be implemented in classes\nimplementing this virtual interface).\n  In the case of this class, these files are generated:\n		- \"filNamePrefix\"+\"_3D.m\": A script for MATLAB for drawing landmarks\nas\n3D ellipses.\n		- \"filNamePrefix\"+\"_3D.3DScene\": A 3D scene with a \"ground plane\ngrid\"\nand the set of ellipsoids in 3D.\n\nC++: mrpt::maps::CLandmarksMap::saveMetricMapRepresentationToFile(const std::string &) const --> void", pybind11::arg("filNamePrefix"));
		cl.def("getVisualizationInto", (void (mrpt::maps::CLandmarksMap::*)(class mrpt::opengl::CSetOfObjects &) const) &mrpt::maps::CLandmarksMap::getVisualizationInto, "Returns a 3D object representing the map.\n \n\n COLOR_LANDMARKS_IN_3DSCENES\n\nC++: mrpt::maps::CLandmarksMap::getVisualizationInto(class mrpt::opengl::CSetOfObjects &) const --> void", pybind11::arg("outObj"));
		cl.def("auxParticleFilterCleanUp", (void (mrpt::maps::CLandmarksMap::*)()) &mrpt::maps::CLandmarksMap::auxParticleFilterCleanUp, "C++: mrpt::maps::CLandmarksMap::auxParticleFilterCleanUp() --> void");
		cl.def("assign", (class mrpt::maps::CLandmarksMap & (mrpt::maps::CLandmarksMap::*)(const class mrpt::maps::CLandmarksMap &)) &mrpt::maps::CLandmarksMap::operator=, "C++: mrpt::maps::CLandmarksMap::operator=(const class mrpt::maps::CLandmarksMap &) --> class mrpt::maps::CLandmarksMap &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks file:mrpt/maps/CLandmarksMap.h line:133
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks, std::shared_ptr<mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks>> cl(enclosing_class, "TCustomSequenceLandmarks", "The list of landmarks: the wrapper class is just for maintaining the\n KD-Tree representation");
			cl.def( pybind11::init( [](){ return new mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks(); } ) );
			cl.def( pybind11::init( [](mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks const &o){ return new mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks(o); } ) );
			cl.def("clear", (void (mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks::*)()) &mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks::clear, "C++: mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks::clear() --> void");
			cl.def("size", (size_t (mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks::*)() const) &mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks::size, "C++: mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks::size() const --> size_t");
			cl.def("push_back", (void (mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks::*)(const class mrpt::maps::CLandmark &)) &mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks::push_back, "The object is copied, thus the original copy passed as a parameter\n can be released.\n\nC++: mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks::push_back(const class mrpt::maps::CLandmark &) --> void", pybind11::arg("lm"));
			cl.def("get", (class mrpt::maps::CLandmark * (mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks::*)(unsigned int)) &mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks::get, "C++: mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks::get(unsigned int) --> class mrpt::maps::CLandmark *", pybind11::return_value_policy::automatic, pybind11::arg("indx"));
			cl.def("isToBeModified", (void (mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks::*)(unsigned int)) &mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks::isToBeModified, "C++: mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks::isToBeModified(unsigned int) --> void", pybind11::arg("indx"));
			cl.def("hasBeenModified", (void (mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks::*)(unsigned int)) &mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks::hasBeenModified, "C++: mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks::hasBeenModified(unsigned int) --> void", pybind11::arg("indx"));
			cl.def("hasBeenModifiedAll", (void (mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks::*)()) &mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks::hasBeenModifiedAll, "C++: mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks::hasBeenModifiedAll() --> void");
			cl.def("erase", (void (mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks::*)(unsigned int)) &mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks::erase, "C++: mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks::erase(unsigned int) --> void", pybind11::arg("indx"));
			cl.def("getByID", (const class mrpt::maps::CLandmark * (mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks::*)(int64_t) const) &mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks::getByID, "Returns the landmark with a given landmrk ID, or nullptr if not\n found\n\nC++: mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks::getByID(int64_t) const --> const class mrpt::maps::CLandmark *", pybind11::return_value_policy::automatic, pybind11::arg("ID"));
			cl.def("getByBeaconID", (const class mrpt::maps::CLandmark * (mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks::*)(unsigned int) const) &mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks::getByBeaconID, "Returns the landmark with a given beacon ID, or nullptr if not found\n\nC++: mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks::getByBeaconID(unsigned int) const --> const class mrpt::maps::CLandmark *", pybind11::return_value_policy::automatic, pybind11::arg("ID"));
			cl.def("getLargestDistanceFromOrigin", (float (mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks::*)() const) &mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks::getLargestDistanceFromOrigin, "This method returns the largest distance from the origin to any of\n the points, such as a sphere centered at the origin with this radius\n cover ALL the points in the map (the results are buffered, such as,\n if the map is not modified, the second call will be much faster than\n the first one).\n\nC++: mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks::getLargestDistanceFromOrigin() const --> float");
			cl.def("assign", (struct mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks & (mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks::*)(const struct mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks &)) &mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks::operator=, "C++: mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks::operator=(const struct mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks &) --> struct mrpt::maps::CLandmarksMap::TCustomSequenceLandmarks &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::maps::CLandmarksMap::TInsertionOptions file:mrpt/maps/CLandmarksMap.h line:227
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CLandmarksMap::TInsertionOptions, std::shared_ptr<mrpt::maps::CLandmarksMap::TInsertionOptions>, PyCallBack_mrpt_maps_CLandmarksMap_TInsertionOptions, mrpt::config::CLoadableOptions> cl(enclosing_class, "TInsertionOptions", "With this struct options are provided to the observation insertion\n process.");
			cl.def( pybind11::init( [](){ return new mrpt::maps::CLandmarksMap::TInsertionOptions(); }, [](){ return new PyCallBack_mrpt_maps_CLandmarksMap_TInsertionOptions(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_maps_CLandmarksMap_TInsertionOptions const &o){ return new PyCallBack_mrpt_maps_CLandmarksMap_TInsertionOptions(o); } ) );
			cl.def( pybind11::init( [](mrpt::maps::CLandmarksMap::TInsertionOptions const &o){ return new mrpt::maps::CLandmarksMap::TInsertionOptions(o); } ) );
			cl.def_readwrite("insert_SIFTs_from_monocular_images", &mrpt::maps::CLandmarksMap::TInsertionOptions::insert_SIFTs_from_monocular_images);
			cl.def_readwrite("insert_SIFTs_from_stereo_images", &mrpt::maps::CLandmarksMap::TInsertionOptions::insert_SIFTs_from_stereo_images);
			cl.def_readwrite("insert_Landmarks_from_range_scans", &mrpt::maps::CLandmarksMap::TInsertionOptions::insert_Landmarks_from_range_scans);
			cl.def_readwrite("SiftCorrRatioThreshold", &mrpt::maps::CLandmarksMap::TInsertionOptions::SiftCorrRatioThreshold);
			cl.def_readwrite("SiftLikelihoodThreshold", &mrpt::maps::CLandmarksMap::TInsertionOptions::SiftLikelihoodThreshold);
			cl.def_readwrite("SiftEDDThreshold", &mrpt::maps::CLandmarksMap::TInsertionOptions::SiftEDDThreshold);
			cl.def_readwrite("SIFTMatching3DMethod", &mrpt::maps::CLandmarksMap::TInsertionOptions::SIFTMatching3DMethod);
			cl.def_readwrite("SIFTLikelihoodMethod", &mrpt::maps::CLandmarksMap::TInsertionOptions::SIFTLikelihoodMethod);
			cl.def_readwrite("SIFTsLoadDistanceOfTheMean", &mrpt::maps::CLandmarksMap::TInsertionOptions::SIFTsLoadDistanceOfTheMean);
			cl.def_readwrite("SIFTsLoadEllipsoidWidth", &mrpt::maps::CLandmarksMap::TInsertionOptions::SIFTsLoadEllipsoidWidth);
			cl.def_readwrite("SIFTs_stdXY", &mrpt::maps::CLandmarksMap::TInsertionOptions::SIFTs_stdXY);
			cl.def_readwrite("SIFTs_stdDisparity", &mrpt::maps::CLandmarksMap::TInsertionOptions::SIFTs_stdDisparity);
			cl.def_readwrite("SIFTs_numberOfKLTKeypoints", &mrpt::maps::CLandmarksMap::TInsertionOptions::SIFTs_numberOfKLTKeypoints);
			cl.def_readwrite("SIFTs_stereo_maxDepth", &mrpt::maps::CLandmarksMap::TInsertionOptions::SIFTs_stereo_maxDepth);
			cl.def_readwrite("SIFTs_epipolar_TH", &mrpt::maps::CLandmarksMap::TInsertionOptions::SIFTs_epipolar_TH);
			cl.def_readwrite("PLOT_IMAGES", &mrpt::maps::CLandmarksMap::TInsertionOptions::PLOT_IMAGES);
			cl.def_readwrite("SIFT_feat_options", &mrpt::maps::CLandmarksMap::TInsertionOptions::SIFT_feat_options);
			cl.def("loadFromConfigFile", (void (mrpt::maps::CLandmarksMap::TInsertionOptions::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::maps::CLandmarksMap::TInsertionOptions::loadFromConfigFile, "C++: mrpt::maps::CLandmarksMap::TInsertionOptions::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
			cl.def("assign", (struct mrpt::maps::CLandmarksMap::TInsertionOptions & (mrpt::maps::CLandmarksMap::TInsertionOptions::*)(const struct mrpt::maps::CLandmarksMap::TInsertionOptions &)) &mrpt::maps::CLandmarksMap::TInsertionOptions::operator=, "C++: mrpt::maps::CLandmarksMap::TInsertionOptions::operator=(const struct mrpt::maps::CLandmarksMap::TInsertionOptions &) --> struct mrpt::maps::CLandmarksMap::TInsertionOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::maps::CLandmarksMap::TLikelihoodOptions file:mrpt/maps/CLandmarksMap.h line:341
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CLandmarksMap::TLikelihoodOptions, std::shared_ptr<mrpt::maps::CLandmarksMap::TLikelihoodOptions>, PyCallBack_mrpt_maps_CLandmarksMap_TLikelihoodOptions, mrpt::config::CLoadableOptions> cl(enclosing_class, "TLikelihoodOptions", "With this struct options are provided to the likelihood computations.");
			cl.def( pybind11::init( [](){ return new mrpt::maps::CLandmarksMap::TLikelihoodOptions(); }, [](){ return new PyCallBack_mrpt_maps_CLandmarksMap_TLikelihoodOptions(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_maps_CLandmarksMap_TLikelihoodOptions const &o){ return new PyCallBack_mrpt_maps_CLandmarksMap_TLikelihoodOptions(o); } ) );
			cl.def( pybind11::init( [](mrpt::maps::CLandmarksMap::TLikelihoodOptions const &o){ return new mrpt::maps::CLandmarksMap::TLikelihoodOptions(o); } ) );
			cl.def_readwrite("rangeScan2D_decimation", &mrpt::maps::CLandmarksMap::TLikelihoodOptions::rangeScan2D_decimation);
			cl.def_readwrite("SIFTs_sigma_euclidean_dist", &mrpt::maps::CLandmarksMap::TLikelihoodOptions::SIFTs_sigma_euclidean_dist);
			cl.def_readwrite("SIFTs_sigma_descriptor_dist", &mrpt::maps::CLandmarksMap::TLikelihoodOptions::SIFTs_sigma_descriptor_dist);
			cl.def_readwrite("SIFTs_mahaDist_std", &mrpt::maps::CLandmarksMap::TLikelihoodOptions::SIFTs_mahaDist_std);
			cl.def_readwrite("SIFTnullCorrespondenceDistance", &mrpt::maps::CLandmarksMap::TLikelihoodOptions::SIFTnullCorrespondenceDistance);
			cl.def_readwrite("SIFTs_decimation", &mrpt::maps::CLandmarksMap::TLikelihoodOptions::SIFTs_decimation);
			cl.def_readwrite("SIFT_feat_options", &mrpt::maps::CLandmarksMap::TLikelihoodOptions::SIFT_feat_options);
			cl.def_readwrite("beaconRangesStd", &mrpt::maps::CLandmarksMap::TLikelihoodOptions::beaconRangesStd);
			cl.def_readwrite("beaconRangesUseObservationStd", &mrpt::maps::CLandmarksMap::TLikelihoodOptions::beaconRangesUseObservationStd);
			cl.def_readwrite("extRobotPoseStd", &mrpt::maps::CLandmarksMap::TLikelihoodOptions::extRobotPoseStd);
			cl.def_readwrite("GPSOrigin", &mrpt::maps::CLandmarksMap::TLikelihoodOptions::GPSOrigin);
			cl.def_readwrite("GPS_sigma", &mrpt::maps::CLandmarksMap::TLikelihoodOptions::GPS_sigma);
			cl.def("loadFromConfigFile", (void (mrpt::maps::CLandmarksMap::TLikelihoodOptions::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::maps::CLandmarksMap::TLikelihoodOptions::loadFromConfigFile, "C++: mrpt::maps::CLandmarksMap::TLikelihoodOptions::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
			cl.def("assign", (struct mrpt::maps::CLandmarksMap::TLikelihoodOptions & (mrpt::maps::CLandmarksMap::TLikelihoodOptions::*)(const struct mrpt::maps::CLandmarksMap::TLikelihoodOptions &)) &mrpt::maps::CLandmarksMap::TLikelihoodOptions::operator=, "C++: mrpt::maps::CLandmarksMap::TLikelihoodOptions::operator=(const struct mrpt::maps::CLandmarksMap::TLikelihoodOptions &) --> struct mrpt::maps::CLandmarksMap::TLikelihoodOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));

			{ // mrpt::maps::CLandmarksMap::TLikelihoodOptions::TGPSOrigin file:mrpt/maps/CLandmarksMap.h line:401
				auto & enclosing_class = cl;
				pybind11::class_<mrpt::maps::CLandmarksMap::TLikelihoodOptions::TGPSOrigin, std::shared_ptr<mrpt::maps::CLandmarksMap::TLikelihoodOptions::TGPSOrigin>> cl(enclosing_class, "TGPSOrigin", "This struct store de GPS longitude, latitude (in degrees ) and\n altitude (in meters) for the first GPS observation\n compose with de sensor position on the robot ");
				cl.def( pybind11::init( [](){ return new mrpt::maps::CLandmarksMap::TLikelihoodOptions::TGPSOrigin(); } ) );
				cl.def( pybind11::init( [](mrpt::maps::CLandmarksMap::TLikelihoodOptions::TGPSOrigin const &o){ return new mrpt::maps::CLandmarksMap::TLikelihoodOptions::TGPSOrigin(o); } ) );
				cl.def_readwrite("longitude", &mrpt::maps::CLandmarksMap::TLikelihoodOptions::TGPSOrigin::longitude);
				cl.def_readwrite("latitude", &mrpt::maps::CLandmarksMap::TLikelihoodOptions::TGPSOrigin::latitude);
				cl.def_readwrite("altitude", &mrpt::maps::CLandmarksMap::TLikelihoodOptions::TGPSOrigin::altitude);
				cl.def_readwrite("ang", &mrpt::maps::CLandmarksMap::TLikelihoodOptions::TGPSOrigin::ang);
				cl.def_readwrite("x_shift", &mrpt::maps::CLandmarksMap::TLikelihoodOptions::TGPSOrigin::x_shift);
				cl.def_readwrite("y_shift", &mrpt::maps::CLandmarksMap::TLikelihoodOptions::TGPSOrigin::y_shift);
				cl.def_readwrite("min_sat", &mrpt::maps::CLandmarksMap::TLikelihoodOptions::TGPSOrigin::min_sat);
				cl.def("assign", (struct mrpt::maps::CLandmarksMap::TLikelihoodOptions::TGPSOrigin & (mrpt::maps::CLandmarksMap::TLikelihoodOptions::TGPSOrigin::*)(const struct mrpt::maps::CLandmarksMap::TLikelihoodOptions::TGPSOrigin &)) &mrpt::maps::CLandmarksMap::TLikelihoodOptions::TGPSOrigin::operator=, "C++: mrpt::maps::CLandmarksMap::TLikelihoodOptions::TGPSOrigin::operator=(const struct mrpt::maps::CLandmarksMap::TLikelihoodOptions::TGPSOrigin &) --> struct mrpt::maps::CLandmarksMap::TLikelihoodOptions::TGPSOrigin &", pybind11::return_value_policy::automatic, pybind11::arg(""));
			}

		}

		{ // mrpt::maps::CLandmarksMap::TInsertionResults file:mrpt/maps/CLandmarksMap.h line:428
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CLandmarksMap::TInsertionResults, std::shared_ptr<mrpt::maps::CLandmarksMap::TInsertionResults>> cl(enclosing_class, "TInsertionResults", "This struct stores extra results from invoking insertObservation");
			cl.def( pybind11::init( [](){ return new mrpt::maps::CLandmarksMap::TInsertionResults(); } ) );
			cl.def_readwrite("nSiftL", &mrpt::maps::CLandmarksMap::TInsertionResults::nSiftL);
			cl.def_readwrite("nSiftR", &mrpt::maps::CLandmarksMap::TInsertionResults::nSiftR);
			cl.def("assign", (struct mrpt::maps::CLandmarksMap::TInsertionResults & (mrpt::maps::CLandmarksMap::TInsertionResults::*)(const struct mrpt::maps::CLandmarksMap::TInsertionResults &)) &mrpt::maps::CLandmarksMap::TInsertionResults::operator=, "C++: mrpt::maps::CLandmarksMap::TInsertionResults::operator=(const struct mrpt::maps::CLandmarksMap::TInsertionResults &) --> struct mrpt::maps::CLandmarksMap::TInsertionResults &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::maps::CLandmarksMap::TFuseOptions file:mrpt/maps/CLandmarksMap.h line:439
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CLandmarksMap::TFuseOptions, std::shared_ptr<mrpt::maps::CLandmarksMap::TFuseOptions>> cl(enclosing_class, "TFuseOptions", "With this struct options are provided to the fusion process.");
			cl.def( pybind11::init( [](){ return new mrpt::maps::CLandmarksMap::TFuseOptions(); } ) );
			cl.def_readwrite("minTimesSeen", &mrpt::maps::CLandmarksMap::TFuseOptions::minTimesSeen);
			cl.def_readwrite("ellapsedTime", &mrpt::maps::CLandmarksMap::TFuseOptions::ellapsedTime);
			cl.def("assign", (struct mrpt::maps::CLandmarksMap::TFuseOptions & (mrpt::maps::CLandmarksMap::TFuseOptions::*)(const struct mrpt::maps::CLandmarksMap::TFuseOptions &)) &mrpt::maps::CLandmarksMap::TFuseOptions::operator=, "C++: mrpt::maps::CLandmarksMap::TFuseOptions::operator=(const struct mrpt::maps::CLandmarksMap::TFuseOptions &) --> struct mrpt::maps::CLandmarksMap::TFuseOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::maps::CLandmarksMap::TMapDefinitionBase file: line:74
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CLandmarksMap::TMapDefinitionBase, std::shared_ptr<mrpt::maps::CLandmarksMap::TMapDefinitionBase>> cl(enclosing_class, "TMapDefinitionBase", "");
		}

		{ // mrpt::maps::CLandmarksMap::TMapDefinition file: line:78
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CLandmarksMap::TMapDefinition, std::shared_ptr<mrpt::maps::CLandmarksMap::TMapDefinition>, PyCallBack_mrpt_maps_CLandmarksMap_TMapDefinition, mrpt::maps::CLandmarksMap::TMapDefinitionBase> cl(enclosing_class, "TMapDefinition", "");
			cl.def( pybind11::init( [](){ return new mrpt::maps::CLandmarksMap::TMapDefinition(); }, [](){ return new PyCallBack_mrpt_maps_CLandmarksMap_TMapDefinition(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_maps_CLandmarksMap_TMapDefinition const &o){ return new PyCallBack_mrpt_maps_CLandmarksMap_TMapDefinition(o); } ) );
			cl.def( pybind11::init( [](mrpt::maps::CLandmarksMap::TMapDefinition const &o){ return new mrpt::maps::CLandmarksMap::TMapDefinition(o); } ) );
			cl.def_readwrite("initialBeacons", &mrpt::maps::CLandmarksMap::TMapDefinition::initialBeacons);
			cl.def_readwrite("insertionOpts", &mrpt::maps::CLandmarksMap::TMapDefinition::insertionOpts);
			cl.def_readwrite("likelihoodOpts", &mrpt::maps::CLandmarksMap::TMapDefinition::likelihoodOpts);
		}

	}
}
