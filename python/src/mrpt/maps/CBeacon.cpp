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
#include <mrpt/core/safe_pointers.h>
#include <mrpt/img/TColor.h>
#include <mrpt/maps/CBeacon.h>
#include <mrpt/maps/CBeaconMap.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/TMetricMapInitializer.h>
#include <mrpt/maps/metric_map_types.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CProbabilityDensityFunction.h>
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
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPointPDF.h>
#include <mrpt/poses/CPointPDFGaussian.h>
#include <mrpt/poses/CPointPDFSOG.h>
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
#include <tuple>
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

// mrpt::maps::CBeacon file:mrpt/maps/CBeacon.h line:34
struct PyCallBack_mrpt_maps_CBeacon : public mrpt::maps::CBeacon {
	using mrpt::maps::CBeacon::CBeacon;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeacon *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CBeacon::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeacon *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CBeacon::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeacon *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CBeacon::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeacon *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CBeacon::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeacon *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CBeacon::serializeFrom(a0, a1);
	}
	void getMean(class mrpt::poses::CPoint3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeacon *>(this), "getMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CBeacon::getMean(a0);
	}
	using _binder_ret_0 = class std::tuple<class mrpt::math::CMatrixFixed<double, 3, 3>, class mrpt::poses::CPoint3D>;
	_binder_ret_0 getCovarianceAndMean() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeacon *>(this), "getCovarianceAndMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<_binder_ret_0>::value) {
				static pybind11::detail::override_caster_t<_binder_ret_0> caster;
				return pybind11::detail::cast_ref<_binder_ret_0>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<_binder_ret_0>(std::move(o));
		}
		return CBeacon::getCovarianceAndMean();
	}
	void copyFrom(const class mrpt::poses::CPointPDF & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeacon *>(this), "copyFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CBeacon::copyFrom(a0);
	}
	bool saveToTextFile(const std::string & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeacon *>(this), "saveToTextFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CBeacon::saveToTextFile(a0);
	}
	void changeCoordinatesReference(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeacon *>(this), "changeCoordinatesReference");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CBeacon::changeCoordinatesReference(a0);
	}
	void drawSingleSample(class mrpt::poses::CPoint3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeacon *>(this), "drawSingleSample");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CBeacon::drawSingleSample(a0);
	}
	void bayesianFusion(const class mrpt::poses::CPointPDF & a0, const class mrpt::poses::CPointPDF & a1, const double a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeacon *>(this), "bayesianFusion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CBeacon::bayesianFusion(a0, a1, a2);
	}
	bool isInfType() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeacon *>(this), "isInfType");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CProbabilityDensityFunction::isInfType();
	}
	void getInformationMatrix(class mrpt::math::CMatrixFixed<double, 3, 3> & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeacon *>(this), "getInformationMatrix");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CProbabilityDensityFunction::getInformationMatrix(a0);
	}
};

// mrpt::maps::CBeaconMap file:mrpt/maps/CBeaconMap.h line:43
struct PyCallBack_mrpt_maps_CBeaconMap : public mrpt::maps::CBeaconMap {
	using mrpt::maps::CBeaconMap::CBeaconMap;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeaconMap *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CBeaconMap::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeaconMap *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CBeaconMap::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeaconMap *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CBeaconMap::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeaconMap *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CBeaconMap::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeaconMap *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CBeaconMap::serializeFrom(a0, a1);
	}
	void internal_clear() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeaconMap *>(this), "internal_clear");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CBeaconMap::internal_clear();
	}
	double internal_computeObservationLikelihood(const class mrpt::obs::CObservation & a0, const class mrpt::poses::CPose3D & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeaconMap *>(this), "internal_computeObservationLikelihood");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CBeaconMap::internal_computeObservationLikelihood(a0, a1);
	}
	std::string asString() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeaconMap *>(this), "asString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CBeaconMap::asString();
	}
	float compute3DMatchingRatio(const class mrpt::maps::CMetricMap * a0, const class mrpt::poses::CPose3D & a1, const struct mrpt::maps::TMatchingRatioParams & a2) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeaconMap *>(this), "compute3DMatchingRatio");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<float>::value) {
				static pybind11::detail::override_caster_t<float> caster;
				return pybind11::detail::cast_ref<float>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<float>(std::move(o));
		}
		return CBeaconMap::compute3DMatchingRatio(a0, a1, a2);
	}
	void determineMatching2D(const class mrpt::maps::CMetricMap * a0, const class mrpt::poses::CPose2D & a1, class mrpt::tfest::TMatchingPairListTempl<float> & a2, const struct mrpt::maps::TMatchingParams & a3, struct mrpt::maps::TMatchingExtraResults & a4) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeaconMap *>(this), "determineMatching2D");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CBeaconMap::determineMatching2D(a0, a1, a2, a3, a4);
	}
	bool isEmpty() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeaconMap *>(this), "isEmpty");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CBeaconMap::isEmpty();
	}
	void saveMetricMapRepresentationToFile(const std::string & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeaconMap *>(this), "saveMetricMapRepresentationToFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CBeaconMap::saveMetricMapRepresentationToFile(a0);
	}
	void getVisualizationInto(class mrpt::opengl::CSetOfObjects & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeaconMap *>(this), "getVisualizationInto");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CBeaconMap::getVisualizationInto(a0);
	}
	struct mrpt::math::TBoundingBox_<float> boundingBox() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeaconMap *>(this), "boundingBox");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeaconMap *>(this), "canComputeObservationLikelihood");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeaconMap *>(this), "determineMatching3D");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeaconMap *>(this), "auxParticleFilterCleanUp");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeaconMap *>(this), "squareDistanceToClosestCorrespondence");
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

// mrpt::maps::CBeaconMap::TLikelihoodOptions file:mrpt/maps/CBeaconMap.h line:112
struct PyCallBack_mrpt_maps_CBeaconMap_TLikelihoodOptions : public mrpt::maps::CBeaconMap::TLikelihoodOptions {
	using mrpt::maps::CBeaconMap::TLikelihoodOptions::TLikelihoodOptions;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeaconMap::TLikelihoodOptions *>(this), "loadFromConfigFile");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeaconMap::TLikelihoodOptions *>(this), "saveToConfigFile");
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

// mrpt::maps::CBeaconMap::TInsertionOptions file:mrpt/maps/CBeaconMap.h line:129
struct PyCallBack_mrpt_maps_CBeaconMap_TInsertionOptions : public mrpt::maps::CBeaconMap::TInsertionOptions {
	using mrpt::maps::CBeaconMap::TInsertionOptions::TInsertionOptions;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeaconMap::TInsertionOptions *>(this), "loadFromConfigFile");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeaconMap::TInsertionOptions *>(this), "saveToConfigFile");
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

// mrpt::maps::CBeaconMap::TMapDefinition file: line:78
struct PyCallBack_mrpt_maps_CBeaconMap_TMapDefinition : public mrpt::maps::CBeaconMap::TMapDefinition {
	using mrpt::maps::CBeaconMap::TMapDefinition::TMapDefinition;

	void loadFromConfigFile_map_specific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeaconMap::TMapDefinition *>(this), "loadFromConfigFile_map_specific");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeaconMap::TMapDefinition *>(this), "loadFromConfigFile");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CBeaconMap::TMapDefinition *>(this), "saveToConfigFile");
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

void bind_mrpt_maps_CBeacon(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::maps::CBeacon file:mrpt/maps/CBeacon.h line:34
		pybind11::class_<mrpt::maps::CBeacon, std::shared_ptr<mrpt::maps::CBeacon>, PyCallBack_mrpt_maps_CBeacon, mrpt::poses::CPointPDF> cl(M("mrpt::maps"), "CBeacon", "The class for storing individual \"beacon landmarks\" under a variety of 3D\n position PDF distributions.\n  This class is used for storage within the class CBeaconMap.\n  The class implements the same methods than the interface \"CPointPDF\", and\n invoking them actually becomes\n   a mapping into the methods of the current PDF representation of the\n beacon, selectable by means of \"m_typePDF\"\n \n\n CBeaconMap, CPointPDFSOG\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::maps::CBeacon(); }, [](){ return new PyCallBack_mrpt_maps_CBeacon(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_maps_CBeacon const &o){ return new PyCallBack_mrpt_maps_CBeacon(o); } ) );
		cl.def( pybind11::init( [](mrpt::maps::CBeacon const &o){ return new mrpt::maps::CBeacon(o); } ) );

		pybind11::enum_<mrpt::maps::CBeacon::TTypePDF>(cl, "TTypePDF", pybind11::arithmetic(), "See m_typePDF")
			.value("pdfMonteCarlo", mrpt::maps::CBeacon::pdfMonteCarlo)
			.value("pdfGauss", mrpt::maps::CBeacon::pdfGauss)
			.value("pdfSOG", mrpt::maps::CBeacon::pdfSOG)
			.export_values();

		cl.def_readwrite("m_typePDF", &mrpt::maps::CBeacon::m_typePDF);
		cl.def_readwrite("m_locationMC", &mrpt::maps::CBeacon::m_locationMC);
		cl.def_readwrite("m_locationGauss", &mrpt::maps::CBeacon::m_locationGauss);
		cl.def_readwrite("m_locationSOG", &mrpt::maps::CBeacon::m_locationSOG);
		cl.def_readwrite("m_ID", &mrpt::maps::CBeacon::m_ID);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::maps::CBeacon::GetRuntimeClassIdStatic, "C++: mrpt::maps::CBeacon::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::maps::CBeacon::*)() const) &mrpt::maps::CBeacon::GetRuntimeClass, "C++: mrpt::maps::CBeacon::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::maps::CBeacon::*)() const) &mrpt::maps::CBeacon::clone, "C++: mrpt::maps::CBeacon::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::maps::CBeacon::CreateObject, "C++: mrpt::maps::CBeacon::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("getMean", (void (mrpt::maps::CBeacon::*)(class mrpt::poses::CPoint3D &) const) &mrpt::maps::CBeacon::getMean, "C++: mrpt::maps::CBeacon::getMean(class mrpt::poses::CPoint3D &) const --> void", pybind11::arg("mean_point"));
		cl.def("getCovarianceAndMean", (class std::tuple<class mrpt::math::CMatrixFixed<double, 3, 3>, class mrpt::poses::CPoint3D> (mrpt::maps::CBeacon::*)() const) &mrpt::maps::CBeacon::getCovarianceAndMean, "C++: mrpt::maps::CBeacon::getCovarianceAndMean() const --> class std::tuple<class mrpt::math::CMatrixFixed<double, 3, 3>, class mrpt::poses::CPoint3D>");
		cl.def("copyFrom", (void (mrpt::maps::CBeacon::*)(const class mrpt::poses::CPointPDF &)) &mrpt::maps::CBeacon::copyFrom, "Copy operator, translating if necesary (for example, between particles\n and gaussian representations) \n\nC++: mrpt::maps::CBeacon::copyFrom(const class mrpt::poses::CPointPDF &) --> void", pybind11::arg("o"));
		cl.def("saveToTextFile", (bool (mrpt::maps::CBeacon::*)(const std::string &) const) &mrpt::maps::CBeacon::saveToTextFile, "Save PDF's particles to a text file. See derived classes for more\n information about the format of generated files \n\nC++: mrpt::maps::CBeacon::saveToTextFile(const std::string &) const --> bool", pybind11::arg("file"));
		cl.def("changeCoordinatesReference", (void (mrpt::maps::CBeacon::*)(const class mrpt::poses::CPose3D &)) &mrpt::maps::CBeacon::changeCoordinatesReference, "this = p (+) this. This can be used to convert a PDF from local\n coordinates to global, providing the point (newReferenceBase) from which\n   \"to project\" the current pdf. Result PDF substituted the currently\n stored one in the object.\n\nC++: mrpt::maps::CBeacon::changeCoordinatesReference(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newReferenceBase"));
		cl.def("getVisualizationInto", (void (mrpt::maps::CBeacon::*)(class mrpt::opengl::CSetOfObjects &) const) &mrpt::maps::CBeacon::getVisualizationInto, "Saves a 3D representation of the beacon into a given OpenGL scene  \n\nC++: mrpt::maps::CBeacon::getVisualizationInto(class mrpt::opengl::CSetOfObjects &) const --> void", pybind11::arg("o"));
		cl.def("getAsMatlabDrawCommands", (void (mrpt::maps::CBeacon::*)(class std::vector<std::string > &) const) &mrpt::maps::CBeacon::getAsMatlabDrawCommands, "Gets a set of MATLAB commands which draw the current state of the\n beacon: \n\nC++: mrpt::maps::CBeacon::getAsMatlabDrawCommands(class std::vector<std::string > &) const --> void", pybind11::arg("out_Str"));
		cl.def("drawSingleSample", (void (mrpt::maps::CBeacon::*)(class mrpt::poses::CPoint3D &) const) &mrpt::maps::CBeacon::drawSingleSample, "Draw a sample from the pdf. \n\nC++: mrpt::maps::CBeacon::drawSingleSample(class mrpt::poses::CPoint3D &) const --> void", pybind11::arg("outSample"));
		cl.def("bayesianFusion", [](mrpt::maps::CBeacon &o, const class mrpt::poses::CPointPDF & a0, const class mrpt::poses::CPointPDF & a1) -> void { return o.bayesianFusion(a0, a1); }, "", pybind11::arg("p1"), pybind11::arg("p2"));
		cl.def("bayesianFusion", (void (mrpt::maps::CBeacon::*)(const class mrpt::poses::CPointPDF &, const class mrpt::poses::CPointPDF &, const double)) &mrpt::maps::CBeacon::bayesianFusion, "Bayesian fusion of two point distributions (product of two\n distributions->new distribution), then save the result in this object\n (WARNING: See implementing classes to see classes that can and cannot be\n mixtured!)\n \n\n The first distribution to fuse\n \n\n The second distribution to fuse\n \n\n If set to different of 0, the result of\n very separate Gaussian modes (that will result in negligible components)\n in SOGs will be dropped to reduce the number of modes in the output.\n\nC++: mrpt::maps::CBeacon::bayesianFusion(const class mrpt::poses::CPointPDF &, const class mrpt::poses::CPointPDF &, const double) --> void", pybind11::arg("p1"), pybind11::arg("p2"), pybind11::arg("minMahalanobisDistToDrop"));
		cl.def("generateObservationModelDistribution", [](mrpt::maps::CBeacon const &o, float const & a0, class mrpt::poses::CPointPDFSOG & a1, const class mrpt::maps::CBeaconMap * a2, const class mrpt::poses::CPoint3D & a3) -> void { return o.generateObservationModelDistribution(a0, a1, a2, a3); }, "", pybind11::arg("sensedRange"), pybind11::arg("outPDF"), pybind11::arg("myBeaconMap"), pybind11::arg("sensorPntOnRobot"));
		cl.def("generateObservationModelDistribution", [](mrpt::maps::CBeacon const &o, float const & a0, class mrpt::poses::CPointPDFSOG & a1, const class mrpt::maps::CBeaconMap * a2, const class mrpt::poses::CPoint3D & a3, const class mrpt::poses::CPoint3D & a4) -> void { return o.generateObservationModelDistribution(a0, a1, a2, a3, a4); }, "", pybind11::arg("sensedRange"), pybind11::arg("outPDF"), pybind11::arg("myBeaconMap"), pybind11::arg("sensorPntOnRobot"), pybind11::arg("centerPoint"));
		cl.def("generateObservationModelDistribution", (void (mrpt::maps::CBeacon::*)(float, class mrpt::poses::CPointPDFSOG &, const class mrpt::maps::CBeaconMap *, const class mrpt::poses::CPoint3D &, const class mrpt::poses::CPoint3D &, float) const) &mrpt::maps::CBeacon::generateObservationModelDistribution, "Compute the observation model p(z_t|x_t) for a given observation (range\n value), and return it as an approximate SOG.\n  Note that if the beacon is a SOG itself, the number of gaussian modes\n will be square.\n  As a speed-up, if a \"center point\"+\"maxDistanceFromCenter\" is supplied\n (maxDistanceFromCenter!=0), those modes farther than this sphere will be\n discarded.\n  Parameters such as the stdSigma of the sensor are gathered from\n \"myBeaconMap\"\n  The result is one \"ring\" for each Gaussian mode that represent the\n beacon position in this object.\n  The position of the sensor on the robot is used to shift the resulting\n densities such as they represent the position of the robot, not the\n sensor.\n  \n\n CBeaconMap::insertionOptions, generateRingSOG\n\nC++: mrpt::maps::CBeacon::generateObservationModelDistribution(float, class mrpt::poses::CPointPDFSOG &, const class mrpt::maps::CBeaconMap *, const class mrpt::poses::CPoint3D &, const class mrpt::poses::CPoint3D &, float) const --> void", pybind11::arg("sensedRange"), pybind11::arg("outPDF"), pybind11::arg("myBeaconMap"), pybind11::arg("sensorPntOnRobot"), pybind11::arg("centerPoint"), pybind11::arg("maxDistanceFromCenter"));
		cl.def_static("generateRingSOG", [](float const & a0, class mrpt::poses::CPointPDFSOG & a1, const class mrpt::maps::CBeaconMap * a2, const class mrpt::poses::CPoint3D & a3) -> void { return mrpt::maps::CBeacon::generateRingSOG(a0, a1, a2, a3); }, "", pybind11::arg("sensedRange"), pybind11::arg("outPDF"), pybind11::arg("myBeaconMap"), pybind11::arg("sensorPnt"));
		cl.def_static("generateRingSOG", [](float const & a0, class mrpt::poses::CPointPDFSOG & a1, const class mrpt::maps::CBeaconMap * a2, const class mrpt::poses::CPoint3D & a3, const class mrpt::math::CMatrixFixed<double, 3, 3> * a4) -> void { return mrpt::maps::CBeacon::generateRingSOG(a0, a1, a2, a3, a4); }, "", pybind11::arg("sensedRange"), pybind11::arg("outPDF"), pybind11::arg("myBeaconMap"), pybind11::arg("sensorPnt"), pybind11::arg("covarianceCompositionToAdd"));
		cl.def_static("generateRingSOG", [](float const & a0, class mrpt::poses::CPointPDFSOG & a1, const class mrpt::maps::CBeaconMap * a2, const class mrpt::poses::CPoint3D & a3, const class mrpt::math::CMatrixFixed<double, 3, 3> * a4, bool const & a5) -> void { return mrpt::maps::CBeacon::generateRingSOG(a0, a1, a2, a3, a4, a5); }, "", pybind11::arg("sensedRange"), pybind11::arg("outPDF"), pybind11::arg("myBeaconMap"), pybind11::arg("sensorPnt"), pybind11::arg("covarianceCompositionToAdd"), pybind11::arg("clearPreviousContentsOutPDF"));
		cl.def_static("generateRingSOG", [](float const & a0, class mrpt::poses::CPointPDFSOG & a1, const class mrpt::maps::CBeaconMap * a2, const class mrpt::poses::CPoint3D & a3, const class mrpt::math::CMatrixFixed<double, 3, 3> * a4, bool const & a5, const class mrpt::poses::CPoint3D & a6) -> void { return mrpt::maps::CBeacon::generateRingSOG(a0, a1, a2, a3, a4, a5, a6); }, "", pybind11::arg("sensedRange"), pybind11::arg("outPDF"), pybind11::arg("myBeaconMap"), pybind11::arg("sensorPnt"), pybind11::arg("covarianceCompositionToAdd"), pybind11::arg("clearPreviousContentsOutPDF"), pybind11::arg("centerPoint"));
		cl.def_static("generateRingSOG", (void (*)(float, class mrpt::poses::CPointPDFSOG &, const class mrpt::maps::CBeaconMap *, const class mrpt::poses::CPoint3D &, const class mrpt::math::CMatrixFixed<double, 3, 3> *, bool, const class mrpt::poses::CPoint3D &, float)) &mrpt::maps::CBeacon::generateRingSOG, "This static method returns a SOG with ring-shape (or as a 3D sphere)\n that can be used to initialize a beacon if observed the first time.\n  sensorPnt is the center of the ring/sphere, i.e. the absolute position\n of the range sensor.\n  If clearPreviousContentsOutPDF=false, the SOG modes will be added to\n the current contents of outPDF\n  If the 3x3 matrix covarianceCompositionToAdd is provided, it will be\n add to every Gaussian (to model the composition of uncertainty).\n \n\n generateObservationModelDistribution\n\nC++: mrpt::maps::CBeacon::generateRingSOG(float, class mrpt::poses::CPointPDFSOG &, const class mrpt::maps::CBeaconMap *, const class mrpt::poses::CPoint3D &, const class mrpt::math::CMatrixFixed<double, 3, 3> *, bool, const class mrpt::poses::CPoint3D &, float) --> void", pybind11::arg("sensedRange"), pybind11::arg("outPDF"), pybind11::arg("myBeaconMap"), pybind11::arg("sensorPnt"), pybind11::arg("covarianceCompositionToAdd"), pybind11::arg("clearPreviousContentsOutPDF"), pybind11::arg("centerPoint"), pybind11::arg("maxDistanceFromCenter"));
		cl.def("assign", (class mrpt::maps::CBeacon & (mrpt::maps::CBeacon::*)(const class mrpt::maps::CBeacon &)) &mrpt::maps::CBeacon::operator=, "C++: mrpt::maps::CBeacon::operator=(const class mrpt::maps::CBeacon &) --> class mrpt::maps::CBeacon &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::maps::CBeaconMap file:mrpt/maps/CBeaconMap.h line:43
		pybind11::class_<mrpt::maps::CBeaconMap, std::shared_ptr<mrpt::maps::CBeaconMap>, PyCallBack_mrpt_maps_CBeaconMap, mrpt::maps::CMetricMap> cl(M("mrpt::maps"), "CBeaconMap", "A class for storing a map of 3D probabilistic beacons, using a Montecarlo,\nGaussian, or Sum of Gaussians (SOG) representation (for range-only SLAM).\n \n  The individual beacons are defined as mrpt::maps::CBeacon objects.\n \n  When invoking CBeaconMap::insertObservation(), landmarks will be extracted\nand fused into the map.\n   The only currently supported observation type is\nmrpt::obs::CObservationBeaconRanges.\n   See insertionOptions and likelihoodOptions for parameters used when\ncreating and fusing beacon landmarks.\n \n   Use \"TInsertionOptions::insertAsMonteCarlo\" to select between 2 different\nbehaviors:\n		- Initial PDF of beacons: MonteCarlo, after convergence, pass to\nGaussians; or\n		- Initial PDF of beacons: SOG, after convergence, a single Gaussian.\n\n   Refer to the papers: []\n\n \n\n \n CMetricMap");
		cl.def( pybind11::init( [](){ return new mrpt::maps::CBeaconMap(); }, [](){ return new PyCallBack_mrpt_maps_CBeaconMap(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_maps_CBeaconMap const &o){ return new PyCallBack_mrpt_maps_CBeaconMap(o); } ) );
		cl.def( pybind11::init( [](mrpt::maps::CBeaconMap const &o){ return new mrpt::maps::CBeaconMap(o); } ) );
		cl.def_readwrite("likelihoodOptions", &mrpt::maps::CBeaconMap::likelihoodOptions);
		cl.def_readwrite("insertionOptions", &mrpt::maps::CBeaconMap::insertionOptions);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::maps::CBeaconMap::GetRuntimeClassIdStatic, "C++: mrpt::maps::CBeaconMap::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::maps::CBeaconMap::*)() const) &mrpt::maps::CBeaconMap::GetRuntimeClass, "C++: mrpt::maps::CBeaconMap::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::maps::CBeaconMap::*)() const) &mrpt::maps::CBeaconMap::clone, "C++: mrpt::maps::CBeaconMap::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::maps::CBeaconMap::CreateObject, "C++: mrpt::maps::CBeaconMap::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("resize", (void (mrpt::maps::CBeaconMap::*)(size_t)) &mrpt::maps::CBeaconMap::resize, "Resize the number of SOG modes \n\nC++: mrpt::maps::CBeaconMap::resize(size_t) --> void", pybind11::arg("N"));
		cl.def("asString", (std::string (mrpt::maps::CBeaconMap::*)() const) &mrpt::maps::CBeaconMap::asString, "Returns a short description of the map. \n\nC++: mrpt::maps::CBeaconMap::asString() const --> std::string");
		cl.def("__getitem__", (class mrpt::maps::CBeacon & (mrpt::maps::CBeaconMap::*)(size_t)) &mrpt::maps::CBeaconMap::operator[], "Access to individual beacons \n\nC++: mrpt::maps::CBeaconMap::operator[](size_t) --> class mrpt::maps::CBeacon &", pybind11::return_value_policy::automatic, pybind11::arg("i"));
		cl.def("get", (class mrpt::maps::CBeacon & (mrpt::maps::CBeaconMap::*)(size_t)) &mrpt::maps::CBeaconMap::get, "Access to individual beacons \n\nC++: mrpt::maps::CBeaconMap::get(size_t) --> class mrpt::maps::CBeacon &", pybind11::return_value_policy::automatic, pybind11::arg("i"));
		cl.def("push_back", (void (mrpt::maps::CBeaconMap::*)(const class mrpt::maps::CBeacon &)) &mrpt::maps::CBeaconMap::push_back, "Inserts a copy of the given mode into the SOG \n\nC++: mrpt::maps::CBeaconMap::push_back(const class mrpt::maps::CBeacon &) --> void", pybind11::arg("m"));
		cl.def("compute3DMatchingRatio", (float (mrpt::maps::CBeaconMap::*)(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose3D &, const struct mrpt::maps::TMatchingRatioParams &) const) &mrpt::maps::CBeaconMap::compute3DMatchingRatio, "C++: mrpt::maps::CBeaconMap::compute3DMatchingRatio(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose3D &, const struct mrpt::maps::TMatchingRatioParams &) const --> float", pybind11::arg("otherMap"), pybind11::arg("otherMapPose"), pybind11::arg("params"));
		cl.def("saveToMATLABScript3D", [](mrpt::maps::CBeaconMap const &o, const std::string & a0) -> bool { return o.saveToMATLABScript3D(a0); }, "", pybind11::arg("file"));
		cl.def("saveToMATLABScript3D", [](mrpt::maps::CBeaconMap const &o, const std::string & a0, const char * a1) -> bool { return o.saveToMATLABScript3D(a0, a1); }, "", pybind11::arg("file"), pybind11::arg("style"));
		cl.def("saveToMATLABScript3D", (bool (mrpt::maps::CBeaconMap::*)(const std::string &, const char *, float) const) &mrpt::maps::CBeaconMap::saveToMATLABScript3D, "Save to a MATLAB script which displays 3D error ellipses for the map.\n	\n\n		The name of the file to save the script to.\n  \n\n	The MATLAB-like string for the style of the lines (see\n'help plot' in MATLAB for possibilities)\n  \n\n The ellipsoids will be drawn from the center to a given\nconfidence interval in [0,1], e.g. 2 sigmas=0.95 (default is 2std = 0.95\nconfidence intervals)\n\n  \n Returns false if any error occured, true elsewere.\n\nC++: mrpt::maps::CBeaconMap::saveToMATLABScript3D(const std::string &, const char *, float) const --> bool", pybind11::arg("file"), pybind11::arg("style"), pybind11::arg("confInterval"));
		cl.def("size", (size_t (mrpt::maps::CBeaconMap::*)() const) &mrpt::maps::CBeaconMap::size, "Returns the stored landmarks count.\n\nC++: mrpt::maps::CBeaconMap::size() const --> size_t");
		cl.def("determineMatching2D", (void (mrpt::maps::CBeaconMap::*)(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose2D &, class mrpt::tfest::TMatchingPairListTempl<float> &, const struct mrpt::maps::TMatchingParams &, struct mrpt::maps::TMatchingExtraResults &) const) &mrpt::maps::CBeaconMap::determineMatching2D, "C++: mrpt::maps::CBeaconMap::determineMatching2D(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose2D &, class mrpt::tfest::TMatchingPairListTempl<float> &, const struct mrpt::maps::TMatchingParams &, struct mrpt::maps::TMatchingExtraResults &) const --> void", pybind11::arg("otherMap"), pybind11::arg("otherMapPose"), pybind11::arg("correspondences"), pybind11::arg("params"), pybind11::arg("extraResults"));
		cl.def("changeCoordinatesReference", (void (mrpt::maps::CBeaconMap::*)(const class mrpt::poses::CPose3D &)) &mrpt::maps::CBeaconMap::changeCoordinatesReference, "Changes the reference system of the map to a given 3D pose.\n\nC++: mrpt::maps::CBeaconMap::changeCoordinatesReference(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newOrg"));
		cl.def("changeCoordinatesReference", (void (mrpt::maps::CBeaconMap::*)(const class mrpt::poses::CPose3D &, const class mrpt::maps::CBeaconMap *)) &mrpt::maps::CBeaconMap::changeCoordinatesReference, "Changes the reference system of the map \"otherMap\" and save the result\n in \"this\" map.\n\nC++: mrpt::maps::CBeaconMap::changeCoordinatesReference(const class mrpt::poses::CPose3D &, const class mrpt::maps::CBeaconMap *) --> void", pybind11::arg("newOrg"), pybind11::arg("otherMap"));
		cl.def("isEmpty", (bool (mrpt::maps::CBeaconMap::*)() const) &mrpt::maps::CBeaconMap::isEmpty, "Returns true if the map is empty/no observation has been inserted.\n\nC++: mrpt::maps::CBeaconMap::isEmpty() const --> bool");
		cl.def("simulateBeaconReadings", (void (mrpt::maps::CBeaconMap::*)(const class mrpt::poses::CPose3D &, const class mrpt::poses::CPoint3D &, class mrpt::obs::CObservationBeaconRanges &) const) &mrpt::maps::CBeaconMap::simulateBeaconReadings, "Simulates a reading toward each of the beacons in the landmarks map, if\n any.\n \n\n This robot pose is used to simulate the ranges to\n each beacon.\n \n\n The 3D position of the sensor on the\n robot\n \n\n The results will be stored here. NOTICE that the\n fields\n \"CObservationBeaconRanges::minSensorDistance\",\"CObservationBeaconRanges::maxSensorDistance\"\n and \"CObservationBeaconRanges::stdError\" MUST BE FILLED OUT before\n calling this function.\n An observation will be generated for each beacon in the map, but notice\n that some of them may be missed if out of the sensor maximum range.\n\nC++: mrpt::maps::CBeaconMap::simulateBeaconReadings(const class mrpt::poses::CPose3D &, const class mrpt::poses::CPoint3D &, class mrpt::obs::CObservationBeaconRanges &) const --> void", pybind11::arg("in_robotPose"), pybind11::arg("in_sensorLocationOnRobot"), pybind11::arg("out_Observations"));
		cl.def("saveMetricMapRepresentationToFile", (void (mrpt::maps::CBeaconMap::*)(const std::string &) const) &mrpt::maps::CBeaconMap::saveMetricMapRepresentationToFile, "This virtual method saves the map to a file \"filNamePrefix\"+<\nsome_file_extension >, as an image or in any other applicable way (Notice\nthat other methods to save the map may be implemented in classes\nimplementing this virtual interface).\n  In the case of this class, these files are generated:\n		- \"filNamePrefix\"+\"_3D.m\": A script for MATLAB for drawing landmarks\nas\n3D ellipses.\n		- \"filNamePrefix\"+\"_3D.3DScene\": A 3D scene with a \"ground plane\ngrid\"\nand the set of ellipsoids in 3D.\n		- \"filNamePrefix\"+\"_covs.m\": A textual representation (see\nsaveToTextFile)\n\nC++: mrpt::maps::CBeaconMap::saveMetricMapRepresentationToFile(const std::string &) const --> void", pybind11::arg("filNamePrefix"));
		cl.def("saveToTextFile", (void (mrpt::maps::CBeaconMap::*)(const std::string &) const) &mrpt::maps::CBeaconMap::saveToTextFile, "Save a text file with a row per beacon, containing this 11 elements:\n  - X Y Z: Mean values\n  - VX VY VZ: Variances of each dimension (C11, C22, C33)\n  - DET2D DET3D: Determinant of the 2D and 3D covariance matrixes.\n  - C12, C13, C23: Cross covariances\n\nC++: mrpt::maps::CBeaconMap::saveToTextFile(const std::string &) const --> void", pybind11::arg("fil"));
		cl.def("getVisualizationInto", (void (mrpt::maps::CBeaconMap::*)(class mrpt::opengl::CSetOfObjects &) const) &mrpt::maps::CBeaconMap::getVisualizationInto, "Returns a 3D object representing the map. \n\nC++: mrpt::maps::CBeaconMap::getVisualizationInto(class mrpt::opengl::CSetOfObjects &) const --> void", pybind11::arg("outObj"));
		cl.def("getBeaconByID", (class mrpt::maps::CBeacon * (mrpt::maps::CBeaconMap::*)(int64_t)) &mrpt::maps::CBeaconMap::getBeaconByID, "Returns a pointer to the beacon with the given ID, or nullptr if it does\n not exist. \n\nC++: mrpt::maps::CBeaconMap::getBeaconByID(int64_t) --> class mrpt::maps::CBeacon *", pybind11::return_value_policy::automatic, pybind11::arg("id"));
		cl.def("assign", (class mrpt::maps::CBeaconMap & (mrpt::maps::CBeaconMap::*)(const class mrpt::maps::CBeaconMap &)) &mrpt::maps::CBeaconMap::operator=, "C++: mrpt::maps::CBeaconMap::operator=(const class mrpt::maps::CBeaconMap &) --> class mrpt::maps::CBeaconMap &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::maps::CBeaconMap::TLikelihoodOptions file:mrpt/maps/CBeaconMap.h line:112
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CBeaconMap::TLikelihoodOptions, std::shared_ptr<mrpt::maps::CBeaconMap::TLikelihoodOptions>, PyCallBack_mrpt_maps_CBeaconMap_TLikelihoodOptions, mrpt::config::CLoadableOptions> cl(enclosing_class, "TLikelihoodOptions", "With this struct options are provided to the likelihood computations ");
			cl.def( pybind11::init( [](){ return new mrpt::maps::CBeaconMap::TLikelihoodOptions(); }, [](){ return new PyCallBack_mrpt_maps_CBeaconMap_TLikelihoodOptions(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_maps_CBeaconMap_TLikelihoodOptions const &o){ return new PyCallBack_mrpt_maps_CBeaconMap_TLikelihoodOptions(o); } ) );
			cl.def( pybind11::init( [](mrpt::maps::CBeaconMap::TLikelihoodOptions const &o){ return new mrpt::maps::CBeaconMap::TLikelihoodOptions(o); } ) );
			cl.def_readwrite("rangeStd", &mrpt::maps::CBeaconMap::TLikelihoodOptions::rangeStd);
			cl.def("loadFromConfigFile", (void (mrpt::maps::CBeaconMap::TLikelihoodOptions::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::maps::CBeaconMap::TLikelihoodOptions::loadFromConfigFile, "C++: mrpt::maps::CBeaconMap::TLikelihoodOptions::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
			cl.def("assign", (struct mrpt::maps::CBeaconMap::TLikelihoodOptions & (mrpt::maps::CBeaconMap::TLikelihoodOptions::*)(const struct mrpt::maps::CBeaconMap::TLikelihoodOptions &)) &mrpt::maps::CBeaconMap::TLikelihoodOptions::operator=, "C++: mrpt::maps::CBeaconMap::TLikelihoodOptions::operator=(const struct mrpt::maps::CBeaconMap::TLikelihoodOptions &) --> struct mrpt::maps::CBeaconMap::TLikelihoodOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::maps::CBeaconMap::TInsertionOptions file:mrpt/maps/CBeaconMap.h line:129
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CBeaconMap::TInsertionOptions, std::shared_ptr<mrpt::maps::CBeaconMap::TInsertionOptions>, PyCallBack_mrpt_maps_CBeaconMap_TInsertionOptions, mrpt::config::CLoadableOptions> cl(enclosing_class, "TInsertionOptions", "This struct contains data for choosing the method by which new beacons\n are inserted in the map.");
			cl.def( pybind11::init( [](){ return new mrpt::maps::CBeaconMap::TInsertionOptions(); }, [](){ return new PyCallBack_mrpt_maps_CBeaconMap_TInsertionOptions(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_maps_CBeaconMap_TInsertionOptions const &o){ return new PyCallBack_mrpt_maps_CBeaconMap_TInsertionOptions(o); } ) );
			cl.def( pybind11::init( [](mrpt::maps::CBeaconMap::TInsertionOptions const &o){ return new mrpt::maps::CBeaconMap::TInsertionOptions(o); } ) );
			cl.def_readwrite("insertAsMonteCarlo", &mrpt::maps::CBeaconMap::TInsertionOptions::insertAsMonteCarlo);
			cl.def_readwrite("maxElevation_deg", &mrpt::maps::CBeaconMap::TInsertionOptions::maxElevation_deg);
			cl.def_readwrite("minElevation_deg", &mrpt::maps::CBeaconMap::TInsertionOptions::minElevation_deg);
			cl.def_readwrite("MC_numSamplesPerMeter", &mrpt::maps::CBeaconMap::TInsertionOptions::MC_numSamplesPerMeter);
			cl.def_readwrite("MC_maxStdToGauss", &mrpt::maps::CBeaconMap::TInsertionOptions::MC_maxStdToGauss);
			cl.def_readwrite("MC_thresholdNegligible", &mrpt::maps::CBeaconMap::TInsertionOptions::MC_thresholdNegligible);
			cl.def_readwrite("MC_performResampling", &mrpt::maps::CBeaconMap::TInsertionOptions::MC_performResampling);
			cl.def_readwrite("MC_afterResamplingNoise", &mrpt::maps::CBeaconMap::TInsertionOptions::MC_afterResamplingNoise);
			cl.def_readwrite("SOG_thresholdNegligible", &mrpt::maps::CBeaconMap::TInsertionOptions::SOG_thresholdNegligible);
			cl.def_readwrite("SOG_maxDistBetweenGaussians", &mrpt::maps::CBeaconMap::TInsertionOptions::SOG_maxDistBetweenGaussians);
			cl.def_readwrite("SOG_separationConstant", &mrpt::maps::CBeaconMap::TInsertionOptions::SOG_separationConstant);
			cl.def("loadFromConfigFile", (void (mrpt::maps::CBeaconMap::TInsertionOptions::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::maps::CBeaconMap::TInsertionOptions::loadFromConfigFile, "Initilization of default parameters \n\nC++: mrpt::maps::CBeaconMap::TInsertionOptions::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
			cl.def("assign", (struct mrpt::maps::CBeaconMap::TInsertionOptions & (mrpt::maps::CBeaconMap::TInsertionOptions::*)(const struct mrpt::maps::CBeaconMap::TInsertionOptions &)) &mrpt::maps::CBeaconMap::TInsertionOptions::operator=, "C++: mrpt::maps::CBeaconMap::TInsertionOptions::operator=(const struct mrpt::maps::CBeaconMap::TInsertionOptions &) --> struct mrpt::maps::CBeaconMap::TInsertionOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::maps::CBeaconMap::TMapDefinitionBase file: line:74
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CBeaconMap::TMapDefinitionBase, std::shared_ptr<mrpt::maps::CBeaconMap::TMapDefinitionBase>> cl(enclosing_class, "TMapDefinitionBase", "");
		}

		{ // mrpt::maps::CBeaconMap::TMapDefinition file: line:78
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CBeaconMap::TMapDefinition, std::shared_ptr<mrpt::maps::CBeaconMap::TMapDefinition>, PyCallBack_mrpt_maps_CBeaconMap_TMapDefinition, mrpt::maps::CBeaconMap::TMapDefinitionBase> cl(enclosing_class, "TMapDefinition", "");
			cl.def( pybind11::init( [](){ return new mrpt::maps::CBeaconMap::TMapDefinition(); }, [](){ return new PyCallBack_mrpt_maps_CBeaconMap_TMapDefinition(); } ) );
			cl.def_readwrite("insertionOpts", &mrpt::maps::CBeaconMap::TMapDefinition::insertionOpts);
			cl.def_readwrite("likelihoodOpts", &mrpt::maps::CBeaconMap::TMapDefinition::likelihoodOpts);
		}

	}
}
