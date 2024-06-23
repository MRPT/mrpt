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
#include <mrpt/img/CCanvas.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/img/TColor.h>
#include <mrpt/img/TPixelCoord.h>
#include <mrpt/io/CStream.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/CRandomFieldGridMap2D.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CWirelessPowerGridMap2D.h>
#include <mrpt/maps/TMetricMapInitializer.h>
#include <mrpt/maps/metric_map_types.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/MatrixVectorBase.h>
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
#include <mrpt/obs/CSensoryFrame.h>
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

// mrpt::maps::CWirelessPowerGridMap2D file:mrpt/maps/CWirelessPowerGridMap2D.h line:32
struct PyCallBack_mrpt_maps_CWirelessPowerGridMap2D : public mrpt::maps::CWirelessPowerGridMap2D {
	using mrpt::maps::CWirelessPowerGridMap2D::CWirelessPowerGridMap2D;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CWirelessPowerGridMap2D *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CWirelessPowerGridMap2D::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CWirelessPowerGridMap2D *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CWirelessPowerGridMap2D::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CWirelessPowerGridMap2D *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CWirelessPowerGridMap2D::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CWirelessPowerGridMap2D *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CWirelessPowerGridMap2D::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CWirelessPowerGridMap2D *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CWirelessPowerGridMap2D::serializeFrom(a0, a1);
	}
	void getVisualizationInto(class mrpt::opengl::CSetOfObjects & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CWirelessPowerGridMap2D *>(this), "getVisualizationInto");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CWirelessPowerGridMap2D::getVisualizationInto(a0);
	}
	struct mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon * getCommonInsertOptions() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CWirelessPowerGridMap2D *>(this), "getCommonInsertOptions");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon *>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon *> caster;
				return pybind11::detail::cast_ref<struct mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon *>(std::move(o));
		}
		return CWirelessPowerGridMap2D::getCommonInsertOptions();
	}
	void internal_clear() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CWirelessPowerGridMap2D *>(this), "internal_clear");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CWirelessPowerGridMap2D::internal_clear();
	}
	double internal_computeObservationLikelihood(const class mrpt::obs::CObservation & a0, const class mrpt::poses::CPose3D & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CWirelessPowerGridMap2D *>(this), "internal_computeObservationLikelihood");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CWirelessPowerGridMap2D::internal_computeObservationLikelihood(a0, a1);
	}
	float cell2float(const struct mrpt::maps::TRandomFieldCell & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CWirelessPowerGridMap2D *>(this), "cell2float");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<float>::value) {
				static pybind11::detail::override_caster_t<float> caster;
				return pybind11::detail::cast_ref<float>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<float>(std::move(o));
		}
		return CRandomFieldGridMap2D::cell2float(a0);
	}
	std::string asString() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CWirelessPowerGridMap2D *>(this), "asString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CRandomFieldGridMap2D::asString();
	}
	bool isEmpty() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CWirelessPowerGridMap2D *>(this), "isEmpty");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CRandomFieldGridMap2D::isEmpty();
	}
	void saveAsBitmapFile(const std::string & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CWirelessPowerGridMap2D *>(this), "saveAsBitmapFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRandomFieldGridMap2D::saveAsBitmapFile(a0);
	}
	void getAsBitmapFile(class mrpt::img::CImage & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CWirelessPowerGridMap2D *>(this), "getAsBitmapFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRandomFieldGridMap2D::getAsBitmapFile(a0);
	}
	void getAsMatrix(class mrpt::math::CMatrixDynamic<double> & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CWirelessPowerGridMap2D *>(this), "getAsMatrix");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRandomFieldGridMap2D::getAsMatrix(a0);
	}
	void resize(double a0, double a1, double a2, double a3, const struct mrpt::maps::TRandomFieldCell & a4, double a5) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CWirelessPowerGridMap2D *>(this), "resize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4, a5);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRandomFieldGridMap2D::resize(a0, a1, a2, a3, a4, a5);
	}
	void setSize(const double a0, const double a1, const double a2, const double a3, const double a4, const struct mrpt::maps::TRandomFieldCell * a5) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CWirelessPowerGridMap2D *>(this), "setSize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4, a5);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRandomFieldGridMap2D::setSize(a0, a1, a2, a3, a4, a5);
	}
	float compute3DMatchingRatio(const class mrpt::maps::CMetricMap * a0, const class mrpt::poses::CPose3D & a1, const struct mrpt::maps::TMatchingRatioParams & a2) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CWirelessPowerGridMap2D *>(this), "compute3DMatchingRatio");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<float>::value) {
				static pybind11::detail::override_caster_t<float> caster;
				return pybind11::detail::cast_ref<float>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<float>(std::move(o));
		}
		return CRandomFieldGridMap2D::compute3DMatchingRatio(a0, a1, a2);
	}
	void saveMetricMapRepresentationToFile(const std::string & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CWirelessPowerGridMap2D *>(this), "saveMetricMapRepresentationToFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRandomFieldGridMap2D::saveMetricMapRepresentationToFile(a0);
	}
	void saveAsMatlab3DGraph(const std::string & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CWirelessPowerGridMap2D *>(this), "saveAsMatlab3DGraph");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRandomFieldGridMap2D::saveAsMatlab3DGraph(a0);
	}
	void getAs3DObject(class mrpt::opengl::CSetOfObjects & a0, class mrpt::opengl::CSetOfObjects & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CWirelessPowerGridMap2D *>(this), "getAs3DObject");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRandomFieldGridMap2D::getAs3DObject(a0, a1);
	}
	void predictMeasurement(const double a0, const double a1, double & a2, double & a3, bool a4, const enum mrpt::maps::CRandomFieldGridMap2D::TGridInterpolationMethod a5) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CWirelessPowerGridMap2D *>(this), "predictMeasurement");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4, a5);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRandomFieldGridMap2D::predictMeasurement(a0, a1, a2, a3, a4, a5);
	}
	struct mrpt::math::TBoundingBox_<float> boundingBox() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CWirelessPowerGridMap2D *>(this), "boundingBox");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CWirelessPowerGridMap2D *>(this), "canComputeObservationLikelihood");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CWirelessPowerGridMap2D *>(this), "determineMatching2D");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CWirelessPowerGridMap2D *>(this), "determineMatching3D");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CWirelessPowerGridMap2D *>(this), "auxParticleFilterCleanUp");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CWirelessPowerGridMap2D *>(this), "squareDistanceToClosestCorrespondence");
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

// mrpt::maps::CWirelessPowerGridMap2D::TInsertionOptions file:mrpt/maps/CWirelessPowerGridMap2D.h line:50
struct PyCallBack_mrpt_maps_CWirelessPowerGridMap2D_TInsertionOptions : public mrpt::maps::CWirelessPowerGridMap2D::TInsertionOptions {
	using mrpt::maps::CWirelessPowerGridMap2D::TInsertionOptions::TInsertionOptions;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CWirelessPowerGridMap2D::TInsertionOptions *>(this), "loadFromConfigFile");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CWirelessPowerGridMap2D::TInsertionOptions *>(this), "saveToConfigFile");
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

// mrpt::maps::CWirelessPowerGridMap2D::TMapDefinition file: line:78
struct PyCallBack_mrpt_maps_CWirelessPowerGridMap2D_TMapDefinition : public mrpt::maps::CWirelessPowerGridMap2D::TMapDefinition {
	using mrpt::maps::CWirelessPowerGridMap2D::TMapDefinition::TMapDefinition;

	void loadFromConfigFile_map_specific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CWirelessPowerGridMap2D::TMapDefinition *>(this), "loadFromConfigFile_map_specific");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CWirelessPowerGridMap2D::TMapDefinition *>(this), "loadFromConfigFile");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CWirelessPowerGridMap2D::TMapDefinition *>(this), "saveToConfigFile");
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

void bind_mrpt_maps_CWirelessPowerGridMap2D(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::maps::CWirelessPowerGridMap2D file:mrpt/maps/CWirelessPowerGridMap2D.h line:32
		pybind11::class_<mrpt::maps::CWirelessPowerGridMap2D, std::shared_ptr<mrpt::maps::CWirelessPowerGridMap2D>, PyCallBack_mrpt_maps_CWirelessPowerGridMap2D, mrpt::maps::CRandomFieldGridMap2D> cl(M("mrpt::maps"), "CWirelessPowerGridMap2D", "CWirelessPowerGridMap2D represents a PDF of wifi concentrations over a 2D\n area.\n\n  There are a number of methods available to build the wifi grid-map,\n depending on the value of\n    \"TMapRepresentation maptype\" passed in the constructor (see\n CRandomFieldGridMap2D for a discussion).\n\n Update the map with insertIndividualReading() or insertObservation()\n\n \n mrpt::maps::CRandomFieldGridMap2D, mrpt::maps::CMetricMap,\n mrpt::containers::CDynamicGrid, The application icp-slam,\n mrpt::maps::CMultiMetricMap\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::maps::CWirelessPowerGridMap2D(); }, [](){ return new PyCallBack_mrpt_maps_CWirelessPowerGridMap2D(); } ), "doc");
		cl.def( pybind11::init( [](enum mrpt::maps::CRandomFieldGridMap2D::TMapRepresentation const & a0){ return new mrpt::maps::CWirelessPowerGridMap2D(a0); }, [](enum mrpt::maps::CRandomFieldGridMap2D::TMapRepresentation const & a0){ return new PyCallBack_mrpt_maps_CWirelessPowerGridMap2D(a0); } ), "doc");
		cl.def( pybind11::init( [](enum mrpt::maps::CRandomFieldGridMap2D::TMapRepresentation const & a0, double const & a1){ return new mrpt::maps::CWirelessPowerGridMap2D(a0, a1); }, [](enum mrpt::maps::CRandomFieldGridMap2D::TMapRepresentation const & a0, double const & a1){ return new PyCallBack_mrpt_maps_CWirelessPowerGridMap2D(a0, a1); } ), "doc");
		cl.def( pybind11::init( [](enum mrpt::maps::CRandomFieldGridMap2D::TMapRepresentation const & a0, double const & a1, double const & a2){ return new mrpt::maps::CWirelessPowerGridMap2D(a0, a1, a2); }, [](enum mrpt::maps::CRandomFieldGridMap2D::TMapRepresentation const & a0, double const & a1, double const & a2){ return new PyCallBack_mrpt_maps_CWirelessPowerGridMap2D(a0, a1, a2); } ), "doc");
		cl.def( pybind11::init( [](enum mrpt::maps::CRandomFieldGridMap2D::TMapRepresentation const & a0, double const & a1, double const & a2, double const & a3){ return new mrpt::maps::CWirelessPowerGridMap2D(a0, a1, a2, a3); }, [](enum mrpt::maps::CRandomFieldGridMap2D::TMapRepresentation const & a0, double const & a1, double const & a2, double const & a3){ return new PyCallBack_mrpt_maps_CWirelessPowerGridMap2D(a0, a1, a2, a3); } ), "doc");
		cl.def( pybind11::init( [](enum mrpt::maps::CRandomFieldGridMap2D::TMapRepresentation const & a0, double const & a1, double const & a2, double const & a3, double const & a4){ return new mrpt::maps::CWirelessPowerGridMap2D(a0, a1, a2, a3, a4); }, [](enum mrpt::maps::CRandomFieldGridMap2D::TMapRepresentation const & a0, double const & a1, double const & a2, double const & a3, double const & a4){ return new PyCallBack_mrpt_maps_CWirelessPowerGridMap2D(a0, a1, a2, a3, a4); } ), "doc");
		cl.def( pybind11::init<enum mrpt::maps::CRandomFieldGridMap2D::TMapRepresentation, double, double, double, double, double>(), pybind11::arg("mapType"), pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("resolution") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_maps_CWirelessPowerGridMap2D const &o){ return new PyCallBack_mrpt_maps_CWirelessPowerGridMap2D(o); } ) );
		cl.def( pybind11::init( [](mrpt::maps::CWirelessPowerGridMap2D const &o){ return new mrpt::maps::CWirelessPowerGridMap2D(o); } ) );
		cl.def_readwrite("insertionOptions", &mrpt::maps::CWirelessPowerGridMap2D::insertionOptions);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::maps::CWirelessPowerGridMap2D::GetRuntimeClassIdStatic, "C++: mrpt::maps::CWirelessPowerGridMap2D::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::maps::CWirelessPowerGridMap2D::*)() const) &mrpt::maps::CWirelessPowerGridMap2D::GetRuntimeClass, "C++: mrpt::maps::CWirelessPowerGridMap2D::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::maps::CWirelessPowerGridMap2D::*)() const) &mrpt::maps::CWirelessPowerGridMap2D::clone, "C++: mrpt::maps::CWirelessPowerGridMap2D::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::maps::CWirelessPowerGridMap2D::CreateObject, "C++: mrpt::maps::CWirelessPowerGridMap2D::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("getVisualizationInto", (void (mrpt::maps::CWirelessPowerGridMap2D::*)(class mrpt::opengl::CSetOfObjects &) const) &mrpt::maps::CWirelessPowerGridMap2D::getVisualizationInto, "Returns a 3D object representing the map  \n\nC++: mrpt::maps::CWirelessPowerGridMap2D::getVisualizationInto(class mrpt::opengl::CSetOfObjects &) const --> void", pybind11::arg("outObj"));
		cl.def("assign", (class mrpt::maps::CWirelessPowerGridMap2D & (mrpt::maps::CWirelessPowerGridMap2D::*)(const class mrpt::maps::CWirelessPowerGridMap2D &)) &mrpt::maps::CWirelessPowerGridMap2D::operator=, "C++: mrpt::maps::CWirelessPowerGridMap2D::operator=(const class mrpt::maps::CWirelessPowerGridMap2D &) --> class mrpt::maps::CWirelessPowerGridMap2D &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::maps::CWirelessPowerGridMap2D::TInsertionOptions file:mrpt/maps/CWirelessPowerGridMap2D.h line:50
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CWirelessPowerGridMap2D::TInsertionOptions, std::shared_ptr<mrpt::maps::CWirelessPowerGridMap2D::TInsertionOptions>, PyCallBack_mrpt_maps_CWirelessPowerGridMap2D_TInsertionOptions, mrpt::config::CLoadableOptions, mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon> cl(enclosing_class, "TInsertionOptions", "Parameters related with inserting observations into the map:");
			cl.def( pybind11::init( [](){ return new mrpt::maps::CWirelessPowerGridMap2D::TInsertionOptions(); }, [](){ return new PyCallBack_mrpt_maps_CWirelessPowerGridMap2D_TInsertionOptions(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_maps_CWirelessPowerGridMap2D_TInsertionOptions const &o){ return new PyCallBack_mrpt_maps_CWirelessPowerGridMap2D_TInsertionOptions(o); } ) );
			cl.def( pybind11::init( [](mrpt::maps::CWirelessPowerGridMap2D::TInsertionOptions const &o){ return new mrpt::maps::CWirelessPowerGridMap2D::TInsertionOptions(o); } ) );
			cl.def("loadFromConfigFile", (void (mrpt::maps::CWirelessPowerGridMap2D::TInsertionOptions::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::maps::CWirelessPowerGridMap2D::TInsertionOptions::loadFromConfigFile, "C++: mrpt::maps::CWirelessPowerGridMap2D::TInsertionOptions::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
			cl.def("assign", (struct mrpt::maps::CWirelessPowerGridMap2D::TInsertionOptions & (mrpt::maps::CWirelessPowerGridMap2D::TInsertionOptions::*)(const struct mrpt::maps::CWirelessPowerGridMap2D::TInsertionOptions &)) &mrpt::maps::CWirelessPowerGridMap2D::TInsertionOptions::operator=, "C++: mrpt::maps::CWirelessPowerGridMap2D::TInsertionOptions::operator=(const struct mrpt::maps::CWirelessPowerGridMap2D::TInsertionOptions &) --> struct mrpt::maps::CWirelessPowerGridMap2D::TInsertionOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::maps::CWirelessPowerGridMap2D::TMapDefinitionBase file: line:74
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CWirelessPowerGridMap2D::TMapDefinitionBase, std::shared_ptr<mrpt::maps::CWirelessPowerGridMap2D::TMapDefinitionBase>> cl(enclosing_class, "TMapDefinitionBase", "");
		}

		{ // mrpt::maps::CWirelessPowerGridMap2D::TMapDefinition file: line:78
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CWirelessPowerGridMap2D::TMapDefinition, std::shared_ptr<mrpt::maps::CWirelessPowerGridMap2D::TMapDefinition>, PyCallBack_mrpt_maps_CWirelessPowerGridMap2D_TMapDefinition, mrpt::maps::CWirelessPowerGridMap2D::TMapDefinitionBase> cl(enclosing_class, "TMapDefinition", "");
			cl.def( pybind11::init( [](){ return new mrpt::maps::CWirelessPowerGridMap2D::TMapDefinition(); }, [](){ return new PyCallBack_mrpt_maps_CWirelessPowerGridMap2D_TMapDefinition(); } ) );
			cl.def_readwrite("min_x", &mrpt::maps::CWirelessPowerGridMap2D::TMapDefinition::min_x);
			cl.def_readwrite("max_x", &mrpt::maps::CWirelessPowerGridMap2D::TMapDefinition::max_x);
			cl.def_readwrite("min_y", &mrpt::maps::CWirelessPowerGridMap2D::TMapDefinition::min_y);
			cl.def_readwrite("max_y", &mrpt::maps::CWirelessPowerGridMap2D::TMapDefinition::max_y);
			cl.def_readwrite("resolution", &mrpt::maps::CWirelessPowerGridMap2D::TMapDefinition::resolution);
			cl.def_readwrite("mapType", &mrpt::maps::CWirelessPowerGridMap2D::TMapDefinition::mapType);
			cl.def_readwrite("insertionOpts", &mrpt::maps::CWirelessPowerGridMap2D::TMapDefinition::insertionOpts);
		}

	}
}
