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
#include <mrpt/maps/CHeightGridMap2D_Base.h>
#include <mrpt/maps/CHeightGridMap2D_MRF.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/CRandomFieldGridMap2D.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
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

// mrpt::maps::CHeightGridMap2D_MRF file:mrpt/maps/CHeightGridMap2D_MRF.h line:33
struct PyCallBack_mrpt_maps_CHeightGridMap2D_MRF : public mrpt::maps::CHeightGridMap2D_MRF {
	using mrpt::maps::CHeightGridMap2D_MRF::CHeightGridMap2D_MRF;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CHeightGridMap2D_MRF::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CHeightGridMap2D_MRF::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CHeightGridMap2D_MRF::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CHeightGridMap2D_MRF::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CHeightGridMap2D_MRF::serializeFrom(a0, a1);
	}
	void getVisualizationInto(class mrpt::opengl::CSetOfObjects & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF *>(this), "getVisualizationInto");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CHeightGridMap2D_MRF::getVisualizationInto(a0);
	}
	void getAs3DObject(class mrpt::opengl::CSetOfObjects & a0, class mrpt::opengl::CSetOfObjects & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF *>(this), "getAs3DObject");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CHeightGridMap2D_MRF::getAs3DObject(a0, a1);
	}
	bool insertIndividualPoint(const double a0, const double a1, const double a2, const struct mrpt::maps::CHeightGridMap2D_Base::TPointInsertParams & a3) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF *>(this), "insertIndividualPoint");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CHeightGridMap2D_MRF::insertIndividualPoint(a0, a1, a2, a3);
	}
	double dem_get_resolution() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF *>(this), "dem_get_resolution");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CHeightGridMap2D_MRF::dem_get_resolution();
	}
	size_t dem_get_size_x() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF *>(this), "dem_get_size_x");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CHeightGridMap2D_MRF::dem_get_size_x();
	}
	size_t dem_get_size_y() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF *>(this), "dem_get_size_y");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CHeightGridMap2D_MRF::dem_get_size_y();
	}
	bool dem_get_z_by_cell(size_t a0, size_t a1, double & a2) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF *>(this), "dem_get_z_by_cell");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CHeightGridMap2D_MRF::dem_get_z_by_cell(a0, a1, a2);
	}
	bool dem_get_z(const double a0, const double a1, double & a2) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF *>(this), "dem_get_z");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CHeightGridMap2D_MRF::dem_get_z(a0, a1, a2);
	}
	void dem_update_map() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF *>(this), "dem_update_map");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CHeightGridMap2D_MRF::dem_update_map();
	}
	struct mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon * getCommonInsertOptions() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF *>(this), "getCommonInsertOptions");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon *>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon *> caster;
				return pybind11::detail::cast_ref<struct mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon *>(std::move(o));
		}
		return CHeightGridMap2D_MRF::getCommonInsertOptions();
	}
	void internal_clear() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF *>(this), "internal_clear");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CHeightGridMap2D_MRF::internal_clear();
	}
	double internal_computeObservationLikelihood(const class mrpt::obs::CObservation & a0, const class mrpt::poses::CPose3D & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF *>(this), "internal_computeObservationLikelihood");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CHeightGridMap2D_MRF::internal_computeObservationLikelihood(a0, a1);
	}
	float cell2float(const struct mrpt::maps::TRandomFieldCell & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF *>(this), "cell2float");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF *>(this), "asString");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF *>(this), "isEmpty");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF *>(this), "saveAsBitmapFile");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF *>(this), "getAsBitmapFile");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF *>(this), "getAsMatrix");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF *>(this), "resize");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF *>(this), "setSize");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF *>(this), "compute3DMatchingRatio");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF *>(this), "saveMetricMapRepresentationToFile");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF *>(this), "saveAsMatlab3DGraph");
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
	void predictMeasurement(const double a0, const double a1, double & a2, double & a3, bool a4, const enum mrpt::maps::CRandomFieldGridMap2D::TGridInterpolationMethod a5) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF *>(this), "predictMeasurement");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF *>(this), "boundingBox");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF *>(this), "canComputeObservationLikelihood");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF *>(this), "determineMatching2D");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF *>(this), "determineMatching3D");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF *>(this), "auxParticleFilterCleanUp");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF *>(this), "squareDistanceToClosestCorrespondence");
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

// mrpt::maps::CHeightGridMap2D_MRF::TInsertionOptions file:mrpt/maps/CHeightGridMap2D_MRF.h line:50
struct PyCallBack_mrpt_maps_CHeightGridMap2D_MRF_TInsertionOptions : public mrpt::maps::CHeightGridMap2D_MRF::TInsertionOptions {
	using mrpt::maps::CHeightGridMap2D_MRF::TInsertionOptions::TInsertionOptions;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF::TInsertionOptions *>(this), "loadFromConfigFile");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF::TInsertionOptions *>(this), "saveToConfigFile");
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

// mrpt::maps::CHeightGridMap2D_MRF::TMapDefinition file: line:78
struct PyCallBack_mrpt_maps_CHeightGridMap2D_MRF_TMapDefinition : public mrpt::maps::CHeightGridMap2D_MRF::TMapDefinition {
	using mrpt::maps::CHeightGridMap2D_MRF::TMapDefinition::TMapDefinition;

	void loadFromConfigFile_map_specific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF::TMapDefinition *>(this), "loadFromConfigFile_map_specific");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF::TMapDefinition *>(this), "loadFromConfigFile");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_MRF::TMapDefinition *>(this), "saveToConfigFile");
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

void bind_mrpt_maps_CHeightGridMap2D_MRF(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::maps::CHeightGridMap2D_MRF file:mrpt/maps/CHeightGridMap2D_MRF.h line:33
		pybind11::class_<mrpt::maps::CHeightGridMap2D_MRF, std::shared_ptr<mrpt::maps::CHeightGridMap2D_MRF>, PyCallBack_mrpt_maps_CHeightGridMap2D_MRF, mrpt::maps::CRandomFieldGridMap2D, mrpt::maps::CHeightGridMap2D_Base> cl(M("mrpt::maps"), "CHeightGridMap2D_MRF", "CHeightGridMap2D_MRF represents digital-elevation-model over a 2D area, with\n uncertainty, based on a Markov-Random-Field (MRF) estimator.\n\n  There are a number of methods available to build the gas grid-map,\n depending on the value of\n    \"TMapRepresentation maptype\" passed in the constructor (see base class\n mrpt::maps::CRandomFieldGridMap2D).\n\n Update the map with insertIndividualReading() or insertObservation()\n\n \n mrpt::maps::CRandomFieldGridMap2D, mrpt::maps::CMetricMap,\n mrpt::containers::CDynamicGrid, The application icp-slam,\n mrpt::maps::CMultiMetricMap\n \n\n New in MRPT 1.4.0\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::maps::CHeightGridMap2D_MRF(); }, [](){ return new PyCallBack_mrpt_maps_CHeightGridMap2D_MRF(); } ), "doc");
		cl.def( pybind11::init( [](enum mrpt::maps::CRandomFieldGridMap2D::TMapRepresentation const & a0){ return new mrpt::maps::CHeightGridMap2D_MRF(a0); }, [](enum mrpt::maps::CRandomFieldGridMap2D::TMapRepresentation const & a0){ return new PyCallBack_mrpt_maps_CHeightGridMap2D_MRF(a0); } ), "doc");
		cl.def( pybind11::init( [](enum mrpt::maps::CRandomFieldGridMap2D::TMapRepresentation const & a0, double const & a1){ return new mrpt::maps::CHeightGridMap2D_MRF(a0, a1); }, [](enum mrpt::maps::CRandomFieldGridMap2D::TMapRepresentation const & a0, double const & a1){ return new PyCallBack_mrpt_maps_CHeightGridMap2D_MRF(a0, a1); } ), "doc");
		cl.def( pybind11::init( [](enum mrpt::maps::CRandomFieldGridMap2D::TMapRepresentation const & a0, double const & a1, double const & a2){ return new mrpt::maps::CHeightGridMap2D_MRF(a0, a1, a2); }, [](enum mrpt::maps::CRandomFieldGridMap2D::TMapRepresentation const & a0, double const & a1, double const & a2){ return new PyCallBack_mrpt_maps_CHeightGridMap2D_MRF(a0, a1, a2); } ), "doc");
		cl.def( pybind11::init( [](enum mrpt::maps::CRandomFieldGridMap2D::TMapRepresentation const & a0, double const & a1, double const & a2, double const & a3){ return new mrpt::maps::CHeightGridMap2D_MRF(a0, a1, a2, a3); }, [](enum mrpt::maps::CRandomFieldGridMap2D::TMapRepresentation const & a0, double const & a1, double const & a2, double const & a3){ return new PyCallBack_mrpt_maps_CHeightGridMap2D_MRF(a0, a1, a2, a3); } ), "doc");
		cl.def( pybind11::init( [](enum mrpt::maps::CRandomFieldGridMap2D::TMapRepresentation const & a0, double const & a1, double const & a2, double const & a3, double const & a4){ return new mrpt::maps::CHeightGridMap2D_MRF(a0, a1, a2, a3, a4); }, [](enum mrpt::maps::CRandomFieldGridMap2D::TMapRepresentation const & a0, double const & a1, double const & a2, double const & a3, double const & a4){ return new PyCallBack_mrpt_maps_CHeightGridMap2D_MRF(a0, a1, a2, a3, a4); } ), "doc");
		cl.def( pybind11::init( [](enum mrpt::maps::CRandomFieldGridMap2D::TMapRepresentation const & a0, double const & a1, double const & a2, double const & a3, double const & a4, double const & a5){ return new mrpt::maps::CHeightGridMap2D_MRF(a0, a1, a2, a3, a4, a5); }, [](enum mrpt::maps::CRandomFieldGridMap2D::TMapRepresentation const & a0, double const & a1, double const & a2, double const & a3, double const & a4, double const & a5){ return new PyCallBack_mrpt_maps_CHeightGridMap2D_MRF(a0, a1, a2, a3, a4, a5); } ), "doc");
		cl.def( pybind11::init<enum mrpt::maps::CRandomFieldGridMap2D::TMapRepresentation, double, double, double, double, double, bool>(), pybind11::arg("mapType"), pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("resolution"), pybind11::arg("run_first_map_estimation_now") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_maps_CHeightGridMap2D_MRF const &o){ return new PyCallBack_mrpt_maps_CHeightGridMap2D_MRF(o); } ) );
		cl.def( pybind11::init( [](mrpt::maps::CHeightGridMap2D_MRF const &o){ return new mrpt::maps::CHeightGridMap2D_MRF(o); } ) );
		cl.def_readwrite("insertionOptions", &mrpt::maps::CHeightGridMap2D_MRF::insertionOptions);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::maps::CHeightGridMap2D_MRF::GetRuntimeClassIdStatic, "C++: mrpt::maps::CHeightGridMap2D_MRF::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::maps::CHeightGridMap2D_MRF::*)() const) &mrpt::maps::CHeightGridMap2D_MRF::GetRuntimeClass, "C++: mrpt::maps::CHeightGridMap2D_MRF::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::maps::CHeightGridMap2D_MRF::*)() const) &mrpt::maps::CHeightGridMap2D_MRF::clone, "C++: mrpt::maps::CHeightGridMap2D_MRF::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::maps::CHeightGridMap2D_MRF::CreateObject, "C++: mrpt::maps::CHeightGridMap2D_MRF::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("getVisualizationInto", (void (mrpt::maps::CHeightGridMap2D_MRF::*)(class mrpt::opengl::CSetOfObjects &) const) &mrpt::maps::CHeightGridMap2D_MRF::getVisualizationInto, "Returns a 3D object representing the map \n\nC++: mrpt::maps::CHeightGridMap2D_MRF::getVisualizationInto(class mrpt::opengl::CSetOfObjects &) const --> void", pybind11::arg("outObj"));
		cl.def("getAs3DObject", (void (mrpt::maps::CHeightGridMap2D_MRF::*)(class mrpt::opengl::CSetOfObjects &, class mrpt::opengl::CSetOfObjects &) const) &mrpt::maps::CHeightGridMap2D_MRF::getAs3DObject, "Returns two 3D objects representing the mean and variance maps \n\nC++: mrpt::maps::CHeightGridMap2D_MRF::getAs3DObject(class mrpt::opengl::CSetOfObjects &, class mrpt::opengl::CSetOfObjects &) const --> void", pybind11::arg("meanObj"), pybind11::arg("varObj"));
		cl.def("insertIndividualPoint", [](mrpt::maps::CHeightGridMap2D_MRF &o, const double & a0, const double & a1, const double & a2) -> bool { return o.insertIndividualPoint(a0, a1, a2); }, "", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("insertIndividualPoint", (bool (mrpt::maps::CHeightGridMap2D_MRF::*)(const double, const double, const double, const struct mrpt::maps::CHeightGridMap2D_Base::TPointInsertParams &)) &mrpt::maps::CHeightGridMap2D_MRF::insertIndividualPoint, "C++: mrpt::maps::CHeightGridMap2D_MRF::insertIndividualPoint(const double, const double, const double, const struct mrpt::maps::CHeightGridMap2D_Base::TPointInsertParams &) --> bool", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("params"));
		cl.def("dem_get_resolution", (double (mrpt::maps::CHeightGridMap2D_MRF::*)() const) &mrpt::maps::CHeightGridMap2D_MRF::dem_get_resolution, "C++: mrpt::maps::CHeightGridMap2D_MRF::dem_get_resolution() const --> double");
		cl.def("dem_get_size_x", (size_t (mrpt::maps::CHeightGridMap2D_MRF::*)() const) &mrpt::maps::CHeightGridMap2D_MRF::dem_get_size_x, "C++: mrpt::maps::CHeightGridMap2D_MRF::dem_get_size_x() const --> size_t");
		cl.def("dem_get_size_y", (size_t (mrpt::maps::CHeightGridMap2D_MRF::*)() const) &mrpt::maps::CHeightGridMap2D_MRF::dem_get_size_y, "C++: mrpt::maps::CHeightGridMap2D_MRF::dem_get_size_y() const --> size_t");
		cl.def("dem_get_z_by_cell", (bool (mrpt::maps::CHeightGridMap2D_MRF::*)(size_t, size_t, double &) const) &mrpt::maps::CHeightGridMap2D_MRF::dem_get_z_by_cell, "C++: mrpt::maps::CHeightGridMap2D_MRF::dem_get_z_by_cell(size_t, size_t, double &) const --> bool", pybind11::arg("cx"), pybind11::arg("cy"), pybind11::arg("z_out"));
		cl.def("dem_get_z", (bool (mrpt::maps::CHeightGridMap2D_MRF::*)(const double, const double, double &) const) &mrpt::maps::CHeightGridMap2D_MRF::dem_get_z, "C++: mrpt::maps::CHeightGridMap2D_MRF::dem_get_z(const double, const double, double &) const --> bool", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z_out"));
		cl.def("dem_update_map", (void (mrpt::maps::CHeightGridMap2D_MRF::*)()) &mrpt::maps::CHeightGridMap2D_MRF::dem_update_map, "C++: mrpt::maps::CHeightGridMap2D_MRF::dem_update_map() --> void");
		cl.def("getCommonInsertOptions", (struct mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon * (mrpt::maps::CHeightGridMap2D_MRF::*)()) &mrpt::maps::CHeightGridMap2D_MRF::getCommonInsertOptions, "Get the part of the options common to all CRandomFieldGridMap2D classes\n\nC++: mrpt::maps::CHeightGridMap2D_MRF::getCommonInsertOptions() --> struct mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon *", pybind11::return_value_policy::automatic);
		cl.def("internal_clear", (void (mrpt::maps::CHeightGridMap2D_MRF::*)()) &mrpt::maps::CHeightGridMap2D_MRF::internal_clear, "C++: mrpt::maps::CHeightGridMap2D_MRF::internal_clear() --> void");
		cl.def("internal_computeObservationLikelihood", (double (mrpt::maps::CHeightGridMap2D_MRF::*)(const class mrpt::obs::CObservation &, const class mrpt::poses::CPose3D &) const) &mrpt::maps::CHeightGridMap2D_MRF::internal_computeObservationLikelihood, "C++: mrpt::maps::CHeightGridMap2D_MRF::internal_computeObservationLikelihood(const class mrpt::obs::CObservation &, const class mrpt::poses::CPose3D &) const --> double", pybind11::arg("obs"), pybind11::arg("takenFrom"));
		cl.def("assign", (class mrpt::maps::CHeightGridMap2D_MRF & (mrpt::maps::CHeightGridMap2D_MRF::*)(const class mrpt::maps::CHeightGridMap2D_MRF &)) &mrpt::maps::CHeightGridMap2D_MRF::operator=, "C++: mrpt::maps::CHeightGridMap2D_MRF::operator=(const class mrpt::maps::CHeightGridMap2D_MRF &) --> class mrpt::maps::CHeightGridMap2D_MRF &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::maps::CHeightGridMap2D_MRF::TInsertionOptions file:mrpt/maps/CHeightGridMap2D_MRF.h line:50
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CHeightGridMap2D_MRF::TInsertionOptions, std::shared_ptr<mrpt::maps::CHeightGridMap2D_MRF::TInsertionOptions>, PyCallBack_mrpt_maps_CHeightGridMap2D_MRF_TInsertionOptions, mrpt::config::CLoadableOptions, mrpt::maps::CRandomFieldGridMap2D::TInsertionOptionsCommon> cl(enclosing_class, "TInsertionOptions", "Parameters related with inserting observations into the map ");
			cl.def( pybind11::init( [](){ return new mrpt::maps::CHeightGridMap2D_MRF::TInsertionOptions(); }, [](){ return new PyCallBack_mrpt_maps_CHeightGridMap2D_MRF_TInsertionOptions(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_maps_CHeightGridMap2D_MRF_TInsertionOptions const &o){ return new PyCallBack_mrpt_maps_CHeightGridMap2D_MRF_TInsertionOptions(o); } ) );
			cl.def( pybind11::init( [](mrpt::maps::CHeightGridMap2D_MRF::TInsertionOptions const &o){ return new mrpt::maps::CHeightGridMap2D_MRF::TInsertionOptions(o); } ) );
			cl.def("loadFromConfigFile", (void (mrpt::maps::CHeightGridMap2D_MRF::TInsertionOptions::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::maps::CHeightGridMap2D_MRF::TInsertionOptions::loadFromConfigFile, "C++: mrpt::maps::CHeightGridMap2D_MRF::TInsertionOptions::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
			cl.def("assign", (struct mrpt::maps::CHeightGridMap2D_MRF::TInsertionOptions & (mrpt::maps::CHeightGridMap2D_MRF::TInsertionOptions::*)(const struct mrpt::maps::CHeightGridMap2D_MRF::TInsertionOptions &)) &mrpt::maps::CHeightGridMap2D_MRF::TInsertionOptions::operator=, "C++: mrpt::maps::CHeightGridMap2D_MRF::TInsertionOptions::operator=(const struct mrpt::maps::CHeightGridMap2D_MRF::TInsertionOptions &) --> struct mrpt::maps::CHeightGridMap2D_MRF::TInsertionOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::maps::CHeightGridMap2D_MRF::TMapDefinitionBase file: line:74
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CHeightGridMap2D_MRF::TMapDefinitionBase, std::shared_ptr<mrpt::maps::CHeightGridMap2D_MRF::TMapDefinitionBase>> cl(enclosing_class, "TMapDefinitionBase", "");
		}

		{ // mrpt::maps::CHeightGridMap2D_MRF::TMapDefinition file: line:78
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CHeightGridMap2D_MRF::TMapDefinition, std::shared_ptr<mrpt::maps::CHeightGridMap2D_MRF::TMapDefinition>, PyCallBack_mrpt_maps_CHeightGridMap2D_MRF_TMapDefinition, mrpt::maps::CHeightGridMap2D_MRF::TMapDefinitionBase> cl(enclosing_class, "TMapDefinition", "");
			cl.def( pybind11::init( [](){ return new mrpt::maps::CHeightGridMap2D_MRF::TMapDefinition(); }, [](){ return new PyCallBack_mrpt_maps_CHeightGridMap2D_MRF_TMapDefinition(); } ) );
			cl.def_readwrite("run_map_estimation_at_ctor", &mrpt::maps::CHeightGridMap2D_MRF::TMapDefinition::run_map_estimation_at_ctor);
			cl.def_readwrite("min_x", &mrpt::maps::CHeightGridMap2D_MRF::TMapDefinition::min_x);
			cl.def_readwrite("max_x", &mrpt::maps::CHeightGridMap2D_MRF::TMapDefinition::max_x);
			cl.def_readwrite("min_y", &mrpt::maps::CHeightGridMap2D_MRF::TMapDefinition::min_y);
			cl.def_readwrite("max_y", &mrpt::maps::CHeightGridMap2D_MRF::TMapDefinition::max_y);
			cl.def_readwrite("resolution", &mrpt::maps::CHeightGridMap2D_MRF::TMapDefinition::resolution);
			cl.def_readwrite("mapType", &mrpt::maps::CHeightGridMap2D_MRF::TMapDefinition::mapType);
			cl.def_readwrite("insertionOpts", &mrpt::maps::CHeightGridMap2D_MRF::TMapDefinition::insertionOpts);
		}

	}
}
