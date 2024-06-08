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
#include <mrpt/img/TColor.h>
#include <mrpt/maps/CHeightGridMap2D.h>
#include <mrpt/maps/CHeightGridMap2D_Base.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/TMetricMapInitializer.h>
#include <mrpt/maps/metric_map_types.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TLine2D.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TObject2D.h>
#include <mrpt/math/TObject3D.h>
#include <mrpt/math/TPlane.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPolygon2D.h>
#include <mrpt/math/TPolygon3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TPose3DQuat.h>
#include <mrpt/math/TSegment2D.h>
#include <mrpt/math/TSegment3D.h>
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

// mrpt::maps::CHeightGridMap2D_Base file:mrpt/maps/CHeightGridMap2D_Base.h line:23
struct PyCallBack_mrpt_maps_CHeightGridMap2D_Base : public mrpt::maps::CHeightGridMap2D_Base {
	using mrpt::maps::CHeightGridMap2D_Base::CHeightGridMap2D_Base;

	bool insertIndividualPoint(const double a0, const double a1, const double a2, const struct mrpt::maps::CHeightGridMap2D_Base::TPointInsertParams & a3) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_Base *>(this), "insertIndividualPoint");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CHeightGridMap2D_Base::insertIndividualPoint\"");
	}
	double dem_get_resolution() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_Base *>(this), "dem_get_resolution");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CHeightGridMap2D_Base::dem_get_resolution\"");
	}
	size_t dem_get_size_x() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_Base *>(this), "dem_get_size_x");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CHeightGridMap2D_Base::dem_get_size_x\"");
	}
	size_t dem_get_size_y() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_Base *>(this), "dem_get_size_y");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CHeightGridMap2D_Base::dem_get_size_y\"");
	}
	bool dem_get_z_by_cell(size_t a0, size_t a1, double & a2) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_Base *>(this), "dem_get_z_by_cell");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CHeightGridMap2D_Base::dem_get_z_by_cell\"");
	}
	bool dem_get_z(const double a0, const double a1, double & a2) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_Base *>(this), "dem_get_z");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CHeightGridMap2D_Base::dem_get_z\"");
	}
	void dem_update_map() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D_Base *>(this), "dem_update_map");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CHeightGridMap2D_Base::dem_update_map\"");
	}
};

// mrpt::maps::CHeightGridMap2D file:mrpt/maps/CHeightGridMap2D.h line:62
struct PyCallBack_mrpt_maps_CHeightGridMap2D : public mrpt::maps::CHeightGridMap2D {
	using mrpt::maps::CHeightGridMap2D::CHeightGridMap2D;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CHeightGridMap2D::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CHeightGridMap2D::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CHeightGridMap2D::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CHeightGridMap2D::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CHeightGridMap2D::serializeFrom(a0, a1);
	}
	float cell2float(const struct mrpt::maps::THeightGridmapCell & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D *>(this), "cell2float");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<float>::value) {
				static pybind11::detail::override_caster_t<float> caster;
				return pybind11::detail::cast_ref<float>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<float>(std::move(o));
		}
		return CHeightGridMap2D::cell2float(a0);
	}
	std::string asString() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D *>(this), "asString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CHeightGridMap2D::asString();
	}
	bool isEmpty() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D *>(this), "isEmpty");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CHeightGridMap2D::isEmpty();
	}
	float compute3DMatchingRatio(const class mrpt::maps::CMetricMap * a0, const class mrpt::poses::CPose3D & a1, const struct mrpt::maps::TMatchingRatioParams & a2) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D *>(this), "compute3DMatchingRatio");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<float>::value) {
				static pybind11::detail::override_caster_t<float> caster;
				return pybind11::detail::cast_ref<float>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<float>(std::move(o));
		}
		return CHeightGridMap2D::compute3DMatchingRatio(a0, a1, a2);
	}
	void saveMetricMapRepresentationToFile(const std::string & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D *>(this), "saveMetricMapRepresentationToFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CHeightGridMap2D::saveMetricMapRepresentationToFile(a0);
	}
	void getVisualizationInto(class mrpt::opengl::CSetOfObjects & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D *>(this), "getVisualizationInto");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CHeightGridMap2D::getVisualizationInto(a0);
	}
	bool insertIndividualPoint(const double a0, const double a1, const double a2, const struct mrpt::maps::CHeightGridMap2D_Base::TPointInsertParams & a3) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D *>(this), "insertIndividualPoint");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CHeightGridMap2D::insertIndividualPoint(a0, a1, a2, a3);
	}
	double dem_get_resolution() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D *>(this), "dem_get_resolution");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CHeightGridMap2D::dem_get_resolution();
	}
	size_t dem_get_size_x() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D *>(this), "dem_get_size_x");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CHeightGridMap2D::dem_get_size_x();
	}
	size_t dem_get_size_y() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D *>(this), "dem_get_size_y");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CHeightGridMap2D::dem_get_size_y();
	}
	bool dem_get_z_by_cell(size_t a0, size_t a1, double & a2) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D *>(this), "dem_get_z_by_cell");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CHeightGridMap2D::dem_get_z_by_cell(a0, a1, a2);
	}
	bool dem_get_z(const double a0, const double a1, double & a2) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D *>(this), "dem_get_z");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CHeightGridMap2D::dem_get_z(a0, a1, a2);
	}
	void dem_update_map() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D *>(this), "dem_update_map");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CHeightGridMap2D::dem_update_map();
	}
	void internal_clear() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D *>(this), "internal_clear");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CHeightGridMap2D::internal_clear();
	}
	double internal_computeObservationLikelihood(const class mrpt::obs::CObservation & a0, const class mrpt::poses::CPose3D & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D *>(this), "internal_computeObservationLikelihood");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CHeightGridMap2D::internal_computeObservationLikelihood(a0, a1);
	}
	struct mrpt::math::TBoundingBox_<float> boundingBox() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D *>(this), "boundingBox");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D *>(this), "canComputeObservationLikelihood");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D *>(this), "determineMatching2D");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D *>(this), "determineMatching3D");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D *>(this), "auxParticleFilterCleanUp");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D *>(this), "squareDistanceToClosestCorrespondence");
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
	void resize(double a0, double a1, double a2, double a3, const struct mrpt::maps::THeightGridmapCell & a4, double a5) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D *>(this), "resize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4, a5);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CDynamicGrid::resize(a0, a1, a2, a3, a4, a5);
	}
};

// mrpt::maps::CHeightGridMap2D::TInsertionOptions file:mrpt/maps/CHeightGridMap2D.h line:105
struct PyCallBack_mrpt_maps_CHeightGridMap2D_TInsertionOptions : public mrpt::maps::CHeightGridMap2D::TInsertionOptions {
	using mrpt::maps::CHeightGridMap2D::TInsertionOptions::TInsertionOptions;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D::TInsertionOptions *>(this), "loadFromConfigFile");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D::TInsertionOptions *>(this), "saveToConfigFile");
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

// mrpt::maps::CHeightGridMap2D::TMapDefinition file: line:78
struct PyCallBack_mrpt_maps_CHeightGridMap2D_TMapDefinition : public mrpt::maps::CHeightGridMap2D::TMapDefinition {
	using mrpt::maps::CHeightGridMap2D::TMapDefinition::TMapDefinition;

	void loadFromConfigFile_map_specific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D::TMapDefinition *>(this), "loadFromConfigFile_map_specific");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D::TMapDefinition *>(this), "loadFromConfigFile");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CHeightGridMap2D::TMapDefinition *>(this), "saveToConfigFile");
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

void bind_mrpt_maps_CHeightGridMap2D_Base(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::maps::CHeightGridMap2D_Base file:mrpt/maps/CHeightGridMap2D_Base.h line:23
		pybind11::class_<mrpt::maps::CHeightGridMap2D_Base, std::shared_ptr<mrpt::maps::CHeightGridMap2D_Base>, PyCallBack_mrpt_maps_CHeightGridMap2D_Base> cl(M("mrpt::maps"), "CHeightGridMap2D_Base", "Virtual base class for Digital Elevation Model (DEM) maps. See derived\n classes for details.\n This class implements those operations which are especific to DEMs.\n \n");
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_maps_CHeightGridMap2D_Base(); } ) );
		cl.def(pybind11::init<PyCallBack_mrpt_maps_CHeightGridMap2D_Base const &>());
		cl.def("intersectLine3D", (bool (mrpt::maps::CHeightGridMap2D_Base::*)(const struct mrpt::math::TLine3D &, struct mrpt::math::TObject3D &) const) &mrpt::maps::CHeightGridMap2D_Base::intersectLine3D, "@{ \n\n Gets the intersection between a 3D line and a Height Grid map (taking\n into account the different heights of each individual cell)  \n\nC++: mrpt::maps::CHeightGridMap2D_Base::intersectLine3D(const struct mrpt::math::TLine3D &, struct mrpt::math::TObject3D &) const --> bool", pybind11::arg("r1"), pybind11::arg("obj"));
		cl.def("getMinMaxHeight", (bool (mrpt::maps::CHeightGridMap2D_Base::*)(float &, float &) const) &mrpt::maps::CHeightGridMap2D_Base::getMinMaxHeight, "Computes the minimum and maximum height in the grid.\n \n\n False if there is no observed cell yet. \n\nC++: mrpt::maps::CHeightGridMap2D_Base::getMinMaxHeight(float &, float &) const --> bool", pybind11::arg("z_min"), pybind11::arg("z_max"));
		cl.def("insertIndividualPoint", [](mrpt::maps::CHeightGridMap2D_Base &o, const double & a0, const double & a1, const double & a2) -> bool { return o.insertIndividualPoint(a0, a1, a2); }, "", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("insertIndividualPoint", (bool (mrpt::maps::CHeightGridMap2D_Base::*)(const double, const double, const double, const struct mrpt::maps::CHeightGridMap2D_Base::TPointInsertParams &)) &mrpt::maps::CHeightGridMap2D_Base::insertIndividualPoint, "Update the DEM with one new point.\n \n\n mrpt::maps::CMetricMap::insertObservation() for inserting\n higher-level objects like 2D/3D LIDAR scans\n \n\n true if updated OK, false if (x,y) is out of bounds \n\nC++: mrpt::maps::CHeightGridMap2D_Base::insertIndividualPoint(const double, const double, const double, const struct mrpt::maps::CHeightGridMap2D_Base::TPointInsertParams &) --> bool", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("params"));
		cl.def("dem_get_resolution", (double (mrpt::maps::CHeightGridMap2D_Base::*)() const) &mrpt::maps::CHeightGridMap2D_Base::dem_get_resolution, "C++: mrpt::maps::CHeightGridMap2D_Base::dem_get_resolution() const --> double");
		cl.def("dem_get_size_x", (size_t (mrpt::maps::CHeightGridMap2D_Base::*)() const) &mrpt::maps::CHeightGridMap2D_Base::dem_get_size_x, "C++: mrpt::maps::CHeightGridMap2D_Base::dem_get_size_x() const --> size_t");
		cl.def("dem_get_size_y", (size_t (mrpt::maps::CHeightGridMap2D_Base::*)() const) &mrpt::maps::CHeightGridMap2D_Base::dem_get_size_y, "C++: mrpt::maps::CHeightGridMap2D_Base::dem_get_size_y() const --> size_t");
		cl.def("dem_get_z_by_cell", (bool (mrpt::maps::CHeightGridMap2D_Base::*)(size_t, size_t, double &) const) &mrpt::maps::CHeightGridMap2D_Base::dem_get_z_by_cell, "Get cell 'z' by (cx,cy) cell indices. \n false if out of bounds or\n un-observed cell. \n\nC++: mrpt::maps::CHeightGridMap2D_Base::dem_get_z_by_cell(size_t, size_t, double &) const --> bool", pybind11::arg("cx"), pybind11::arg("cy"), pybind11::arg("z_out"));
		cl.def("dem_get_z", (bool (mrpt::maps::CHeightGridMap2D_Base::*)(const double, const double, double &) const) &mrpt::maps::CHeightGridMap2D_Base::dem_get_z, "Get cell 'z' (x,y) by metric coordinates. \n false if out of bounds\n or un-observed cell. \n\nC++: mrpt::maps::CHeightGridMap2D_Base::dem_get_z(const double, const double, double &) const --> bool", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z_out"));
		cl.def("dem_update_map", (void (mrpt::maps::CHeightGridMap2D_Base::*)()) &mrpt::maps::CHeightGridMap2D_Base::dem_update_map, "Ensure that all observations are reflected in the map estimate \n\nC++: mrpt::maps::CHeightGridMap2D_Base::dem_update_map() --> void");
		cl.def("assign", (class mrpt::maps::CHeightGridMap2D_Base & (mrpt::maps::CHeightGridMap2D_Base::*)(const class mrpt::maps::CHeightGridMap2D_Base &)) &mrpt::maps::CHeightGridMap2D_Base::operator=, "C++: mrpt::maps::CHeightGridMap2D_Base::operator=(const class mrpt::maps::CHeightGridMap2D_Base &) --> class mrpt::maps::CHeightGridMap2D_Base &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::maps::CHeightGridMap2D_Base::TPointInsertParams file:mrpt/maps/CHeightGridMap2D_Base.h line:40
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CHeightGridMap2D_Base::TPointInsertParams, std::shared_ptr<mrpt::maps::CHeightGridMap2D_Base::TPointInsertParams>> cl(enclosing_class, "TPointInsertParams", "Extra params for insertIndividualPoint() ");
			cl.def( pybind11::init( [](){ return new mrpt::maps::CHeightGridMap2D_Base::TPointInsertParams(); } ) );
			cl.def( pybind11::init( [](mrpt::maps::CHeightGridMap2D_Base::TPointInsertParams const &o){ return new mrpt::maps::CHeightGridMap2D_Base::TPointInsertParams(o); } ) );
			cl.def_readwrite("pt_z_std", &mrpt::maps::CHeightGridMap2D_Base::TPointInsertParams::pt_z_std);
			cl.def_readwrite("update_map_after_insertion", &mrpt::maps::CHeightGridMap2D_Base::TPointInsertParams::update_map_after_insertion);
		}

	}
	{ // mrpt::maps::THeightGridmapCell file:mrpt/maps/CHeightGridMap2D.h line:27
		pybind11::class_<mrpt::maps::THeightGridmapCell, std::shared_ptr<mrpt::maps::THeightGridmapCell>> cl(M("mrpt::maps"), "THeightGridmapCell", "The contents of each cell in a CHeightGridMap2D map ");
		cl.def( pybind11::init( [](){ return new mrpt::maps::THeightGridmapCell(); } ) );
		cl.def_readwrite("h", &mrpt::maps::THeightGridmapCell::h);
		cl.def_readwrite("var", &mrpt::maps::THeightGridmapCell::var);
		cl.def_readwrite("u", &mrpt::maps::THeightGridmapCell::u);
		cl.def_readwrite("v", &mrpt::maps::THeightGridmapCell::v);
		cl.def_readwrite("w", &mrpt::maps::THeightGridmapCell::w);
	}
	{ // mrpt::maps::CHeightGridMap2D file:mrpt/maps/CHeightGridMap2D.h line:62
		pybind11::class_<mrpt::maps::CHeightGridMap2D, std::shared_ptr<mrpt::maps::CHeightGridMap2D>, PyCallBack_mrpt_maps_CHeightGridMap2D, mrpt::maps::CMetricMap, mrpt::containers::CDynamicGrid<mrpt::maps::THeightGridmapCell>, mrpt::maps::CHeightGridMap2D_Base> cl(M("mrpt::maps"), "CHeightGridMap2D", "Digital Elevation Model (DEM), a mesh or grid representation of a surface\n which keeps the estimated height for each (x,y) location.\n  Important implemented features are the insertion of 2D laser scans (from\n arbitrary 6D poses) and the exportation as 3D scenes.\n\n Each cell contains the up-to-date average height from measured falling in\n that cell. Algorithms that can be used:\n   - mrSimpleAverage: Each cell only stores the current average value.\n\n  This class implements generic version of\n mrpt::maps::CMetric::insertObservation() accepting these types of sensory\n data:\n   - mrpt::obs::CObservation2DRangeScan: 2D range scans\n   - mrpt::obs::CObservationVelodyneScan\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::maps::CHeightGridMap2D(); }, [](){ return new PyCallBack_mrpt_maps_CHeightGridMap2D(); } ), "doc");
		cl.def( pybind11::init( [](enum mrpt::maps::CHeightGridMap2D::TMapRepresentation const & a0){ return new mrpt::maps::CHeightGridMap2D(a0); }, [](enum mrpt::maps::CHeightGridMap2D::TMapRepresentation const & a0){ return new PyCallBack_mrpt_maps_CHeightGridMap2D(a0); } ), "doc");
		cl.def( pybind11::init( [](enum mrpt::maps::CHeightGridMap2D::TMapRepresentation const & a0, double const & a1){ return new mrpt::maps::CHeightGridMap2D(a0, a1); }, [](enum mrpt::maps::CHeightGridMap2D::TMapRepresentation const & a0, double const & a1){ return new PyCallBack_mrpt_maps_CHeightGridMap2D(a0, a1); } ), "doc");
		cl.def( pybind11::init( [](enum mrpt::maps::CHeightGridMap2D::TMapRepresentation const & a0, double const & a1, double const & a2){ return new mrpt::maps::CHeightGridMap2D(a0, a1, a2); }, [](enum mrpt::maps::CHeightGridMap2D::TMapRepresentation const & a0, double const & a1, double const & a2){ return new PyCallBack_mrpt_maps_CHeightGridMap2D(a0, a1, a2); } ), "doc");
		cl.def( pybind11::init( [](enum mrpt::maps::CHeightGridMap2D::TMapRepresentation const & a0, double const & a1, double const & a2, double const & a3){ return new mrpt::maps::CHeightGridMap2D(a0, a1, a2, a3); }, [](enum mrpt::maps::CHeightGridMap2D::TMapRepresentation const & a0, double const & a1, double const & a2, double const & a3){ return new PyCallBack_mrpt_maps_CHeightGridMap2D(a0, a1, a2, a3); } ), "doc");
		cl.def( pybind11::init( [](enum mrpt::maps::CHeightGridMap2D::TMapRepresentation const & a0, double const & a1, double const & a2, double const & a3, double const & a4){ return new mrpt::maps::CHeightGridMap2D(a0, a1, a2, a3, a4); }, [](enum mrpt::maps::CHeightGridMap2D::TMapRepresentation const & a0, double const & a1, double const & a2, double const & a3, double const & a4){ return new PyCallBack_mrpt_maps_CHeightGridMap2D(a0, a1, a2, a3, a4); } ), "doc");
		cl.def( pybind11::init<enum mrpt::maps::CHeightGridMap2D::TMapRepresentation, double, double, double, double, double>(), pybind11::arg("mapType"), pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("resolution") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_maps_CHeightGridMap2D const &o){ return new PyCallBack_mrpt_maps_CHeightGridMap2D(o); } ) );
		cl.def( pybind11::init( [](mrpt::maps::CHeightGridMap2D const &o){ return new mrpt::maps::CHeightGridMap2D(o); } ) );

		pybind11::enum_<mrpt::maps::CHeightGridMap2D::TMapRepresentation>(cl, "TMapRepresentation", pybind11::arithmetic(), "The type of map representation to be used.\n  See mrpt::maps::CHeightGridMap2D for discussion.")
			.value("mrSimpleAverage", mrpt::maps::CHeightGridMap2D::mrSimpleAverage)
			.export_values();

		cl.def_readwrite("insertionOptions", &mrpt::maps::CHeightGridMap2D::insertionOptions);
		cl.def_readwrite("m_mapType", &mrpt::maps::CHeightGridMap2D::m_mapType);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::maps::CHeightGridMap2D::GetRuntimeClassIdStatic, "C++: mrpt::maps::CHeightGridMap2D::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::maps::CHeightGridMap2D::*)() const) &mrpt::maps::CHeightGridMap2D::GetRuntimeClass, "C++: mrpt::maps::CHeightGridMap2D::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::maps::CHeightGridMap2D::*)() const) &mrpt::maps::CHeightGridMap2D::clone, "C++: mrpt::maps::CHeightGridMap2D::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::maps::CHeightGridMap2D::CreateObject, "C++: mrpt::maps::CHeightGridMap2D::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("clear", (void (mrpt::maps::CHeightGridMap2D::*)()) &mrpt::maps::CHeightGridMap2D::clear, "Calls the base CMetricMap::clear\n Declared here to avoid ambiguity between the two clear() in both base\n classes.\n\nC++: mrpt::maps::CHeightGridMap2D::clear() --> void");
		cl.def("cell2float", (float (mrpt::maps::CHeightGridMap2D::*)(const struct mrpt::maps::THeightGridmapCell &) const) &mrpt::maps::CHeightGridMap2D::cell2float, "C++: mrpt::maps::CHeightGridMap2D::cell2float(const struct mrpt::maps::THeightGridmapCell &) const --> float", pybind11::arg("c"));
		cl.def("asString", (std::string (mrpt::maps::CHeightGridMap2D::*)() const) &mrpt::maps::CHeightGridMap2D::asString, "Returns a short description of the map. \n\nC++: mrpt::maps::CHeightGridMap2D::asString() const --> std::string");
		cl.def("isEmpty", (bool (mrpt::maps::CHeightGridMap2D::*)() const) &mrpt::maps::CHeightGridMap2D::isEmpty, "Returns true if the map is empty/no observation has been inserted. \n\nC++: mrpt::maps::CHeightGridMap2D::isEmpty() const --> bool");
		cl.def("compute3DMatchingRatio", (float (mrpt::maps::CHeightGridMap2D::*)(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose3D &, const struct mrpt::maps::TMatchingRatioParams &) const) &mrpt::maps::CHeightGridMap2D::compute3DMatchingRatio, "See docs in base class: in this class it always returns 0 \n\nC++: mrpt::maps::CHeightGridMap2D::compute3DMatchingRatio(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose3D &, const struct mrpt::maps::TMatchingRatioParams &) const --> float", pybind11::arg("otherMap"), pybind11::arg("otherMapPose"), pybind11::arg("params"));
		cl.def("saveMetricMapRepresentationToFile", (void (mrpt::maps::CHeightGridMap2D::*)(const std::string &) const) &mrpt::maps::CHeightGridMap2D::saveMetricMapRepresentationToFile, "C++: mrpt::maps::CHeightGridMap2D::saveMetricMapRepresentationToFile(const std::string &) const --> void", pybind11::arg("filNamePrefix"));
		cl.def("getVisualizationInto", (void (mrpt::maps::CHeightGridMap2D::*)(class mrpt::opengl::CSetOfObjects &) const) &mrpt::maps::CHeightGridMap2D::getVisualizationInto, "Returns a 3D object representing the map: by default, it will be a\n mrpt::opengl::CMesh object, unless\n   it is specified otherwise in\n mrpt::global_settings::HEIGHTGRIDMAP_EXPORT3D_AS_MESH \n\nC++: mrpt::maps::CHeightGridMap2D::getVisualizationInto(class mrpt::opengl::CSetOfObjects &) const --> void", pybind11::arg("outObj"));
		cl.def("getMapType", (enum mrpt::maps::CHeightGridMap2D::TMapRepresentation (mrpt::maps::CHeightGridMap2D::*)()) &mrpt::maps::CHeightGridMap2D::getMapType, "Return the type of the gas distribution map, according to parameters\n passed on construction \n\nC++: mrpt::maps::CHeightGridMap2D::getMapType() --> enum mrpt::maps::CHeightGridMap2D::TMapRepresentation");
		cl.def("countObservedCells", (size_t (mrpt::maps::CHeightGridMap2D::*)() const) &mrpt::maps::CHeightGridMap2D::countObservedCells, "Return the number of cells with at least one height data inserted. \n\nC++: mrpt::maps::CHeightGridMap2D::countObservedCells() const --> size_t");
		cl.def("insertIndividualPoint", [](mrpt::maps::CHeightGridMap2D &o, const double & a0, const double & a1, const double & a2) -> bool { return o.insertIndividualPoint(a0, a1, a2); }, "", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("insertIndividualPoint", (bool (mrpt::maps::CHeightGridMap2D::*)(const double, const double, const double, const struct mrpt::maps::CHeightGridMap2D_Base::TPointInsertParams &)) &mrpt::maps::CHeightGridMap2D::insertIndividualPoint, "C++: mrpt::maps::CHeightGridMap2D::insertIndividualPoint(const double, const double, const double, const struct mrpt::maps::CHeightGridMap2D_Base::TPointInsertParams &) --> bool", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("params"));
		cl.def("dem_get_resolution", (double (mrpt::maps::CHeightGridMap2D::*)() const) &mrpt::maps::CHeightGridMap2D::dem_get_resolution, "C++: mrpt::maps::CHeightGridMap2D::dem_get_resolution() const --> double");
		cl.def("dem_get_size_x", (size_t (mrpt::maps::CHeightGridMap2D::*)() const) &mrpt::maps::CHeightGridMap2D::dem_get_size_x, "C++: mrpt::maps::CHeightGridMap2D::dem_get_size_x() const --> size_t");
		cl.def("dem_get_size_y", (size_t (mrpt::maps::CHeightGridMap2D::*)() const) &mrpt::maps::CHeightGridMap2D::dem_get_size_y, "C++: mrpt::maps::CHeightGridMap2D::dem_get_size_y() const --> size_t");
		cl.def("dem_get_z_by_cell", (bool (mrpt::maps::CHeightGridMap2D::*)(size_t, size_t, double &) const) &mrpt::maps::CHeightGridMap2D::dem_get_z_by_cell, "C++: mrpt::maps::CHeightGridMap2D::dem_get_z_by_cell(size_t, size_t, double &) const --> bool", pybind11::arg("cx"), pybind11::arg("cy"), pybind11::arg("z_out"));
		cl.def("dem_get_z", (bool (mrpt::maps::CHeightGridMap2D::*)(const double, const double, double &) const) &mrpt::maps::CHeightGridMap2D::dem_get_z, "C++: mrpt::maps::CHeightGridMap2D::dem_get_z(const double, const double, double &) const --> bool", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z_out"));
		cl.def("dem_update_map", (void (mrpt::maps::CHeightGridMap2D::*)()) &mrpt::maps::CHeightGridMap2D::dem_update_map, "C++: mrpt::maps::CHeightGridMap2D::dem_update_map() --> void");
		cl.def("internal_clear", (void (mrpt::maps::CHeightGridMap2D::*)()) &mrpt::maps::CHeightGridMap2D::internal_clear, "C++: mrpt::maps::CHeightGridMap2D::internal_clear() --> void");
		cl.def("internal_computeObservationLikelihood", (double (mrpt::maps::CHeightGridMap2D::*)(const class mrpt::obs::CObservation &, const class mrpt::poses::CPose3D &) const) &mrpt::maps::CHeightGridMap2D::internal_computeObservationLikelihood, "C++: mrpt::maps::CHeightGridMap2D::internal_computeObservationLikelihood(const class mrpt::obs::CObservation &, const class mrpt::poses::CPose3D &) const --> double", pybind11::arg("obs"), pybind11::arg("takenFrom"));
		cl.def("assign", (class mrpt::maps::CHeightGridMap2D & (mrpt::maps::CHeightGridMap2D::*)(const class mrpt::maps::CHeightGridMap2D &)) &mrpt::maps::CHeightGridMap2D::operator=, "C++: mrpt::maps::CHeightGridMap2D::operator=(const class mrpt::maps::CHeightGridMap2D &) --> class mrpt::maps::CHeightGridMap2D &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::maps::CHeightGridMap2D::TInsertionOptions file:mrpt/maps/CHeightGridMap2D.h line:105
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CHeightGridMap2D::TInsertionOptions, std::shared_ptr<mrpt::maps::CHeightGridMap2D::TInsertionOptions>, PyCallBack_mrpt_maps_CHeightGridMap2D_TInsertionOptions, mrpt::config::CLoadableOptions> cl(enclosing_class, "TInsertionOptions", "Parameters related with inserting observations into the map ");
			cl.def( pybind11::init( [](){ return new mrpt::maps::CHeightGridMap2D::TInsertionOptions(); }, [](){ return new PyCallBack_mrpt_maps_CHeightGridMap2D_TInsertionOptions(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_maps_CHeightGridMap2D_TInsertionOptions const &o){ return new PyCallBack_mrpt_maps_CHeightGridMap2D_TInsertionOptions(o); } ) );
			cl.def( pybind11::init( [](mrpt::maps::CHeightGridMap2D::TInsertionOptions const &o){ return new mrpt::maps::CHeightGridMap2D::TInsertionOptions(o); } ) );
			cl.def_readwrite("filterByHeight", &mrpt::maps::CHeightGridMap2D::TInsertionOptions::filterByHeight);
			cl.def_readwrite("z_min", &mrpt::maps::CHeightGridMap2D::TInsertionOptions::z_min);
			cl.def_readwrite("z_max", &mrpt::maps::CHeightGridMap2D::TInsertionOptions::z_max);
			cl.def_readwrite("colorMap", &mrpt::maps::CHeightGridMap2D::TInsertionOptions::colorMap);
			cl.def("loadFromConfigFile", (void (mrpt::maps::CHeightGridMap2D::TInsertionOptions::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::maps::CHeightGridMap2D::TInsertionOptions::loadFromConfigFile, "C++: mrpt::maps::CHeightGridMap2D::TInsertionOptions::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
			cl.def("assign", (struct mrpt::maps::CHeightGridMap2D::TInsertionOptions & (mrpt::maps::CHeightGridMap2D::TInsertionOptions::*)(const struct mrpt::maps::CHeightGridMap2D::TInsertionOptions &)) &mrpt::maps::CHeightGridMap2D::TInsertionOptions::operator=, "C++: mrpt::maps::CHeightGridMap2D::TInsertionOptions::operator=(const struct mrpt::maps::CHeightGridMap2D::TInsertionOptions &) --> struct mrpt::maps::CHeightGridMap2D::TInsertionOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::maps::CHeightGridMap2D::TMapDefinitionBase file: line:74
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CHeightGridMap2D::TMapDefinitionBase, std::shared_ptr<mrpt::maps::CHeightGridMap2D::TMapDefinitionBase>> cl(enclosing_class, "TMapDefinitionBase", "");
		}

		{ // mrpt::maps::CHeightGridMap2D::TMapDefinition file: line:78
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CHeightGridMap2D::TMapDefinition, std::shared_ptr<mrpt::maps::CHeightGridMap2D::TMapDefinition>, PyCallBack_mrpt_maps_CHeightGridMap2D_TMapDefinition, mrpt::maps::CHeightGridMap2D::TMapDefinitionBase> cl(enclosing_class, "TMapDefinition", "");
			cl.def( pybind11::init( [](){ return new mrpt::maps::CHeightGridMap2D::TMapDefinition(); }, [](){ return new PyCallBack_mrpt_maps_CHeightGridMap2D_TMapDefinition(); } ) );
			cl.def_readwrite("min_x", &mrpt::maps::CHeightGridMap2D::TMapDefinition::min_x);
			cl.def_readwrite("max_x", &mrpt::maps::CHeightGridMap2D::TMapDefinition::max_x);
			cl.def_readwrite("min_y", &mrpt::maps::CHeightGridMap2D::TMapDefinition::min_y);
			cl.def_readwrite("max_y", &mrpt::maps::CHeightGridMap2D::TMapDefinition::max_y);
			cl.def_readwrite("resolution", &mrpt::maps::CHeightGridMap2D::TMapDefinition::resolution);
			cl.def_readwrite("mapType", &mrpt::maps::CHeightGridMap2D::TMapDefinition::mapType);
			cl.def_readwrite("insertionOpts", &mrpt::maps::CHeightGridMap2D::TMapDefinition::insertionOpts);
		}

	}
}
