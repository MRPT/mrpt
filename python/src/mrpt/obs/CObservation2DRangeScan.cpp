#include <chrono>
#include <ios>
#include <iterator>
#include <memory>
#include <mrpt/core/Clock.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CPolygon.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TPose3DQuat.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/T2DScanProperties.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ostream>
#include <ratio>
#include <sstream> // __str__
#include <streambuf>
#include <string>
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

// mrpt::obs::CObservation2DRangeScan file:mrpt/obs/CObservation2DRangeScan.h line:55
struct PyCallBack_mrpt_obs_CObservation2DRangeScan : public mrpt::obs::CObservation2DRangeScan {
	using mrpt::obs::CObservation2DRangeScan::CObservation2DRangeScan;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation2DRangeScan *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CObservation2DRangeScan::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation2DRangeScan *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CObservation2DRangeScan::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation2DRangeScan *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CObservation2DRangeScan::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation2DRangeScan *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservation2DRangeScan::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation2DRangeScan *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservation2DRangeScan::serializeFrom(a0, a1);
	}
	void getSensorPose(class mrpt::poses::CPose3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation2DRangeScan *>(this), "getSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservation2DRangeScan::getSensorPose(a0);
	}
	void setSensorPose(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation2DRangeScan *>(this), "setSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservation2DRangeScan::setSensorPose(a0);
	}
	bool exportTxtSupported() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation2DRangeScan *>(this), "exportTxtSupported");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CObservation2DRangeScan::exportTxtSupported();
	}
	std::string exportTxtHeader() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation2DRangeScan *>(this), "exportTxtHeader");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CObservation2DRangeScan::exportTxtHeader();
	}
	std::string exportTxtDataRow() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation2DRangeScan *>(this), "exportTxtDataRow");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CObservation2DRangeScan::exportTxtDataRow();
	}
	using _binder_ret_0 = mrpt::Clock::time_point;
	_binder_ret_0 getOriginalReceivedTimeStamp() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation2DRangeScan *>(this), "getOriginalReceivedTimeStamp");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<_binder_ret_0>::value) {
				static pybind11::detail::override_caster_t<_binder_ret_0> caster;
				return pybind11::detail::cast_ref<_binder_ret_0>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<_binder_ret_0>(std::move(o));
		}
		return CObservation::getOriginalReceivedTimeStamp();
	}
	std::string asString() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation2DRangeScan *>(this), "asString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CObservation::asString();
	}
	void unload() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation2DRangeScan *>(this), "unload");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservation::unload();
	}
	void load_impl() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservation2DRangeScan *>(this), "load_impl");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservation::load_impl();
	}
};

void bind_mrpt_obs_CObservation2DRangeScan(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::obs::CObservation2DRangeScan file:mrpt/obs/CObservation2DRangeScan.h line:55
		pybind11::class_<mrpt::obs::CObservation2DRangeScan, std::shared_ptr<mrpt::obs::CObservation2DRangeScan>, PyCallBack_mrpt_obs_CObservation2DRangeScan, mrpt::obs::CObservation> cl(M("mrpt::obs"), "CObservation2DRangeScan", "A \"CObservation\"-derived class that represents a 2D range scan measurement\n (typically from a laser scanner).\n The data structures are generic enough to hold a wide variety of 2D\n scanners and \"3D\" planar rotating 2D lasers.\n\n These are the most important data fields:\n - Scan ranges: A vector of float values with all the range measurements\n [meters]. Access via `CObservation2DRangeScan::getScanRange()` and\n `CObservation2DRangeScan::setScanRange()`.\n - Range validity: A vector (of identical size to scan), it holds\n `true` for those ranges than are valid (i.e. will be zero for non-reflected\n rays, etc.), `false` for scan rays without a valid lidar return.\n - Reflection intensity: A vector (of identical size to scan) a\n unitless int values representing the relative strength of each return. Higher\n values indicate a more intense return. This is useful for filtering out low\n intensity (noisy) returns or detecting intense landmarks.\n - CObservation2DRangeScan::aperture: The field-of-view of the scanner,\n in radians (typically, M_PI = 180deg).\n - CObservation2DRangeScan::sensorPose: The 6D location of the sensor on\n the robot reference frame (default=at the origin), i.e. wrt `base_link`\n following ROS conventions.\n - CObservation2DRangeScan::rightToLeft: The scanning direction:\n true=counterclockwise (default), false=clockwise.\n\n Note that the *angle of each range* in the vectors above is implicitly\n defined by the index within the vector.\n \n\n CObservation, CPointsMap, T2DScanProperties\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::obs::CObservation2DRangeScan(); }, [](){ return new PyCallBack_mrpt_obs_CObservation2DRangeScan(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_CObservation2DRangeScan const &o){ return new PyCallBack_mrpt_obs_CObservation2DRangeScan(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::CObservation2DRangeScan const &o){ return new mrpt::obs::CObservation2DRangeScan(o); } ) );
		cl.def_readwrite("aperture", &mrpt::obs::CObservation2DRangeScan::aperture);
		cl.def_readwrite("rightToLeft", &mrpt::obs::CObservation2DRangeScan::rightToLeft);
		cl.def_readwrite("maxRange", &mrpt::obs::CObservation2DRangeScan::maxRange);
		cl.def_readwrite("sensorPose", &mrpt::obs::CObservation2DRangeScan::sensorPose);
		cl.def_readwrite("stdError", &mrpt::obs::CObservation2DRangeScan::stdError);
		cl.def_readwrite("beamAperture", &mrpt::obs::CObservation2DRangeScan::beamAperture);
		cl.def_readwrite("deltaPitch", &mrpt::obs::CObservation2DRangeScan::deltaPitch);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::obs::CObservation2DRangeScan::GetRuntimeClassIdStatic, "C++: mrpt::obs::CObservation2DRangeScan::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::obs::CObservation2DRangeScan::*)() const) &mrpt::obs::CObservation2DRangeScan::GetRuntimeClass, "C++: mrpt::obs::CObservation2DRangeScan::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::obs::CObservation2DRangeScan::*)() const) &mrpt::obs::CObservation2DRangeScan::clone, "C++: mrpt::obs::CObservation2DRangeScan::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::obs::CObservation2DRangeScan::CreateObject, "C++: mrpt::obs::CObservation2DRangeScan::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("resizeScan", (void (mrpt::obs::CObservation2DRangeScan::*)(size_t)) &mrpt::obs::CObservation2DRangeScan::resizeScan, "@{ \n\n Resizes all data vectors to allocate a given number of scan rays \n\nC++: mrpt::obs::CObservation2DRangeScan::resizeScan(size_t) --> void", pybind11::arg("len"));
		cl.def("resizeScanAndAssign", [](mrpt::obs::CObservation2DRangeScan &o, size_t const & a0, const float & a1, const bool & a2) -> void { return o.resizeScanAndAssign(a0, a1, a2); }, "", pybind11::arg("len"), pybind11::arg("rangeVal"), pybind11::arg("rangeValidity"));
		cl.def("resizeScanAndAssign", (void (mrpt::obs::CObservation2DRangeScan::*)(size_t, const float, const bool, const int)) &mrpt::obs::CObservation2DRangeScan::resizeScanAndAssign, "Resizes all data vectors to allocate a given number of scan rays and\n assign default values. \n\nC++: mrpt::obs::CObservation2DRangeScan::resizeScanAndAssign(size_t, const float, const bool, const int) --> void", pybind11::arg("len"), pybind11::arg("rangeVal"), pybind11::arg("rangeValidity"), pybind11::arg("rangeIntensity"));
		cl.def("getScanSize", (size_t (mrpt::obs::CObservation2DRangeScan::*)() const) &mrpt::obs::CObservation2DRangeScan::getScanSize, "Get number of scan rays \n\nC++: mrpt::obs::CObservation2DRangeScan::getScanSize() const --> size_t");
		cl.def("getScanRange", (float & (mrpt::obs::CObservation2DRangeScan::*)(size_t)) &mrpt::obs::CObservation2DRangeScan::getScanRange, "C++: mrpt::obs::CObservation2DRangeScan::getScanRange(size_t) --> float &", pybind11::return_value_policy::reference, pybind11::arg("i"));
		cl.def("setScanRange", (void (mrpt::obs::CObservation2DRangeScan::*)(size_t, const float)) &mrpt::obs::CObservation2DRangeScan::setScanRange, "C++: mrpt::obs::CObservation2DRangeScan::setScanRange(size_t, const float) --> void", pybind11::arg("i"), pybind11::arg("val"));
		cl.def("getScanIntensity", (int & (mrpt::obs::CObservation2DRangeScan::*)(size_t)) &mrpt::obs::CObservation2DRangeScan::getScanIntensity, "C++: mrpt::obs::CObservation2DRangeScan::getScanIntensity(size_t) --> int &", pybind11::return_value_policy::automatic, pybind11::arg("i"));
		cl.def("setScanIntensity", (void (mrpt::obs::CObservation2DRangeScan::*)(size_t, const int)) &mrpt::obs::CObservation2DRangeScan::setScanIntensity, "C++: mrpt::obs::CObservation2DRangeScan::setScanIntensity(size_t, const int) --> void", pybind11::arg("i"), pybind11::arg("val"));
		cl.def("getScanRangeValidity", (bool (mrpt::obs::CObservation2DRangeScan::*)(size_t) const) &mrpt::obs::CObservation2DRangeScan::getScanRangeValidity, "It's false (=0) on no reflected rays, referenced to elements in \n   \n\nC++: mrpt::obs::CObservation2DRangeScan::getScanRangeValidity(size_t) const --> bool", pybind11::arg("i"));
		cl.def("setScanRangeValidity", (void (mrpt::obs::CObservation2DRangeScan::*)(size_t, const bool)) &mrpt::obs::CObservation2DRangeScan::setScanRangeValidity, "C++: mrpt::obs::CObservation2DRangeScan::setScanRangeValidity(size_t, const bool) --> void", pybind11::arg("i"), pybind11::arg("val"));
		cl.def("getScanAngle", (float (mrpt::obs::CObservation2DRangeScan::*)(size_t) const) &mrpt::obs::CObservation2DRangeScan::getScanAngle, "Returns the computed direction (relative heading in radians, with\n 0=forward) of the given ray index, following the following formula:\n \n\n\n\n\n\n\n\n\n\n\n \n Index of the ray, from `0` to `size()-1`.\n\n \n (New in MRPT 2.3.1)\n\nC++: mrpt::obs::CObservation2DRangeScan::getScanAngle(size_t) const --> float", pybind11::arg("idx"));
		cl.def("getScanProperties", (void (mrpt::obs::CObservation2DRangeScan::*)(struct mrpt::obs::T2DScanProperties &) const) &mrpt::obs::CObservation2DRangeScan::getScanProperties, "Fill out a T2DScanProperties structure with the parameters of this scan\n\nC++: mrpt::obs::CObservation2DRangeScan::getScanProperties(struct mrpt::obs::T2DScanProperties &) const --> void", pybind11::arg("p"));
		cl.def("loadFromVectors", (void (mrpt::obs::CObservation2DRangeScan::*)(size_t, const float *, const char *)) &mrpt::obs::CObservation2DRangeScan::loadFromVectors, "@} \n\nC++: mrpt::obs::CObservation2DRangeScan::loadFromVectors(size_t, const float *, const char *) --> void", pybind11::arg("nRays"), pybind11::arg("scanRanges"), pybind11::arg("scanValidity"));
		cl.def("isPlanarScan", [](mrpt::obs::CObservation2DRangeScan const &o) -> bool { return o.isPlanarScan(); }, "");
		cl.def("isPlanarScan", (bool (mrpt::obs::CObservation2DRangeScan::*)(const double) const) &mrpt::obs::CObservation2DRangeScan::isPlanarScan, "Return true if the laser scanner is \"horizontal\", so it has an absolute\n value of \"pitch\" and \"roll\" less or equal to the given tolerance (in\n rads, default=0) (with the normal vector either upwards or downwards).\n\nC++: mrpt::obs::CObservation2DRangeScan::isPlanarScan(const double) const --> bool", pybind11::arg("tolerance"));
		cl.def("hasIntensity", (bool (mrpt::obs::CObservation2DRangeScan::*)() const) &mrpt::obs::CObservation2DRangeScan::hasIntensity, "Return true if scan has intensity \n\nC++: mrpt::obs::CObservation2DRangeScan::hasIntensity() const --> bool");
		cl.def("setScanHasIntensity", (void (mrpt::obs::CObservation2DRangeScan::*)(bool)) &mrpt::obs::CObservation2DRangeScan::setScanHasIntensity, "Marks this scan as having or not intensity data. \n\nC++: mrpt::obs::CObservation2DRangeScan::setScanHasIntensity(bool) --> void", pybind11::arg("setHasIntensityFlag"));
		cl.def("getSensorPose", (void (mrpt::obs::CObservation2DRangeScan::*)(class mrpt::poses::CPose3D &) const) &mrpt::obs::CObservation2DRangeScan::getSensorPose, "C++: mrpt::obs::CObservation2DRangeScan::getSensorPose(class mrpt::poses::CPose3D &) const --> void", pybind11::arg("out_sensorPose"));
		cl.def("setSensorPose", (void (mrpt::obs::CObservation2DRangeScan::*)(const class mrpt::poses::CPose3D &)) &mrpt::obs::CObservation2DRangeScan::setSensorPose, "C++: mrpt::obs::CObservation2DRangeScan::setSensorPose(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newSensorPose"));
		cl.def("truncateByDistanceAndAngle", [](mrpt::obs::CObservation2DRangeScan &o, float const & a0, float const & a1) -> void { return o.truncateByDistanceAndAngle(a0, a1); }, "", pybind11::arg("min_distance"), pybind11::arg("max_angle"));
		cl.def("truncateByDistanceAndAngle", [](mrpt::obs::CObservation2DRangeScan &o, float const & a0, float const & a1, float const & a2) -> void { return o.truncateByDistanceAndAngle(a0, a1, a2); }, "", pybind11::arg("min_distance"), pybind11::arg("max_angle"), pybind11::arg("min_height"));
		cl.def("truncateByDistanceAndAngle", [](mrpt::obs::CObservation2DRangeScan &o, float const & a0, float const & a1, float const & a2, float const & a3) -> void { return o.truncateByDistanceAndAngle(a0, a1, a2, a3); }, "", pybind11::arg("min_distance"), pybind11::arg("max_angle"), pybind11::arg("min_height"), pybind11::arg("max_height"));
		cl.def("truncateByDistanceAndAngle", (void (mrpt::obs::CObservation2DRangeScan::*)(float, float, float, float, float)) &mrpt::obs::CObservation2DRangeScan::truncateByDistanceAndAngle, "A general method to truncate the scan by defining a minimum valid\n   distance and a maximum valid angle as well as minimun and maximum heights\n   (NOTE: the laser z-coordinate must be provided).\n\nC++: mrpt::obs::CObservation2DRangeScan::truncateByDistanceAndAngle(float, float, float, float, float) --> void", pybind11::arg("min_distance"), pybind11::arg("max_angle"), pybind11::arg("min_height"), pybind11::arg("max_height"), pybind11::arg("h"));
		cl.def("exportTxtSupported", (bool (mrpt::obs::CObservation2DRangeScan::*)() const) &mrpt::obs::CObservation2DRangeScan::exportTxtSupported, "C++: mrpt::obs::CObservation2DRangeScan::exportTxtSupported() const --> bool");
		cl.def("exportTxtHeader", (std::string (mrpt::obs::CObservation2DRangeScan::*)() const) &mrpt::obs::CObservation2DRangeScan::exportTxtHeader, "C++: mrpt::obs::CObservation2DRangeScan::exportTxtHeader() const --> std::string");
		cl.def("exportTxtDataRow", (std::string (mrpt::obs::CObservation2DRangeScan::*)() const) &mrpt::obs::CObservation2DRangeScan::exportTxtDataRow, "C++: mrpt::obs::CObservation2DRangeScan::exportTxtDataRow() const --> std::string");
		cl.def("assign", (class mrpt::obs::CObservation2DRangeScan & (mrpt::obs::CObservation2DRangeScan::*)(const class mrpt::obs::CObservation2DRangeScan &)) &mrpt::obs::CObservation2DRangeScan::operator=, "C++: mrpt::obs::CObservation2DRangeScan::operator=(const class mrpt::obs::CObservation2DRangeScan &) --> class mrpt::obs::CObservation2DRangeScan &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
