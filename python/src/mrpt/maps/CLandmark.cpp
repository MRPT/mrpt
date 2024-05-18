#include <iterator>
#include <memory>
#include <mrpt/maps/CLandmark.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPointPDF.h>
#include <mrpt/poses/CPointPDFGaussian.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/typemeta/static_string.h>
#include <mrpt/vision/types.h>
#include <sstream> // __str__
#include <string>
#include <tuple>
#include <variant>

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

// mrpt::maps::CLandmark file:mrpt/maps/CLandmark.h line:35
struct PyCallBack_mrpt_maps_CLandmark : public mrpt::maps::CLandmark {
	using mrpt::maps::CLandmark::CLandmark;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CLandmark *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CLandmark::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CLandmark *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CLandmark::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CLandmark *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CLandmark::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CLandmark *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CLandmark::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CLandmark *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CLandmark::serializeFrom(a0, a1);
	}
};

void bind_mrpt_maps_CLandmark(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::maps::CLandmark file:mrpt/maps/CLandmark.h line:35
		pybind11::class_<mrpt::maps::CLandmark, std::shared_ptr<mrpt::maps::CLandmark>, PyCallBack_mrpt_maps_CLandmark, mrpt::serialization::CSerializable> cl(M("mrpt::maps"), "CLandmark", "The class for storing \"landmarks\" (visual or laser-scan-extracted\n features,...)\n\n  The descriptors for each kind of descriptor are stored in the vector\n \"features\", which\n   will typically consists of only 1 element, or 2 elements for landmarks\n obtained from stereo images.\n\n \n CLandmarksMap\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::maps::CLandmark(); }, [](){ return new PyCallBack_mrpt_maps_CLandmark(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_maps_CLandmark const &o){ return new PyCallBack_mrpt_maps_CLandmark(o); } ) );
		cl.def( pybind11::init( [](mrpt::maps::CLandmark const &o){ return new mrpt::maps::CLandmark(o); } ) );
		cl.def_readwrite("features", &mrpt::maps::CLandmark::features);
		cl.def_readwrite("pose_mean", &mrpt::maps::CLandmark::pose_mean);
		cl.def_readwrite("normal", &mrpt::maps::CLandmark::normal);
		cl.def_readwrite("pose_cov_11", &mrpt::maps::CLandmark::pose_cov_11);
		cl.def_readwrite("pose_cov_22", &mrpt::maps::CLandmark::pose_cov_22);
		cl.def_readwrite("pose_cov_33", &mrpt::maps::CLandmark::pose_cov_33);
		cl.def_readwrite("pose_cov_12", &mrpt::maps::CLandmark::pose_cov_12);
		cl.def_readwrite("pose_cov_13", &mrpt::maps::CLandmark::pose_cov_13);
		cl.def_readwrite("pose_cov_23", &mrpt::maps::CLandmark::pose_cov_23);
		cl.def_readwrite("ID", &mrpt::maps::CLandmark::ID);
		cl.def_readwrite("timestampLastSeen", &mrpt::maps::CLandmark::timestampLastSeen);
		cl.def_readwrite("seenTimesCount", &mrpt::maps::CLandmark::seenTimesCount);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::maps::CLandmark::GetRuntimeClassIdStatic, "C++: mrpt::maps::CLandmark::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::maps::CLandmark::*)() const) &mrpt::maps::CLandmark::GetRuntimeClass, "C++: mrpt::maps::CLandmark::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::maps::CLandmark::*)() const) &mrpt::maps::CLandmark::clone, "C++: mrpt::maps::CLandmark::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::maps::CLandmark::CreateObject, "C++: mrpt::maps::CLandmark::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("getPose", (void (mrpt::maps::CLandmark::*)(class mrpt::poses::CPointPDFGaussian &) const) &mrpt::maps::CLandmark::getPose, "Returns the pose as an object:\n\nC++: mrpt::maps::CLandmark::getPose(class mrpt::poses::CPointPDFGaussian &) const --> void", pybind11::arg("p"));
		cl.def("getPose", (void (mrpt::maps::CLandmark::*)(class mrpt::poses::CPoint3D &, class mrpt::math::CMatrixDynamic<double> &) const) &mrpt::maps::CLandmark::getPose, "C++: mrpt::maps::CLandmark::getPose(class mrpt::poses::CPoint3D &, class mrpt::math::CMatrixDynamic<double> &) const --> void", pybind11::arg("p"), pybind11::arg("COV"));
		cl.def("setPose", (void (mrpt::maps::CLandmark::*)(const class mrpt::poses::CPointPDFGaussian &)) &mrpt::maps::CLandmark::setPose, "Sets the pose from an object:\n\nC++: mrpt::maps::CLandmark::setPose(const class mrpt::poses::CPointPDFGaussian &) --> void", pybind11::arg("p"));
		cl.def("getType", (enum mrpt::vision::TKeyPointMethod (mrpt::maps::CLandmark::*)() const) &mrpt::maps::CLandmark::getType, "Gets the type of the first feature in its feature vector. The vector\n must not be empty.\n\nC++: mrpt::maps::CLandmark::getType() const --> enum mrpt::vision::TKeyPointMethod");
		cl.def("createOneFeature", (void (mrpt::maps::CLandmark::*)()) &mrpt::maps::CLandmark::createOneFeature, "Creates one feature in the vector \"features\", calling the appropriate\n constructor of the smart pointer, so after calling this method\n \"features[0]\" is a valid pointer to a CFeature object.\n\nC++: mrpt::maps::CLandmark::createOneFeature() --> void");
		cl.def("assign", (class mrpt::maps::CLandmark & (mrpt::maps::CLandmark::*)(const class mrpt::maps::CLandmark &)) &mrpt::maps::CLandmark::operator=, "C++: mrpt::maps::CLandmark::operator=(const class mrpt::maps::CLandmark &) --> class mrpt::maps::CLandmark &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
