#include <iterator>
#include <memory>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/obs/CAction.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPose3DQuatPDFGaussian.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ostream>
#include <sstream> // __str__
#include <string>
#include <tuple>
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

// mrpt::obs::CActionCollection file:mrpt/obs/CActionCollection.h line:26
struct PyCallBack_mrpt_obs_CActionCollection : public mrpt::obs::CActionCollection {
	using mrpt::obs::CActionCollection::CActionCollection;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CActionCollection *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CActionCollection::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CActionCollection *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CActionCollection::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CActionCollection *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CActionCollection::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CActionCollection *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CActionCollection::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CActionCollection *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CActionCollection::serializeFrom(a0, a1);
	}
};

void bind_mrpt_obs_CActionCollection(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::obs::CActionCollection file:mrpt/obs/CActionCollection.h line:26
		pybind11::class_<mrpt::obs::CActionCollection, std::shared_ptr<mrpt::obs::CActionCollection>, PyCallBack_mrpt_obs_CActionCollection, mrpt::serialization::CSerializable> cl(M("mrpt::obs"), "CActionCollection", "Declares a class for storing a collection of robot actions. It is used in\n mrpt::obs::CRawlog,\n    for logs storage and particle filter based simulations.\n\n \n CAction, CRawlog\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::obs::CActionCollection(); }, [](){ return new PyCallBack_mrpt_obs_CActionCollection(); } ) );
		cl.def( pybind11::init<class mrpt::obs::CAction &>(), pybind11::arg("a") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_CActionCollection const &o){ return new PyCallBack_mrpt_obs_CActionCollection(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::CActionCollection const &o){ return new mrpt::obs::CActionCollection(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::obs::CActionCollection::GetRuntimeClassIdStatic, "C++: mrpt::obs::CActionCollection::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::obs::CActionCollection::*)() const) &mrpt::obs::CActionCollection::GetRuntimeClass, "C++: mrpt::obs::CActionCollection::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::obs::CActionCollection::*)() const) &mrpt::obs::CActionCollection::clone, "C++: mrpt::obs::CActionCollection::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::obs::CActionCollection::CreateObject, "C++: mrpt::obs::CActionCollection::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("clear", (void (mrpt::obs::CActionCollection::*)()) &mrpt::obs::CActionCollection::clear, "Erase all actions from the list \n\nC++: mrpt::obs::CActionCollection::clear() --> void");
		cl.def("get", (class std::shared_ptr<class mrpt::obs::CAction> (mrpt::obs::CActionCollection::*)(size_t)) &mrpt::obs::CActionCollection::get, "Access the i'th action.DO NOT MODIFY the returned object, make a copy of\n ir with \"CSerializable::duplicate\" if desired.\n  First element is 0.\n \n\n std::exception On index out of bounds.\n\nC++: mrpt::obs::CActionCollection::get(size_t) --> class std::shared_ptr<class mrpt::obs::CAction>", pybind11::arg("index"));
		cl.def("insert", (void (mrpt::obs::CActionCollection::*)(const class mrpt::obs::CAction &)) &mrpt::obs::CActionCollection::insert, "Add a new object to the list, making a deep copy. \n\nC++: mrpt::obs::CActionCollection::insert(const class mrpt::obs::CAction &) --> void", pybind11::arg("action"));
		cl.def("insertPtr", (void (mrpt::obs::CActionCollection::*)(const class std::shared_ptr<class mrpt::obs::CAction> &)) &mrpt::obs::CActionCollection::insertPtr, "Add a new object to the list (no deep copy).\n \n\n (New in MRPT 2.3.1) \n\nC++: mrpt::obs::CActionCollection::insertPtr(const class std::shared_ptr<class mrpt::obs::CAction> &) --> void", pybind11::arg("action"));
		cl.def("size", (size_t (mrpt::obs::CActionCollection::*)() const) &mrpt::obs::CActionCollection::size, "Returns the actions count in the collection.\n\nC++: mrpt::obs::CActionCollection::size() const --> size_t");
		cl.def("getBestMovementEstimation", (class std::shared_ptr<class mrpt::obs::CActionRobotMovement2D> (mrpt::obs::CActionCollection::*)() const) &mrpt::obs::CActionCollection::getBestMovementEstimation, "Returns the best pose increment estimator in the collection, based on\n the determinant of its pose change covariance matrix.\n \n\n The estimation, or nullptr if none is available.\n\nC++: mrpt::obs::CActionCollection::getBestMovementEstimation() const --> class std::shared_ptr<class mrpt::obs::CActionRobotMovement2D>");
		cl.def("getMovementEstimationByType", (class std::shared_ptr<class mrpt::obs::CActionRobotMovement2D> (mrpt::obs::CActionCollection::*)(enum mrpt::obs::CActionRobotMovement2D::TEstimationMethod)) &mrpt::obs::CActionCollection::getMovementEstimationByType, "Returns the pose increment estimator in the collection having the\n specified type.\n \n\n The estimation, or nullptr if none is available.\n\nC++: mrpt::obs::CActionCollection::getMovementEstimationByType(enum mrpt::obs::CActionRobotMovement2D::TEstimationMethod) --> class std::shared_ptr<class mrpt::obs::CActionRobotMovement2D>", pybind11::arg("method"));
		cl.def("getFirstMovementEstimationMean", (bool (mrpt::obs::CActionCollection::*)(class mrpt::poses::CPose3D &) const) &mrpt::obs::CActionCollection::getFirstMovementEstimationMean, "Look for the first 2D or 3D \"odometry\" found in this collection of\n actions, and return the \"mean\" increment of the robot according to it.\n \n\n true on success,false on no odometry found.\n\nC++: mrpt::obs::CActionCollection::getFirstMovementEstimationMean(class mrpt::poses::CPose3D &) const --> bool", pybind11::arg("out_pose_increment"));
		cl.def("getFirstMovementEstimation", (bool (mrpt::obs::CActionCollection::*)(class mrpt::poses::CPose3DPDFGaussian &) const) &mrpt::obs::CActionCollection::getFirstMovementEstimation, "Look for the first 2D or 3D \"odometry\" found in this collection of\n actions, and return the \"mean\" increment of the robot and its covariance\n according to it.\n \n\n true on success,false on no odometry found.\n\nC++: mrpt::obs::CActionCollection::getFirstMovementEstimation(class mrpt::poses::CPose3DPDFGaussian &) const --> bool", pybind11::arg("out_pose_increment"));
		cl.def("eraseByIndex", (void (mrpt::obs::CActionCollection::*)(size_t)) &mrpt::obs::CActionCollection::eraseByIndex, "Remove an action from the list by its index.\n \n\n std::exception On index out of bounds.\n\nC++: mrpt::obs::CActionCollection::eraseByIndex(size_t) --> void", pybind11::arg("index"));
		cl.def("assign", (class mrpt::obs::CActionCollection & (mrpt::obs::CActionCollection::*)(const class mrpt::obs::CActionCollection &)) &mrpt::obs::CActionCollection::operator=, "C++: mrpt::obs::CActionCollection::operator=(const class mrpt::obs::CActionCollection &) --> class mrpt::obs::CActionCollection &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
