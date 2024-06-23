#include <iterator>
#include <memory>
#include <mrpt/kinematics/CKinematicChain.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/opengl/CSetOfObjects.h>
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
#include <sstream> // __str__
#include <string>
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

// mrpt::kinematics::CKinematicChain file:mrpt/kinematics/CKinematicChain.h line:68
struct PyCallBack_mrpt_kinematics_CKinematicChain : public mrpt::kinematics::CKinematicChain {
	using mrpt::kinematics::CKinematicChain::CKinematicChain;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CKinematicChain *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CKinematicChain::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CKinematicChain *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CKinematicChain::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CKinematicChain *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CKinematicChain::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CKinematicChain *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CKinematicChain::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::kinematics::CKinematicChain *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CKinematicChain::serializeFrom(a0, a1);
	}
};

void bind_mrpt_kinematics_CKinematicChain(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::kinematics::TKinematicLink file:mrpt/kinematics/CKinematicChain.h line:31
		pybind11::class_<mrpt::kinematics::TKinematicLink, std::shared_ptr<mrpt::kinematics::TKinematicLink>> cl(M("mrpt::kinematics"), "TKinematicLink", "An individual kinematic chain element (one link) which builds up a\n CKinematicChain.\n The parameterization of the SE(3) transformation from the starting point to\n the end point\n follows a Denavit-Hartenberg standard parameterization: [theta, d, a,\n alpha].\n \n\n\n ");
		cl.def( pybind11::init<double, double, double, double, bool>(), pybind11::arg("_theta"), pybind11::arg("_d"), pybind11::arg("_a"), pybind11::arg("_alpha"), pybind11::arg("_is_prismatic") );

		cl.def( pybind11::init( [](){ return new mrpt::kinematics::TKinematicLink(); } ) );
		cl.def_readwrite("theta", &mrpt::kinematics::TKinematicLink::theta);
		cl.def_readwrite("d", &mrpt::kinematics::TKinematicLink::d);
		cl.def_readwrite("a", &mrpt::kinematics::TKinematicLink::a);
		cl.def_readwrite("alpha", &mrpt::kinematics::TKinematicLink::alpha);
		cl.def_readwrite("is_prismatic", &mrpt::kinematics::TKinematicLink::is_prismatic);
	}
	{ // mrpt::kinematics::CKinematicChain file:mrpt/kinematics/CKinematicChain.h line:68
		pybind11::class_<mrpt::kinematics::CKinematicChain, std::shared_ptr<mrpt::kinematics::CKinematicChain>, PyCallBack_mrpt_kinematics_CKinematicChain, mrpt::serialization::CSerializable> cl(M("mrpt::kinematics"), "CKinematicChain", "A open-loop kinematic chain model, suitable to robotic manipulators.\n  Each link is parameterized with standard Denavit-Hartenberg standard\n parameterization [theta, d, a, alpha].\n\n  The orientation of the first link can be modified with setOriginPose(),\n which defaults to standard XYZ axes with +Z pointing upwards.\n\n \n CPose3D\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::kinematics::CKinematicChain(); }, [](){ return new PyCallBack_mrpt_kinematics_CKinematicChain(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_kinematics_CKinematicChain const &o){ return new PyCallBack_mrpt_kinematics_CKinematicChain(o); } ) );
		cl.def( pybind11::init( [](mrpt::kinematics::CKinematicChain const &o){ return new mrpt::kinematics::CKinematicChain(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::kinematics::CKinematicChain::GetRuntimeClassIdStatic, "C++: mrpt::kinematics::CKinematicChain::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::kinematics::CKinematicChain::*)() const) &mrpt::kinematics::CKinematicChain::GetRuntimeClass, "C++: mrpt::kinematics::CKinematicChain::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::kinematics::CKinematicChain::*)() const) &mrpt::kinematics::CKinematicChain::clone, "C++: mrpt::kinematics::CKinematicChain::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::kinematics::CKinematicChain::CreateObject, "C++: mrpt::kinematics::CKinematicChain::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("size", (size_t (mrpt::kinematics::CKinematicChain::*)() const) &mrpt::kinematics::CKinematicChain::size, "Return the number of links \n\nC++: mrpt::kinematics::CKinematicChain::size() const --> size_t");
		cl.def("clear", (void (mrpt::kinematics::CKinematicChain::*)()) &mrpt::kinematics::CKinematicChain::clear, "Erases all links and leave the robot arm empty. \n\nC++: mrpt::kinematics::CKinematicChain::clear() --> void");
		cl.def("addLink", (void (mrpt::kinematics::CKinematicChain::*)(double, double, double, double, bool)) &mrpt::kinematics::CKinematicChain::addLink, "Appends a new link to the robotic arm, with the given Denavit-Hartenberg\n parameters (see TKinematicLink for further details) \n\nC++: mrpt::kinematics::CKinematicChain::addLink(double, double, double, double, bool) --> void", pybind11::arg("theta"), pybind11::arg("d"), pybind11::arg("a"), pybind11::arg("alpha"), pybind11::arg("is_prismatic"));
		cl.def("removeLink", (void (mrpt::kinematics::CKinematicChain::*)(size_t)) &mrpt::kinematics::CKinematicChain::removeLink, "Removes one link from the kinematic chain (0<=idx<N) \n\nC++: mrpt::kinematics::CKinematicChain::removeLink(size_t) --> void", pybind11::arg("idx"));
		cl.def("getLink", (const struct mrpt::kinematics::TKinematicLink & (mrpt::kinematics::CKinematicChain::*)(size_t) const) &mrpt::kinematics::CKinematicChain::getLink, "Get a ref to a given link (read-only) \n\nC++: mrpt::kinematics::CKinematicChain::getLink(size_t) const --> const struct mrpt::kinematics::TKinematicLink &", pybind11::return_value_policy::automatic, pybind11::arg("idx"));
		cl.def("getLinkRef", (struct mrpt::kinematics::TKinematicLink & (mrpt::kinematics::CKinematicChain::*)(size_t)) &mrpt::kinematics::CKinematicChain::getLinkRef, "Get a ref to a given link (read-write) \n\nC++: mrpt::kinematics::CKinematicChain::getLinkRef(size_t) --> struct mrpt::kinematics::TKinematicLink &", pybind11::return_value_policy::automatic, pybind11::arg("idx"));
		cl.def("setOriginPose", (void (mrpt::kinematics::CKinematicChain::*)(const class mrpt::poses::CPose3D &)) &mrpt::kinematics::CKinematicChain::setOriginPose, "Can be used to define a first degree of freedom along a +Z axis which\n does not coincide with the global +Z axis. \n\nC++: mrpt::kinematics::CKinematicChain::setOriginPose(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("new_pose"));
		cl.def("getOriginPose", (const class mrpt::poses::CPose3D & (mrpt::kinematics::CKinematicChain::*)() const) &mrpt::kinematics::CKinematicChain::getOriginPose, "Returns the current pose of the first link. \n\nC++: mrpt::kinematics::CKinematicChain::getOriginPose() const --> const class mrpt::poses::CPose3D &", pybind11::return_value_policy::automatic);
		cl.def("assign", (class mrpt::kinematics::CKinematicChain & (mrpt::kinematics::CKinematicChain::*)(const class mrpt::kinematics::CKinematicChain &)) &mrpt::kinematics::CKinematicChain::operator=, "C++: mrpt::kinematics::CKinematicChain::operator=(const class mrpt::kinematics::CKinematicChain &) --> class mrpt::kinematics::CKinematicChain &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
