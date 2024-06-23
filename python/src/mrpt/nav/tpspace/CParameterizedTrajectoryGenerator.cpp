#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/kinematics/CVehicleVelCmd.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CPolygon.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TSegment3D.h>
#include <mrpt/math/TTwist2D.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/nav/holonomic/CAbstractHolonomicReactiveMethod.h>
#include <mrpt/nav/holonomic/ClearanceDiagram.h>
#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/typemeta/static_string.h>
#include <sstream> // __str__
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

// mrpt::nav::CHolonomicLogFileRecord file: line:24
struct PyCallBack_mrpt_nav_CHolonomicLogFileRecord : public mrpt::nav::CHolonomicLogFileRecord {
	using mrpt::nav::CHolonomicLogFileRecord::CHolonomicLogFileRecord;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicLogFileRecord *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CHolonomicLogFileRecord::GetRuntimeClass();
	}
	const class mrpt::math::CMatrixD * getDirectionScores() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicLogFileRecord *>(this), "getDirectionScores");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const class mrpt::math::CMatrixD *>::value) {
				static pybind11::detail::override_caster_t<const class mrpt::math::CMatrixD *> caster;
				return pybind11::detail::cast_ref<const class mrpt::math::CMatrixD *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const class mrpt::math::CMatrixD *>(std::move(o));
		}
		return CHolonomicLogFileRecord::getDirectionScores();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicLogFileRecord *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CSerializable::serializeGetVersion\"");
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicLogFileRecord *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CSerializable::serializeTo\"");
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicLogFileRecord *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CSerializable::serializeFrom\"");
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CHolonomicLogFileRecord *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CObject::clone\"");
	}
};

// mrpt::nav::CAbstractHolonomicReactiveMethod file:mrpt/nav/holonomic/CAbstractHolonomicReactiveMethod.h line:29
struct PyCallBack_mrpt_nav_CAbstractHolonomicReactiveMethod : public mrpt::nav::CAbstractHolonomicReactiveMethod {
	using mrpt::nav::CAbstractHolonomicReactiveMethod::CAbstractHolonomicReactiveMethod;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractHolonomicReactiveMethod *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CAbstractHolonomicReactiveMethod::GetRuntimeClass();
	}
	void navigate(const struct mrpt::nav::CAbstractHolonomicReactiveMethod::NavInput & a0, struct mrpt::nav::CAbstractHolonomicReactiveMethod::NavOutput & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractHolonomicReactiveMethod *>(this), "navigate");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CAbstractHolonomicReactiveMethod::navigate\"");
	}
	void initialize(const class mrpt::config::CConfigFileBase & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractHolonomicReactiveMethod *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CAbstractHolonomicReactiveMethod::initialize\"");
	}
	void saveConfigFile(class mrpt::config::CConfigFileBase & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractHolonomicReactiveMethod *>(this), "saveConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CAbstractHolonomicReactiveMethod::saveConfigFile\"");
	}
	double getTargetApproachSlowDownDistance() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractHolonomicReactiveMethod *>(this), "getTargetApproachSlowDownDistance");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CAbstractHolonomicReactiveMethod::getTargetApproachSlowDownDistance\"");
	}
	void setTargetApproachSlowDownDistance(const double a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractHolonomicReactiveMethod *>(this), "setTargetApproachSlowDownDistance");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CAbstractHolonomicReactiveMethod::setTargetApproachSlowDownDistance\"");
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractHolonomicReactiveMethod *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CSerializable::serializeGetVersion\"");
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractHolonomicReactiveMethod *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CSerializable::serializeTo\"");
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractHolonomicReactiveMethod *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CSerializable::serializeFrom\"");
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractHolonomicReactiveMethod *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CObject::clone\"");
	}
};

void bind_mrpt_nav_tpspace_CParameterizedTrajectoryGenerator(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::nav::CPTG_RobotShape_Polygonal file:mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h line:501
		pybind11::class_<mrpt::nav::CPTG_RobotShape_Polygonal, std::shared_ptr<mrpt::nav::CPTG_RobotShape_Polygonal>, mrpt::nav::CParameterizedTrajectoryGenerator> cl(M("mrpt::nav"), "CPTG_RobotShape_Polygonal", "Base class for all PTGs using a 2D polygonal robot shape model.\n  \n\n\n ");
		cl.def("setRobotShape", (void (mrpt::nav::CPTG_RobotShape_Polygonal::*)(const class mrpt::math::CPolygon &)) &mrpt::nav::CPTG_RobotShape_Polygonal::setRobotShape, "@{ *\n\n Robot shape must be set before initialization, either from ctor params\n or via this method. \n\nC++: mrpt::nav::CPTG_RobotShape_Polygonal::setRobotShape(const class mrpt::math::CPolygon &) --> void", pybind11::arg("robotShape"));
		cl.def("getRobotShape", (const class mrpt::math::CPolygon & (mrpt::nav::CPTG_RobotShape_Polygonal::*)() const) &mrpt::nav::CPTG_RobotShape_Polygonal::getRobotShape, "C++: mrpt::nav::CPTG_RobotShape_Polygonal::getRobotShape() const --> const class mrpt::math::CPolygon &", pybind11::return_value_policy::automatic);
		cl.def("getMaxRobotRadius", (double (mrpt::nav::CPTG_RobotShape_Polygonal::*)() const) &mrpt::nav::CPTG_RobotShape_Polygonal::getMaxRobotRadius, "C++: mrpt::nav::CPTG_RobotShape_Polygonal::getMaxRobotRadius() const --> double");
		cl.def("evalClearanceToRobotShape", (double (mrpt::nav::CPTG_RobotShape_Polygonal::*)(const double, const double) const) &mrpt::nav::CPTG_RobotShape_Polygonal::evalClearanceToRobotShape, "C++: mrpt::nav::CPTG_RobotShape_Polygonal::evalClearanceToRobotShape(const double, const double) const --> double", pybind11::arg("ox"), pybind11::arg("oy"));
		cl.def("isPointInsideRobotShape", (bool (mrpt::nav::CPTG_RobotShape_Polygonal::*)(const double, const double) const) &mrpt::nav::CPTG_RobotShape_Polygonal::isPointInsideRobotShape, "@} \n\nC++: mrpt::nav::CPTG_RobotShape_Polygonal::isPointInsideRobotShape(const double, const double) const --> bool", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("add_robotShape_to_setOfLines", [](mrpt::nav::CPTG_RobotShape_Polygonal const &o, class mrpt::opengl::CSetOfLines & a0) -> void { return o.add_robotShape_to_setOfLines(a0); }, "", pybind11::arg("gl_shape"));
		cl.def("add_robotShape_to_setOfLines", (void (mrpt::nav::CPTG_RobotShape_Polygonal::*)(class mrpt::opengl::CSetOfLines &, const class mrpt::poses::CPose2D &) const) &mrpt::nav::CPTG_RobotShape_Polygonal::add_robotShape_to_setOfLines, "C++: mrpt::nav::CPTG_RobotShape_Polygonal::add_robotShape_to_setOfLines(class mrpt::opengl::CSetOfLines &, const class mrpt::poses::CPose2D &) const --> void", pybind11::arg("gl_shape"), pybind11::arg("origin"));
		cl.def_static("static_add_robotShape_to_setOfLines", (void (*)(class mrpt::opengl::CSetOfLines &, const class mrpt::poses::CPose2D &, const class mrpt::math::CPolygon &)) &mrpt::nav::CPTG_RobotShape_Polygonal::static_add_robotShape_to_setOfLines, "C++: mrpt::nav::CPTG_RobotShape_Polygonal::static_add_robotShape_to_setOfLines(class mrpt::opengl::CSetOfLines &, const class mrpt::poses::CPose2D &, const class mrpt::math::CPolygon &) --> void", pybind11::arg("gl_shape"), pybind11::arg("origin"), pybind11::arg("robotShape"));
		cl.def("assign", (class mrpt::nav::CPTG_RobotShape_Polygonal & (mrpt::nav::CPTG_RobotShape_Polygonal::*)(const class mrpt::nav::CPTG_RobotShape_Polygonal &)) &mrpt::nav::CPTG_RobotShape_Polygonal::operator=, "C++: mrpt::nav::CPTG_RobotShape_Polygonal::operator=(const class mrpt::nav::CPTG_RobotShape_Polygonal &) --> class mrpt::nav::CPTG_RobotShape_Polygonal &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::nav::CPTG_RobotShape_Circular file:mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h line:545
		pybind11::class_<mrpt::nav::CPTG_RobotShape_Circular, std::shared_ptr<mrpt::nav::CPTG_RobotShape_Circular>, mrpt::nav::CParameterizedTrajectoryGenerator> cl(M("mrpt::nav"), "CPTG_RobotShape_Circular", "Base class for all PTGs using a 2D circular robot shape model.\n  \n\n\n ");
		cl.def("setRobotShapeRadius", (void (mrpt::nav::CPTG_RobotShape_Circular::*)(const double)) &mrpt::nav::CPTG_RobotShape_Circular::setRobotShapeRadius, "@{ *\n\n Robot shape must be set before initialization, either from ctor params\n or via this method. \n\nC++: mrpt::nav::CPTG_RobotShape_Circular::setRobotShapeRadius(const double) --> void", pybind11::arg("robot_radius"));
		cl.def("getRobotShapeRadius", (double (mrpt::nav::CPTG_RobotShape_Circular::*)() const) &mrpt::nav::CPTG_RobotShape_Circular::getRobotShapeRadius, "C++: mrpt::nav::CPTG_RobotShape_Circular::getRobotShapeRadius() const --> double");
		cl.def("getMaxRobotRadius", (double (mrpt::nav::CPTG_RobotShape_Circular::*)() const) &mrpt::nav::CPTG_RobotShape_Circular::getMaxRobotRadius, "C++: mrpt::nav::CPTG_RobotShape_Circular::getMaxRobotRadius() const --> double");
		cl.def("evalClearanceToRobotShape", (double (mrpt::nav::CPTG_RobotShape_Circular::*)(const double, const double) const) &mrpt::nav::CPTG_RobotShape_Circular::evalClearanceToRobotShape, "C++: mrpt::nav::CPTG_RobotShape_Circular::evalClearanceToRobotShape(const double, const double) const --> double", pybind11::arg("ox"), pybind11::arg("oy"));
		cl.def("add_robotShape_to_setOfLines", [](mrpt::nav::CPTG_RobotShape_Circular const &o, class mrpt::opengl::CSetOfLines & a0) -> void { return o.add_robotShape_to_setOfLines(a0); }, "", pybind11::arg("gl_shape"));
		cl.def("add_robotShape_to_setOfLines", (void (mrpt::nav::CPTG_RobotShape_Circular::*)(class mrpt::opengl::CSetOfLines &, const class mrpt::poses::CPose2D &) const) &mrpt::nav::CPTG_RobotShape_Circular::add_robotShape_to_setOfLines, "@} \n\nC++: mrpt::nav::CPTG_RobotShape_Circular::add_robotShape_to_setOfLines(class mrpt::opengl::CSetOfLines &, const class mrpt::poses::CPose2D &) const --> void", pybind11::arg("gl_shape"), pybind11::arg("origin"));
		cl.def_static("static_add_robotShape_to_setOfLines", (void (*)(class mrpt::opengl::CSetOfLines &, const class mrpt::poses::CPose2D &, const double)) &mrpt::nav::CPTG_RobotShape_Circular::static_add_robotShape_to_setOfLines, "C++: mrpt::nav::CPTG_RobotShape_Circular::static_add_robotShape_to_setOfLines(class mrpt::opengl::CSetOfLines &, const class mrpt::poses::CPose2D &, const double) --> void", pybind11::arg("gl_shape"), pybind11::arg("origin"), pybind11::arg("robotRadius"));
		cl.def("isPointInsideRobotShape", (bool (mrpt::nav::CPTG_RobotShape_Circular::*)(const double, const double) const) &mrpt::nav::CPTG_RobotShape_Circular::isPointInsideRobotShape, "C++: mrpt::nav::CPTG_RobotShape_Circular::isPointInsideRobotShape(const double, const double) const --> bool", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("assign", (class mrpt::nav::CPTG_RobotShape_Circular & (mrpt::nav::CPTG_RobotShape_Circular::*)(const class mrpt::nav::CPTG_RobotShape_Circular &)) &mrpt::nav::CPTG_RobotShape_Circular::operator=, "C++: mrpt::nav::CPTG_RobotShape_Circular::operator=(const class mrpt::nav::CPTG_RobotShape_Circular &) --> class mrpt::nav::CPTG_RobotShape_Circular &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::nav::CHolonomicLogFileRecord file: line:24
		pybind11::class_<mrpt::nav::CHolonomicLogFileRecord, std::shared_ptr<mrpt::nav::CHolonomicLogFileRecord>, PyCallBack_mrpt_nav_CHolonomicLogFileRecord, mrpt::serialization::CSerializable> cl(M("mrpt::nav"), "CHolonomicLogFileRecord", "A base class for log records for different holonomic navigation methods.\n\n \n CReactiveNavigationSystem, CHolonomicLogFileRecord");
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_nav_CHolonomicLogFileRecord(); } ) );
		cl.def(pybind11::init<PyCallBack_mrpt_nav_CHolonomicLogFileRecord const &>());
		cl.def_readwrite("dirs_eval", &mrpt::nav::CHolonomicLogFileRecord::dirs_eval);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::nav::CHolonomicLogFileRecord::*)() const) &mrpt::nav::CHolonomicLogFileRecord::GetRuntimeClass, "C++: mrpt::nav::CHolonomicLogFileRecord::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::nav::CHolonomicLogFileRecord::GetRuntimeClassIdStatic, "C++: mrpt::nav::CHolonomicLogFileRecord::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("getDirectionScores", (const class mrpt::math::CMatrixD * (mrpt::nav::CHolonomicLogFileRecord::*)() const) &mrpt::nav::CHolonomicLogFileRecord::getDirectionScores, "C++: mrpt::nav::CHolonomicLogFileRecord::getDirectionScores() const --> const class mrpt::math::CMatrixD *", pybind11::return_value_policy::automatic);
		cl.def("assign", (class mrpt::nav::CHolonomicLogFileRecord & (mrpt::nav::CHolonomicLogFileRecord::*)(const class mrpt::nav::CHolonomicLogFileRecord &)) &mrpt::nav::CHolonomicLogFileRecord::operator=, "C++: mrpt::nav::CHolonomicLogFileRecord::operator=(const class mrpt::nav::CHolonomicLogFileRecord &) --> class mrpt::nav::CHolonomicLogFileRecord &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::nav::CAbstractHolonomicReactiveMethod file:mrpt/nav/holonomic/CAbstractHolonomicReactiveMethod.h line:29
		pybind11::class_<mrpt::nav::CAbstractHolonomicReactiveMethod, std::shared_ptr<mrpt::nav::CAbstractHolonomicReactiveMethod>, PyCallBack_mrpt_nav_CAbstractHolonomicReactiveMethod, mrpt::serialization::CSerializable> cl(M("mrpt::nav"), "CAbstractHolonomicReactiveMethod", "A base class for holonomic reactive navigation methods.\n  \n\n CHolonomicVFF,CHolonomicND,CHolonomicFullEval, CReactiveNavigationSystem");
		cl.def( pybind11::init<const std::string &>(), pybind11::arg("defaultCfgSectionName") );

		cl.def(pybind11::init<PyCallBack_mrpt_nav_CAbstractHolonomicReactiveMethod const &>());
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::nav::CAbstractHolonomicReactiveMethod::*)() const) &mrpt::nav::CAbstractHolonomicReactiveMethod::GetRuntimeClass, "C++: mrpt::nav::CAbstractHolonomicReactiveMethod::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::nav::CAbstractHolonomicReactiveMethod::GetRuntimeClassIdStatic, "C++: mrpt::nav::CAbstractHolonomicReactiveMethod::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def_static("Factory", (class std::shared_ptr<class mrpt::nav::CAbstractHolonomicReactiveMethod> (*)(const std::string &)) &mrpt::nav::CAbstractHolonomicReactiveMethod::Factory, "C++: mrpt::nav::CAbstractHolonomicReactiveMethod::Factory(const std::string &) --> class std::shared_ptr<class mrpt::nav::CAbstractHolonomicReactiveMethod>", pybind11::arg("className"));
		cl.def("navigate", (void (mrpt::nav::CAbstractHolonomicReactiveMethod::*)(const struct mrpt::nav::CAbstractHolonomicReactiveMethod::NavInput &, struct mrpt::nav::CAbstractHolonomicReactiveMethod::NavOutput &)) &mrpt::nav::CAbstractHolonomicReactiveMethod::navigate, "Invokes the holonomic navigation algorithm itself. See the description\n of the input/output structures for details on each parameter. \n\nC++: mrpt::nav::CAbstractHolonomicReactiveMethod::navigate(const struct mrpt::nav::CAbstractHolonomicReactiveMethod::NavInput &, struct mrpt::nav::CAbstractHolonomicReactiveMethod::NavOutput &) --> void", pybind11::arg("ni"), pybind11::arg("no"));
		cl.def("initialize", (void (mrpt::nav::CAbstractHolonomicReactiveMethod::*)(const class mrpt::config::CConfigFileBase &)) &mrpt::nav::CAbstractHolonomicReactiveMethod::initialize, "Initialize the parameters of the navigator, reading from the default\n section name (see derived classes) or the one set via\n setConfigFileSectionName() \n\nC++: mrpt::nav::CAbstractHolonomicReactiveMethod::initialize(const class mrpt::config::CConfigFileBase &) --> void", pybind11::arg("c"));
		cl.def("saveConfigFile", (void (mrpt::nav::CAbstractHolonomicReactiveMethod::*)(class mrpt::config::CConfigFileBase &) const) &mrpt::nav::CAbstractHolonomicReactiveMethod::saveConfigFile, "saves all available parameters, in a forma loadable by `initialize()` \n\nC++: mrpt::nav::CAbstractHolonomicReactiveMethod::saveConfigFile(class mrpt::config::CConfigFileBase &) const --> void", pybind11::arg("c"));
		cl.def("setConfigFileSectionName", (void (mrpt::nav::CAbstractHolonomicReactiveMethod::*)(const std::string &)) &mrpt::nav::CAbstractHolonomicReactiveMethod::setConfigFileSectionName, "Defines the name of the section used in initialize() \n\nC++: mrpt::nav::CAbstractHolonomicReactiveMethod::setConfigFileSectionName(const std::string &) --> void", pybind11::arg("sectName"));
		cl.def("getConfigFileSectionName", (std::string (mrpt::nav::CAbstractHolonomicReactiveMethod::*)() const) &mrpt::nav::CAbstractHolonomicReactiveMethod::getConfigFileSectionName, "Gets the name of the section used in initialize() \n\nC++: mrpt::nav::CAbstractHolonomicReactiveMethod::getConfigFileSectionName() const --> std::string");
		cl.def("getTargetApproachSlowDownDistance", (double (mrpt::nav::CAbstractHolonomicReactiveMethod::*)() const) &mrpt::nav::CAbstractHolonomicReactiveMethod::getTargetApproachSlowDownDistance, "Returns the actual value of this parameter [m], as set via the children\n class options structure. \n\n setTargetApproachSlowDownDistance() \n\nC++: mrpt::nav::CAbstractHolonomicReactiveMethod::getTargetApproachSlowDownDistance() const --> double");
		cl.def("setTargetApproachSlowDownDistance", (void (mrpt::nav::CAbstractHolonomicReactiveMethod::*)(const double)) &mrpt::nav::CAbstractHolonomicReactiveMethod::setTargetApproachSlowDownDistance, "Sets the actual value of this parameter [m]. \n\n getTargetApproachSlowDownDistance() \n\nC++: mrpt::nav::CAbstractHolonomicReactiveMethod::setTargetApproachSlowDownDistance(const double) --> void", pybind11::arg("dist"));
		cl.def("setAssociatedPTG", (void (mrpt::nav::CAbstractHolonomicReactiveMethod::*)(class mrpt::nav::CParameterizedTrajectoryGenerator *)) &mrpt::nav::CAbstractHolonomicReactiveMethod::setAssociatedPTG, "Optionally, sets the associated PTG, just in case a derived class\n requires this info (not required for methods where the robot kinematics\n are totally abstracted) \n\nC++: mrpt::nav::CAbstractHolonomicReactiveMethod::setAssociatedPTG(class mrpt::nav::CParameterizedTrajectoryGenerator *) --> void", pybind11::arg("ptg"));
		cl.def("getAssociatedPTG", (class mrpt::nav::CParameterizedTrajectoryGenerator * (mrpt::nav::CAbstractHolonomicReactiveMethod::*)() const) &mrpt::nav::CAbstractHolonomicReactiveMethod::getAssociatedPTG, "Returns the pointer set by setAssociatedPTG() \n\nC++: mrpt::nav::CAbstractHolonomicReactiveMethod::getAssociatedPTG() const --> class mrpt::nav::CParameterizedTrajectoryGenerator *", pybind11::return_value_policy::automatic);
		cl.def("enableApproachTargetSlowDown", (void (mrpt::nav::CAbstractHolonomicReactiveMethod::*)(bool)) &mrpt::nav::CAbstractHolonomicReactiveMethod::enableApproachTargetSlowDown, "C++: mrpt::nav::CAbstractHolonomicReactiveMethod::enableApproachTargetSlowDown(bool) --> void", pybind11::arg("enable"));
		cl.def("assign", (class mrpt::nav::CAbstractHolonomicReactiveMethod & (mrpt::nav::CAbstractHolonomicReactiveMethod::*)(const class mrpt::nav::CAbstractHolonomicReactiveMethod &)) &mrpt::nav::CAbstractHolonomicReactiveMethod::operator=, "C++: mrpt::nav::CAbstractHolonomicReactiveMethod::operator=(const class mrpt::nav::CAbstractHolonomicReactiveMethod &) --> class mrpt::nav::CAbstractHolonomicReactiveMethod &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::nav::CAbstractHolonomicReactiveMethod::NavInput file:mrpt/nav/holonomic/CAbstractHolonomicReactiveMethod.h line:34
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::nav::CAbstractHolonomicReactiveMethod::NavInput, std::shared_ptr<mrpt::nav::CAbstractHolonomicReactiveMethod::NavInput>> cl(enclosing_class, "NavInput", "Input parameters for CAbstractHolonomicReactiveMethod::navigate() ");
			cl.def( pybind11::init( [](){ return new mrpt::nav::CAbstractHolonomicReactiveMethod::NavInput(); } ) );
			cl.def( pybind11::init( [](mrpt::nav::CAbstractHolonomicReactiveMethod::NavInput const &o){ return new mrpt::nav::CAbstractHolonomicReactiveMethod::NavInput(o); } ) );
			cl.def_readwrite("obstacles", &mrpt::nav::CAbstractHolonomicReactiveMethod::NavInput::obstacles);
			cl.def_readwrite("targets", &mrpt::nav::CAbstractHolonomicReactiveMethod::NavInput::targets);
			cl.def_readwrite("maxRobotSpeed", &mrpt::nav::CAbstractHolonomicReactiveMethod::NavInput::maxRobotSpeed);
			cl.def_readwrite("maxObstacleDist", &mrpt::nav::CAbstractHolonomicReactiveMethod::NavInput::maxObstacleDist);
			cl.def("assign", (struct mrpt::nav::CAbstractHolonomicReactiveMethod::NavInput & (mrpt::nav::CAbstractHolonomicReactiveMethod::NavInput::*)(const struct mrpt::nav::CAbstractHolonomicReactiveMethod::NavInput &)) &mrpt::nav::CAbstractHolonomicReactiveMethod::NavInput::operator=, "C++: mrpt::nav::CAbstractHolonomicReactiveMethod::NavInput::operator=(const struct mrpt::nav::CAbstractHolonomicReactiveMethod::NavInput &) --> struct mrpt::nav::CAbstractHolonomicReactiveMethod::NavInput &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::nav::CAbstractHolonomicReactiveMethod::NavOutput file:mrpt/nav/holonomic/CAbstractHolonomicReactiveMethod.h line:61
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::nav::CAbstractHolonomicReactiveMethod::NavOutput, std::shared_ptr<mrpt::nav::CAbstractHolonomicReactiveMethod::NavOutput>> cl(enclosing_class, "NavOutput", "Output for CAbstractHolonomicReactiveMethod::navigate() ");
			cl.def( pybind11::init( [](){ return new mrpt::nav::CAbstractHolonomicReactiveMethod::NavOutput(); } ) );
			cl.def( pybind11::init( [](mrpt::nav::CAbstractHolonomicReactiveMethod::NavOutput const &o){ return new mrpt::nav::CAbstractHolonomicReactiveMethod::NavOutput(o); } ) );
			cl.def_readwrite("desiredDirection", &mrpt::nav::CAbstractHolonomicReactiveMethod::NavOutput::desiredDirection);
			cl.def_readwrite("desiredSpeed", &mrpt::nav::CAbstractHolonomicReactiveMethod::NavOutput::desiredSpeed);
			cl.def_readwrite("logRecord", &mrpt::nav::CAbstractHolonomicReactiveMethod::NavOutput::logRecord);
			cl.def("assign", (struct mrpt::nav::CAbstractHolonomicReactiveMethod::NavOutput & (mrpt::nav::CAbstractHolonomicReactiveMethod::NavOutput::*)(const struct mrpt::nav::CAbstractHolonomicReactiveMethod::NavOutput &)) &mrpt::nav::CAbstractHolonomicReactiveMethod::NavOutput::operator=, "C++: mrpt::nav::CAbstractHolonomicReactiveMethod::NavOutput::operator=(const struct mrpt::nav::CAbstractHolonomicReactiveMethod::NavOutput &) --> struct mrpt::nav::CAbstractHolonomicReactiveMethod::NavOutput &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
}
