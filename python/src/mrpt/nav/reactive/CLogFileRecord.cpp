#include <iterator>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/nav/reactive/CLogFileRecord.h>
#include <mrpt/nav/reactive/CMultiObjectiveMotionOptimizerBase.h>
#include <mrpt/nav/reactive/TCandidateMovementPTG.h>
#include <mrpt/nav/reactive/TWaypoint.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/COutputLogger.h>
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

// mrpt::nav::CLogFileRecord file:mrpt/nav/reactive/CLogFileRecord.h line:30
struct PyCallBack_mrpt_nav_CLogFileRecord : public mrpt::nav::CLogFileRecord {
	using mrpt::nav::CLogFileRecord::CLogFileRecord;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CLogFileRecord *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CLogFileRecord::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CLogFileRecord *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CLogFileRecord::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CLogFileRecord *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CLogFileRecord::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CLogFileRecord *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CLogFileRecord::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CLogFileRecord *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CLogFileRecord::serializeFrom(a0, a1);
	}
};

// mrpt::nav::CMultiObjectiveMotionOptimizerBase::TParamsBase file:mrpt/nav/reactive/CMultiObjectiveMotionOptimizerBase.h line:56
struct PyCallBack_mrpt_nav_CMultiObjectiveMotionOptimizerBase_TParamsBase : public mrpt::nav::CMultiObjectiveMotionOptimizerBase::TParamsBase {
	using mrpt::nav::CMultiObjectiveMotionOptimizerBase::TParamsBase::TParamsBase;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CMultiObjectiveMotionOptimizerBase::TParamsBase *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TParamsBase::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CMultiObjectiveMotionOptimizerBase::TParamsBase *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TParamsBase::saveToConfigFile(a0, a1);
	}
};

void bind_mrpt_nav_reactive_CLogFileRecord(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::nav::CLogFileRecord file:mrpt/nav/reactive/CLogFileRecord.h line:30
		pybind11::class_<mrpt::nav::CLogFileRecord, std::shared_ptr<mrpt::nav::CLogFileRecord>, PyCallBack_mrpt_nav_CLogFileRecord, mrpt::serialization::CSerializable> cl(M("mrpt::nav"), "CLogFileRecord", "A class for storing, saving and loading a reactive navigation\n   log record for the CReactiveNavigationSystem class.\n \n\n CReactiveNavigationSystem, CHolonomicLogFileRecord\n  \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::nav::CLogFileRecord(); }, [](){ return new PyCallBack_mrpt_nav_CLogFileRecord(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_nav_CLogFileRecord const &o){ return new PyCallBack_mrpt_nav_CLogFileRecord(o); } ) );
		cl.def( pybind11::init( [](mrpt::nav::CLogFileRecord const &o){ return new mrpt::nav::CLogFileRecord(o); } ) );
		cl.def_readwrite("navDynState", &mrpt::nav::CLogFileRecord::navDynState);
		cl.def_readwrite("nPTGs", &mrpt::nav::CLogFileRecord::nPTGs);
		cl.def_readwrite("infoPerPTG", &mrpt::nav::CLogFileRecord::infoPerPTG);
		cl.def_readwrite("nSelectedPTG", &mrpt::nav::CLogFileRecord::nSelectedPTG);
		cl.def_readwrite("values", &mrpt::nav::CLogFileRecord::values);
		cl.def_readwrite("timestamps", &mrpt::nav::CLogFileRecord::timestamps);
		cl.def_readwrite("additional_debug_msgs", &mrpt::nav::CLogFileRecord::additional_debug_msgs);
		cl.def_readwrite("WS_Obstacles", &mrpt::nav::CLogFileRecord::WS_Obstacles);
		cl.def_readwrite("WS_Obstacles_original", &mrpt::nav::CLogFileRecord::WS_Obstacles_original);
		cl.def_readwrite("robotPoseLocalization", &mrpt::nav::CLogFileRecord::robotPoseLocalization);
		cl.def_readwrite("robotPoseOdometry", &mrpt::nav::CLogFileRecord::robotPoseOdometry);
		cl.def_readwrite("relPoseSense", &mrpt::nav::CLogFileRecord::relPoseSense);
		cl.def_readwrite("relPoseVelCmd", &mrpt::nav::CLogFileRecord::relPoseVelCmd);
		cl.def_readwrite("WS_targets_relative", &mrpt::nav::CLogFileRecord::WS_targets_relative);
		cl.def_readwrite("cmd_vel", &mrpt::nav::CLogFileRecord::cmd_vel);
		cl.def_readwrite("cmd_vel_original", &mrpt::nav::CLogFileRecord::cmd_vel_original);
		cl.def_readwrite("cur_vel", &mrpt::nav::CLogFileRecord::cur_vel);
		cl.def_readwrite("cur_vel_local", &mrpt::nav::CLogFileRecord::cur_vel_local);
		cl.def_readwrite("robotShape_x", &mrpt::nav::CLogFileRecord::robotShape_x);
		cl.def_readwrite("robotShape_y", &mrpt::nav::CLogFileRecord::robotShape_y);
		cl.def_readwrite("robotShape_radius", &mrpt::nav::CLogFileRecord::robotShape_radius);
		cl.def_readwrite("ptg_index_NOP", &mrpt::nav::CLogFileRecord::ptg_index_NOP);
		cl.def_readwrite("ptg_last_k_NOP", &mrpt::nav::CLogFileRecord::ptg_last_k_NOP);
		cl.def_readwrite("rel_cur_pose_wrt_last_vel_cmd_NOP", &mrpt::nav::CLogFileRecord::rel_cur_pose_wrt_last_vel_cmd_NOP);
		cl.def_readwrite("rel_pose_PTG_origin_wrt_sense_NOP", &mrpt::nav::CLogFileRecord::rel_pose_PTG_origin_wrt_sense_NOP);
		cl.def_readwrite("ptg_last_navDynState", &mrpt::nav::CLogFileRecord::ptg_last_navDynState);
		cl.def_readwrite("visuals", &mrpt::nav::CLogFileRecord::visuals);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::nav::CLogFileRecord::GetRuntimeClassIdStatic, "C++: mrpt::nav::CLogFileRecord::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::nav::CLogFileRecord::*)() const) &mrpt::nav::CLogFileRecord::GetRuntimeClass, "C++: mrpt::nav::CLogFileRecord::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::nav::CLogFileRecord::*)() const) &mrpt::nav::CLogFileRecord::clone, "C++: mrpt::nav::CLogFileRecord::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::nav::CLogFileRecord::CreateObject, "C++: mrpt::nav::CLogFileRecord::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("assign", (class mrpt::nav::CLogFileRecord & (mrpt::nav::CLogFileRecord::*)(const class mrpt::nav::CLogFileRecord &)) &mrpt::nav::CLogFileRecord::operator=, "C++: mrpt::nav::CLogFileRecord::operator=(const class mrpt::nav::CLogFileRecord &) --> class mrpt::nav::CLogFileRecord &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::nav::CLogFileRecord::TInfoPerPTG file:mrpt/nav/reactive/CLogFileRecord.h line:41
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::nav::CLogFileRecord::TInfoPerPTG, std::shared_ptr<mrpt::nav::CLogFileRecord::TInfoPerPTG>> cl(enclosing_class, "TInfoPerPTG", "The structure used to store all relevant information about each\n  transformation into TP-Space.\n  \n");
			cl.def( pybind11::init( [](){ return new mrpt::nav::CLogFileRecord::TInfoPerPTG(); } ) );
			cl.def( pybind11::init( [](mrpt::nav::CLogFileRecord::TInfoPerPTG const &o){ return new mrpt::nav::CLogFileRecord::TInfoPerPTG(o); } ) );
			cl.def_readwrite("PTG_desc", &mrpt::nav::CLogFileRecord::TInfoPerPTG::PTG_desc);
			cl.def_readwrite("TP_Obstacles", &mrpt::nav::CLogFileRecord::TInfoPerPTG::TP_Obstacles);
			cl.def_readwrite("TP_Targets", &mrpt::nav::CLogFileRecord::TInfoPerPTG::TP_Targets);
			cl.def_readwrite("TP_Robot", &mrpt::nav::CLogFileRecord::TInfoPerPTG::TP_Robot);
			cl.def_readwrite("timeForTPObsTransformation", &mrpt::nav::CLogFileRecord::TInfoPerPTG::timeForTPObsTransformation);
			cl.def_readwrite("timeForHolonomicMethod", &mrpt::nav::CLogFileRecord::TInfoPerPTG::timeForHolonomicMethod);
			cl.def_readwrite("desiredDirection", &mrpt::nav::CLogFileRecord::TInfoPerPTG::desiredDirection);
			cl.def_readwrite("desiredSpeed", &mrpt::nav::CLogFileRecord::TInfoPerPTG::desiredSpeed);
			cl.def_readwrite("evaluation", &mrpt::nav::CLogFileRecord::TInfoPerPTG::evaluation);
			cl.def_readwrite("evalFactors", &mrpt::nav::CLogFileRecord::TInfoPerPTG::evalFactors);
			cl.def_readwrite("HLFR", &mrpt::nav::CLogFileRecord::TInfoPerPTG::HLFR);
			cl.def_readwrite("ptg", &mrpt::nav::CLogFileRecord::TInfoPerPTG::ptg);
			cl.def_readwrite("clearance", &mrpt::nav::CLogFileRecord::TInfoPerPTG::clearance);
			cl.def("assign", (struct mrpt::nav::CLogFileRecord::TInfoPerPTG & (mrpt::nav::CLogFileRecord::TInfoPerPTG::*)(const struct mrpt::nav::CLogFileRecord::TInfoPerPTG &)) &mrpt::nav::CLogFileRecord::TInfoPerPTG::operator=, "C++: mrpt::nav::CLogFileRecord::TInfoPerPTG::operator=(const struct mrpt::nav::CLogFileRecord::TInfoPerPTG &) --> struct mrpt::nav::CLogFileRecord::TInfoPerPTG &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
	{ // mrpt::nav::TCandidateMovementPTG file:mrpt/nav/reactive/TCandidateMovementPTG.h line:22
		pybind11::class_<mrpt::nav::TCandidateMovementPTG, std::shared_ptr<mrpt::nav::TCandidateMovementPTG>> cl(M("mrpt::nav"), "TCandidateMovementPTG", "Stores a candidate movement in TP-Space-based navigation.\n\n CReactiveNavigationSystem, CReactiveNavigationSystem3D\n  \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::nav::TCandidateMovementPTG(); } ) );
		cl.def( pybind11::init( [](mrpt::nav::TCandidateMovementPTG const &o){ return new mrpt::nav::TCandidateMovementPTG(o); } ) );
		cl.def_readwrite("direction", &mrpt::nav::TCandidateMovementPTG::direction);
		cl.def_readwrite("speed", &mrpt::nav::TCandidateMovementPTG::speed);
		cl.def_readwrite("starting_robot_dir", &mrpt::nav::TCandidateMovementPTG::starting_robot_dir);
		cl.def_readwrite("starting_robot_dist", &mrpt::nav::TCandidateMovementPTG::starting_robot_dist);
		cl.def_readwrite("props", &mrpt::nav::TCandidateMovementPTG::props);
		cl.def("assign", (struct mrpt::nav::TCandidateMovementPTG & (mrpt::nav::TCandidateMovementPTG::*)(const struct mrpt::nav::TCandidateMovementPTG &)) &mrpt::nav::TCandidateMovementPTG::operator=, "C++: mrpt::nav::TCandidateMovementPTG::operator=(const struct mrpt::nav::TCandidateMovementPTG &) --> struct mrpt::nav::TCandidateMovementPTG &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::nav::CMultiObjectiveMotionOptimizerBase file:mrpt/nav/reactive/CMultiObjectiveMotionOptimizerBase.h line:24
		pybind11::class_<mrpt::nav::CMultiObjectiveMotionOptimizerBase, std::shared_ptr<mrpt::nav::CMultiObjectiveMotionOptimizerBase>, mrpt::rtti::CObject> cl(M("mrpt::nav"), "CMultiObjectiveMotionOptimizerBase", "Virtual base class for multi-objective motion choosers, as used for reactive\nnavigation engines.\n\n CReactiveNavigationSystem, CReactiveNavigationSystem3D\n  \n\n\n ");
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::nav::CMultiObjectiveMotionOptimizerBase::*)() const) &mrpt::nav::CMultiObjectiveMotionOptimizerBase::GetRuntimeClass, "C++: mrpt::nav::CMultiObjectiveMotionOptimizerBase::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::nav::CMultiObjectiveMotionOptimizerBase::GetRuntimeClassIdStatic, "C++: mrpt::nav::CMultiObjectiveMotionOptimizerBase::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def_static("Factory", (class std::shared_ptr<class mrpt::nav::CMultiObjectiveMotionOptimizerBase> (*)(const std::string &)) &mrpt::nav::CMultiObjectiveMotionOptimizerBase::Factory, "Class factory from C++ class name \n\nC++: mrpt::nav::CMultiObjectiveMotionOptimizerBase::Factory(const std::string &) --> class std::shared_ptr<class mrpt::nav::CMultiObjectiveMotionOptimizerBase>", pybind11::arg("className"));
		cl.def("loadConfigFile", (void (mrpt::nav::CMultiObjectiveMotionOptimizerBase::*)(const class mrpt::config::CConfigFileBase &)) &mrpt::nav::CMultiObjectiveMotionOptimizerBase::loadConfigFile, "C++: mrpt::nav::CMultiObjectiveMotionOptimizerBase::loadConfigFile(const class mrpt::config::CConfigFileBase &) --> void", pybind11::arg("c"));
		cl.def("saveConfigFile", (void (mrpt::nav::CMultiObjectiveMotionOptimizerBase::*)(class mrpt::config::CConfigFileBase &) const) &mrpt::nav::CMultiObjectiveMotionOptimizerBase::saveConfigFile, "C++: mrpt::nav::CMultiObjectiveMotionOptimizerBase::saveConfigFile(class mrpt::config::CConfigFileBase &) const --> void", pybind11::arg("c"));
		cl.def("clear", (void (mrpt::nav::CMultiObjectiveMotionOptimizerBase::*)()) &mrpt::nav::CMultiObjectiveMotionOptimizerBase::clear, "Resets the object state; use if the parameters change, so they are\n re-read and applied. \n\nC++: mrpt::nav::CMultiObjectiveMotionOptimizerBase::clear() --> void");

		{ // mrpt::nav::CMultiObjectiveMotionOptimizerBase::TResultInfo file:mrpt/nav/reactive/CMultiObjectiveMotionOptimizerBase.h line:31
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::nav::CMultiObjectiveMotionOptimizerBase::TResultInfo, std::shared_ptr<mrpt::nav::CMultiObjectiveMotionOptimizerBase::TResultInfo>> cl(enclosing_class, "TResultInfo", "");
			cl.def( pybind11::init( [](){ return new mrpt::nav::CMultiObjectiveMotionOptimizerBase::TResultInfo(); } ) );
			cl.def( pybind11::init( [](mrpt::nav::CMultiObjectiveMotionOptimizerBase::TResultInfo const &o){ return new mrpt::nav::CMultiObjectiveMotionOptimizerBase::TResultInfo(o); } ) );
			cl.def_readwrite("score_values", &mrpt::nav::CMultiObjectiveMotionOptimizerBase::TResultInfo::score_values);
			cl.def_readwrite("final_evaluation", &mrpt::nav::CMultiObjectiveMotionOptimizerBase::TResultInfo::final_evaluation);
			cl.def_readwrite("log_entries", &mrpt::nav::CMultiObjectiveMotionOptimizerBase::TResultInfo::log_entries);
			cl.def("assign", (struct mrpt::nav::CMultiObjectiveMotionOptimizerBase::TResultInfo & (mrpt::nav::CMultiObjectiveMotionOptimizerBase::TResultInfo::*)(const struct mrpt::nav::CMultiObjectiveMotionOptimizerBase::TResultInfo &)) &mrpt::nav::CMultiObjectiveMotionOptimizerBase::TResultInfo::operator=, "C++: mrpt::nav::CMultiObjectiveMotionOptimizerBase::TResultInfo::operator=(const struct mrpt::nav::CMultiObjectiveMotionOptimizerBase::TResultInfo &) --> struct mrpt::nav::CMultiObjectiveMotionOptimizerBase::TResultInfo &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::nav::CMultiObjectiveMotionOptimizerBase::TParamsBase file:mrpt/nav/reactive/CMultiObjectiveMotionOptimizerBase.h line:56
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::nav::CMultiObjectiveMotionOptimizerBase::TParamsBase, std::shared_ptr<mrpt::nav::CMultiObjectiveMotionOptimizerBase::TParamsBase>, PyCallBack_mrpt_nav_CMultiObjectiveMotionOptimizerBase_TParamsBase, mrpt::config::CLoadableOptions> cl(enclosing_class, "TParamsBase", "Common params for all children ");
			cl.def( pybind11::init( [](){ return new mrpt::nav::CMultiObjectiveMotionOptimizerBase::TParamsBase(); }, [](){ return new PyCallBack_mrpt_nav_CMultiObjectiveMotionOptimizerBase_TParamsBase(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_nav_CMultiObjectiveMotionOptimizerBase_TParamsBase const &o){ return new PyCallBack_mrpt_nav_CMultiObjectiveMotionOptimizerBase_TParamsBase(o); } ) );
			cl.def( pybind11::init( [](mrpt::nav::CMultiObjectiveMotionOptimizerBase::TParamsBase const &o){ return new mrpt::nav::CMultiObjectiveMotionOptimizerBase::TParamsBase(o); } ) );
			cl.def_readwrite("formula_score", &mrpt::nav::CMultiObjectiveMotionOptimizerBase::TParamsBase::formula_score);
			cl.def_readwrite("movement_assert", &mrpt::nav::CMultiObjectiveMotionOptimizerBase::TParamsBase::movement_assert);
			cl.def_readwrite("scores_to_normalize", &mrpt::nav::CMultiObjectiveMotionOptimizerBase::TParamsBase::scores_to_normalize);
			cl.def("loadFromConfigFile", (void (mrpt::nav::CMultiObjectiveMotionOptimizerBase::TParamsBase::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::nav::CMultiObjectiveMotionOptimizerBase::TParamsBase::loadFromConfigFile, "C++: mrpt::nav::CMultiObjectiveMotionOptimizerBase::TParamsBase::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
			cl.def("saveToConfigFile", (void (mrpt::nav::CMultiObjectiveMotionOptimizerBase::TParamsBase::*)(class mrpt::config::CConfigFileBase &, const std::string &) const) &mrpt::nav::CMultiObjectiveMotionOptimizerBase::TParamsBase::saveToConfigFile, "C++: mrpt::nav::CMultiObjectiveMotionOptimizerBase::TParamsBase::saveToConfigFile(class mrpt::config::CConfigFileBase &, const std::string &) const --> void", pybind11::arg("cfg"), pybind11::arg("section"));
			cl.def("assign", (struct mrpt::nav::CMultiObjectiveMotionOptimizerBase::TParamsBase & (mrpt::nav::CMultiObjectiveMotionOptimizerBase::TParamsBase::*)(const struct mrpt::nav::CMultiObjectiveMotionOptimizerBase::TParamsBase &)) &mrpt::nav::CMultiObjectiveMotionOptimizerBase::TParamsBase::operator=, "C++: mrpt::nav::CMultiObjectiveMotionOptimizerBase::TParamsBase::operator=(const struct mrpt::nav::CMultiObjectiveMotionOptimizerBase::TParamsBase &) --> struct mrpt::nav::CMultiObjectiveMotionOptimizerBase::TParamsBase &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
	{ // mrpt::nav::TWaypoint file:mrpt/nav/reactive/TWaypoint.h line:26
		pybind11::class_<mrpt::nav::TWaypoint, std::shared_ptr<mrpt::nav::TWaypoint>> cl(M("mrpt::nav"), "TWaypoint", "A single waypoint within TWaypointSequence. ");
		cl.def( pybind11::init( [](){ return new mrpt::nav::TWaypoint(); } ) );
		cl.def( pybind11::init( [](mrpt::nav::TWaypoint const &o){ return new mrpt::nav::TWaypoint(o); } ) );
		cl.def_readwrite("target", &mrpt::nav::TWaypoint::target);
		cl.def_readwrite("target_heading", &mrpt::nav::TWaypoint::target_heading);
		cl.def_readwrite("target_frame_id", &mrpt::nav::TWaypoint::target_frame_id);
		cl.def_readwrite("allowed_distance", &mrpt::nav::TWaypoint::allowed_distance);
		cl.def_readwrite("speed_ratio", &mrpt::nav::TWaypoint::speed_ratio);
		cl.def_readwrite("allow_skip", &mrpt::nav::TWaypoint::allow_skip);
		cl.def_readwrite("user_data", &mrpt::nav::TWaypoint::user_data);
		cl.def("isValid", (bool (mrpt::nav::TWaypoint::*)() const) &mrpt::nav::TWaypoint::isValid, "Check whether all the minimum mandatory fields have been filled by the\n user. \n\nC++: mrpt::nav::TWaypoint::isValid() const --> bool");
		cl.def("getAsText", (std::string (mrpt::nav::TWaypoint::*)() const) &mrpt::nav::TWaypoint::getAsText, "get in human-readable format \n\nC++: mrpt::nav::TWaypoint::getAsText() const --> std::string");
		cl.def("assign", (struct mrpt::nav::TWaypoint & (mrpt::nav::TWaypoint::*)(const struct mrpt::nav::TWaypoint &)) &mrpt::nav::TWaypoint::operator=, "C++: mrpt::nav::TWaypoint::operator=(const struct mrpt::nav::TWaypoint &) --> struct mrpt::nav::TWaypoint &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
