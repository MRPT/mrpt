#include <chrono>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/core/Clock.h>
#include <mrpt/kinematics/CVehicleVelCmd.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TTwist2D.h>
#include <mrpt/nav/holonomic/ClearanceDiagram.h>
#include <mrpt/nav/reactive/CAbstractNavigator.h>
#include <mrpt/nav/reactive/CAbstractPTGBasedReactive.h>
#include <mrpt/nav/reactive/CLogFileRecord.h>
#include <mrpt/nav/reactive/CRobot2NavInterface.h>
#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/typemeta/static_string.h>
#include <ratio>
#include <sstream> // __str__
#include <string>
#include <string_view>
#include <utility>
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

// mrpt::nav::CAbstractPTGBasedReactive::TNavigationParamsPTG file:mrpt/nav/reactive/CAbstractPTGBasedReactive.h line:98
struct PyCallBack_mrpt_nav_CAbstractPTGBasedReactive_TNavigationParamsPTG : public mrpt::nav::CAbstractPTGBasedReactive::TNavigationParamsPTG {
	using mrpt::nav::CAbstractPTGBasedReactive::TNavigationParamsPTG::TNavigationParamsPTG;

	std::string getAsText() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractPTGBasedReactive::TNavigationParamsPTG *>(this), "getAsText");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return TNavigationParamsPTG::getAsText();
	}
	bool isEqual(const struct mrpt::nav::CAbstractNavigator::TNavigationParamsBase & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractPTGBasedReactive::TNavigationParamsPTG *>(this), "isEqual");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return TNavigationParamsPTG::isEqual(a0);
	}
};

// mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams file:mrpt/nav/reactive/CAbstractPTGBasedReactive.h line:162
struct PyCallBack_mrpt_nav_CAbstractPTGBasedReactive_TAbstractPTGNavigatorParams : public mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams {
	using mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams::TAbstractPTGNavigatorParams;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TAbstractPTGNavigatorParams::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TAbstractPTGNavigatorParams::saveToConfigFile(a0, a1);
	}
};

void bind_mrpt_nav_reactive_CAbstractPTGBasedReactive(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::nav::CAbstractPTGBasedReactive file:mrpt/nav/reactive/CAbstractPTGBasedReactive.h line:93
		pybind11::class_<mrpt::nav::CAbstractPTGBasedReactive, std::shared_ptr<mrpt::nav::CAbstractPTGBasedReactive>, mrpt::nav::CWaypointsNavigator> cl(M("mrpt::nav"), "CAbstractPTGBasedReactive", "Base class for reactive navigator systems based on TP-Space, with an\n arbitrary holonomic\n reactive method running on it and any number of PTGs for transforming the\n navigation space.\n Both, the holonomic method and the PTGs can be customized by the apropriate\n user derived classes.\n\n How to use:\n  - Instantiate a reactive navigation object (one of the derived classes of\n this virtual class).\n  - A class with callbacks must be defined by the user and provided to the\n constructor (derived from CRobot2NavInterface)\n  - loadConfigFile() must be called to set up the bunch of parameters from a\n config file (could be a memory-based virtual config file).\n  - navigationStep() must be called periodically in order to effectively run\n the navigation. This method will internally call the callbacks to gather\n sensor data and robot positioning data.\n\n For working examples, refer to the source code of the apps:\n  -\n [ReactiveNavigationDemo](http://www.mrpt.org/list-of-mrpt-apps/application-reactivenavigationdemo/)\n  -\n [ReactiveNav3D-Demo](http://www.mrpt.org/list-of-mrpt-apps/application-reactivenav3d-demo/)\n\n Publications:\n  - See derived classes for papers on each specific method.\n\n Available \"variables\" or \"score names\" for each motion candidate (these can\n be used in runtime-compiled expressions\n in the configuration files of motion deciders):\n\n - `clearance`: Clearance (larger means larger distances to obstacles) for\n the path from \"current pose\" up to \"end of trajectory\".\n - `collision_free_distance`: Normalized [0,1] collision-free distance in\n selected path. For NOP candidates, the traveled distances is substracted.\n - `dist_eucl_final`: Euclidean distance (in the real-world WordSpace)\n between \"end of trajectory\" and target.\n - `eta`: Estimated Time of Arrival at \"end of trajectory\".\n - `holo_stage_eval`: Final evaluation of the selected direction from inside\n of the holonomic algorithm.\n - `hysteresis`: Measure of similarity with previous command [0,1]\n - `is_PTG_cont`: 1 (is \"NOP\" motion command), 0 otherwise\n - `is_slowdown`: 1 if PTG returns true in\n CParameterizedTrajectoryGenerator::supportSpeedAtTarget() for this step.\n - `move_cur_d`: Normalized distance already traveled over the selected PTG.\n Normally 0, unless in a \"NOP motion\".\n - `move_k`: Motion candidate path 0-based index.\n - `num_paths`: Number of paths in the PTG\n - `original_col_free_dist`: Only for \"NOP motions\", the collision-free\n distance when the motion command was originally issued.\n - `ptg_idx`: PTG index (0-based)\n - `ptg_priority`: Product of PTG getScorePriority() times PTG\n evalPathRelativePriority()\n - `ref_dist`: PTG ref distance [m]\n - `robpose_x`, `robpose_y`, `robpose_phi`: Robot pose ([m] and [rad]) at the\n \"end of trajectory\": at collision or at target distance.\n - `target_d_norm`: Normalized target distance. Can be >1 if distance is\n larger than ref_distance.\n - `target_dir`: Angle of target in TP-Space [rad]\n - `target_k`: Same as target_dir but in discrete path 0-based indices.\n - `WS_target_x`, `WS_target_y`: Target coordinates in realworld [m]\n\n \n CReactiveNavigationSystem, CReactiveNavigationSystem3D\n  \n\n\n ");
		cl.def_readwrite("params_abstract_ptg_navigator", &mrpt::nav::CAbstractPTGBasedReactive::params_abstract_ptg_navigator);
		cl.def("initialize", (void (mrpt::nav::CAbstractPTGBasedReactive::*)()) &mrpt::nav::CAbstractPTGBasedReactive::initialize, "Must be called for loading collision grids, or the first navigation\n command may last a long time to be executed.\n Internally, it just calls STEP1_CollisionGridsBuilder().\n\nC++: mrpt::nav::CAbstractPTGBasedReactive::initialize() --> void");
		cl.def("setHolonomicMethod", (void (mrpt::nav::CAbstractPTGBasedReactive::*)(const std::string &, const class mrpt::config::CConfigFileBase &)) &mrpt::nav::CAbstractPTGBasedReactive::setHolonomicMethod, "Selects which one from the set of available holonomic methods will be\n used\n  into transformed TP-Space, and sets its configuration from a\n configuration file.\n Available methods: class names of those derived from\n CAbstractHolonomicReactiveMethod\n\nC++: mrpt::nav::CAbstractPTGBasedReactive::setHolonomicMethod(const std::string &, const class mrpt::config::CConfigFileBase &) --> void", pybind11::arg("method"), pybind11::arg("cfgBase"));
		cl.def("getLastLogRecord", (void (mrpt::nav::CAbstractPTGBasedReactive::*)(class mrpt::nav::CLogFileRecord &)) &mrpt::nav::CAbstractPTGBasedReactive::getLastLogRecord, "Provides a copy of the last log record with information about execution.\n \n\n An object where the log will be stored into.\n \n\n Log records are not prepared unless either \"enableLogFile\" is\n enabled in the constructor or \"enableLogFile()\" has been called.\n\nC++: mrpt::nav::CAbstractPTGBasedReactive::getLastLogRecord(class mrpt::nav::CLogFileRecord &) --> void", pybind11::arg("o"));
		cl.def("enableKeepLogRecords", [](mrpt::nav::CAbstractPTGBasedReactive &o) -> void { return o.enableKeepLogRecords(); }, "");
		cl.def("enableKeepLogRecords", (void (mrpt::nav::CAbstractPTGBasedReactive::*)(bool)) &mrpt::nav::CAbstractPTGBasedReactive::enableKeepLogRecords, "Enables keeping an internal registry of navigation logs that can be\n queried with getLastLogRecord() \n\nC++: mrpt::nav::CAbstractPTGBasedReactive::enableKeepLogRecords(bool) --> void", pybind11::arg("enable"));
		cl.def("enableLogFile", (void (mrpt::nav::CAbstractPTGBasedReactive::*)(bool)) &mrpt::nav::CAbstractPTGBasedReactive::enableLogFile, "Enables/disables saving log files. \n\nC++: mrpt::nav::CAbstractPTGBasedReactive::enableLogFile(bool) --> void", pybind11::arg("enable"));
		cl.def("setLogFileDirectory", (void (mrpt::nav::CAbstractPTGBasedReactive::*)(const std::string &)) &mrpt::nav::CAbstractPTGBasedReactive::setLogFileDirectory, "Changes the prefix for new log files. \n\nC++: mrpt::nav::CAbstractPTGBasedReactive::setLogFileDirectory(const std::string &) --> void", pybind11::arg("sDir"));
		cl.def("getLogFileDirectory", (std::string (mrpt::nav::CAbstractPTGBasedReactive::*)() const) &mrpt::nav::CAbstractPTGBasedReactive::getLogFileDirectory, "C++: mrpt::nav::CAbstractPTGBasedReactive::getLogFileDirectory() const --> std::string");
		cl.def("loadConfigFile", (void (mrpt::nav::CAbstractPTGBasedReactive::*)(const class mrpt::config::CConfigFileBase &)) &mrpt::nav::CAbstractPTGBasedReactive::loadConfigFile, "C++: mrpt::nav::CAbstractPTGBasedReactive::loadConfigFile(const class mrpt::config::CConfigFileBase &) --> void", pybind11::arg("c"));
		cl.def("saveConfigFile", (void (mrpt::nav::CAbstractPTGBasedReactive::*)(class mrpt::config::CConfigFileBase &) const) &mrpt::nav::CAbstractPTGBasedReactive::saveConfigFile, "C++: mrpt::nav::CAbstractPTGBasedReactive::saveConfigFile(class mrpt::config::CConfigFileBase &) const --> void", pybind11::arg("c"));
		cl.def("enableTimeLog", [](mrpt::nav::CAbstractPTGBasedReactive &o) -> void { return o.enableTimeLog(); }, "");
		cl.def("enableTimeLog", (void (mrpt::nav::CAbstractPTGBasedReactive::*)(bool)) &mrpt::nav::CAbstractPTGBasedReactive::enableTimeLog, "Enables/disables the detailed time logger (default:disabled upon\n construction)\n  When enabled, a report will be dumped to std::cout upon destruction.\n \n\n getTimeLogger\n\nC++: mrpt::nav::CAbstractPTGBasedReactive::enableTimeLog(bool) --> void", pybind11::arg("enable"));
		cl.def("getTimeLogger", (const class mrpt::system::CTimeLogger & (mrpt::nav::CAbstractPTGBasedReactive::*)() const) &mrpt::nav::CAbstractPTGBasedReactive::getTimeLogger, "Gives access to a const-ref to the internal time logger \n\n enableTimeLog \n\nC++: mrpt::nav::CAbstractPTGBasedReactive::getTimeLogger() const --> const class mrpt::system::CTimeLogger &", pybind11::return_value_policy::automatic);
		cl.def("getPTG_count", (size_t (mrpt::nav::CAbstractPTGBasedReactive::*)() const) &mrpt::nav::CAbstractPTGBasedReactive::getPTG_count, "Returns the number of different PTGs that have been setup \n\nC++: mrpt::nav::CAbstractPTGBasedReactive::getPTG_count() const --> size_t");
		cl.def("getPTG", (class mrpt::nav::CParameterizedTrajectoryGenerator * (mrpt::nav::CAbstractPTGBasedReactive::*)(size_t)) &mrpt::nav::CAbstractPTGBasedReactive::getPTG, "Gets the i'th PTG \n\nC++: mrpt::nav::CAbstractPTGBasedReactive::getPTG(size_t) --> class mrpt::nav::CParameterizedTrajectoryGenerator *", pybind11::return_value_policy::automatic, pybind11::arg("i"));
		cl.def("getCurrentRobotSpeedLimits", (const struct mrpt::kinematics::CVehicleVelCmd::TVelCmdParams & (mrpt::nav::CAbstractPTGBasedReactive::*)() const) &mrpt::nav::CAbstractPTGBasedReactive::getCurrentRobotSpeedLimits, "Get the current, global (honored for all PTGs) robot speed limits \n\nC++: mrpt::nav::CAbstractPTGBasedReactive::getCurrentRobotSpeedLimits() const --> const struct mrpt::kinematics::CVehicleVelCmd::TVelCmdParams &", pybind11::return_value_policy::automatic);
		cl.def("changeCurrentRobotSpeedLimits", (struct mrpt::kinematics::CVehicleVelCmd::TVelCmdParams & (mrpt::nav::CAbstractPTGBasedReactive::*)()) &mrpt::nav::CAbstractPTGBasedReactive::changeCurrentRobotSpeedLimits, "Changes the current, global (honored for all PTGs) robot speed limits,\n via returning a reference to a structure that holds those limits \n\nC++: mrpt::nav::CAbstractPTGBasedReactive::changeCurrentRobotSpeedLimits() --> struct mrpt::kinematics::CVehicleVelCmd::TVelCmdParams &", pybind11::return_value_policy::automatic);
		cl.def("setTargetApproachSlowDownDistance", (void (mrpt::nav::CAbstractPTGBasedReactive::*)(const double)) &mrpt::nav::CAbstractPTGBasedReactive::setTargetApproachSlowDownDistance, "Changes this parameter in all inner holonomic navigator instances [m].\n\nC++: mrpt::nav::CAbstractPTGBasedReactive::setTargetApproachSlowDownDistance(const double) --> void", pybind11::arg("dist"));
		cl.def("getTargetApproachSlowDownDistance", (double (mrpt::nav::CAbstractPTGBasedReactive::*)() const) &mrpt::nav::CAbstractPTGBasedReactive::getTargetApproachSlowDownDistance, "Returns this parameter for the first inner holonomic navigator instances\n [m] (should be the same in all of them?) \n\nC++: mrpt::nav::CAbstractPTGBasedReactive::getTargetApproachSlowDownDistance() const --> double");

		{ // mrpt::nav::CAbstractPTGBasedReactive::TNavigationParamsPTG file:mrpt/nav/reactive/CAbstractPTGBasedReactive.h line:98
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::nav::CAbstractPTGBasedReactive::TNavigationParamsPTG, std::shared_ptr<mrpt::nav::CAbstractPTGBasedReactive::TNavigationParamsPTG>, PyCallBack_mrpt_nav_CAbstractPTGBasedReactive_TNavigationParamsPTG, mrpt::nav::CWaypointsNavigator::TNavigationParamsWaypoints> cl(enclosing_class, "TNavigationParamsPTG", "The struct for configuring navigation requests to\n CAbstractPTGBasedReactive and derived classes. ");
			cl.def( pybind11::init( [](PyCallBack_mrpt_nav_CAbstractPTGBasedReactive_TNavigationParamsPTG const &o){ return new PyCallBack_mrpt_nav_CAbstractPTGBasedReactive_TNavigationParamsPTG(o); } ) );
			cl.def( pybind11::init( [](mrpt::nav::CAbstractPTGBasedReactive::TNavigationParamsPTG const &o){ return new mrpt::nav::CAbstractPTGBasedReactive::TNavigationParamsPTG(o); } ) );
			cl.def( pybind11::init( [](){ return new mrpt::nav::CAbstractPTGBasedReactive::TNavigationParamsPTG(); }, [](){ return new PyCallBack_mrpt_nav_CAbstractPTGBasedReactive_TNavigationParamsPTG(); } ) );
			cl.def_readwrite("restrict_PTG_indices", &mrpt::nav::CAbstractPTGBasedReactive::TNavigationParamsPTG::restrict_PTG_indices);
			cl.def("getAsText", (std::string (mrpt::nav::CAbstractPTGBasedReactive::TNavigationParamsPTG::*)() const) &mrpt::nav::CAbstractPTGBasedReactive::TNavigationParamsPTG::getAsText, "C++: mrpt::nav::CAbstractPTGBasedReactive::TNavigationParamsPTG::getAsText() const --> std::string");
			cl.def("assign", (struct mrpt::nav::CAbstractPTGBasedReactive::TNavigationParamsPTG & (mrpt::nav::CAbstractPTGBasedReactive::TNavigationParamsPTG::*)(const struct mrpt::nav::CAbstractPTGBasedReactive::TNavigationParamsPTG &)) &mrpt::nav::CAbstractPTGBasedReactive::TNavigationParamsPTG::operator=, "C++: mrpt::nav::CAbstractPTGBasedReactive::TNavigationParamsPTG::operator=(const struct mrpt::nav::CAbstractPTGBasedReactive::TNavigationParamsPTG &) --> struct mrpt::nav::CAbstractPTGBasedReactive::TNavigationParamsPTG &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams file:mrpt/nav/reactive/CAbstractPTGBasedReactive.h line:162
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams, std::shared_ptr<mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams>, PyCallBack_mrpt_nav_CAbstractPTGBasedReactive_TAbstractPTGNavigatorParams, mrpt::config::CLoadableOptions> cl(enclosing_class, "TAbstractPTGNavigatorParams", "");
			cl.def( pybind11::init( [](){ return new mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams(); }, [](){ return new PyCallBack_mrpt_nav_CAbstractPTGBasedReactive_TAbstractPTGNavigatorParams(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_nav_CAbstractPTGBasedReactive_TAbstractPTGNavigatorParams const &o){ return new PyCallBack_mrpt_nav_CAbstractPTGBasedReactive_TAbstractPTGNavigatorParams(o); } ) );
			cl.def( pybind11::init( [](mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams const &o){ return new mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams(o); } ) );
			cl.def_readwrite("holonomic_method", &mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams::holonomic_method);
			cl.def_readwrite("motion_decider_method", &mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams::motion_decider_method);
			cl.def_readwrite("ptg_cache_files_directory", &mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams::ptg_cache_files_directory);
			cl.def_readwrite("ref_distance", &mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams::ref_distance);
			cl.def_readwrite("speedfilter_tau", &mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams::speedfilter_tau);
			cl.def_readwrite("secure_distance_start", &mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams::secure_distance_start);
			cl.def_readwrite("secure_distance_end", &mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams::secure_distance_end);
			cl.def_readwrite("use_delays_model", &mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams::use_delays_model);
			cl.def_readwrite("max_distance_predicted_actual_path", &mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams::max_distance_predicted_actual_path);
			cl.def_readwrite("min_normalized_free_space_for_ptg_continuation", &mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams::min_normalized_free_space_for_ptg_continuation);
			cl.def_readwrite("robot_absolute_speed_limits", &mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams::robot_absolute_speed_limits);
			cl.def_readwrite("enable_obstacle_filtering", &mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams::enable_obstacle_filtering);
			cl.def_readwrite("evaluate_clearance", &mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams::evaluate_clearance);
			cl.def_readwrite("max_dist_for_timebased_path_prediction", &mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams::max_dist_for_timebased_path_prediction);
			cl.def("loadFromConfigFile", (void (mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams::loadFromConfigFile, "C++: mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("c"), pybind11::arg("s"));
			cl.def("saveToConfigFile", (void (mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams::*)(class mrpt::config::CConfigFileBase &, const std::string &) const) &mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams::saveToConfigFile, "C++: mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams::saveToConfigFile(class mrpt::config::CConfigFileBase &, const std::string &) const --> void", pybind11::arg("c"), pybind11::arg("s"));
			cl.def("assign", (struct mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams & (mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams::*)(const struct mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams &)) &mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams::operator=, "C++: mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams::operator=(const struct mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams &) --> struct mrpt::nav::CAbstractPTGBasedReactive::TAbstractPTGNavigatorParams &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
}
