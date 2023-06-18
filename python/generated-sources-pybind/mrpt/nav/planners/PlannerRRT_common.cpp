#include <iterator>
#include <memory>
#include <mrpt/nav/planners/PlannerRRT_common.h>
#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <mrpt/system/CTimeLogger.h>
#include <sstream> // __str__
#include <string>
#include <string_view>
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

void bind_mrpt_nav_planners_PlannerRRT_common(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::nav::PlannerTPS_VirtualBase file:mrpt/nav/planners/PlannerRRT_common.h line:132
		pybind11::class_<mrpt::nav::PlannerTPS_VirtualBase, std::shared_ptr<mrpt::nav::PlannerTPS_VirtualBase>> cl(M("mrpt::nav"), "PlannerTPS_VirtualBase", "Virtual base class for TP-Space-based path planners ");
		cl.def( pybind11::init( [](){ return new mrpt::nav::PlannerTPS_VirtualBase(); } ) );
		cl.def( pybind11::init( [](mrpt::nav::PlannerTPS_VirtualBase const &o){ return new mrpt::nav::PlannerTPS_VirtualBase(o); } ) );
		cl.def_readwrite("end_criteria", &mrpt::nav::PlannerTPS_VirtualBase::end_criteria);
		cl.def_readwrite("params", &mrpt::nav::PlannerTPS_VirtualBase::params);
		cl.def("getProfiler", (class mrpt::system::CTimeLogger & (mrpt::nav::PlannerTPS_VirtualBase::*)()) &mrpt::nav::PlannerTPS_VirtualBase::getProfiler, "C++: mrpt::nav::PlannerTPS_VirtualBase::getProfiler() --> class mrpt::system::CTimeLogger &", pybind11::return_value_policy::automatic);
		cl.def("assign", (class mrpt::nav::PlannerTPS_VirtualBase & (mrpt::nav::PlannerTPS_VirtualBase::*)(const class mrpt::nav::PlannerTPS_VirtualBase &)) &mrpt::nav::PlannerTPS_VirtualBase::operator=, "C++: mrpt::nav::PlannerTPS_VirtualBase::operator=(const class mrpt::nav::PlannerTPS_VirtualBase &) --> class mrpt::nav::PlannerTPS_VirtualBase &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions file:mrpt/nav/planners/PlannerRRT_common.h line:145
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions, std::shared_ptr<mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions>> cl(enclosing_class, "TRenderPlannedPathOptions", "Options for renderMoveTree()  ");
			cl.def( pybind11::init( [](){ return new mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions(); } ) );
			cl.def( pybind11::init( [](mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions const &o){ return new mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions(o); } ) );
			cl.def_readwrite("highlight_path_to_node_id", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::highlight_path_to_node_id);
			cl.def_readwrite("draw_shape_decimation", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::draw_shape_decimation);
			cl.def_readwrite("xyzcorners_scale", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::xyzcorners_scale);
			cl.def_readwrite("highlight_last_added_edge", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::highlight_last_added_edge);
			cl.def_readwrite("ground_xy_grid_frequency", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::ground_xy_grid_frequency);
			cl.def_readwrite("color_vehicle", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::color_vehicle);
			cl.def_readwrite("color_obstacles", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::color_obstacles);
			cl.def_readwrite("color_local_obstacles", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::color_local_obstacles);
			cl.def_readwrite("color_start", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::color_start);
			cl.def_readwrite("color_goal", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::color_goal);
			cl.def_readwrite("color_ground_xy_grid", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::color_ground_xy_grid);
			cl.def_readwrite("color_normal_edge", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::color_normal_edge);
			cl.def_readwrite("color_last_edge", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::color_last_edge);
			cl.def_readwrite("color_optimal_edge", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::color_optimal_edge);
			cl.def_readwrite("width_last_edge", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::width_last_edge);
			cl.def_readwrite("width_normal_edge", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::width_normal_edge);
			cl.def_readwrite("width_optimal_edge", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::width_optimal_edge);
			cl.def_readwrite("point_size_obstacles", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::point_size_obstacles);
			cl.def_readwrite("point_size_local_obstacles", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::point_size_local_obstacles);
			cl.def_readwrite("vehicle_shape_z", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::vehicle_shape_z);
			cl.def_readwrite("vehicle_line_width", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::vehicle_line_width);
			cl.def_readwrite("draw_obstacles", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::draw_obstacles);
			cl.def_readwrite("log_msg", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::log_msg);
			cl.def_readwrite("log_msg_position", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::log_msg_position);
			cl.def_readwrite("log_msg_scale", &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::log_msg_scale);
			cl.def("assign", (struct mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions & (mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::*)(const struct mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions &)) &mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::operator=, "C++: mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions::operator=(const struct mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions &) --> struct mrpt::nav::PlannerTPS_VirtualBase::TRenderPlannedPathOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
}
