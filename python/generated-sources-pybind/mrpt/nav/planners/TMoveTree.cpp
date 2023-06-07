#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/containers/map_as_vector.h>
#include <mrpt/containers/traits_map.h>
#include <mrpt/kinematics/CVehicleVelCmd.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TTwist2D.h>
#include <mrpt/nav/holonomic/ClearanceDiagram.h>
#include <mrpt/nav/planners/PlannerRRT_SE2_TPS.h>
#include <mrpt/nav/planners/PlannerSimple2D.h>
#include <mrpt/nav/planners/TMoveTree.h>
#include <mrpt/nav/planners/nav_plan_geometry_utils.h>
#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>
#include <optional>
#include <sstream> // __str__
#include <string>
#include <utility>
#include <vector>

#include <functional>
#include <pybind11/pybind11.h>
#include <string>
#include <stl_binders.hpp>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

void bind_mrpt_nav_planners_TMoveTree(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::nav::TMoveTree file:mrpt/nav/planners/TMoveTree.h line:51
		pybind11::class_<mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP,mrpt::nav::TMoveEdgeSE2_TP,mrpt::containers::map_traits_map_as_vector>, std::shared_ptr<mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP,mrpt::nav::TMoveEdgeSE2_TP,mrpt::containers::map_traits_map_as_vector>>, mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>> cl(M("mrpt::nav"), "TMoveTree_mrpt_nav_TNodeSE2_TP_mrpt_nav_TMoveEdgeSE2_TP_mrpt_containers_map_traits_map_as_vector_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP,mrpt::nav::TMoveEdgeSE2_TP,mrpt::containers::map_traits_map_as_vector>(); } ) );
		cl.def( pybind11::init( [](mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP,mrpt::nav::TMoveEdgeSE2_TP,mrpt::containers::map_traits_map_as_vector> const &o){ return new mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP,mrpt::nav::TMoveEdgeSE2_TP,mrpt::containers::map_traits_map_as_vector>(o); } ) );
		cl.def("insertNodeAndEdge", (void (mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP,mrpt::nav::TMoveEdgeSE2_TP,mrpt::containers::map_traits_map_as_vector>::*)(const unsigned long, const unsigned long, const struct mrpt::nav::TNodeSE2_TP &, const struct mrpt::nav::TMoveEdgeSE2_TP &)) &mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP, mrpt::nav::TMoveEdgeSE2_TP, mrpt::containers::map_traits_map_as_vector>::insertNodeAndEdge, "C++: mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP, mrpt::nav::TMoveEdgeSE2_TP, mrpt::containers::map_traits_map_as_vector>::insertNodeAndEdge(const unsigned long, const unsigned long, const struct mrpt::nav::TNodeSE2_TP &, const struct mrpt::nav::TMoveEdgeSE2_TP &) --> void", pybind11::arg("parent_id"), pybind11::arg("new_child_id"), pybind11::arg("new_child_node_data"), pybind11::arg("new_edge_data"));
		cl.def("insertNode", (void (mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP,mrpt::nav::TMoveEdgeSE2_TP,mrpt::containers::map_traits_map_as_vector>::*)(const unsigned long, const struct mrpt::nav::TNodeSE2_TP &)) &mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP, mrpt::nav::TMoveEdgeSE2_TP, mrpt::containers::map_traits_map_as_vector>::insertNode, "C++: mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP, mrpt::nav::TMoveEdgeSE2_TP, mrpt::containers::map_traits_map_as_vector>::insertNode(const unsigned long, const struct mrpt::nav::TNodeSE2_TP &) --> void", pybind11::arg("node_id"), pybind11::arg("node_data"));
		cl.def("getNextFreeNodeID", (mrpt::graphs::TNodeID (mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP,mrpt::nav::TMoveEdgeSE2_TP,mrpt::containers::map_traits_map_as_vector>::*)() const) &mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP, mrpt::nav::TMoveEdgeSE2_TP, mrpt::containers::map_traits_map_as_vector>::getNextFreeNodeID, "C++: mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP, mrpt::nav::TMoveEdgeSE2_TP, mrpt::containers::map_traits_map_as_vector>::getNextFreeNodeID() const --> mrpt::graphs::TNodeID");
		cl.def("assign", (class mrpt::nav::TMoveTree<struct mrpt::nav::TNodeSE2_TP, struct mrpt::nav::TMoveEdgeSE2_TP, struct mrpt::containers::map_traits_map_as_vector> & (mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP,mrpt::nav::TMoveEdgeSE2_TP,mrpt::containers::map_traits_map_as_vector>::*)(const class mrpt::nav::TMoveTree<struct mrpt::nav::TNodeSE2_TP, struct mrpt::nav::TMoveEdgeSE2_TP, struct mrpt::containers::map_traits_map_as_vector> &)) &mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP, mrpt::nav::TMoveEdgeSE2_TP, mrpt::containers::map_traits_map_as_vector>::operator=, "C++: mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP, mrpt::nav::TMoveEdgeSE2_TP, mrpt::containers::map_traits_map_as_vector>::operator=(const class mrpt::nav::TMoveTree<struct mrpt::nav::TNodeSE2_TP, struct mrpt::nav::TMoveEdgeSE2_TP, struct mrpt::containers::map_traits_map_as_vector> &) --> class mrpt::nav::TMoveTree<struct mrpt::nav::TNodeSE2_TP, struct mrpt::nav::TMoveEdgeSE2_TP, struct mrpt::containers::map_traits_map_as_vector> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		cl.def_readwrite("root", &mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::root);
		cl.def_readwrite("edges_to_children", &mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::edges_to_children);
		cl.def("clear", (void (mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::*)()) &mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::clear, "C++: mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::clear() --> void");
		cl.def("getAsTextDescription", (std::string (mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::*)() const) &mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::getAsTextDescription, "C++: mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::getAsTextDescription() const --> std::string");
		cl.def("assign", (class mrpt::graphs::CDirectedTree<struct mrpt::nav::TMoveEdgeSE2_TP> & (mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::*)(const class mrpt::graphs::CDirectedTree<struct mrpt::nav::TMoveEdgeSE2_TP> &)) &mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::operator=, "C++: mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::operator=(const class mrpt::graphs::CDirectedTree<struct mrpt::nav::TMoveEdgeSE2_TP> &) --> class mrpt::graphs::CDirectedTree<struct mrpt::nav::TMoveEdgeSE2_TP> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::nav::TMoveEdgeSE2_TP file:mrpt/nav/planners/TMoveTree.h line:184
		pybind11::class_<mrpt::nav::TMoveEdgeSE2_TP, std::shared_ptr<mrpt::nav::TMoveEdgeSE2_TP>> cl(M("mrpt::nav"), "TMoveEdgeSE2_TP", "An edge for the move tree used for planning in SE2 and TP-space ");
		cl.def( pybind11::init( [](){ return new mrpt::nav::TMoveEdgeSE2_TP(); } ) );
		cl.def( pybind11::init<const unsigned long, const struct mrpt::math::TPose2D>(), pybind11::arg("parent_id_"), pybind11::arg("end_pose_") );

		cl.def_readwrite("parent_id", &mrpt::nav::TMoveEdgeSE2_TP::parent_id);
		cl.def_readwrite("end_state", &mrpt::nav::TMoveEdgeSE2_TP::end_state);
		cl.def_readwrite("cost", &mrpt::nav::TMoveEdgeSE2_TP::cost);
		cl.def_readwrite("ptg_index", &mrpt::nav::TMoveEdgeSE2_TP::ptg_index);
		cl.def_readwrite("ptg_K", &mrpt::nav::TMoveEdgeSE2_TP::ptg_K);
		cl.def_readwrite("ptg_dist", &mrpt::nav::TMoveEdgeSE2_TP::ptg_dist);
	}
	{ // mrpt::nav::TNodeSE2 file:mrpt/nav/planners/TMoveTree.h line:216
		pybind11::class_<mrpt::nav::TNodeSE2, std::shared_ptr<mrpt::nav::TNodeSE2>> cl(M("mrpt::nav"), "TNodeSE2", "");
		cl.def( pybind11::init<const struct mrpt::math::TPose2D &>(), pybind11::arg("state_") );

		cl.def( pybind11::init( [](){ return new mrpt::nav::TNodeSE2(); } ) );
		cl.def_readwrite("state", &mrpt::nav::TNodeSE2::state);
	}
	{ // mrpt::nav::PoseDistanceMetric file:mrpt/nav/planners/TMoveTree.h line:226
		pybind11::class_<mrpt::nav::PoseDistanceMetric<mrpt::nav::TNodeSE2>, std::shared_ptr<mrpt::nav::PoseDistanceMetric<mrpt::nav::TNodeSE2>>> cl(M("mrpt::nav"), "PoseDistanceMetric_mrpt_nav_TNodeSE2_t", "Pose metric for SE(2) ");
		cl.def( pybind11::init( [](){ return new mrpt::nav::PoseDistanceMetric<mrpt::nav::TNodeSE2>(); } ) );
		cl.def("cannotBeNearerThan", (bool (mrpt::nav::PoseDistanceMetric<mrpt::nav::TNodeSE2>::*)(const struct mrpt::nav::TNodeSE2 &, const struct mrpt::nav::TNodeSE2 &, const double) const) &mrpt::nav::PoseDistanceMetric<mrpt::nav::TNodeSE2>::cannotBeNearerThan, "C++: mrpt::nav::PoseDistanceMetric<mrpt::nav::TNodeSE2>::cannotBeNearerThan(const struct mrpt::nav::TNodeSE2 &, const struct mrpt::nav::TNodeSE2 &, const double) const --> bool", pybind11::arg("a"), pybind11::arg("b"), pybind11::arg("d"));
		cl.def("distance", (double (mrpt::nav::PoseDistanceMetric<mrpt::nav::TNodeSE2>::*)(const struct mrpt::nav::TNodeSE2 &, const struct mrpt::nav::TNodeSE2 &) const) &mrpt::nav::PoseDistanceMetric<mrpt::nav::TNodeSE2>::distance, "C++: mrpt::nav::PoseDistanceMetric<mrpt::nav::TNodeSE2>::distance(const struct mrpt::nav::TNodeSE2 &, const struct mrpt::nav::TNodeSE2 &) const --> double", pybind11::arg("a"), pybind11::arg("b"));
	}
	{ // mrpt::nav::TNodeSE2_TP file:mrpt/nav/planners/TMoveTree.h line:245
		pybind11::class_<mrpt::nav::TNodeSE2_TP, std::shared_ptr<mrpt::nav::TNodeSE2_TP>> cl(M("mrpt::nav"), "TNodeSE2_TP", "");
		cl.def( pybind11::init<const struct mrpt::math::TPose2D &>(), pybind11::arg("state_") );

		cl.def( pybind11::init( [](){ return new mrpt::nav::TNodeSE2_TP(); } ) );
		cl.def_readwrite("state", &mrpt::nav::TNodeSE2_TP::state);
	}
	{ // mrpt::nav::PoseDistanceMetric file:mrpt/nav/planners/TMoveTree.h line:256
		pybind11::class_<mrpt::nav::PoseDistanceMetric<mrpt::nav::TNodeSE2_TP>, std::shared_ptr<mrpt::nav::PoseDistanceMetric<mrpt::nav::TNodeSE2_TP>>> cl(M("mrpt::nav"), "PoseDistanceMetric_mrpt_nav_TNodeSE2_TP_t", "Pose metric for SE(2) limited to a given PTG manifold. NOTE: This 'metric'\n is NOT symmetric for all PTGs: d(a,b)!=d(b,a) ");
		cl.def( pybind11::init<const class mrpt::nav::CParameterizedTrajectoryGenerator &>(), pybind11::arg("ptg") );

		cl.def("cannotBeNearerThan", (bool (mrpt::nav::PoseDistanceMetric<mrpt::nav::TNodeSE2_TP>::*)(const struct mrpt::nav::TNodeSE2_TP &, const struct mrpt::nav::TNodeSE2_TP &, const double) const) &mrpt::nav::PoseDistanceMetric<mrpt::nav::TNodeSE2_TP>::cannotBeNearerThan, "C++: mrpt::nav::PoseDistanceMetric<mrpt::nav::TNodeSE2_TP>::cannotBeNearerThan(const struct mrpt::nav::TNodeSE2_TP &, const struct mrpt::nav::TNodeSE2_TP &, const double) const --> bool", pybind11::arg("a"), pybind11::arg("b"), pybind11::arg("d"));
		cl.def("distance", (double (mrpt::nav::PoseDistanceMetric<mrpt::nav::TNodeSE2_TP>::*)(const struct mrpt::nav::TNodeSE2_TP &, const struct mrpt::nav::TNodeSE2_TP &) const) &mrpt::nav::PoseDistanceMetric<mrpt::nav::TNodeSE2_TP>::distance, "C++: mrpt::nav::PoseDistanceMetric<mrpt::nav::TNodeSE2_TP>::distance(const struct mrpt::nav::TNodeSE2_TP &, const struct mrpt::nav::TNodeSE2_TP &) const --> double", pybind11::arg("src"), pybind11::arg("dst"));
	}
	{ // mrpt::nav::PlannerRRT_SE2_TPS file:mrpt/nav/planners/PlannerRRT_SE2_TPS.h line:67
		pybind11::class_<mrpt::nav::PlannerRRT_SE2_TPS, std::shared_ptr<mrpt::nav::PlannerRRT_SE2_TPS>, mrpt::nav::PlannerTPS_VirtualBase> cl(M("mrpt::nav"), "PlannerRRT_SE2_TPS", "TP Space-based RRT path planning for SE(2) (planar) robots.\n\n  This planner algorithm is described in the paper:\n   - M. Bellone, J.L. Blanco, A. Gimenez, \"TP-Space RRT: Kinematic path\n planning of non-holonomic any-shape vehicles\", International Journal of\n Advanced Robotic Systems, 2015.\n\n  Typical usage:\n \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n  - Changes history:\n    - 06/MAR/2014: Creation (MB)\n    - 06/JAN/2015: Refactoring (JLBC)\n\n  \n Factorize into more generic path planner classes!  //template <class\n POSE, class MOTIONS>...");
		cl.def( pybind11::init( [](){ return new mrpt::nav::PlannerRRT_SE2_TPS(); } ) );
		cl.def("loadConfig", [](mrpt::nav::PlannerRRT_SE2_TPS &o, const class mrpt::config::CConfigFileBase & a0) -> void { return o.loadConfig(a0); }, "", pybind11::arg("cfgSource"));
		cl.def("loadConfig", (void (mrpt::nav::PlannerRRT_SE2_TPS::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::nav::PlannerRRT_SE2_TPS::loadConfig, "Load all params from a config file source \n\nC++: mrpt::nav::PlannerRRT_SE2_TPS::loadConfig(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("cfgSource"), pybind11::arg("sSectionName"));
		cl.def("initialize", (void (mrpt::nav::PlannerRRT_SE2_TPS::*)()) &mrpt::nav::PlannerRRT_SE2_TPS::initialize, "Must be called after setting all params (see `loadConfig()`) and before\n calling `solve()` \n\nC++: mrpt::nav::PlannerRRT_SE2_TPS::initialize() --> void");
		cl.def("solve", (void (mrpt::nav::PlannerRRT_SE2_TPS::*)(const struct mrpt::nav::PlannerRRT_SE2_TPS::TPlannerInput &, struct mrpt::nav::PlannerRRT_SE2_TPS::TPlannerResult &)) &mrpt::nav::PlannerRRT_SE2_TPS::solve, "The main API entry point: tries to find a planned path from 'goal' to\n 'target' \n\nC++: mrpt::nav::PlannerRRT_SE2_TPS::solve(const struct mrpt::nav::PlannerRRT_SE2_TPS::TPlannerInput &, struct mrpt::nav::PlannerRRT_SE2_TPS::TPlannerResult &) --> void", pybind11::arg("pi"), pybind11::arg("result"));

		{ // mrpt::nav::PlannerRRT_SE2_TPS::TPlannerInput file:mrpt/nav/planners/PlannerRRT_SE2_TPS.h line:73
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::nav::PlannerRRT_SE2_TPS::TPlannerInput, std::shared_ptr<mrpt::nav::PlannerRRT_SE2_TPS::TPlannerInput>, mrpt::nav::TPlannerInputTempl<mrpt::math::TPose2D,mrpt::math::TPose2D>> cl(enclosing_class, "TPlannerInput", "");
			cl.def( pybind11::init( [](){ return new mrpt::nav::PlannerRRT_SE2_TPS::TPlannerInput(); } ) );
		}

		{ // mrpt::nav::PlannerRRT_SE2_TPS::TPlannerResult file:mrpt/nav/planners/PlannerRRT_SE2_TPS.h line:84
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::nav::PlannerRRT_SE2_TPS::TPlannerResult, std::shared_ptr<mrpt::nav::PlannerRRT_SE2_TPS::TPlannerResult>, mrpt::nav::TPlannerResultTempl<mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP, mrpt::nav::TMoveEdgeSE2_TP, mrpt::containers::map_traits_map_as_vector>>> cl(enclosing_class, "TPlannerResult", "");
			cl.def( pybind11::init( [](){ return new mrpt::nav::PlannerRRT_SE2_TPS::TPlannerResult(); } ) );
		}

	}
	{ // mrpt::nav::PlannerSimple2D file:mrpt/nav/planners/PlannerSimple2D.h line:31
		pybind11::class_<mrpt::nav::PlannerSimple2D, std::shared_ptr<mrpt::nav::PlannerSimple2D>> cl(M("mrpt::nav"), "PlannerSimple2D", "Searches for collision-free path in 2D occupancy grids for holonomic\n circular robots.\n  The implementation first enlargest obstacles with robot radius, then applies\n a\n  wavefront algorithm to find the shortest free path between origin and target\n 2D points.\n\n Notice that this simple planner does not take into account robot kinematic\n constraints.");
		cl.def( pybind11::init( [](){ return new mrpt::nav::PlannerSimple2D(); } ) );
		cl.def_readwrite("occupancyThreshold", &mrpt::nav::PlannerSimple2D::occupancyThreshold);
		cl.def_readwrite("minStepInReturnedPath", &mrpt::nav::PlannerSimple2D::minStepInReturnedPath);
		cl.def_readwrite("robotRadius", &mrpt::nav::PlannerSimple2D::robotRadius);
		cl.def("assign", (class mrpt::nav::PlannerSimple2D & (mrpt::nav::PlannerSimple2D::*)(const class mrpt::nav::PlannerSimple2D &)) &mrpt::nav::PlannerSimple2D::operator=, "C++: mrpt::nav::PlannerSimple2D::operator=(const class mrpt::nav::PlannerSimple2D &) --> class mrpt::nav::PlannerSimple2D &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	// mrpt::nav::collision_free_dist_segment_circ_robot(const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPoint2D_<double> &, const double, const struct mrpt::math::TPoint2D_<double> &, double &) file:mrpt/nav/planners/nav_plan_geometry_utils.h line:28
	M("mrpt::nav").def("collision_free_dist_segment_circ_robot", (bool (*)(const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPoint2D_<double> &, const double, const struct mrpt::math::TPoint2D_<double> &, double &)) &mrpt::nav::collision_free_dist_segment_circ_robot, "Computes the collision-free distance for a linear segment path between two\n points, for a circular robot, and a point obstacle (ox,oy).\n \n\n true if a collision exists, and the distance along the segment will\n be in out_col_dist; false otherwise.\n \n\n std::runtime_error If the two points are closer than an epsilon\n (1e-10)\n\nC++: mrpt::nav::collision_free_dist_segment_circ_robot(const struct mrpt::math::TPoint2D_<double> &, const struct mrpt::math::TPoint2D_<double> &, const double, const struct mrpt::math::TPoint2D_<double> &, double &) --> bool", pybind11::arg("p_start"), pybind11::arg("p_end"), pybind11::arg("robot_radius"), pybind11::arg("obstacle"), pybind11::arg("out_col_dist"));

	// mrpt::nav::collision_free_dist_arc_circ_robot(const double, const double, const struct mrpt::math::TPoint2D_<double> &, double &) file:mrpt/nav/planners/nav_plan_geometry_utils.h line:39
	M("mrpt::nav").def("collision_free_dist_arc_circ_robot", (bool (*)(const double, const double, const struct mrpt::math::TPoint2D_<double> &, double &)) &mrpt::nav::collision_free_dist_arc_circ_robot, "Computes the collision-free distance for a forward path (+X) circular arc\n path segment from pose (0,0,0) and radius of curvature R (>0 -> +Y, <0 ->\n -Y), a circular robot and a point obstacle (ox,oy). \n\n true if a\n collision exists, and the distance along the path will be in out_col_dist;\n false otherwise.\n\nC++: mrpt::nav::collision_free_dist_arc_circ_robot(const double, const double, const struct mrpt::math::TPoint2D_<double> &, double &) --> bool", pybind11::arg("arc_radius"), pybind11::arg("robot_radius"), pybind11::arg("obstacle"), pybind11::arg("out_col_dist"));

}
