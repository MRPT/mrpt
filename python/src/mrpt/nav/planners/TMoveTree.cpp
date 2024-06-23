#include <chrono>
#include <functional>
#include <istream>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/containers/map_as_vector.h>
#include <mrpt/containers/traits_map.h>
#include <mrpt/core/Clock.h>
#include <mrpt/core/aligned_allocator.h>
#include <mrpt/core/safe_pointers.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/img/color_maps.h>
#include <mrpt/kinematics/CVehicleVelCmd.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/TMetricMapInitializer.h>
#include <mrpt/maps/metric_map_types.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CPolygon.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TBoundingBox.h>
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
#include <mrpt/nav/reactive/CRobot2NavInterface.h>
#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/obs/T2DScanProperties.h>
#include <mrpt/obs/T3DPointsTo2DScanParams.h>
#include <mrpt/obs/TRangeImageFilter.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ostream>
#include <ratio>
#include <sstream> // __str__
#include <string>
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

// mrpt::nav::CRobot2NavInterface file:mrpt/nav/reactive/CRobot2NavInterface.h line:43
struct PyCallBack_mrpt_nav_CRobot2NavInterface : public mrpt::nav::CRobot2NavInterface {
	using mrpt::nav::CRobot2NavInterface::CRobot2NavInterface;

	bool getCurrentPoseAndSpeeds(struct mrpt::math::TPose2D & a0, struct mrpt::math::TTwist2D & a1, mrpt::Clock::time_point & a2, struct mrpt::math::TPose2D & a3, std::string & a4) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "getCurrentPoseAndSpeeds");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CRobot2NavInterface::getCurrentPoseAndSpeeds\"");
	}
	bool changeSpeeds(const class mrpt::kinematics::CVehicleVelCmd & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "changeSpeeds");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CRobot2NavInterface::changeSpeeds\"");
	}
	bool changeSpeedsNOP() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "changeSpeedsNOP");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CRobot2NavInterface::changeSpeedsNOP();
	}
	bool stop(bool a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "stop");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CRobot2NavInterface::stop\"");
	}
	class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> getEmergencyStopCmd() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "getEmergencyStopCmd");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>::value) {
				static pybind11::detail::override_caster_t<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>> caster;
				return pybind11::detail::cast_ref<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CRobot2NavInterface::getEmergencyStopCmd\"");
	}
	class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> getStopCmd() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "getStopCmd");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>::value) {
				static pybind11::detail::override_caster_t<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>> caster;
				return pybind11::detail::cast_ref<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CRobot2NavInterface::getStopCmd\"");
	}
	class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> getAlignCmd(const double a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "getAlignCmd");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>::value) {
				static pybind11::detail::override_caster_t<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>> caster;
				return pybind11::detail::cast_ref<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>>(std::move(o));
		}
		return CRobot2NavInterface::getAlignCmd(a0);
	}
	bool startWatchdog(float a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "startWatchdog");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CRobot2NavInterface::startWatchdog(a0);
	}
	bool stopWatchdog() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "stopWatchdog");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CRobot2NavInterface::stopWatchdog();
	}
	bool senseObstacles(class mrpt::maps::CSimplePointsMap & a0, mrpt::Clock::time_point & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "senseObstacles");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CRobot2NavInterface::senseObstacles\"");
	}
	void sendNavigationStartEvent() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "sendNavigationStartEvent");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRobot2NavInterface::sendNavigationStartEvent();
	}
	void sendNavigationEndEvent() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "sendNavigationEndEvent");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRobot2NavInterface::sendNavigationEndEvent();
	}
	void sendWaypointReachedEvent(int a0, bool a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "sendWaypointReachedEvent");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRobot2NavInterface::sendWaypointReachedEvent(a0, a1);
	}
	void sendNewWaypointTargetEvent(int a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "sendNewWaypointTargetEvent");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRobot2NavInterface::sendNewWaypointTargetEvent(a0);
	}
	void sendNavigationEndDueToErrorEvent() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "sendNavigationEndDueToErrorEvent");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRobot2NavInterface::sendNavigationEndDueToErrorEvent();
	}
	void sendWaySeemsBlockedEvent() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "sendWaySeemsBlockedEvent");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRobot2NavInterface::sendWaySeemsBlockedEvent();
	}
	void sendApparentCollisionEvent() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "sendApparentCollisionEvent");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRobot2NavInterface::sendApparentCollisionEvent();
	}
	void sendCannotGetCloserToBlockedTargetEvent() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "sendCannotGetCloserToBlockedTargetEvent");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRobot2NavInterface::sendCannotGetCloserToBlockedTargetEvent();
	}
	double getNavigationTime() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "getNavigationTime");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CRobot2NavInterface::getNavigationTime();
	}
	void resetNavigationTimer() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::nav::CRobot2NavInterface *>(this), "resetNavigationTimer");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRobot2NavInterface::resetNavigationTimer();
	}
};

void bind_mrpt_nav_planners_TMoveTree(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::nav::TMoveTree file:mrpt/nav/planners/TMoveTree.h line:52
		pybind11::class_<mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP,mrpt::nav::TMoveEdgeSE2_TP,mrpt::containers::map_traits_map_as_vector>, std::shared_ptr<mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP,mrpt::nav::TMoveEdgeSE2_TP,mrpt::containers::map_traits_map_as_vector>>, mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>> cl(M("mrpt::nav"), "TMoveTree_mrpt_nav_TNodeSE2_TP_mrpt_nav_TMoveEdgeSE2_TP_mrpt_containers_map_traits_map_as_vector_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP,mrpt::nav::TMoveEdgeSE2_TP,mrpt::containers::map_traits_map_as_vector>(); } ) );
		cl.def( pybind11::init( [](mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP,mrpt::nav::TMoveEdgeSE2_TP,mrpt::containers::map_traits_map_as_vector> const &o){ return new mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP,mrpt::nav::TMoveEdgeSE2_TP,mrpt::containers::map_traits_map_as_vector>(o); } ) );
		cl.def("insertNodeAndEdge", (void (mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP,mrpt::nav::TMoveEdgeSE2_TP,mrpt::containers::map_traits_map_as_vector>::*)(const unsigned long, const unsigned long, const struct mrpt::nav::TNodeSE2_TP &, const struct mrpt::nav::TMoveEdgeSE2_TP &)) &mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP, mrpt::nav::TMoveEdgeSE2_TP>::insertNodeAndEdge, "C++: mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP, mrpt::nav::TMoveEdgeSE2_TP>::insertNodeAndEdge(const unsigned long, const unsigned long, const struct mrpt::nav::TNodeSE2_TP &, const struct mrpt::nav::TMoveEdgeSE2_TP &) --> void", pybind11::arg("parent_id"), pybind11::arg("new_child_id"), pybind11::arg("new_child_node_data"), pybind11::arg("new_edge_data"));
		cl.def("insertNode", (void (mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP,mrpt::nav::TMoveEdgeSE2_TP,mrpt::containers::map_traits_map_as_vector>::*)(const unsigned long, const struct mrpt::nav::TNodeSE2_TP &)) &mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP, mrpt::nav::TMoveEdgeSE2_TP>::insertNode, "C++: mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP, mrpt::nav::TMoveEdgeSE2_TP>::insertNode(const unsigned long, const struct mrpt::nav::TNodeSE2_TP &) --> void", pybind11::arg("node_id"), pybind11::arg("node_data"));
		cl.def("getNextFreeNodeID", (mrpt::graphs::TNodeID (mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP,mrpt::nav::TMoveEdgeSE2_TP,mrpt::containers::map_traits_map_as_vector>::*)() const) &mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP, mrpt::nav::TMoveEdgeSE2_TP>::getNextFreeNodeID, "C++: mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP, mrpt::nav::TMoveEdgeSE2_TP>::getNextFreeNodeID() const --> mrpt::graphs::TNodeID");
		cl.def("assign", (class mrpt::nav::TMoveTree<struct mrpt::nav::TNodeSE2_TP, struct mrpt::nav::TMoveEdgeSE2_TP> & (mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP,mrpt::nav::TMoveEdgeSE2_TP,mrpt::containers::map_traits_map_as_vector>::*)(const class mrpt::nav::TMoveTree<struct mrpt::nav::TNodeSE2_TP, struct mrpt::nav::TMoveEdgeSE2_TP> &)) &mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP, mrpt::nav::TMoveEdgeSE2_TP>::operator=, "C++: mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP, mrpt::nav::TMoveEdgeSE2_TP>::operator=(const class mrpt::nav::TMoveTree<struct mrpt::nav::TNodeSE2_TP, struct mrpt::nav::TMoveEdgeSE2_TP> &) --> class mrpt::nav::TMoveTree<struct mrpt::nav::TNodeSE2_TP, struct mrpt::nav::TMoveEdgeSE2_TP> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		cl.def_readwrite("root", &mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::root);
		cl.def_readwrite("edges_to_children", &mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::edges_to_children);
		cl.def("clear", (void (mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::*)()) &mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::clear, "C++: mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::clear() --> void");
		cl.def("getAsTextDescription", (std::string (mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::*)() const) &mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::getAsTextDescription, "C++: mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::getAsTextDescription() const --> std::string");
		cl.def("assign", (class mrpt::graphs::CDirectedTree<struct mrpt::nav::TMoveEdgeSE2_TP> & (mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::*)(const class mrpt::graphs::CDirectedTree<struct mrpt::nav::TMoveEdgeSE2_TP> &)) &mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::operator=, "C++: mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::operator=(const class mrpt::graphs::CDirectedTree<struct mrpt::nav::TMoveEdgeSE2_TP> &) --> class mrpt::graphs::CDirectedTree<struct mrpt::nav::TMoveEdgeSE2_TP> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::nav::TMoveEdgeSE2_TP file:mrpt/nav/planners/TMoveTree.h line:179
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
	{ // mrpt::nav::TNodeSE2 file:mrpt/nav/planners/TMoveTree.h line:209
		pybind11::class_<mrpt::nav::TNodeSE2, std::shared_ptr<mrpt::nav::TNodeSE2>> cl(M("mrpt::nav"), "TNodeSE2", "");
		cl.def( pybind11::init<const struct mrpt::math::TPose2D &>(), pybind11::arg("state_") );

		cl.def( pybind11::init( [](){ return new mrpt::nav::TNodeSE2(); } ) );
		cl.def_readwrite("state", &mrpt::nav::TNodeSE2::state);
	}
	{ // mrpt::nav::PoseDistanceMetric file:mrpt/nav/planners/TMoveTree.h line:219
		pybind11::class_<mrpt::nav::PoseDistanceMetric<mrpt::nav::TNodeSE2>, std::shared_ptr<mrpt::nav::PoseDistanceMetric<mrpt::nav::TNodeSE2>>> cl(M("mrpt::nav"), "PoseDistanceMetric_mrpt_nav_TNodeSE2_t", "Pose metric for SE(2) ");
		cl.def( pybind11::init( [](){ return new mrpt::nav::PoseDistanceMetric<mrpt::nav::TNodeSE2>(); } ) );
		cl.def("cannotBeNearerThan", (bool (mrpt::nav::PoseDistanceMetric<mrpt::nav::TNodeSE2>::*)(const struct mrpt::nav::TNodeSE2 &, const struct mrpt::nav::TNodeSE2 &, const double) const) &mrpt::nav::PoseDistanceMetric<mrpt::nav::TNodeSE2>::cannotBeNearerThan, "C++: mrpt::nav::PoseDistanceMetric<mrpt::nav::TNodeSE2>::cannotBeNearerThan(const struct mrpt::nav::TNodeSE2 &, const struct mrpt::nav::TNodeSE2 &, const double) const --> bool", pybind11::arg("a"), pybind11::arg("b"), pybind11::arg("d"));
		cl.def("distance", (double (mrpt::nav::PoseDistanceMetric<mrpt::nav::TNodeSE2>::*)(const struct mrpt::nav::TNodeSE2 &, const struct mrpt::nav::TNodeSE2 &) const) &mrpt::nav::PoseDistanceMetric<mrpt::nav::TNodeSE2>::distance, "C++: mrpt::nav::PoseDistanceMetric<mrpt::nav::TNodeSE2>::distance(const struct mrpt::nav::TNodeSE2 &, const struct mrpt::nav::TNodeSE2 &) const --> double", pybind11::arg("a"), pybind11::arg("b"));
	}
	{ // mrpt::nav::TNodeSE2_TP file:mrpt/nav/planners/TMoveTree.h line:236
		pybind11::class_<mrpt::nav::TNodeSE2_TP, std::shared_ptr<mrpt::nav::TNodeSE2_TP>> cl(M("mrpt::nav"), "TNodeSE2_TP", "");
		cl.def( pybind11::init<const struct mrpt::math::TPose2D &>(), pybind11::arg("state_") );

		cl.def( pybind11::init( [](){ return new mrpt::nav::TNodeSE2_TP(); } ) );
		cl.def_readwrite("state", &mrpt::nav::TNodeSE2_TP::state);
	}
	{ // mrpt::nav::PoseDistanceMetric file:mrpt/nav/planners/TMoveTree.h line:247
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
			pybind11::class_<mrpt::nav::PlannerRRT_SE2_TPS::TPlannerResult, std::shared_ptr<mrpt::nav::PlannerRRT_SE2_TPS::TPlannerResult>, mrpt::nav::TPlannerResultTempl<mrpt::nav::TMoveTree<mrpt::nav::TNodeSE2_TP, mrpt::nav::TMoveEdgeSE2_TP>>> cl(enclosing_class, "TPlannerResult", "");
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

	// mrpt::nav::collision_free_dist_arc_circ_robot(const double, const double, const struct mrpt::math::TPoint2D_<double> &, double &) file:mrpt/nav/planners/nav_plan_geometry_utils.h line:41
	M("mrpt::nav").def("collision_free_dist_arc_circ_robot", (bool (*)(const double, const double, const struct mrpt::math::TPoint2D_<double> &, double &)) &mrpt::nav::collision_free_dist_arc_circ_robot, "Computes the collision-free distance for a forward path (+X) circular arc\n path segment from pose (0,0,0) and radius of curvature R (>0 -> +Y, <0 ->\n -Y), a circular robot and a point obstacle (ox,oy). \n\n true if a\n collision exists, and the distance along the path will be in out_col_dist;\n false otherwise.\n\nC++: mrpt::nav::collision_free_dist_arc_circ_robot(const double, const double, const struct mrpt::math::TPoint2D_<double> &, double &) --> bool", pybind11::arg("arc_radius"), pybind11::arg("robot_radius"), pybind11::arg("obstacle"), pybind11::arg("out_col_dist"));

	{ // mrpt::nav::CRobot2NavInterface file:mrpt/nav/reactive/CRobot2NavInterface.h line:43
		pybind11::class_<mrpt::nav::CRobot2NavInterface, std::shared_ptr<mrpt::nav::CRobot2NavInterface>, PyCallBack_mrpt_nav_CRobot2NavInterface> cl(M("mrpt::nav"), "CRobot2NavInterface", "The pure virtual interface between a real or simulated robot and any\n `CAbstractNavigator`-derived class.\n\n  The user must define a new class derived from `CRobot2NavInterface` and\n reimplement\n   all pure virtual and the desired virtual methods according to the\n documentation in this class.\n\n This class does not make assumptions about the kinematic\n model of the robot, so it can work with either\n Ackermann, differential-driven or holonomic robots. It will depend on the\n used PTGs, so checkout\n each PTG documentation for the length and meaning of velocity commands.\n\n If used for a simulator, users may prefer to inherit from one of these\n classes, which already provide partial implementations:\n  - mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven\n  - mrpt::nav::CRobot2NavInterfaceForSimulator_Holo\n\n \n CReactiveNavigationSystem, CAbstractNavigator\n  \n\n\n ");
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_nav_CRobot2NavInterface(); } ) );
		cl.def(pybind11::init<PyCallBack_mrpt_nav_CRobot2NavInterface const &>());
		cl.def("getCurrentPoseAndSpeeds", (bool (mrpt::nav::CRobot2NavInterface::*)(struct mrpt::math::TPose2D &, struct mrpt::math::TTwist2D &, mrpt::Clock::time_point &, struct mrpt::math::TPose2D &, std::string &)) &mrpt::nav::CRobot2NavInterface::getCurrentPoseAndSpeeds, "Get the current pose and velocity of the robot. The implementation\n should not take too much time to return,\n   so if it might take more than ~10ms to ask the robot for the\n instantaneous data, it may be good enough to\n   return the latest values from a cache which is updated in a parallel\n thread.\n \n\n false on any error retrieving these values from the robot.\n \n\n\nC++: mrpt::nav::CRobot2NavInterface::getCurrentPoseAndSpeeds(struct mrpt::math::TPose2D &, struct mrpt::math::TTwist2D &, mrpt::Clock::time_point &, struct mrpt::math::TPose2D &, std::string &) --> bool", pybind11::arg("curPose"), pybind11::arg("curVelGlobal"), pybind11::arg("timestamp"), pybind11::arg("curOdometry"), pybind11::arg("frame_id"));
		cl.def("changeSpeeds", (bool (mrpt::nav::CRobot2NavInterface::*)(const class mrpt::kinematics::CVehicleVelCmd &)) &mrpt::nav::CRobot2NavInterface::changeSpeeds, "Sends a velocity command to the robot.\n The number components in each command depends on children classes of\n mrpt::kinematics::CVehicleVelCmd.\n One robot may accept one or more different CVehicleVelCmd classes.\n This method resets the watchdog timer (that may be or may be not\n implemented in a particular robotic platform) started with\n startWatchdog()\n \n\n false on any error.\n \n\n startWatchdog\n \n\n\n   \n\nC++: mrpt::nav::CRobot2NavInterface::changeSpeeds(const class mrpt::kinematics::CVehicleVelCmd &) --> bool", pybind11::arg("vel_cmd"));
		cl.def("changeSpeedsNOP", (bool (mrpt::nav::CRobot2NavInterface::*)()) &mrpt::nav::CRobot2NavInterface::changeSpeedsNOP, "Just like changeSpeeds(), but will be called when the last velocity\n command is still the preferred solution,\n so there is no need to change that past command. The unique effect of\n this callback would be resetting the watchdog timer.\n \n\n false on any error.\n \n\n changeSpeeds(), startWatchdog()\n \n\n\nC++: mrpt::nav::CRobot2NavInterface::changeSpeedsNOP() --> bool");
		cl.def("stop", [](mrpt::nav::CRobot2NavInterface &o) -> bool { return o.stop(); }, "");
		cl.def("stop", (bool (mrpt::nav::CRobot2NavInterface::*)(bool)) &mrpt::nav::CRobot2NavInterface::stop, "Stop the robot right now.\n  \n\n true if stop is due to some unexpected error.\n false if \"stop\" happens as part of a normal operation (e.g. target\n reached).\n \n\n false on any error.\n\nC++: mrpt::nav::CRobot2NavInterface::stop(bool) --> bool", pybind11::arg("isEmergencyStop"));
		cl.def("getEmergencyStopCmd", (class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> (mrpt::nav::CRobot2NavInterface::*)()) &mrpt::nav::CRobot2NavInterface::getEmergencyStopCmd, "Gets the emergency stop command for the current robot\n \n\n the emergency stop command\n\nC++: mrpt::nav::CRobot2NavInterface::getEmergencyStopCmd() --> class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>");
		cl.def("getStopCmd", (class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> (mrpt::nav::CRobot2NavInterface::*)()) &mrpt::nav::CRobot2NavInterface::getStopCmd, "Gets the emergency stop command for the current robot\n \n\n the emergency stop command\n\nC++: mrpt::nav::CRobot2NavInterface::getStopCmd() --> class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>");
		cl.def("getAlignCmd", (class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> (mrpt::nav::CRobot2NavInterface::*)(const double)) &mrpt::nav::CRobot2NavInterface::getAlignCmd, "Gets a motion command to make the robot to align with a given *relative*\n heading, without translating.\n Only for circular robots that can rotate in place; otherwise, return an\n empty smart pointer to indicate\n that the operation is not possible (this is what the default\n implementation does). \n\nC++: mrpt::nav::CRobot2NavInterface::getAlignCmd(const double) --> class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>", pybind11::arg("relative_heading_radians"));
		cl.def("startWatchdog", (bool (mrpt::nav::CRobot2NavInterface::*)(float)) &mrpt::nav::CRobot2NavInterface::startWatchdog, "Start the watchdog timer of the robot platform, if any, for maximum\n expected delay between consecutive calls to changeSpeeds().\n \n\n Period, in ms.\n \n\n false on any error. \n\nC++: mrpt::nav::CRobot2NavInterface::startWatchdog(float) --> bool", pybind11::arg("T_ms"));
		cl.def("stopWatchdog", (bool (mrpt::nav::CRobot2NavInterface::*)()) &mrpt::nav::CRobot2NavInterface::stopWatchdog, "Stop the watchdog timer.\n \n\n false on any error. \n startWatchdog \n\nC++: mrpt::nav::CRobot2NavInterface::stopWatchdog() --> bool");
		cl.def("senseObstacles", (bool (mrpt::nav::CRobot2NavInterface::*)(class mrpt::maps::CSimplePointsMap &, mrpt::Clock::time_point &)) &mrpt::nav::CRobot2NavInterface::senseObstacles, "Return the current set of obstacle points, as seen from the local\n coordinate frame of the robot.\n \n\n false on any error.\n \n\n  A representation of obstacles in robot-centric\n coordinates.\n \n\n  The timestamp for the read obstacles. Use\n mrpt::Clock::now() unless you have something more accurate.\n\nC++: mrpt::nav::CRobot2NavInterface::senseObstacles(class mrpt::maps::CSimplePointsMap &, mrpt::Clock::time_point &) --> bool", pybind11::arg("obstacles"), pybind11::arg("timestamp"));
		cl.def("sendNavigationStartEvent", (void (mrpt::nav::CRobot2NavInterface::*)()) &mrpt::nav::CRobot2NavInterface::sendNavigationStartEvent, "@{ \n\n Callback: Start of navigation command \n\nC++: mrpt::nav::CRobot2NavInterface::sendNavigationStartEvent() --> void");
		cl.def("sendNavigationEndEvent", (void (mrpt::nav::CRobot2NavInterface::*)()) &mrpt::nav::CRobot2NavInterface::sendNavigationEndEvent, "Callback: End of navigation command (reach of single goal, or final\n waypoint of waypoint list) \n\nC++: mrpt::nav::CRobot2NavInterface::sendNavigationEndEvent() --> void");
		cl.def("sendWaypointReachedEvent", (void (mrpt::nav::CRobot2NavInterface::*)(int, bool)) &mrpt::nav::CRobot2NavInterface::sendWaypointReachedEvent, "Callback: Reached an intermediary waypoint in waypoint list navigation.\n reached_nSkipped will be `true` if the waypoint was physically reached;\n `false` if it was actually \"skipped\".\n\nC++: mrpt::nav::CRobot2NavInterface::sendWaypointReachedEvent(int, bool) --> void", pybind11::arg("waypoint_index"), pybind11::arg("reached_nSkipped"));
		cl.def("sendNewWaypointTargetEvent", (void (mrpt::nav::CRobot2NavInterface::*)(int)) &mrpt::nav::CRobot2NavInterface::sendNewWaypointTargetEvent, "Callback: Heading towards a new intermediary/final waypoint in waypoint\n list navigation \n\nC++: mrpt::nav::CRobot2NavInterface::sendNewWaypointTargetEvent(int) --> void", pybind11::arg("waypoint_index"));
		cl.def("sendNavigationEndDueToErrorEvent", (void (mrpt::nav::CRobot2NavInterface::*)()) &mrpt::nav::CRobot2NavInterface::sendNavigationEndDueToErrorEvent, "Callback: Error asking sensory data from robot or sending motor\n commands. \n\nC++: mrpt::nav::CRobot2NavInterface::sendNavigationEndDueToErrorEvent() --> void");
		cl.def("sendWaySeemsBlockedEvent", (void (mrpt::nav::CRobot2NavInterface::*)()) &mrpt::nav::CRobot2NavInterface::sendWaySeemsBlockedEvent, "Callback: No progression made towards target for a predefined period of\n time. \n\nC++: mrpt::nav::CRobot2NavInterface::sendWaySeemsBlockedEvent() --> void");
		cl.def("sendApparentCollisionEvent", (void (mrpt::nav::CRobot2NavInterface::*)()) &mrpt::nav::CRobot2NavInterface::sendApparentCollisionEvent, "Callback: Apparent collision event (i.e. there is at least one obstacle\n point inside the robot shape) \n\nC++: mrpt::nav::CRobot2NavInterface::sendApparentCollisionEvent() --> void");
		cl.def("sendCannotGetCloserToBlockedTargetEvent", (void (mrpt::nav::CRobot2NavInterface::*)()) &mrpt::nav::CRobot2NavInterface::sendCannotGetCloserToBlockedTargetEvent, "Callback: Target seems to be blocked by an obstacle. \n\nC++: mrpt::nav::CRobot2NavInterface::sendCannotGetCloserToBlockedTargetEvent() --> void");
		cl.def("getNavigationTime", (double (mrpt::nav::CRobot2NavInterface::*)()) &mrpt::nav::CRobot2NavInterface::getNavigationTime, "Returns the number of seconds ellapsed since the constructor of this\n class was invoked, or since\n the last call of resetNavigationTimer(). This will be normally\n wall-clock time, except in simulators where this method will return\n simulation time. \n\nC++: mrpt::nav::CRobot2NavInterface::getNavigationTime() --> double");
		cl.def("resetNavigationTimer", (void (mrpt::nav::CRobot2NavInterface::*)()) &mrpt::nav::CRobot2NavInterface::resetNavigationTimer, "see getNavigationTime() \n\nC++: mrpt::nav::CRobot2NavInterface::resetNavigationTimer() --> void");
		cl.def("assign", (class mrpt::nav::CRobot2NavInterface & (mrpt::nav::CRobot2NavInterface::*)(const class mrpt::nav::CRobot2NavInterface &)) &mrpt::nav::CRobot2NavInterface::operator=, "C++: mrpt::nav::CRobot2NavInterface::operator=(const class mrpt::nav::CRobot2NavInterface &) --> class mrpt::nav::CRobot2NavInterface &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
