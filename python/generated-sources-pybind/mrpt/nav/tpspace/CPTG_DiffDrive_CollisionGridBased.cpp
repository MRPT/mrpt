#include <iterator>
#include <memory>
#include <mrpt/kinematics/CVehicleVelCmd.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/nav/tpspace/CPTG_DiffDrive_CollisionGridBased.h>
#include <sstream> // __str__
#include <string>
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

void bind_mrpt_nav_tpspace_CPTG_DiffDrive_CollisionGridBased(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::nav::CPTG_DiffDrive_CollisionGridBased file:mrpt/nav/tpspace/CPTG_DiffDrive_CollisionGridBased.h line:52
		pybind11::class_<mrpt::nav::CPTG_DiffDrive_CollisionGridBased, std::shared_ptr<mrpt::nav::CPTG_DiffDrive_CollisionGridBased>, mrpt::nav::CPTG_RobotShape_Polygonal> cl(M("mrpt::nav"), "CPTG_DiffDrive_CollisionGridBased", "Base class for all PTGs suitable to non-holonomic, differentially-driven (or\n Ackermann) vehicles\n based on numerical integration of the trajectories and collision\n look-up-table.\n Regarding `initialize()`: in this this family of PTGs, the method builds the\n collision grid or load it from a cache file.\n Collision grids must be calculated before calling getTPObstacle(). Robot\n shape must be set before initializing with setRobotShape().\n The rest of PTG parameters should have been set at the constructor.");
		cl.def("ptgDiffDriveSteeringFunction", (void (mrpt::nav::CPTG_DiffDrive_CollisionGridBased::*)(float, float, float, float, float, float &, float &) const) &mrpt::nav::CPTG_DiffDrive_CollisionGridBased::ptgDiffDriveSteeringFunction, "The main method to be implemented in derived classes: it defines the\n differential-driven differential equation \n\nC++: mrpt::nav::CPTG_DiffDrive_CollisionGridBased::ptgDiffDriveSteeringFunction(float, float, float, float, float, float &, float &) const --> void", pybind11::arg("alpha"), pybind11::arg("t"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("phi"), pybind11::arg("v"), pybind11::arg("w"));
		cl.def("inverseMap_WS2TP", [](mrpt::nav::CPTG_DiffDrive_CollisionGridBased const &o, double const & a0, double const & a1, int & a2, double & a3) -> bool { return o.inverseMap_WS2TP(a0, a1, a2, a3); }, "", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("out_k"), pybind11::arg("out_d"));
		cl.def("inverseMap_WS2TP", (bool (mrpt::nav::CPTG_DiffDrive_CollisionGridBased::*)(double, double, int &, double &, double) const) &mrpt::nav::CPTG_DiffDrive_CollisionGridBased::inverseMap_WS2TP, "The default implementation in this class relies on a look-up-table.\n Derived classes may redefine this to closed-form expressions, when they\n exist.\n See full docs in base class\n CParameterizedTrajectoryGenerator::inverseMap_WS2TP() \n\nC++: mrpt::nav::CPTG_DiffDrive_CollisionGridBased::inverseMap_WS2TP(double, double, int &, double &, double) const --> bool", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("out_k"), pybind11::arg("out_d"), pybind11::arg("tolerance_dist"));
		cl.def("directionToMotionCommand", (class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> (mrpt::nav::CPTG_DiffDrive_CollisionGridBased::*)(unsigned short) const) &mrpt::nav::CPTG_DiffDrive_CollisionGridBased::directionToMotionCommand, "In this class, `out_action_cmd` contains: [0]: linear velocity (m/s),\n [1]: angular velocity (rad/s).\n See more docs in\n CParameterizedTrajectoryGenerator::directionToMotionCommand() \n\nC++: mrpt::nav::CPTG_DiffDrive_CollisionGridBased::directionToMotionCommand(unsigned short) const --> class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>", pybind11::arg("k"));
		cl.def("getSupportedKinematicVelocityCommand", (class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd> (mrpt::nav::CPTG_DiffDrive_CollisionGridBased::*)() const) &mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getSupportedKinematicVelocityCommand, "C++: mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getSupportedKinematicVelocityCommand() const --> class std::shared_ptr<class mrpt::kinematics::CVehicleVelCmd>");
		cl.def("setRefDistance", (void (mrpt::nav::CPTG_DiffDrive_CollisionGridBased::*)(const double)) &mrpt::nav::CPTG_DiffDrive_CollisionGridBased::setRefDistance, "Launches an exception in this class: it is not allowed in numerical\n integration-based PTGs to change the reference distance\n after initialization. \n\nC++: mrpt::nav::CPTG_DiffDrive_CollisionGridBased::setRefDistance(const double) --> void", pybind11::arg("refDist"));
		cl.def("getPathStepCount", (unsigned long (mrpt::nav::CPTG_DiffDrive_CollisionGridBased::*)(unsigned short) const) &mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getPathStepCount, "C++: mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getPathStepCount(unsigned short) const --> unsigned long", pybind11::arg("k"));
		cl.def("getPathPose", (struct mrpt::math::TPose2D (mrpt::nav::CPTG_DiffDrive_CollisionGridBased::*)(unsigned short, unsigned int) const) &mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getPathPose, "C++: mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getPathPose(unsigned short, unsigned int) const --> struct mrpt::math::TPose2D", pybind11::arg("k"), pybind11::arg("step"));
		cl.def("getPathDist", (double (mrpt::nav::CPTG_DiffDrive_CollisionGridBased::*)(unsigned short, unsigned int) const) &mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getPathDist, "C++: mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getPathDist(unsigned short, unsigned int) const --> double", pybind11::arg("k"), pybind11::arg("step"));
		cl.def("getPathStepForDist", (bool (mrpt::nav::CPTG_DiffDrive_CollisionGridBased::*)(unsigned short, double, unsigned int &) const) &mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getPathStepForDist, "C++: mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getPathStepForDist(unsigned short, double, unsigned int &) const --> bool", pybind11::arg("k"), pybind11::arg("dist"), pybind11::arg("out_step"));
		cl.def("getPathStepDuration", (double (mrpt::nav::CPTG_DiffDrive_CollisionGridBased::*)() const) &mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getPathStepDuration, "C++: mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getPathStepDuration() const --> double");
		cl.def("getMaxLinVel", (double (mrpt::nav::CPTG_DiffDrive_CollisionGridBased::*)() const) &mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getMaxLinVel, "C++: mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getMaxLinVel() const --> double");
		cl.def("getMaxAngVel", (double (mrpt::nav::CPTG_DiffDrive_CollisionGridBased::*)() const) &mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getMaxAngVel, "C++: mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getMaxAngVel() const --> double");
		cl.def("updateTPObstacleSingle", (void (mrpt::nav::CPTG_DiffDrive_CollisionGridBased::*)(double, double, unsigned short, double &) const) &mrpt::nav::CPTG_DiffDrive_CollisionGridBased::updateTPObstacleSingle, "C++: mrpt::nav::CPTG_DiffDrive_CollisionGridBased::updateTPObstacleSingle(double, double, unsigned short, double &) const --> void", pybind11::arg("ox"), pybind11::arg("oy"), pybind11::arg("k"), pybind11::arg("tp_obstacle_k"));
		cl.def("onNewNavDynamicState", (void (mrpt::nav::CPTG_DiffDrive_CollisionGridBased::*)()) &mrpt::nav::CPTG_DiffDrive_CollisionGridBased::onNewNavDynamicState, "This family of PTGs ignores the dynamic states \n\nC++: mrpt::nav::CPTG_DiffDrive_CollisionGridBased::onNewNavDynamicState() --> void");
		cl.def("getMax_V", (double (mrpt::nav::CPTG_DiffDrive_CollisionGridBased::*)() const) &mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getMax_V, "@} \n\nC++: mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getMax_V() const --> double");
		cl.def("getMax_W", (double (mrpt::nav::CPTG_DiffDrive_CollisionGridBased::*)() const) &mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getMax_W, "C++: mrpt::nav::CPTG_DiffDrive_CollisionGridBased::getMax_W() const --> double");
		cl.def("assign", (class mrpt::nav::CPTG_DiffDrive_CollisionGridBased & (mrpt::nav::CPTG_DiffDrive_CollisionGridBased::*)(const class mrpt::nav::CPTG_DiffDrive_CollisionGridBased &)) &mrpt::nav::CPTG_DiffDrive_CollisionGridBased::operator=, "C++: mrpt::nav::CPTG_DiffDrive_CollisionGridBased::operator=(const class mrpt::nav::CPTG_DiffDrive_CollisionGridBased &) --> class mrpt::nav::CPTG_DiffDrive_CollisionGridBased &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
