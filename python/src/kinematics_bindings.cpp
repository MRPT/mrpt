/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#include "bindings.h"

/* MRPT */
#include <mrpt/kinematics/CVehicleSimul_DiffDriven.h>

/* STD */
#include <cstdint>

using namespace boost::python;
using namespace mrpt::kinematics;
using namespace mrpt::poses;
using namespace mrpt::math;

// CVehicleSimul_DiffDriven
CPose2D CVehicleSimul_DiffDriven_getOdometry(CVehicleSimul_DiffDriven& self)
{
	return CPose2D(self.getCurrentOdometricPose());
}

CPose2D CVehicleSimul_DiffDriven_getRealPose(CVehicleSimul_DiffDriven& self)
{
	return CPose2D(self.getCurrentGTPose());
}

TTwist2D CVehicleSimul_DiffDriven_getCurrentGTVel(
	CVehicleSimul_DiffDriven& self)
{
	return self.getCurrentGTVel();
}
TTwist2D CVehicleSimul_DiffDriven_getCurrentGTVelLocal(
	CVehicleSimul_DiffDriven& self)
{
	return self.getCurrentGTVelLocal();
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(
	CVehicleSimul_DiffDriven_setDelayModelParams_overloads, setDelayModelParams,
	0, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(
	CVehicleSimul_DiffDriven_setOdometryErrors_overloads, setOdometryErrors, 1,
	7)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(
	CVehicleSimul_DiffDriven_resetOdometry_overloads, resetOdometry, 0, 1)
// end of CVehicleSimul_DiffDriven

// exporter
void export_kinematics()
{
	// map namespace to be submodule of mrpt package
	MAKE_SUBMODULE(kinematics)

	// CVehicleSimul_DiffDriven
	{
		class_<CVehicleSimul_DiffDriven>("CVehicleSimul_DiffDriven", init<>())
			.def(
				"setDelayModelParams",
				&CVehicleSimul_DiffDriven::setDelayModelParams,
				CVehicleSimul_DiffDriven_setDelayModelParams_overloads(
					args("TAU_delay_sec=1.8f", "CMD_delay_sec=0.3f"),
					"Change the model of delays used for the orders sent to "
					"the robot"))
			.def(
				"setOdometryErrors",
				&CVehicleSimul_DiffDriven::setOdometryErrors,
				CVehicleSimul_DiffDriven_setOdometryErrors_overloads(
					args(
						"enabled", "Ax_err_bias", "Ax_err_std", "Ay_err_bias",
						"Ay_err_std", "Aphi_err_bias", "Aphi_err_std"),
					"Enable/Disable odometry errors. Errors in odometry are "
					"introduced per millisecond."))
			//			.def("setRealPose",
			//&CVehicleSimul_DiffDriven_setRealPose,
			//"Reset actual robot pose (inmediately, without simulating the
			// movement along time).")
			.def(
				"getCurrentGTPose", &CVehicleSimul_DiffDriven_getRealPose,
				"Returns the instantaneous, ground truth pose in world "
				"coordinates.")
			.def(
				"getCurrentOdometricPose",
				&CVehicleSimul_DiffDriven_getOdometry,
				" Returns the current pose according to (noisy) odometry.")
			.def(
				"getTime", &CVehicleSimul_DiffDriven::getTime,
				"Get the current simulation time.")
			.def(
				"getCurrentGTVel", &CVehicleSimul_DiffDriven_getCurrentGTVel,
				"Returns the instantaneous, ground truth velocity vector "
				"(vx,vy,omega) in world coordinates.")
			.def(
				"getCurrentGTVelLocal",
				&CVehicleSimul_DiffDriven_getCurrentGTVelLocal,
				"Returns the instantaneous, ground truth velocity vector "
				"(vx,vy,omega) in the robot local frame .")
			.def(
				"setV", &CVehicleSimul_DiffDriven::setV, args("v"),
				"Set actual robot velocity, error-free status of the simulated "
				"robot.")
			.def(
				"setW", &CVehicleSimul_DiffDriven::setW, args("w"),
				"Set actual robot turnrate, error-free status of the simulated "
				"robot.")
			.def(
				"movementCommand", &CVehicleSimul_DiffDriven::movementCommand,
				args("lin_vel", "ang_vel"),
				"Used to command the robot a desired movement (velocities).")
			.def(
				"resetStatus", &CVehicleSimul_DiffDriven::resetStatus,
				"Reset all the simulator variables to 0 (All but current "
				"simulator time).")
			.def(
				"resetTime", &CVehicleSimul_DiffDriven::resetTime,
				"Reset time counter.")
			.def(
				"simulateOneTimeStep",
				&CVehicleSimul_DiffDriven::simulateOneTimeStep, args("dt"),
				"This method must be called periodically to simulate discrete "
				"time intervals.")
			.def(
				"resetStatus", &CVehicleSimul_DiffDriven::resetStatus,
				"Reset all simulator variables to 0 (except the simulation "
				"time).");
	}
}
