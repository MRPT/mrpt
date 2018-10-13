/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/kinematics/CVehicleVelCmd.h>

namespace mrpt::kinematics
{
/** Kinematic model for
 *
 * \ingroup mrpt_kinematics_grp
 */
class CVehicleVelCmd_Holo : public CVehicleVelCmd
{
	DEFINE_SERIALIZABLE(CVehicleVelCmd_Holo)
   public:
	/** speed(m / s) */
	double vel{.0};
	/**: direction, **relative** to the current robot heading (radians). 0 means
	 * forward. */
	double dir_local{.0};
	/**: Blending time between current and target time. */
	double ramp_time{.0};
	/**: (rad/s) rotational speed for rotating such as the robot slowly faces
	 * forward. */
	double rot_speed{.0};

	CVehicleVelCmd_Holo();
	CVehicleVelCmd_Holo(
		double vel, double dir_local, double ramp_time, double rot_speed);
	~CVehicleVelCmd_Holo() override;
	size_t getVelCmdLength() const override;
	std::string getVelCmdDescription(const int index) const override;
	double getVelCmdElement(const int index) const override;
	void setVelCmdElement(const int index, const double val) override;
	bool isStopCmd() const override;
	void setToStop() override;

	// See base class docs.
	void cmdVel_scale(double vel_scale) override;
	double cmdVel_limits(
		const mrpt::kinematics::CVehicleVelCmd& prev_vel_cmd, const double beta,
		const TVelCmdParams& params) override;
};

}  // namespace mrpt::kinematics
