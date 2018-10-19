/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/serialization/CSerializable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <string>

namespace mrpt::kinematics
{
/** Virtual base for velocity commands of different kinematic models of planar
 * mobile robot.
 * \ingroup mrpt_kinematics_grp */
class CVehicleVelCmd : public mrpt::serialization::CSerializable
{
	DEFINE_VIRTUAL_SERIALIZABLE(CVehicleVelCmd)
   public:
	CVehicleVelCmd();
	CVehicleVelCmd(const CVehicleVelCmd& other);
	~CVehicleVelCmd() override;
	CVehicleVelCmd& operator=(const CVehicleVelCmd& other);

	/** Get number of components in each velocity command */
	virtual size_t getVelCmdLength() const = 0;
	/** Get textual, human-readable description of each velocity command
	 * component */
	virtual std::string getVelCmdDescription(const int index) const = 0;
	/** Get each velocity command component */
	virtual double getVelCmdElement(const int index) const = 0;
	/** Set each velocity command component */
	virtual void setVelCmdElement(const int index, const double val) = 0;
	/** Returns true if the command means "do not move" / "stop". \sa setToStop
	 */
	virtual bool isStopCmd() const = 0;
	/** Set to a command that means "do not move" / "stop". \sa isStopCmd */
	virtual void setToStop() = 0;
	/** Returns a human readable description of the cmd */
	std::string asString() const;

	/** Parameters that may be used by cmdVel_limits() in any derived classes.
	 */
	struct TVelCmdParams
	{
		/** Max. linear speed (m/s) [Default=-1 (not set), will raise exception
		 * if needed and not set] */
		double robotMax_V_mps{-1.};
		/** Max. angular speed (rad/s) [Default=-1 (not set), will raise
		 * exception if needed and not set] */
		double robotMax_W_radps{-1.};
		/** Min. radius of curvature of paths (m) [Default=-1 (not set), will
		 * raise exception if needed and not set] */
		double robotMinCurvRadius{-1.};

		TVelCmdParams();
		/** Load any parameter required by a CVehicleVelCmd derived class. */
		void loadConfigFile(
			const mrpt::config::CConfigFileBase& cfg,
			const std::string& section);
		void saveToConfigFile(
			mrpt::config::CConfigFileBase& c, const std::string& s) const;
	};

	/** Scale the velocity command encoded in this object.
	 * \param[in] vel_scale A scale within [0,1] reflecting how much should be
	 * the raw velocity command be lessen (e.g. for safety reasons,...).
	 * \param[out] out_vel_cmd
	 *
	 * Users can directly inherit from existing implementations instead of
	 * manually redefining this method:
	 *  - mrpt::kinematics::CVehicleVelCmd_DiffDriven
	 *  - mrpt::kinematics::CVehicleVelCmd_Holo
	 */
	virtual void cmdVel_scale(double vel_scale) = 0;

	/** Updates this command, computing a blended version of `beta` (within
	 * [0,1]) of `vel_cmd` and `1-beta` of `prev_vel_cmd`, simultaneously
	 * to honoring any user-side maximum velocities.
	 * \return The [0,1] ratio that the cmdvel had to be scaled down, or 1.0 if
	 * none.
	 */
	virtual double cmdVel_limits(
		const mrpt::kinematics::CVehicleVelCmd& prev_vel_cmd, const double beta,
		const TVelCmdParams& params) = 0;
};

}  // namespace mrpt::kinematics
