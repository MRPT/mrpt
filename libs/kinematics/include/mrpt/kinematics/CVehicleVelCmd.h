/* +---------------------------------------------------------------------------+
|                     Mobile Robot Programming Toolkit (MRPT)               |
|                          http://www.mrpt.org/                             |
|                                                                           |
| Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
| See: http://www.mrpt.org/Authors - All rights reserved.                   |
| Released under BSD License. See details in http://www.mrpt.org/License    |
+---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/kinematics/link_pragmas.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <string>

namespace mrpt
{
	namespace kinematics
	{
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE(CVehicleVelCmd, mrpt::utils::CSerializable, KINEMATICS_IMPEXP)

		/** Virtual base for velocity commands of different kinematic models of planar mobile robot.
		 * \ingroup mrpt_kinematics_grp */
		class KINEMATICS_IMPEXP CVehicleVelCmd : public mrpt::utils::CSerializable
		{
			DEFINE_VIRTUAL_SERIALIZABLE(CVehicleVelCmd)
		public:
			CVehicleVelCmd();
			CVehicleVelCmd(const CVehicleVelCmd &other);
			virtual ~CVehicleVelCmd();
			CVehicleVelCmd & operator =(const CVehicleVelCmd &other);

			virtual size_t getVelCmdLength() const = 0;  //!< Get number of components in each velocity command
			virtual std::string getVelCmdDescription(const int index) const = 0; //!< Get textual, human-readable description of each velocity command component
			virtual double getVelCmdElement(const int index) const = 0;  //!< Get each velocity command component
			virtual void setVelCmdElement(const int index, const double val) = 0;  //!< Set each velocity command component
			virtual bool isStopCmd() const = 0; //!< Returns true if the command means "do not move" / "stop". \sa setToStop
			virtual void setToStop() = 0; //!< Set to a command that means "do not move" / "stop". \sa isStopCmd
			std::string asString() const; //!< Returns a human readable description of the cmd

			/** Parameters that may be used by cmdVel_limits() in any derived classes. */
			struct KINEMATICS_IMPEXP TVelCmdParams
			{
				double  robotMax_V_mps;       //!< Max. linear speed (m/s) [Default=-1 (not set), will raise exception if needed and not set]
				double  robotMax_W_radps;     //!< Max. angular speed (rad/s) [Default=-1 (not set), will raise exception if needed and not set]
				double  robotMinCurvRadius;   //!< Min. radius of curvature of paths (m) [Default=-1 (not set), will raise exception if needed and not set]

				TVelCmdParams();
				/** Load any parameter required by a CVehicleVelCmd derived class. */
				void loadConfigFile(const mrpt::utils::CConfigFileBase &cfg, const std::string &section);
			};

			/** Scale the velocity command encoded in this object.
			* \param[in] vel_scale A scale within [0,1] reflecting how much should be the raw velocity command be lessen (e.g. for safety reasons,...).
			* \param[out] out_vel_cmd
			*
			* Users can directly inherit from existing implementations instead of manually redefining this method:
			*  - mrpt::nav::CReactiveInterfaceImplementation_DiffDriven
			*  - mrpt::nav::CReactiveInterfaceImplementation_Holo
			*/
			virtual void cmdVel_scale(double vel_scale) = 0;

			/** Updates this command, computing a blended version of `beta` (within [0,1]) of `vel_cmd` and `1-beta` of `prev_vel_cmd`, simultaneously
			* to honoring any user-side maximum velocities.
			*/
			virtual void cmdVel_limits(const mrpt::kinematics::CVehicleVelCmd &prev_vel_cmd, const double beta, const TVelCmdParams &params) = 0;
		};
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE(CVehicleVelCmd, mrpt::utils::CSerializable, KINEMATICS_IMPEXP)


	} // End of namespace
} // End of namespace
