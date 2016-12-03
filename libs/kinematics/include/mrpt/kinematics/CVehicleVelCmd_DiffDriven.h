/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/kinematics/CVehicleVelCmd.h>

namespace mrpt
{
namespace kinematics
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE(CVehicleVelCmd_DiffDriven, CVehicleVelCmd, KINEMATICS_IMPEXP)

	/** Kinematic model for
	 *
	 * \ingroup mrpt_kinematics_grp
	 */
	class KINEMATICS_IMPEXP CVehicleVelCmd_DiffDriven : public CVehicleVelCmd
	{
		DEFINE_SERIALIZABLE(CVehicleVelCmd_DiffDriven)
	public:
		double lin_vel; //!< Linear velocity (m/s)
		double ang_vel; //!< Angular velocity (rad/s)

		CVehicleVelCmd_DiffDriven();
		virtual ~CVehicleVelCmd_DiffDriven();
		size_t getVelCmdLength() const MRPT_OVERRIDE;
		std::string getVelCmdDescription(const int index) const MRPT_OVERRIDE;
		double getVelCmdElement(const int index) const  MRPT_OVERRIDE;
		void setVelCmdElement(const int index, const double val) MRPT_OVERRIDE;
		bool isStopCmd() const MRPT_OVERRIDE;
		void setToStop() MRPT_OVERRIDE;


		/** This class recognizes these parameters: `robotMax_V_mps`, `robotMax_W_degps` */
		struct KINEMATICS_IMPEXP TVelCmdParams_DiffDriven : public CVehicleVelCmd::TVelCmdParams
		{
			double  robotMax_V_mps;       //!< Max. linear speed (m/s)
			double  robotMax_W_radps;     //!< Max. angular speed (rad/s)

			virtual void loadConfigFile(const mrpt::utils::CConfigFileBase &cfg, const std::string &section) MRPT_OVERRIDE;
			TVelCmdParams_DiffDriven();
		};

		/** See docs of method in base class. The implementation for differential-driven robots of this method
		* just multiplies all the components of vel_cmd times vel_scale, which is appropriate
		*  for differential-driven kinematic models (v,w).
		*/
		void cmdVel_scale(mrpt::kinematics::CVehicleVelCmd &vel_cmd, double vel_scale) MRPT_OVERRIDE;

		// See base class docs.
		void cmdVel_limits(mrpt::kinematics::CVehicleVelCmd &vel_cmd, const mrpt::kinematics::CVehicleVelCmd &prev_vel_cmd, const double beta, const TVelCmdParams &params)  MRPT_OVERRIDE;

	private:
		void filter_max_vw(double &v, double &w, const TVelCmdParams_DiffDriven &p);
	};
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE(CVehicleVelCmd_DiffDriven, CVehicleVelCmd, KINEMATICS_IMPEXP)

	} // End of namespace
} // End of namespace
