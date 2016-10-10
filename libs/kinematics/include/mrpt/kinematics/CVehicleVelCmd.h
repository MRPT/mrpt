/* +---------------------------------------------------------------------------+
|                     Mobile Robot Programming Toolkit (MRPT)               |
|                          http://www.mrpt.org/                             |
|                                                                           |
| Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
| See: http://www.mrpt.org/Authors - All rights reserved.                   |
| Released under BSD License. See details in http://www.mrpt.org/License    |
+---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/kinematics/link_pragmas.h>
#include <mrpt/utils/CSerializable.h>
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
		};
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE(CVehicleVelCmd, mrpt::utils::CSerializable, KINEMATICS_IMPEXP)


	} // End of namespace
} // End of namespace
